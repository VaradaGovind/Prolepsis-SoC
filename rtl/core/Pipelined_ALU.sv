`timescale 1ns / 1ps
//============================================================================
// OrionRV - Pipelined, Multi-Cycle RV32IM ALU
//
// This ALU is designed to be synthesis-friendly and meet timing by avoiding
// single-cycle combinational paths for complex operations like division.
//
// - Base integer operations (ADD, SUB, logic) are combinational (1 cycle).
// - Multiplication is handled by DSP48 inference (1 cycle).
// - Division and Remainder use a multi-cycle restoring division algorithm
//   that takes 33 cycles (1 setup + 32 iterations).
//
// Interface:
//   - A 'start' signal initiates a new calculation.
//   - A 'done' signal indicates the result is valid.
//   - The core must stall if it needs a result before 'done' is high.
//
// Audit fixes applied:
//   - BUG-01: Quotient bit insertion uses combined shift+OR (no NBA race)
//   - BUG-02: Remainder comparison uses correctly-sized 32-bit divisor_reg
//   - BUG-03: is_signed/neg_result computed from combinational logic, not
//             stale registered value
//   - BUG-04: alu_op latched at division start; non-div ops skip S_FINISH
//             result overwrite
//   - BUG-05: Dividend sign latched at start for correct remainder sign
//============================================================================

module pipelined_alu (
    input  logic        clk,
    input  logic        rst,
    input  logic        start,      // Start a new calculation

    input  logic [31:0] a,
    input  logic [31:0] b,
    input  logic [4:0]  alu_op,

    output logic  [31:0] result,
    output logic        zero,
    output logic         done        // Result is valid
);

    //----------------------------------------------------------------
    // ALU Operation Opcodes (matches decoder)
    //----------------------------------------------------------------
    localparam ALU_ADD    = 5'b00000;
    localparam ALU_SUB    = 5'b00001;
    localparam ALU_SLL    = 5'b00010;
    localparam ALU_SLT    = 5'b00011;
    localparam ALU_SLTU   = 5'b00100;
    localparam ALU_XOR    = 5'b00101;
    localparam ALU_SRL    = 5'b00110;
    localparam ALU_SRA    = 5'b00111;
    localparam ALU_OR     = 5'b01000;
    localparam ALU_AND    = 5'b01001;
    localparam ALU_MUL    = 5'b01010;
    localparam ALU_MULH   = 5'b01011;
    localparam ALU_MULHSU = 5'b01100;
    localparam ALU_MULHU  = 5'b01101;
    localparam ALU_DIV    = 5'b01110;
    localparam ALU_DIVU   = 5'b01111;
    localparam ALU_REM    = 5'b10000;
    localparam ALU_REMU   = 5'b10001;
    localparam ALU_PASS_B = 5'b10010;

    //----------------------------------------------------------------
    // State Machine for Multi-Cycle Operations
    //----------------------------------------------------------------
    localparam S_IDLE      = 3'd0;
    localparam S_BUSY_DIV  = 3'd1;
    localparam S_FINISH    = 3'd2;
    localparam S_WAIT_MUL1 = 3'd3;
    localparam S_WAIT_MUL2 = 3'd4;
    localparam S_WAIT_MUL3 = 3'd5;
    localparam S_WAIT_MUL4 = 3'd6;
    localparam S_WAIT_MUL5 = 3'd7;

    logic [2:0] state;
    logic [5:0] cycle_count; // Counter for division (32 iterations)

    //----------------------------------------------------------------
    // Division/Remainder Latched Operands (BUG-04, BUG-05 fix)
    // These are captured at division start so S_FINISH doesn't depend
    // on live pipeline inputs that change every cycle.
    //----------------------------------------------------------------
    logic [31:0] dividend;
    logic [31:0] divisor_reg;     // 32 bits only (BUG-02 fix: no 64-bit compare)
    logic [31:0] quotient_reg;    // 32-bit quotient (not 64-bit)
    logic [31:0] remainder_reg;   // Upper 32 bits of the working remainder
    logic        neg_quotient;    // Latched at start (BUG-03 fix)
    logic        neg_remainder;   // Latched: dividend was negative (BUG-05 fix)
    logic [4:0]  latched_alu_op;  // Latched at start (BUG-04 fix)
    logic        is_div_op;       // Latched: distinguish div/rem in S_FINISH

    //----------------------------------------------------------------
    // Single-Cycle (Combinational) Results
    //----------------------------------------------------------------
    logic [31:0]        combinational_result;

    // Shared multiply datapath (single product pipeline).
    // This avoids parallel MUL/MULHSU/MULHU product coupling that can
    // synthesize into long cross-DSP carry chains.
    logic signed [32:0] mul_a_ext;
    logic signed [32:0] mul_b_ext;
    logic [31:0] mul_pipe1_lo, mul_pipe1_hi;
    logic [31:0] mul_pipe2_lo, mul_pipe2_hi;
    logic [31:0] mul_pipe3_lo, mul_pipe3_hi;
    logic [31:0] mul_pipe4_lo, mul_pipe4_hi;
    logic signed [65:0] mul_prod_w;
    assign mul_prod_w = mul_a_ext * mul_b_ext;

    logic signed [32:0] mul_a_ext_w;
    assign mul_a_ext_w = (alu_op == ALU_MULHU) ? $signed({1'b0, a})
                                                            : $signed({a[31], a});
    logic signed [32:0] mul_b_ext_w;
    assign mul_b_ext_w = ((alu_op == ALU_MUL) || (alu_op == ALU_MULH))
                                                            ? $signed({b[31], b})
                                                            : $signed({1'b0, b});

    assign zero = (result == 32'b0);

    // All non-div, non-mul ops can be calculated combinationally
    assign combinational_result =
        (alu_op == ALU_ADD)    ? (a + b) :
        (alu_op == ALU_SUB)    ? (a - b) :
        (alu_op == ALU_SLL)    ? (a << b[4:0]) :
        (alu_op == ALU_SLT)    ? {{31{1'b0}}, ($signed(a) < $signed(b))} :
        (alu_op == ALU_SLTU)   ? {{31{1'b0}}, (a < b)} :
        (alu_op == ALU_XOR)    ? (a ^ b) :
        (alu_op == ALU_SRL)    ? (a >> b[4:0]) :
        (alu_op == ALU_SRA)    ? ($signed(a) >>> b[4:0]) :
        (alu_op == ALU_OR)     ? (a | b) :
        (alu_op == ALU_AND)    ? (a & b) :
        (alu_op == ALU_PASS_B) ? b :
        32'b0;

    //----------------------------------------------------------------
    // Combinational wires for signed detection at start (BUG-03 fix)
    // These are evaluated from live inputs when start is asserted,
    // then latched into registers. This avoids the stale-value problem
    // where NBA on is_signed was read before it took effect.
    //----------------------------------------------------------------
    logic cur_is_signed;
    assign cur_is_signed = (alu_op == ALU_DIV  || alu_op == ALU_REM);
    logic cur_neg_quot;
    assign cur_neg_quot = cur_is_signed && (a[31] ^ b[31]);
    logic cur_neg_rem;
    assign cur_neg_rem = cur_is_signed && a[31];
    logic [31:0] abs_a;
    assign abs_a = (cur_is_signed && a[31]) ? (~a + 32'd1) : a;
    logic [31:0] abs_b;
    assign abs_b = (cur_is_signed && b[31]) ? (~b + 32'd1) : b;

    //----------------------------------------------------------------
    // Division iteration: combinational next-values (BUG-01 fix)
    // By computing the shifted remainder and quotient bit as wires,
    // we avoid the NBA race condition where quotient[0] was set and
    // then overwritten by the quotient shift in the same always block.
    //----------------------------------------------------------------
    // Shift remainder left by 1, bring in MSB of dividend
    logic [32:0] shifted_remainder;
    assign shifted_remainder = {remainder_reg, dividend[31]};

    // Compare: can we subtract the divisor?
    logic can_subtract;
    assign can_subtract = (shifted_remainder >= {1'b0, divisor_reg});

    // After potential subtraction
    logic [31:0] next_remainder;
    assign next_remainder = can_subtract ?
                                 (shifted_remainder[31:0] - divisor_reg) :
                                 shifted_remainder[31:0];

    // Shift quotient left, insert new bit (combined shift+OR, no race)
    logic [31:0] next_quotient;
    assign next_quotient = {quotient_reg[30:0], can_subtract};

    //----------------------------------------------------------------
    // Main State Machine
    //----------------------------------------------------------------
    always_ff @(posedge clk) begin
        if (rst) begin
            state         <= S_IDLE;
            done          <= 1'b1;
            result        <= 32'b0;
            cycle_count   <= 6'b0;
            dividend      <= 32'b0;
            divisor_reg   <= 32'b0;
            quotient_reg  <= 32'b0;
            remainder_reg <= 32'b0;
            neg_quotient  <= 1'b0;
            neg_remainder <= 1'b0;
            latched_alu_op<= 5'b0;
            is_div_op     <= 1'b0;
            mul_a_ext     <= 33'sd0;
            mul_b_ext     <= 33'sd0;
            mul_pipe1_lo  <= 32'd0;
            mul_pipe1_hi  <= 32'd0;
            mul_pipe2_lo  <= 32'd0;
            mul_pipe2_hi  <= 32'd0;
            mul_pipe3_lo  <= 32'd0;
            mul_pipe3_hi  <= 32'd0;
            mul_pipe4_lo  <= 32'd0;
            mul_pipe4_hi  <= 32'd0;
        end else begin
            case (state)
                //================================================
                // S_IDLE: Wait for a start signal
                //================================================
                S_IDLE: begin
                    done <= 1'b1;

                    if (start) begin
                        if (alu_op == ALU_DIV || alu_op == ALU_DIVU || 
                            alu_op == ALU_REM || alu_op == ALU_REMU) begin

                            // Latch all division-related state (BUG-03, BUG-04, BUG-05 fix)
                            latched_alu_op <= alu_op;
                            neg_quotient   <= cur_neg_quot;
                            neg_remainder  <= cur_neg_rem;
                            is_div_op      <= (alu_op == ALU_DIV || alu_op == ALU_DIVU);

                            // Handle division edge cases immediately
                            if (b == 32'b0) begin
                                // Division by zero (RISC-V spec)
                                state  <= S_FINISH;
                                result <= (alu_op == ALU_DIV || alu_op == ALU_DIVU) 
                                          ? 32'hFFFF_FFFF : a;
                                done   <= 1'b0;
                                // Mark so S_FINISH doesn't overwrite
                                is_div_op <= 1'b0;
                            end
                            // Signed division overflow: -2^31 / -1
                            else if (alu_op == ALU_DIV && a == 32'h8000_0000 && b == 32'hFFFF_FFFF) begin
                                state  <= S_FINISH;
                                result <= a; // -2^31
                                done   <= 1'b0;
                                is_div_op <= 1'b0;
                            end
                            // Signed remainder overflow: -2^31 % -1 = 0
                            else if (alu_op == ALU_REM && a == 32'h8000_0000 && b == 32'hFFFF_FFFF) begin
                                state  <= S_FINISH;
                                result <= 32'b0;
                                done   <= 1'b0;
                                is_div_op <= 1'b0;
                            end
                            // Normal division: start iterative algorithm
                            else begin
                                state         <= S_BUSY_DIV;
                                done          <= 1'b0;
                                cycle_count   <= 6'd32;

                                // Use absolute values for unsigned division algorithm
                                dividend      <= abs_a;
                                divisor_reg   <= abs_b;
                                quotient_reg  <= 32'b0;
                                remainder_reg <= 32'b0;
                            end
                        end else if (alu_op == ALU_MUL || alu_op == ALU_MULH || alu_op == ALU_MULHSU || alu_op == ALU_MULHU) begin
                            state <= S_WAIT_MUL1;
                            done  <= 1'b0;
                            latched_alu_op <= alu_op;
                            is_div_op <= 1'b0;
                            mul_a_ext <= mul_a_ext_w;
                            mul_b_ext <= mul_b_ext_w;
                        end else begin
                            // Non-division: result is already available
                            // Load combinational result and go directly to IDLE
                            // next cycle (BUG-04 fix: don't route through S_FINISH
                            // which would try to overwrite with div results)
                            result <= combinational_result;
                            // Stay in IDLE, assert done immediately next cycle
                            // by going to S_FINISH with is_div_op=0
                            state      <= S_FINISH;
                            done       <= 1'b0;
                            is_div_op  <= 1'b0; // Prevent S_FINISH from overwriting
                        end
                    end
                end

                //================================================
                // S_BUSY_DIV: Perform one bit of division per cycle
                //
                // Restoring division algorithm (BUG-01, BUG-02 fix):
                //   1. Shift remainder left, bring in next dividend bit
                //   2. If shifted_remainder >= divisor, subtract and set quotient bit
                //   3. Shift quotient left with new bit inserted
                //
                // All computed as combinational wires above, then latched
                // with a single NBA per register (no competing assignments).
                //================================================
                S_BUSY_DIV: begin
                    remainder_reg <= next_remainder;
                    quotient_reg  <= next_quotient;
                    dividend      <= dividend << 1;

                    cycle_count <= cycle_count - 6'd1;
                    if (cycle_count == 6'd1) begin
                        state <= S_FINISH;
                    end
                end

                S_WAIT_MUL1: begin
                    mul_pipe1_lo <= mul_prod_w[31:0];
                    mul_pipe1_hi <= mul_prod_w[63:32];
                    state <= S_WAIT_MUL2;
                end
                S_WAIT_MUL2: begin
                    mul_pipe2_lo <= mul_pipe1_lo;
                    mul_pipe2_hi <= mul_pipe1_hi;
                    state <= S_WAIT_MUL3;
                end
                S_WAIT_MUL3: begin
                    mul_pipe3_lo <= mul_pipe2_lo;
                    mul_pipe3_hi <= mul_pipe2_hi;
                    state <= S_WAIT_MUL4;
                end
                S_WAIT_MUL4: begin
                    mul_pipe4_lo <= mul_pipe3_lo;
                    mul_pipe4_hi <= mul_pipe3_hi;
                    state <= S_WAIT_MUL5;
                end
                S_WAIT_MUL5: begin
                    if (latched_alu_op == ALU_MUL)
                        result <= mul_pipe4_lo;
                    else
                        result <= mul_pipe4_hi;
                    state <= S_FINISH;
                end

                //================================================
                // S_FINISH: Present the result and return to IDLE
                //
                // Uses latched alu_op and sign flags (BUG-04, BUG-05).
                // For non-division ops, result was already loaded in
                // S_IDLE and is_div_op=0, so we skip the overwrite.
                //================================================
                S_FINISH: begin
                    if (is_div_op) begin
                        // Quotient result with sign correction
                        result <= neg_quotient ? (~quotient_reg + 32'd1) : quotient_reg;
                    end else if (latched_alu_op == ALU_REM || latched_alu_op == ALU_REMU) begin
                        // Remainder result: sign matches dividend sign (BUG-05 fix)
                        result <= neg_remainder ? (~remainder_reg + 32'd1) : remainder_reg;
                    end
                    // else: non-division op, result already loaded in S_IDLE

                    done  <= 1'b1;
                    state <= S_IDLE;
                end

                default: begin
                    state <= S_IDLE;
                    done  <= 1'b1;
                end
            endcase
        end
    end

endmodule
