`timescale 1ns / 1ps
//============================================================================
// OrionRV — RV32C Decompressor (Phase 7.5)
//
// Combinational expansion of 16-bit RV32C compressed instructions into
// their 32-bit RV32I equivalents. Placed between I-cache output and
// the main decoder.
//
// Supported quadrants: C0, C1, C2 (all RV32C instructions).
// Input:  16-bit compressed instruction
// Output: 32-bit expanded instruction + is_compressed flag
//============================================================================

module RVC_Decompressor (
    input  logic [15:0] instr_c,       // 16-bit compressed instruction
    output logic  [31:0] instr_out,     // Expanded 32-bit instruction
    output logic        is_compressed, // 1 = input was a valid C instruction
    output logic        illegal_c      // 1 = unrecognized C encoding
);

    // A compressed instruction has bits [1:0] != 2'b11
    assign is_compressed = (instr_c[1:0] != 2'b11);

    logic [2:0] funct3;
    assign funct3 = instr_c[15:13];
    logic [1:0] op;
    assign op = instr_c[1:0];

    // CIW/CL/CS register mapping: rs' = 8 + instr[field]
    logic [4:0] rd_prime;
    assign rd_prime = {2'b01, instr_c[4:2]};   // x8-x15
    logic [4:0] rs1_prime;                            // x8-x15
    assign rs1_prime = {2'b01, instr_c[9:7]};
    logic [4:0] rs2_prime;                            // x8-x15
    assign rs2_prime = {2'b01, instr_c[4:2]};

    // CI-format rd/rs1 (full 5-bit)
    logic [4:0] ci_rd;
    assign ci_rd = instr_c[11:7];
    logic [4:0] cr_rs2;
    assign cr_rs2 = instr_c[6:2];

    logic illegal_r;
    assign illegal_c = illegal_r && is_compressed;

    always_comb begin
        instr_out = 32'h0000_0013;  // Default: NOP
        illegal_r = 1'b0;

        if (!is_compressed) begin
            // Not compressed — should not be called, but pass through lower 16 bits
            instr_out = {16'b0, instr_c};
        end
        else begin
            case (op)
                // ===========================================================
                // Quadrant 0 (op = 2'b00)
                // ===========================================================
                2'b00: begin
                    case (funct3)
                        3'b000: begin // C.ADDI4SPN -> addi rd', x2, nzuimm
                            // nzuimm = {instr[10:7], instr[12:11], instr[5], instr[6], 2'b00}
                            instr_out = {2'b0, instr_c[10:7], instr_c[12:11],
                                         instr_c[5], instr_c[6], 2'b00,
                                         5'd2, 3'b000, rd_prime, 7'b0010011};
                            if (instr_c[12:5] == 8'b0) illegal_r = 1'b1;
                        end
                        3'b010: begin // C.LW -> lw rd', offset(rs1')
                            // offset = {instr[5], instr[12:10], instr[6], 2'b00}
                            instr_out = {5'b0, instr_c[5], instr_c[12:10],
                                         instr_c[6], 2'b00,
                                         rs1_prime, 3'b010, rd_prime, 7'b0000011};
                        end
                        3'b110: begin // C.SW -> sw rs2', offset(rs1')
                            instr_out = {5'b0, instr_c[5], instr_c[12],
                                         rs2_prime, rs1_prime, 3'b010,
                                         instr_c[11:10], instr_c[6], 2'b00,
                                         7'b0100011};
                        end
                        default: illegal_r = 1'b1;
                    endcase
                end

                // ===========================================================
                // Quadrant 1 (op = 2'b01)
                // ===========================================================
                2'b01: begin
                    case (funct3)
                        3'b000: begin // C.ADDI / C.NOP
                            // addi rd, rd, nzimm (sign-extended)
                            instr_out = {{6{instr_c[12]}}, instr_c[12],
                                         instr_c[6:2], ci_rd, 3'b000, ci_rd, 7'b0010011};
                        end
                        3'b001: begin // C.JAL -> jal x1, offset
                            instr_out = {instr_c[12], instr_c[8], instr_c[10:9],
                                         instr_c[6], instr_c[7], instr_c[2],
                                         instr_c[11], instr_c[5:3],
                                         {9{instr_c[12]}},
                                         5'd1, 7'b1101111};
                        end
                        3'b010: begin // C.LI -> addi rd, x0, imm
                            instr_out = {{6{instr_c[12]}}, instr_c[12],
                                         instr_c[6:2], 5'd0, 3'b000, ci_rd, 7'b0010011};
                        end
                        3'b011: begin
                            if (ci_rd == 5'd2) begin // C.ADDI16SP
                                instr_out = {{3{instr_c[12]}}, instr_c[12],
                                             instr_c[4:3], instr_c[5], instr_c[2],
                                             instr_c[6], 4'b0000,
                                             5'd2, 3'b000, 5'd2, 7'b0010011};
                            end else begin // C.LUI
                                instr_out = {{14{instr_c[12]}}, instr_c[12],
                                             instr_c[6:2], ci_rd, 7'b0110111};
                            end
                        end
                        3'b100: begin // ALU operations on compressed regs
                            case (instr_c[11:10])
                                2'b00: // C.SRLI
                                    instr_out = {7'b0000000, instr_c[6:2],
                                                 rs1_prime, 3'b101, rs1_prime, 7'b0010011};
                                2'b01: // C.SRAI
                                    instr_out = {7'b0100000, instr_c[6:2],
                                                 rs1_prime, 3'b101, rs1_prime, 7'b0010011};
                                2'b10: // C.ANDI
                                    instr_out = {{6{instr_c[12]}}, instr_c[12],
                                                 instr_c[6:2], rs1_prime, 3'b111,
                                                 rs1_prime, 7'b0010011};
                                2'b11: begin
                                    case ({instr_c[12], instr_c[6:5]})
                                        3'b000: // C.SUB
                                            instr_out = {7'b0100000, rs2_prime,
                                                         rs1_prime, 3'b000, rs1_prime, 7'b0110011};
                                        3'b001: // C.XOR
                                            instr_out = {7'b0000000, rs2_prime,
                                                         rs1_prime, 3'b100, rs1_prime, 7'b0110011};
                                        3'b010: // C.OR
                                            instr_out = {7'b0000000, rs2_prime,
                                                         rs1_prime, 3'b110, rs1_prime, 7'b0110011};
                                        3'b011: // C.AND
                                            instr_out = {7'b0000000, rs2_prime,
                                                         rs1_prime, 3'b111, rs1_prime, 7'b0110011};
                                        default: illegal_r = 1'b1;
                                    endcase
                                end
                            endcase
                        end
                        3'b101: begin // C.J -> jal x0, offset
                            instr_out = {instr_c[12], instr_c[8], instr_c[10:9],
                                         instr_c[6], instr_c[7], instr_c[2],
                                         instr_c[11], instr_c[5:3],
                                         {9{instr_c[12]}},
                                         5'd0, 7'b1101111};
                        end
                        3'b110: begin // C.BEQZ -> beq rs1', x0, offset
                            instr_out = {{3{instr_c[12]}}, instr_c[12],
                                         instr_c[6:5], instr_c[2],
                                         5'd0, rs1_prime, 3'b000,
                                         instr_c[11:10], instr_c[4:3],
                                         instr_c[12], 7'b1100011};
                        end
                        3'b111: begin // C.BNEZ -> bne rs1', x0, offset
                            instr_out = {{3{instr_c[12]}}, instr_c[12],
                                         instr_c[6:5], instr_c[2],
                                         5'd0, rs1_prime, 3'b001,
                                         instr_c[11:10], instr_c[4:3],
                                         instr_c[12], 7'b1100011};
                        end
                    endcase
                end

                // ===========================================================
                // Quadrant 2 (op = 2'b10)
                // ===========================================================
                2'b10: begin
                    case (funct3)
                        3'b000: begin // C.SLLI
                            instr_out = {7'b0000000, instr_c[6:2],
                                         ci_rd, 3'b001, ci_rd, 7'b0010011};
                        end
                        3'b010: begin // C.LWSP -> lw rd, offset(x2)
                            instr_out = {4'b0, instr_c[3:2], instr_c[12],
                                         instr_c[6:4], 2'b00,
                                         5'd2, 3'b010, ci_rd, 7'b0000011};
                        end
                        3'b100: begin
                            if (instr_c[12] == 1'b0) begin
                                if (cr_rs2 == 5'd0) // C.JR -> jalr x0, rs1, 0
                                    instr_out = {12'b0, ci_rd, 3'b000, 5'd0, 7'b1100111};
                                else // C.MV -> add rd, x0, rs2
                                    instr_out = {7'b0000000, cr_rs2,
                                                 5'd0, 3'b000, ci_rd, 7'b0110011};
                            end else begin
                                if (cr_rs2 == 5'd0 && ci_rd == 5'd0) begin
                                    // C.EBREAK -> ebreak
                                    instr_out = 32'h0010_0073;
                                end
                                else if (cr_rs2 == 5'd0) // C.JALR -> jalr x1, rs1, 0
                                    instr_out = {12'b0, ci_rd, 3'b000, 5'd1, 7'b1100111};
                                else // C.ADD -> add rd, rd, rs2
                                    instr_out = {7'b0000000, cr_rs2,
                                                 ci_rd, 3'b000, ci_rd, 7'b0110011};
                            end
                        end
                        3'b110: begin // C.SWSP -> sw rs2, offset(x2)
                            instr_out = {4'b0, instr_c[8:7], instr_c[12],
                                         cr_rs2, 5'd2, 3'b010,
                                         instr_c[11:9], 2'b00, 7'b0100011};
                        end
                        default: illegal_r = 1'b1;
                    endcase
                end

                default: illegal_r = 1'b1;
            endcase
        end
    end
endmodule
