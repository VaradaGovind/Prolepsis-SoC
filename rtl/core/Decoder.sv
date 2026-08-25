`timescale 1ns / 1ps
//============================================================================
// OrionRV - Instruction Decoder
// Generates all control signals from the instruction opcode/funct fields
//============================================================================

module decoder (
    input  logic [31:0] instr,

    // ALU control
    output logic  [4:0]  alu_op,
    output logic         alu_src,      // 0=rs2, 1=immediate

    // Memory control
    output logic         mem_read,
    output logic         mem_write,
    output logic  [2:0]  mem_funct3,   // For load/store size + sign extension

    // Register write control
    output logic         reg_write,
    output logic  [1:0]  wb_sel,       // 00=ALU, 01=memory, 10=PC+4

    // Branch/jump control
    output logic         is_branch,
    output logic         is_jal,
    output logic         is_jalr,

    // Atomic
    output logic         is_atomic,
    output logic  [4:0]  atomic_funct5,

    // CSR
    output logic         is_csr,
    output logic  [1:0]  csr_op,       // 00=none, 01=RW, 10=RS, 11=RC

    // System
    output logic         is_ecall,
    output logic         is_mret,

    // Validity
    output logic         illegal_instr
);

    // -------------------------------------------------------------------
    // Instruction fields
    // -------------------------------------------------------------------
    logic [6:0] opcode;
    assign opcode = instr[6:0];
    logic [2:0] funct3;
    assign funct3 = instr[14:12];
    logic [6:0] funct7;
    assign funct7 = instr[31:25];
    logic [4:0] funct5;
    assign funct5 = instr[31:27];

    // -------------------------------------------------------------------
    // ALU op encoding (matches alu.v)
    // -------------------------------------------------------------------
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

    // -------------------------------------------------------------------
    // Decode logic
    // -------------------------------------------------------------------
    always_comb begin
        // Defaults
        alu_op        = ALU_ADD;
        alu_src       = 1'b0;
        mem_read      = 1'b0;
        mem_write     = 1'b0;
        mem_funct3    = 3'b010;
        reg_write     = 1'b0;
        wb_sel        = 2'b00;
        is_branch     = 1'b0;
        is_jal        = 1'b0;
        is_jalr       = 1'b0;
        is_atomic     = 1'b0;
        atomic_funct5 = 5'b0;
        is_csr        = 1'b0;
        csr_op        = 2'b00;
        is_ecall      = 1'b0;
        is_mret       = 1'b0;
        illegal_instr = 1'b0;

        case (opcode)

            // ---------------------------------------------------------
            // R-type: register-register operations
            // ---------------------------------------------------------
            7'b0110011: begin
                reg_write = 1'b1;
                alu_src   = 1'b0;

                if (funct7 == 7'b0000001) begin
                    // M-extension
                    case (funct3)
                        3'b000: alu_op = ALU_MUL;
                        3'b001: alu_op = ALU_MULH;
                        3'b010: alu_op = ALU_MULHSU;
                        3'b011: alu_op = ALU_MULHU;
                        3'b100: alu_op = ALU_DIV;
                        3'b101: alu_op = ALU_DIVU;
                        3'b110: alu_op = ALU_REM;
                        3'b111: alu_op = ALU_REMU;
                    endcase
                end
                else begin
                    case (funct3)
                        3'b000: alu_op = (funct7[5]) ? ALU_SUB : ALU_ADD;
                        3'b001: alu_op = ALU_SLL;
                        3'b010: alu_op = ALU_SLT;
                        3'b011: alu_op = ALU_SLTU;
                        3'b100: alu_op = ALU_XOR;
                        3'b101: alu_op = (funct7[5]) ? ALU_SRA : ALU_SRL;
                        3'b110: alu_op = ALU_OR;
                        3'b111: alu_op = ALU_AND;
                    endcase
                end
            end

            // ---------------------------------------------------------
            // I-type: immediate ALU operations
            // ---------------------------------------------------------
            7'b0010011: begin
                reg_write = 1'b1;
                alu_src   = 1'b1;

                case (funct3)
                    3'b000: alu_op = ALU_ADD;   // ADDI
                    3'b010: alu_op = ALU_SLT;   // SLTI
                    3'b011: alu_op = ALU_SLTU;  // SLTIU
                    3'b100: alu_op = ALU_XOR;   // XORI
                    3'b110: alu_op = ALU_OR;    // ORI
                    3'b111: alu_op = ALU_AND;   // ANDI
                    3'b001: alu_op = ALU_SLL;   // SLLI
                    3'b101: alu_op = (funct7[5]) ? ALU_SRA : ALU_SRL; // SRLI/SRAI
                endcase
            end

            // ---------------------------------------------------------
            // Load instructions
            // ---------------------------------------------------------
            7'b0000011: begin
                reg_write  = 1'b1;
                alu_src    = 1'b1;
                alu_op     = ALU_ADD;
                mem_read   = 1'b1;
                mem_funct3 = funct3;
                wb_sel     = 2'b01;
            end

            // ---------------------------------------------------------
            // Store instructions
            // ---------------------------------------------------------
            7'b0100011: begin
                alu_src    = 1'b1;
                alu_op     = ALU_ADD;
                mem_write  = 1'b1;
                mem_funct3 = funct3;
            end

            // ---------------------------------------------------------
            // Branch instructions
            // ---------------------------------------------------------
            7'b1100011: begin
                is_branch  = 1'b1;
                alu_src    = 1'b0;
                // ALU computes subtraction for comparison
                case (funct3)
                    3'b000: alu_op = ALU_SUB;   // BEQ
                    3'b001: alu_op = ALU_SUB;   // BNE
                    3'b100: alu_op = ALU_SLT;   // BLT
                    3'b101: alu_op = ALU_SLT;   // BGE
                    3'b110: alu_op = ALU_SLTU;  // BLTU
                    3'b111: alu_op = ALU_SLTU;  // BGEU
                    default: illegal_instr = 1'b1;
                endcase
            end

            // ---------------------------------------------------------
            // LUI: load upper immediate
            // ---------------------------------------------------------
            7'b0110111: begin
                reg_write = 1'b1;
                alu_src   = 1'b1;
                alu_op    = ALU_PASS_B;
            end

            // ---------------------------------------------------------
            // AUIPC: add upper immediate to PC
            // ---------------------------------------------------------
            7'b0010111: begin
                reg_write = 1'b1;
                alu_src   = 1'b1;
                alu_op    = ALU_ADD;  // ALU adds PC + imm (a = PC in pipeline)
            end

            // ---------------------------------------------------------
            // JAL: jump and link
            // ---------------------------------------------------------
            7'b1101111: begin
                reg_write = 1'b1;
                is_jal    = 1'b1;
                wb_sel    = 2'b10;   // Write PC+4 to rd
            end

            // ---------------------------------------------------------
            // JALR: jump and link register
            // ---------------------------------------------------------
            7'b1100111: begin
                reg_write = 1'b1;
                is_jalr   = 1'b1;
                alu_src   = 1'b1;
                alu_op    = ALU_ADD;
                wb_sel    = 2'b10;
            end

            // ---------------------------------------------------------
            // Atomic (A extension): LR.W, SC.W, AMO*
            // ---------------------------------------------------------
            7'b0101111: begin
                is_atomic     = 1'b1;
                atomic_funct5 = funct5;
                reg_write     = 1'b1;
                mem_funct3    = funct3;
            end

            // ---------------------------------------------------------
            // SYSTEM: CSR, ECALL, EBREAK, MRET
            // ---------------------------------------------------------
            7'b1110011: begin
                if (funct3 == 3'b000) begin
                    case (instr[31:20])
                        12'b000000000000: is_ecall = 1'b1;  // ECALL
                        12'b000000000001: ;                   // EBREAK (nop for now)
                        12'b001100000010: is_mret  = 1'b1;   // MRET
                        default: ;
                    endcase
                end
                else begin
                    // CSR instructions
                    is_csr    = 1'b1;
                    reg_write = 1'b1;
                    case (funct3[1:0])
                        2'b01: csr_op = 2'b01;  // CSRRW / CSRRWI
                        2'b10: csr_op = 2'b10;  // CSRRS / CSRRSI
                        2'b11: csr_op = 2'b11;  // CSRRC / CSRRCI
                        default: csr_op = 2'b00;
                    endcase
                end
            end

            // FENCE (treat as NOP for now)
            7'b0001111: begin
                // No operation in simple in-order pipeline
            end

            default: begin
                illegal_instr = 1'b1;
            end

        endcase
    end

endmodule
