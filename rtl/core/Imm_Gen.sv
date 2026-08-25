`timescale 1ns / 1ps
//============================================================================
// OrionRV - Immediate Generator
// Extracts and sign-extends immediates for all RV32I instruction formats
//============================================================================

module imm_gen (
    input  logic [31:0] instr,
    output logic  [31:0] imm
);

    logic [6:0] opcode;
    assign opcode = instr[6:0];

    always_comb begin
        case (opcode)
            // I-type: ALU immediate, Loads, JALR
            7'b0010011,  // OP-IMM (ADDI, SLTI, etc.)
            7'b0000011,  // LOAD   (LB, LH, LW, LBU, LHU)
            7'b1100111:  // JALR
                imm = {{20{instr[31]}}, instr[31:20]};

            // S-type: Stores
            7'b0100011:  // STORE  (SB, SH, SW)
                imm = {{20{instr[31]}}, instr[31:25], instr[11:7]};

            // B-type: Branches
            7'b1100011:  // BRANCH (BEQ, BNE, BLT, BGE, BLTU, BGEU)
                imm = {{19{instr[31]}}, instr[31], instr[7], instr[30:25], instr[11:8], 1'b0};

            // U-type: LUI, AUIPC
            7'b0110111,  // LUI
            7'b0010111:  // AUIPC
                imm = {instr[31:12], 12'b0};

            // J-type: JAL
            7'b1101111:  // JAL
                imm = {{11{instr[31]}}, instr[31], instr[19:12], instr[20], instr[30:21], 1'b0};

            // System (CSR immediates use rs1 field as zimm)
            7'b1110011:  // SYSTEM
                imm = {27'b0, instr[19:15]};  // Zero-extended zimm[4:0]

            default:
                imm = 32'b0;
        endcase
    end

endmodule
