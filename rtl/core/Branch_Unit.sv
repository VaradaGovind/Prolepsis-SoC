`timescale 1ns / 1ps
//============================================================================
// OrionRV - Branch Comparison Unit
// Evaluates branch conditions for B-type instructions
//============================================================================

module branch_unit (
    input  logic [31:0] rs1_data,
    input  logic [31:0] rs2_data,
    input  logic [2:0]  funct3,
    input  logic        is_branch,
    output logic         branch_taken
);

    // Keep BEQ/BNE on a pure XOR-reduction path to shorten the most common
    // branch compare cone that currently limits setup timing.
    logic [31:0] cmp_xor;
    assign cmp_xor = rs1_data ^ rs2_data;
    logic        cmp_eq;
    assign cmp_eq = ~|cmp_xor;

    // Signed/unsigned less-than are still needed for BLT/BGE/BLTU/BGEU.
    logic        cmp_ltu;
    assign cmp_ltu = (rs1_data < rs2_data);
    logic        cmp_lts;
    assign cmp_lts = (rs1_data[31] ^ rs2_data[31]) ? rs1_data[31]
                                                       : (rs1_data[30:0] < rs2_data[30:0]);

    always_comb begin
        branch_taken = 1'b0;

        if (is_branch) begin
            case (funct3)
                3'b000:  branch_taken =  cmp_eq;   // BEQ
                3'b001:  branch_taken = ~cmp_eq;   // BNE
                3'b100:  branch_taken =  cmp_lts;  // BLT
                3'b101:  branch_taken = ~cmp_lts;  // BGE
                3'b110:  branch_taken =  cmp_ltu;  // BLTU
                3'b111:  branch_taken = ~cmp_ltu;  // BGEU
                default: branch_taken = 1'b0;
            endcase
        end
    end

endmodule
