`timescale 1ns / 1ps
//============================================================================
// OrionRV — Hardware Loop Buffer (Phase 7.5)
//
// Captures tight backward-branch loops (up to 16 instructions).
// Once captured, serves instructions from the buffer, allowing the
// I-cache and AXI fetch path to be clock-gated for power savings.
//============================================================================

module Loop_Buffer #(
    parameter DEPTH = 16
)(
    input  logic        clk,
    input  logic        rst,
    input  logic [31:0] fetch_pc,
    input  logic [31:0] fetch_instr,
    input  logic        fetch_valid,
    input  logic        branch_taken,
    input  logic        branch_is_back,
    input  logic [31:0] branch_target,
    input  logic [31:0] branch_pc,
    input  logic        branch_valid,
    input  logic        pipeline_flush,
    output logic  [31:0] lb_instr,
    output logic         lb_valid,
    output logic        loop_active,
    input  logic        pipe_en,
    input  logic        stall
);
    localparam ADDR_W = $clog2(DEPTH);
    localparam ST_IDLE   = 2'd0;
    localparam ST_FILL   = 2'd1;
    localparam ST_ACTIVE = 2'd2;

    logic [1:0] state;
    logic [31:0] buf_instr [0:DEPTH-1];
    logic [ADDR_W-1:0] buf_count;
    logic [ADDR_W-1:0] buf_ptr;
    logic [31:0] loop_end_pc;

    assign loop_active = (state == ST_ACTIVE);
    integer i;

    always_ff @(posedge clk) begin
        if (rst) begin
            state <= ST_IDLE;
            buf_count <= 0; buf_ptr <= 0;
            loop_end_pc <= 32'b0;
            lb_instr <= 32'h0000_0013; lb_valid <= 1'b0;
            for (i = 0; i < DEPTH; i = i + 1) begin
                buf_instr[i] <= 32'h0000_0013;
            end
        end
        else if (pipeline_flush) begin
            state <= ST_IDLE; buf_count <= 0; buf_ptr <= 0; lb_valid <= 1'b0;
        end
        else if (pipe_en && !stall) begin
            case (state)
                ST_IDLE: begin
                    lb_valid <= 1'b0;
                    if (branch_valid && branch_taken && branch_is_back) begin
                        state <= ST_FILL;
                        loop_end_pc   <= branch_pc;
                        buf_count     <= 0;
                    end
                end
                ST_FILL: begin
                    lb_valid <= 1'b0;
                    if (fetch_valid) begin
                        if (buf_count == DEPTH - 1) begin
                            state <= ST_IDLE; buf_count <= 0;
                        end else begin
                            buf_instr[buf_count] <= fetch_instr;
                            buf_count <= buf_count + 1'b1;
                            if (fetch_pc == loop_end_pc) begin
                                state <= ST_ACTIVE; buf_ptr <= 0;
                                buf_count <= buf_count + 1'b1;
                            end
                        end
                    end
                    if (branch_valid && !branch_taken) begin
                        state <= ST_IDLE; buf_count <= 0;
                    end
                end
                ST_ACTIVE: begin
                    lb_instr <= buf_instr[buf_ptr];
                    lb_valid <= 1'b1;
                    if (buf_ptr == buf_count - 1) buf_ptr <= 0;
                    else buf_ptr <= buf_ptr + 1'b1;
                    if (branch_valid && !branch_taken && branch_pc == loop_end_pc) begin
                        state <= ST_IDLE; lb_valid <= 1'b0; buf_ptr <= 0;
                    end
                end
                default: begin state <= ST_IDLE; lb_valid <= 1'b0; end
            endcase
        end
    end
endmodule
