`timescale 1ns / 1ps
//============================================================================
// OrionRV - AXI4-Lite Slave Wrapper
//
// Bridges the AXI4-Lite slave interface to a simple memory interface
// (addr, wdata, rdata, we, wstrb, req, ready). Use this to wrap any
// peripheral or memory block to connect it to the AXI interconnect.
//
// Supports:
//   - Single-cycle slaves (ready always high)
//   - Multi-cycle slaves (ready deasserted until data is available)
//   - Byte strobes for sub-word writes
//============================================================================

module axi_slave_wrapper (
    input  logic        clk,
    input  logic        rst,

    // -------------------------------------------------------------------
    // AXI4-Lite slave interface (from interconnect)
    // -------------------------------------------------------------------
    // Write
    input  logic [31:0] s_awaddr,
    input  logic [31:0] s_wdata,
    input  logic [3:0]  s_wstrb,
    input  logic        s_awvalid,
    input  logic        s_wvalid,
    output logic         s_awready,
    output logic  [1:0]  s_bresp,
    output logic         s_bvalid,
    input  logic        s_bready,

    // Read
    input  logic [31:0] s_araddr,
    input  logic        s_arvalid,
    output logic         s_arready,
    output logic  [31:0] s_rdata,
    output logic  [1:0]  s_rresp,
    output logic         s_rvalid,
    input  logic        s_rready,

    // -------------------------------------------------------------------
    // Simple memory interface (to peripheral / memory)
    // -------------------------------------------------------------------
    output logic  [31:0] mem_addr,
    output logic  [31:0] mem_wdata,
    input  logic [31:0] mem_rdata,
    output logic  [3:0]  mem_wstrb,
    output logic         mem_we,
    output logic         mem_req,
    input  logic        mem_ready
);

    // ===================================================================
    // State machine
    // ===================================================================
    localparam IDLE     = 3'd0;
    localparam WR_EXEC  = 3'd1;
    localparam WR_RESP  = 3'd2;
    localparam RD_EXEC  = 3'd3;
    localparam RD_RESP  = 3'd4;

    logic [2:0] state;

    always_ff @(posedge clk) begin
        if (rst) begin
            state      <= IDLE;
            s_awready  <= 1'b0;
            s_arready  <= 1'b0;
            s_bvalid   <= 1'b0;
            s_rvalid   <= 1'b0;
            s_bresp    <= 2'b00;
            s_rresp    <= 2'b00;
            s_rdata    <= 32'b0;
            mem_addr   <= 32'b0;
            mem_wdata  <= 32'b0;
            mem_wstrb  <= 4'b0;
            mem_we     <= 1'b0;
            mem_req    <= 1'b0;
        end
        else begin
            // Default de-assert
            s_awready <= 1'b0;
            s_arready <= 1'b0;

            case (state)

                IDLE: begin
                    s_bvalid <= 1'b0;
                    s_rvalid <= 1'b0;
                    mem_req  <= 1'b0;

                    if (s_awvalid && s_wvalid) begin
                        // Accept write
                        s_awready <= 1'b1;
                        mem_addr  <= s_awaddr;
                        mem_wdata <= s_wdata;
                        mem_wstrb <= s_wstrb;
                        mem_we    <= 1'b1;
                        mem_req   <= 1'b1;
                        state     <= WR_EXEC;
                    end
                    else if (s_arvalid) begin
                        // Accept read
                        s_arready <= 1'b1;
                        mem_addr  <= s_araddr;
                        mem_we    <= 1'b0;
                        mem_req   <= 1'b1;
                        state     <= RD_EXEC;
                    end
                end

                WR_EXEC: begin
                    if (mem_ready) begin
                        mem_req  <= 1'b0;
                        s_bvalid <= 1'b1;
                        s_bresp  <= 2'b00;  // OKAY
                        state    <= WR_RESP;
                    end
                end

                WR_RESP: begin
                    if (s_bready) begin
                        s_bvalid <= 1'b0;
                        state    <= IDLE;
                    end
                end

                RD_EXEC: begin
                    if (mem_ready) begin
                        mem_req  <= 1'b0;
                        s_rvalid <= 1'b1;
                        s_rdata  <= mem_rdata;
                        s_rresp  <= 2'b00;  // OKAY
                        state    <= RD_RESP;
                    end
                end

                RD_RESP: begin
                    if (s_rready) begin
                        s_rvalid <= 1'b0;
                        state    <= IDLE;
                    end
                end

                default: state <= IDLE;

            endcase
        end
    end

endmodule
