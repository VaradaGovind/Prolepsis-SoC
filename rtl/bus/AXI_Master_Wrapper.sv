`timescale 1ns / 1ps
//============================================================================
// OrionRV - AXI4-Lite Master Wrapper
//
// Bridges a simple memory interface (from rv32_core data/instruction ports)
// to the AXI4-Lite master interface for the interconnect.
//
// Translates:
//   req + we -> AXI write (AW+W channel) or read (AR channel)
//   ready    <- AXI write response (B) or read data (R)
//============================================================================

module axi_master_wrapper #(
    parameter QOS_LEVEL = 3'd2   // Default QoS for this master (can be overridden)
)(
    input  logic        clk,
    input  logic        rst,
    input  logic [2:0]  qos_override, // Dynamic QoS support

    // -------------------------------------------------------------------
    // Simple memory interface (from core)
    // -------------------------------------------------------------------
    input  logic [31:0] mem_addr,
    input  logic [31:0] mem_wdata,
    output logic  [31:0] mem_rdata,
    input  logic [3:0]  mem_wstrb,
    input  logic        mem_we,
    input  logic        mem_req,
    output logic         mem_ready,

    // Atomic lock signal (for LR/SC)
    input  logic        mem_lock,

    // -------------------------------------------------------------------
    // AXI4-Lite master interface (to interconnect)
    // -------------------------------------------------------------------
    // Write
    output logic  [31:0] m_awaddr,
    output logic  [31:0] m_wdata,
    output logic  [3:0]  m_wstrb,
    output logic         m_awvalid,
    input  logic        m_awready,
    input  logic [1:0]  m_bresp,
    input  logic        m_bvalid,
    output logic         m_bready,

    // Read
    output logic  [31:0] m_araddr,
    output logic         m_arvalid,
    input  logic        m_arready,
    input  logic [31:0] m_rdata,
    input  logic [1:0]  m_rresp,
    input  logic        m_rvalid,
    output logic         m_rready,

    // QoS output
    output logic [2:0]  m_qos,

    // Lock output
    output logic        m_lock
);

    // Top-level already provides default and adjusted QoS, including legal 0.
    logic [2:0] m_qos_reg;
    always_ff @(posedge clk) begin
        if (rst) m_qos_reg <= QOS_LEVEL;
        else m_qos_reg <= qos_override;
    end
    assign m_qos = m_qos_reg;
    assign m_lock = mem_lock;

    // ===================================================================
    // State machine
    // ===================================================================
    localparam IDLE      = 3'd0;
    localparam WR_ADDR   = 3'd1;
    localparam WR_RESP   = 3'd2;
    localparam RD_ADDR   = 3'd3;
    localparam RD_DATA   = 3'd4;

    logic [2:0] state;
    logic      write_resp_ok;
    assign write_resp_ok = (m_bresp == 2'b00);
    logic      read_resp_ok;
    assign read_resp_ok = (m_rresp == 2'b00);

    always_ff @(posedge clk) begin
        if (rst) begin
            state     <= IDLE;
            m_awvalid <= 1'b0;
            m_arvalid <= 1'b0;
            m_bready  <= 1'b0;
            m_rready  <= 1'b0;
            mem_ready <= 1'b0;
            m_awaddr  <= 32'b0;
            m_araddr  <= 32'b0;
            m_wdata   <= 32'b0;
            m_wstrb   <= 4'b0;
            mem_rdata <= 32'b0;
        end
        else begin
            case (state)

                IDLE: begin
                    mem_ready <= 1'b0;
                    m_awvalid <= 1'b0;
                    m_arvalid <= 1'b0;
                    m_bready  <= 1'b0;
                    m_rready  <= 1'b0;

                    if (mem_req && mem_we) begin
                        // Write request
                        m_awaddr  <= mem_addr;
                        m_wdata   <= mem_wdata;
                        m_wstrb   <= mem_wstrb;
                        m_awvalid <= 1'b1;
                        state     <= WR_ADDR;
                    end
                    else if (mem_req && !mem_we) begin
                        // Read request
                        m_araddr  <= mem_addr;
                        m_arvalid <= 1'b1;
                        state     <= RD_ADDR;
                    end
                end

                // Write: present address+data, wait for awready
                WR_ADDR: begin
                    if (m_awready) begin
                        m_awvalid <= 1'b0;
                        m_bready  <= 1'b1;
                        state     <= WR_RESP;
                    end
                end

                // Write: wait for write response
                WR_RESP: begin
                    if (m_bvalid) begin
                        m_bready  <= 1'b0;
                        if (!write_resp_ok)
                            mem_rdata <= 32'hBADB_0000;
                        mem_ready <= 1'b1;
                        state     <= IDLE;
                    end
                end

                // Read: present address, wait for arready
                RD_ADDR: begin
                    if (m_arready) begin
                        m_arvalid <= 1'b0;
                        m_rready  <= 1'b1;
                        state     <= RD_DATA;
                    end
                end

                // Read: wait for data
                RD_DATA: begin
                    if (m_rvalid) begin
                        mem_rdata <= read_resp_ok ? m_rdata : {30'd0, m_rresp};
                        m_rready  <= 1'b0;
                        mem_ready <= 1'b1;
                        state     <= IDLE;
                    end
                end

                default: state <= IDLE;

            endcase
        end
    end

endmodule
