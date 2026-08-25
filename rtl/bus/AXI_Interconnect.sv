`timescale 1ns / 1ps
//============================================================================
// OrionRV - AXI4-Lite Interconnect
//
// Multi-master, multi-slave interconnect for the hybrid 4+1 core SoC.
//
// Architecture:
//   - N master ports (up to 11: 5xI-cache, 5xdata, 1xMAC)
//   - 5 slave ports (Boot ROM, RAM, VGA, MMIO, MAC Accelerator)
//   - QoS-aware arbitration with age-based starvation prevention
//   - Address-based routing via decoder
//   - AXI4-Lite protocol (no bursts on the master interface;
//     I-cache line fills issue sequential single-word reads)
//
// AXI4-Lite channels implemented:
//   - AW (Write Address)   - simplified: combined with W
//   - W  (Write Data)      - with byte strobes
//   - B  (Write Response)
//   - AR (Read Address)
//   - R  (Read Data + Response)
//
// For FPGA synthesis, this uses a shared-bus topology (not a full crossbar)
// to keep resource usage manageable. The QoS arbiter ensures fairness.
//============================================================================

module axi_interconnect #(
    parameter NUM_MASTERS = 11,
    parameter NUM_SLAVES  = 5,
    parameter ADDR_WIDTH  = 32,
    parameter DATA_WIDTH  = 32,
    parameter STRB_WIDTH  = 4,
    parameter QOS_WIDTH   = 3
)(
    input  logic clk,
    input  logic rst,

    // ===================================================================
    // Master ports (from cores / accelerators)
    // ===================================================================
    // Write address + data (combined for AXI4-Lite simplicity)
    input  logic [NUM_MASTERS*ADDR_WIDTH-1:0] m_awaddr,
    input  logic [NUM_MASTERS*DATA_WIDTH-1:0] m_wdata,
    input  logic [NUM_MASTERS*STRB_WIDTH-1:0] m_wstrb,
    input  logic [NUM_MASTERS-1:0]            m_awvalid,
    output logic [NUM_MASTERS-1:0]            m_awready,
    // Write response
    output logic [NUM_MASTERS*2-1:0]          m_bresp,
    output logic [NUM_MASTERS-1:0]            m_bvalid,
    input  logic [NUM_MASTERS-1:0]            m_bready,

    // Read address
    input  logic [NUM_MASTERS*ADDR_WIDTH-1:0] m_araddr,
    input  logic [NUM_MASTERS-1:0]            m_arvalid,
    output logic [NUM_MASTERS-1:0]            m_arready,
    // Read data
    output logic [NUM_MASTERS*DATA_WIDTH-1:0] m_rdata,
    output logic [NUM_MASTERS*2-1:0]          m_rresp,
    output logic [NUM_MASTERS-1:0]            m_rvalid,
    input  logic [NUM_MASTERS-1:0]            m_rready,

    // QoS level per master
    input  logic [NUM_MASTERS*QOS_WIDTH-1:0]  m_qos,

    // Lock (for atomic LR/SC sequences)
    input  logic [NUM_MASTERS-1:0]            m_lock,

    // ===================================================================
    // Slave ports (to memory / peripherals)
    // ===================================================================
    // Write address + data
    output logic [NUM_SLAVES*ADDR_WIDTH-1:0]  s_awaddr,
    output logic [NUM_SLAVES*DATA_WIDTH-1:0]  s_wdata,
    output logic [NUM_SLAVES*STRB_WIDTH-1:0]  s_wstrb,
    output logic [NUM_SLAVES-1:0]             s_awvalid,
    output logic [NUM_SLAVES-1:0]             s_wvalid,
    input  logic [NUM_SLAVES-1:0]             s_awready,
    // Write response
    input  logic [NUM_SLAVES*2-1:0]           s_bresp,
    input  logic [NUM_SLAVES-1:0]             s_bvalid,
    output logic [NUM_SLAVES-1:0]             s_bready,

    // Read address
    output logic [NUM_SLAVES*ADDR_WIDTH-1:0]  s_araddr,
    output logic [NUM_SLAVES-1:0]             s_arvalid,
    input  logic [NUM_SLAVES-1:0]             s_arready,
    // Read data
    input  logic [NUM_SLAVES*DATA_WIDTH-1:0]  s_rdata,
    input  logic [NUM_SLAVES*2-1:0]           s_rresp,
    input  logic [NUM_SLAVES-1:0]             s_rvalid,
    output logic [NUM_SLAVES-1:0]             s_rready
);

    localparam M_IDX_W = $clog2(NUM_MASTERS);
    localparam S_IDX_W = 3;  // Enough for 5 slaves

    // ===================================================================
    // Internal: unpack master signals for easier indexing
    // ===================================================================
    logic [ADDR_WIDTH-1:0] mi_awaddr [0:NUM_MASTERS-1];
    logic [DATA_WIDTH-1:0] mi_wdata  [0:NUM_MASTERS-1];
    logic [STRB_WIDTH-1:0] mi_wstrb  [0:NUM_MASTERS-1];
    logic [ADDR_WIDTH-1:0] mi_araddr [0:NUM_MASTERS-1];

    genvar g;
    generate
        for (g = 0; g < NUM_MASTERS; g = g + 1) begin : unpack_m
            assign mi_awaddr[g] = m_awaddr[ADDR_WIDTH*g +: ADDR_WIDTH];
            assign mi_wdata[g]  = m_wdata [DATA_WIDTH*g +: DATA_WIDTH];
            assign mi_wstrb[g]  = m_wstrb [STRB_WIDTH*g +: STRB_WIDTH];
            assign mi_araddr[g] = m_araddr[ADDR_WIDTH*g +: ADDR_WIDTH];
        end
    endgenerate

    // ===================================================================
    // Internal: unpack slave signals
    // ===================================================================
    logic [DATA_WIDTH-1:0] si_rdata [0:NUM_SLAVES-1];
    logic [1:0]            si_bresp [0:NUM_SLAVES-1];
    logic [1:0]            si_rresp [0:NUM_SLAVES-1];

    generate
        for (g = 0; g < NUM_SLAVES; g = g + 1) begin : unpack_s
            assign si_rdata[g] = s_rdata[DATA_WIDTH*g +: DATA_WIDTH];
            assign si_bresp[g] = s_bresp[2*g +: 2];
            assign si_rresp[g] = s_rresp[2*g +: 2];
        end
    endgenerate

    // ===================================================================
    // Arbiter input staging
    //
    // Registering request/QoS/lock at the interconnect boundary breaks the
    // long cross-module route from master valid flops into the arbitration
    // cone. This is the dominant setup path in timing reports.
    // ===================================================================
    (* keep = "true", dont_touch = "true" *) logic [NUM_MASTERS-1:0]           wr_req_r;
    (* keep = "true", dont_touch = "true" *) logic [NUM_MASTERS-1:0]           rd_req_r;
    (* keep = "true", dont_touch = "true" *) logic [NUM_MASTERS*QOS_WIDTH-1:0] qos_r;
    (* keep = "true", dont_touch = "true" *) logic [NUM_MASTERS-1:0]           lock_r;

    // Second-stage boundary near arbiter to cut rd_req/qos fanout path.
    (* keep = "true", dont_touch = "true" *) logic [NUM_MASTERS-1:0]           wr_req_arb_r;
    (* keep = "true", dont_touch = "true" *) logic [NUM_MASTERS-1:0]           rd_req_arb_r;
    (* keep = "true", dont_touch = "true" *) logic [NUM_MASTERS*QOS_WIDTH-1:0] qos_arb_r;
    (* keep = "true", dont_touch = "true" *) logic [NUM_MASTERS-1:0]           lock_arb_r;

    always_ff @(posedge clk) begin
        if (rst) begin
            wr_req_r <= {NUM_MASTERS{1'b0}};
            rd_req_r <= {NUM_MASTERS{1'b0}};
            qos_r    <= {(NUM_MASTERS*QOS_WIDTH){1'b0}};
            lock_r   <= {NUM_MASTERS{1'b0}};
            wr_req_arb_r <= {NUM_MASTERS{1'b0}};
            rd_req_arb_r <= {NUM_MASTERS{1'b0}};
            qos_arb_r    <= {(NUM_MASTERS*QOS_WIDTH){1'b0}};
            lock_arb_r   <= {NUM_MASTERS{1'b0}};
        end else begin
            wr_req_r <= m_awvalid;
            rd_req_r <= m_arvalid;
            qos_r    <= m_qos;
            lock_r   <= m_lock;

            wr_req_arb_r <= wr_req_r;
            rd_req_arb_r <= rd_req_r;
            qos_arb_r    <= qos_r;
            lock_arb_r   <= lock_r;
        end
    end

    // ===================================================================
    // Write and Read request detection (staged)
    // ===================================================================
    logic [NUM_MASTERS-1:0] wr_req;
    assign wr_req = wr_req_arb_r;
    logic [NUM_MASTERS-1:0] rd_req;
    assign rd_req = rd_req_arb_r;
    logic [NUM_MASTERS-1:0] any_req;
    assign any_req = wr_req | rd_req;

    // ===================================================================
    // QoS Arbiter -- selects one master per cycle
    // ===================================================================
    logic [NUM_MASTERS-1:0]  arb_grant;
    logic [M_IDX_W-1:0]     arb_grant_idx;
    logic                    arb_grant_valid;

    qos_arbiter #(
        .NUM_MASTERS     (NUM_MASTERS),
        .QOS_WIDTH       (QOS_WIDTH),
        .AGE_BITS        (8),
        .AGE_BOOST_THRESH(200)
    ) arbiter_inst (
        .clk         (clk),
        .rst         (rst),
        .req         (any_req),
        .qos_in      (qos_arb_r),
        .lock        (lock_arb_r),
        .grant       (arb_grant),
        .grant_idx   (arb_grant_idx),
        .grant_valid (arb_grant_valid)
    );

    // Register arbiter outputs at the interconnect boundary to cut
    // direct cross-module fanout from arbiter flops into bus launch logic.
    (* max_fanout = 8 *) logic [M_IDX_W-1:0] arb_grant_idx_r;
    logic                   arb_grant_valid_r;

    always_ff @(posedge clk) begin
        if (rst) begin
            arb_grant_idx_r   <= {M_IDX_W{1'b0}};
            arb_grant_valid_r <= 1'b0;
        end else begin
            arb_grant_idx_r   <= arb_grant_idx;
            arb_grant_valid_r <= arb_grant_valid;
        end
    end

    // ===================================================================
    // Granted master's signals
    // ===================================================================
    logic              grant_req_active;
    assign grant_req_active = wr_req[arb_grant_idx_r] | rd_req[arb_grant_idx_r];
    logic              arb_grant_fire;
    assign arb_grant_fire = arb_grant_valid_r & grant_req_active;
    logic              granted_is_write;
    assign granted_is_write = arb_grant_fire & wr_req[arb_grant_idx_r];
    logic              granted_is_read;
    assign granted_is_read = arb_grant_fire & rd_req[arb_grant_idx_r] & ~wr_req[arb_grant_idx_r];

    logic [31:0] granted_addr;
    assign granted_addr = granted_is_write ? mi_awaddr[arb_grant_idx_r] :
                                                  mi_araddr[arb_grant_idx_r];

    // ===================================================================
    // Address Decoder -- which slave does this address map to?
    // ===================================================================
    logic [S_IDX_W-1:0] target_slave;
    logic               addr_valid;

    address_decoder addr_dec_inst (
        .addr      (granted_addr),
        .slave_sel (target_slave),
        .valid     (addr_valid)
    );

    // ===================================================================
    // Transaction state machine
    // ===================================================================
    localparam ST_IDLE     = 3'd0;
    localparam ST_WR_ADDR  = 3'd1;
    localparam ST_WR_RESP  = 3'd2;
    localparam ST_RD_ADDR  = 3'd3;
    localparam ST_RD_DATA  = 3'd4;
    localparam ST_DECERR   = 3'd5;

    logic [2:0]          state;
    logic [M_IDX_W-1:0]  active_master;
    logic [S_IDX_W-1:0]  active_slave;
    logic                active_is_write;
    logic [ADDR_WIDTH-1:0] active_addr;
    logic [DATA_WIDTH-1:0] active_wdata;
    logic [STRB_WIDTH-1:0] active_wstrb;

    // ===================================================================
    // Slave-side outputs (combinational launch from active transaction)
    // ===================================================================
    logic [NUM_SLAVES-1:0] active_slave_1hot;
    assign active_slave_1hot = ({{(NUM_SLAVES-1){1'b0}}, 1'b1} << active_slave);
    logic [NUM_SLAVES-1:0] wr_launch_sel;
    assign wr_launch_sel = (state == ST_WR_ADDR) ? active_slave_1hot : {NUM_SLAVES{1'b0}};
    logic [NUM_SLAVES-1:0] rd_launch_sel;
    assign rd_launch_sel = (state == ST_RD_ADDR) ? active_slave_1hot : {NUM_SLAVES{1'b0}};
    logic [NUM_SLAVES-1:0] wr_resp_sel;
    assign wr_resp_sel = (state == ST_WR_RESP) ? active_slave_1hot : {NUM_SLAVES{1'b0}};
    logic [NUM_SLAVES-1:0] rd_data_sel;
    assign rd_data_sel = (state == ST_RD_DATA) ? active_slave_1hot : {NUM_SLAVES{1'b0}};
    logic active_master_bready;
    assign active_master_bready = m_bready[active_master];
    logic active_master_rready;
    assign active_master_rready = m_rready[active_master];

    assign s_awvalid = wr_launch_sel;
    assign s_wvalid  = wr_launch_sel;
    assign s_arvalid = rd_launch_sel;
    assign s_bready  = active_master_bready ? wr_resp_sel : {NUM_SLAVES{1'b0}};
    assign s_rready  = active_master_rready ? rd_data_sel : {NUM_SLAVES{1'b0}};

    genvar gs;
    generate
        for (gs = 0; gs < NUM_SLAVES; gs = gs + 1) begin : slave_out_mux
            assign s_awaddr[ADDR_WIDTH*gs +: ADDR_WIDTH] =
                wr_launch_sel[gs] ? active_addr : {ADDR_WIDTH{1'b0}};
            assign s_araddr[ADDR_WIDTH*gs +: ADDR_WIDTH] =
                rd_launch_sel[gs] ? active_addr : {ADDR_WIDTH{1'b0}};
            assign s_wdata[DATA_WIDTH*gs +: DATA_WIDTH] =
                wr_launch_sel[gs] ? active_wdata : {DATA_WIDTH{1'b0}};
            assign s_wstrb[STRB_WIDTH*gs +: STRB_WIDTH] =
                wr_launch_sel[gs] ? active_wstrb : {STRB_WIDTH{1'b0}};
        end
    endgenerate

    // ===================================================================
    // Master-side output registers
    // ===================================================================
    logic [NUM_MASTERS-1:0]            mo_awready;
    logic [NUM_MASTERS-1:0]            mo_arready;
    logic [NUM_MASTERS-1:0]            mo_bvalid;
    logic [NUM_MASTERS*2-1:0]          mo_bresp;
    logic [NUM_MASTERS-1:0]            mo_rvalid;
    logic [NUM_MASTERS*DATA_WIDTH-1:0] mo_rdata;
    logic [NUM_MASTERS*2-1:0]          mo_rresp;

    assign m_awready = mo_awready;
    assign m_arready = mo_arready;
    assign m_bvalid  = mo_bvalid;
    assign m_bresp   = mo_bresp;
    assign m_rvalid  = mo_rvalid;
    assign m_rdata   = mo_rdata;
    assign m_rresp   = mo_rresp;

    // ===================================================================
    // Main state machine
    // ===================================================================
    always_ff @(posedge clk) begin
        if (rst) begin
            state         <= ST_IDLE;
            active_master <= 0;
            active_slave  <= 0;
            active_addr   <= 0;
            active_wdata  <= 0;
            active_wstrb  <= 0;
            mo_awready    <= 0;
            mo_arready    <= 0;
            mo_bvalid     <= 0;
            mo_bresp      <= 0;
            mo_rvalid     <= 0;
            mo_rdata      <= 0;
            mo_rresp      <= 0;
        end
        else begin
            // Default: clear transient handshake signals
            mo_awready <= 0;
            mo_arready <= 0;
            mo_bvalid  <= 0;
            mo_rvalid  <= 0;

            case (state)

                // ---------------------------------------------------------
                // IDLE: Accept a new transaction from the arbiter
                // ---------------------------------------------------------
                ST_IDLE: begin
                    if (arb_grant_fire) begin
                        active_master     <= arb_grant_idx_r;
                        active_slave      <= target_slave;

                        if (!addr_valid) begin
                            // Unmapped address -- return decode error
                            active_is_write <= granted_is_write;
                            state           <= ST_DECERR;
                        end
                        else if (granted_is_write) begin
                            // Write transaction: latch payload, launch next cycle.
                            active_is_write <= 1'b1;
                            active_addr     <= mi_awaddr[arb_grant_idx_r];
                            active_wdata    <= mi_wdata[arb_grant_idx_r];
                            active_wstrb    <= mi_wstrb[arb_grant_idx_r];
                            state <= ST_WR_ADDR;
                        end
                        else begin
                            // Read transaction: latch address, launch next cycle.
                            active_is_write <= 1'b0;
                            active_addr     <= mi_araddr[arb_grant_idx_r];
                            state <= ST_RD_ADDR;
                        end
                    end
                end

                // ---------------------------------------------------------
                // Write address/data launch
                // ---------------------------------------------------------
                ST_WR_ADDR: begin
                    if (s_awready[active_slave]) begin
                        mo_awready[active_master] <= 1'b1;
                        state <= ST_WR_RESP;
                    end
                end

                // ---------------------------------------------------------
                // Write: wait for slave write response
                // ---------------------------------------------------------
                ST_WR_RESP: begin
                    // Only consume B channel when the selected master is
                    // ready; otherwise keep waiting so the slave response
                    // is not dropped/stuck.
                    if (s_bvalid[active_slave] && active_master_bready) begin
                        // Forward response to master
                        mo_bvalid[active_master] <= 1'b1;
                        mo_bresp[2*active_master +: 2] <= si_bresp[active_slave];

                        state <= ST_IDLE;
                    end
                end

                // ---------------------------------------------------------
                // Read: wait for slave read data
                // ---------------------------------------------------------
                ST_RD_ADDR: begin
                    if (s_arready[active_slave]) begin
                        mo_arready[active_master] <= 1'b1;
                        state <= ST_RD_DATA;
                    end
                end

                // ---------------------------------------------------------
                // Read: wait for slave read data
                // ---------------------------------------------------------
                ST_RD_DATA: begin
                    // Complete R-channel transfer only when both slave has
                    // data and selected master is ready. This prevents
                    // stale rvalid from blocking subsequent fetches.
                    if (s_rvalid[active_slave] && active_master_rready) begin
                        // Forward data + response to master
                        mo_rvalid[active_master] <= 1'b1;
                        mo_rdata[DATA_WIDTH*active_master +: DATA_WIDTH] <= si_rdata[active_slave];
                        mo_rresp[2*active_master +: 2] <= si_rresp[active_slave];

                        state <= ST_IDLE;
                    end
                end

                // ---------------------------------------------------------
                // Decode error: return error response to master
                // ---------------------------------------------------------
                ST_DECERR: begin
                    if (active_is_write) begin
                        mo_bvalid[active_master] <= 1'b1;
                        mo_bresp[2*active_master +: 2] <= axi_pkg::RESP_DECERR;
                    end
                    else begin
                        mo_rvalid[active_master] <= 1'b1;
                        mo_rdata[DATA_WIDTH*active_master +: DATA_WIDTH] <= 32'hDEAD_BEEF;
                        mo_rresp[2*active_master +: 2] <= axi_pkg::RESP_DECERR;
                    end
                    state <= ST_IDLE;
                end

                default: state <= ST_IDLE;

            endcase
        end
    end

endmodule
