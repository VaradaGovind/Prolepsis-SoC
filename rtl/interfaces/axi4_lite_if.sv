// ============================================================================
// Prolepsis - AXI4-Lite interface
//
// The current top-level interconnect remains vector-based for compatibility
// with the existing multi-master fabric.  New leaf blocks can use this
// interface and its modports without repeating channel declarations.
// ============================================================================
interface axi4_lite_if #(
    parameter int ADDR_WIDTH = 32,
    parameter int DATA_WIDTH = 32,
    parameter int STRB_WIDTH = DATA_WIDTH / 8
) (
    input logic clk,
    input logic rst
);
    logic [ADDR_WIDTH-1:0] awaddr;
    logic                  awvalid;
    logic                  awready;
    logic [DATA_WIDTH-1:0] wdata;
    logic [STRB_WIDTH-1:0] wstrb;
    logic                  wvalid;
    logic                  wready;
    logic [1:0]            bresp;
    logic                  bvalid;
    logic                  bready;
    logic [ADDR_WIDTH-1:0] araddr;
    logic                  arvalid;
    logic                  arready;
    logic [DATA_WIDTH-1:0] rdata;
    logic [1:0]            rresp;
    logic                  rvalid;
    logic                  rready;

    modport master (
        input  clk, rst, awready, wready, bresp, bvalid, arready, rdata,
               rresp, rvalid,
        output awaddr, awvalid, wdata, wstrb, wvalid, bready, araddr,
               arvalid, rready
    );

    modport slave (
        input  clk, rst, awaddr, awvalid, wdata, wstrb, wvalid, bready,
               araddr, arvalid, rready,
        output awready, wready, bresp, bvalid, arready, rdata, rresp, rvalid
    );

// Define PROLEPSIS_SVA in a simulator with full SVA support to enable these
// checks. Keeping them opt-in preserves compatibility with lightweight RTL
// simulators used by the regression scripts.
`ifdef PROLEPSIS_SVA
    // AXI VALID must remain asserted until the receiver accepts the beat.
    ap_awvalid_stable: assert property (@(posedge clk) disable iff (rst)
        awvalid && !awready |=> awvalid);
    ap_wvalid_stable: assert property (@(posedge clk) disable iff (rst)
        wvalid && !wready |=> wvalid);
    ap_arvalid_stable: assert property (@(posedge clk) disable iff (rst)
        arvalid && !arready |=> arvalid);
    ap_bvalid_stable: assert property (@(posedge clk) disable iff (rst)
        bvalid && !bready |=> bvalid);
    ap_rvalid_stable: assert property (@(posedge clk) disable iff (rst)
        rvalid && !rready |=> rvalid);
`endif
endinterface
