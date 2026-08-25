// ============================================================================
// Prolepsis - Shared AXI and address-map definitions
//
// This package is the SystemVerilog replacement for the legacy AXI macro
// header. Keeping the constants typed and
// namespaced prevents accidental collisions as the SoC grows.
// ============================================================================
package axi_pkg;

    typedef enum logic [1:0] {
        RESP_OKAY   = 2'b00,
        RESP_EXOKAY = 2'b01,
        RESP_SLVERR = 2'b10,
        RESP_DECERR = 2'b11
    } axi_resp_t;

    typedef logic [2:0] qos_t;

    localparam qos_t QOS_LOW          = 3'd0;
    localparam qos_t QOS_NORMAL       = 3'd2;
    localparam qos_t QOS_HIGH         = 3'd4;
    localparam qos_t QOS_CRITICAL     = 3'd7;
    localparam qos_t QOS_PCORE_IFETCH = 3'd3;
    localparam qos_t QOS_PCORE_DATA   = 3'd4;
    localparam qos_t QOS_ECORE_IFETCH = 3'd1;
    localparam qos_t QOS_ECORE_DATA   = 3'd2;
    localparam qos_t QOS_MAC_ACCEL    = 3'd5;
    localparam qos_t QOS_DMA          = 3'd6;

    localparam logic [31:0] ADDR_BOOT_BASE = 32'h0000_0000;
    localparam logic [31:0] ADDR_BOOT_MASK = 32'hFFFF_F000;
    localparam logic [31:0] ADDR_RAM_BASE  = 32'h2000_0000;
    localparam logic [31:0] ADDR_RAM_MASK  = 32'hFF80_0000;
    localparam logic [31:0] ADDR_VGA_BASE  = 32'h4000_0000;
    localparam logic [31:0] ADDR_VGA_MASK  = 32'hFFF0_0000;
    localparam logic [31:0] ADDR_MMIO_BASE = 32'h9000_0000;
    localparam logic [31:0] ADDR_MMIO_MASK = 32'hFFFF_0000;
    localparam logic [31:0] ADDR_MAC_BASE  = 32'hA000_0000;
    localparam logic [31:0] ADDR_MAC_MASK  = 32'hFFFF_FF00;

    localparam logic [3:0] MMIO_SUB_CORE  = 4'h0;
    localparam logic [3:0] MMIO_SUB_UART  = 4'h1;
    localparam logic [3:0] MMIO_SUB_TIMER = 4'h2;

    localparam logic [2:0] SLAVE_BOOT  = 3'd0;
    localparam logic [2:0] SLAVE_RAM   = 3'd1;
    localparam logic [2:0] SLAVE_VGA   = 3'd2;
    localparam logic [2:0] SLAVE_MMIO  = 3'd3;
    localparam logic [2:0] SLAVE_MAC   = 3'd4;
    localparam int unsigned SLAVE_COUNT = 5;
    localparam int unsigned MASTER_COUNT = 11;

    typedef struct packed {
        logic [31:0] addr;
        logic [31:0] data;
        logic [3:0]  strb;
        logic        valid;
        logic        ready;
    } axi_lite_write_t;

    typedef struct packed {
        logic [31:0] data;
        axi_resp_t   resp;
        logic        valid;
        logic        ready;
    } axi_lite_read_t;

endpackage
