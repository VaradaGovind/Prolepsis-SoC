`timescale 1ns / 1ps
module Boot_ROM (
    input  logic        clk,
    input  logic        rst_n,
    
    // AXI4-Lite Slave Interface
    input  logic [31:0] s_axi_awaddr,
    input  logic        s_axi_awvalid,
    output logic        s_axi_awready,
    input  logic [31:0] s_axi_wdata,
    input  logic [3:0]  s_axi_wstrb,
    input  logic        s_axi_wvalid,
    output logic        s_axi_wready,
    output logic [1:0]  s_axi_bresp,
    output logic        s_axi_bvalid,
    input  logic        s_axi_bready,
    input  logic [31:0] s_axi_araddr,
    input  logic        s_axi_arvalid,
    output logic        s_axi_arready,
    output logic [31:0] s_axi_rdata,
    output logic [1:0]  s_axi_rresp,
    output logic        s_axi_rvalid,
    input  logic        s_axi_rready
);

    logic [31:0] rom_data [0:1023]; // 4KB ROM
    localparam [31:0] ROM_NOP = 32'h0000_0013;
    parameter BOOT_HEX_FILE = "boot.hex";

    integer i;

    initial begin
        // Initialize to NOP so missing hex files do not leave unknown contents.
        for (i = 0; i < 1024; i = i + 1)
            rom_data[i] = ROM_NOP;
        $readmemh(BOOT_HEX_FILE, rom_data);
    end

    logic bvalid_reg;
    logic boot_wr_in_range;
    assign boot_wr_in_range = (s_axi_awaddr < 32'd4096);
    logic boot_rd_in_range;
    assign boot_rd_in_range = (s_axi_araddr < 32'd4096);
    logic wr_payload_parity;
    assign wr_payload_parity = ^{s_axi_wdata, s_axi_wstrb};

    // Read-only slave: accept write address/data and return SLVERR response.
    assign s_axi_awready = !bvalid_reg;
    assign s_axi_wready  = !bvalid_reg;
    assign s_axi_bvalid  = bvalid_reg;
    assign s_axi_bresp   = boot_wr_in_range ? 2'b10 : 2'b11; // SLVERR / DECERR

    logic rvalid_reg;
    logic [31:0] rdata_reg;

    always_ff @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            rdata_reg <= 32'b0;
            rvalid_reg <= 1'b0;
            bvalid_reg <= 1'b0;
        end else begin
            if (s_axi_awvalid && s_axi_wvalid && s_axi_awready)
                bvalid_reg <= 1'b1;
            else if (s_axi_bready && bvalid_reg)
                bvalid_reg <= 1'b0;

            if (s_axi_arvalid && s_axi_arready) begin
                rvalid_reg <= 1'b1;
                if (boot_rd_in_range)
                    rdata_reg <= rom_data[s_axi_araddr[11:2]]; // Read from ROM
                else
                    rdata_reg <= {29'b0, wr_payload_parity, s_axi_awaddr[31], s_axi_araddr[31]};
            end else if (s_axi_rready && rvalid_reg) begin
                rvalid_reg <= 1'b0;
            end
        end
    end

    assign s_axi_arready = !rvalid_reg;
    assign s_axi_rvalid  = rvalid_reg;
    assign s_axi_rdata   = rdata_reg;
    assign s_axi_rresp   = 2'b00;

endmodule
