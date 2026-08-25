`timescale 1ns / 1ps
module Timer (
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
    input  logic        s_axi_rready,
    
    // Timer Interrupt
    output logic        timer_irq
);

    function [31:0] apply_wstrb32;
        input [31:0] cur;
        input [31:0] wdata;
        input [3:0]  wstrb;
        begin
            apply_wstrb32 = cur;
            if (wstrb[0]) apply_wstrb32[7:0]   = wdata[7:0];
            if (wstrb[1]) apply_wstrb32[15:8]  = wdata[15:8];
            if (wstrb[2]) apply_wstrb32[23:16] = wdata[23:16];
            if (wstrb[3]) apply_wstrb32[31:24] = wdata[31:24];
        end
    endfunction

    logic [63:0] mtime;
    logic [63:0] mtimecmp;
    logic        bvalid_reg;

    logic wr_fire;
    assign wr_fire = s_axi_awvalid && s_axi_wvalid && s_axi_awready;
    logic awaddr_hi_nonzero;
    assign awaddr_hi_nonzero = |s_axi_awaddr[31:8];
    logic araddr_hi_nonzero;
    assign araddr_hi_nonzero = |s_axi_araddr[31:8];
    logic [5:0] timer_dbg_flags;
    assign timer_dbg_flags = {awaddr_hi_nonzero, araddr_hi_nonzero, s_axi_wstrb};

    logic we_mtime_low;
    assign we_mtime_low = (s_axi_awaddr[7:0] == 8'h10) && wr_fire;
    logic we_mtime_high;
    assign we_mtime_high = (s_axi_awaddr[7:0] == 8'h14) && wr_fire;
    logic we_mtimecmp_low;
    assign we_mtimecmp_low = (s_axi_awaddr[7:0] == 8'h18) && wr_fire;
    logic we_mtimecmp_high;
    assign we_mtimecmp_high = (s_axi_awaddr[7:0] == 8'h1C) && wr_fire;

    always_ff @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            mtime <= 64'b0;
            mtimecmp <= 64'hFFFFFFFFFFFFFFFF;
            bvalid_reg <= 1'b0;
        end else begin
            mtime <= mtime + 1'b1;

            if (wr_fire)
                bvalid_reg <= 1'b1;
            else if (s_axi_bready && bvalid_reg)
                bvalid_reg <= 1'b0;
            
            if (we_mtime_low) mtime[31:0] <= apply_wstrb32(mtime[31:0], s_axi_wdata, s_axi_wstrb);
            if (we_mtime_high) mtime[63:32] <= apply_wstrb32(mtime[63:32], s_axi_wdata, s_axi_wstrb);
            if (we_mtimecmp_low) mtimecmp[31:0] <= apply_wstrb32(mtimecmp[31:0], s_axi_wdata, s_axi_wstrb);
            if (we_mtimecmp_high) mtimecmp[63:32] <= apply_wstrb32(mtimecmp[63:32], s_axi_wdata, s_axi_wstrb);
        end
    end

    assign timer_irq = (mtime >= mtimecmp);

    logic [31:0] rdata_reg;
    logic rvalid_reg;
    always_ff @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            rdata_reg <= 32'b0;
            rvalid_reg <= 1'b0;
        end else begin
            if (s_axi_arvalid && s_axi_arready) begin
                rvalid_reg <= 1'b1;
                case (s_axi_araddr[7:0])
                    8'h10: rdata_reg <= mtime[31:0];
                    8'h14: rdata_reg <= mtime[63:32];
                    8'h18: rdata_reg <= mtimecmp[31:0];
                    8'h1C: rdata_reg <= mtimecmp[63:32];
                    default: rdata_reg <= {26'b0, timer_dbg_flags};
                endcase
            end else if (s_axi_rready && s_axi_rvalid) begin
                rvalid_reg <= 1'b0;
            end
        end
    end

    // Simple AXI-Lite responses
    assign s_axi_awready = !bvalid_reg;
    assign s_axi_wready  = !bvalid_reg;
    assign s_axi_bvalid  = bvalid_reg;
    assign s_axi_bresp   = 2'b00;

    assign s_axi_arready = !rvalid_reg;
    assign s_axi_rvalid  = rvalid_reg;
    assign s_axi_rdata   = rdata_reg;
    assign s_axi_rresp   = 2'b00;

endmodule
