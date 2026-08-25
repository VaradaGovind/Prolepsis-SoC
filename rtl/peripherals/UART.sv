`timescale 1ns / 1ps
module UART (
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
    
    // UART interface
    input  logic        rx,
    output logic        tx
);

    localparam integer CLKS_PER_BIT = 868; // ~115200 baud @ 100 MHz

    logic [7:0] tx_data;
    logic tx_start;
    logic tx_busy;
    logic tx_reg;
    logic [3:0] tx_bit_idx;
    logic [9:0] tx_shift;
    logic [15:0] tx_baud_ctr;
    logic bvalid_reg;
    logic wr_fire;
    assign wr_fire = s_axi_awvalid && s_axi_wvalid && s_axi_awready;
    logic awaddr_hi_nonzero;
    assign awaddr_hi_nonzero = |s_axi_awaddr[31:8];
    logic araddr_hi_nonzero;
    assign araddr_hi_nonzero = |s_axi_araddr[31:8];
    logic wdata_hi_nonzero;
    assign wdata_hi_nonzero = |s_axi_wdata[31:8];
    logic wstrb_any;
    assign wstrb_any = |s_axi_wstrb;
    logic [7:0] uart_status;
    assign uart_status = {rx, tx_busy, tx_start, awaddr_hi_nonzero,
                              araddr_hi_nonzero, wdata_hi_nonzero, wstrb_any, 1'b0};
    
    assign tx = tx_reg;

    // Simple AXI-Lite responses
    assign s_axi_awready = !bvalid_reg;
    assign s_axi_wready  = !bvalid_reg;
    assign s_axi_bvalid  = bvalid_reg;
    assign s_axi_bresp   = 2'b00;

    logic rvalid_reg;
    logic [31:0] rdata_reg;

    always_ff @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            rdata_reg <= 32'b0;
            rvalid_reg <= 1'b0;
            tx_data <= 8'b0;
            tx_start <= 1'b0;
            tx_busy <= 1'b0;
            tx_reg <= 1'b1;
            tx_bit_idx <= 4'd0;
            tx_shift <= 10'h3FF;
            tx_baud_ctr <= 16'd0;
            bvalid_reg <= 1'b0;
        end else begin
            if (wr_fire && s_axi_awaddr[7:0] == 8'h08 && s_axi_wstrb[0]) begin
                tx_data <= s_axi_wdata[7:0];
                tx_start <= 1'b1;
            end else begin
                tx_start <= 1'b0;
            end

            if (wr_fire)
                bvalid_reg <= 1'b1;
            else if (s_axi_bready && bvalid_reg)
                bvalid_reg <= 1'b0;

            if (s_axi_arvalid && s_axi_arready) begin
                rvalid_reg <= 1'b1;
                case (s_axi_araddr[7:0])
                    8'h08: rdata_reg <= {24'b0, tx_data}; // TX payload shadow
                    8'h0C: rdata_reg <= {24'b0, uart_status}; // Status/debug
                    default: rdata_reg <= 32'b0;
                endcase
            end else if (s_axi_rready && rvalid_reg) begin
                rvalid_reg <= 1'b0;
            end

            // Minimal UART TX engine: start bit, 8 data bits LSB-first, stop bit.
            if (!tx_busy) begin
                tx_reg <= 1'b1;
                tx_baud_ctr <= 16'd0;
                tx_bit_idx <= 4'd0;
                if (tx_start) begin
                    tx_busy <= 1'b1;
                    tx_shift <= {1'b1, tx_data, 1'b0};
                    tx_reg <= 1'b0;
                end
            end else begin
                if (tx_baud_ctr == (CLKS_PER_BIT - 1)) begin
                    tx_baud_ctr <= 16'd0;
                    tx_bit_idx <= tx_bit_idx + 4'd1;
                    tx_shift <= {1'b1, tx_shift[9:1]};
                    tx_reg <= tx_shift[1];

                    if (tx_bit_idx == 4'd9) begin
                        tx_busy <= 1'b0;
                        tx_reg <= 1'b1;
                    end
                end else begin
                    tx_baud_ctr <= tx_baud_ctr + 16'd1;
                end
            end
        end
    end

    assign s_axi_arready = !rvalid_reg;
    assign s_axi_rvalid  = rvalid_reg;
    assign s_axi_rdata   = rdata_reg;
    assign s_axi_rresp   = 2'b00;

endmodule
