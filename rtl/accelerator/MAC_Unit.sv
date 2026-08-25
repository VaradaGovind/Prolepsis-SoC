`timescale 1ns / 1ps
module MAC_Unit (
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

    logic [31:0] operand_a;
    logic [31:0] operand_b;
    logic [63:0] accumulator;
    logic [31:0] control_status; // 0: Start, 1: Busy, 2: Done, 3: Clear, 4-7: Mode
    logic        bvalid_reg;
    logic [3:0]  op_mode;

    // Modes:
    // 0: Standard 32x32 = 64 MAC
    // 1: Q16.16 Fixed Point MAC
    // 2: Newton-Raphson Reciprocal (Division support)
    
    logic wr_fire;
    assign wr_fire = s_axi_awvalid && s_axi_wvalid && s_axi_awready;
    logic awaddr_hi_nonzero;
    assign awaddr_hi_nonzero = |s_axi_awaddr[31:8];
    logic araddr_hi_nonzero;
    assign araddr_hi_nonzero = |s_axi_araddr[31:8];
    logic we_a;
    assign we_a = (s_axi_awaddr[7:0] == 8'h00) && wr_fire;
    logic we_b;
    assign we_b = (s_axi_awaddr[7:0] == 8'h04) && wr_fire;
    logic we_accL;
    assign we_accL = (s_axi_awaddr[7:0] == 8'h08) && wr_fire;
    logic we_accH;
    assign we_accH = (s_axi_awaddr[7:0] == 8'h0C) && wr_fire;
    logic we_ctrl;
    assign we_ctrl = (s_axi_awaddr[7:0] == 8'h10) && wr_fire;

    logic [31:0] control_status_next;
    logic [63:0] accumulator_next;
    logic [63:0] mult_result_next;
    logic        mac_busy_phase2_next;
    logic [3:0]  op_mode_next;
    
    // Pipeline register for multiplier
    (* keep = "true", dont_touch = "true" *) logic [63:0] mult_result;
    
    // Extra state tracking
    logic mac_busy_phase2;

    always_comb begin
        // Next-state defaults
        control_status_next = control_status;
        accumulator_next    = accumulator;
        mult_result_next    = mult_result;
        mac_busy_phase2_next = mac_busy_phase2;
        op_mode_next        = op_mode;

        if (we_accL)
            accumulator_next[31:0] = apply_wstrb32(accumulator_next[31:0], s_axi_wdata, s_axi_wstrb);
        if (we_accH)
            accumulator_next[63:32] = apply_wstrb32(accumulator_next[63:32], s_axi_wdata, s_axi_wstrb);
        if (we_ctrl)
            control_status_next = apply_wstrb32(control_status_next, s_axi_wdata, s_axi_wstrb);

        // Basic execution dummy logic
        if (control_status_next[0] && !control_status_next[1]) begin // START signal
            control_status_next[1] = 1'b1; // BUSY
            control_status_next[0] = 1'b0; // CLEAR START
            mult_result_next = {32'b0, operand_a} * {32'b0, operand_b};
            // Latch mode at start so execution phase is decoupled from
            // live control register writes.
            op_mode_next = control_status_next[7:4];
            mac_busy_phase2_next = 1'b1;
        end else if (mac_busy_phase2_next) begin
            mac_busy_phase2_next = 1'b0;
            case (op_mode)
                4'd0: accumulator_next = accumulator_next + mult_result;
                4'd1: accumulator_next = accumulator_next + (mult_result >> 16);
                4'd2: accumulator_next = 64'b1; // NR Subbed out for testing temporarily
                default: accumulator_next = accumulator_next;
            endcase
        end else if (control_status_next[1]) begin
            control_status_next[1] = 1'b0; // End BUSY
            control_status_next[2] = 1'b1; // DONE
        end else if (control_status_next[3]) begin
            accumulator_next = 64'b0; // CLEAR
            control_status_next[3] = 1'b0;
            control_status_next[2] = 1'b0;
        end
    end

    always_ff @(posedge clk) begin
        if (!rst_n) begin
            operand_a <= 32'b0;
            operand_b <= 32'b0;
            accumulator <= 64'b0;
            control_status <= 32'b0;
            bvalid_reg <= 1'b0;
            mult_result <= 64'b0;
            mac_busy_phase2 <= 1'b0;
            op_mode <= 4'b0;
        end else begin
            if (wr_fire)
                bvalid_reg <= 1'b1;
            else if (s_axi_bready && bvalid_reg)
                bvalid_reg <= 1'b0;

            if (we_a) operand_a <= apply_wstrb32(operand_a, s_axi_wdata, s_axi_wstrb);
            if (we_b) operand_b <= apply_wstrb32(operand_b, s_axi_wdata, s_axi_wstrb);

            control_status <= control_status_next;
            accumulator    <= accumulator_next;
            mult_result    <= mult_result_next;
            mac_busy_phase2 <= mac_busy_phase2_next;
            op_mode <= op_mode_next;
        end
    end

    logic [31:0] rdata_reg;
    logic rvalid_reg;
    always_ff @(posedge clk) begin
        if (!rst_n) begin
            rdata_reg <= 32'b0;
            rvalid_reg <= 1'b0;
        end else begin
            if (s_axi_arvalid && s_axi_arready) begin
                rvalid_reg <= 1'b1;
                case (s_axi_araddr[7:0])
                    8'h00: rdata_reg <= operand_a;
                    8'h04: rdata_reg <= operand_b;
                    8'h08: rdata_reg <= accumulator[31:0];
                    8'h0C: rdata_reg <= accumulator[63:32];
                    8'h10: rdata_reg <= control_status;
                    default: rdata_reg <= {30'b0, awaddr_hi_nonzero, araddr_hi_nonzero};
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
