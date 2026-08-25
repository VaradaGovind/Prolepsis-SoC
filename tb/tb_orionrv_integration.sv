`timescale 1ns / 1ps

module tb_orionrv_integration;
    logic clk;
    logic rst;
    
    logic [3:0] buttons;
    logic [1:0] sw;
    
    logic [7:0] leds;
    logic [6:0] seg;
    logic       dp;
    logic [7:0] an;
    logic [3:0] vga_r;
    logic [3:0] vga_g;
    logic [3:0] vga_b;
    logic       vga_hsync;
    logic       vga_vsync;
    logic       uart_tx;
    
    orionrv dut (
        .clk(clk),
        .rst(rst),
        .buttons(buttons),
        .leds(leds),
        .sw(sw),
        .seg(seg),
        .dp(dp),
        .an(an),
        .vga_r(vga_r),
        .vga_g(vga_g),
        .vga_b(vga_b),
        .vga_hsync(vga_hsync),
        .vga_vsync(vga_vsync),
        .uart_rx(1'b1),
        .uart_tx(uart_tx)
    );

    always #5 clk = ~clk;

    task press_and_check_mode;
        input [3:0] btn_mask;
        input [1:0] expected_mode;
        begin
            buttons = btn_mask;
            repeat (4) @(posedge clk);
            buttons = 4'b0000;
            repeat (4) @(posedge clk);

            if (dut.eval_active_mode !== expected_mode) begin
                $display("FAIL: D-pad mode mismatch expected=%0d observed=%0d", expected_mode, dut.eval_active_mode);
                $finish;
            end
        end
    endtask

    initial begin
        clk = 0;
        // The reset in OrionRV top module is active low due to Nexys4DDR
        rst = 0;
        buttons = 0;
        sw = 0;
        
        #50 rst = 1;

        // Allow reset deassertion and internal synchronizers to settle.
        repeat (12) @(posedge clk);

        if (dut.eval_active_mode !== 2'd3) begin
            $display("FAIL: Expected default mode 3 after reset, observed=%0d", dut.eval_active_mode);
            $finish;
        end

        // D-pad mapping in this build: BTNC/BTNU/BTNL/BTNR -> modes 0/1/2/3.
        press_and_check_mode(4'b0001, 2'd0);
        press_and_check_mode(4'b0010, 2'd1);
        press_and_check_mode(4'b0100, 2'd2);
        press_and_check_mode(4'b1000, 2'd3);

        // Let the rest of the integration run briefly after mode changes.
        #300;
        
        $display("PASS: OrionRV integration + D-pad mode control verified");
        $finish;
    end

endmodule
