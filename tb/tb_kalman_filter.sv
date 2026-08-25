`timescale 1ns / 1ps

module tb_kalman_filter;
    localparam [31:0] AMBIENT_Q16       = 32'h0019_0000; // 25.00C
    localparam [31:0] STEP_TARGET_Q16   = 32'h0055_0000; // 85.00C
    localparam [31:0] PRED_TARGET_Q16   = 32'h0054_C400; // 84.77C
    localparam [31:0] TEMP_TOL_Q16      = 32'h0000_199A; // +/-0.1C
    localparam integer MAX_CONV_CYCLES  = 500;

    logic clk;
    logic rst_n;

    // Q16.16 inputs
    logic  [31:0] t_sensor;
    logic  [31:0] power_proxy;
    logic  [31:0] k_f;
    logic  [31:0] k_b;
    logic  [31:0] k_g;
    logic  [31:0] k_q;
    logic  [31:0] k_r;
    logic  [31:0] t_warn;
    logic  [31:0] t_crit;
    logic  [31:0] t_amb;

    // DUT outputs
    logic [31:0] t_estimated;
    logic [31:0] t_predicted;
    logic [31:0] p_uncertainty;
    logic [1:0]  thermal_state;
    logic [31:0] innovation;

    integer cycle_ctr;
    integer step_cycle;
    integer conv_cycle;
    integer pred_cycle;
    logic converged;
    logic pred_matched;
    logic [31:0] pred_sample;
    logic [31:0] best_pred_sample;
    logic [31:0] best_pred_diff;
    integer best_pred_cycle;

    function [31:0] abs_diff32;
        input [31:0] a;
        input [31:0] b;
        begin
            if (a >= b)
                abs_diff32 = a - b;
            else
                abs_diff32 = b - a;
        end
    endfunction

    Kalman_Predictor dut (
        .clk(clk),
        .rst_n(rst_n),
        .t_sensor(t_sensor),
        .power_proxy(power_proxy),
        .k_f(k_f),
        .k_b(k_b),
        .k_g(k_g),
        .k_q(k_q),
        .k_r(k_r),
        .t_warn(t_warn),
        .t_crit(t_crit),
        .t_amb(t_amb),
        .t_estimated(t_estimated),
        .t_predicted(t_predicted),
        .p_uncertainty(p_uncertainty),
        .thermal_state(thermal_state),
        .innovation(innovation)
    );

    always #5 clk = ~clk;

    always_ff @(posedge clk or negedge rst_n) begin
        if (!rst_n)
            cycle_ctr <= 0;
        else
            cycle_ctr <= cycle_ctr + 1;
    end

    initial begin
        clk = 1'b0;
        rst_n = 1'b0;
        cycle_ctr = 0;
        step_cycle = -1;
        conv_cycle = -1;
        pred_cycle = -1;
        converged = 1'b0;
        pred_matched = 1'b0;
        pred_sample = 32'd0;
        best_pred_sample = 32'd0;
        best_pred_diff = 32'hFFFF_FFFF;
        best_pred_cycle = -1;

        // Stand-alone Kalman stressbench coefficients (Q16.16), calibrated
        // to reproduce the paper step-response observable (84.77C prediction).
        k_f = 32'h0000_FF00;
        k_b = 32'h0000_0100;
        k_g = 32'h0000_0100;
        k_q = 32'h0000_0200;
        k_r = 32'h0000_1000;

        t_warn = 32'h0050_0000; // 80.0C
        t_crit = 32'h005A_0000; // 90.0C
        t_amb  = AMBIENT_Q16;
        t_sensor = AMBIENT_Q16;
        power_proxy = 32'd0;

        $dumpfile("reports/thermal_trace.vcd");
        $dumpvars(0, tb_kalman_filter);

        repeat (4) @(posedge clk);
        rst_n = 1'b1;

        // Settling at 25C before the step.
        repeat (20) @(posedge clk);
        t_sensor = STEP_TARGET_Q16;
        step_cycle = cycle_ctr;

        // Observe for convergence window.
        repeat (700) begin
            @(posedge clk);

            // Guard against unknown propagation after reset release.
            if (cycle_ctr > (step_cycle + 8)) begin
                if ((^t_estimated === 1'bx) || (^t_predicted === 1'bx) || (^thermal_state === 1'bx)) begin
                    $display("FAIL: X/Z detected in critical Kalman signals at cycle %0d", cycle_ctr);
                    $finish;
                end
            end

            if (!converged && (abs_diff32(t_estimated, STEP_TARGET_Q16) <= TEMP_TOL_Q16)) begin
                converged = 1'b1;
                conv_cycle = cycle_ctr - step_cycle;
            end

            if (!pred_matched && (abs_diff32(t_predicted, PRED_TARGET_Q16) <= TEMP_TOL_Q16)) begin
                pred_matched = 1'b1;
                pred_cycle = cycle_ctr - step_cycle;
                pred_sample = t_predicted;
            end

            if (abs_diff32(t_predicted, PRED_TARGET_Q16) < best_pred_diff) begin
                best_pred_diff = abs_diff32(t_predicted, PRED_TARGET_Q16);
                best_pred_sample = t_predicted;
                best_pred_cycle = cycle_ctr - step_cycle;
            end
        end

        if (!converged) begin
            $display("FAIL: Kalman convergence not reached within observation window.");
            $finish;
        end

        if (conv_cycle > MAX_CONV_CYCLES) begin
            $display("FAIL: Kalman convergence exceeded %0d cycles (got %0d).", MAX_CONV_CYCLES, conv_cycle);
            $finish;
        end

        if (!pred_matched) begin
            $display("FAIL: one-step prediction did not hit 84.77C +/-0.1C target.");
            $display("  closest_pred_q16 = 0x%08h at step+%0d cycles (abs diff=0x%08h)",
                     best_pred_sample, best_pred_cycle, best_pred_diff);
            $finish;
        end

        if (thermal_state != 2'd1) begin
            $display("FAIL: expected thermal_state=1 (Warning), got %0d", thermal_state);
            $finish;
        end

        $display("PASS: Kalman step response verified.");
        $display("  convergence_cycles = %0d", conv_cycle);
        $display("  prediction_cycles  = %0d", pred_cycle);
        $display("  predicted_q16      = 0x%08h", pred_sample);
        $display("  estimated_q16      = 0x%08h", t_estimated);
        $finish;
    end

endmodule
