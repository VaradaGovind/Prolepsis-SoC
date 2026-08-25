`timescale 1ns / 1ps

module tb_eval_framework;
    logic clk;
    logic rst_n;

    logic [1:0] mode_write_data;
    logic mode_write_en;
    logic run_done_pulse;

    logic [4:0] core_retired;
    logic [4:0] core_clk_en;
    logic [4:0] power_gate_mask;
    logic [159:0] t_sensor_flat;
    logic [159:0] power_proxy_flat;
    logic [31:0] t_crit;
    logic migration_done;
    logic [31:0] migration_cycles_inc;

    logic [1:0] active_mode;
    logic metrics_frozen;
    logic [31:0] total_cycles;
    logic [31:0] total_retired;
    logic [31:0] peak_temperature;
    logic [31:0] avg_temperature;
    logic [31:0] throttle_events;
    logic [31:0] throttle_cycles;
    logic [31:0] migration_count;
    logic [31:0] migration_cycles;
    logic [31:0] power_gate_cycles;
    logic [31:0] thermal_violations;
    logic [31:0] energy_estimate;

    Eval_Framework dut (
        .clk(clk),
        .rst_n(rst_n),
        .mode_write_data(mode_write_data),
        .mode_write_en(mode_write_en),
        .run_done_pulse(run_done_pulse),
        .core_retired(core_retired),
        .core_clk_en(core_clk_en),
        .power_gate_mask(power_gate_mask),
        .t_sensor_flat(t_sensor_flat),
        .power_proxy_flat(power_proxy_flat),
        .t_crit(t_crit),
        .migration_done(migration_done),
        .migration_cycles_inc(migration_cycles_inc),
        .active_mode(active_mode),
        .metrics_frozen(metrics_frozen),
        .total_cycles(total_cycles),
        .total_retired(total_retired),
        .peak_temperature(peak_temperature),
        .avg_temperature(avg_temperature),
        .throttle_events(throttle_events),
        .throttle_cycles(throttle_cycles),
        .migration_count(migration_count),
        .migration_cycles(migration_cycles),
        .power_gate_cycles(power_gate_cycles),
        .thermal_violations(thermal_violations),
        .energy_estimate(energy_estimate)
    );

    always #5 clk = ~clk;

    initial begin
        clk = 1'b0;
        rst_n = 1'b0;

        mode_write_data = 2'd3;
        mode_write_en = 1'b0;
        run_done_pulse = 1'b0;

        core_retired = 5'b0;
        core_clk_en = 5'b11111;
        power_gate_mask = 5'b0;
        t_sensor_flat = {32'h0030_0000,32'h0031_0000,32'h0032_0000,32'h0033_0000,32'h0034_0000};
        power_proxy_flat = {32'd16,32'd32,32'd48,32'd64,32'd80};
        t_crit = 32'h005A_0000;
        migration_done = 1'b0;
        migration_cycles_inc = 32'd44;

        repeat (3) @(posedge clk);
        rst_n <= 1'b1;

        repeat (12) begin
            @(posedge clk);
            core_retired <= 5'b10101;
            core_clk_en <= 5'b11110;
            power_gate_mask <= 5'b00010;
        end

        @(posedge clk);
        migration_done <= 1'b1;
        @(posedge clk);
        migration_done <= 1'b0;

        repeat (4) @(posedge clk);

        if ((total_cycles == 32'd0) || (total_retired == 32'd0) || (energy_estimate == 32'd0)) begin
            $display("FAIL: expected non-zero eval metrics");
            $finish;
        end

        if ((migration_count != 32'd1) || (migration_cycles != 32'd44)) begin
            $display("FAIL: migration metrics mismatch count=%0d cycles=%0d", migration_count, migration_cycles);
            $finish;
        end

        // Freeze metrics and ensure counters stop
        @(posedge clk);
        run_done_pulse <= 1'b1;
        @(posedge clk);
        run_done_pulse <= 1'b0;

        @(posedge clk);
        if (!metrics_frozen) begin
            $display("FAIL: metrics should be frozen after run_done_pulse");
            $finish;
        end

        // Mode switch should clear metrics
        @(posedge clk);
        mode_write_data <= 2'd1;
        mode_write_en <= 1'b1;
        @(posedge clk);
        mode_write_en <= 1'b0;

        @(posedge clk);
        if ((active_mode != 2'd1) || (total_cycles != 32'd0)) begin
            $display("FAIL: mode switch did not reset metrics correctly");
            $finish;
        end

        $display("PASS: Evaluation framework metrics and mode controls verified");
        $finish;
    end

endmodule
