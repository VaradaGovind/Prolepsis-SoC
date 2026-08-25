`timescale 1ns / 1ps

module tb_power_gate_controller;
    logic clk;
    logic rst_n;
    logic enable;
    logic [9:0] phase_flat;
    logic [9:0] thermal_state_flat;
    logic [4:0] base_core_enable;
    logic migration_req;
    logic migration_busy;
    logic [2:0] migration_dst_core;

    logic [4:0] power_gate_mask;
    logic [159:0] pwr_gated_cycles_flat;
    logic [31:0] total_pwr_gated_cycles;
    logic [31:0] power_gate_events;
    logic [4:0] pg_save_req;
    logic [4:0] pg_restore_req;
    logic [4:0] pg_ack;

    integer wake_cycles;
    logic save_seen;
    logic restore_seen;

    Power_Gate_Controller #(
        .NUM_CORES(5),
        .IDLE_THRESHOLD(12'd4),
        .WAKE_LATENCY(8'd3)
    ) dut (
        .clk(clk),
        .rst_n(rst_n),
        .enable(enable),
        .phase_flat(phase_flat),
        .thermal_state_flat(thermal_state_flat),
        .base_core_enable(base_core_enable),
        .migration_req(migration_req),
        .migration_busy(migration_busy),
        .migration_dst_core(migration_dst_core),
        .power_gate_mask(power_gate_mask),
        .pwr_gated_cycles_flat(pwr_gated_cycles_flat),
        .total_pwr_gated_cycles(total_pwr_gated_cycles),
        .power_gate_events(power_gate_events),
        .pg_save_req(pg_save_req),
        .pg_restore_req(pg_restore_req),
        .pg_ack(pg_ack)
    );

    always #5 clk = ~clk;
    assign pg_ack = pg_save_req | pg_restore_req;

    always_ff @(posedge clk) begin
        if (pg_save_req[3])
            save_seen <= 1'b1;
        if (pg_restore_req[3])
            restore_seen <= 1'b1;
    end

    initial begin
        clk = 1'b0;
        rst_n = 1'b0;
        enable = 1'b1;
        base_core_enable = 5'b11111;
        migration_req = 1'b0;
        migration_busy = 1'b0;
        migration_dst_core = 3'd0;
        wake_cycles = 0;
        save_seen = 1'b0;
        restore_seen = 1'b0;

        // phase order in flat bus: core4..core0
        phase_flat = {2'd2, 2'd2, 2'd2, 2'd2, 2'd2};
        thermal_state_flat = {2'd0, 2'd0, 2'd0, 2'd0, 2'd0};

        repeat (3) @(posedge clk);
        rst_n <= 1'b1;

        // Keep core3 idle long enough to power gate
        phase_flat = {2'd2, 2'd3, 2'd2, 2'd2, 2'd2};
        repeat (10) @(posedge clk);

        if (!power_gate_mask[3]) begin
            $display("FAIL: core3 should be power gated");
            $finish;
        end

        if (!save_seen) begin
            $display("FAIL: save handshake was not observed before power gate entry");
            $finish;
        end

        migration_req <= 1'b1;
        migration_dst_core <= 3'd3;
        @(posedge clk);
        migration_req <= 1'b0;
        wake_cycles = 0;
        while (power_gate_mask[3] && (wake_cycles < 20)) begin
            wake_cycles = wake_cycles + 1;
            @(posedge clk);
        end

        if (power_gate_mask[3]) begin
            $display("FAIL: core3 should wake on migration request");
            $finish;
        end

        if (wake_cycles < 3) begin
            $display("FAIL: wake latency too short, measured %0d cycles", wake_cycles);
            $finish;
        end

        if (wake_cycles > 12) begin
            $display("FAIL: wake latency too long, measured %0d cycles", wake_cycles);
            $finish;
        end

        if (!restore_seen) begin
            $display("FAIL: restore handshake was not observed during wake");
            $finish;
        end

        if ((total_pwr_gated_cycles == 32'd0) || (power_gate_events == 32'd0)) begin
            $display("FAIL: expected non-zero power gate metrics");
            $finish;
        end

        $display("PASS: Power gate controller idles and wakes cores correctly");
        $finish;
    end

endmodule
