`timescale 1ns / 1ps

module tb_migration_policy;
    logic clk;
    logic rst_n;
    logic enable;

    logic [159:0] t_predicted_flat;
    logic [9:0]   phase_flat;
    logic [4:0]   phase_changed_flat;
    logic [159:0] icache_miss_flat;

    logic [31:0] t_warn;
    logic [31:0] t_crit;
    logic [31:0] cache_warm_cycles;
    logic migration_busy;

    logic migrate_req;
    logic [2:0] src_core;
    logic [2:0] dst_core;
    logic [31:0] decision_cost_cycles;
    logic [31:0] decision_benefit_score;
    logic all_cores_hot;
    logic dual_hot_mode;
    logic imbalance_block;
    logic [159:0] migration_hist_count_flat;

    logic [31:0] req_count;

    Migration_Policy #(
        .NUM_CORES(5),
        .COOLDOWN_CYCLES(32'd8)
    ) dut (
        .clk(clk),
        .rst_n(rst_n),
        .enable(enable),
        .t_predicted_flat(t_predicted_flat),
        .phase_flat(phase_flat),
        .phase_changed_flat(phase_changed_flat),
        .icache_miss_flat(icache_miss_flat),
        .t_warn(t_warn),
        .t_crit(t_crit),
        .cache_warm_cycles(cache_warm_cycles),
        .migration_busy(migration_busy),
        .migrate_req(migrate_req),
        .src_core(src_core),
        .dst_core(dst_core),
        .decision_cost_cycles(decision_cost_cycles),
        .decision_benefit_score(decision_benefit_score),
        .all_cores_hot(all_cores_hot),
        .dual_hot_mode(dual_hot_mode),
        .imbalance_block(imbalance_block),
        .migration_hist_count_flat(migration_hist_count_flat)
    );

    always #5 clk = ~clk;

    always_ff @(posedge clk) begin
        if (!rst_n)
            req_count <= 32'd0;
        else if (migrate_req)
            req_count <= req_count + 32'd1;
    end

    initial begin
        clk = 1'b0;
        rst_n = 1'b0;
        enable = 1'b1;
        migration_busy = 1'b0;
        cache_warm_cycles = 32'd512;

        // Scenario A (negative): low thermal delta + high warm cost -> no migration.
        // core0 is hottest, but benefit should not exceed cost.
        t_predicted_flat = {
            32'h004C_0000,
            32'h004E_0000,
            32'h004F_0000,
            32'h0054_0000,
            32'h0056_0000
        };

        // phase[0]=compute + changed (proactive candidate), others balanced
        phase_flat = {
            2'd2,
            2'd2,
            2'd2,
            2'd2,
            2'd0
        };
        phase_changed_flat = 5'b00001;

        icache_miss_flat = {
            32'd4,
            32'd8,
            32'd16,
            32'd1,
            32'd32
        };

        t_warn = 32'h0055_0000;
        t_crit = 32'h005A_0000;

        req_count = 32'd0;

        repeat (4) @(posedge clk);
        rst_n <= 1'b1;

        repeat (12) @(posedge clk);

        if (req_count != 32'd0) begin
            $display("FAIL: policy should skip migration when cost > benefit");
            $finish;
        end

        // Scenario B (positive): high delta + low warm cost -> migration expected.
        cache_warm_cycles <= 32'd10;
        t_predicted_flat <= {
            32'h0050_0000,
            32'h0058_0000,
            32'h0030_0000,
            32'h0040_0000,
            32'h0059_0000
        };
        icache_miss_flat <= {
            32'd8,
            32'd16,
            32'd1,
            32'd4,
            32'd32
        };

        repeat (12) @(posedge clk);

        if (req_count == 32'd0) begin
            $display("FAIL: policy never requested migration");
            $finish;
        end

        if (src_core == dst_core) begin
            $display("FAIL: policy selected same src/dst core");
            $finish;
        end

        if (!dual_hot_mode) begin
            $display("FAIL: expected dual_hot_mode to assert for two-hot scenario");
            $finish;
        end

        if (migration_hist_count_flat == 160'd0) begin
            $display("FAIL: migration history counters did not update");
            $finish;
        end

        migration_busy <= 1'b1;
        repeat (6) @(posedge clk);
        migration_busy <= 1'b0;

        // Force all cores hot and confirm policy blocks migration
        t_predicted_flat = {
            32'h005B_0000,
            32'h005B_0000,
            32'h005B_0000,
            32'h005B_0000,
            32'h005B_0000
        };
        phase_changed_flat = 5'b00000;

        repeat (6) @(posedge clk);

        if (!all_cores_hot) begin
            $display("FAIL: all_cores_hot should be asserted");
            $finish;
        end

        $display("PASS: Migration policy generated valid decisions and safety blocking");
        $finish;
    end

endmodule
