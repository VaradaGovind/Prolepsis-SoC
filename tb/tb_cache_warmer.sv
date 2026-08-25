`timescale 1ns / 1ps

module tb_cache_warmer;
    logic clk;
    logic rst_n;
    logic start;
    logic [2:0] src_core;
    logic [2:0] dst_core;
    logic [31:0] src_pc;
    logic [31:0] warm_entries_cfg;
    logic [7:0] inspect_index;
    logic inspect_valid;
    logic [19:0] inspect_tag;

    logic busy;
    logic done;
    logic [31:0] cache_warm_cycles;
    logic [31:0] prefetch_count;
    logic [31:0] prefetch_addr;
    logic prefetch_valid;

    Cache_Warmer dut (
        .clk(clk),
        .rst_n(rst_n),
        .start(start),
        .src_core(src_core),
        .dst_core(dst_core),
        .src_pc(src_pc),
        .warm_entries_cfg(warm_entries_cfg),
        .inspect_index(inspect_index),
        .inspect_valid(inspect_valid),
        .inspect_tag(inspect_tag),
        .busy(busy),
        .done(done),
        .cache_warm_cycles(cache_warm_cycles),
        .prefetch_count(prefetch_count),
        .prefetch_addr(prefetch_addr),
        .prefetch_valid(prefetch_valid)
    );

    always #5 clk = ~clk;

    initial begin
        clk = 1'b0;
        rst_n = 1'b0;
        start = 1'b0;
        src_core = 3'd0;
        dst_core = 3'd1;
        src_pc = 32'h2000_0100;
        warm_entries_cfg = 32'd8;
        inspect_valid = 1'b1;
        inspect_tag = 20'hABCDE;

        repeat (3) @(posedge clk);
        rst_n <= 1'b1;

        @(posedge clk);
        start <= 1'b1;
        @(posedge clk);
        start <= 1'b0;

        wait (done == 1'b1);
        @(posedge clk);

        if (cache_warm_cycles != 32'd8) begin
            $display("FAIL: expected cache_warm_cycles=8 got %0d", cache_warm_cycles);
            $finish;
        end

        // src==dst should skip warming
        src_core <= 3'd2;
        dst_core <= 3'd2;
        @(posedge clk);
        start <= 1'b1;
        @(posedge clk);
        start <= 1'b0;
        @(posedge clk);

        if (!done) begin
            $display("FAIL: same-core warm should complete immediately");
            $finish;
        end

        $display("PASS: Cache warmer timing behavior verified");
        $finish;
    end

endmodule
