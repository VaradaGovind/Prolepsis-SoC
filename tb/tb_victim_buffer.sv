`timescale 1ns / 1ps

module tb_victim_buffer;
    logic clk;
    logic rst;

    logic         evict_valid;
    logic [19:0]  evict_tag;
    logic [7:0]   evict_index;
    logic [127:0] evict_data;

    logic         lookup_valid;
    logic [19:0]  lookup_tag;
    logic [7:0]   lookup_index;
    logic        lookup_hit;
    logic [127:0] lookup_data;

    logic         swap_valid;
    logic [19:0]  swap_tag;
    logic [7:0]   swap_index;
    logic [127:0] swap_data;

    logic [19:0]  snoop_tag;
    logic [7:0]   snoop_index;
    logic         snoop_valid;

    logic flush_all;

    Victim_Buffer dut (
        .clk(clk),
        .rst(rst),
        .evict_valid(evict_valid),
        .evict_tag(evict_tag),
        .evict_index(evict_index),
        .evict_data(evict_data),
        .lookup_valid(lookup_valid),
        .lookup_tag(lookup_tag),
        .lookup_index(lookup_index),
        .lookup_hit(lookup_hit),
        .lookup_data(lookup_data),
        .swap_valid(swap_valid),
        .swap_tag(swap_tag),
        .swap_index(swap_index),
        .swap_data(swap_data),
        .snoop_tag(snoop_tag),
        .snoop_index(snoop_index),
        .snoop_valid(snoop_valid),
        .flush_all(flush_all)
    );

    always #5 clk = ~clk;

    task do_lookup;
        input [19:0] tag;
        input [7:0] idx;
        begin
            lookup_valid = 1'b1;
            lookup_tag = tag;
            lookup_index = idx;
            #1;
        end
    endtask

    initial begin
        clk = 1'b0;
        rst = 1'b1;
        evict_valid = 1'b0;
        evict_tag = 20'b0;
        evict_index = 8'b0;
        evict_data = 128'b0;
        lookup_valid = 1'b0;
        lookup_tag = 20'b0;
        lookup_index = 8'b0;
        swap_valid = 1'b0;
        swap_tag = 20'b0;
        swap_index = 8'b0;
        swap_data = 128'b0;
        snoop_tag = 20'b0;
        snoop_index = 8'b0;
        snoop_valid = 1'b0;
        flush_all = 1'b0;

        repeat (2) @(posedge clk);
        rst <= 1'b0;

        // Fill both victim entries.
        evict_valid <= 1'b1;
        evict_tag <= 20'h00111;
        evict_index <= 8'h10;
        evict_data <= 128'hAAAA_BBBB_CCCC_DDDD_1111_2222_3333_4444;
        @(posedge clk);

        evict_tag <= 20'h00222;
        evict_index <= 8'h20;
        evict_data <= 128'hDEAD_BEEF_0000_1111_2222_3333_4444_5555;
        @(posedge clk);
        evict_valid <= 1'b0;

        // Both lookups should hit.
        do_lookup(20'h00111, 8'h10);
        if (!lookup_hit) begin
            $display("FAIL: expected lookup hit for first entry");
            $finish;
        end
        if (lookup_data !== 128'hAAAA_BBBB_CCCC_DDDD_1111_2222_3333_4444) begin
            $display("FAIL: first entry data mismatch");
            $finish;
        end

        do_lookup(20'h00222, 8'h20);
        if (!lookup_hit) begin
            $display("FAIL: expected lookup hit for second entry");
            $finish;
        end

        // Third eviction should replace the oldest slot (FIFO pointer wrap).
        lookup_valid <= 1'b0;
        evict_valid <= 1'b1;
        evict_tag <= 20'h00333;
        evict_index <= 8'h30;
        evict_data <= 128'h0123_4567_89AB_CDEF_1111_1111_2222_2222;
        @(posedge clk);
        evict_valid <= 1'b0;

        do_lookup(20'h00111, 8'h10);
        if (lookup_hit) begin
            $display("FAIL: first entry should have been replaced");
            $finish;
        end

        do_lookup(20'h00333, 8'h30);
        if (!lookup_hit) begin
            $display("FAIL: replacement entry missing");
            $finish;
        end

        // Snoop invalidate second entry.
        lookup_valid <= 1'b0;
        snoop_valid <= 1'b1;
        snoop_tag <= 20'h00222;
        snoop_index <= 8'h20;
        @(posedge clk);
        snoop_valid <= 1'b0;

        do_lookup(20'h00222, 8'h20);
        if (lookup_hit) begin
            $display("FAIL: snoop invalidate did not clear entry");
            $finish;
        end

        // Flush all should clear remaining entries.
        flush_all <= 1'b1;
        @(posedge clk);
        flush_all <= 1'b0;

        do_lookup(20'h00333, 8'h30);
        if (lookup_hit) begin
            $display("FAIL: flush_all did not clear victim buffer");
            $finish;
        end

        $display("PASS: Victim buffer eviction/lookup/invalidate behavior verified");
        $finish;
    end
endmodule
