`timescale 1ns / 1ps

module tb_stride_prefetcher;
    logic clk;
    logic rst;

    logic        miss_valid;
    logic [31:0] miss_addr;
    logic [31:0] miss_pc;

    logic       pf_req;
    logic [31:0] pf_addr;
    logic        pf_ack;

    Stride_Prefetcher dut (
        .clk(clk),
        .rst(rst),
        .miss_valid(miss_valid),
        .miss_addr(miss_addr),
        .miss_pc(miss_pc),
        .pf_req(pf_req),
        .pf_addr(pf_addr),
        .pf_ack(pf_ack)
    );

    always #5 clk = ~clk;

    task send_miss;
        input [31:0] addr;
        input [31:0] pc;
        begin
            @(negedge clk);
            miss_addr = addr;
            miss_pc = pc;
            miss_valid = 1'b1;
            @(posedge clk);
            @(negedge clk);
            miss_valid = 1'b0;
            @(posedge clk);
        end
    endtask

    initial begin
        clk = 1'b0;
        rst = 1'b1;
        miss_valid = 1'b0;
        miss_addr = 32'b0;
        miss_pc = 32'b0;
        pf_ack = 1'b0;

        repeat (2) @(posedge clk);
        rst <= 1'b0;

        // Train stride=+4 on the same miss PC. Prefetch should appear after
        // confidence saturates and one additional confirming miss arrives.
        send_miss(32'h0000_1000, 32'h0000_0040);
        send_miss(32'h0000_1004, 32'h0000_0040);
        send_miss(32'h0000_1008, 32'h0000_0040);
        send_miss(32'h0000_100C, 32'h0000_0040);
        send_miss(32'h0000_1010, 32'h0000_0040);

        #1;
        if (!pf_req) begin
            $display("FAIL: expected prefetch request after stride confidence saturates");
            $finish;
        end
        if (pf_addr != 32'h0000_1014) begin
            $display("FAIL: prefetch address mismatch, got %h", pf_addr);
            $finish;
        end

        // Ack clears request.
        @(negedge clk);
        pf_ack = 1'b1;
        @(posedge clk);
        @(negedge clk);
        pf_ack = 1'b0;
        #1;
        if (pf_req) begin
            $display("FAIL: prefetch request should clear after ack");
            $finish;
        end

        $display("PASS: Stride prefetcher confidence and request behavior verified");
        $finish;
    end
endmodule
