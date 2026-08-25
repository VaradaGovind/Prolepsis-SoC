`timescale 1ns / 1ps

module tb_pipelined_alu;
    logic clk;
    logic rst;
    logic start;
    logic [31:0] a;
    logic [31:0] b;
    logic [4:0] alu_op;

    logic [31:0] result;
    logic zero;
    logic done;

    localparam ALU_MUL  = 5'b01010;
    localparam ALU_DIV  = 5'b01110;
    localparam ALU_DIVU = 5'b01111;
    localparam ALU_REM  = 5'b10000;
    localparam ALU_REMU = 5'b10001;

    pipelined_alu dut (
        .clk(clk),
        .rst(rst),
        .start(start),
        .a(a),
        .b(b),
        .alu_op(alu_op),
        .result(result),
        .zero(zero),
        .done(done)
    );

    always #5 clk = ~clk;

    task run_check;
        input [31:0] in_a;
        input [31:0] in_b;
        input [4:0] in_op;
        input [31:0] expected;
        input [31:0] max_wait;
        input [255:0] label;
        integer cycles;
        begin
            a = in_a;
            b = in_b;
            alu_op = in_op;

            @(posedge clk);
            start <= 1'b1;
            @(posedge clk);
            start <= 1'b0;

            cycles = 0;
            while (done && (cycles < max_wait)) begin
                cycles = cycles + 1;
                @(posedge clk);
            end

            if (done) begin
                $display("FAIL: %0s did not start (done never deasserted)", label);
                $finish;
            end

            cycles = 0;
            while (!done && (cycles < max_wait)) begin
                cycles = cycles + 1;
                @(posedge clk);
            end

            if (!done) begin
                $display("FAIL: %0s timed out", label);
                $finish;
            end

            #1;
            if (result !== expected) begin
                $display("FAIL: %0s expected=%h got=%h", label, expected, result);
                $finish;
            end
        end
    endtask

    initial begin
        clk = 1'b0;
        rst = 1'b1;
        start = 1'b0;
        a = 32'b0;
        b = 32'b0;
        alu_op = 5'b0;

        repeat (3) @(posedge clk);
        rst <= 1'b0;

        // M-extension edge cases
        run_check(32'h1234_5678, 32'h0000_0000, ALU_DIV, 32'hFFFF_FFFF, 64, "DIV by zero");
        run_check(32'h8000_0000, 32'hFFFF_FFFF, ALU_DIV, 32'h8000_0000, 64, "DIV signed overflow");
        run_check(32'h8000_0000, 32'hFFFF_FFFF, ALU_REM, 32'h0000_0000, 64, "REM signed overflow");

        // Basic multiply sanity to ensure M datapath still works.
        run_check(32'h0000_0011, 32'h0000_0003, ALU_MUL, 32'h0000_0033, 16, "MUL 17*3");

        $display("PASS: Pipelined ALU M-extension edge/normal behavior verified");
        $finish;
    end

endmodule
