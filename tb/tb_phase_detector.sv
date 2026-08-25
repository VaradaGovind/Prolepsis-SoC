`timescale 1ns / 1ps

module tb_phase_detector;
    logic clk;
    logic rst_n;
    
    logic [31:0] minstret_delta;
    logic [31:0] mcycle_delta;
    logic [31:0] icache_miss;
    logic [31:0] icache_hit;
    
    logic [1:0] phase;
    logic       phase_changed;
    integer timeout;

    Phase_Detector dut (
        .clk(clk),
        .rst_n(rst_n),
        .minstret_delta(minstret_delta),
        .mcycle_delta(mcycle_delta),
        .icache_miss(icache_miss),
        .icache_hit(icache_hit),
        .phase(phase),
        .phase_changed(phase_changed)
    );

    always #5 clk = ~clk;

    initial begin
        clk = 0;
        rst_n = 0;
        
        minstret_delta = 0;
        mcycle_delta = 1024;
        icache_miss = 0;
        icache_hit = 0;
        
        #20 rst_n = 1;
        
        // 1. Simulate IDLE phase (IPC ~ 0)
        minstret_delta = 10;
        icache_hit = 100;
        icache_miss = 1;
        
        // Provide enough samples to pass hysteresis (8 samples)
        repeat (10) @(posedge clk);
        
        if (phase != 2'd3) begin
            $display("FAIL: Expected IDLE phase (3), got %0d", phase);
            $finish;
        end
        
        // 2. Simulate COMPUTE BOUND phase (very high IPC, very low miss)
        minstret_delta = 1000;
        icache_hit = 4096;
        icache_miss = 1;

        timeout = 0;
        while ((phase != 2'd0) && (timeout < 80)) begin
            timeout = timeout + 1;
            @(posedge clk);
        end

        if (phase != 2'd0) begin
            $display("FAIL: Expected COMPUTE_BOUND phase (0), got %0d", phase);
            $finish;
        end
        
        // 3. Simulate MEMORY BOUND phase (IPC < 0.125, High miss)
        minstret_delta = 32;
        icache_hit = 64;
        icache_miss = 512;

        timeout = 0;
        while ((phase != 2'd1) && (timeout < 100)) begin
            timeout = timeout + 1;
            @(posedge clk);
        end

        if (phase != 2'd1) begin
            $display("FAIL: Expected MEMORY_BOUND phase (1), got %0d", phase);
            $finish;
        end

        // 4. Branch-heavy proxy workload: moderate IPC and moderate miss rate
        // should map to BALANCED in the current 4-state detector.
        minstret_delta = 32'd320;
        icache_hit = 32'd192;
        icache_miss = 32'd64;

        timeout = 0;
        while ((phase != 2'd2) && (timeout < 100)) begin
            timeout = timeout + 1;
            @(posedge clk);
        end

        if (phase != 2'd2) begin
            $display("FAIL: Expected BALANCED phase (2) for branch-heavy proxy, got %0d", phase);
            $finish;
        end
        
        $display("PASS: Phase Detector transitions verified");
        $finish;
    end

endmodule
