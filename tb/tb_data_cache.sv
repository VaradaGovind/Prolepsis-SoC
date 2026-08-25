`timescale 1ns / 1ps

module tb_data_cache;
    logic clk;
    logic rst;

    logic [31:0] cpu_addr;
    logic [31:0] cpu_wdata;
    logic [3:0]  cpu_wstrb;
    logic        cpu_we;
    logic        cpu_req;
    logic [31:0] cpu_rdata;
    logic        cpu_ready;

    logic [31:0] mem_addr;
    logic [31:0] mem_wdata;
    logic [3:0]  mem_wstrb;
    logic        mem_we;
    logic        mem_req;
    logic  [31:0] mem_rdata;
    logic         mem_ready;

    logic [31:0] snoop_addr;
    logic        snoop_we;
    logic [2:0]  snoop_src;
    logic        flush_all;

    localparam MEM_WORDS = 1024;
    localparam [31:0] ADDR_A = 32'h0000_0020;
    localparam [31:0] ADDR_B = 32'h0000_0140;
    localparam integer IDX_A = (32'h0000_0020 >> 2);
    localparam integer IDX_B = (32'h0000_0140 >> 2);

    logic [31:0] mem_model [0:MEM_WORDS-1];

    logic        pending;
    logic        pending_we;
    logic [31:0] pending_addr;
    logic [31:0] pending_wdata;
    logic [3:0]  pending_wstrb;

    integer i;

    function [31:0] apply_wstrb32;
        input [31:0] cur;
        input [31:0] wdata;
        input [3:0]  wstrb;
        begin
            apply_wstrb32 = cur;
            if (wstrb[0]) apply_wstrb32[7:0]   = wdata[7:0];
            if (wstrb[1]) apply_wstrb32[15:8]  = wdata[15:8];
            if (wstrb[2]) apply_wstrb32[23:16] = wdata[23:16];
            if (wstrb[3]) apply_wstrb32[31:24] = wdata[31:24];
        end
    endfunction

    Data_Cache #(
        .CACHE_LINES(16),
        .LINE_WORDS(4),
        .INDEX_WIDTH(4),
        .CORE_ID(0)
    ) dut (
        .clk(clk),
        .rst(rst),

        .cpu_addr(cpu_addr),
        .cpu_wdata(cpu_wdata),
        .cpu_wstrb(cpu_wstrb),
        .cpu_we(cpu_we),
        .cpu_req(cpu_req),
        .cpu_rdata(cpu_rdata),
        .cpu_ready(cpu_ready),

        .mem_addr(mem_addr),
        .mem_wdata(mem_wdata),
        .mem_wstrb(mem_wstrb),
        .mem_we(mem_we),
        .mem_req(mem_req),
        .mem_rdata(mem_rdata),
        .mem_ready(mem_ready),

        .snoop_addr(snoop_addr),
        .snoop_we(snoop_we),
        .snoop_src(snoop_src),

        .flush_all(flush_all)
    );

    always #5 clk = ~clk;

    // Simple one-cycle-latency memory model.
    always_ff @(posedge clk) begin
        mem_ready <= 1'b0;

        if (pending) begin
            if (pending_we)
                mem_model[pending_addr[11:2]] <= apply_wstrb32(mem_model[pending_addr[11:2]], pending_wdata, pending_wstrb);

            mem_rdata <= pending_we ?
                         apply_wstrb32(mem_model[pending_addr[11:2]], pending_wdata, pending_wstrb) :
                         mem_model[pending_addr[11:2]];
            mem_ready <= 1'b1;
            pending   <= 1'b0;
        end

        if (!pending && mem_req) begin
            pending      <= 1'b1;
            pending_we   <= mem_we;
            pending_addr <= mem_addr;
            pending_wdata<= mem_wdata;
            pending_wstrb<= mem_wstrb;
        end
    end

    task wait_cpu_ready;
        integer timeout;
        begin
            timeout = 0;
            while (!cpu_ready && timeout < 200) begin
                timeout = timeout + 1;
                @(posedge clk);
            end

            if (!cpu_ready) begin
                $display("FAIL: timed out waiting for cpu_ready");
                $finish;
            end
        end
    endtask

    task do_load;
        input  [31:0] addr;
        output [31:0] data;
        begin
            cpu_addr  <= addr;
            cpu_wdata <= 32'b0;
            cpu_wstrb <= 4'b0000;
            cpu_we    <= 1'b0;
            cpu_req   <= 1'b1;

            wait_cpu_ready();
            data = cpu_rdata;

            @(posedge clk);
            cpu_req <= 1'b0;
        end
    endtask

    task do_store;
        input [31:0] addr;
        input [31:0] data;
        input [3:0]  wstrb;
        begin
            cpu_addr  <= addr;
            cpu_wdata <= data;
            cpu_wstrb <= wstrb;
            cpu_we    <= 1'b1;
            cpu_req   <= 1'b1;

            wait_cpu_ready();

            @(posedge clk);
            cpu_req   <= 1'b0;
            cpu_we    <= 1'b0;
            cpu_wstrb <= 4'b0000;
        end
    endtask

    logic [31:0] rd;
    logic [31:0] expected;

    initial begin
        clk       = 1'b0;
        rst       = 1'b1;
        cpu_addr  = 32'b0;
        cpu_wdata = 32'b0;
        cpu_wstrb = 4'b0;
        cpu_we    = 1'b0;
        cpu_req   = 1'b0;
        mem_rdata = 32'b0;
        mem_ready = 1'b0;
        snoop_addr= 32'b0;
        snoop_we  = 1'b0;
        snoop_src = 3'b0;
        flush_all = 1'b0;
        pending   = 1'b0;
        pending_we= 1'b0;
        pending_addr = 32'b0;
        pending_wdata= 32'b0;
        pending_wstrb= 4'b0;

        for (i = 0; i < MEM_WORDS; i = i + 1)
            mem_model[i] = 32'h1000_0000 + i;

        repeat (3) @(posedge clk);
        rst <= 1'b0;

        // 1) Load miss then hit
        do_load(ADDR_A, rd);
        expected = mem_model[IDX_A];
        if (rd !== expected) begin
            $display("FAIL: load miss expected %h got %h", expected, rd);
            $finish;
        end

        do_load(ADDR_A, rd);
        if (rd !== expected) begin
            $display("FAIL: load hit expected %h got %h", expected, rd);
            $finish;
        end

        // 2) Store hit: update cache line and memory (write-through)
        do_store(ADDR_A, 32'hDEAD_BEEF, 4'b1111);

        do_load(ADDR_A, rd);
        if (rd !== 32'hDEAD_BEEF) begin
            $display("FAIL: store hit readback expected DEAD_BEEF got %h", rd);
            $finish;
        end

        if (mem_model[IDX_A] !== 32'hDEAD_BEEF) begin
            $display("FAIL: write-through memory not updated, got %h", mem_model[IDX_A]);
            $finish;
        end

        // 3) Coherence invalidate on snoop from another core
        mem_model[IDX_A] = 32'hCAFE_BABE;
        snoop_addr <= ADDR_A;
        snoop_src  <= 3'd2;
        snoop_we   <= 1'b1;
        @(posedge clk);
        snoop_we   <= 1'b0;

        do_load(ADDR_A, rd);
        if (rd !== 32'hCAFE_BABE) begin
            $display("FAIL: snoop invalidate failed, expected CAFE_BABE got %h", rd);
            $finish;
        end

        // 4) Store miss with write-allocate and byte strobe merge
        mem_model[IDX_B] = 32'h1111_1111;
        do_store(ADDR_B, 32'h0000_AA00, 4'b0010);

        do_load(ADDR_B, rd);
        if (rd !== 32'h1111_AA11) begin
            $display("FAIL: write-allocate byte merge expected 1111_AA11 got %h", rd);
            $finish;
        end

        if (mem_model[IDX_B] !== 32'h1111_AA11) begin
            $display("FAIL: write-through byte merge in memory expected 1111_AA11 got %h", mem_model[IDX_B]);
            $finish;
        end

        // 5) Flush invalidates cached copy
        flush_all <= 1'b1;
        @(posedge clk);
        flush_all <= 1'b0;
        mem_model[IDX_A] = 32'hFEED_1234;

        do_load(ADDR_A, rd);
        if (rd !== 32'hFEED_1234) begin
            $display("FAIL: flush invalidation expected FEED_1234 got %h", rd);
            $finish;
        end

        $display("PASS: Data cache write-through, allocate, and coherence invalidate verified");
        $finish;
    end

endmodule
