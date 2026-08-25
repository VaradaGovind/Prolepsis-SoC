`timescale 1ns / 1ps

module tb_migration_controller;
    localparam NUM_CORES  = 5;
    localparam CORE_IDX_W = 3;

    logic clk;
    logic rst_n;

    logic start;
    logic clear_halt;
    logic cache_warm_done;
    logic [CORE_IDX_W-1:0] src_core;
    logic [CORE_IDX_W-1:0] dst_core;

    logic  [NUM_CORES*32-1:0] pc_save_flat;
    logic  [NUM_CORES*32-1:0] mig_rdata_flat;
    logic  [NUM_CORES*32-1:0] csr_rdata_flat;
    logic [NUM_CORES-1:0]    mig_we;
    logic [NUM_CORES*5-1:0]  mig_addr_flat;
    logic [NUM_CORES*32-1:0] mig_wdata_flat;
    logic [NUM_CORES-1:0]    csr_we;
    logic [NUM_CORES*4-1:0]  csr_addr_flat;
    logic [NUM_CORES*32-1:0] csr_wdata_flat;
    logic [NUM_CORES-1:0]    pc_load_en;
    logic [NUM_CORES*32-1:0] pc_load_flat;
    logic [NUM_CORES-1:0]    pause_mask;
    logic [NUM_CORES-1:0]    halt_mask;
    logic                    busy;
    logic                    done;
    logic                    error;
    logic [31:0]             last_migration_cycles;
    logic [31:0]             migration_count;
    logic                    cache_warm_start;
    logic [CORE_IDX_W-1:0]   active_src_core;
    logic [CORE_IDX_W-1:0]   active_dst_core;

    logic [31:0] model_regs [0:NUM_CORES-1][0:31];
    logic [31:0] model_pc   [0:NUM_CORES-1];
    logic [31:0] model_csr  [0:NUM_CORES-1][0:15];

    integer c;
    integer r;
    integer err_count;

    Migration_Controller #(
        .NUM_CORES  (NUM_CORES),
        .CORE_IDX_W (CORE_IDX_W)
    ) dut (
        .clk                  (clk),
        .rst_n                (rst_n),
        .start                (start),
        .clear_halt           (clear_halt),
        .src_core             (src_core),
        .dst_core             (dst_core),
        .cache_warm_done      (cache_warm_done),
        .cache_warm_start     (cache_warm_start),
        .pc_save_flat         (pc_save_flat),
        .mig_rdata_flat       (mig_rdata_flat),
        .csr_rdata_flat       (csr_rdata_flat),
        .mig_we               (mig_we),
        .mig_addr_flat        (mig_addr_flat),
        .mig_wdata_flat       (mig_wdata_flat),
        .csr_we               (csr_we),
        .csr_addr_flat        (csr_addr_flat),
        .csr_wdata_flat       (csr_wdata_flat),
        .pc_load_en           (pc_load_en),
        .pc_load_flat         (pc_load_flat),
        .pause_mask           (pause_mask),
        .halt_mask            (halt_mask),
        .busy                 (busy),
        .done                 (done),
        .error                (error),
        .last_migration_cycles(last_migration_cycles),
        .migration_count      (migration_count),
        .active_src_core      (active_src_core),
        .active_dst_core      (active_dst_core)
    );

    always #5 clk = ~clk;

    always_comb begin
        pc_save_flat  = {(NUM_CORES*32){1'b0}};
        mig_rdata_flat = {(NUM_CORES*32){1'b0}};
        csr_rdata_flat = {(NUM_CORES*32){1'b0}};

        for (c = 0; c < NUM_CORES; c = c + 1) begin
            pc_save_flat[(c*32) +: 32] = model_pc[c];
            mig_rdata_flat[(c*32) +: 32] = model_regs[c][mig_addr_flat[(c*5) +: 5]];
            csr_rdata_flat[(c*32) +: 32] = model_csr[c][csr_addr_flat[(c*4) +: 4]];
        end
    end

    always_ff @(posedge clk) begin
        for (c = 0; c < NUM_CORES; c = c + 1) begin
            if (mig_we[c]) begin
                model_regs[c][mig_addr_flat[(c*5) +: 5]] <= mig_wdata_flat[(c*32) +: 32];
            end

            if (csr_we[c]) begin
                model_csr[c][csr_addr_flat[(c*4) +: 4]] <= csr_wdata_flat[(c*32) +: 32];
            end

            if (pc_load_en[c]) begin
                model_pc[c] <= pc_load_flat[(c*32) +: 32];
            end
        end
    end

    initial begin
        clk = 1'b0;
        rst_n = 1'b0;
        start = 1'b0;
        clear_halt = 1'b0;
        cache_warm_done = 1'b1;
        src_core = 3'd0;
        dst_core = 3'd0;
        err_count = 0;

        for (c = 0; c < NUM_CORES; c = c + 1) begin
            model_pc[c] = 32'b0;
            for (r = 0; r < 32; r = r + 1) begin
                model_regs[c][r] = 32'b0;
            end
            for (r = 0; r < 16; r = r + 1) begin
                model_csr[c][r] = 32'b0;
            end
        end

        for (r = 1; r < 32; r = r + 1) begin
            model_regs[1][r] = 32'h1000_0000 + r;
        end
        for (r = 0; r < 13; r = r + 1) begin
            model_csr[1][r] = 32'h3000_0000 + r;
            model_csr[3][r] = 32'hDEAD_0000 + r;
        end
        model_pc[1] = 32'h2000_0100;
        model_pc[3] = 32'h0000_0040;

        repeat (4) @(posedge clk);
        rst_n <= 1'b1;

        @(posedge clk);
        src_core <= 3'd1;
        dst_core <= 3'd3;
        start <= 1'b1;

        @(posedge clk);
        start <= 1'b0;

        wait (done == 1'b1);
        @(posedge clk);

        for (r = 1; r < 32; r = r + 1) begin
            if (model_regs[3][r] !== model_regs[1][r]) begin
                $display("FAIL: register x%0d mismatch dst=%h src=%h", r, model_regs[3][r], model_regs[1][r]);
                err_count = err_count + 1;
            end
        end

        if (model_pc[3] !== model_pc[1]) begin
            $display("FAIL: destination PC mismatch dst=%h src=%h", model_pc[3], model_pc[1]);
            err_count = err_count + 1;
        end

        for (r = 0; r < 13; r = r + 1) begin
            if (model_csr[3][r] !== model_csr[1][r]) begin
                $display("FAIL: CSR slot %0d mismatch dst=%h src=%h", r, model_csr[3][r], model_csr[1][r]);
                err_count = err_count + 1;
            end
        end

        if (halt_mask[1] !== 1'b1) begin
            $display("FAIL: source core was not halted after migration");
            err_count = err_count + 1;
        end

        if (migration_count !== 32'd1) begin
            $display("FAIL: migration_count expected 1 got %0d", migration_count);
            err_count = err_count + 1;
        end

        if ((active_src_core !== 3'd1) || (active_dst_core !== 3'd3)) begin
            $display("FAIL: active route mismatch src=%0d dst=%0d", active_src_core, active_dst_core);
            err_count = err_count + 1;
        end

        clear_halt <= 1'b1;
        @(posedge clk);
        clear_halt <= 1'b0;
        @(posedge clk);

        if (halt_mask !== {NUM_CORES{1'b0}}) begin
            $display("FAIL: halt mask was not cleared");
            err_count = err_count + 1;
        end

        if (err_count == 0) begin
            $display("PASS: Migration controller transferred registers and PC correctly");
        end else begin
            $display("FAIL: %0d checks failed", err_count);
        end

        $finish;
    end

endmodule
