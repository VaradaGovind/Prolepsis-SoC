`timescale 1ns / 1ps

module tb_rv32_core;
    logic clk;
    logic rst;
    
    // Core inputs
    logic  clk_en;
    logic  core_en;
    logic  timer_irq;
    logic  ext_irq;
    logic  thermal_irq;
    logic  [31:0] thermal_level;
    logic  perf_overflow_irq;
    logic  [31:0] thermal_reading;
    logic  mig_we;
    logic  [4:0] mig_addr;
    logic  [31:0] mig_wdata;
    logic  mig_csr_we;
    logic  [3:0] mig_csr_addr;
    logic  [31:0] mig_csr_wdata;
    logic  migration_event;
    logic  pc_load_en;
    logic  [31:0] pc_load;
    
    // Mock memory
    logic  [31:0] mem [0:255];
    
    logic [31:0] imem_addr;
    logic  [31:0] imem_rdata;
    logic        imem_req;
    logic         imem_ready;
    
    logic [31:0] dmem_addr;
    logic [31:0] dmem_wdata;
    logic  [31:0] dmem_rdata;
    logic [3:0]  dmem_wstrb;
    logic        dmem_we;
    logic        dmem_req;
    logic         dmem_ready;
    
    logic [31:0] snoop_addr;
    assign snoop_addr = 32'b0;
    logic        snoop_we;
    assign snoop_we = 1'b0;
    
    logic [31:0] mig_rdata;
    logic [31:0] mig_csr_rdata;
    logic [31:0] pc_save;
    logic        active;
    logic        retired;

    rv32_core #(
        .HART_ID(0),
        .RESET_PC(32'h0000_0000)
    ) dut (
        .clk(clk),
        .rst(rst),
        .clk_en(clk_en),
        .core_en(core_en),
        .imem_addr(imem_addr),
        .imem_rdata(imem_rdata),
        .imem_req(imem_req),
        .imem_ready(imem_ready),
        .dmem_addr(dmem_addr),
        .dmem_wdata(dmem_wdata),
        .dmem_rdata(dmem_rdata),
        .dmem_wstrb(dmem_wstrb),
        .dmem_we(dmem_we),
        .dmem_req(dmem_req),
        .dmem_ready(dmem_ready),
        .snoop_addr(snoop_addr),
        .snoop_we(snoop_we),
        .timer_irq(timer_irq),
        .ext_irq(ext_irq),
        .thermal_irq(thermal_irq),
        .thermal_level(thermal_level),
        .perf_overflow_irq(perf_overflow_irq),
        .thermal_reading(thermal_reading),
        .mig_we(mig_we),
        .mig_addr(mig_addr),
        .mig_wdata(mig_wdata),
        .mig_rdata(mig_rdata),
        .mig_csr_we(mig_csr_we),
        .mig_csr_addr(mig_csr_addr),
        .mig_csr_wdata(mig_csr_wdata),
        .mig_csr_rdata(mig_csr_rdata),
        .migration_event(migration_event),
        .pc_save(pc_save),
        .pc_load_en(pc_load_en),
        .pc_load(pc_load),
        .active(active),
        .retired(retired)
    );

    always #5 clk = ~clk;

    always_ff @(posedge clk) begin
        // Mock instruction memory
        if (imem_req) begin
            imem_rdata <= mem[imem_addr[9:2]];
            imem_ready <= 1'b1;
        end else begin
            imem_ready <= 1'b0;
        end
        
        // Mock data memory
        if (dmem_req) begin
            if (dmem_we) begin
                if (dmem_wstrb[0]) mem[dmem_addr[9:2]][7:0]   <= dmem_wdata[7:0];
                if (dmem_wstrb[1]) mem[dmem_addr[9:2]][15:8]  <= dmem_wdata[15:8];
                if (dmem_wstrb[2]) mem[dmem_addr[9:2]][23:16] <= dmem_wdata[23:16];
                if (dmem_wstrb[3]) mem[dmem_addr[9:2]][31:24] <= dmem_wdata[31:24];
            end else begin
                dmem_rdata <= mem[dmem_addr[9:2]];
            end
            dmem_ready <= 1'b1;
        end else begin
            dmem_ready <= 1'b0;
        end
    end

    initial begin
        clk = 0;
        rst = 1;
        clk_en = 1;
        core_en = 1;
        timer_irq = 0;
        ext_irq = 0;
        thermal_irq = 0;
        thermal_level = 0;
        perf_overflow_irq = 0;
        thermal_reading = 0;
        mig_we = 0;
        mig_addr = 0;
        mig_wdata = 0;
        mig_csr_we = 0;
        mig_csr_addr = 0;
        mig_csr_wdata = 0;
        migration_event = 0;
        pc_load_en = 0;
        pc_load = 0;

        // Load basic instructions into mem
        // ADDI x1, x0, 5
        mem[0] = 32'h00500093;
        // ADDI x2, x0, 10
        mem[1] = 32'h00a00113;
        // ADD x3, x1, x2 (15)
        mem[2] = 32'h002081b3;
        // SW x3, 0x40(x0)
        mem[3] = 32'h04302023;
        // LW x4, 0x40(x0)
        mem[4] = 32'h04002203;
        // JAL x5, 8
        mem[5] = 32'h008002ef;
        mem[6] = 32'h00000013; // NOP
        // BEQ x1, x1, -4 (Infinite loop at 7)
        mem[7] = 32'hfe108ee3; 
        
        #20 rst = 0;

        #300;

        if (mem[16] != 32'd15) begin
            $display("FAIL: memory write-back failed, got %0d", mem[16]);
            $finish;
        end
        
        $display("PASS: RV32 Core basic execution verified");
        $finish;
    end

endmodule
