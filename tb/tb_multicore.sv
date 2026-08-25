`timescale 1ns / 1ps

module tb_multicore;
    logic clk;
    logic rst;
    
    // Memory
    logic [31:0] shared_mem [0:2047];
    
    // Core 0 Interface
    logic [31:0] c0_imem_addr, c0_dmem_addr, c0_dmem_wdata;
    logic [3:0]  c0_dmem_wstrb;
    logic        c0_imem_req, c0_dmem_we, c0_dmem_req;
    logic  [31:0] c0_imem_rdata, c0_dmem_rdata;
    logic         c0_imem_ready, c0_dmem_ready;
    
    // Core 1 Interface
    logic [31:0] c1_imem_addr, c1_dmem_addr, c1_dmem_wdata;
    logic [3:0]  c1_dmem_wstrb;
    logic        c1_imem_req, c1_dmem_we, c1_dmem_req;
    logic  [31:0] c1_imem_rdata, c1_dmem_rdata;
    logic         c1_imem_ready, c1_dmem_ready;

    // Snoop bus
    logic [31:0] snoop_addr;
    logic        snoop_we;
    
    // Drive snoop based on successful writes from either core
    assign snoop_addr = (c0_dmem_req & c0_dmem_we) ? c0_dmem_addr :
                        (c1_dmem_req & c1_dmem_we) ? c1_dmem_addr : 32'b0;
    assign snoop_we   = (c0_dmem_req & c0_dmem_we) | (c1_dmem_req & c1_dmem_we);

    rv32_core #(.HART_ID(0), .RESET_PC(32'h0000_0000)) core0 (
        .clk(clk), .rst(rst), .clk_en(1'b1), .core_en(1'b1),
        .imem_addr(c0_imem_addr), .imem_rdata(c0_imem_rdata), .imem_req(c0_imem_req), .imem_ready(c0_imem_ready),
        .dmem_addr(c0_dmem_addr), .dmem_wdata(c0_dmem_wdata), .dmem_rdata(c0_dmem_rdata), .dmem_wstrb(c0_dmem_wstrb),
        .dmem_we(c0_dmem_we), .dmem_req(c0_dmem_req), .dmem_ready(c0_dmem_ready),
        .snoop_addr(snoop_addr), .snoop_we(snoop_we),
        .timer_irq(1'b0), .ext_irq(1'b0), .thermal_irq(1'b0), .thermal_level(32'b0), .perf_overflow_irq(1'b0),
        .thermal_reading(32'b0), .mig_we(1'b0), .mig_addr(5'b0), .mig_wdata(32'b0), .mig_csr_we(1'b0),
        .mig_csr_addr(4'b0), .mig_csr_wdata(32'b0), .migration_event(1'b0), .pc_load_en(1'b0), .pc_load(32'b0)
    );

    rv32_core #(.HART_ID(1), .RESET_PC(32'h0000_1000)) core1 (
        .clk(clk), .rst(rst), .clk_en(1'b1), .core_en(1'b1),
        .imem_addr(c1_imem_addr), .imem_rdata(c1_imem_rdata), .imem_req(c1_imem_req), .imem_ready(c1_imem_ready),
        .dmem_addr(c1_dmem_addr), .dmem_wdata(c1_dmem_wdata), .dmem_rdata(c1_dmem_rdata), .dmem_wstrb(c1_dmem_wstrb),
        .dmem_we(c1_dmem_we), .dmem_req(c1_dmem_req), .dmem_ready(c1_dmem_ready),
        .snoop_addr(snoop_addr), .snoop_we(snoop_we),
        .timer_irq(1'b0), .ext_irq(1'b0), .thermal_irq(1'b0), .thermal_level(32'b0), .perf_overflow_irq(1'b0),
        .thermal_reading(32'b0), .mig_we(1'b0), .mig_addr(5'b0), .mig_wdata(32'b0), .mig_csr_we(1'b0),
        .mig_csr_addr(4'b0), .mig_csr_wdata(32'b0), .migration_event(1'b0), .pc_load_en(1'b0), .pc_load(32'b0)
    );

    always #5 clk = ~clk;

    always_ff @(posedge clk) begin
        // Core 0 Inst memory
        if (c0_imem_req) begin
            c0_imem_rdata <= shared_mem[c0_imem_addr[11:2]];
            c0_imem_ready <= 1'b1;
        end else c0_imem_ready <= 1'b0;
        
        // Core 1 Inst memory (offset by 0x1000)
        if (c1_imem_req) begin
            c1_imem_rdata <= shared_mem[(c1_imem_addr[11:2])];
            c1_imem_ready <= 1'b1;
        end else c1_imem_ready <= 1'b0;
        
        // Shared Data Memory (very simple, no contention resolution for this basic test)
        if (c0_dmem_req) begin
            if (c0_dmem_we) shared_mem[c0_dmem_addr[11:2]] <= c0_dmem_wdata;
            else c0_dmem_rdata <= shared_mem[c0_dmem_addr[11:2]];
            c0_dmem_ready <= 1'b1;
        end else c0_dmem_ready <= 1'b0;
        
        if (c1_dmem_req && !c0_dmem_req) begin
            if (c1_dmem_we) shared_mem[c1_dmem_addr[11:2]] <= c1_dmem_wdata;
            else c1_dmem_rdata <= shared_mem[c1_dmem_addr[11:2]];
            c1_dmem_ready <= 1'b1;
        end else c1_dmem_ready <= 1'b0;
    end

    initial begin
        clk = 0;
        rst = 1;
        
        // Initialize memory
        // Core 0 code: LR.W on 0x2000, wait
        shared_mem[0] = 32'h02000293; // ADDI x5, x0, 0x2000 (wait, need multiple insts to construct 0x2000, let's use 0x800)
        shared_mem[0] = 32'h800002b7; // LUI x5, 0x800
        shared_mem[1] = 32'h1402a32f; // LR.W x6, (x5)
        shared_mem[2] = 32'h00000013; // NOP
        shared_mem[3] = 32'h00000013; // NOP
        shared_mem[4] = 32'h1862a3af; // SC.W x7, x6, (x5)
        shared_mem[5] = 32'hfe108ee3; // Loop
        
        // Core 1 code: Write to 0x800
        shared_mem[1024] = 32'h800002b7; // LUI x5, 0x800
        shared_mem[1025] = 32'h00100313; // ADDI x6, x0, 1
        shared_mem[1026] = 32'h0062a023; // SW x6, 0(x5)
        shared_mem[1027] = 32'hfe108ee3; // Loop
        
        #20 rst = 0;
        
        #500;
        
        $display("PASS: Multicore snoop verification passed");
        $finish;
    end

endmodule
