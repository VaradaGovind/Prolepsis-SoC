`timescale 1ns / 1ps

module tb_csr_unit;
    logic clk;
    logic rst;

    logic  [11:0] csr_addr;
    logic  [31:0] csr_wdata;
    logic  [1:0]  csr_op;
    logic         csr_we;
    logic [31:0] csr_rdata;

    logic         trap_enter;
    logic  [31:0] trap_cause;
    logic  [31:0] trap_val;
    logic  [31:0] trap_pc;
    logic         mret;
    logic [31:0] mtvec_out;
    logic [31:0] mepc_out;
    logic        mstatus_mie;
    logic        interrupt_pending;

    logic         timer_irq;
    logic         ext_irq;
    logic         thermal_irq;
    logic  [31:0] thermal_level;
    logic         perf_overflow_irq;
    logic  [31:0] thermal_reading;

    logic         instr_retired;
    logic         migration_event;
    logic         mig_csr_we;
    logic  [3:0]  mig_csr_addr;
    logic  [31:0] mig_csr_wdata;
    logic [31:0] mig_csr_rdata;

    localparam ADDR_MSTATUS  = 12'h300;
    localparam ADDR_MIE      = 12'h304;
    localparam ADDR_MTVEC    = 12'h305;
    localparam ADDR_MSCRATCH = 12'h340;
    localparam ADDR_MEPC     = 12'h341;
    localparam ADDR_MCAUSE   = 12'h342;

    csr_unit #(.HART_ID(3)) dut (
        .clk(clk),
        .rst(rst),
        .csr_addr(csr_addr),
        .csr_wdata(csr_wdata),
        .csr_op(csr_op),
        .csr_we(csr_we),
        .csr_rdata(csr_rdata),
        .trap_enter(trap_enter),
        .trap_cause(trap_cause),
        .trap_val(trap_val),
        .trap_pc(trap_pc),
        .mret(mret),
        .mtvec_out(mtvec_out),
        .mepc_out(mepc_out),
        .mstatus_mie(mstatus_mie),
        .interrupt_pending(interrupt_pending),
        .timer_irq(timer_irq),
        .ext_irq(ext_irq),
        .thermal_irq(thermal_irq),
        .thermal_level(thermal_level),
        .perf_overflow_irq(perf_overflow_irq),
        .thermal_reading(thermal_reading),
        .instr_retired(instr_retired),
        .migration_event(migration_event),
        .mig_csr_we(mig_csr_we),
        .mig_csr_addr(mig_csr_addr),
        .mig_csr_wdata(mig_csr_wdata),
        .mig_csr_rdata(mig_csr_rdata)
    );

    always #5 clk = ~clk;

    task csr_write;
        input [11:0] addr;
        input [31:0] data;
        input [1:0] op;
        begin
            @(posedge clk);
            csr_addr  <= addr;
            csr_wdata <= data;
            csr_op    <= op;
            csr_we    <= 1'b1;
            @(posedge clk);
            csr_we    <= 1'b0;
            csr_op    <= 2'b00;
            csr_wdata <= 32'b0;
        end
    endtask

    task expect_read;
        input [11:0] addr;
        input [31:0] expected;
        input [255:0] msg;
        begin
            csr_addr = addr;
            #1;
            if (csr_rdata !== expected) begin
                $display("FAIL: %0s expected=%h got=%h", msg, expected, csr_rdata);
                $finish;
            end
        end
    endtask

    initial begin
        clk = 1'b0;
        rst = 1'b1;
        csr_addr = 12'b0;
        csr_wdata = 32'b0;
        csr_op = 2'b00;
        csr_we = 1'b0;

        trap_enter = 1'b0;
        trap_cause = 32'b0;
        trap_val = 32'b0;
        trap_pc = 32'b0;
        mret = 1'b0;

        timer_irq = 1'b0;
        ext_irq = 1'b0;
        thermal_irq = 1'b0;
        thermal_level = 32'b0;
        perf_overflow_irq = 1'b0;
        thermal_reading = 32'h0019_0000;

        instr_retired = 1'b0;
        migration_event = 1'b0;
        mig_csr_we = 1'b0;
        mig_csr_addr = 4'b0;
        mig_csr_wdata = 32'b0;

        repeat (3) @(posedge clk);
        rst <= 1'b0;

        // CSR read/write verification
        csr_write(ADDR_MTVEC, 32'h0000_0100, 2'b01);
        expect_read(ADDR_MTVEC, 32'h0000_0100, "mtvec write/read");

        csr_write(ADDR_MSCRATCH, 32'h1234_5678, 2'b01);
        expect_read(ADDR_MSCRATCH, 32'h1234_5678, "mscratch write/read");

        // Enable global and timer/external interrupt bits.
        csr_write(ADDR_MSTATUS, 32'h0000_0008, 2'b01);
        #1;
        if (!mstatus_mie) begin
            $display("FAIL: mstatus.MIE should be set");
            $finish;
        end

        csr_write(ADDR_MIE, 32'h0000_0880, 2'b01); // MTIE + MEIE
        expect_read(ADDR_MIE, 32'h0000_0880, "mie write/read");

        // Timer interrupt fires.
        timer_irq <= 1'b1;
        @(posedge clk);
        #1;
        if (!interrupt_pending) begin
            $display("FAIL: timer interrupt should be pending");
            $finish;
        end

        // Enter timer trap and ensure nested interrupts are blocked by MIE clear.
        trap_pc <= 32'h0000_0200;
        trap_cause <= 32'h8000_0007;
        trap_enter <= 1'b1;
        @(posedge clk);
        trap_enter <= 1'b0;
        #1;

        if (mepc_out !== 32'h0000_0200) begin
            $display("FAIL: timer trap did not capture mepc correctly");
            $finish;
        end
        expect_read(ADDR_MCAUSE, 32'h8000_0007, "timer trap mcause");

        if (mstatus_mie) begin
            $display("FAIL: MIE must clear on trap entry (nested prevention)");
            $finish;
        end

        // While in handler (MIE=0), additional IRQ should not be pending.
        timer_irq <= 1'b0;
        ext_irq <= 1'b1;
        @(posedge clk);
        #1;
        if (interrupt_pending) begin
            $display("FAIL: interrupt should be masked while in handler");
            $finish;
        end

        // Return from trap and confirm MIE restoration.
        mret <= 1'b1;
        @(posedge clk);
        mret <= 1'b0;
        #1;
        if (!mstatus_mie) begin
            $display("FAIL: MIE should restore on MRET");
            $finish;
        end

        // External interrupt handling.
        @(posedge clk);
        #1;
        if (!interrupt_pending) begin
            $display("FAIL: external interrupt should be pending after MRET");
            $finish;
        end

        trap_pc <= 32'h0000_0204;
        trap_cause <= 32'h8000_000B;
        trap_enter <= 1'b1;
        @(posedge clk);
        trap_enter <= 1'b0;
        #1;
        expect_read(ADDR_MCAUSE, 32'h8000_000B, "external trap mcause");

        // ECALL + MRET flow.
        ext_irq <= 1'b0;
        mret <= 1'b1;
        @(posedge clk);
        mret <= 1'b0;
        #1;

        trap_pc <= 32'h0000_0208;
        trap_cause <= 32'd11;
        trap_enter <= 1'b1;
        @(posedge clk);
        trap_enter <= 1'b0;
        #1;

        if (mepc_out !== 32'h0000_0208) begin
            $display("FAIL: ecall trap did not capture mepc correctly");
            $finish;
        end
        expect_read(ADDR_MCAUSE, 32'd11, "ecall trap mcause");

        mret <= 1'b1;
        @(posedge clk);
        mret <= 1'b0;
        #1;
        if (!mstatus_mie) begin
            $display("FAIL: MIE should restore after ECALL/MRET sequence");
            $finish;
        end

        // MEPC remains readable and aligned.
        expect_read(ADDR_MEPC, 32'h0000_0208, "mepc readback");

        $display("PASS: CSR read/write + ECALL/MRET + timer/external interrupt behavior verified");
        $finish;
    end

endmodule
