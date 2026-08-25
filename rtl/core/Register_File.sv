`timescale 1ns / 1ps
//============================================================================
// OrionRV - Register File (2-read, 1-write, with migration port)
// x0 is hardwired to zero per RISC-V spec
//============================================================================

module regfile (
    input  logic        clk,

    // Normal write port (from WB stage)
    input  logic        we,
    input  logic [4:0]  waddr,
    input  logic [31:0] wdata,

    // Read ports (for ID stage)
    input  logic [4:0]  raddr1,
    input  logic [4:0]  raddr2,
    output logic [31:0] rdata1,
    output logic [31:0] rdata2,

    // Migration port (for task migration save/restore)
    input  logic        mig_we,
    input  logic [4:0]  mig_addr,
    input  logic [31:0] mig_wdata,
    output logic [31:0] mig_rdata
);

    // -------------------------------------------------------------------
    // 32 x 32-bit register file
    // -------------------------------------------------------------------
    logic [31:0] regs [0:31];

    // x0 always reads as zero
    assign rdata1    = (raddr1 == 5'b0) ? 32'b0 : regs[raddr1];
    assign rdata2    = (raddr2 == 5'b0) ? 32'b0 : regs[raddr2];
    assign mig_rdata = (mig_addr == 5'b0) ? 32'b0 : regs[mig_addr];

    // -------------------------------------------------------------------
    // Write logic: migration port takes priority during migration
    // Normal write happens during pipeline WB stage
    // -------------------------------------------------------------------
    always_ff @(posedge clk) begin
        if (mig_we && mig_addr != 5'b0) begin
            regs[mig_addr] <= mig_wdata;
        end
        else if (we && waddr != 5'b0) begin
            regs[waddr] <= wdata;
        end
    end

    // -------------------------------------------------------------------
    // Initialize x0 (synthesis will ignore, simulation needs it)
    // -------------------------------------------------------------------
    initial begin
        regs[0] = 32'b0;
    end

endmodule
