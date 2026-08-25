`timescale 1ns / 1ps
//============================================================================
// OrionRV - CSR Unit (Machine-Mode)
// Implements essential CSR registers for:
//   - Hart identification (mhartid)
//   - Trap handling (mstatus, mtvec, mepc, mcause, mtval, mie, mip)
//   - Performance counters (mcycle, minstret)
//   - Thermal sensor readout (custom CSRs for thermal management)
//============================================================================

module csr_unit #(
    parameter HART_ID = 0
)(
    input  logic        clk,
    input  logic        rst,

    // CSR access port (from pipeline EX/MEM stage)
    input  logic [11:0] csr_addr,
    input  logic [31:0] csr_wdata,
    input  logic [1:0]  csr_op,       // 00=none, 01=RW, 10=RS, 11=RC
    input  logic        csr_we,       // Combined write enable
    output logic  [31:0] csr_rdata,

    // Trap interface
    input  logic        trap_enter,    // ECALL / illegal / interrupt
    input  logic [31:0] trap_cause,
    input  logic [31:0] trap_val,      // mtval value
    input  logic [31:0] trap_pc,       // PC of trapping instruction
    input  logic        mret,          // MRET instruction
    output logic [31:0] mtvec_out,     // Trap vector address
    output logic [31:0] mepc_out,      // Return address for MRET
    output logic        mstatus_mie,   // Global interrupt enable
    output logic        interrupt_pending, // Pending IRQ

    // Timer interrupt (from platform timer)
    input  logic        timer_irq,
    input  logic        ext_irq,
    input  logic        thermal_irq,
    input  logic [31:0] thermal_level,
    input  logic        perf_overflow_irq,

    // Thermal sensor input (custom CSR)
    input  logic [31:0] thermal_reading,

    // Performance (active cycles / instructions retired)
    input  logic        instr_retired,

    // Migration hooks
    input  logic        migration_event,
    input  logic        mig_csr_we,
    input  logic [3:0]  mig_csr_addr,
    input  logic [31:0] mig_csr_wdata,
    output logic  [31:0] mig_csr_rdata
);

    // ===================================================================
    // CSR addresses (RISC-V Machine-Mode)
    // ===================================================================
    localparam ADDR_MSTATUS    = 12'h300;
    localparam ADDR_MISA       = 12'h301;
    localparam ADDR_MIE        = 12'h304;
    localparam ADDR_MTVEC      = 12'h305;
    localparam ADDR_MSCRATCH   = 12'h340;
    localparam ADDR_MEPC       = 12'h341;
    localparam ADDR_MCAUSE     = 12'h342;
    localparam ADDR_MTVAL      = 12'h343;
    localparam ADDR_MIP        = 12'h344;

    localparam ADDR_MCYCLE     = 12'hB00;
    localparam ADDR_MCYCLEH    = 12'hB80;
    localparam ADDR_MINSTRET   = 12'hB02;
    localparam ADDR_MINSTRETH  = 12'hB82;

    localparam ADDR_MHARTID    = 12'hF14;

    // Custom CSRs for thermal management (0x7C0-0x7FF = custom M-mode R/W)
    localparam ADDR_MTHERM     = 12'h7C0;  // Current thermal reading
    localparam ADDR_MTHERM_CFG = 12'h7C1;  // Thermal config register
    localparam ADDR_MHPC_MIG   = 12'h7C9;  // Migration participation counter

    // Compact migration CSR index map
    localparam MIGCSR_MSTATUS   = 4'd0;
    localparam MIGCSR_MIE       = 4'd1;
    localparam MIGCSR_MTVEC     = 4'd2;
    localparam MIGCSR_MSCRATCH  = 4'd3;
    localparam MIGCSR_MEPC      = 4'd4;
    localparam MIGCSR_MCAUSE    = 4'd5;
    localparam MIGCSR_MTVAL     = 4'd6;
    localparam MIGCSR_MTHERMCFG = 4'd7;
    localparam MIGCSR_MCYCLE_LO = 4'd8;
    localparam MIGCSR_MCYCLE_HI = 4'd9;
    localparam MIGCSR_MINST_LO  = 4'd10;
    localparam MIGCSR_MINST_HI  = 4'd11;
    localparam MIGCSR_MHPC_MIG  = 4'd12;

    // ===================================================================
    // CSR registers
    // ===================================================================
    logic [31:0] mstatus;     // Only MIE (bit 3) and MPIE (bit 7) used
    logic [31:0] mie;         // Interrupt enable: MTIE=bit7, MEIE=bit11
    logic [31:0] mtvec;
    logic [31:0] mscratch;
    logic [31:0] mepc;
    logic [31:0] mcause;
    logic [31:0] mtval;
    logic [31:0] mip;

    logic [63:0] mcycle;
    logic [63:0] minstret;

    logic [31:0] mtherm_cfg;
    logic [31:0] mhpc_migration;

    // ===================================================================
    // Fixed values
    // ===================================================================
    // MISA: RV32IMA (I=bit8, M=bit12, A=bit0)
    logic [31:0] misa;
    assign misa = {2'b01, 4'b0, 26'b00000000000001000100000001};
    //                   MXL=32  ----   Z..A  => I + M + A set

    logic [31:0] mhartid;
    assign mhartid = HART_ID;

    // ===================================================================
    // Outputs
    // ===================================================================
    assign mtvec_out    = mtvec;
    assign mepc_out     = mepc;
    assign mstatus_mie  = mstatus[3];
    assign interrupt_pending = ((mie & mip) != 32'b0) & mstatus[3];

    // MIP external inputs are now merged into the main sequential block below
    // to avoid multi-driver synthesis issues on the `mip` register.

    // ===================================================================
    // Read logic (combinational)
    // ===================================================================
    always_comb begin
        case (csr_addr)
            ADDR_MSTATUS:   csr_rdata = mstatus;
            ADDR_MISA:      csr_rdata = misa;
            ADDR_MIE:       csr_rdata = mie;
            ADDR_MTVEC:     csr_rdata = mtvec;
            ADDR_MSCRATCH:  csr_rdata = mscratch;
            ADDR_MEPC:      csr_rdata = mepc;
            ADDR_MCAUSE:    csr_rdata = mcause;
            ADDR_MTVAL:     csr_rdata = mtval;
            ADDR_MIP:       csr_rdata = mip;
            ADDR_MCYCLE:    csr_rdata = mcycle[31:0];
            ADDR_MCYCLEH:   csr_rdata = mcycle[63:32];
            ADDR_MINSTRET:  csr_rdata = minstret[31:0];
            ADDR_MINSTRETH: csr_rdata = minstret[63:32];
            ADDR_MHARTID:   csr_rdata = mhartid;
            ADDR_MTHERM:    csr_rdata = thermal_reading;
            ADDR_MTHERM_CFG:csr_rdata = mtherm_cfg;
            ADDR_MHPC_MIG:  csr_rdata = mhpc_migration;
            default:        csr_rdata = 32'b0;
        endcase
    end

    always_comb begin
        case (mig_csr_addr)
            MIGCSR_MSTATUS:   mig_csr_rdata = mstatus;
            MIGCSR_MIE:       mig_csr_rdata = mie;
            MIGCSR_MTVEC:     mig_csr_rdata = mtvec;
            MIGCSR_MSCRATCH:  mig_csr_rdata = mscratch;
            MIGCSR_MEPC:      mig_csr_rdata = mepc;
            MIGCSR_MCAUSE:    mig_csr_rdata = mcause;
            MIGCSR_MTVAL:     mig_csr_rdata = mtval;
            MIGCSR_MTHERMCFG: mig_csr_rdata = mtherm_cfg;
            MIGCSR_MCYCLE_LO: mig_csr_rdata = mcycle[31:0];
            MIGCSR_MCYCLE_HI: mig_csr_rdata = mcycle[63:32];
            MIGCSR_MINST_LO:  mig_csr_rdata = minstret[31:0];
            MIGCSR_MINST_HI:  mig_csr_rdata = minstret[63:32];
            MIGCSR_MHPC_MIG:  mig_csr_rdata = mhpc_migration;
            default:          mig_csr_rdata = 32'b0;
        endcase
    end

    // ===================================================================
    // Write logic (sequential)
    // ===================================================================
    function [31:0] csr_apply_op;
        input [31:0] csr_cur;
        input [31:0] csr_w;
        input [1:0]  op;
        begin
            case (op)
                2'b01:   csr_apply_op = csr_w;              // CSRRW
                2'b10:   csr_apply_op = csr_cur | csr_w;    // CSRRS
                2'b11:   csr_apply_op = csr_cur & (~csr_w); // CSRRC
                default: csr_apply_op = csr_cur;
            endcase
        end
    endfunction

    always_ff @(posedge clk) begin
        if (rst) begin
            mstatus   <= 32'h0000_1800;  // MPP=11 (machine mode)
            mie       <= 32'b0;
            mtvec     <= 32'h0000_0000;
            mscratch  <= 32'b0;
            mepc      <= 32'b0;
            mcause    <= 32'b0;
            mtval     <= 32'b0;
            mip       <= 32'b0;
            mcycle    <= 64'b0;
            minstret  <= 64'b0;
            mtherm_cfg<= 32'b0;
            mhpc_migration <= 32'b0;
        end
        else begin
            // Performance counters always tick
            mcycle <= mcycle + 1;
            if (instr_retired)
                minstret <= minstret + 1;
            if (migration_event)
                mhpc_migration <= mhpc_migration + 1;

            // MIP: latch external interrupt inputs every cycle
            // (merged here to avoid multi-driver on mip register)
            mip[7]  <= timer_irq;   // MTIP
            mip[11] <= ext_irq;     // MEIP
            mip[16] <= thermal_irq; // Custom thermal IRQ
            mip[17] <= thermal_level[31]; // Thermal sign/overflow telemetry bit
            mip[18] <= thermal_level[30]; // Thermal telemetry extension bit
            mip[19] <= thermal_level[29]; // Thermal telemetry extension bit
            mip[20] <= thermal_level[28]; // Thermal telemetry extension bit
            mip[21] <= thermal_level[27]; // Thermal telemetry extension bit
            mip[22] <= thermal_level[26]; // Thermal telemetry extension bit
            mip[23] <= thermal_level[25]; // Thermal telemetry extension bit
            mip[24] <= thermal_level[24]; // Thermal telemetry extension bit
            mip[25] <= thermal_level[23]; // Thermal telemetry extension bit

            if (mig_csr_we) begin
                case (mig_csr_addr)
                    MIGCSR_MSTATUS:   mstatus <= (mig_csr_wdata & 32'h0000_0088) | (mstatus & ~32'h0000_0088);
                    MIGCSR_MIE:       mie <= mig_csr_wdata;
                    MIGCSR_MTVEC:     mtvec <= mig_csr_wdata;
                    MIGCSR_MSCRATCH:  mscratch <= mig_csr_wdata;
                    MIGCSR_MEPC:      mepc <= {mig_csr_wdata[31:2], 2'b00};
                    MIGCSR_MCAUSE:    mcause <= mig_csr_wdata;
                    MIGCSR_MTVAL:     mtval <= mig_csr_wdata;
                    MIGCSR_MTHERMCFG: mtherm_cfg <= mig_csr_wdata;
                    MIGCSR_MCYCLE_LO: mcycle[31:0] <= mig_csr_wdata;
                    MIGCSR_MCYCLE_HI: mcycle[63:32] <= mig_csr_wdata;
                    MIGCSR_MINST_LO:  minstret[31:0] <= mig_csr_wdata;
                    MIGCSR_MINST_HI:  minstret[63:32] <= mig_csr_wdata;
                    MIGCSR_MHPC_MIG:  mhpc_migration <= mig_csr_wdata;
                    default: ;
                endcase
            end
            // Trap entry: save state and disable interrupts
            else if (trap_enter) begin
                mepc           <= trap_pc;
                mcause         <= trap_cause;
                mtval          <= trap_val;
                mstatus[7]     <= mstatus[3];   // MPIE = MIE
                mstatus[3]     <= 1'b0;          // MIE = 0
            end
            // MRET: restore interrupt state
            else if (mret) begin
                mstatus[3]     <= mstatus[7];   // MIE = MPIE
                mstatus[7]     <= 1'b1;          // MPIE = 1
            end
            // Normal CSR write
            else if (csr_we && (csr_op != 2'b00)) begin
                case (csr_addr)
                    // mstatus: only MIE(3) and MPIE(7) are software-writable;
                    // preserve MPP(12:11) and other fixed fields
                    ADDR_MSTATUS:   mstatus  <= (csr_apply_op(mstatus, csr_wdata, csr_op) & 32'h0000_0088)
                                              | (mstatus    & ~32'h0000_0088);
                    ADDR_MIE:       mie      <= csr_apply_op(mie, csr_wdata, csr_op);
                    ADDR_MTVEC:     mtvec    <= csr_apply_op(mtvec, csr_wdata, csr_op);
                    ADDR_MSCRATCH:  mscratch <= csr_apply_op(mscratch, csr_wdata, csr_op);
                    ADDR_MEPC:      mepc     <= (csr_apply_op(mepc, csr_wdata, csr_op) & 32'hFFFF_FFFC); // 4-byte aligned
                    ADDR_MCAUSE:    mcause   <= csr_apply_op(mcause, csr_wdata, csr_op);
                    ADDR_MTVAL:     mtval    <= csr_apply_op(mtval, csr_wdata, csr_op);
                    ADDR_MCYCLE:    mcycle[31:0] <= csr_apply_op(mcycle[31:0], csr_wdata, csr_op);
                    ADDR_MCYCLEH:   mcycle[63:32] <= csr_apply_op(mcycle[63:32], csr_wdata, csr_op);
                    ADDR_MINSTRET:  minstret[31:0] <= csr_apply_op(minstret[31:0], csr_wdata, csr_op);
                    ADDR_MINSTRETH: minstret[63:32] <= csr_apply_op(minstret[63:32], csr_wdata, csr_op);
                    ADDR_MTHERM_CFG:mtherm_cfg <= csr_apply_op(mtherm_cfg, csr_wdata, csr_op);
                    ADDR_MHPC_MIG:  mhpc_migration <= csr_apply_op(mhpc_migration, csr_wdata, csr_op);
                    default: ;
                endcase
            end
        end
    end

endmodule
