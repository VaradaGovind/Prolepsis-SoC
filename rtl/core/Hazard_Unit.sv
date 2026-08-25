`timescale 1ns / 1ps
//============================================================================
// OrionRV - Hazard Detection & Forwarding Unit (Phase 7.5 upgrade)
//
// Handles data hazards (forwarding + stalls) and control hazards (flushes)
// for a 5-stage in-order pipeline: IF | ID | EX | MEM | WB
//
// Phase 7.5: Flushes on branch misprediction instead of every taken branch.
// Correctly-predicted branches cause zero pipeline bubbles.
//============================================================================

module hazard_unit (
    // -------------------------------------------------------------------
    // Pipeline register addresses
    // -------------------------------------------------------------------
    input  logic [4:0]  id_raw_rs1,
    input  logic [4:0]  id_raw_rs2,

    input  logic [4:0]  ex_rd,
    input  logic        ex_reg_write,
    input  logic        ex_mem_read,     // Load in EX -> load-use hazard

    input  logic [4:0]  mem_rd,
    input  logic        mem_reg_write,

    input  logic [4:0]  wb_rd,
    input  logic        wb_reg_write,

    // -------------------------------------------------------------------
    // Branch/jump control (from EX stage)
    // -------------------------------------------------------------------
    input  logic        is_branch_id,    // Branch currently in ID stage
    input  logic        branch_taken,    // Branch resolved as taken in EX
    input  logic        is_jal,          // JAL detected in ID
    input  logic        is_jalr,         // JALR resolved in EX

    // -------------------------------------------------------------------
    // Branch prediction misprediction (Phase 7.5)
    // -------------------------------------------------------------------
    input  logic        bp_mispredict,   // Branch predictor was wrong

    // -------------------------------------------------------------------
    // Atomic multi-cycle stall
    // -------------------------------------------------------------------
    input  logic        atomic_busy,

    // -------------------------------------------------------------------
    // ALU multi-cycle stall
    // -------------------------------------------------------------------
    input  logic        alu_busy,

    // -------------------------------------------------------------------
    // I-Cache miss stall
    // -------------------------------------------------------------------
    input  logic        icache_miss,

    // -------------------------------------------------------------------
    // Data memory stall
    // -------------------------------------------------------------------
    input  logic        dmem_stall,

    // -------------------------------------------------------------------
    // Pipeline control outputs
    // -------------------------------------------------------------------
    output logic        stall_if,
    output logic        stall_id,
    output logic        stall_ex,
    output logic        stall_mem,
    output logic        flush_if,
    output logic        flush_id,
    output logic        flush_ex
);

    // -------------------------------------------------------------------
    // Load-use hazard detection
    // -------------------------------------------------------------------
    logic load_use_hazard;
    assign load_use_hazard = ex_mem_read && (
        ((ex_rd == id_raw_rs1) && (id_raw_rs1 != 5'b0)) ||
        ((ex_rd == id_raw_rs2) && (id_raw_rs2 != 5'b0))
    );

    // Branch-dependency stall: hold branch in ID until source registers
    // become architecturally visible.
    logic branch_dep_ex;
    assign branch_dep_ex = ex_reg_write && (ex_rd != 5'b0) &&
                         ((ex_rd == id_raw_rs1) || (ex_rd == id_raw_rs2));
    logic branch_dep_mem;
    assign branch_dep_mem = mem_reg_write && (mem_rd != 5'b0) &&
                          ((mem_rd == id_raw_rs1) || (mem_rd == id_raw_rs2));
    logic branch_dep_wb;
    assign branch_dep_wb = wb_reg_write && (wb_rd != 5'b0) &&
                         ((wb_rd == id_raw_rs1) || (wb_rd == id_raw_rs2));
    logic branch_dep_hazard;
    assign branch_dep_hazard = is_branch_id &&
                             (branch_dep_ex || branch_dep_mem || branch_dep_wb);

    // -------------------------------------------------------------------
    // Control hazard (Phase 7.5): flush only on MISPREDICTION
    // Correctly-predicted branches cause no flush.
    // JALR without prediction still causes a flush.
    // -------------------------------------------------------------------
    logic control_hazard;
    assign control_hazard = bp_mispredict | (is_jalr & ~bp_mispredict);

    // -------------------------------------------------------------------
    // Stall logic
    // -------------------------------------------------------------------
    logic icache_stall;
    assign icache_stall = icache_miss & ~is_jal;
    logic any_stall;
    assign any_stall = load_use_hazard | branch_dep_hazard |
                     atomic_busy | alu_busy | icache_stall | dmem_stall;

    assign stall_if  = any_stall;
    assign stall_id  = any_stall;
    assign stall_ex  = atomic_busy | alu_busy | dmem_stall;
    assign stall_mem = dmem_stall;

    // -------------------------------------------------------------------
    // Flush logic
    // -------------------------------------------------------------------
    assign flush_if  = 1'b0;  // IF is re-steered via PC, not flushed
    assign flush_id  = control_hazard;
    assign flush_ex  = load_use_hazard | branch_dep_hazard | control_hazard;

endmodule

