`timescale 1ns / 1ps
//============================================================================
// OrionRV - 5-Stage Pipelined RV32IMA Core (rv32_core)
//
// Pipeline stages: IF -> ID -> EX -> MEM -> WB
//
// Supported ISA:
//   RV32I  - Full base integer (LUI, AUIPC, JAL, JALR, branches,
//            loads LB/LH/LW/LBU/LHU, stores SB/SH/SW, ALU logic/imm)
//   RV32M  - MUL, MULH, MULHSU, MULHU, DIV, DIVU, REM, REMU
//   RV32A  - LR.W, SC.W (for multi-core synchronization)
//   CSRs   - CSRRW, CSRRS, CSRRC, CSRRWI, CSRRSI, CSRRCI
//   System - ECALL, MRET, FENCE (nop)
//
// Features:
//   - Full data forwarding (EX->EX, MEM->EX, WB->EX)
//   - Load-use hazard detection with 1-cycle stall
//   - Branch resolution in EX stage with flush
//   - Instruction cache interface
//   - Atomic reservation station (LR/SC with snoop invalidation)
//   - CSR unit with timer/external interrupts
//   - Clock gating + core enable for thermal management
//   - Migration port on register file for task migration
//   - PC save/restore interface for task migration
//   - (Phase 7.5) GShare branch predictor with BTB + RAS
//   - (Phase 7.5) Hardware loop buffer for tight loop power savings
//   - (Phase 7.5) RV32C compressed instruction decompressor
//============================================================================

module rv32_core #(
    parameter HART_ID   = 0,
    parameter RESET_PC  = 32'h0000_0000
)(
    input  logic        clk,
    input  logic        rst,

    // Thermal / power management
    input  logic        clk_en,          // Clock gate from throttle unit
    input  logic        core_en,         // Core enable from thermal manager

    // Instruction memory interface (to I-Cache)
    output logic [31:0] imem_addr,
    input  logic [31:0] imem_rdata,
    output logic        imem_req,
    input  logic        imem_ready,      // 1 = data valid, 0 = cache miss

    // Data memory interface (to bus/interconnect)
    output logic  [31:0] dmem_addr,
    output logic  [31:0] dmem_wdata,
    input  logic [31:0] dmem_rdata,
    output logic  [3:0]  dmem_wstrb,      // Byte write strobes
    output logic         dmem_we,
    output logic         dmem_req,
    input  logic        dmem_ready,      // 1 = data valid / write accepted

    // Snoop interface (for LR/SC coherence)
    input  logic [31:0] snoop_addr,
    input  logic        snoop_we,

    // Interrupt inputs
    input  logic        timer_irq,
    input  logic        ext_irq,
    input  logic        thermal_irq,
    input  logic [31:0] thermal_level,
    input  logic        perf_overflow_irq,

    // Thermal sensor input (routed to CSR)
    input  logic [31:0] thermal_reading,

    // Migration port (external controller can save/restore regs)
    input  logic        mig_we,
    input  logic [4:0]  mig_addr,
    input  logic [31:0] mig_wdata,
    output logic [31:0] mig_rdata,

    // Migration CSR checkpoint/restore
    input  logic        mig_csr_we,
    input  logic [3:0]  mig_csr_addr,
    input  logic [31:0] mig_csr_wdata,
    output logic [31:0] mig_csr_rdata,

    // Migration participation pulse (source or destination)
    input  logic        migration_event,

    // Migration PC checkpoint/restore
    output logic [31:0] pc_save,
    input  logic        pc_load_en,
    input  logic [31:0] pc_load,

    // Status
    output logic        active,          // Core is actively executing
    output logic        retired          // Instruction retired this cycle
);

    // ===================================================================
    // Pipeline gating (Registered for timing isolation)
    // ===================================================================
    logic pipe_en;
    always_ff @(posedge clk) begin
        if (rst)
            pipe_en <= 1'b0;
        else
            pipe_en <= clk_en & core_en;
    end

    // ===================================================================
    // Stall / flush signals from hazard unit
    // ===================================================================
    logic        stall_if, stall_id, stall_ex, stall_mem;
    logic        flush_if, flush_id, flush_ex;
    logic [1:0]  fwd_a, fwd_b;

    // Forward declarations for hazard inputs
    logic        hz_ex_mem_read;
    logic [4:0]  hz_ex_rd, hz_mem_rd, hz_wb_rd;
    logic        hz_ex_reg_write, hz_mem_reg_write, hz_wb_reg_write;
    logic                   hz_is_jal;
    (* keep = "true", max_fanout = 2 *) logic branch_taken_pc_mux;
    (* keep = "true", max_fanout = 2 *) logic branch_taken_pc_we;
    (* keep = "true", max_fanout = 2 *) logic branch_taken_if;
    (* keep = "true", max_fanout = 2 *) logic branch_taken_hz;
    (* keep = "true", max_fanout = 2 *) logic jalr_pc_mux;
    (* keep = "true", max_fanout = 2 *) logic jalr_pc_we;
    (* keep = "true", max_fanout = 2 *) logic jalr_if;
    (* keep = "true", max_fanout = 2 *) logic jalr_hz;
    (* keep = "true", max_fanout = 2 *) logic hz_is_jal_pc;
    (* keep = "true", max_fanout = 2 *) logic hz_is_jal_if;
    logic        hz_icache_miss, hz_dmem_stall, hz_atomic_busy, hz_alu_busy;
    
    // Explicit logic declarations to prevent undeclared variable warnings
    logic        wb_reg_write_en;
    logic [4:0]  wb_rd;
    logic [31:0] wb_wdata;
    logic [31:0] id_jal_target;

    // -------------------------------------------------------------------
    // Forward declarations of EX registers and branch_taken for IF stage
    // -------------------------------------------------------------------
    logic [31:0] idex_pc;
    logic [31:0] idex_rs1_data;
    logic [31:0] idex_rs2_data;
    logic [31:0] idex_imm;
    logic [4:0]  idex_rs1, idex_rd;
    logic [4:0]  idex_alu_op;
    logic        idex_alu_src;
    logic        idex_mem_read, idex_mem_write;
    logic [2:0]  idex_mem_funct3;
    logic        idex_reg_write;
    logic [1:0]  idex_wb_sel;
    logic        idex_is_branch;
    logic        idex_is_jal, idex_is_jalr;
    logic        idex_is_atomic;
    logic [4:0]  idex_atomic_funct5;
    logic        idex_is_csr;
    logic [1:0]  idex_csr_op;
    logic [11:0] idex_csr_addr;
    logic        idex_is_ecall, idex_is_mret;
    logic [2:0]  idex_funct3;
    logic        idex_valid;
    logic [31:0] idex_instr;
    (* max_fanout = 4 *) logic [1:0]  idex_fwd_a;
    (* max_fanout = 4 *) logic [1:0]  idex_fwd_b;
    logic        idex_bp_taken;
    logic [31:0] idex_bp_target;

    logic branch_taken_raw;


    // ===================================================================
    //  STAGE 1: INSTRUCTION FETCH (IF)
    // ===================================================================
    (* max_fanout = 32 *) logic [31:0] pc;
    logic [31:0] pc_next;
    logic [31:0] pc_commit_next;
    logic [31:0] pc_plus4;
    assign pc_plus4 = pc + 32'd4;

    // Branch/jump target from EX stage
    logic [31:0] branch_target;
    (* max_fanout = 4 *) logic pc_sel_mux;  // 1 = take branch/jump target
    (* max_fanout = 4 *) logic pc_sel_we;

    assign imem_addr = pc;
    assign imem_req  = pipe_en;

    // ---------------------------------------------------------------
    // Loop Buffer (Phase 7.5)
    // ---------------------------------------------------------------
    logic [31:0] lb_instr;
    logic        lb_valid;
    logic        loop_active;

    // Detect backward branch from EX stage for loop buffer
    logic ex_branch_is_back;
    assign ex_branch_is_back = (idex_is_branch & branch_taken_raw & idex_valid & pipe_en) &&
                             ($signed(idex_imm) < 0);

    Loop_Buffer #(.DEPTH(16)) loop_buf_inst (
        .clk           (clk),
        .rst           (rst),
        .fetch_pc      (pc),
        .fetch_instr   (imem_rdata),
        .fetch_valid   (imem_ready & pipe_en),
        .branch_taken  (branch_taken_raw & idex_valid & pipe_en),
        .branch_is_back(ex_branch_is_back),
        .branch_target (branch_target),
        .branch_pc     (idex_pc),
        .branch_valid  (idex_is_branch & idex_valid & pipe_en),
        .pipeline_flush(flush_id | flush_ex),
        .lb_instr      (lb_instr),
        .lb_valid      (lb_valid),
        .loop_active   (loop_active),
        .pipe_en       (pipe_en),
        .stall         (stall_if)
    );

    // Instruction source mux: loop buffer overrides I-cache when active
    logic [31:0] if_instr_raw;
    assign if_instr_raw = lb_valid ? lb_instr : imem_rdata;
    logic        if_instr_valid;
    assign if_instr_valid = lb_valid ? 1'b1 : imem_ready;

    // ---------------------------------------------------------------
    // RVC Decompressor (Phase 7.5)
    // ---------------------------------------------------------------
    logic [31:0] if_instr_expanded;
    logic        if_instr_is_compressed;
    logic        if_instr_illegal_c;

    RVC_Decompressor rvc_decomp_inst (
        .instr_c       (if_instr_raw[15:0]),
        .instr_out     (if_instr_expanded),
        .is_compressed (if_instr_is_compressed),
        .illegal_c     (if_instr_illegal_c)
    );

    // Select expanded or original instruction
    // Compressed if bits [1:0] != 2'b11 and instruction is valid
    logic use_compressed;
    assign use_compressed = if_instr_valid && (if_instr_raw[1:0] != 2'b11);
    logic [31:0] if_instr_final;
    assign if_instr_final = use_compressed ? if_instr_expanded : if_instr_raw;
    logic [31:0] if_pc_increment;
    assign if_pc_increment = use_compressed ? 32'd2 : 32'd4;

    // ---------------------------------------------------------------
    // Branch Predictor (Phase 7.5 — GShare + BTB + RAS)
    // ---------------------------------------------------------------
    logic        bp_predict_taken;
    logic [31:0] bp_predict_target;
    logic        bp_mispredict;

    // Update signals from EX stage (wired below after branch resolution)
    logic        bp_update_valid;
    assign bp_update_valid = idex_is_branch & idex_valid & pipe_en & ~stall_ex;
    logic        bp_branch_taken_actual;
    assign bp_branch_taken_actual = branch_taken_raw;

    Branch_Predictor #(
        .BHT_ENTRIES(64),
        .BTB_ENTRIES(64),
        .RAS_DEPTH(8),
        .GHR_WIDTH(6)
    ) bp_inst (
        .clk              (clk),
        .rst              (rst),
        .pc_if            (pc),
        .instr_if         (if_instr_final),
        .valid_if         (if_instr_valid & pipe_en & ~stall_if),
        .predict_taken    (bp_predict_taken),
        .predict_target   (bp_predict_target),
        .pc_ex            (idex_pc),
        .is_branch_ex     (idex_is_branch),
        .branch_taken_ex  (bp_branch_taken_actual),
        .branch_target_ex (branch_target),
        .update_valid     (bp_update_valid)
    );

    assign hz_icache_miss = imem_req & ~if_instr_valid;

    // PC mux — with branch prediction
    logic branch_redirect_if;
    assign branch_redirect_if = branch_taken_if | jalr_if;
    assign pc_sel_mux = bp_mispredict | jalr_pc_mux;
    assign pc_sel_we  = bp_mispredict | jalr_pc_we;

    // Next PC priority: misprediction correction > JAL > prediction > sequential
    logic bp_redirect;
    assign bp_redirect = bp_predict_taken & pipe_en & ~stall_if;
    assign pc_next = hz_is_jal_pc ? id_jal_target :
                     bp_redirect  ? bp_predict_target :
                     (pc + if_pc_increment);
    assign pc_commit_next = pc_sel_mux ? branch_target : pc_next;

    logic pc_we;
    assign pc_we = pipe_en & (!stall_if | pc_sel_we);
    // PC register
    always_ff @(posedge clk) begin
        if (rst)
            pc <= RESET_PC;
        else if (pc_load_en)
            pc <= pc_load;
        else if (pc_we)
            pc <= pc_commit_next;
    end

    assign pc_save = pc;

    // -------------------------------------------------------------------
    // IF/ID Pipeline Register (with prediction metadata)
    // -------------------------------------------------------------------
    logic [31:0] ifid_instr;
    logic [31:0] ifid_pc;
    logic        ifid_valid;
    logic        ifid_bp_taken;     // Phase 7.5: prediction carried down pipe
    logic [31:0] ifid_bp_target;    // Phase 7.5: predicted target

    always_ff @(posedge clk) begin
        if (rst || pc_load_en) begin
            ifid_instr     <= 32'h0000_0013;  // NOP (ADDI x0, x0, 0)
            ifid_pc        <= 32'b0;
            ifid_valid     <= 1'b0;
            ifid_bp_taken  <= 1'b0;
            ifid_bp_target <= 32'b0;
        end
        else if (pipe_en && (flush_id || branch_redirect_if || hz_is_jal_if || bp_mispredict)) begin
            ifid_valid <= 1'b0;
            ifid_bp_taken <= 1'b0;
        end
        else if (pipe_en && !stall_id) begin
            ifid_instr     <= if_instr_final;
            ifid_pc        <= pc;
            ifid_valid     <= if_instr_valid;
            ifid_bp_taken  <= bp_predict_taken;
            ifid_bp_target <= bp_predict_target;
        end
    end


    // ===================================================================
    //  STAGE 2: INSTRUCTION DECODE (ID)
    // ===================================================================

    // Decoder outputs
    logic [4:0]  dec_alu_op;
    logic        dec_alu_src;
    logic        dec_mem_read, dec_mem_write;
    logic [2:0]  dec_mem_funct3;
    logic        dec_reg_write;
    logic [1:0]  dec_wb_sel;
    logic        dec_is_branch, dec_is_jal, dec_is_jalr;
    logic        dec_is_atomic;
    logic [4:0]  dec_atomic_funct5;
    logic        dec_is_csr;
    logic [1:0]  dec_csr_op;
    logic        dec_is_ecall, dec_is_mret;
    logic        dec_illegal;

    decoder decoder_inst (
        .instr         (ifid_instr),
        .alu_op        (dec_alu_op),
        .alu_src       (dec_alu_src),
        .mem_read      (dec_mem_read),
        .mem_write     (dec_mem_write),
        .mem_funct3    (dec_mem_funct3),
        .reg_write     (dec_reg_write),
        .wb_sel        (dec_wb_sel),
        .is_branch     (dec_is_branch),
        .is_jal        (dec_is_jal),
        .is_jalr       (dec_is_jalr),
        .is_atomic     (dec_is_atomic),
        .atomic_funct5 (dec_atomic_funct5),
        .is_csr        (dec_is_csr),
        .csr_op        (dec_csr_op),
        .is_ecall      (dec_is_ecall),
        .is_mret       (dec_is_mret),
        .illegal_instr (dec_illegal)
    );

    // Register file addresses
    logic [4:0] id_rs1;
    assign id_rs1 = ifid_instr[19:15];
    logic [4:0] id_rs2;
    assign id_rs2 = ifid_instr[24:20];
    logic [4:0] id_rd;
    assign id_rd = ifid_instr[11:7];

    // Immediate generation
    logic [31:0] id_imm;
    imm_gen imm_gen_inst (
        .instr(ifid_instr),
        .imm  (id_imm)
    );

    // Register file reads
    logic [31:0] rf_rdata1, rf_rdata2;

    regfile regfile_inst (
        .clk       (clk),
        .we        (wb_reg_write_en),
        .waddr     (wb_rd),
        .wdata     (wb_wdata),
        .raddr1    (id_rs1),
        .raddr2    (id_rs2),
        .rdata1    (rf_rdata1),
        .rdata2    (rf_rdata2),
        .mig_we    (mig_we),
        .mig_addr  (mig_addr),
        .mig_wdata (mig_wdata),
        .mig_rdata (mig_rdata)
    );

    // JAL target (resolved in ID stage for early branch)
    assign id_jal_target = ifid_pc + id_imm;
    logic jal_dec;
    assign jal_dec = dec_is_jal & ifid_valid & pipe_en;
    assign hz_is_jal = jal_dec;
    assign hz_is_jal_pc = hz_is_jal;
    assign hz_is_jal_if = hz_is_jal;

    // -------------------------------------------------------------------
    // ID/EX Pipeline Register
    // -------------------------------------------------------------------

    // EX/MEM register declarations
    (* max_fanout = 4 *) logic [31:0] exmem_alu_result;
    logic [31:0] exmem_rs2_data;
    logic [4:0]  exmem_rd;
    logic        exmem_mem_read, exmem_mem_write;
    logic [2:0]  exmem_mem_funct3;
    logic        exmem_reg_write;
    logic [1:0]  exmem_wb_sel;
    logic        exmem_valid;
    logic        exmem_is_lr;
    logic        exmem_is_sc;
    logic        exmem_sc_success;

    // Evaluate forwarding for NEXT cycle based on CURRENT pipeline contents.
    // Declarations above avoid implicit-logic creation in synthesis.
    logic [1:0] fwd_a_next;
    assign fwd_a_next = (idex_reg_write && idex_rd != 5'b0 && idex_rd == id_rs1) ? 2'b01 :
                            (exmem_reg_write && exmem_rd != 5'b0 && exmem_rd == id_rs1) ? 2'b10 : 2'b00;

    logic [1:0] fwd_b_next;
    assign fwd_b_next = (idex_reg_write && idex_rd != 5'b0 && idex_rd == id_rs2) ? 2'b01 :
                            (exmem_reg_write && exmem_rd != 5'b0 && exmem_rd == id_rs2) ? 2'b10 : 2'b00;

    always_ff @(posedge clk) begin
        if (rst || pc_load_en) begin
            idex_pc           <= 32'b0;
            idex_rs1_data     <= 32'b0;
            idex_rs2_data     <= 32'b0;
            idex_imm          <= 32'b0;
            idex_rs1          <= 5'b0;
            idex_rd           <= 5'b0;
            idex_alu_op       <= 5'b0;
            idex_alu_src      <= 1'b0;
            idex_mem_read     <= 1'b0;
            idex_mem_write    <= 1'b0;
            idex_mem_funct3   <= 3'b0;
            idex_reg_write    <= 1'b0;
            idex_wb_sel       <= 2'b0;
            idex_is_branch    <= 1'b0;
            idex_is_jal       <= 1'b0;
            idex_is_jalr      <= 1'b0;
            idex_is_atomic    <= 1'b0;
            idex_atomic_funct5<= 5'b0;
            idex_is_csr       <= 1'b0;
            idex_csr_op       <= 2'b0;
            idex_csr_addr     <= 12'b0;
            idex_is_ecall     <= 1'b0;
            idex_is_mret      <= 1'b0;
            idex_funct3       <= 3'b0;
            idex_valid        <= 1'b0;
            idex_instr        <= 32'h0000_0013;
            idex_fwd_a        <= 2'b00;
            idex_fwd_b        <= 2'b00;
            idex_bp_taken     <= 1'b0;
            idex_bp_target    <= 32'b0;
        end
        else if (pipe_en && flush_ex) begin
            // Inject EX bubble through valid/control bits instead of flushing
            // the whole payload to avoid deep control-to-reset timing paths.
            idex_mem_read      <= 1'b0;
            idex_mem_write     <= 1'b0;
            idex_reg_write     <= 1'b0;
            idex_wb_sel        <= 2'b0;
            idex_is_branch     <= 1'b0;
            idex_is_jal        <= 1'b0;
            idex_is_jalr       <= 1'b0;
            idex_is_atomic     <= 1'b0;
            idex_is_csr        <= 1'b0;
            idex_is_ecall      <= 1'b0;
            idex_is_mret       <= 1'b0;
            idex_valid         <= 1'b0;
            idex_instr         <= 32'h0000_0013;
            idex_fwd_a         <= 2'b00;
            idex_fwd_b         <= 2'b00;
            idex_bp_taken      <= 1'b0;
            idex_bp_target     <= 32'b0;
        end
        else if (pipe_en && !stall_ex) begin
            idex_pc           <= ifid_pc;
            idex_rs1_data     <= rf_rdata1;
            idex_rs2_data     <= rf_rdata2;
            idex_imm          <= id_imm;
            idex_rs1          <= id_rs1;
            idex_rd           <= id_rd;
            idex_alu_op       <= dec_alu_op;
            idex_alu_src      <= dec_alu_src;
            idex_mem_read     <= dec_mem_read;
            idex_mem_write    <= dec_mem_write;
            idex_mem_funct3   <= dec_mem_funct3;
            idex_reg_write    <= dec_reg_write;
            idex_wb_sel       <= dec_wb_sel;
            idex_is_branch    <= dec_is_branch;
            idex_is_jal       <= dec_is_jal;
            idex_is_jalr      <= dec_is_jalr;
            idex_is_atomic    <= dec_is_atomic;
            idex_atomic_funct5<= dec_atomic_funct5;
            idex_is_csr       <= dec_is_csr;
            idex_csr_op       <= dec_csr_op;
            idex_csr_addr     <= ifid_instr[31:20];
            idex_is_ecall     <= dec_is_ecall;
            idex_is_mret      <= dec_is_mret;
            idex_funct3       <= ifid_instr[14:12];
            idex_valid        <= ifid_valid;
            idex_instr        <= ifid_instr;
            idex_fwd_a        <= fwd_a_next;
            idex_fwd_b        <= fwd_b_next;
            idex_bp_taken     <= ifid_bp_taken;
            idex_bp_target    <= ifid_bp_target;
        end
    end


    // ===================================================================
    //  STAGE 3: EXECUTE (EX)
    // ===================================================================

    // Forwarding muxes (bit-select style reduces compare/mux depth).
    logic [31:0] ex_rs1_fwd_pre;
    assign ex_rs1_fwd_pre = idex_fwd_a[0] ? exmem_alu_result : idex_rs1_data;
    logic [31:0] ex_rs2_fwd_pre;
    assign ex_rs2_fwd_pre = idex_fwd_b[0] ? exmem_alu_result : idex_rs2_data;
    logic [31:0] ex_rs1_fwd;
    assign ex_rs1_fwd = idex_fwd_a[1] ? wb_wdata : ex_rs1_fwd_pre;
    logic [31:0] ex_rs2_fwd;
    assign ex_rs2_fwd = idex_fwd_b[1] ? wb_wdata : ex_rs2_fwd_pre;
    // Branch compare uses ID/EX latched operands. RAW dependencies are
    // handled by branch-dependency stalls in hazard_unit.
    (* max_fanout = 4 *) logic [31:0] ex_rs1_branch = idex_rs1_data;
    (* max_fanout = 4 *) logic [31:0] ex_rs2_branch = idex_rs2_data;

    // ALU input B mux: register or immediate
    logic [31:0] alu_b_in;
    assign alu_b_in = idex_alu_src ? idex_imm : ex_rs2_fwd;

    // ALU input A: for AUIPC, use PC; otherwise use rs1
    logic is_auipc;
    assign is_auipc = (idex_instr[6:0] == 7'b0010111);
    logic [31:0] alu_a_in;
    assign alu_a_in = is_auipc ? idex_pc : ex_rs1_fwd;

    // ALU
    logic [31:0] alu_result;
    logic        alu_zero;
    logic        alu_done;
    logic        alu_start;
    assign alu_start = idex_valid & pipe_en & ~stall_ex;

    pipelined_alu alu_inst (
        .clk    (clk),
        .rst    (rst),
        .start  (alu_start),
        .a      (alu_a_in),
        .b      (alu_b_in),
        .alu_op (idex_alu_op),
        .result (alu_result),
        .zero   (alu_zero),
        .done   (alu_done)
    );

    // Branch unit

    branch_unit branch_inst (
        .rs1_data     (ex_rs1_branch),
        .rs2_data     (ex_rs2_branch),
        .funct3       (idex_funct3),
        .is_branch    (idex_is_branch),
        .branch_taken (branch_taken_raw)
    );

    logic ex_ctrl_valid;
    assign ex_ctrl_valid = idex_valid & pipe_en;

    // ---------------------------------------------------------------
    // Branch Misprediction Detection (Phase 7.5)
    // ---------------------------------------------------------------
    // Misprediction = predicted taken but actually not taken, OR
    //                 predicted not-taken but actually taken, OR
    //                 predicted taken but to the wrong target.
    logic bp_direction_wrong;
    assign bp_direction_wrong = (idex_bp_taken != branch_taken_raw);
    logic bp_target_wrong;
    assign bp_target_wrong = idex_bp_taken && branch_taken_raw &&
                              (idex_bp_target != (idex_is_jalr ? {alu_result[31:1], 1'b0}
                                                              : idex_pc + idex_imm));
    assign bp_mispredict = idex_is_branch & ex_ctrl_valid &
                           (bp_direction_wrong | bp_target_wrong);

    // On correct prediction: no flush needed.
    // On misprediction: redirect PC to actual target (or PC+4 if not taken).
    (* keep = "true", max_fanout = 1 *) logic branch_taken_pc_mux_w = bp_mispredict;
    (* keep = "true", max_fanout = 1 *) logic branch_taken_pc_we_w  = bp_mispredict;
    (* keep = "true", max_fanout = 1 *) logic branch_taken_if_w = bp_mispredict;
    (* keep = "true", max_fanout = 1 *) logic branch_taken_hz_w = bp_mispredict;
    (* keep = "true", max_fanout = 1 *) logic jalr_pc_mux_w     = idex_is_jalr & ex_ctrl_valid;
    (* keep = "true", max_fanout = 1 *) logic jalr_pc_we_w      = idex_is_jalr & ex_ctrl_valid;
    (* keep = "true", max_fanout = 1 *) logic jalr_if_w         = idex_is_jalr & ex_ctrl_valid;
    (* keep = "true", max_fanout = 1 *) logic jalr_hz_w         = idex_is_jalr & ex_ctrl_valid;

    assign branch_taken_pc_mux = branch_taken_pc_mux_w;
    assign branch_taken_pc_we  = branch_taken_pc_we_w;
    assign branch_taken_if = branch_taken_if_w;
    assign branch_taken_hz = branch_taken_hz_w;
    assign jalr_pc_mux     = jalr_pc_mux_w;
    assign jalr_pc_we      = jalr_pc_we_w;
    assign jalr_if         = jalr_if_w;
    assign jalr_hz         = jalr_hz_w;

    // Branch / jump target (misprediction correction: use actual target)
    assign branch_target = idex_is_jalr ? {alu_result[31:1], 1'b0} :
                                          idex_pc + idex_imm;

    // CSR unit
    logic [31:0] csr_rdata;
    logic [31:0] csr_wdata_val;
    assign csr_wdata_val = idex_funct3[2] ? {27'b0, idex_rs1} : ex_rs1_fwd; // CSRR*I uses zimm
    logic        csr_we_ex;
    assign csr_we_ex = idex_is_csr & idex_valid & pipe_en & ~stall_ex;

    logic [31:0] mtvec_val, mepc_val;
    logic        mstatus_mie;
    logic        interrupt_pending; // Pending bit from CSR

    // Trap logic
    logic trap_enter;
    assign trap_enter = (idex_is_ecall | interrupt_pending) & idex_valid & pipe_en;
    
    // Priority encoder for causes
    logic [31:0] trap_cause;
    assign trap_cause = idex_is_ecall ? 32'd11 :          // M-mode ecall
                        (interrupt_pending & timer_irq)   ? 32'h8000_0007 :
                        (interrupt_pending & ext_irq)     ? 32'h8000_000B :
                        (interrupt_pending & thermal_irq) ? 32'h8000_0010 :
                        32'd2; // Illegal

    logic mret_ex;
    assign mret_ex = idex_is_mret & idex_valid & pipe_en;

    csr_unit #(.HART_ID(HART_ID)) csr_inst (
        .clk            (clk),
        .rst            (rst),
        .csr_addr       (idex_csr_addr),
        .csr_wdata      (csr_wdata_val),
        .csr_op         (idex_csr_op),
        .csr_we         (csr_we_ex),
        .csr_rdata      (csr_rdata),
        .trap_enter     (trap_enter),
        .trap_cause     (trap_cause),
        .trap_val       (32'b0),
        .trap_pc        (idex_pc),
        .mret           (mret_ex),
        .mtvec_out      (mtvec_val),
        .mepc_out       (mepc_val),
        .mstatus_mie    (mstatus_mie),
        .interrupt_pending(interrupt_pending),
        .timer_irq      (timer_irq),
        .ext_irq        (ext_irq),
        .thermal_irq    (thermal_irq),
        .thermal_level  (thermal_level),
        .perf_overflow_irq(perf_overflow_irq),
        .thermal_reading (thermal_reading),
        .instr_retired  (retired),
        .migration_event(migration_event),
        .mig_csr_we     (mig_csr_we),
        .mig_csr_addr   (mig_csr_addr),
        .mig_csr_wdata  (mig_csr_wdata),
        .mig_csr_rdata  (mig_csr_rdata)
    );

    // Atomic reservation logic
    logic [31:0] reservation_addr;
    logic [15:0] reservation_tag;
    logic        reservation_valid;

    logic idex_is_lr;
    assign idex_is_lr = idex_is_atomic && (idex_atomic_funct5 == 5'b00010);
    logic idex_is_sc;
    assign idex_is_sc = idex_is_atomic && (idex_atomic_funct5 == 5'b00011);
    logic snoop_hit;
    assign snoop_hit = snoop_we && reservation_valid && (snoop_addr[17:2] == reservation_tag);
    logic lr_set;
    assign lr_set = pipe_en && idex_valid && idex_is_lr;
    logic sc_success;
    assign sc_success = reservation_valid && (ex_rs1_fwd == reservation_addr);
    logic sc_clear;
    assign sc_clear = pipe_en && idex_valid && idex_is_sc && sc_success;

    // Keep reservation address write-enable independent of snoop invalidate
    // so cross-core snoop routing does not feed CE timing on this register bank.
    always_ff @(posedge clk) begin
        if (rst || pc_load_en)
            reservation_addr <= 32'b0;
        else if (lr_set)
            reservation_addr <= ex_rs1_fwd;
    end

    always_ff @(posedge clk) begin
        if (rst || pc_load_en)
            reservation_tag <= 16'b0;
        else if (lr_set)
            reservation_tag <= ex_rs1_fwd[17:2];
    end

    always_ff @(posedge clk) begin
        if (rst || pc_load_en)
            reservation_valid <= 1'b0;
        else if (snoop_hit)
            reservation_valid <= 1'b0;  // Snoop invalidate
        else if (lr_set)
            reservation_valid <= 1'b1;
        else if (sc_clear)
            reservation_valid <= 1'b0;  // SC.W success -- clear reservation
    end

    // EX result mux (ALU, CSR, PC+4, SC result)
    logic [31:0] ex_result;
    assign ex_result = idex_is_csr ? csr_rdata :
                            (idex_is_jal | idex_is_jalr) ? (idex_pc + 32'd4) :
                            idex_is_sc ?
                                {31'b0, ~sc_success} :  // SC.W returns 0 on success
                            alu_result;

    // Hazard unit connections
    assign hz_ex_rd        = idex_rd;
    assign hz_ex_reg_write = idex_reg_write & idex_valid;
    assign hz_ex_mem_read  = idex_mem_read & idex_valid;
    assign hz_alu_busy     = ~alu_done;
    assign hz_atomic_busy  = 1'b0;  // Single-cycle atomics for now

    // -------------------------------------------------------------------
    // EX/MEM Pipeline Register
    // -------------------------------------------------------------------
    always_ff @(posedge clk) begin
        if (rst || pc_load_en) begin
            exmem_alu_result    <= 32'b0;
            exmem_rs2_data      <= 32'b0;
            exmem_rd            <= 5'b0;
            exmem_mem_read      <= 1'b0;
            exmem_mem_write     <= 1'b0;
            exmem_mem_funct3    <= 3'b0;
            exmem_reg_write     <= 1'b0;
            exmem_wb_sel        <= 2'b0;
            exmem_valid         <= 1'b0;
            exmem_is_lr         <= 1'b0;
            exmem_is_sc         <= 1'b0;
            exmem_sc_success    <= 1'b0;
        end
        else if (pipe_en && !stall_mem) begin
            exmem_alu_result    <= ex_result;
            exmem_rs2_data      <= ex_rs2_fwd;
            exmem_rd            <= idex_rd;
            exmem_mem_read      <= idex_mem_read;
            exmem_mem_write     <= idex_mem_write;
            exmem_mem_funct3    <= idex_mem_funct3;
            exmem_reg_write     <= idex_reg_write & idex_valid;
            exmem_wb_sel        <= idex_wb_sel;
            exmem_valid         <= idex_valid;
            exmem_is_lr         <= idex_is_lr;
            exmem_is_sc         <= idex_is_sc;
            exmem_sc_success    <= sc_success;
        end
    end


    // ===================================================================
    //  STAGE 4: MEMORY ACCESS (MEM)
    // ===================================================================

    // Data memory request generation
    logic mem_is_load;
    assign mem_is_load = exmem_mem_read & exmem_valid;
    logic mem_is_store;
    assign mem_is_store = exmem_mem_write & exmem_valid;
    logic mem_is_lr;
    assign mem_is_lr = exmem_is_lr & exmem_valid;
    logic mem_is_sc;
    assign mem_is_sc = exmem_is_sc & exmem_valid;

    // Byte/halfword/word alignment
    logic [1:0] mem_byte_offset;
    assign mem_byte_offset = exmem_alu_result[1:0];

    // Write strobe generation based on funct3 (SB/SH/SW)
    logic [3:0]  mem_wstrb_gen;
    logic [31:0] mem_wdata_gen;

    always_comb begin
        mem_wstrb_gen = 4'b0000;
        mem_wdata_gen = exmem_rs2_data;

        case (exmem_mem_funct3[1:0])
            2'b00: begin // SB
                case (mem_byte_offset)
                    2'b00: begin mem_wstrb_gen = 4'b0001; mem_wdata_gen = {24'b0, exmem_rs2_data[7:0]};       end
                    2'b01: begin mem_wstrb_gen = 4'b0010; mem_wdata_gen = {16'b0, exmem_rs2_data[7:0], 8'b0}; end
                    2'b10: begin mem_wstrb_gen = 4'b0100; mem_wdata_gen = {8'b0, exmem_rs2_data[7:0], 16'b0}; end
                    2'b11: begin mem_wstrb_gen = 4'b1000; mem_wdata_gen = {exmem_rs2_data[7:0], 24'b0};       end
                endcase
            end
            2'b01: begin // SH
                case (mem_byte_offset[1])
                    1'b0: begin mem_wstrb_gen = 4'b0011; mem_wdata_gen = {16'b0, exmem_rs2_data[15:0]};       end
                    1'b1: begin mem_wstrb_gen = 4'b1100; mem_wdata_gen = {exmem_rs2_data[15:0], 16'b0};       end
                endcase
            end
            2'b10: begin // SW
                mem_wstrb_gen = 4'b1111;
                mem_wdata_gen = exmem_rs2_data;
            end
            default: begin
                mem_wstrb_gen = 4'b1111;
                mem_wdata_gen = exmem_rs2_data;
            end
        endcase
    end

    // Drive data memory interface
    always_comb begin
        dmem_addr  = {exmem_alu_result[31:2], 2'b00};  // Word-aligned
        dmem_wdata = mem_wdata_gen;
        dmem_wstrb = mem_wstrb_gen;
        dmem_we    = 1'b0;
        dmem_req   = 1'b0;

        if (mem_is_load || mem_is_lr) begin
            dmem_req = 1'b1;
            dmem_we  = 1'b0;
        end
        else if (mem_is_store) begin
            dmem_req = 1'b1;
            dmem_we  = 1'b1;
        end
        else if (mem_is_sc && exmem_sc_success) begin
            dmem_req = 1'b1;
            dmem_we  = 1'b1;
        end
    end

    assign hz_dmem_stall = dmem_req & ~dmem_ready;

    // Load data extraction (sign/zero extension for LB/LH/LBU/LHU/LW)
    logic [31:0] load_data;

    always_comb begin
        case (exmem_mem_funct3)
            3'b000: begin // LB (sign-extend byte)
                case (mem_byte_offset)
                    2'b00: load_data = {{24{dmem_rdata[7]}},  dmem_rdata[7:0]};
                    2'b01: load_data = {{24{dmem_rdata[15]}}, dmem_rdata[15:8]};
                    2'b10: load_data = {{24{dmem_rdata[23]}}, dmem_rdata[23:16]};
                    2'b11: load_data = {{24{dmem_rdata[31]}}, dmem_rdata[31:24]};
                endcase
            end
            3'b001: begin // LH (sign-extend halfword)
                case (mem_byte_offset[1])
                    1'b0: load_data = {{16{dmem_rdata[15]}}, dmem_rdata[15:0]};
                    1'b1: load_data = {{16{dmem_rdata[31]}}, dmem_rdata[31:16]};
                endcase
            end
            3'b010: load_data = dmem_rdata;  // LW
            3'b100: begin // LBU (zero-extend byte)
                case (mem_byte_offset)
                    2'b00: load_data = {24'b0, dmem_rdata[7:0]};
                    2'b01: load_data = {24'b0, dmem_rdata[15:8]};
                    2'b10: load_data = {24'b0, dmem_rdata[23:16]};
                    2'b11: load_data = {24'b0, dmem_rdata[31:24]};
                endcase
            end
            3'b101: begin // LHU (zero-extend halfword)
                case (mem_byte_offset[1])
                    1'b0: load_data = {16'b0, dmem_rdata[15:0]};
                    1'b1: load_data = {16'b0, dmem_rdata[31:16]};
                endcase
            end
            default: load_data = dmem_rdata;
        endcase
    end

    // Hazard unit connections for MEM stage
    assign hz_mem_rd        = exmem_rd;
    assign hz_mem_reg_write = exmem_reg_write;

    // -------------------------------------------------------------------
    // MEM/WB Pipeline Register
    // -------------------------------------------------------------------
    logic [31:0] memwb_wdata_r;
    logic [4:0]  memwb_rd;
    logic        memwb_reg_write;
    logic        memwb_valid;

    always_ff @(posedge clk) begin
        if (rst || pc_load_en) begin
            memwb_wdata_r    <= 32'b0;
            memwb_rd         <= 5'b0;
            memwb_reg_write  <= 1'b0;
            memwb_valid      <= 1'b0;
        end
        else if (pipe_en) begin
            memwb_wdata_r    <= (exmem_wb_sel == 2'b01) ? load_data : exmem_alu_result;
            memwb_rd         <= exmem_rd;
            memwb_reg_write  <= exmem_reg_write;
            memwb_valid      <= exmem_valid;
        end
    end


    // ===================================================================
    //  STAGE 5: WRITE BACK (WB)
    // ===================================================================
    assign wb_rd = memwb_rd;
    assign wb_reg_write_en = memwb_reg_write & memwb_valid & pipe_en;

    // Final WB value is precomputed in MEM/WB to reduce WB control fanout.
    assign wb_wdata = memwb_wdata_r;

    assign hz_wb_rd        = wb_rd;
    assign hz_wb_reg_write = wb_reg_write_en;


    // ===================================================================
    //  HAZARD UNIT INSTANTIATION
    // ===================================================================
    hazard_unit hazard_inst (
        .id_raw_rs1     (id_rs1),
        .id_raw_rs2     (id_rs2),
        .ex_rd          (hz_ex_rd),
        .ex_reg_write   (hz_ex_reg_write),
        .ex_mem_read    (hz_ex_mem_read),
        .mem_rd         (hz_mem_rd),
        .mem_reg_write  (hz_mem_reg_write),
        .wb_rd          (hz_wb_rd),
        .wb_reg_write   (hz_wb_reg_write),
        .is_branch_id   (dec_is_branch & ifid_valid),
        .branch_taken   (branch_taken_hz),
        .is_jal         (jal_dec),
        .is_jalr        (jalr_hz),
        .bp_mispredict  (bp_mispredict),
        .atomic_busy    (hz_atomic_busy),
        .alu_busy       (hz_alu_busy),
        .icache_miss    (hz_icache_miss),
        .dmem_stall     (hz_dmem_stall),
        .stall_if       (stall_if),
        .stall_id       (stall_id),
        .stall_ex       (stall_ex),
        .stall_mem      (stall_mem),
        .flush_if       (flush_if),
        .flush_id       (flush_id),
        .flush_ex       (flush_ex)
    );


    // ===================================================================
    //  STATUS OUTPUTS
    // ===================================================================
    assign active  = pipe_en & (ifid_valid | idex_valid | exmem_valid | memwb_valid | dmem_req);
    assign retired = wb_reg_write_en | (memwb_valid & pipe_en & ~memwb_reg_write);
    // retired fires whenever an instruction completes WB (writes or not)

endmodule
