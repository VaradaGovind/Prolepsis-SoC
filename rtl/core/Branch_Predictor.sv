`timescale 1ns / 1ps
//============================================================================
// OrionRV — Branch Predictor (Phase 7.5)
//
// Combines three prediction mechanisms:
//   1. GShare BHT — 64-entry, 2-bit saturating counters indexed by
//      PC[7:2] XOR GHR[5:0].  Captures correlated branch patterns.
//   2. BTB — 64-entry Branch Target Buffer.  Stores last-known target
//      address per PC hash.  Supplies redirect target on predicted-taken.
//   3. RAS — 8-entry Return Address Stack.  Pushes on JAL/JALR with
//      rd = x1/x5 (call); pops on JALR with rs1 = x1/x5, rd = x0 (ret).
//
// Prediction is delivered combinationally in the IF stage so the PC
// can redirect in the same cycle (no bubble on correct prediction).
//
// Update arrives one or two cycles later from the EX stage when the
// branch is resolved.
//============================================================================

module Branch_Predictor #(
    parameter BHT_ENTRIES = 64,
    parameter BTB_ENTRIES = 64,
    parameter RAS_DEPTH   = 8,
    parameter GHR_WIDTH   = 6
)(
    input  logic        clk,
    input  logic        rst,

    // ------------------------------------------------------------------
    // Predict port (IF stage — combinational read)
    // ------------------------------------------------------------------
    input  logic [31:0] pc_if,            // Current fetch PC
    input  logic [31:0] instr_if,         // Fetched instruction (for RAS call/ret detect)
    input  logic        valid_if,         // Instruction is valid this cycle

    output logic        predict_taken,    // Prediction: taken
    output logic [31:0] predict_target,   // Predicted target address

    // ------------------------------------------------------------------
    // Update port (EX stage — sequential write)
    // ------------------------------------------------------------------
    input  logic [31:0] pc_ex,            // PC of the resolved branch
    input  logic        is_branch_ex,     // Instruction in EX is a branch
    input  logic        branch_taken_ex,  // Actual outcome
    input  logic [31:0] branch_target_ex, // Actual target
    input  logic        update_valid      // Update is valid this cycle
);

    // ==================================================================
    // Index hashing
    // ==================================================================
    localparam IDX_W = $clog2(BHT_ENTRIES);  // 6

    // Global History Register (GHR)
    logic [GHR_WIDTH-1:0] ghr;

    logic [IDX_W-1:0] predict_idx;
    assign predict_idx = pc_if[IDX_W+1:2] ^ ghr;
    logic [IDX_W-1:0] update_idx;
    assign update_idx = pc_ex[IDX_W+1:2] ^ ghr;

    // ==================================================================
    // BHT — 2-bit saturating counters
    //   00 = Strongly Not-Taken
    //   01 = Weakly Not-Taken
    //   10 = Weakly Taken
    //   11 = Strongly Taken
    // ==================================================================
    logic [1:0] bht [0:BHT_ENTRIES-1];

    logic [1:0] bht_counter;
    assign bht_counter = bht[predict_idx];
    logic       bht_taken;
    assign bht_taken = bht_counter[1];  // MSB = direction

    // ==================================================================
    // BTB — Branch Target Buffer
    // ==================================================================
    logic [31:0]         btb_target [0:BTB_ENTRIES-1];
    logic [19:0]         btb_tag    [0:BTB_ENTRIES-1];
    logic                btb_valid  [0:BTB_ENTRIES-1];

    logic [IDX_W-1:0]  btb_pred_idx;
    assign btb_pred_idx = pc_if[IDX_W+1:2];
    logic [19:0]        btb_pred_tag;
    assign btb_pred_tag = pc_if[31:12];
    logic               btb_hit;
    assign btb_hit = btb_valid[btb_pred_idx] &&
                                      (btb_tag[btb_pred_idx] == btb_pred_tag);

    // ==================================================================
    // RAS — Return Address Stack
    // ==================================================================
    logic [31:0] ras_stack [0:RAS_DEPTH-1];
    logic [$clog2(RAS_DEPTH)-1:0] ras_tos;  // Top-of-stack pointer

    // Call/return detection from instruction bits (IF stage)
    // Call: JAL/JALR with rd = x1 or x5
    // Ret:  JALR with rs1 = x1 or x5, rd = x0
    logic [6:0]  opcode_if;
    assign opcode_if = instr_if[6:0];
    logic [4:0]  rd_if;
    assign rd_if = instr_if[11:7];
    logic [4:0]  rs1_if;
    assign rs1_if = instr_if[19:15];

    logic is_jal_if;
    assign is_jal_if = (opcode_if == 7'b1101111);
    logic is_jalr_if;
    assign is_jalr_if = (opcode_if == 7'b1100111);
    logic rd_is_link;
    assign rd_is_link = (rd_if == 5'd1) || (rd_if == 5'd5);
    logic rs1_is_link;
    assign rs1_is_link = (rs1_if == 5'd1) || (rs1_if == 5'd5);

    logic is_call;
    assign is_call = valid_if && (is_jal_if || is_jalr_if) && rd_is_link;
    logic is_ret;
    assign is_ret = valid_if && is_jalr_if && rs1_is_link && (rd_if == 5'd0);

    logic [31:0] ras_top;
    assign ras_top = ras_stack[ras_tos];

    // ==================================================================
    // Prediction output mux
    // ==================================================================
    // Priority: RAS return > BHT+BTB prediction
    logic predict_from_ras;
    assign predict_from_ras = is_ret;
    logic predict_from_bht;
    assign predict_from_bht = bht_taken && btb_hit && !is_ret;

    assign predict_taken  = valid_if && (predict_from_ras || predict_from_bht);
    assign predict_target = predict_from_ras ? ras_top :
                            predict_from_bht ? btb_target[btb_pred_idx] :
                            32'b0;

    // ==================================================================
    // Sequential update logic
    // ==================================================================
    integer i;

    always_ff @(posedge clk) begin
        if (rst) begin
            ghr <= {GHR_WIDTH{1'b0}};
            ras_tos <= {$clog2(RAS_DEPTH){1'b0}};

            for (i = 0; i < BHT_ENTRIES; i = i + 1)
                bht[i] <= 2'b01;  // Weakly Not-Taken

            for (i = 0; i < BTB_ENTRIES; i = i + 1) begin
                btb_target[i] <= 32'b0;
                btb_tag[i]    <= 20'b0;
                btb_valid[i]  <= 1'b0;
            end

            for (i = 0; i < RAS_DEPTH; i = i + 1)
                ras_stack[i] <= 32'b0;
        end
        else begin
            // -------------------------------------------------------
            // RAS push / pop (IF stage timing)
            // -------------------------------------------------------
            if (is_call && !is_ret) begin
                // Push return address
                ras_tos <= (ras_tos == RAS_DEPTH - 1) ? {$clog2(RAS_DEPTH){1'b0}}
                                                      : ras_tos + 1'b1;
                ras_stack[(ras_tos == RAS_DEPTH-1) ? {$clog2(RAS_DEPTH){1'b0}}
                                                  : ras_tos + 1'b1] <= pc_if + 32'd4;
            end
            else if (is_ret && !is_call) begin
                // Pop (just move pointer; data stays)
                ras_tos <= (ras_tos == {$clog2(RAS_DEPTH){1'b0}}) ? RAS_DEPTH[$clog2(RAS_DEPTH)-1:0] - 1'b1
                                                                  : ras_tos - 1'b1;
            end
            // Simultaneous call+ret (e.g., coroutine swap): push then pop = no change

            // -------------------------------------------------------
            // BHT counter update (EX stage resolution)
            // -------------------------------------------------------
            if (update_valid && is_branch_ex) begin
                if (branch_taken_ex) begin
                    // Saturating increment
                    if (bht[update_idx] != 2'b11)
                        bht[update_idx] <= bht[update_idx] + 2'b01;
                end
                else begin
                    // Saturating decrement
                    if (bht[update_idx] != 2'b00)
                        bht[update_idx] <= bht[update_idx] - 2'b01;
                end
            end

            // -------------------------------------------------------
            // BTB target update (EX stage)
            // -------------------------------------------------------
            if (update_valid && is_branch_ex && branch_taken_ex) begin
                btb_target[pc_ex[IDX_W+1:2]] <= branch_target_ex;
                btb_tag[pc_ex[IDX_W+1:2]]    <= pc_ex[31:12];
                btb_valid[pc_ex[IDX_W+1:2]]  <= 1'b1;
            end

            // -------------------------------------------------------
            // GHR shift (EX stage — shift in actual outcome)
            // -------------------------------------------------------
            if (update_valid && is_branch_ex) begin
                ghr <= {ghr[GHR_WIDTH-2:0], branch_taken_ex};
            end
        end
    end

endmodule
