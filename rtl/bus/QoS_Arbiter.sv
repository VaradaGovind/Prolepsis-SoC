`timescale 1ns / 1ps
//============================================================================
// OrionRV - Quality of Service (QoS) Arbiter
//
// N-master priority arbiter with:
//   - 3-bit QoS priority per master (0=lowest, 7=highest)
//   - Age-based starvation prevention (masters waiting too long get boosted)
//   - Round-robin tie-breaking among equal priorities
//   - Lock support for atomic multi-beat transactions
//
// Timing architecture:
//   - Stage A: Register boosted QoS per requester
//   - Stage B: Register best-QoS result + snapshots
//   - Stage C: Build and register candidate mask
//   - Stage D: Round-robin pick + register grant
//
// This breaks the long age->grant combinational cone seen in timing reports.
//============================================================================

module qos_arbiter #(
    parameter NUM_MASTERS     = 11,
    parameter QOS_WIDTH       = 3,
    parameter AGE_BITS        = 8,      // Starvation counter width
    parameter AGE_BOOST_THRESH = 200    // Cycles before age boost kicks in
)(
    input  logic clk,
    input  logic rst,

    // -------------------------------------------------------------------
    // Master request and QoS inputs
    // -------------------------------------------------------------------
    input  logic [NUM_MASTERS-1:0]              req,        // Request from each master
    input  logic [NUM_MASTERS*QOS_WIDTH-1:0]    qos_in,     // QoS level per master
    input  logic [NUM_MASTERS-1:0]              lock,       // Transaction lock (for atomics)

    // -------------------------------------------------------------------
    // Grant output
    // -------------------------------------------------------------------
    output logic  [NUM_MASTERS-1:0]              grant,      // One-hot grant
    output logic  [$clog2(NUM_MASTERS)-1:0]      grant_idx,  // Binary grant index
    output logic                                 grant_valid // At least one master granted
);

    localparam IDX_W = $clog2(NUM_MASTERS);

    // ===================================================================
    // Per-master age counters (starvation prevention)
    // ===================================================================
    logic [AGE_BITS-1:0] age [0:NUM_MASTERS-1];

    integer a;
    always_ff @(posedge clk) begin
        if (rst) begin
            for (a = 0; a < NUM_MASTERS; a = a + 1) begin
                age[a] <= 0;
            end
        end
        else begin
            for (a = 0; a < NUM_MASTERS; a = a + 1) begin
                if (grant[a]) begin
                    age[a] <= 0;  // Reset age on grant
                end
                else if (req[a] && age[a] < {AGE_BITS{1'b1}}) begin
                    age[a] <= age[a] + 1;  // Increment while waiting
                end
            end
        end
    end

    // ===================================================================
    // Round-robin pointer (for tie-breaking)
    // ===================================================================
    logic [IDX_W-1:0] rr_ptr;

    always_ff @(posedge clk) begin
        if (rst)
            rr_ptr <= 0;
        else if (grant_valid)
            rr_ptr <= (grant_idx == NUM_MASTERS - 1) ? 0 : grant_idx + 1;
    end

    // ===================================================================
    // Lock support: if a master holds a lock, it keeps the grant
    // ===================================================================
    logic                locked;
    logic [IDX_W-1:0]    locked_idx;

    always_ff @(posedge clk) begin
        if (rst) begin
            locked     <= 1'b0;
            locked_idx <= 0;
        end
        else if (locked && !req[locked_idx]) begin
            // Lock holder released request
            locked <= 1'b0;
        end
        else if (grant_valid && lock[grant_idx] && !locked) begin
            // New lock acquired
            locked     <= 1'b1;
            locked_idx <= grant_idx;
        end
    end

    // ===================================================================
    // Pipelined grant selection
    // ===================================================================
    localparam EFF_QW = QOS_WIDTH + 1;

    integer i;
    integer j;
    logic [QOS_WIDTH:0] cand_qos;
    logic [IDX_W-1:0]   idx;
    logic               found;

    // Stage A (registered): request snapshot + boosted effective QoS
    (* keep = "true", dont_touch = "true" *) logic [NUM_MASTERS-1:0]         req_a;
    (* keep = "true", dont_touch = "true" *) logic [NUM_MASTERS*EFF_QW-1:0]  eff_qos_a;

    // Stage B (registered): best QoS and snapshot for mask generation
    (* keep = "true", dont_touch = "true" *) logic [NUM_MASTERS-1:0]         req_b;
    (* keep = "true", dont_touch = "true" *) logic [NUM_MASTERS*EFF_QW-1:0]  eff_qos_b;
    (* keep = "true", dont_touch = "true" *) logic [EFF_QW-1:0]              best_qos_b;
    logic [IDX_W-1:0]               rr_ptr_b;
    logic                           locked_active_b;
    logic [IDX_W-1:0]               locked_idx_b;

    // Balanced max-reduction tree for best QoS to avoid deep compare chains.
    localparam ST1_W = (NUM_MASTERS + 1) / 2;
    localparam ST2_W = (ST1_W + 1) / 2;
    localparam ST3_W = (ST2_W + 1) / 2;
    localparam ST4_W = (ST3_W + 1) / 2;
    localparam ST5_W = (ST4_W + 1) / 2;

    logic [EFF_QW-1:0] best_qos_a_comb;
    logic [EFF_QW-1:0] max_stage0 [0:NUM_MASTERS-1];
    logic [EFF_QW-1:0] max_stage1 [0:ST1_W-1];
    logic [EFF_QW-1:0] max_stage2 [0:ST2_W-1];
    logic [EFF_QW-1:0] max_stage3 [0:ST3_W-1];
    logic [EFF_QW-1:0] max_stage4 [0:ST4_W-1];
    logic [EFF_QW-1:0] max_stage5 [0:ST5_W-1];

    function [EFF_QW-1:0] max2;
        input [EFF_QW-1:0] a_qos;
        input [EFF_QW-1:0] b_qos;
        begin
            if (a_qos >= b_qos)
                max2 = a_qos;
            else
                max2 = b_qos;
        end
    endfunction

    genvar g0;
    genvar g1;
    genvar g2;
    genvar g3;
    genvar g4;
    generate
        for (g0 = 0; g0 < NUM_MASTERS; g0 = g0 + 1) begin : max_stage0_gen
            assign max_stage0[g0] = req_a[g0] ? eff_qos_a[EFF_QW*g0 +: EFF_QW] : {EFF_QW{1'b0}};
        end

        for (g1 = 0; g1 < ST1_W; g1 = g1 + 1) begin : max_stage1_gen
            if (((2*g1) + 1) < NUM_MASTERS)
                assign max_stage1[g1] = max2(max_stage0[2*g1], max_stage0[(2*g1) + 1]);
            else
                assign max_stage1[g1] = max_stage0[2*g1];
        end

        for (g2 = 0; g2 < ST2_W; g2 = g2 + 1) begin : max_stage2_gen
            if (((2*g2) + 1) < ST1_W)
                assign max_stage2[g2] = max2(max_stage1[2*g2], max_stage1[(2*g2) + 1]);
            else
                assign max_stage2[g2] = max_stage1[2*g2];
        end

        for (g3 = 0; g3 < ST3_W; g3 = g3 + 1) begin : max_stage3_gen
            if (((2*g3) + 1) < ST2_W)
                assign max_stage3[g3] = max2(max_stage2[2*g3], max_stage2[(2*g3) + 1]);
            else
                assign max_stage3[g3] = max_stage2[2*g3];
        end

        for (g4 = 0; g4 < ST4_W; g4 = g4 + 1) begin : max_stage4_gen
            if (((2*g4) + 1) < ST3_W)
                assign max_stage4[g4] = max2(max_stage3[2*g4], max_stage3[(2*g4) + 1]);
            else
                assign max_stage4[g4] = max_stage3[2*g4];
        end
    endgenerate

    generate
        if (ST4_W > 1) begin : max_stage5_gen_two
            assign max_stage5[0] = max2(max_stage4[0], max_stage4[1]);
        end else begin : max_stage5_gen_one
            assign max_stage5[0] = max_stage4[0];
        end
    endgenerate
    assign best_qos_a_comb = max_stage5[0];

    // Stage C (combinational): candidate mask from registered best QoS
    logic [NUM_MASTERS-1:0]     s0_req_mask;
    logic [IDX_W-1:0]           s0_rr_ptr;
    logic                       s0_locked_active;
    logic [IDX_W-1:0]           s0_locked_idx;

    // Stage 1 (registered): candidate mask snapshot
    logic [NUM_MASTERS-1:0]     s1_req_mask;
    logic [IDX_W-1:0]           s1_rr_ptr;
    logic                       s1_locked_active;
    logic [IDX_W-1:0]           s1_locked_idx;

    logic [NUM_MASTERS-1:0]              next_grant;
    logic [$clog2(NUM_MASTERS)-1:0]      next_grant_idx;
    logic                                next_grant_valid;

    // Stage C helper: parallel per-master eligibility compare to avoid
    // long chained logic in procedural for-loops.
    logic [NUM_MASTERS-1:0] s0_req_mask_match;
    genvar gm;
    generate
        for (gm = 0; gm < NUM_MASTERS; gm = gm + 1) begin : req_mask_match_gen
            logic [EFF_QW-1:0] eff_qos_b_i;
            assign eff_qos_b_i = eff_qos_b[EFF_QW*gm +: EFF_QW];
            assign s0_req_mask_match[gm] = req_b[gm] & (eff_qos_b_i == best_qos_b);
        end
    endgenerate

    always_ff @(posedge clk) begin
        if (rst) begin
            req_a     <= {NUM_MASTERS{1'b0}};
            eff_qos_a <= {(NUM_MASTERS*EFF_QW){1'b0}};
        end else begin
            req_a <= req;
            for (i = 0; i < NUM_MASTERS; i = i + 1) begin
                if (age[i] >= AGE_BOOST_THRESH)
                    eff_qos_a[EFF_QW*i +: EFF_QW] <= {1'b1, qos_in[QOS_WIDTH*i +: QOS_WIDTH]};
                else
                    eff_qos_a[EFF_QW*i +: EFF_QW] <= {1'b0, qos_in[QOS_WIDTH*i +: QOS_WIDTH]};
            end
        end
    end

    always_ff @(posedge clk) begin
        if (rst) begin
            req_b           <= {NUM_MASTERS{1'b0}};
            eff_qos_b       <= {(NUM_MASTERS*EFF_QW){1'b0}};
            best_qos_b      <= {EFF_QW{1'b0}};
            rr_ptr_b        <= {IDX_W{1'b0}};
            locked_active_b <= 1'b0;
            locked_idx_b    <= {IDX_W{1'b0}};
        end else begin
            req_b           <= req_a;
            eff_qos_b       <= eff_qos_a;
            best_qos_b      <= best_qos_a_comb;
            rr_ptr_b        <= rr_ptr;
            locked_active_b <= locked && req_a[locked_idx];
            locked_idx_b    <= locked_idx;
        end
    end

    always_comb begin
        s0_req_mask      = s0_req_mask_match;
        s0_rr_ptr        = rr_ptr_b;
        s0_locked_active = locked_active_b;
        s0_locked_idx    = locked_idx_b;

        if (s0_locked_active) begin
            // Locked owner always stays in the candidate set.
            s0_req_mask = {NUM_MASTERS{1'b0}};
            s0_req_mask[s0_locked_idx] = 1'b1;
        end
    end

    always_ff @(posedge clk) begin
        if (rst) begin
            s1_req_mask      <= {NUM_MASTERS{1'b0}};
            s1_rr_ptr        <= 0;
            s1_locked_active <= 1'b0;
            s1_locked_idx    <= 0;
        end else begin
            s1_req_mask      <= s0_req_mask;
            s1_rr_ptr        <= s0_rr_ptr;
            s1_locked_active <= s0_locked_active;
            s1_locked_idx    <= s0_locked_idx;
        end
    end

    always_comb begin
        next_grant       = {NUM_MASTERS{1'b0}};
        next_grant_idx   = 0;
        next_grant_valid = 1'b0;

        if (s1_locked_active) begin
            next_grant[s1_locked_idx] = 1'b1;
            next_grant_idx            = s1_locked_idx;
            next_grant_valid       = 1'b1;
        end
        else if (|s1_req_mask) begin
            found = 1'b0;
            for (j = 0; j < NUM_MASTERS; j = j + 1) begin
                if (!found) begin
                    if ((s1_rr_ptr + j) >= NUM_MASTERS)
                        idx = (s1_rr_ptr + j) - NUM_MASTERS;
                    else
                        idx = s1_rr_ptr + j;

                    if (s1_req_mask[idx]) begin
                        next_grant[idx]   = 1'b1;
                        next_grant_idx    = idx;
                        next_grant_valid  = 1'b1;
                        found             = 1'b1;
                    end
                end
            end
        end
    end

    always_ff @(posedge clk) begin
        if (rst) begin
            grant       <= {NUM_MASTERS{1'b0}};
            grant_idx   <= 0;
            grant_valid <= 1'b0;
        end else begin
            grant       <= next_grant;
            grant_idx   <= next_grant_idx;
            grant_valid <= next_grant_valid;
        end
    end

endmodule
