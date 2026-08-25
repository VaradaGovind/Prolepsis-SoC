`timescale 1ns / 1ps
//============================================================================
// OrionRV - Cost-Aware Migration Policy Engine (Phase 5.3)
//
// Chooses source/destination cores based on predicted thermal headroom and
// requests migration only when projected benefit exceeds projected cost.
//============================================================================

module Migration_Policy #(
    parameter NUM_CORES = 5,
    parameter COOLDOWN_CYCLES = 32'd10000,
    parameter [31:0] HEADROOM_IMBALANCE_THRESH = 32'h0004_0000
)(
    input  logic                  clk,
    input  logic                  rst_n,
    input  logic                  enable,

    input  logic [NUM_CORES*32-1:0] t_predicted_flat,
    input  logic [NUM_CORES*2-1:0]  phase_flat,
    input  logic [NUM_CORES-1:0]    phase_changed_flat,
    input  logic [NUM_CORES*32-1:0] icache_miss_flat,

    input  logic [31:0]           t_warn,
    input  logic [31:0]           t_crit,
    input  logic [31:0]           cache_warm_cycles,
    input  logic                  migration_busy,

    output logic                   migrate_req,
    output logic  [2:0]            src_core,
    output logic  [2:0]            dst_core,
    output logic  [31:0]           decision_cost_cycles,
    output logic  [31:0]           decision_benefit_score,
    output logic                   all_cores_hot,
    output logic                   dual_hot_mode,
    output logic                   imbalance_block,
    output logic  [NUM_CORES*32-1:0] migration_hist_count_flat,
    output logic [255:0]          recent_history_flat
);

    integer i;

    logic [31:0] t_pred     [0:NUM_CORES-1];
    logic [1:0]  phase_c    [0:NUM_CORES-1];
    logic [31:0] miss_c     [0:NUM_CORES-1];
    logic [31:0] headroom   [0:NUM_CORES-1];
    logic [NUM_CORES-1:0] hot_mask;

    logic [2:0] src_sel_next;
    logic [2:0] dst_sel_next;
    logic [31:0] src_temp_next;
    logic [31:0] dst_temp_next;
    logic [31:0] dst_headroom_next;
    logic [1:0]  src_phase_next;
    logic        src_phase_changed_next;
    logic [31:0] src_miss_next;

    logic [31:0] benefit_next;
    logic [31:0] cost_next;
    logic        proactive_next;
    logic        all_hot_next;
    logic [2:0]  hot_count_next;
    logic [31:0] max_headroom_next;
    logic [31:0] min_headroom_next;
    logic        dual_hot_next;
    logic        imbalance_ok_next;

    logic [31:0] cooldown_ctr;
    logic [2:0]  src_sel_q;
    logic [2:0]  dst_sel_q;
    logic [31:0] benefit_q;
    logic [31:0] cost_q;
    logic        proactive_q;
    logic        all_hot_q;
    logic        dual_hot_q;
    logic        imbalance_ok_q;
    
    // Pipeline stage to break critical path: register intermediate temp_delta computation
    logic [31:0] src_temp_p1;
    logic [31:0] dst_temp_p1;
    logic [1:0]  src_phase_p1;
    logic        src_phase_changed_p1;
    logic [31:0] src_miss_p1;
    logic        proactive_p1;
    logic [31:0] temp_delta_p1;
    logic [31:0] phase_bonus_p1;
    logic [31:0] miss_penalty_p1;

    logic [31:0] migration_hist_count [0:NUM_CORES-1];
    integer h;

    logic [31:0] temp_delta;
    logic [31:0] phase_bonus;
    logic [31:0] miss_penalty;

    logic [63:0] history_ring [0:15];
    logic [3:0]  history_ptr;
    logic [31:0] global_cycle;

    logic [3:0] ptr_m1;
    assign ptr_m1 = history_ptr - 4'd1;
    logic [3:0] ptr_m2;
    assign ptr_m2 = history_ptr - 4'd2;
    logic [3:0] ptr_m3;
    assign ptr_m3 = history_ptr - 4'd3;
    logic [3:0] ptr_m4;
    assign ptr_m4 = history_ptr - 4'd4;
    
    assign recent_history_flat = {
        history_ring[ptr_m4],
        history_ring[ptr_m3],
        history_ring[ptr_m2],
        history_ring[ptr_m1]
    };

    always_comb begin
        for (i = 0; i < NUM_CORES; i = i + 1) begin
            t_pred[i]   = t_predicted_flat[(i*32) +: 32];
            phase_c[i]  = phase_flat[(i*2) +: 2];
            miss_c[i]   = icache_miss_flat[(i*32) +: 32];
            headroom[i] = (t_crit > t_pred[i]) ? (t_crit - t_pred[i]) : 32'd0;
            hot_mask[i] = (t_pred[i] > t_warn);
        end
    end

    always_comb begin
        src_sel_next          = 3'd0;
        dst_sel_next          = 3'd0;
        src_temp_next         = t_pred[0];
        dst_temp_next         = t_pred[0];
        dst_headroom_next     = 32'd0;
        src_phase_next        = phase_c[0];
        src_phase_changed_next = phase_changed_flat[0];
        src_miss_next         = miss_c[0];

        for (i = 1; i < NUM_CORES; i = i + 1) begin
            if (t_pred[i] > src_temp_next) begin
                src_temp_next          = t_pred[i];
                src_sel_next           = i[2:0];
                src_phase_next         = phase_c[i];
                src_phase_changed_next = phase_changed_flat[i];
                src_miss_next          = miss_c[i];
            end
        end

        for (i = 0; i < NUM_CORES; i = i + 1) begin
            if ((i[2:0] != src_sel_next) &&
                ((dst_sel_next == src_sel_next) || (headroom[i] > dst_headroom_next))) begin
                dst_headroom_next = headroom[i];
                dst_sel_next      = i[2:0];
                dst_temp_next     = t_pred[i];
            end
        end

        all_hot_next = &hot_mask;
        hot_count_next = 3'd0;
        max_headroom_next = 32'd0;
        min_headroom_next = 32'hFFFF_FFFF;

        for (i = 0; i < NUM_CORES; i = i + 1) begin
            if (hot_mask[i])
                hot_count_next = hot_count_next + 3'd1;
            if (headroom[i] > max_headroom_next)
                max_headroom_next = headroom[i];
            if (headroom[i] < min_headroom_next)
                min_headroom_next = headroom[i];
        end

        dual_hot_next = (hot_count_next >= 3'd2);
        imbalance_ok_next = (max_headroom_next > (min_headroom_next + HEADROOM_IMBALANCE_THRESH));

        temp_delta = (src_temp_next > dst_temp_next) ?
                     (src_temp_next - dst_temp_next) : 32'd0;

        proactive_next = src_phase_changed_next && (src_phase_next == 2'd0);
        phase_bonus    = proactive_next ? 32'd64 : 32'd0;
        miss_penalty   = src_miss_next >> 2;

        // Q16.16 temperature delta is scaled down to cycle-like score.
        // Register intermediate values to break critical path
        benefit_next = (temp_delta >> 12) + phase_bonus;
        cost_next    = 32'd40 + cache_warm_cycles + miss_penalty;
    end

    always_ff @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            // Pipeline stage 1 resets
            src_temp_p1 <= 32'd0;
            dst_temp_p1 <= 32'd0;
            src_phase_p1 <= 2'd0;
            src_phase_changed_p1 <= 1'b0;
            src_miss_p1 <= 32'd0;
            proactive_p1 <= 1'b0;
            temp_delta_p1 <= 32'd0;
            phase_bonus_p1 <= 32'd0;
            miss_penalty_p1 <= 32'd0;
            
            migrate_req          <= 1'b0;
            cooldown_ctr         <= 32'd0;
            decision_cost_cycles <= 32'd0;
            decision_benefit_score <= 32'd0;
            all_cores_hot        <= 1'b0;
            dual_hot_mode        <= 1'b0;
            imbalance_block      <= 1'b0;
            history_ptr          <= 4'd0;
            global_cycle         <= 32'd0;
            src_core             <= 3'd0;
            dst_core             <= 3'd1;
            src_sel_q            <= 3'd0;
            dst_sel_q            <= 3'd1;
            benefit_q            <= 32'd0;
            cost_q               <= 32'd0;
            proactive_q          <= 1'b0;
            all_hot_q            <= 1'b0;
            dual_hot_q           <= 1'b0;
            imbalance_ok_q       <= 1'b0;
            migration_hist_count_flat <= {(NUM_CORES*32){1'b0}};
            for (h = 0; h < NUM_CORES; h = h + 1) begin
                migration_hist_count[h] <= 32'd0;
            end
            for (h = 0; h < 16; h = h + 1) begin
                history_ring[h] <= 64'd0;
            end
        end else if (enable) begin
            // Pipeline stage 1: register intermediate values to break critical path
            src_temp_p1 <= src_temp_next;
            dst_temp_p1 <= dst_temp_next;
            src_phase_p1 <= src_phase_next;
            src_phase_changed_p1 <= src_phase_changed_next;
            src_miss_p1 <= src_miss_next;
            proactive_p1 <= proactive_next;
            temp_delta_p1 <= temp_delta;
            phase_bonus_p1 <= phase_bonus;
            miss_penalty_p1 <= miss_penalty;
            
            global_cycle <= global_cycle + 32'd1;
            src_sel_q   <= src_sel_next;
            dst_sel_q   <= dst_sel_next;
            
            // Use pipelined values for benefit/cost computation
            benefit_q   <= (temp_delta_p1 >> 12) + phase_bonus_p1;
            cost_q      <= 32'd40 + cache_warm_cycles + miss_penalty_p1;
            
            proactive_q <= proactive_p1;
            all_hot_q   <= all_hot_next;
            dual_hot_q  <= dual_hot_next;
            imbalance_ok_q <= imbalance_ok_next;

            if (cooldown_ctr > 0) begin
                cooldown_ctr <= cooldown_ctr - 32'd1;
                migrate_req  <= 1'b0;
            end else if (proactive_q && benefit_q > cost_q && !all_hot_q && (!dual_hot_q || imbalance_ok_q) && !migration_busy) begin
                migrate_req          <= 1'b1;
                src_core             <= src_sel_q;
                dst_core             <= dst_sel_q;
                decision_cost_cycles <= cost_q;
                decision_benefit_score <= benefit_q;
                all_cores_hot        <= all_hot_q;
                dual_hot_mode        <= dual_hot_q;
                imbalance_block      <= (dual_hot_q && !imbalance_ok_q);
                
                migration_hist_count[src_sel_q] <= migration_hist_count[src_sel_q] + 32'd1;
                if (dst_sel_q != src_sel_q)
                    migration_hist_count[dst_sel_q] <= migration_hist_count[dst_sel_q] + 32'd1;
                
                history_ring[history_ptr] <= {global_cycle, src_sel_q, dst_sel_q, 26'd0};
                history_ptr <= history_ptr + 4'd1;
                
                cooldown_ctr <= COOLDOWN_CYCLES;
            end else begin
                migrate_req <= 1'b0;
                all_cores_hot <= all_hot_q;
                dual_hot_mode <= dual_hot_q;
                imbalance_block <= (dual_hot_q && !imbalance_ok_q);
            end

            for (h = 0; h < NUM_CORES; h = h + 1)
                migration_hist_count_flat[(h*32) +: 32] <= migration_hist_count[h];
        end
    end

endmodule
