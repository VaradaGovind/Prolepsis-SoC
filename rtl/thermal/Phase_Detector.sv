// ============================================================================
// OrionRV - Workload Phase Detector (Synthesis-Safe)
//
// Classifies each core's workload phase using EMA-filtered HPC data.
// All division replaced with shift-based ratio estimation for timing closure.
//
// Phase classifications:
//   0: COMPUTE_BOUND  - High IPC, low cache miss rate
//   1: MEMORY_BOUND   - Low IPC, high cache miss rate
//   2: BALANCED        - Moderate activity
//   3: IDLE            - Near-zero activity
//
// Update rate: sampled inputs should be delta-counters over a fixed window
// (e.g., 1024 or 4096 cycles), provided by the top-level module.
// ============================================================================

module Phase_Detector (
    input  logic        clk,
    input  logic        rst_n,
    
    // Core HPC deltas (sampled every N cycles by top-level)
    input  logic [31:0] minstret_delta,    // Instructions retired in window
    input  logic [31:0] mcycle_delta,      // Cycles elapsed in window (should be ~N)
    input  logic [31:0] icache_miss,       // I-cache misses in window
    input  logic [31:0] icache_hit,        // I-cache hits in window
    
    // Output Phase Classification
    output logic [1:0]  phase,             // 0: COMPUTE, 1: MEMORY, 2: BALANCED, 3: IDLE
    output logic        phase_changed      // Pulse when classification changes
);

    logic [1:0] current_phase;
    logic [1:0] previous_phase;
    
    // EMA smoothers (Q16.16 format for fractional precision)
    logic [31:0] ema_long_ipc;
    logic [31:0] ema_short_ipc;
    logic [31:0] ema_long_miss;
    logic [31:0] ema_short_miss;
    
    // Hysteresis counter — phase must persist for M samples before change
    logic [3:0]  hysteresis_cnt;
    logic [1:0]  candidate_phase;

    // ---------------------------------------------------------------
    // Shift-based IPC estimation (avoids division):
    //   IPC = minstret / mcycle  (range 0.0 to ~1.0 for in-order)
    //
    // Instead of division, we compare:
    //   minstret vs mcycle >> N to estimate IPC thresholds
    //
    // IPC > 0.5:  minstret > mcycle >> 1  (compute-heavy)
    // IPC < 0.1:  minstret < mcycle >> 3  (nearly idle)
    // ---------------------------------------------------------------
    
    // Cache miss ratio estimation (shift-based):
    //   miss_ratio = miss / (hit + miss)
    //   High miss: miss > (hit + miss) >> 2  (>25% miss rate)
    //   Low miss:  miss < (hit + miss) >> 4  (<6% miss rate)
    logic [31:0] total_cache;
    assign total_cache = icache_hit + icache_miss;
    
    // IPC category signals
    logic ipc_high;
    assign ipc_high = (minstret_delta > (mcycle_delta >> 1));   // IPC > 0.5
    logic ipc_low;                                             // IPC < 0.125
    assign ipc_low = (minstret_delta < (mcycle_delta >> 3));
    logic ipc_idle;                                            // IPC < 0.016
    assign ipc_idle = (minstret_delta < (mcycle_delta >> 6));
    
    // Miss rate category signals
    logic miss_high;                                           // Miss > 25%
    assign miss_high = (icache_miss > (total_cache >> 2));
    logic miss_low;                                            // Miss < 6%
    assign miss_low = (icache_miss < (total_cache >> 4));

    // EMA-filtered versions for trend detection
    // Short EMA alpha ~1/8, Long EMA alpha ~1/64
    logic [31:0] ipc_proxy;                  // Use raw retired count as IPC proxy
    assign ipc_proxy = minstret_delta;
    logic        ema_ipc_high;
    assign ema_ipc_high = (ema_short_ipc  > (mcycle_delta >> 1));
    logic        ema_ipc_low;
    assign ema_ipc_low = (ema_short_ipc  < (mcycle_delta >> 3));
    logic        ema_miss_high;
    assign ema_miss_high = (ema_short_miss > (total_cache >> 2));
    logic        ema_miss_low;
    assign ema_miss_low = (ema_short_miss < (total_cache >> 4));
    logic        ipc_trend_up;
    assign ipc_trend_up = (ema_short_ipc  >= ema_long_ipc);
    logic        miss_trend_up;
    assign miss_trend_up = (ema_short_miss >  ema_long_miss);

    always_ff @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            ema_long_ipc   <= 32'd0;
            ema_short_ipc  <= 32'd0;
            ema_long_miss  <= 32'd0;
            ema_short_miss <= 32'd0;
            current_phase  <= 2'd3; // Start IDLE
            previous_phase <= 2'd3;
            candidate_phase <= 2'd3;
            hysteresis_cnt <= 4'd0;
        end else begin
            // EMA updates
            ema_short_ipc  <= ema_short_ipc  + (($signed(ipc_proxy)     - $signed(ema_short_ipc))  >>> 3);
            ema_long_ipc   <= ema_long_ipc   + (($signed(ipc_proxy)     - $signed(ema_long_ipc))   >>> 6);
            ema_short_miss <= ema_short_miss  + (($signed(icache_miss)   - $signed(ema_short_miss)) >>> 3);
            ema_long_miss  <= ema_long_miss   + (($signed(icache_miss)   - $signed(ema_long_miss))  >>> 6);

            // Determine candidate phase
            if (ipc_idle && (ema_short_ipc < (mcycle_delta >> 5))) begin
                candidate_phase <= 2'd3; // IDLE
            end else if ((ipc_high || ema_ipc_high || ipc_trend_up) &&
                         (miss_low || ema_miss_low) &&
                         !miss_trend_up) begin
                candidate_phase <= 2'd0; // COMPUTE_BOUND
            end else if ((ipc_low || ema_ipc_low) &&
                         (miss_high || ema_miss_high || miss_trend_up)) begin
                candidate_phase <= 2'd1; // MEMORY_BOUND
            end else begin
                candidate_phase <= 2'd2; // BALANCED
            end
            
            // Hysteresis: require 8 consecutive matching samples
            if (candidate_phase == current_phase) begin
                hysteresis_cnt <= 4'd0;
            end else begin
                if (hysteresis_cnt >= 4'd7) begin
                    previous_phase <= current_phase;
                    current_phase  <= candidate_phase;
                    hysteresis_cnt <= 4'd0;
                end else begin
                    hysteresis_cnt <= hysteresis_cnt + 4'd1;
                end
            end
        end
    end
    
    assign phase = current_phase;
    assign phase_changed = (current_phase != previous_phase);

endmodule
