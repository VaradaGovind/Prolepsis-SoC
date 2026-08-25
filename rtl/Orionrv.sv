`timescale 1ns / 1ps
//============================================================================
// OrionRV - Top-Level SoC (Hybrid 4+1 Core RISC-V)
//
// Current integration: Phase 1-4 (Pipeline + Cache + AXI + Thermal)
//
// Changes from previous version:
//   - UART and Timer sub-decoded within MMIO (0x9000_xxxx)
//   - 7-segment display driver for real-time thermal dashboard
//   - Phase Detector connected to real core activity signals
//   - Power proxy is now proportional (not binary)
//   - Thermal thresholds lowered for testing (45°C/50°C)
//   - Saturation guards and synthesis fixes throughout
//   - Hang detection and error code display
//============================================================================

module orionrv (
    input  logic        clk,
    input  logic        rst,

    // -------------------------------------------------------------------
    // External I/O
    // -------------------------------------------------------------------
    input  logic [3:0]  buttons,
    output logic [7:0]  leds,
    
    // Slide switches for core temperature selection
    input  logic [1:0]  sw,

    // -------------------------------------------------------------------
    // 7-Segment Display (Nexys 4 DDR)
    // -------------------------------------------------------------------
    output logic [6:0]  seg,
    output logic        dp,
    output logic [7:0]  an,

    // -------------------------------------------------------------------
    // VGA output
    // -------------------------------------------------------------------
    output logic [3:0]  vga_r,
    output logic [3:0]  vga_g,
    output logic [3:0]  vga_b,
    output logic        vga_hsync,
    output logic        vga_vsync,

    // -------------------------------------------------------------------
    // UART
    // -------------------------------------------------------------------
    input  logic        uart_rx,
    output logic        uart_tx
);

    // ===================================================================
    // Parameters
    // ===================================================================
    localparam NUM_PCORES  = 4;
    localparam NUM_ECORES  = 1;
    localparam TOTAL_CORES = NUM_PCORES + NUM_ECORES;  // 5
    localparam NUM_MASTERS = TOTAL_CORES * 2 + 1;      // 11 (5 I + 5 D + 1 MAC)
    localparam NUM_SLAVES  = axi_pkg::SLAVE_COUNT;              // 5

    // ===================================================================
    // Reset Synchronizer (BUG-06 fix)
    // Nexys 4 DDR CPU_RESETN (C12) is active-low. We synchronize the
    // deassertion through a 2-stage FF chain to prevent metastability.
    // Active-high synchronized reset drives local reset distribution flops.
    // ===================================================================
    logic [1:0] rst_sync;
    always_ff @(posedge clk or negedge rst) begin
        if (!rst)
            rst_sync <= 2'b11;          // Assert immediately on button press
        else
            rst_sync <= {rst_sync[0], 1'b0}; // Deassert synchronously
    end
    logic sys_rst;
    assign sys_rst = rst_sync[1];       // Active-high, synchronized

    // Stage reset distribution to reduce high-fanout reset route depth.
    logic [TOTAL_CORES-1:0] core_rst_r;
    logic                   bus_rst_r;
    logic [TOTAL_CORES-1:0] core_rst_n;
    assign core_rst_n = ~core_rst_r;
    logic                   bus_rst_n;
    assign bus_rst_n = ~bus_rst_r;
    always_ff @(posedge clk) begin
        if (sys_rst) begin
            core_rst_r <= {TOTAL_CORES{1'b1}};
            bus_rst_r  <= 1'b1;
        end else begin
            core_rst_r <= {TOTAL_CORES{1'b0}};
            bus_rst_r  <= 1'b0;
        end
    end

    // ===================================================================
    // VGA debug stubs (VGA framebuffer pipeline not yet implemented)
    // Keep these output-driven from live inputs so synthesis does not trim
    // VGA pins as constants while preserving harmless bring-up behavior.
    // ===================================================================
    assign vga_r     = buttons[3:0];
    assign vga_g     = {buttons[3], buttons[2], sw[1:0]};
    assign vga_b     = {uart_rx, rst, sw[1:0]};
    assign vga_hsync = buttons[0] | sw[0];
    assign vga_vsync = buttons[1] | sw[1];

    // ===================================================================
    // Thermal Thresholds (Q16.16 format)
    // Use realistic on-board limits so thermal status does not pin to
    // critical under normal bench bring-up.
    // t_warn = 85°C = 0x0055_0000
    // t_crit = 90°C = 0x005A_0000
    // t_l1   = t_crit - dt_max = 90 - 3 = 87°C = 0x0057_0000
    // t_amb  = 25°C = 0x0019_0000
    // ===================================================================
    localparam [31:0] THERMAL_WARN = 32'h0055_0000;  // 85°C
    localparam [31:0] THERMAL_CRIT = 32'h005A_0000;  // 90°C
    localparam [31:0] THERMAL_L1   = 32'h0057_0000;  // 87°C (safety cap)
    localparam [31:0] THERMAL_AMB  = 32'h0019_0000;  // 25°C
    localparam [31:0] THERMAL_DMAX = 32'h0003_0000;  // 3°C max delta

    // ===================================================================
    // Thermal System Wires
    // ===================================================================
    logic [TOTAL_CORES-1:0] core_clk_en;
    logic  [TOTAL_CORES-1:0] core_enable;
    logic [TOTAL_CORES-1:0] core_active;
    logic [TOTAL_CORES-1:0] core_retired;

    // Phase 5 migration control signals
    logic [TOTAL_CORES-1:0] mig_we_vec;
    logic [TOTAL_CORES*5-1:0] mig_addr_flat;
    logic [TOTAL_CORES*32-1:0] mig_wdata_flat;
    logic [TOTAL_CORES*32-1:0] mig_rdata_flat;
    logic [TOTAL_CORES-1:0] mig_csr_we_vec;
    logic [TOTAL_CORES*4-1:0] mig_csr_addr_flat;
    logic [TOTAL_CORES*32-1:0] mig_csr_wdata_flat;
    logic [TOTAL_CORES*32-1:0] mig_csr_rdata_flat;
    logic [TOTAL_CORES*32-1:0] core_mig_csr_rdata_flat;
    logic [TOTAL_CORES*32-1:0] pc_save_flat;
    logic [TOTAL_CORES*32-1:0] pc_load_flat;
    logic [TOTAL_CORES-1:0] pc_load_en_vec;
    logic [TOTAL_CORES-1:0] mig_pause_mask;
    logic [TOTAL_CORES-1:0] mig_halt_mask;
    
    logic [TOTAL_CORES-1:0] pg_save_req;
    logic [TOTAL_CORES-1:0] pg_restore_req;
    logic [TOTAL_CORES-1:0] pg_ack;
    logic  [TOTAL_CORES-1:0] migration_event_core;
    logic migration_busy;
    logic migration_done;
    logic migration_error;
    logic [31:0] migration_last_cycles;
    logic [31:0] migration_count;
    logic [2:0] migration_active_src;
    logic [2:0] migration_active_dst;
    logic  migration_start_pulse;
    logic  migration_clear_halt_pulse;
    logic  [2:0] migration_src_sel;
    logic  [2:0] migration_dst_sel;

    // Phase 5.3 policy / 5.4 power gate / 5.5 cache warm / 5.6 eval
    logic policy_migrate_req;
    logic [2:0] policy_src_core;
    logic [2:0] policy_dst_core;
    logic [31:0] policy_cost_cycles;
    logic [31:0] policy_benefit_score;
    logic policy_all_cores_hot;
    logic policy_dual_hot_mode;
    logic policy_imbalance_block;
    logic [TOTAL_CORES*32-1:0] policy_migration_hist_flat;
    logic [255:0] policy_recent_history_flat;

    logic cache_warm_start;
    logic cache_warm_busy;
    logic cache_warm_done;
    logic [31:0] cache_warm_cycles;
    logic [31:0] cache_prefetch_count;
    logic [31:0] cache_prefetch_addr;
    logic cache_prefetch_valid;

    logic [TOTAL_CORES*20-1:0] icache_tag_flat;
    logic [TOTAL_CORES-1:0]    icache_valid_vec;
    logic [7:0]                cache_warmer_inspect_idx;

    logic [TOTAL_CORES-1:0] power_gate_mask;
    logic [TOTAL_CORES*32-1:0] pwr_gated_cycles_flat;
    logic [31:0] total_power_gate_cycles;
    logic [31:0] power_gate_events;

    logic  eval_mode_we;
    logic  [1:0] eval_mode_wdata;
    logic  eval_run_done_pulse;
    logic  [3:0] buttons_meta;
    logic  [3:0] buttons_sync;
    logic  [3:0] buttons_sync_d;
    logic        dpad_mode_we;
    logic  [1:0] dpad_mode_wdata;
    logic [1:0] eval_active_mode;
    logic eval_metrics_frozen;
    logic [31:0] eval_total_cycles;
    logic [31:0] eval_total_retired;
    logic [31:0] eval_peak_temperature;
    logic [31:0] eval_avg_temperature;
    logic [31:0] eval_throttle_events;
    logic [31:0] eval_throttle_cycles;
    logic [31:0] eval_migration_count;
    logic [31:0] eval_migration_cycles;
    logic [31:0] eval_power_gate_cycles;
    logic [31:0] eval_thermal_violations;
    logic [31:0] eval_energy_estimate;

    // Thermal sensor outputs (Q16.16 format)
    logic [31:0] t_sensor [0:TOTAL_CORES-1];
    logic [31:0] t_sensor_fused [0:TOTAL_CORES-1];

    // Phase 7.5 branch keeps external XADC fusion disabled.
    // MMIO telemetry addresses remain stable and read zero/invalid.
    logic [31:0] xadc_temp_q16_16;
    assign xadc_temp_q16_16 = 32'd0;
    logic        xadc_temp_valid;
    assign xadc_temp_valid = 1'b0;
    logic [15:0] xadc_temp_raw;
    assign xadc_temp_raw = 16'd0;

    // Kalman predictor outputs
    logic [31:0] t_estimated [0:TOTAL_CORES-1];
    logic [31:0] t_predicted [0:TOTAL_CORES-1];
    logic [31:0] p_uncertainty [0:TOTAL_CORES-1];
    logic [1:0]  thermal_state [0:TOTAL_CORES-1];
    logic [31:0] innovation [0:TOTAL_CORES-1];

    // Phase detector outputs
    logic [1:0]  phase [0:TOTAL_CORES-1];
    logic        phase_changed [0:TOTAL_CORES-1];

    // QoS adjuster outputs
    logic [2:0]  adjusted_qos [0:TOTAL_CORES-1];
    logic [2:0]  default_qos [0:TOTAL_CORES-1];

    // Thermal controller outputs
    logic [TOTAL_CORES-1:0] controller_clk_en;
    logic [31:0] power_budget [0:TOTAL_CORES-1];
    logic [TOTAL_CORES-1:0] thermal_irq;
    logic timer_irq_global;

    // Register thermal signals at the core boundary to cut long thermal
    // sensor/controller to core CSR timing paths.
    logic [TOTAL_CORES-1:0] thermal_irq_core_r;
    logic [31:0] thermal_level_core_r [0:TOTAL_CORES-1];
    logic [31:0] thermal_pred_core_r [0:TOTAL_CORES-1];
    integer thermal_sync_i;

    always_ff @(posedge clk) begin
        if (bus_rst_r) begin
            thermal_irq_core_r <= {TOTAL_CORES{1'b0}};
            buttons_meta <= 4'b0000;
            buttons_sync <= 4'b0000;
            buttons_sync_d <= 4'b0000;
            for (thermal_sync_i = 0; thermal_sync_i < TOTAL_CORES; thermal_sync_i = thermal_sync_i + 1) begin
                thermal_level_core_r[thermal_sync_i] <= THERMAL_AMB;
                thermal_pred_core_r[thermal_sync_i]  <= THERMAL_AMB;
            end
        end else begin
            thermal_irq_core_r <= thermal_irq;
            buttons_meta <= buttons;
            buttons_sync <= buttons_meta;
            buttons_sync_d <= buttons_sync;
            for (thermal_sync_i = 0; thermal_sync_i < TOTAL_CORES; thermal_sync_i = thermal_sync_i + 1) begin
                thermal_level_core_r[thermal_sync_i] <= t_sensor_fused[thermal_sync_i];
                thermal_pred_core_r[thermal_sync_i]  <= t_predicted[thermal_sync_i];
            end
        end
    end

    logic [3:0] buttons_rise;
    assign buttons_rise = buttons_sync & ~buttons_sync_d;

    always_comb begin
        dpad_mode_we = 1'b0;
        dpad_mode_wdata = 2'd3;

        // D-pad mode map (Nexys buttons in this branch):
        //   BTNC -> mode 0 (Baseline)
        //   BTNU -> mode 1 (Reactive)
        //   BTNL -> mode 2 (Predictive/Kalman)
        //   BTNR -> mode 3 (Full system)
        if (buttons_rise[0]) begin
            dpad_mode_we = 1'b1;
            dpad_mode_wdata = 2'd0;
        end else if (buttons_rise[1]) begin
            dpad_mode_we = 1'b1;
            dpad_mode_wdata = 2'd1;
        end else if (buttons_rise[2]) begin
            dpad_mode_we = 1'b1;
            dpad_mode_wdata = 2'd2;
        end else if (buttons_rise[3]) begin
            dpad_mode_we = 1'b1;
            dpad_mode_wdata = 2'd3;
        end
    end

    // Power proxy (proportional, based on activity counter)
    logic [31:0] power_proxy [0:TOTAL_CORES-1];
    logic  [31:0] power_proxy_r [0:TOTAL_CORES-1];

    // Phase 7.5 fusion path: modeled per-core thermal values only.
    genvar tf;
    generate
        for (tf = 0; tf < TOTAL_CORES; tf = tf + 1) begin : thermal_fuse_gen
            assign t_sensor_fused[tf] = t_sensor[tf];
        end
    endgenerate

    // Keep cores ungated briefly after reset (or until first retirement)
    // so boot cannot be blocked by early thermal-control transients.
    logic [23:0] core_startup_force;
    logic        any_core_retired_seen;
    logic [23:0] multicore_release_cnt;
    logic        multicore_released;
    always_ff @(posedge clk) begin
        if (bus_rst_r) begin
            core_startup_force   <= 24'hFFFFFF;
            any_core_retired_seen <= 1'b0;
            multicore_release_cnt <= 24'h3FFFFF;
            multicore_released    <= 1'b0;
            core_enable           <= {{(TOTAL_CORES-1){1'b0}}, 1'b1};
        end else begin
            if (|core_retired)
                any_core_retired_seen <= 1'b1;

            if (!multicore_released) begin
                if (core_retired[0] || (multicore_release_cnt == 24'd0))
                    multicore_released <= 1'b1;
                else
                    multicore_release_cnt <= multicore_release_cnt - 24'd1;
            end

            core_enable <= multicore_released ? {TOTAL_CORES{1'b1}} :
                                               {{(TOTAL_CORES-1){1'b0}}, 1'b1};

            if (!any_core_retired_seen && (core_startup_force != 24'd0))
                core_startup_force <= core_startup_force - 24'd1;
        end
    end

    logic force_core_clocks;
    assign force_core_clocks = (!any_core_retired_seen) && (core_startup_force != 24'd0);
    assign core_clk_en = force_core_clocks ? {TOTAL_CORES{1'b1}} : controller_clk_en;
    logic [TOTAL_CORES-1:0] core_enable_runtime;
    assign core_enable_runtime = core_enable & ~mig_pause_mask & ~mig_halt_mask & ~power_gate_mask;
    
    // Per-core thermal reading for CSR (selected core 0 for backward compat)
    logic [31:0] thermal_reading;
    assign thermal_reading = thermal_level_core_r[0];

    // ===================================================================
    // Per-core Activity Counters (for Phase Detector & Power Proxy)
    // ===================================================================
    // Accumulate core_active and core_retired signals over a sampling window.
    // Window = 2^12 = 4096 cycles. Delta counters reset each window.
    logic [11:0] sample_counter;
    logic       sample_tick;
    assign sample_tick = (sample_counter == 12'hFFF);
    
    always_ff @(posedge clk) begin
        if (bus_rst_r)
            sample_counter <= 12'd0;
        else
            sample_counter <= sample_counter + 12'd1;
    end
    
    // Per-core delta counters
    logic [31:0] retired_delta [0:TOTAL_CORES-1];
    logic [31:0] active_delta  [0:TOTAL_CORES-1];
    logic [31:0] retired_snap  [0:TOTAL_CORES-1]; // Snapshot for Phase Detector
    logic [31:0] active_snap   [0:TOTAL_CORES-1];
    
    genvar ai;
    generate
        for (ai = 0; ai < TOTAL_CORES; ai = ai + 1) begin : activity_counters
            // Proportional power proxy: active cycles as fraction of window (Q16.16)
            // power = (active_delta / 4096) in Q16.16 = active_delta << 4
            // Saturate before shift to avoid overflow wrapping in long activity bursts.
            logic [31:0] active_for_power;
            assign active_for_power = (|active_snap[ai][31:28]) ? 32'h0FFF_FFFF : active_snap[ai];

            logic write_active;
            assign write_active = mig_csr_we_vec[ai] && (mig_csr_addr_flat[(ai*4)+:4] == 4'd13);
            logic write_retired;
            assign write_retired = mig_csr_we_vec[ai] && (mig_csr_addr_flat[(ai*4)+:4] == 4'd14);

            always_ff @(posedge clk) begin
                if (core_rst_r[ai]) begin
                    retired_delta[ai] <= 32'd0;
                    active_delta[ai]  <= 32'd0;
                    retired_snap[ai]  <= 32'd0;
                    active_snap[ai]   <= 32'd0;
                    power_proxy_r[ai] <= 32'd0;
                end else begin

                    if (write_active) active_snap[ai] <= mig_csr_wdata_flat[(ai*32)+:32];
                    else if (sample_tick) active_snap[ai] <= active_delta[ai];

                    if (write_retired) retired_snap[ai] <= mig_csr_wdata_flat[(ai*32)+:32];
                    else if (sample_tick) retired_snap[ai] <= retired_delta[ai];

                    if (sample_tick) begin
                        retired_delta[ai] <= {31'd0, core_retired[ai]};
                        active_delta[ai]  <= {31'd0, core_active[ai]};
                    end else begin
                        retired_delta[ai] <= retired_delta[ai] + {31'd0, core_retired[ai]};
                        active_delta[ai]  <= active_delta[ai]  + {31'd0, core_active[ai]};
                    end

                    // Register the proxy before feeding thermal DSP pipelines.
                    power_proxy_r[ai] <= {active_for_power[27:0], 4'b0};
                end
            end

            assign power_proxy[ai] = power_proxy_r[ai];
        end
    endgenerate

    // ===================================================================
    // AXI Interconnect wires
    // ===================================================================
    logic [NUM_MASTERS*32-1:0]  axi_m_awaddr;
    logic [NUM_MASTERS*32-1:0]  axi_m_wdata;
    logic [NUM_MASTERS*4-1:0]   axi_m_wstrb;
    logic [NUM_MASTERS-1:0]     axi_m_awvalid;
    logic [NUM_MASTERS-1:0]     axi_m_awready;
    logic [NUM_MASTERS*2-1:0]   axi_m_bresp;
    logic [NUM_MASTERS-1:0]     axi_m_bvalid;
    logic [NUM_MASTERS-1:0]     axi_m_bready;

    logic [NUM_MASTERS*32-1:0]  axi_m_araddr;
    logic [NUM_MASTERS-1:0]     axi_m_arvalid;
    logic [NUM_MASTERS-1:0]     axi_m_arready;
    logic [NUM_MASTERS*32-1:0]  axi_m_rdata;
    logic [NUM_MASTERS*2-1:0]   axi_m_rresp;
    logic [NUM_MASTERS-1:0]     axi_m_rvalid;
    logic [NUM_MASTERS-1:0]     axi_m_rready;

    logic [NUM_MASTERS*3-1:0]   axi_m_qos;
    logic [NUM_MASTERS-1:0]     axi_m_lock;

    // Slave-side buses (flat packed)
    logic [NUM_SLAVES*32-1:0]   axi_s_awaddr;
    logic [NUM_SLAVES*32-1:0]   axi_s_wdata;
    logic [NUM_SLAVES*4-1:0]    axi_s_wstrb;
    logic [NUM_SLAVES-1:0]      axi_s_awvalid;
    logic [NUM_SLAVES-1:0]      axi_s_wvalid;
    logic [NUM_SLAVES-1:0]      axi_s_awready;
    logic [NUM_SLAVES*2-1:0]    axi_s_bresp;
    logic [NUM_SLAVES-1:0]      axi_s_bvalid;
    logic [NUM_SLAVES-1:0]      axi_s_bready;

    logic [NUM_SLAVES*32-1:0]   axi_s_araddr;
    logic [NUM_SLAVES-1:0]      axi_s_arvalid;
    logic [NUM_SLAVES-1:0]      axi_s_arready;
    logic [NUM_SLAVES*32-1:0]   axi_s_rdata;
    logic [NUM_SLAVES*2-1:0]    axi_s_rresp;
    logic [NUM_SLAVES-1:0]      axi_s_rvalid;
    logic [NUM_SLAVES-1:0]      axi_s_rready;


    // ===================================================================
    // Per-core internal wires
    // ===================================================================
    logic [31:0] imem_addr  [0:TOTAL_CORES-1];
    logic [31:0] imem_rdata [0:TOTAL_CORES-1];
    logic        imem_req   [0:TOTAL_CORES-1];
    logic        imem_ready [0:TOTAL_CORES-1];

    logic [31:0] icache_mem_addr  [0:TOTAL_CORES-1];
    logic [31:0] icache_mem_rdata [0:TOTAL_CORES-1];
    logic        icache_mem_req   [0:TOTAL_CORES-1];
    logic        icache_mem_ready [0:TOTAL_CORES-1];

    logic [31:0] dmem_addr  [0:TOTAL_CORES-1];
    logic [31:0] dmem_wdata [0:TOTAL_CORES-1];
    logic [31:0] dmem_rdata [0:TOTAL_CORES-1];
    logic [3:0]  dmem_wstrb [0:TOTAL_CORES-1];
    logic        dmem_we    [0:TOTAL_CORES-1];
    logic        dmem_req   [0:TOTAL_CORES-1];
    logic        dmem_ready [0:TOTAL_CORES-1];

    logic [31:0] dcache_mem_addr  [0:TOTAL_CORES-1];
    logic [31:0] dcache_mem_wdata [0:TOTAL_CORES-1];
    logic [31:0] dcache_mem_rdata [0:TOTAL_CORES-1];
    logic [3:0]  dcache_mem_wstrb [0:TOTAL_CORES-1];
    logic        dcache_mem_we    [0:TOTAL_CORES-1];
    logic        dcache_mem_req   [0:TOTAL_CORES-1];
    logic        dcache_mem_ready [0:TOTAL_CORES-1];

    // Snoop bus: broadcast committed stores so LR/SC reservations in
    // other cores can be invalidated.
    logic  [31:0] snoop_addr_reg;
    logic         snoop_we_reg;
    logic  [31:0] snoop_addr_next;
    logic         snoop_we_next;
    logic  [2:0]  snoop_src_reg;
    logic  [2:0]  snoop_src_next;
    integer     snoop_i;

    always_comb begin
        snoop_addr_next = snoop_addr_reg;
        snoop_we_next   = 1'b0;
        snoop_src_next  = snoop_src_reg;

        for (snoop_i = 0; snoop_i < TOTAL_CORES; snoop_i = snoop_i + 1) begin
            if (!snoop_we_next && dmem_req[snoop_i] && dmem_we[snoop_i] && dmem_ready[snoop_i]) begin
                snoop_we_next   = 1'b1;
                snoop_addr_next = {dmem_addr[snoop_i][31:2], 2'b00};
                snoop_src_next  = snoop_i[2:0];
            end
        end
    end

    always_ff @(posedge clk) begin
        if (bus_rst_r) begin
            snoop_addr_reg <= 32'b0;
            snoop_we_reg   <= 1'b0;
            snoop_src_reg  <= 3'b000;
        end else begin
            snoop_addr_reg <= snoop_addr_next;
            snoop_we_reg   <= snoop_we_next;
            snoop_src_reg  <= snoop_src_next;
        end
    end

    logic [31:0] snoop_addr;
    assign snoop_addr = snoop_addr_reg;
    logic        snoop_we;
    assign snoop_we = snoop_we_reg;
    logic [2:0]  snoop_src;
    assign snoop_src = snoop_src_reg;


    // ===================================================================
    // I-cache miss counters (sampled over the same 4096-cycle window)
    // ===================================================================
    logic [31:0] icache_miss_delta [0:TOTAL_CORES-1];
    logic [31:0] icache_miss_snap  [0:TOTAL_CORES-1];


    // ===================================================================
    // CORE + I-CACHE + AXI MASTER WRAPPER GENERATION
    // ===================================================================
    genvar ci;
    generate
        for (ci = 0; ci < TOTAL_CORES; ci = ci + 1) begin : core_gen

            localparam IS_ECORE = (ci >= NUM_PCORES);
            localparam [2:0] IFETCH_QOS = IS_ECORE ? axi_pkg::QOS_ECORE_IFETCH : axi_pkg::QOS_PCORE_IFETCH;
            localparam [2:0] DATA_QOS   = IS_ECORE ? axi_pkg::QOS_ECORE_DATA   : axi_pkg::QOS_PCORE_DATA;
            localparam M_IDX_I = ci;
            localparam M_IDX_D = TOTAL_CORES + ci;

            assign default_qos[ci] = DATA_QOS;

            logic core_if_req;
            assign core_if_req = core_enable_runtime[ci] ? imem_req[ci] : 1'b0;
            logic core_d_req;
            assign core_d_req = core_enable_runtime[ci] ? dmem_req[ci] : 1'b0;
            logic core_d_we;
            assign core_d_we = core_enable_runtime[ci] ? dmem_we[ci] : 1'b0;
            logic [3:0] core_d_wstrb;
            assign core_d_wstrb = core_enable_runtime[ci] ? dmem_wstrb[ci] : 4'b0000;

            logic [3:0] current_csr_addr;
            assign current_csr_addr = mig_csr_addr_flat[(ci*4) +: 4];
            assign mig_csr_rdata_flat[(ci*32) +: 32] = 
                (current_csr_addr == 4'd13) ? active_snap[ci] :
                (current_csr_addr == 4'd14) ? retired_snap[ci] :
                (current_csr_addr == 4'd15) ? icache_miss_snap[ci] :
                core_mig_csr_rdata_flat[(ci*32) +: 32];

            // ---------------------------------------------------------
            // rv32_core instance
            // ---------------------------------------------------------
            rv32_core #(
                .HART_ID  (ci),
                .RESET_PC (32'h0000_0000)
            ) core_inst (
                .clk             (clk),
                .rst             (core_rst_r[ci]),
                .clk_en          (core_clk_en[ci]),
                .core_en         (core_enable_runtime[ci]),

                .imem_addr       (imem_addr[ci]),
                .imem_rdata      (imem_rdata[ci]),
                .imem_req        (imem_req[ci]),
                .imem_ready      (imem_ready[ci]),

                .dmem_addr       (dmem_addr[ci]),
                .dmem_wdata      (dmem_wdata[ci]),
                .dmem_rdata      (dmem_rdata[ci]),
                .dmem_wstrb      (dmem_wstrb[ci]),
                .dmem_we         (dmem_we[ci]),
                .dmem_req        (dmem_req[ci]),
                .dmem_ready      (dmem_ready[ci]),

                .snoop_addr      (snoop_addr),
                .snoop_we        (snoop_we),

                .timer_irq       (timer_irq_global),
                .ext_irq         (1'b0),
                .thermal_irq     (thermal_irq_core_r[ci]),
                .thermal_level   (thermal_level_core_r[ci]),
                .perf_overflow_irq(1'b0),

                .thermal_reading (thermal_reading),

                .mig_we          (mig_we_vec[ci]),
                .mig_addr        (mig_addr_flat[(ci*5) +: 5]),
                .mig_wdata       (mig_wdata_flat[(ci*32) +: 32]),
                .mig_rdata       (mig_rdata_flat[(ci*32) +: 32]),

                .mig_csr_we      (mig_csr_we_vec[ci]),
                .mig_csr_addr    (mig_csr_addr_flat[(ci*4) +: 4]),
                .mig_csr_wdata   (mig_csr_wdata_flat[(ci*32) +: 32]),
                .mig_csr_rdata   (core_mig_csr_rdata_flat[(ci*32) +: 32]),

                .migration_event (migration_event_core[ci]),

                .pc_save         (pc_save_flat[(ci*32) +: 32]),
                .pc_load_en      (pc_load_en_vec[ci]),
                .pc_load         (pc_load_flat[(ci*32) +: 32]),

                .active          (core_active[ci]),
                .retired         (core_retired[ci])
            );

            // ---------------------------------------------------------
            // Instruction Cache (per-core, private)
            // ---------------------------------------------------------
            icache #(
                .CACHE_LINES   (256),
                .LINE_WORDS    (4)
            ) icache_inst (
                .clk           (clk),
                .rst           (core_rst_r[ci]),

                .cpu_addr      (imem_addr[ci]),
                .cpu_req       (core_if_req),
                .cpu_rdata     (imem_rdata[ci]),
                .cpu_ready     (imem_ready[ci]),

                .mem_addr      (icache_mem_addr[ci]),
                .mem_rdata     (icache_mem_rdata[ci]),
                .mem_req       (icache_mem_req[ci]),
                .mem_ready     (icache_mem_ready[ci]),

                .flush_all     (1'b0),
                .inspect_index (cache_warmer_inspect_idx),
                .inspect_valid (icache_valid_vec[ci]),
                .inspect_tag   (icache_tag_flat[(ci*20) +: 20])
            );

            // ---------------------------------------------------------
            // Phase 7.5: Per-core Victim Buffer + Stride Prefetcher
            // ---------------------------------------------------------
            logic                       vb_evict_valid;
            logic [19:0]                vb_evict_tag;
            logic [7:0]                 vb_evict_index;
            logic [127:0]               vb_evict_data;
            logic                       vb_lookup_valid;
            logic [19:0]                vb_lookup_tag;
            logic [7:0]                 vb_lookup_index;
            logic                       vb_hit;
            logic [127:0]               vb_hit_data;
            logic                       vb_swap_valid;
            logic [19:0]                vb_swap_tag;
            logic [7:0]                 vb_swap_index;
            logic [127:0]               vb_swap_data;

            logic                       sp_pf_req;
            logic [31:0]                sp_pf_addr;
            logic                       sp_pf_ack;
            logic                       sp_miss_notify;
            logic [31:0]                sp_miss_addr;

            // ---------------------------------------------------------
            // Data Cache (per-core, private, write-through + Phase 7.5)
            // ---------------------------------------------------------
            Data_Cache #(
                .CACHE_LINES   (256),
                .LINE_WORDS    (4),
                .CORE_ID       (ci)
            ) dcache_inst (
                .clk           (clk),
                .rst           (core_rst_r[ci]),

                .cpu_addr      (dmem_addr[ci]),
                .cpu_wdata     (dmem_wdata[ci]),
                .cpu_wstrb     (core_d_wstrb),
                .cpu_we        (core_d_we),
                .cpu_req       (core_d_req),
                .cpu_rdata     (dmem_rdata[ci]),
                .cpu_ready     (dmem_ready[ci]),

                .mem_addr      (dcache_mem_addr[ci]),
                .mem_wdata     (dcache_mem_wdata[ci]),
                .mem_wstrb     (dcache_mem_wstrb[ci]),
                .mem_we        (dcache_mem_we[ci]),
                .mem_req       (dcache_mem_req[ci]),
                .mem_rdata     (dcache_mem_rdata[ci]),
                .mem_ready     (dcache_mem_ready[ci]),

                .snoop_addr    (snoop_addr),
                .snoop_we      (snoop_we),
                .snoop_src     (snoop_src),

                .flush_all     (1'b0),

                // Victim buffer interface
                .victim_evict_valid (vb_evict_valid),
                .victim_evict_tag   (vb_evict_tag),
                .victim_evict_index (vb_evict_index),
                .victim_evict_data  (vb_evict_data),
                .victim_lookup_valid(vb_lookup_valid),
                .victim_lookup_tag  (vb_lookup_tag),
                .victim_lookup_index(vb_lookup_index),
                .victim_hit         (vb_hit),
                .victim_hit_data    (vb_hit_data),
                .victim_swap_valid  (vb_swap_valid),
                .victim_swap_tag    (vb_swap_tag),
                .victim_swap_index  (vb_swap_index),
                .victim_swap_data   (vb_swap_data),

                // Stride prefetcher interface
                .pf_req        (sp_pf_req),
                .pf_addr       (sp_pf_addr),
                .pf_ack        (sp_pf_ack),
                .miss_notify   (sp_miss_notify),
                .miss_notify_addr(sp_miss_addr)
            );

            // ---------------------------------------------------------
            // Victim Buffer (2-entry, per-core)
            // ---------------------------------------------------------
            Victim_Buffer #(
                .NUM_ENTRIES(2),
                .LINE_WORDS (4)
            ) victim_buf_inst (
                .clk           (clk),
                .rst           (core_rst_r[ci]),

                .evict_valid   (vb_evict_valid),
                .evict_tag     (vb_evict_tag),
                .evict_index   (vb_evict_index),
                .evict_data    (vb_evict_data),

                .lookup_valid  (vb_lookup_valid),
                .lookup_tag    (vb_lookup_tag),
                .lookup_index  (vb_lookup_index),
                .lookup_hit    (vb_hit),
                .lookup_data   (vb_hit_data),

                .swap_valid    (vb_swap_valid),
                .swap_tag      (vb_swap_tag),
                .swap_index    (vb_swap_index),
                .swap_data     (vb_swap_data),

                .snoop_tag     (snoop_addr[31:12]),
                .snoop_index   (snoop_addr[11:4]),
                .snoop_valid   (snoop_we && (snoop_src != ci[2:0])),

                .flush_all     (1'b0)
            );

            // ---------------------------------------------------------
            // Stride Prefetcher (4-entry, per-core)
            // ---------------------------------------------------------
            Stride_Prefetcher #(
                .TABLE_ENTRIES(4)
            ) stride_pf_inst (
                .clk           (clk),
                .rst           (core_rst_r[ci]),

                .miss_valid    (sp_miss_notify),
                .miss_addr     (sp_miss_addr),
                .miss_pc       (dmem_addr[ci]),  // Approximate: use data addr as PC proxy

                .pf_req        (sp_pf_req),
                .pf_addr       (sp_pf_addr),
                .pf_ack        (sp_pf_ack)
            );

            // ---------------------------------------------------------
            // AXI Master Wrapper -- I-Cache memory port
            // ---------------------------------------------------------
            axi_master_wrapper #(
                .QOS_LEVEL (IFETCH_QOS)
            ) imem_axi_master (
                .clk       (clk),
                .rst       (core_rst_r[ci]),
                .qos_override(adjusted_qos[ci]),

                .mem_addr  (icache_mem_addr[ci]),
                .mem_wdata (32'b0),
                .mem_rdata (icache_mem_rdata[ci]),
                .mem_wstrb (4'b0000),
                .mem_we    (1'b0),
                .mem_req   (core_if_req && icache_mem_req[ci]),
                .mem_ready (icache_mem_ready[ci]),
                .mem_lock  (core_if_req && icache_mem_req[ci]),

                .m_awaddr  (axi_m_awaddr [32*M_IDX_I +: 32]),
                .m_wdata   (axi_m_wdata  [32*M_IDX_I +: 32]),
                .m_wstrb   (axi_m_wstrb  [4*M_IDX_I  +: 4]),
                .m_awvalid (axi_m_awvalid[M_IDX_I]),
                .m_awready (axi_m_awready[M_IDX_I]),
                .m_bresp   (axi_m_bresp  [2*M_IDX_I  +: 2]),
                .m_bvalid  (axi_m_bvalid [M_IDX_I]),
                .m_bready  (axi_m_bready [M_IDX_I]),

                .m_araddr  (axi_m_araddr [32*M_IDX_I +: 32]),
                .m_arvalid (axi_m_arvalid[M_IDX_I]),
                .m_arready (axi_m_arready[M_IDX_I]),
                .m_rdata   (axi_m_rdata  [32*M_IDX_I +: 32]),
                .m_rresp   (axi_m_rresp  [2*M_IDX_I  +: 2]),
                .m_rvalid  (axi_m_rvalid [M_IDX_I]),
                .m_rready  (axi_m_rready [M_IDX_I]),

                .m_qos     (axi_m_qos   [3*M_IDX_I  +: 3]),
                .m_lock    (axi_m_lock   [M_IDX_I])
            );

            // ---------------------------------------------------------
            // AXI Master Wrapper -- Data port
            // ---------------------------------------------------------
            axi_master_wrapper #(
                .QOS_LEVEL (DATA_QOS)
            ) dmem_axi_master (
                .clk       (clk),
                .rst       (core_rst_r[ci]),
                .qos_override(adjusted_qos[ci]),

                .mem_addr  (dcache_mem_addr[ci]),
                .mem_wdata (dcache_mem_wdata[ci]),
                .mem_rdata (dcache_mem_rdata[ci]),
                .mem_wstrb (dcache_mem_wstrb[ci]),
                .mem_we    (dcache_mem_we[ci]),
                .mem_req   (core_d_req && dcache_mem_req[ci]),
                .mem_ready (dcache_mem_ready[ci]),
                .mem_lock  (core_d_req && dcache_mem_req[ci]),

                .m_awaddr  (axi_m_awaddr [32*M_IDX_D +: 32]),
                .m_wdata   (axi_m_wdata  [32*M_IDX_D +: 32]),
                .m_wstrb   (axi_m_wstrb  [4*M_IDX_D  +: 4]),
                .m_awvalid (axi_m_awvalid[M_IDX_D]),
                .m_awready (axi_m_awready[M_IDX_D]),
                .m_bresp   (axi_m_bresp  [2*M_IDX_D  +: 2]),
                .m_bvalid  (axi_m_bvalid [M_IDX_D]),
                .m_bready  (axi_m_bready [M_IDX_D]),

                .m_araddr  (axi_m_araddr [32*M_IDX_D +: 32]),
                .m_arvalid (axi_m_arvalid[M_IDX_D]),
                .m_arready (axi_m_arready[M_IDX_D]),
                .m_rdata   (axi_m_rdata  [32*M_IDX_D +: 32]),
                .m_rresp   (axi_m_rresp  [2*M_IDX_D  +: 2]),
                .m_rvalid  (axi_m_rvalid [M_IDX_D]),
                .m_rready  (axi_m_rready [M_IDX_D]),

                .m_qos     (axi_m_qos   [3*M_IDX_D  +: 3]),
                .m_lock    (axi_m_lock   [M_IDX_D])
            );

        end
    endgenerate

        genvar mi;
        generate
            for (mi = 0; mi < TOTAL_CORES; mi = mi + 1) begin : icache_miss_counters
                logic miss_cycle;
                assign miss_cycle = imem_req[mi] & ~imem_ready[mi];

                logic write_miss;
                assign write_miss = mig_csr_we_vec[mi] && (mig_csr_addr_flat[(mi*4)+:4] == 4'd15);

                always_ff @(posedge clk) begin
                    if (core_rst_r[mi]) begin
                        icache_miss_delta[mi] <= 32'd0;
                        icache_miss_snap[mi]  <= 32'd0;
                    end else begin

                        if (write_miss) icache_miss_snap[mi] <= mig_csr_wdata_flat[(mi*32)+:32];
                        else if (sample_tick) icache_miss_snap[mi] <= icache_miss_delta[mi];

                        if (sample_tick) begin
                            icache_miss_delta[mi] <= {31'd0, miss_cycle};
                        end else begin
                            icache_miss_delta[mi] <= icache_miss_delta[mi] + {31'd0, miss_cycle};
                        end
                    end
                end
            end
        endgenerate

    logic [TOTAL_CORES*32-1:0] icache_miss_flat;
    assign icache_miss_flat = {
        icache_miss_snap[4], icache_miss_snap[3], icache_miss_snap[2], icache_miss_snap[1], icache_miss_snap[0]
    };
    logic [TOTAL_CORES*32-1:0] power_proxy_flat;
    assign power_proxy_flat = {
        power_proxy[4], power_proxy[3], power_proxy[2], power_proxy[1], power_proxy[0]
    };
    logic [TOTAL_CORES-1:0] phase_changed_vec;
    assign phase_changed_vec = {
        phase_changed[4], phase_changed[3], phase_changed[2], phase_changed[1], phase_changed[0]
    };

    logic policy_enable;
    assign policy_enable = (!bus_rst_r) && (eval_active_mode == 2'd3);
    logic power_gate_enable;
    assign power_gate_enable = (!bus_rst_r) && (eval_active_mode == 2'd3);
    logic [1:0] thermal_l2_mode;
    assign thermal_l2_mode = eval_active_mode;

    logic migration_start_auto;
    assign migration_start_auto = (!bus_rst_r) && policy_enable && policy_migrate_req && !migration_busy;
    logic migration_start_cmd;
    assign migration_start_cmd = migration_start_pulse | migration_start_auto;
    logic [2:0] migration_src_cmd;
    assign migration_src_cmd = migration_start_pulse ? migration_src_sel : policy_src_core;
    logic [2:0] migration_dst_cmd;
    assign migration_dst_cmd = migration_start_pulse ? migration_dst_sel : policy_dst_core;

    always_comb begin
        migration_event_core = {TOTAL_CORES{1'b0}};
        if (migration_done) begin
            if (migration_active_src < TOTAL_CORES)
                migration_event_core[migration_active_src] = 1'b1;
            if (migration_active_dst < TOTAL_CORES)
                migration_event_core[migration_active_dst] = 1'b1;
        end
    end

    // ===================================================================
    // THERMAL SYSTEM INSTANTIATION
    // ===================================================================
    
    genvar ti;
    generate
        for (ti = 0; ti < TOTAL_CORES; ti = ti + 1) begin : thermal_sensor_gen
            Thermal_Sensor sensor_inst (
                .clk                  (clk),
                .rst_n                (core_rst_n[ti]),
                
                .dt_over_c1           (16'h0002),
                .dt_over_c2           (16'h0001),
                .dt_over_c3           (16'h0000),
                .inv_r12              (16'h0004),
                .inv_r23              (16'h0005),
                .inv_r3a              (16'h0004),
                
                .p_dynamic            (power_proxy[ti]),
                .t_amb                (THERMAL_AMB),
                .sensor_bias          (16'h0000),
                .sensor_noise_sigma   (16'h0100),
                
                .t_sensor             (t_sensor[ti])
            );

            Kalman_Predictor kalman_inst (
                .clk                  (clk),
                .rst_n                (core_rst_n[ti]),
                .t_sensor             (t_sensor_fused[ti]),
                .power_proxy          (power_proxy[ti]),
                .k_f                  (32'h000D_6F00),
                .k_b                  (32'h0000_0400),
                .k_g                  (32'h0000_0100),
                .k_q                  (32'h0000_0010),
                .k_r                  (32'h0000_0020),
                .t_warn               (THERMAL_WARN),
                .t_crit               (THERMAL_CRIT),
                .t_amb                (THERMAL_AMB),
                .t_estimated          (t_estimated[ti]),
                .t_predicted          (t_predicted[ti]),
                .p_uncertainty        (p_uncertainty[ti]),
                .thermal_state        (thermal_state[ti]),
                .innovation           (innovation[ti])
            );

            // Phase Detector — connected to REAL activity counters
            Phase_Detector phase_inst (
                .clk                  (clk),
                .rst_n                (core_rst_n[ti]),
                .minstret_delta       (retired_snap[ti]),
                .mcycle_delta         (32'h0000_1000),  // Window size = 4096
                .icache_miss          (icache_miss_snap[ti]),
                .icache_hit           (active_snap[ti]),
                .phase                (phase[ti]),
                .phase_changed        (phase_changed[ti])
            );
        end
    endgenerate

    // Flatten arrays for policy/controller/eval
    logic [159:0] t_sensor_flat;
    assign t_sensor_flat = {thermal_level_core_r[4], thermal_level_core_r[3], thermal_level_core_r[2], thermal_level_core_r[1], thermal_level_core_r[0]};
    logic [159:0] t_predicted_flat;
    assign t_predicted_flat = {thermal_pred_core_r[4], thermal_pred_core_r[3], thermal_pred_core_r[2], thermal_pred_core_r[1], thermal_pred_core_r[0]};
    logic [9:0]   phase_flat;
    assign phase_flat = {phase[4], phase[3], phase[2], phase[1], phase[0]};
    logic [9:0]   thermal_state_flat;
    assign thermal_state_flat = {thermal_state[4], thermal_state[3], thermal_state[2], thermal_state[1], thermal_state[0]};
    logic [14:0] default_qos_flat;
    assign default_qos_flat = {default_qos[4], default_qos[3], default_qos[2], default_qos[1], default_qos[0]};
    logic [14:0] adjusted_qos_flat;
    logic [159:0] power_budget_flat;
    logic [31:0] cache_warm_src_pc;
    assign cache_warm_src_pc = pc_save_flat[(migration_active_src*32) +: 32];

    Migration_Policy #(
        .NUM_CORES (TOTAL_CORES)
    ) migration_policy (
        .clk                  (clk),
        .rst_n                (bus_rst_n),
        .enable               (policy_enable),
        .t_predicted_flat     (t_predicted_flat),
        .phase_flat           (phase_flat),
        .phase_changed_flat   (phase_changed_vec),
        .icache_miss_flat     (icache_miss_flat),
        .t_warn               (THERMAL_WARN),
        .t_crit               (THERMAL_CRIT),
        .cache_warm_cycles    (cache_warm_cycles),
        .migration_busy       (migration_busy),
        .migrate_req          (policy_migrate_req),
        .src_core             (policy_src_core),
        .dst_core             (policy_dst_core),
        .decision_cost_cycles (policy_cost_cycles),
        .decision_benefit_score(policy_benefit_score),
        .all_cores_hot        (policy_all_cores_hot),
        .dual_hot_mode        (policy_dual_hot_mode),
        .imbalance_block      (policy_imbalance_block),
        .migration_hist_count_flat(policy_migration_hist_flat),
        .recent_history_flat  (policy_recent_history_flat)
    );

    Cache_Warmer cache_warmer (
        .clk                  (clk),
        .rst_n                (bus_rst_n),
        .start                (cache_warm_start),
        .src_core             (migration_active_src),
        .dst_core             (migration_active_dst),
        .src_pc               (cache_warm_src_pc[11:4]),
        .warm_entries_cfg     (32'd16),
        .inspect_index        (cache_warmer_inspect_idx),
        .inspect_valid        (icache_valid_vec[migration_active_src]),
        .inspect_tag          (icache_tag_flat[(migration_active_src*20) +: 20]),
        .busy                 (cache_warm_busy),
        .done                 (cache_warm_done),
        .cache_warm_cycles    (cache_warm_cycles),
        .prefetch_count       (cache_prefetch_count),
        .prefetch_addr        (cache_prefetch_addr),
        .prefetch_valid       (cache_prefetch_valid)
    );

    // ===================================================================
    // PHASE 5.1: MIGRATION CONTROLLER
    // Automatic trigger path comes from Migration_Policy.
    // ===================================================================
    Migration_Controller #(
        .NUM_CORES  (TOTAL_CORES),
        .CORE_IDX_W (3)
    ) migration_ctrl (
        .clk                  (clk),
        .rst_n                (bus_rst_n),
        .start                (migration_start_cmd),
        .clear_halt           (migration_clear_halt_pulse),
        .src_core             (migration_src_cmd),
        .dst_core             (migration_dst_cmd),
        .cache_warm_done      (cache_warm_done),
        .cache_warm_start     (cache_warm_start),
        .pc_save_flat         (pc_save_flat),
        .mig_rdata_flat       (mig_rdata_flat),
        .csr_rdata_flat       (mig_csr_rdata_flat),
        .pg_save_req          (pg_save_req),
        .pg_restore_req       (pg_restore_req),
        .pg_ack               (pg_ack),
        .mig_we               (mig_we_vec),
        .mig_addr_flat        (mig_addr_flat),
        .mig_wdata_flat       (mig_wdata_flat),
        .csr_we               (mig_csr_we_vec),
        .csr_addr_flat        (mig_csr_addr_flat),
        .csr_wdata_flat       (mig_csr_wdata_flat),
        .pc_load_en           (pc_load_en_vec),
        .pc_load_flat         (pc_load_flat),
        .pause_mask           (mig_pause_mask),
        .halt_mask            (mig_halt_mask),
        .busy                 (migration_busy),
        .done                 (migration_done),
        .error                (migration_error),
        .last_migration_cycles(migration_last_cycles),
        .migration_count      (migration_count),
        .active_src_core      (migration_active_src),
        .active_dst_core      (migration_active_dst)
    );

    genvar p_i;
    generate
        for (p_i = 0; p_i < 5; p_i = p_i + 1) begin : p_budget_gen
            assign power_budget[p_i] = power_budget_flat[(p_i*32) +: 32];
        end
    endgenerate

    Thermal_Controller thermal_ctrl (
        .clk                  (clk),
        .rst_n                (bus_rst_n),
        .core_id_flat         ({2'b00, 2'b11, 2'b10, 2'b01, 2'b00}),
        .t_sensor_flat        (t_sensor_flat),
        .t_predicted_flat     (t_predicted_flat),
        .phase_flat           (phase_flat),
        .t_warn               (THERMAL_WARN),
        .t_crit               (THERMAL_CRIT),
        .t_l1_cap             (THERMAL_L1),
        .l2_mode              (thermal_l2_mode),
        .dt_max               (THERMAL_DMAX),
        .clk_en               (controller_clk_en),
        .power_budget_flat    (power_budget_flat),
        .thermal_irq          (thermal_irq)
    );

    Power_Gate_Controller #(
        .NUM_CORES (TOTAL_CORES)
    ) power_gate_ctrl (
        .clk                  (clk),
        .rst_n                (bus_rst_n),
        .enable               (power_gate_enable),
        .phase_flat           (phase_flat),
        .thermal_state_flat   (thermal_state_flat),
        .base_core_enable     (core_enable),
        .migration_req        (migration_start_cmd),
        .migration_busy       (migration_busy),
        .migration_dst_core   (migration_dst_cmd),
        .power_gate_mask      (power_gate_mask),
        .pwr_gated_cycles_flat(pwr_gated_cycles_flat),
        .total_pwr_gated_cycles(total_power_gate_cycles),
        .power_gate_events    (power_gate_events),
        .pg_save_req          (pg_save_req),
        .pg_restore_req       (pg_restore_req),
        .pg_ack               (pg_ack)
    );

    logic eval_uart_tx;

    Eval_Framework #(
        .NUM_CORES (TOTAL_CORES)
    ) eval_fw (
        .clk                  (clk),
        .rst_n                (bus_rst_n),
        .mode_write_data      (eval_mode_wdata),
        .mode_write_en        (eval_mode_we),
        .run_done_pulse       (eval_run_done_pulse),
        .core_retired         (core_retired),
        .core_clk_en          (core_clk_en),
        .power_gate_mask      (power_gate_mask),
        .t_sensor_flat        (t_sensor_flat),
        .power_proxy_flat     (power_proxy_flat),
        .t_crit               (THERMAL_CRIT),
        .migration_done       (migration_done),
        .migration_cycles_inc (migration_last_cycles),
        .active_mode          (eval_active_mode),
        .metrics_frozen       (eval_metrics_frozen),
        .total_cycles         (eval_total_cycles),
        .total_retired        (eval_total_retired),
        .peak_temperature     (eval_peak_temperature),
        .avg_temperature      (eval_avg_temperature),
        .throttle_events      (eval_throttle_events),
        .throttle_cycles      (eval_throttle_cycles),
        .migration_count      (eval_migration_count),
        .migration_cycles     (eval_migration_cycles),
        .power_gate_cycles    (eval_power_gate_cycles),
        .thermal_violations   (eval_thermal_violations),
        .energy_estimate      (eval_energy_estimate),
        .uart_tx              (eval_uart_tx)
    );
    
    assign {adjusted_qos[4], adjusted_qos[3], adjusted_qos[2], adjusted_qos[1], adjusted_qos[0]} = adjusted_qos_flat;

    QoS_Adjuster qos_adj (
        .clk                  (clk),
        .rst_n                (bus_rst_n),
        .thermal_state_flat   (thermal_state_flat),
        .phase_flat           (phase_flat),
        .default_qos_flat     (default_qos_flat),
        .adjusted_qos_flat    (adjusted_qos_flat)
    );

    // ===================================================================
    // MAC Accelerator AXI Master -- stub
    // ===================================================================
    localparam MAC_M_IDX = NUM_MASTERS - 1;

    assign axi_m_awaddr [32*MAC_M_IDX +: 32] = 32'b0;
    assign axi_m_wdata  [32*MAC_M_IDX +: 32] = 32'b0;
    assign axi_m_wstrb  [4*MAC_M_IDX  +: 4]  = 4'b0;
    assign axi_m_awvalid[MAC_M_IDX]           = 1'b0;
    assign axi_m_bready [MAC_M_IDX]           = 1'b0;
    assign axi_m_araddr [32*MAC_M_IDX +: 32]  = 32'b0;
    assign axi_m_arvalid[MAC_M_IDX]           = 1'b0;
    assign axi_m_rready [MAC_M_IDX]           = 1'b0;
    assign axi_m_qos    [3*MAC_M_IDX  +: 3]  = axi_pkg::QOS_MAC_ACCEL;
    assign axi_m_lock   [MAC_M_IDX]           = 1'b0;


    // ===================================================================
    // AXI INTERCONNECT
    // ===================================================================
    axi_interconnect #(
        .NUM_MASTERS (NUM_MASTERS),
        .NUM_SLAVES  (NUM_SLAVES)
    ) interconnect_inst (
        .clk         (clk),
        .rst         (bus_rst_r),

        .m_awaddr    (axi_m_awaddr),
        .m_wdata     (axi_m_wdata),
        .m_wstrb     (axi_m_wstrb),
        .m_awvalid   (axi_m_awvalid),
        .m_awready   (axi_m_awready),
        .m_bresp     (axi_m_bresp),
        .m_bvalid    (axi_m_bvalid),
        .m_bready    (axi_m_bready),

        .m_araddr    (axi_m_araddr),
        .m_arvalid   (axi_m_arvalid),
        .m_arready   (axi_m_arready),
        .m_rdata     (axi_m_rdata),
        .m_rresp     (axi_m_rresp),
        .m_rvalid    (axi_m_rvalid),
        .m_rready    (axi_m_rready),

        .m_qos       (axi_m_qos),
        .m_lock      (axi_m_lock),

        .s_awaddr    (axi_s_awaddr),
        .s_wdata     (axi_s_wdata),
        .s_wstrb     (axi_s_wstrb),
        .s_awvalid   (axi_s_awvalid),
        .s_wvalid    (axi_s_wvalid),
        .s_awready   (axi_s_awready),
        .s_bresp     (axi_s_bresp),
        .s_bvalid    (axi_s_bvalid),
        .s_bready    (axi_s_bready),

        .s_araddr    (axi_s_araddr),
        .s_arvalid   (axi_s_arvalid),
        .s_arready   (axi_s_arready),
        .s_rdata     (axi_s_rdata),
        .s_rresp     (axi_s_rresp),
        .s_rvalid    (axi_s_rvalid),
        .s_rready    (axi_s_rready)
    );


    // ===================================================================
    // SLAVE 0: Boot ROM
    // ===================================================================
    Boot_ROM #(
        // Keep the ROM image path repo-relative so both CI and local runs
        // resolve the same boot payload without simulator cwd assumptions.
        .BOOT_HEX_FILE ("boot.hex")
    ) boot_slave (
        .clk(clk), .rst_n(bus_rst_n),
        .s_axi_awaddr  (axi_s_awaddr [32*axi_pkg::SLAVE_BOOT +: 32]),
        .s_axi_wdata   (axi_s_wdata  [32*axi_pkg::SLAVE_BOOT +: 32]),
        .s_axi_wstrb   (axi_s_wstrb  [4*axi_pkg::SLAVE_BOOT  +: 4]),
        .s_axi_awvalid (axi_s_awvalid[axi_pkg::SLAVE_BOOT]),
        .s_axi_wvalid  (axi_s_wvalid[axi_pkg::SLAVE_BOOT]),
        .s_axi_awready (axi_s_awready[axi_pkg::SLAVE_BOOT]),
        .s_axi_wready  (),
        .s_axi_bresp   (axi_s_bresp  [2*axi_pkg::SLAVE_BOOT  +: 2]),
        .s_axi_bvalid  (axi_s_bvalid [axi_pkg::SLAVE_BOOT]),
        .s_axi_bready  (axi_s_bready [axi_pkg::SLAVE_BOOT]),
        .s_axi_araddr  (axi_s_araddr [32*axi_pkg::SLAVE_BOOT +: 32]),
        .s_axi_arvalid (axi_s_arvalid[axi_pkg::SLAVE_BOOT]),
        .s_axi_arready (axi_s_arready[axi_pkg::SLAVE_BOOT]),
        .s_axi_rdata   (axi_s_rdata  [32*axi_pkg::SLAVE_BOOT +: 32]),
        .s_axi_rresp   (axi_s_rresp  [2*axi_pkg::SLAVE_BOOT  +: 2]),
        .s_axi_rvalid  (axi_s_rvalid [axi_pkg::SLAVE_BOOT]),
        .s_axi_rready  (axi_s_rready [axi_pkg::SLAVE_BOOT])
    );


    // ===================================================================
    // SLAVE 1: Main RAM (64KB)
    // ===================================================================
    logic [31:0] ram_mem_addr, ram_mem_wdata, ram_mem_rdata;
    logic [3:0]  ram_mem_wstrb;
    logic        ram_mem_we, ram_mem_req, ram_mem_ready;

    axi_slave_wrapper ram_slave (
        .clk(clk), .rst(bus_rst_r),
        .s_awaddr  (axi_s_awaddr [32*axi_pkg::SLAVE_RAM +: 32]),
        .s_wdata   (axi_s_wdata  [32*axi_pkg::SLAVE_RAM +: 32]),
        .s_wstrb   (axi_s_wstrb  [4*axi_pkg::SLAVE_RAM  +: 4]),
        .s_awvalid (axi_s_awvalid[axi_pkg::SLAVE_RAM]),
        .s_wvalid  (axi_s_wvalid[axi_pkg::SLAVE_RAM]),
        .s_awready (axi_s_awready[axi_pkg::SLAVE_RAM]),
        .s_bresp   (axi_s_bresp  [2*axi_pkg::SLAVE_RAM  +: 2]),
        .s_bvalid  (axi_s_bvalid [axi_pkg::SLAVE_RAM]),
        .s_bready  (axi_s_bready [axi_pkg::SLAVE_RAM]),
        .s_araddr  (axi_s_araddr [32*axi_pkg::SLAVE_RAM +: 32]),
        .s_arvalid (axi_s_arvalid[axi_pkg::SLAVE_RAM]),
        .s_arready (axi_s_arready[axi_pkg::SLAVE_RAM]),
        .s_rdata   (axi_s_rdata  [32*axi_pkg::SLAVE_RAM +: 32]),
        .s_rresp   (axi_s_rresp  [2*axi_pkg::SLAVE_RAM  +: 2]),
        .s_rvalid  (axi_s_rvalid [axi_pkg::SLAVE_RAM]),
        .s_rready  (axi_s_rready [axi_pkg::SLAVE_RAM]),
        .mem_addr  (ram_mem_addr),
        .mem_wdata (ram_mem_wdata),
        .mem_rdata (ram_mem_rdata),
        .mem_wstrb (ram_mem_wstrb),
        .mem_we    (ram_mem_we),
        .mem_req   (ram_mem_req),
        .mem_ready (ram_mem_ready)
    );

    // FIXED: Reduced from 8 MB (2,097,151 entries) to 256 KB (65,535 entries).
    // Original massive size was consuming 1M+ distributed LUTs (no UltraRAM on xcu250).
    // 256 KB is reasonable for embedded thermal+cache demo on this device.
    // Address indexing changed from [22:2] to [17:2] (16-bit address width for 256 KB).
    (* ram_style = "block" *) logic [31:0] ram [0:65535];
    
    // URAM read pipeline stages - marked with keep to prevent optimization
    (* keep = "true" *) logic [31:0] ram_mem_rdata_s0;  // Stage 0: Direct URAM output capture
    (* keep = "true" *) logic [31:0] ram_mem_rdata_pipe;  // Stage 1: Conditional forward
    (* keep = "true" *) logic [31:0] ram_mem_rdata_s1;  // Stage 2: Additional pipeline
    (* keep = "true" *) logic [31:0] ram_mem_rdata_reg;  // Stage 3: Final registered output
    
    // Ready/control signals through pipeline
    (* keep = "true" *) logic        ram_mem_ready_s0;
    (* keep = "true" *) logic        ram_mem_ready_pipe;
    (* keep = "true" *) logic        ram_mem_ready_s1;
    (* keep = "true" *) logic        ram_mem_ready_reg;
    (* keep = "true" *) logic        ram_mem_is_read_s0;  // Track read vs write at s0

    always_ff @(posedge clk) begin
        if (bus_rst_r) begin
            ram_mem_rdata_s0   <= 32'b0;
            ram_mem_rdata_pipe <= 32'b0;
            ram_mem_rdata_s1   <= 32'b0;
            ram_mem_rdata_reg  <= 32'b0;
            ram_mem_ready_s0   <= 1'b0;
            ram_mem_ready_pipe <= 1'b0;
            ram_mem_ready_s1   <= 1'b0;
            ram_mem_ready_reg  <= 1'b0;
            ram_mem_is_read_s0 <= 1'b0;
        end else begin
            // Stage 0: Direct URAM output capture (NEW - breaks critical path)
            // Always read URAM, then select read vs write data in next stage
            ram_mem_rdata_s0   <= ram[ram_mem_addr[17:2]];  // Reduced from [22:2] for 256KB
            ram_mem_ready_s0   <= ram_mem_req;
            ram_mem_is_read_s0 <= (ram_mem_req && !ram_mem_we);  // Latch read indicator
            
            // Stage 1: Conditional forward (only forward on read operations)
            ram_mem_rdata_pipe <= (ram_mem_is_read_s0) ? ram_mem_rdata_s0 : ram_mem_rdata_pipe;
            ram_mem_ready_pipe <= ram_mem_ready_s0;
            
            // Stage 2: Additional pipeline register
            ram_mem_rdata_s1   <= ram_mem_rdata_pipe;
            ram_mem_ready_s1   <= ram_mem_ready_pipe;
            
            // Stage 3: Final registered output
            ram_mem_rdata_reg  <= ram_mem_rdata_s1;
            ram_mem_ready_reg  <= ram_mem_ready_s1;

            // Write operations (not pipelined for now)
            if (ram_mem_req && ram_mem_we) begin
                if (ram_mem_wstrb[0]) ram[ram_mem_addr[17:2]][7:0]   <= ram_mem_wdata[7:0];   // Reduced from [22:2]
                if (ram_mem_wstrb[1]) ram[ram_mem_addr[17:2]][15:8]  <= ram_mem_wdata[15:8];  // Reduced from [22:2]
                if (ram_mem_wstrb[2]) ram[ram_mem_addr[17:2]][23:16] <= ram_mem_wdata[23:16]; // Reduced from [22:2]
                if (ram_mem_wstrb[3]) ram[ram_mem_addr[17:2]][31:24] <= ram_mem_wdata[31:24]; // Reduced from [22:2]
            end
        end
    end

    assign ram_mem_rdata = ram_mem_rdata_reg;
    assign ram_mem_ready = ram_mem_ready_reg;


    // ===================================================================
    // SLAVE 2: VGA Framebuffer -- stub
    // ===================================================================
    axi_slave_wrapper vga_slave (
        .clk(clk), .rst(bus_rst_r),
        .s_awaddr  (axi_s_awaddr [32*axi_pkg::SLAVE_VGA +: 32]),
        .s_wdata   (axi_s_wdata  [32*axi_pkg::SLAVE_VGA +: 32]),
        .s_wstrb   (axi_s_wstrb  [4*axi_pkg::SLAVE_VGA  +: 4]),
        .s_awvalid (axi_s_awvalid[axi_pkg::SLAVE_VGA]),
        .s_wvalid  (axi_s_wvalid[axi_pkg::SLAVE_VGA]),
        .s_awready (axi_s_awready[axi_pkg::SLAVE_VGA]),
        .s_bresp   (axi_s_bresp  [2*axi_pkg::SLAVE_VGA  +: 2]),
        .s_bvalid  (axi_s_bvalid [axi_pkg::SLAVE_VGA]),
        .s_bready  (axi_s_bready [axi_pkg::SLAVE_VGA]),
        .s_araddr  (axi_s_araddr [32*axi_pkg::SLAVE_VGA +: 32]),
        .s_arvalid (axi_s_arvalid[axi_pkg::SLAVE_VGA]),
        .s_arready (axi_s_arready[axi_pkg::SLAVE_VGA]),
        .s_rdata   (axi_s_rdata  [32*axi_pkg::SLAVE_VGA +: 32]),
        .s_rresp   (axi_s_rresp  [2*axi_pkg::SLAVE_VGA  +: 2]),
        .s_rvalid  (axi_s_rvalid [axi_pkg::SLAVE_VGA]),
        .s_rready  (axi_s_rready [axi_pkg::SLAVE_VGA]),
        .mem_addr  (),
        .mem_wdata (),
        .mem_rdata (32'b0),
        .mem_wstrb (),
        .mem_we    (),
        .mem_req   (),
        .mem_ready (1'b1)
    );


    // ===================================================================
    // SLAVE 3: MMIO Region (Sub-decoded: Core MMIO + UART + Timer)
    //
    // Address Map within 0x9000_xxxx:
    //   0x9000_0xxx  Core MMIO (LEDs, Buttons, 7-seg, thermal readout)
    //   0x9000_1xxx  UART
    //   0x9000_2xxx  Timer
    // ===================================================================
    logic [31:0] mmio_mem_addr, mmio_mem_wdata, mmio_mem_rdata;
    logic [3:0]  mmio_mem_wstrb;
    logic        mmio_mem_we, mmio_mem_req, mmio_mem_ready;

    axi_slave_wrapper mmio_slave (
        .clk(clk), .rst(bus_rst_r),
        .s_awaddr  (axi_s_awaddr [32*axi_pkg::SLAVE_MMIO +: 32]),
        .s_wdata   (axi_s_wdata  [32*axi_pkg::SLAVE_MMIO +: 32]),
        .s_wstrb   (axi_s_wstrb  [4*axi_pkg::SLAVE_MMIO  +: 4]),
        .s_awvalid (axi_s_awvalid[axi_pkg::SLAVE_MMIO]),
        .s_wvalid  (axi_s_wvalid[axi_pkg::SLAVE_MMIO]),
        .s_awready (axi_s_awready[axi_pkg::SLAVE_MMIO]),
        .s_bresp   (axi_s_bresp  [2*axi_pkg::SLAVE_MMIO  +: 2]),
        .s_bvalid  (axi_s_bvalid [axi_pkg::SLAVE_MMIO]),
        .s_bready  (axi_s_bready [axi_pkg::SLAVE_MMIO]),
        .s_araddr  (axi_s_araddr [32*axi_pkg::SLAVE_MMIO +: 32]),
        .s_arvalid (axi_s_arvalid[axi_pkg::SLAVE_MMIO]),
        .s_arready (axi_s_arready[axi_pkg::SLAVE_MMIO]),
        .s_rdata   (axi_s_rdata  [32*axi_pkg::SLAVE_MMIO +: 32]),
        .s_rresp   (axi_s_rresp  [2*axi_pkg::SLAVE_MMIO  +: 2]),
        .s_rvalid  (axi_s_rvalid [axi_pkg::SLAVE_MMIO]),
        .s_rready  (axi_s_rready [axi_pkg::SLAVE_MMIO]),
        .mem_addr  (mmio_mem_addr),
        .mem_wdata (mmio_mem_wdata),
        .mem_rdata (mmio_mem_rdata),
        .mem_wstrb (mmio_mem_wstrb),
        .mem_we    (mmio_mem_we),
        .mem_req   (mmio_mem_req),
        .mem_ready (mmio_mem_ready)
    );

    // ---------------------------------------------------------------
    // MMIO Sub-Decoder
    // Route to Core MMIO, UART, or Timer based on addr[15:12]
    // ---------------------------------------------------------------
    logic [3:0] mmio_sub_sel;
    assign mmio_sub_sel = mmio_mem_addr[15:12];
    
    logic mmio_sel_core;
    assign mmio_sel_core = (mmio_sub_sel == axi_pkg::MMIO_SUB_CORE);
    logic mmio_sel_uart;
    assign mmio_sel_uart = (mmio_sub_sel == axi_pkg::MMIO_SUB_UART);
    logic mmio_sel_timer;
    assign mmio_sel_timer = (mmio_sub_sel == axi_pkg::MMIO_SUB_TIMER);

    // --- Core MMIO (LEDs, Buttons, Thermal Readout) ---
    logic [7:0] led_reg;
    logic [31:0] core_mmio_rdata;

    always_ff @(posedge clk) begin
        migration_start_pulse <= 1'b0;
        migration_clear_halt_pulse <= 1'b0;
        eval_mode_we <= 1'b0;
        eval_run_done_pulse <= 1'b0;

        if (bus_rst_r) begin
            led_reg <= 8'b0;
            migration_src_sel <= 3'd0;
            migration_dst_sel <= 3'd1;
            eval_mode_wdata <= 2'd3;
        end else begin
            if (mmio_mem_req && mmio_mem_we && mmio_sel_core) begin
                case (mmio_mem_addr[7:0])
                    8'h00: led_reg <= mmio_mem_wdata[7:0];
                    8'h50: begin
                        migration_src_sel <= mmio_mem_wdata[10:8];
                        migration_dst_sel <= mmio_mem_wdata[14:12];
                        migration_start_pulse <= mmio_mem_wdata[0];
                        migration_clear_halt_pulse <= mmio_mem_wdata[1];
                    end
                    8'h70: begin
                        eval_mode_wdata <= mmio_mem_wdata[1:0];
                        eval_mode_we <= 1'b1;
                    end
                    8'hC0: begin
                        eval_run_done_pulse <= mmio_mem_wdata[0];
                    end
                    default: begin
                    end
                endcase
            end

            if (dpad_mode_we) begin
                eval_mode_wdata <= dpad_mode_wdata;
                eval_mode_we <= 1'b1;
            end
        end
    end

    always_comb begin
        case (mmio_mem_addr[7:0])
            8'h00:   core_mmio_rdata = {24'b0, led_reg};
            8'h04:   core_mmio_rdata = {28'b0, buttons};
            8'h20:   core_mmio_rdata = t_sensor_fused[sw];  // Selected core temperature (fused with XADC)
            8'h24:   core_mmio_rdata = t_estimated[sw];     // Kalman estimate
            8'h28:   core_mmio_rdata = t_predicted[sw];     // Kalman prediction
            8'h2C:   core_mmio_rdata = {30'b0, thermal_state[sw]};
            8'h30:   core_mmio_rdata = {30'b0, phase[sw]};
            8'h34:   core_mmio_rdata = {27'b0, core_clk_en};
            8'h38:   core_mmio_rdata = {27'b0, thermal_irq};
            8'h3C:   core_mmio_rdata = xadc_temp_q16_16;    // XADC die temperature (Q16.16)
            8'h40:   core_mmio_rdata = {31'b0, xadc_temp_valid};
            8'h44:   core_mmio_rdata = {16'b0, xadc_temp_raw};
            8'h50:   core_mmio_rdata = {17'b0, migration_dst_sel, 1'b0, migration_src_sel, 6'b0, 1'b0, 1'b0};
            8'h54:   core_mmio_rdata = {26'b0, migration_error, migration_done, migration_busy, 3'b0};
            8'h58:   core_mmio_rdata = migration_last_cycles;
            8'h5C:   core_mmio_rdata = migration_count;
            8'h60:   core_mmio_rdata = {27'b0, mig_halt_mask};
            8'h64:   core_mmio_rdata = {27'b0, mig_pause_mask};
            8'h68:   core_mmio_rdata = cache_warm_cycles;
            8'h6C:   core_mmio_rdata = cache_prefetch_count;
            8'h70:   core_mmio_rdata = {28'b0, eval_metrics_frozen, 1'b0, eval_active_mode};
            8'h74:   core_mmio_rdata = eval_total_cycles;
            8'h78:   core_mmio_rdata = eval_total_retired;
            8'h7C:   core_mmio_rdata = eval_peak_temperature;
            8'h80:   core_mmio_rdata = eval_avg_temperature;
            8'h84:   core_mmio_rdata = eval_throttle_events;
            8'h88:   core_mmio_rdata = eval_throttle_cycles;
            8'h8C:   core_mmio_rdata = eval_migration_count;
            8'h90:   core_mmio_rdata = eval_migration_cycles;
            8'h94:   core_mmio_rdata = eval_power_gate_cycles;
            8'h98:   core_mmio_rdata = eval_thermal_violations;
            8'h9C:   core_mmio_rdata = eval_energy_estimate;
            8'hA0:   core_mmio_rdata = policy_cost_cycles;
            8'hA4:   core_mmio_rdata = policy_benefit_score;
            8'hA8:   core_mmio_rdata = {24'b0, policy_all_cores_hot, cache_warm_busy, cache_warm_done, power_gate_mask};
            8'hAC:   core_mmio_rdata = total_power_gate_cycles;
            8'hB0:   core_mmio_rdata = {26'b0, policy_dst_core, policy_src_core};
            8'hB4:   core_mmio_rdata = power_gate_events;
            8'hB8:   core_mmio_rdata = cache_prefetch_addr;
            8'hBC:   core_mmio_rdata = {30'b0, policy_imbalance_block, policy_dual_hot_mode};
            8'hC0:   core_mmio_rdata = {28'b0, eval_metrics_frozen, 1'b0, eval_active_mode};
            8'hC4:   core_mmio_rdata = policy_migration_hist_flat[(0*32) +: 32];
            8'hC8:   core_mmio_rdata = policy_migration_hist_flat[(1*32) +: 32];
            8'hCC:   core_mmio_rdata = policy_migration_hist_flat[(2*32) +: 32];
            8'hD0:   core_mmio_rdata = policy_migration_hist_flat[(3*32) +: 32];
            8'hD4:   core_mmio_rdata = policy_migration_hist_flat[(4*32) +: 32];
            8'hD8:   core_mmio_rdata = policy_recent_history_flat[31:0];
            8'hDC:   core_mmio_rdata = policy_recent_history_flat[63:32];
            8'hE0:   core_mmio_rdata = policy_recent_history_flat[95:64];
            8'hE4:   core_mmio_rdata = policy_recent_history_flat[127:96];
            8'hE8:   core_mmio_rdata = policy_recent_history_flat[159:128];
            8'hEC:   core_mmio_rdata = policy_recent_history_flat[191:160];
            8'hF0:   core_mmio_rdata = policy_recent_history_flat[223:192];
            8'hF4:   core_mmio_rdata = policy_recent_history_flat[255:224];
            default: core_mmio_rdata = 32'b0;
        endcase
    end

    // --- UART Sub-Bus ---
    // Create local AXI-like signals for UART within mmio region
    logic uart_aw_valid;
    assign uart_aw_valid = mmio_mem_req && mmio_mem_we && mmio_sel_uart;
    logic uart_ar_valid;
    assign uart_ar_valid = mmio_mem_req && !mmio_mem_we && mmio_sel_uart;
    logic [31:0] uart_rdata;
    logic uart_rvalid;
    logic uart_awready;
    
    logic mmio_uart_tx;
    
    UART uart_inst (
        .clk(clk), .rst_n(bus_rst_n),
        .s_axi_awaddr  ({20'b0, mmio_mem_addr[11:0]}),
        .s_axi_awvalid (uart_aw_valid),
        .s_axi_awready (uart_awready),
        .s_axi_wdata   (mmio_mem_wdata),
        .s_axi_wstrb   (mmio_mem_wstrb),
        .s_axi_wvalid  (uart_aw_valid),
        .s_axi_wready  (),
        .s_axi_bresp   (),
        .s_axi_bvalid  (),
        .s_axi_bready  (1'b1),
        .s_axi_araddr  ({20'b0, mmio_mem_addr[11:0]}),
        .s_axi_arvalid (uart_ar_valid),
        .s_axi_arready (),
        .s_axi_rdata   (uart_rdata),
        .s_axi_rresp   (),
        .s_axi_rvalid  (uart_rvalid),
        .s_axi_rready  (1'b1),
        .rx             (uart_rx),
        .tx             (mmio_uart_tx)
    );
    
    // Eval Framework UART TX takes over when metrics are frozen and dumping
    assign uart_tx = eval_metrics_frozen ? eval_uart_tx : mmio_uart_tx;

    // --- Timer Sub-Bus ---
    logic timer_aw_valid;
    assign timer_aw_valid = mmio_mem_req && mmio_mem_we && mmio_sel_timer;
    logic timer_ar_valid;
    assign timer_ar_valid = mmio_mem_req && !mmio_mem_we && mmio_sel_timer;
    logic [31:0] timer_rdata;
    logic timer_rvalid;
    
    Timer timer_inst (
        .clk(clk), .rst_n(bus_rst_n),
        .s_axi_awaddr  ({20'b0, mmio_mem_addr[11:0]}),
        .s_axi_awvalid (timer_aw_valid),
        .s_axi_awready (),
        .s_axi_wdata   (mmio_mem_wdata),
        .s_axi_wstrb   (mmio_mem_wstrb),
        .s_axi_wvalid  (timer_aw_valid),
        .s_axi_wready  (),
        .s_axi_bresp   (),
        .s_axi_bvalid  (),
        .s_axi_bready  (1'b1),
        .s_axi_araddr  ({20'b0, mmio_mem_addr[11:0]}),
        .s_axi_arvalid (timer_ar_valid),
        .s_axi_arready (),
        .s_axi_rdata   (timer_rdata),
        .s_axi_rresp   (),
        .s_axi_rvalid  (timer_rvalid),
        .s_axi_rready  (1'b1),
        .timer_irq     (timer_irq_global)
    );

    // --- MMIO Read Data Mux ---
    assign mmio_mem_rdata = mmio_sel_uart  ? uart_rdata  :
                            mmio_sel_timer ? timer_rdata  :
                            core_mmio_rdata;
    assign mmio_mem_ready = mmio_mem_req; // Single-cycle for all sub-peripherals


    // ===================================================================
    // SLAVE 4: MAC Accelerator
    // ===================================================================      
    MAC_Unit mac_accelerator (
        .clk(clk), .rst_n(bus_rst_n),
        .s_axi_awaddr  (axi_s_awaddr [32*axi_pkg::SLAVE_MAC +: 32]),
        .s_axi_wdata   (axi_s_wdata  [32*axi_pkg::SLAVE_MAC +: 32]),
        .s_axi_wstrb   (axi_s_wstrb  [4*axi_pkg::SLAVE_MAC  +: 4]),
        .s_axi_awvalid (axi_s_awvalid[axi_pkg::SLAVE_MAC]),
        .s_axi_wvalid  (axi_s_wvalid[axi_pkg::SLAVE_MAC]),
        .s_axi_awready (axi_s_awready[axi_pkg::SLAVE_MAC]),
        .s_axi_wready  (),
        .s_axi_bresp   (axi_s_bresp  [2*axi_pkg::SLAVE_MAC  +: 2]),
        .s_axi_bvalid  (axi_s_bvalid [axi_pkg::SLAVE_MAC]),
        .s_axi_bready  (axi_s_bready [axi_pkg::SLAVE_MAC]),
        .s_axi_araddr  (axi_s_araddr [32*axi_pkg::SLAVE_MAC +: 32]),
        .s_axi_arvalid (axi_s_arvalid[axi_pkg::SLAVE_MAC]),
        .s_axi_arready (axi_s_arready[axi_pkg::SLAVE_MAC]),
        .s_axi_rdata   (axi_s_rdata  [32*axi_pkg::SLAVE_MAC +: 32]),
        .s_axi_rresp   (axi_s_rresp  [2*axi_pkg::SLAVE_MAC  +: 2]),
        .s_axi_rvalid  (axi_s_rvalid [axi_pkg::SLAVE_MAC]),
        .s_axi_rready  (axi_s_rready [axi_pkg::SLAVE_MAC])
    );


    // ===================================================================
    // LED ASSIGNMENT (Hardware Debug Dashboard)
    // ===================================================================
    logic [23:0] retire_heartbeat_ctr;
    always_ff @(posedge clk) begin
        if (bus_rst_r)
            retire_heartbeat_ctr <= 24'd0;
        else
            retire_heartbeat_ctr <= retire_heartbeat_ctr + {23'd0, (|core_retired)};
    end
    logic retire_heartbeat;
    assign retire_heartbeat = retire_heartbeat_ctr[23];

    assign leds[1:0]      = thermal_state[0];  // 00: Normal, 01: Warn, 10: Critical
    assign leds[4:2]      = adjusted_qos[0];   // Dynamic AXI QoS priority
    assign leds[5]        = led_reg[0] | any_core_retired_seen | retire_heartbeat;
    assign leds[6]        = core_clk_en[0];     // Thermal controller clock gate
    assign leds[7]        = thermal_irq[0];     // Core trapped due to overheating


    // ===================================================================
    // 7-SEGMENT DISPLAY DRIVER (Thermal Dashboard)
    //
    // Layout:
    //   [D7][D6][D5][D4] . [D3][D2][D1][D0]
    //    Left 4 digits       Right 4 digits
    //    Core mask/status    Temperature hex
    //
    // Left digits:  Throttle mask [D7:D6] | Phase [D5] | ThermState [D4]
    // Right digits: Selected core temperature (integer part, hex)
    // Decimal point on D4 = any thermal_irq active
    //
    // If hang detected: display "dEAd" on right 4 digits
    // If L1 critical:   display "C0dE" on right 4 digits
    // ===================================================================
    
    // Hang detection with boot grace:
    // - grace window: 2^27 cycles (~1.34s at 100MHz)
    // - timeout:      2^31 cycles (~21.5s at 100MHz)
    // Watchdog uses retired-instruction progress so short pipeline-idle
    // phases do not falsely trip the display into "dEAd".
    localparam [30:0] HANG_TIMEOUT_MAX = 31'h7FFFFFFF;
    localparam [26:0] BOOT_GRACE_INIT  = 27'h7FFFFFF;
    logic [30:0] hang_counter;
    logic [26:0] boot_grace;
    logic        hang_detected;
    logic        seen_first_retire;
    logic [TOTAL_CORES-1:0] watchdog_arm_vec;
    assign watchdog_arm_vec = core_enable_runtime & core_clk_en;
    logic       any_core_watchdog_armed;
    assign any_core_watchdog_armed = |watchdog_arm_vec;
    logic       any_core_progress;
    assign any_core_progress = |core_retired;
    logic        any_core_active_now;
    logic        any_core_imem_req_now;
    logic        any_core_dmem_req_now;
    integer    wd_i;

    always_comb begin
        any_core_active_now   = 1'b0;
        any_core_imem_req_now = 1'b0;
        any_core_dmem_req_now = 1'b0;
        for (wd_i = 0; wd_i < TOTAL_CORES; wd_i = wd_i + 1) begin
            any_core_active_now   = any_core_active_now   | core_active[wd_i];
            any_core_imem_req_now = any_core_imem_req_now | imem_req[wd_i];
            any_core_dmem_req_now = any_core_dmem_req_now | dmem_req[wd_i];
        end
    end

    logic any_core_live_now;
    assign any_core_live_now = any_core_progress |
                             any_core_active_now |
                             any_core_imem_req_now |
                             any_core_dmem_req_now;
    
    always_ff @(posedge clk) begin
        if (bus_rst_r) begin
            hang_counter  <= 31'd0;
            boot_grace    <= BOOT_GRACE_INIT;
            hang_detected <= 1'b0;
            seen_first_retire <= 1'b0;
        end else begin
            if (any_core_progress)
                seen_first_retire <= 1'b1;

            if (boot_grace != 27'd0)
                boot_grace <= boot_grace - 27'd1;

            if ((boot_grace == 27'd0) && any_core_watchdog_armed && seen_first_retire) begin
                if (any_core_live_now) begin
                    hang_counter  <= 31'd0;
                    hang_detected <= 1'b0;
                end
                else if (hang_counter == HANG_TIMEOUT_MAX) begin
                    hang_detected <= 1'b1;
                end
                else begin
                    hang_counter <= hang_counter + 31'd1;
                end
            end
            else begin
                hang_counter  <= 31'd0;
                hang_detected <= 1'b0;
            end
        end
    end

    // Select which core's temperature to display (SW[1:0])
    logic [31:0] display_temp;
    assign display_temp = t_estimated[sw];
    logic [15:0] temp_integer;
    assign temp_integer = display_temp[31:16]; // Integer part of Q16.16
    
    // L1 critical halt on any core
    logic any_l1_critical;
    assign any_l1_critical = |thermal_irq;
    logic no_retire_startup_fail;
    assign no_retire_startup_fail = (!any_core_retired_seen) && (core_startup_force == 24'd0);

    // Blink thermal code so live temperature remains visible while alerting.
    logic [24:0] thermal_alert_blink_ctr;
    always_ff @(posedge clk) begin
        if (bus_rst_r)
            thermal_alert_blink_ctr <= 25'd0;
        else
            thermal_alert_blink_ctr <= thermal_alert_blink_ctr + 25'd1;
    end
    logic show_thermal_code;
    assign show_thermal_code = any_l1_critical && thermal_alert_blink_ctr[24];

    // Latch and display migration route (src -> dst) long enough to read
    // on board after each completed migration.
    localparam [26:0] MIGRATION_DISPLAY_HOLD_MAX = 27'd99_999_999; // ~1s @ 100MHz
    logic [26:0] migration_display_hold_ctr;
    logic        migration_display_active;
    logic [2:0]  migration_display_src_latched;
    logic [2:0]  migration_display_dst_latched;
    logic        migration_display_pending_valid;
    logic [2:0]  migration_display_pending_src;
    logic [2:0]  migration_display_pending_dst;

    always_ff @(posedge clk) begin
        if (bus_rst_r) begin
            migration_display_hold_ctr   <= 27'd0;
            migration_display_active     <= 1'b0;
            migration_display_src_latched <= 3'd0;
            migration_display_dst_latched <= 3'd0;
            migration_display_pending_valid <= 1'b0;
            migration_display_pending_src <= 3'd0;
            migration_display_pending_dst <= 3'd0;
        end else if (migration_done) begin
            if (!migration_display_active) begin
                migration_display_hold_ctr   <= 27'd0;
                migration_display_active     <= 1'b1;
                migration_display_src_latched <= migration_active_src;
                migration_display_dst_latched <= migration_active_dst;
            end else begin
                migration_display_pending_valid <= 1'b1;
                migration_display_pending_src <= migration_active_src;
                migration_display_pending_dst <= migration_active_dst;
            end
        end else if (migration_display_active) begin
            if (migration_display_hold_ctr == MIGRATION_DISPLAY_HOLD_MAX) begin
                migration_display_hold_ctr <= 27'd0;
                if (migration_display_pending_valid) begin
                    migration_display_src_latched <= migration_display_pending_src;
                    migration_display_dst_latched <= migration_display_pending_dst;
                    migration_display_pending_valid <= 1'b0;
                end else begin
                    migration_display_active   <= 1'b0;
                end
            end else begin
                migration_display_hold_ctr <= migration_display_hold_ctr + 27'd1;
            end
        end
    end
    
    // Build right 4 digits: temperature or error code
    // Migration overlay format: 5A<source><dest> (example: 5A13 means core1->core3)
    logic [15:0] right_digits_base;
    assign right_digits_base = show_thermal_code ? 16'hC0DE : temp_integer;
    logic [15:0] right_digits;
    assign right_digits = hang_detected   ? 16'hDEAD :
                               no_retire_startup_fail ? 16'h0BAD :
                               migration_display_active ? {4'h5, 4'hA, {1'b0, migration_display_src_latched}, {1'b0, migration_display_dst_latched}} :
                               right_digits_base;

    // Build left 4 digits: status dashboard
    // D7-D6: Core clock enable mask (5 bits, show lower 4 for hex digit pair)
    // D5:    Phase of selected core
    // D4:    Thermal state of selected core
    logic [3:0] left_d7;
    assign left_d7 = {3'b0, core_clk_en[3]};     // Cores 3 throttle
    logic [3:0] left_d6;                               // Cores 2,1,0 throttle + E-core
    assign left_d6 = {core_clk_en[2:0], 1'b0};
    logic [3:0] left_d5;                               // Phase of selected core
    assign left_d5 = {2'b0, phase[sw]};
    logic [3:0] left_d4;                               // Thermal state
    assign left_d4 = {2'b0, thermal_state[sw]};

    logic [31:0] seg_display_data_next;
    assign seg_display_data_next = {left_d7, left_d6, left_d5, left_d4, right_digits};
    
    // Decimal point: D4 indicates thermal IRQ, D1:D0 indicate migration src/dst overlay.
    logic [7:0] seg_dp_mask_next;
    assign seg_dp_mask_next = (any_l1_critical ? 8'b0001_0000 : 8'b0000_0000) |
                                  (migration_display_active ? 8'b0000_0011 : 8'b0000_0000);
    
    // Blanking: blank nothing (show all digits)
    logic [7:0] seg_blanking_next;
    assign seg_blanking_next = 8'b0000_0000;

    // Register display control to isolate thermal datapath from IOB timing.
    // Latch the visible content at a human-readable rate so rapidly changing
    // status/thermal values do not blur on the board display.
    logic [31:0] seg_display_data_r;
    logic [7:0]  seg_dp_mask_r;
    logic [7:0]  seg_blanking_r;
    localparam [26:0] DISPLAY_HOLD_MAX = 27'd99_999_999; // ~1s at 100MHz
    logic [26:0] display_hold_counter;
    logic       display_latch_tick;
    assign display_latch_tick = (display_hold_counter == DISPLAY_HOLD_MAX);

    always_ff @(posedge clk) begin
        if (bus_rst_r) begin
            seg_display_data_r <= 32'b0;
            seg_dp_mask_r      <= 8'b0;
            seg_blanking_r     <= 8'b0;
            display_hold_counter <= 25'd0;
        end else begin
            if (display_latch_tick) begin
                seg_display_data_r  <= seg_display_data_next;
                seg_dp_mask_r       <= seg_dp_mask_next;
                seg_blanking_r      <= seg_blanking_next;
                display_hold_counter <= 25'd0;
            end else begin
                display_hold_counter <= display_hold_counter + 25'd1;
            end
        end
    end

    Seven_Seg_Driver seg_driver (
        .clk          (clk),
        .rst_n        (bus_rst_n),
        .display_data (seg_display_data_r),
        .dp_mask      (seg_dp_mask_r),
        .blanking     (seg_blanking_r),
        .seg          (seg),
        .dp           (dp),
        .an           (an)
    );

endmodule
