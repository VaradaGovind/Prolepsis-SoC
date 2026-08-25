`timescale 1ns / 1ps
//============================================================================
// OrionRV - Per-Core Power Gate Controller (Phase 5.4)
//
// Identifies long-idle cores and places them into a power-gated state.
//============================================================================

module Power_Gate_Controller #(
    parameter NUM_CORES = 5,
    parameter IDLE_THRESHOLD = 12'd2048,
    parameter [7:0] WAKE_LATENCY = 8'd16
)(
    input  logic                    clk,
    input  logic                    rst_n,
    input  logic                    enable,

    input  logic [NUM_CORES*2-1:0]  phase_flat,
    input  logic [NUM_CORES*2-1:0]  thermal_state_flat,
    input  logic [NUM_CORES-1:0]    base_core_enable,

    input  logic                    migration_req,
    input  logic                    migration_busy,
    input  logic [2:0]              migration_dst_core,

    output logic  [NUM_CORES-1:0]    power_gate_mask,
    output logic  [NUM_CORES*32-1:0] pwr_gated_cycles_flat,
    output logic  [31:0]             total_pwr_gated_cycles,
    output logic  [31:0]             power_gate_events,

    output logic  [NUM_CORES-1:0]    pg_save_req,
    output logic  [NUM_CORES-1:0]    pg_restore_req,
    input  logic [NUM_CORES-1:0]    pg_ack
);

    localparam [3:0] ST_ACTIVE      = 4'd0;
    localparam [3:0] ST_THROTTLED   = 4'd1;
    localparam [3:0] ST_CLOCK_GATED = 4'd2;
    localparam [3:0] ST_SAVE_WAIT   = 4'd3;
    localparam [3:0] ST_POWER_GATED = 4'd4;
    localparam [3:0] ST_WAKE_WAIT   = 4'd5;
    localparam [3:0] ST_RESTORE_WAIT= 4'd6;

    localparam [11:0] THROTTLE_ENTRY   = (IDLE_THRESHOLD >> 2);
    localparam [11:0] CLOCK_GATE_ENTRY = (IDLE_THRESHOLD >> 1);

    integer i;
    integer gc;

    logic [11:0] idle_counter [0:NUM_CORES-1];
    logic [7:0]  wake_counter [0:NUM_CORES-1];
    logic [31:0] gated_cycles [0:NUM_CORES-1];
    logic [3:0]  pwr_state    [0:NUM_CORES-1];

    logic [3:0]  gated_count_now;
    logic [1:0]  phase_i;
    logic [1:0]  therm_i;
    logic        idle_cond;
    logic        wake_req;

    always_comb begin
        gated_count_now = 4'd0;
        for (gc = 0; gc < NUM_CORES; gc = gc + 1) begin
            if ((pwr_state[gc] == ST_POWER_GATED) || (pwr_state[gc] == ST_WAKE_WAIT))
                gated_count_now = gated_count_now + 4'd1;
        end
    end

    always_ff @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            total_pwr_gated_cycles <= 32'd0;
            power_gate_events <= 32'd0;
            for (i = 0; i < NUM_CORES; i = i + 1) begin
                idle_counter[i] <= 12'd0;
                wake_counter[i] <= 8'd0;
                gated_cycles[i] <= 32'd0;
                pwr_state[i] <= ST_ACTIVE;
            end
        end else begin
            total_pwr_gated_cycles <= total_pwr_gated_cycles + gated_count_now;

            for (i = 0; i < NUM_CORES; i = i + 1) begin
                if ((pwr_state[i] == ST_POWER_GATED) || (pwr_state[i] == ST_WAKE_WAIT))
                    gated_cycles[i] <= gated_cycles[i] + 32'd1;

                phase_i = phase_flat[(i*2) +: 2];
                therm_i = thermal_state_flat[(i*2) +: 2];

                idle_cond = enable &&
                            base_core_enable[i] &&
                            (phase_i == 2'd3) &&
                            (therm_i != 2'd2) &&
                            !((migration_req || migration_busy) && (migration_dst_core == i[2:0]));

                wake_req = (!enable) ||
                           (!base_core_enable[i]) ||
                           (phase_i != 2'd3) ||
                           (therm_i == 2'd2) ||
                           ((migration_req || migration_busy) && (migration_dst_core == i[2:0]));

                if (idle_cond) begin
                    if (idle_counter[i] != 12'hFFF)
                        idle_counter[i] <= idle_counter[i] + 12'd1;
                end else begin
                    idle_counter[i] <= 12'd0;
                end

                case (pwr_state[i])
                    ST_ACTIVE: begin
                        wake_counter[i] <= 8'd0;
                        if (idle_counter[i] >= THROTTLE_ENTRY)
                            pwr_state[i] <= ST_THROTTLED;
                    end

                    ST_THROTTLED: begin
                        wake_counter[i] <= 8'd0;
                        if (wake_req)
                            pwr_state[i] <= ST_ACTIVE;
                        else if (idle_counter[i] >= CLOCK_GATE_ENTRY)
                            pwr_state[i] <= ST_CLOCK_GATED;
                    end

                    ST_CLOCK_GATED: begin
                        wake_counter[i] <= 8'd0;
                        if (wake_req)
                            pwr_state[i] <= ST_ACTIVE;
                        else if (idle_counter[i] >= IDLE_THRESHOLD) begin
                            pwr_state[i] <= ST_SAVE_WAIT;
                        end
                    end

                    ST_SAVE_WAIT: begin
                        if (pg_ack[i]) begin
                            pwr_state[i] <= ST_POWER_GATED;
                            power_gate_events <= power_gate_events + 32'd1;
                        end else if (wake_req) begin
                            pwr_state[i] <= ST_ACTIVE;
                        end
                    end

                    ST_POWER_GATED: begin
                        if (wake_req) begin
                            pwr_state[i] <= ST_WAKE_WAIT;
                            wake_counter[i] <= WAKE_LATENCY;
                        end
                    end

                    ST_WAKE_WAIT: begin
                        if (wake_counter[i] == 8'd0)
                            pwr_state[i] <= ST_RESTORE_WAIT;
                        else
                            wake_counter[i] <= wake_counter[i] - 8'd1;
                    end

                    ST_RESTORE_WAIT: begin
                        if (pg_ack[i]) begin
                            pwr_state[i] <= ST_ACTIVE;
                        end
                    end

                    default: begin
                        pwr_state[i] <= ST_ACTIVE;
                        wake_counter[i] <= 8'd0;
                    end
                endcase
            end
        end
    end

    always_comb begin
        power_gate_mask = {NUM_CORES{1'b0}};
        pwr_gated_cycles_flat = {(NUM_CORES*32){1'b0}};
        pg_save_req = {NUM_CORES{1'b0}};
        pg_restore_req = {NUM_CORES{1'b0}};

        for (i = 0; i < NUM_CORES; i = i + 1) begin
            if ((pwr_state[i] == ST_POWER_GATED) || (pwr_state[i] == ST_WAKE_WAIT) || (pwr_state[i] == ST_RESTORE_WAIT))
                power_gate_mask[i] = 1'b1;

            if (pwr_state[i] == ST_SAVE_WAIT)
                pg_save_req[i] = 1'b1;

            if (pwr_state[i] == ST_RESTORE_WAIT)
                pg_restore_req[i] = 1'b1;

            pwr_gated_cycles_flat[(i*32) +: 32] = gated_cycles[i];
        end
    end

endmodule
