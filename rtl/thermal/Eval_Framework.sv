`timescale 1ns / 1ps
//============================================================================
// OrionRV - Comparative Evaluation Framework (Phase 5.6)
//
// Collects per-run metrics across management modes:
//   0 Baseline, 1 Reactive, 2 Predictive, 3 Full System
//============================================================================

module Eval_Framework #(
    parameter NUM_CORES = 5
)(
    input  logic                    clk,
    input  logic                    rst_n,

    input  logic [1:0]              mode_write_data,
    input  logic                    mode_write_en,
    input  logic                    run_done_pulse,

    input  logic [NUM_CORES-1:0]    core_retired,
    input  logic [NUM_CORES-1:0]    core_clk_en,
    input  logic [NUM_CORES-1:0]    power_gate_mask,
    input  logic [NUM_CORES*32-1:0] t_sensor_flat,
    input  logic [NUM_CORES*32-1:0] power_proxy_flat,
    input  logic [31:0]             t_crit,

    input  logic                    migration_done,
    input  logic [31:0]             migration_cycles_inc,

    output logic  [1:0]              active_mode,
    output logic                     metrics_frozen,

    output logic  [31:0]             total_cycles,
    output logic  [31:0]             total_retired,
    output logic  [31:0]             peak_temperature,
    output logic  [31:0]             avg_temperature,
    output logic  [31:0]             throttle_events,
    output logic  [31:0]             throttle_cycles,
    output logic  [31:0]             migration_count,
    output logic  [31:0]             migration_cycles,
    output logic  [31:0]             power_gate_cycles,
    output logic  [31:0]             thermal_violations,
    output logic  [31:0]             energy_estimate,
    
    // UART Auto-dumping
    output logic                    uart_tx
);

    logic [31:0] t0;
    assign t0 = t_sensor_flat[(0*32) +: 32];
    logic [31:0] t1;
    assign t1 = t_sensor_flat[(1*32) +: 32];
    logic [31:0] t2;
    assign t2 = t_sensor_flat[(2*32) +: 32];
    logic [31:0] t3;
    assign t3 = t_sensor_flat[(3*32) +: 32];
    logic [31:0] t4;
    assign t4 = t_sensor_flat[(4*32) +: 32];

    logic [31:0] p0;
    assign p0 = power_proxy_flat[(0*32) +: 32];
    logic [31:0] p1;
    assign p1 = power_proxy_flat[(1*32) +: 32];
    logic [31:0] p2;
    assign p2 = power_proxy_flat[(2*32) +: 32];
    logic [31:0] p3;
    assign p3 = power_proxy_flat[(3*32) +: 32];
    logic [31:0] p4;
    assign p4 = power_proxy_flat[(4*32) +: 32];

    logic [31:0] max01;
    assign max01 = (t0 > t1) ? t0 : t1;
    logic [31:0] max23;
    assign max23 = (t2 > t3) ? t2 : t3;
    logic [31:0] max0123;
    assign max0123 = (max01 > max23) ? max01 : max23;
    logic [31:0] max_temp;
    assign max_temp = (max0123 > t4) ? max0123 : t4;

    logic [32:0] temp_sum01;
    assign temp_sum01 = {1'b0, t0} + {1'b0, t1};
    logic [32:0] temp_sum23;
    assign temp_sum23 = {1'b0, t2} + {1'b0, t3};
    logic [33:0] temp_sum0123;
    assign temp_sum0123 = {1'b0, temp_sum01} + {1'b0, temp_sum23};
    logic [34:0] temp_sum_all;
    assign temp_sum_all = {1'b0, temp_sum0123} + {3'b0, t4};

    // Approximate sum/5 using sum*(1/8 + 1/16 + 1/64) = sum*0.203125.
    // This avoids a deep divide-by-5 cone while keeping eval trend fidelity.
    logic [31:0] temp_div8;
    assign temp_div8 = temp_sum_all[34:3];
    logic [31:0] temp_div16;
    assign temp_div16 = {1'b0, temp_sum_all[34:4]};
    logic [31:0] temp_div64;
    assign temp_div64 = {3'b0, temp_sum_all[34:6]};
    logic [31:0] avg_temp_sample;
    assign avg_temp_sample = temp_div8 + temp_div16 + temp_div64;

    logic [35:0] sum_power;
    assign sum_power = p0 + p1 + p2 + p3 + p4;
    logic [31:0] energy_inc;
    assign energy_inc = sum_power[35:4];

    logic [2:0] retired_count;
    assign retired_count = core_retired[0] + core_retired[1] + core_retired[2] + core_retired[3] + core_retired[4];
    logic [2:0] gated_count;
    assign gated_count = power_gate_mask[0] + power_gate_mask[1] + power_gate_mask[2] + power_gate_mask[3] + power_gate_mask[4];
    logic [2:0] throttled_count;
    assign throttled_count = (1'b0 + (~core_clk_en[0]) + (~core_clk_en[1]) + (~core_clk_en[2]) + (~core_clk_en[3]) + (~core_clk_en[4]));

    logic any_throttled;
    assign any_throttled = (core_clk_en != {NUM_CORES{1'b1}});
    logic any_thermal_violation;
    assign any_thermal_violation = (t0 > t_crit) || (t1 > t_crit) || (t2 > t_crit) || (t3 > t_crit) || (t4 > t_crit);

    logic any_throttled_d;
    (* DONT_TOUCH = "TRUE" *) logic [31:0] max_temp_sample_r;
    (* DONT_TOUCH = "TRUE" *) logic [31:0] avg_temp_sample_r;
    logic        temp_sample_valid_r;

    always_ff @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            active_mode         <= 2'd3;
            metrics_frozen      <= 1'b0;
            total_cycles        <= 32'd0;
            total_retired       <= 32'd0;
            peak_temperature    <= 32'd0;
            avg_temperature     <= 32'd0;
            throttle_events     <= 32'd0;
            throttle_cycles     <= 32'd0;
            migration_count     <= 32'd0;
            migration_cycles    <= 32'd0;
            power_gate_cycles   <= 32'd0;
            thermal_violations  <= 32'd0;
            energy_estimate     <= 32'd0;
            any_throttled_d     <= 1'b0;
            max_temp_sample_r   <= 32'd0;
            avg_temp_sample_r   <= 32'd0;
            temp_sample_valid_r <= 1'b0;
        end else begin
            if (mode_write_en) begin
                active_mode         <= mode_write_data;
                metrics_frozen      <= 1'b0;
                total_cycles        <= 32'd0;
                total_retired       <= 32'd0;
                peak_temperature    <= 32'd0;
                avg_temperature     <= 32'd0;
                throttle_events     <= 32'd0;
                throttle_cycles     <= 32'd0;
                migration_count     <= 32'd0;
                migration_cycles    <= 32'd0;
                power_gate_cycles   <= 32'd0;
                thermal_violations  <= 32'd0;
                energy_estimate     <= 32'd0;
                any_throttled_d     <= 1'b0;
                max_temp_sample_r   <= 32'd0;
                avg_temp_sample_r   <= 32'd0;
                temp_sample_valid_r <= 1'b0;
            end else begin
                if (run_done_pulse)
                    metrics_frozen <= 1'b1;

                if (!metrics_frozen) begin
                    max_temp_sample_r <= max_temp;
                    avg_temp_sample_r <= avg_temp_sample;
                    temp_sample_valid_r <= 1'b1;

                    total_cycles <= total_cycles + 32'd1;
                    total_retired <= total_retired + retired_count;

                    if (temp_sample_valid_r && (max_temp_sample_r > peak_temperature))
                        peak_temperature <= max_temp_sample_r;

                    if (temp_sample_valid_r)
                        avg_temperature <= avg_temp_sample_r;

                    if (any_throttled && !any_throttled_d)
                        throttle_events <= throttle_events + 32'd1;

                    throttle_cycles <= throttle_cycles + throttled_count;
                    power_gate_cycles <= power_gate_cycles + gated_count;
                    energy_estimate <= energy_estimate + energy_inc;

                    if (any_thermal_violation)
                        thermal_violations <= thermal_violations + 32'd1;

                    if (migration_done) begin
                        migration_count <= migration_count + 32'd1;
                        migration_cycles <= migration_cycles + migration_cycles_inc;
                    end
                end else begin
                    temp_sample_valid_r <= 1'b0;
                end

                any_throttled_d <= any_throttled;
            end
        end
    end

    // ===================================================================
    // UART Auto-Dumping FSM
    // ===================================================================
    localparam integer CLKS_PER_BIT = 868;

    logic [3:0] uart_state;
    logic [3:0] dump_idx;
    logic [31:0] dump_data;
    logic [2:0] nibble_idx;
    logic [7:0] tx_char;
    logic tx_start, tx_busy, tx_reg;
    logic [3:0] tx_bit_idx;
    logic [9:0] tx_shift;
    logic [15:0] tx_baud_ctr;
    
    assign uart_tx = tx_reg;

    logic [3:0] cur_nibble;
    assign cur_nibble = dump_data[((7-nibble_idx)*4) +: 4];
    logic [7:0] hex_char;
    assign hex_char = (cur_nibble < 10) ? (8'h30 + cur_nibble) : (8'h41 + cur_nibble - 10);

    always_ff @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            uart_state <= 4'd0;
            dump_idx <= 4'd0;
            nibble_idx <= 3'd0;
            tx_start <= 1'b0;
            tx_busy <= 1'b0;
            tx_reg <= 1'b1;
            tx_bit_idx <= 4'd0;
            tx_shift <= 10'h3FF;
            tx_baud_ctr <= 16'd0;
            dump_data <= 32'd0;
            tx_char <= 8'd0;
        end else begin
            tx_start <= 1'b0;
            
            // FSM
            case (uart_state)
                0: begin // IDLE
                    if (run_done_pulse) begin
                        uart_state <= 1;
                        dump_idx <= 0;
                        nibble_idx <= 0;
                    end
                end
                1: begin // LOAD_DATA
                    case (dump_idx)
                        0: dump_data <= total_cycles;
                        1: dump_data <= total_retired;
                        2: dump_data <= peak_temperature;
                        3: dump_data <= avg_temperature;
                        4: dump_data <= throttle_events;
                        5: dump_data <= throttle_cycles;
                        6: dump_data <= migration_count;
                        7: dump_data <= power_gate_cycles;
                        8: dump_data <= thermal_violations;
                        9: dump_data <= energy_estimate;
                        default: dump_data <= 32'h0;
                    endcase
                    uart_state <= 2;
                end
                2: begin // PREP_CHAR
                    tx_char <= hex_char;
                    uart_state <= 3;
                end
                3: begin // START_TX
                    if (!tx_busy) begin
                        tx_start <= 1'b1;
                        uart_state <= 4;
                    end
                end
                4: begin // WAIT_TX
                    if (!tx_busy && !tx_start) begin
                        if (nibble_idx == 7) begin
                            nibble_idx <= 0;
                            uart_state <= 5;
                        end else begin
                            nibble_idx <= nibble_idx + 1;
                            uart_state <= 2;
                        end
                    end
                end
                5: begin // NEWLINE / NEXT
                    if (!tx_busy) begin
                        tx_char <= 8'h0A; // \n
                        tx_start <= 1'b1;
                        uart_state <= 6;
                    end
                end
                6: begin // WAIT_NEWLINE
                    if (!tx_busy && !tx_start) begin
                        if (dump_idx == 9) begin
                            uart_state <= 0; // DONE
                        end else begin
                            dump_idx <= dump_idx + 1;
                            uart_state <= 1;
                        end
                    end
                end
            endcase

            // UART TX ENGINE
            if (!tx_busy) begin
                tx_reg <= 1'b1;
                tx_baud_ctr <= 16'd0;
                tx_bit_idx <= 4'd0;
                if (tx_start) begin
                    tx_busy <= 1'b1;
                    tx_shift <= {1'b1, tx_char, 1'b0};
                    tx_reg <= 1'b0;
                end
            end else begin
                if (tx_baud_ctr == (CLKS_PER_BIT - 1)) begin
                    tx_baud_ctr <= 16'd0;
                    tx_bit_idx <= tx_bit_idx + 4'd1;
                    tx_shift <= {1'b1, tx_shift[9:1]};
                    tx_reg <= tx_shift[1];
                    if (tx_bit_idx == 4'd9) begin
                        tx_busy <= 1'b0;
                        tx_reg <= 1'b1;
                    end
                end else begin
                    tx_baud_ctr <= tx_baud_ctr + 16'd1;
                end
            end
        end
    end

endmodule
