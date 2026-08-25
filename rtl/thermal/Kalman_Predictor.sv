// ============================================================================
// OrionRV - Kalman Predictor (Pipelined / Synthesis-Safe)
//
// Replaces the original combinational triple-multiply + division design
// that created a ~120ns critical path (WNS = -110ns at 100MHz).
//
// This version uses:
//   - 4-stage pipeline for multiplications and gain/update logic
//   - Shift-based approximation for Kalman gain (avoids division)
//   - Registered gain/correction stage to reduce control-path depth
// ============================================================================

module Kalman_Predictor (
    input  logic        clk,
    input  logic        rst_n,
    
    // Sensor Input (Q16.16 format)
    input  logic [31:0] t_sensor,
    
    // Performance Proxy Input
    input  logic [31:0] power_proxy,
    
    // Configurable Parameters (MMIO)
    input  logic [31:0] k_f,      // State Transition Matrix F
    input  logic [31:0] k_b,      // Control-Input Model B
    input  logic [31:0] k_g,      // Environment Model G
    input  logic [31:0] k_q,      // Process Noise Covariance Q
    input  logic [31:0] k_r,      // Measurement Noise R
    input  logic [31:0] t_warn,   // Warning temp
    input  logic [31:0] t_crit,   // Critical temp
    input  logic [31:0] t_amb,    // Ambient temp
    
    // Output
    output logic [31:0] t_estimated,
    output logic [31:0] t_predicted,
    output logic [31:0] p_uncertainty,
    output logic [1:0]  thermal_state, // 0: Normal, 1: Warning, 2: Critical
    output logic [31:0] innovation
);

    // =================================================================
    // State registers
    // =================================================================
    logic [31:0] t_est;   // Current optimal estimate (Q16.16)
    logic [31:0] p_est;   // Current estimate covariance (Q16.16)

    // =================================================================
    // Pipeline Stage 1: Prediction multiplications (1 mul each)
    // =================================================================
    (* keep = "true" *) logic [47:0] pipe1_f_t_est;
    (* keep = "true" *) logic [47:0] pipe1_b_power;
    (* keep = "true" *) logic [47:0] pipe1_g_t_amb;
    (* keep = "true" *) logic [47:0] pipe1_f_p_est;   // k_f * p_est (first half of F*P*F')
    logic [31:0] pipe1_t_sensor;
    logic [31:0] pipe1_k_q;
    logic [31:0] pipe1_k_r;
    logic [31:0] pipe1_k_f;

    logic signed [31:0] signed_k_f;
    assign signed_k_f = k_f;
    logic signed [31:0] signed_t_est;
    assign signed_t_est = t_est;
    logic signed [31:0] signed_k_b;
    assign signed_k_b = k_b;
    logic signed [31:0] signed_power;
    assign signed_power = power_proxy;
    logic signed [31:0] signed_k_g;
    assign signed_k_g = k_g;
    logic signed [31:0] signed_t_amb;
    assign signed_t_amb = t_amb;
    logic signed [31:0] signed_p_est;
    assign signed_p_est = p_est;

    always_ff @(posedge clk) begin
        // Leave datapath pipeline regs without explicit reset so synthesis
        // does not route heavy reset logic into DSP control pins.
        pipe1_f_t_est  <= $signed(signed_k_f) * $signed(signed_t_est);
        pipe1_b_power  <= $signed(signed_k_b) * $signed(signed_power);
        pipe1_g_t_amb  <= $signed(signed_k_g) * $signed(signed_t_amb);
        pipe1_f_p_est  <= $signed(signed_k_f) * $signed(signed_p_est);
        pipe1_t_sensor <= t_sensor;
        pipe1_k_q      <= k_q;
        pipe1_k_r      <= k_r;
        pipe1_k_f      <= k_f;
    end

    // =================================================================
    // Pipeline Stage 2: Second multiply for P prediction + sums
    // =================================================================
    logic [31:0] pipe2_t_pred;
    // Store only the Q16.16-aligned covariance term used downstream.
    logic [31:0] pipe2_fp_f_q16; // (k_f * p_est * k_f) >>> 16
    logic [31:0] pipe2_t_sensor;
    logic [31:0] pipe2_k_q;
    logic [31:0] pipe2_k_r;

    always_ff @(posedge clk) begin
        // t_pred = F*t_est + B*power + G*t_amb  (all Q16.16)
        pipe2_t_pred   <= pipe1_f_t_est[47:16]
                        + pipe1_b_power[47:16]
                        + pipe1_g_t_amb[47:16];
        // Second half of F*P*F': multiply (k_f*p_est) by k_f and keep Q16.16.
        pipe2_fp_f_q16 <= ($signed(pipe1_f_p_est[47:16]) * $signed(pipe1_k_f)) >>> 16;
        pipe2_t_sensor <= pipe1_t_sensor;
        pipe2_k_q      <= pipe1_k_q;
        pipe2_k_r      <= pipe1_k_r;
    end

    // =================================================================
    // Pipeline Stage 3: Kalman gain/correction precompute
    // =================================================================
    // Instead of true division:  K = P_pred / (P_pred + R)
    // We use a shift-based approximation:
    //   ratio = P_pred / (P_pred + R)
    //   When P >> R:  ratio ≈ 1.0
    //   When P << R:  ratio ≈ 0
    //   When P ≈ R:   ratio ≈ 0.5
    // We approximate with leading-zero comparison.

    logic [31:0] p_pred_w;
    assign p_pred_w = pipe2_fp_f_q16 + pipe2_k_q;
    logic [31:0] s_err_w;
    assign s_err_w = p_pred_w + pipe2_k_r;
    logic [31:0] innov_w;
    assign innov_w = pipe2_t_sensor - pipe2_t_pred;

    logic [31:0] pipe3_t_pred;
    logic [31:0] pipe3_p_pred;
    logic [31:0] pipe3_s_err;
    logic [31:0] pipe3_innov;
    logic [3:0]  pipe_valid;

    // Stage-3 registered values drive the gain-shift comparator in stage 4.
    logic [15:0] gain_numerator_cmp;
    assign gain_numerator_cmp = pipe3_p_pred[31:16];
    logic [15:0] s_err_cmp;
    assign s_err_cmp = pipe3_s_err[31:16];
    logic [3:0]  gain_shift3_w;

    assign gain_shift3_w = (pipe3_s_err == 0)                 ? 4'd0 :
                           (gain_numerator_cmp >= s_err_cmp)        ? 4'd0 :  // K ≈ 1.0
                           (gain_numerator_cmp >= (s_err_cmp >> 1)) ? 4'd1 :  // K ≈ 0.5
                           (gain_numerator_cmp >= (s_err_cmp >> 2)) ? 4'd2 :  // K ≈ 0.25
                           (gain_numerator_cmp >= (s_err_cmp >> 3)) ? 4'd3 :  // K ≈ 0.125
                           (gain_numerator_cmp >= (s_err_cmp >> 4)) ? 4'd4 :  // K ≈ 0.0625
                                                               4'd5;   // K ≈ small

    logic [31:0] pipe4_t_pred;
    logic [31:0] pipe4_p_pred;
    logic [31:0] pipe4_innov;
    logic [3:0]  pipe4_gain_shift;

    always_ff @(posedge clk) begin
        if (!rst_n) begin
            pipe_valid      <= 4'b0000;
            pipe3_t_pred    <= 32'b0;
            pipe3_p_pred    <= 32'b0;
            pipe3_s_err     <= 32'b0;
            pipe3_innov     <= 32'b0;
            pipe4_t_pred    <= 32'b0;
            pipe4_p_pred    <= 32'b0;
            pipe4_innov     <= 32'b0;
            pipe4_gain_shift<= 4'b0;
        end else begin
            pipe_valid       <= {pipe_valid[2:0], 1'b1};
            pipe3_t_pred     <= pipe2_t_pred;
            pipe3_p_pred     <= p_pred_w;
            pipe3_s_err      <= s_err_w;
            pipe3_innov      <= innov_w;

            pipe4_t_pred     <= pipe3_t_pred;
            pipe4_p_pred     <= pipe3_p_pred;
            pipe4_innov      <= pipe3_innov;
            pipe4_gain_shift <= gain_shift3_w;
        end
    end

    logic signed [31:0] correction_w;
    assign correction_w = $signed(pipe4_innov) >>> pipe4_gain_shift;
    logic [31:0]        p_correction_w;
    assign p_correction_w = pipe4_p_pred >> pipe4_gain_shift;
    logic [31:0]        p_est_next_w;
    assign p_est_next_w = pipe4_p_pred - p_correction_w;

    always_ff @(posedge clk) begin
        if (!rst_n) begin
            t_est <= t_amb;
            p_est <= 32'h00010000;  // Q16.16 = 1.0
        end else if (pipe_valid[3]) begin
            t_est <= pipe4_t_pred + correction_w;
            p_est <= p_est_next_w;
        end
    end

    // =================================================================
    // Outputs (directly from registers — no combinational fan-out)
    // =================================================================
    assign t_estimated   = t_est;
    assign t_predicted   = pipe4_t_pred;
    assign p_uncertainty = p_est;
    assign innovation    = pipe4_innov;

    // Thermal state: compare predicted temperature against thresholds
    logic [1:0] thermal_state_reg;
    always_ff @(posedge clk) begin
        if (!rst_n) begin
            thermal_state_reg <= 2'd0;
        end else begin
            thermal_state_reg <= (pipe4_t_pred >= t_crit) ? 2'd2 :
                                 (pipe4_t_pred >= t_warn) ? 2'd1 : 2'd0;
        end
    end
    assign thermal_state = thermal_state_reg;

endmodule
