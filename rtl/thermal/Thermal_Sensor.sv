// ============================================================================
// OrionRV - 3-Node Compact RC Thermal Sensor Model
//
// Implements a 3-node thermal model (junction, spreader, package) with
// configurable RC parameters and PVT sensor variation.
//
// All temperatures in Q16.16 fixed-point format.
// RC parameters are Q8.8 fixed-point (16-bit), used as scaling factors.
//
// Thermal update equations:
//   T_junc   += dt/C1 * (P_dynamic - (T_junc - T_spread) / R12)
//   T_spread += dt/C2 * ((T_junc - T_spread)/R12 - (T_spread - T_pkg)/R23)
//   T_pkg    += dt/C3 * ((T_spread - T_pkg)/R23 - (T_pkg - T_amb)/R3a)
//
// Implementation: multiply temperature deltas by inv_r (1/R in Q8.8),
// then scale by dt_over_c (dt/C in Q8.8). Result is >>>16 to normalize.
// ============================================================================

module Thermal_Sensor (
    input  logic        clk,
    input  logic        rst_n,
    
    // Configurable RC Parameters (Q8.8 fixed-point)
    input  logic [15:0] dt_over_c1,      // dt / C_junction
    input  logic [15:0] dt_over_c2,      // dt / C_spreader
    input  logic [15:0] dt_over_c3,      // dt / C_package
    input  logic [15:0] inv_r12,         // 1 / R_junc_to_spread
    input  logic [15:0] inv_r23,         // 1 / R_spread_to_pkg
    input  logic [15:0] inv_r3a,         // 1 / R_pkg_to_ambient
    
    // Core Activity Input (Proxy for P_dynamic, Q16.16)
    input  logic signed [31:0] p_dynamic,
    
    // Environment (Q16.16)
    input  logic signed [31:0] t_amb,
    
    // PVT Variation Configuration
    input  logic [15:0] sensor_bias,
    input  logic [15:0] sensor_noise_sigma,
    
    // Output (Q16.16)
    output logic [31:0] t_sensor
);

    // Q16.16 fixed-point for internal calculations
    logic signed [31:0] t_junc;
    logic signed [31:0] t_spread;
    logic signed [31:0] t_pkg;
    logic signed [31:0] t_sensor_reg;
    logic signed [47:0] dt_junc_reg, dt_spread_reg, dt_pkg_reg;

    // Pseudo-random noise generation (LFSR)
    logic [11:0] lfsr;
    always_ff @(posedge clk) begin
        if (!rst_n) begin
            lfsr <= 12'hACE;
        end else begin
            lfsr <= {lfsr[10:0], lfsr[11] ^ lfsr[10] ^ lfsr[3] ^ lfsr[0]};
        end
    end

    // Basic noise approximation (sign-extend LFSR)
    logic signed [31:0] noise;
    assign noise = {{20{lfsr[11]}}, lfsr};

    // --- Pipeline Stage 1: Calculate Heat Flows (Multiplication) ---
    // Added 2 additional pipeline stages to resolve wide multiplier timing violations
    logic signed [39:0] flow_r12_mul, flow_r23_mul, flow_r3a_mul;
    logic signed [39:0] flow_r12_s1,  flow_r23_s1,  flow_r3a_s1;
    logic signed [39:0] flow_r12_reg, flow_r23_reg, flow_r3a_reg;
    
    always_ff @(posedge clk) begin
        if (!rst_n) begin
            flow_r12_mul <= 40'b0;
            flow_r23_mul <= 40'b0;
            flow_r3a_mul <= 40'b0;
            
            flow_r12_s1  <= 40'b0;
            flow_r23_s1  <= 40'b0;
            flow_r3a_s1  <= 40'b0;
            
            flow_r12_reg <= 40'b0;
            flow_r23_reg <= 40'b0;
            flow_r3a_reg <= 40'b0;
        end else begin
            // Stage 1a: Multiply (use only needed bits [39:0])
            flow_r12_mul <= (t_junc - t_spread) * $signed({1'b0, inv_r12});
            flow_r23_mul <= (t_spread - t_pkg)  * $signed({1'b0, inv_r23});
            flow_r3a_mul <= (t_pkg - t_amb)     * $signed({1'b0, inv_r3a});
            
            // Stage 1b: Pipeline
            flow_r12_s1  <= flow_r12_mul;
            flow_r23_s1  <= flow_r23_mul;
            flow_r3a_s1  <= flow_r3a_mul;
            
            // Stage 1c: Registered Output for next stage 
            flow_r12_reg <= flow_r12_s1;
            flow_r23_reg <= flow_r23_s1;
            flow_r3a_reg <= flow_r3a_s1;
        end
    end

    // --- Pipeline Stage 2: Calculate Q and Apply Temperature Update ---
    logic signed [31:0] q_junc;
    assign q_junc = p_dynamic - flow_r12_reg[39:8];
    logic signed [31:0] q_spread;
    assign q_spread = flow_r12_reg[39:8] - flow_r23_reg[39:8];
    logic signed [31:0] q_pkg;
    assign q_pkg = flow_r23_reg[39:8] - flow_r3a_reg[39:8];

    // Combinational multiply for stage 2 (safe because they are not chained)
    logic signed [47:0] dt_junc;
    assign dt_junc = q_junc   * $signed({1'b0, dt_over_c1});
    logic signed [47:0] dt_spread;
    assign dt_spread = q_spread * $signed({1'b0, dt_over_c2});
    logic signed [47:0] dt_pkg;
    assign dt_pkg = q_pkg    * $signed({1'b0, dt_over_c3});

    // Register the stage-2 products so T update uses a short adder path.
    // Added 2 additional pipeline stages here as well to fix wide multiplier warnings.
    logic signed [47:0] dt_junc_mul, dt_spread_mul, dt_pkg_mul;
    logic signed [47:0] dt_junc_p1,  dt_spread_p1,  dt_pkg_p1;
    always_ff @(posedge clk) begin
        if (!rst_n) begin
            dt_junc_mul   <= 48'b0;
            dt_spread_mul <= 48'b0;
            dt_pkg_mul    <= 48'b0;
            
            dt_junc_p1    <= 48'b0;
            dt_spread_p1  <= 48'b0;
            dt_pkg_p1     <= 48'b0;
            
            dt_junc_reg   <= 48'b0;
            dt_spread_reg <= 48'b0;
            dt_pkg_reg    <= 48'b0;
        end else begin
            dt_junc_mul   <= dt_junc;
            dt_spread_mul <= dt_spread;
            dt_pkg_mul    <= dt_pkg;
            
            dt_junc_p1    <= dt_junc_mul;
            dt_spread_p1  <= dt_spread_mul;
            dt_pkg_p1     <= dt_pkg_mul;
            
            dt_junc_reg   <= dt_junc_p1;
            dt_spread_reg <= dt_spread_p1;
            dt_pkg_reg    <= dt_pkg_p1;
        end
    end

    // Add bias and noise for final sensor output
    logic signed [31:0] noise_scaled;
    assign noise_scaled = (noise * $signed({1'b0, sensor_noise_sigma})) >>> 12;

    always_ff @(posedge clk) begin
        if (!rst_n) begin
            t_junc   <= t_amb;
            t_spread <= t_amb;
            t_pkg    <= t_amb;
            t_sensor_reg <= t_amb;
        end else begin
            t_junc   <= t_junc   + dt_junc_reg[39:8];
            t_spread <= t_spread + dt_spread_reg[39:8];
            t_pkg    <= t_pkg    + dt_pkg_reg[39:8];
            t_sensor_reg <= t_junc + {{16{sensor_bias[15]}}, sensor_bias} + noise_scaled;
        end
    end

    assign t_sensor = t_sensor_reg;

endmodule
