`timescale 1ns / 1ps
module Thermal_Controller (
    input  logic        clk,
    input  logic        rst_n,
    
    // Core Array (4 P-cores + 1 E-core)
    input  logic [9:0]   core_id_flat,      // 5 x 2-bit
    input  logic [159:0] t_sensor_flat,     // 5 x 32-bit
    input  logic [159:0] t_predicted_flat,  // 5 x 32-bit
    input  logic [9:0]   phase_flat,        // 5 x 2-bit
    
    // MMIO Parameters
    input  logic [31:0] t_warn,
    input  logic [31:0] t_crit,
    input  logic [31:0] t_l1_cap,  // Layer 1 Hard Safety Bound
    input  logic [1:0]  l2_mode,   // 0:Base, 1:Reactive, 2:Predictive, 3:Full
    input  logic [31:0] dt_max,    // Max Thermal Delta
    
    // Output Throttle & QoS Flags
    output logic [4:0]   clk_en,
    output logic [159:0] power_budget_flat, // Dynamic budget per core
    output logic [4:0]   thermal_irq
);

    genvar i;
    generate
        for (i = 0; i < 5; i = i + 1) begin : G_CONTROLLER
            logic [1:0]  c_id;
            assign c_id = core_id_flat[i*2 +: 2];
            logic [31:0] t_sens;
            assign t_sens = t_sensor_flat[i*32 +: 32];
            logic [31:0] t_pred;
            assign t_pred = t_predicted_flat[i*32 +: 32];
            logic [1:0]  c_phase;
            assign c_phase = phase_flat[i*2 +: 2];
            
            // -----------------------------------------------------
            // LAYER 1: SAFETY (Combinational)
            // -----------------------------------------------------
            logic l1_throttle;
            assign l1_throttle = (t_sens > t_l1_cap);
            logic l1_halt;
            assign l1_halt = (t_sens > t_crit);
            
            // -----------------------------------------------------
            // LAYER 2: OPTIMIZATION (Sequential)
            // -----------------------------------------------------
            logic [1:0] l2_throttle_level;
            
            always_ff @(posedge clk or negedge rst_n) begin
                if (!rst_n) begin
                    l2_throttle_level <= 2'b00; // 100%
                end else begin
                    case (l2_mode)
                        2'd1: begin // Reactive
                            if (t_sens > t_warn) l2_throttle_level <= 2'b10; // 50%
                            else l2_throttle_level <= 2'b00; // 100%
                        end
                        2'd2: begin // Predictive (Kalman)
                            if (t_pred > t_warn) l2_throttle_level <= 2'b01; // 75%
                            else l2_throttle_level <= 2'b00; // 100%
                        end
                        2'd3: begin // Full System (Workload Aware)
                            if (t_pred > t_warn && c_phase == 2'd0) begin
                                l2_throttle_level <= 2'b11; // 25% (Aggressive on COMPUTE)
                            end else if (t_pred > t_warn && c_phase == 2'd1) begin
                                l2_throttle_level <= 2'b01; // 75% (Gentle on MEMORY)
                            end else begin
                                l2_throttle_level <= 2'b00; // 100%
                            end
                        end
                        default: l2_throttle_level <= 2'b00; // 100% Base
                    endcase
                end
            end
            
            // -----------------------------------------------------
            // DUTY CYCLE MAPPER
            // -----------------------------------------------------
            logic [3:0] throttle_counter;
            always_ff @(posedge clk or negedge rst_n) begin
                if (!rst_n)
                    throttle_counter <= 4'd0;
                else
                    throttle_counter <= throttle_counter + 4'd1;
            end
            
            logic l2_clk_en;
            assign l2_clk_en = (l2_throttle_level == 2'b00) ? 1'b1 :
                             (l2_throttle_level == 2'b01) ? (throttle_counter[1:0] != 2'b11) : // 75% 3 of 4
                             (l2_throttle_level == 2'b10) ? throttle_counter[0] : // 50% 1 of 2
                             (l2_throttle_level == 2'b11) ? (throttle_counter[1:0] == 2'b00) : 1'b0; // 25% 1 of 4
                             
            // COMBINE L1 & L2 (L1 overrides)
            logic clk_en_comb;
            assign clk_en_comb = l1_halt ? 1'b0 :
                                 l1_throttle ? (throttle_counter[1:0] == 2'b00) : // L1 forces 25%
                                 l2_clk_en;
                               
            logic thermal_irq_comb;
            assign thermal_irq_comb = l1_halt | l1_throttle; // Dedicated highest priority thermal IRQ
            
            // Dynamic Budget Distribution (bounded by dt_max)
            logic [31:0] headroom_w;
            assign headroom_w = (t_warn > t_sens) ? (t_warn - t_sens) : 32'b0;
            logic [31:0] power_budget_comb;
            assign power_budget_comb = (headroom_w > dt_max) ? dt_max : headroom_w;

            // Register outputs to break massive critical path from thermal sensor to core pipelines
            logic clk_en_reg;
            logic thermal_irq_reg;
            logic [31:0] power_budget_reg;
            (* keep = "true" *) logic [1:0] c_id_shadow;

            always_ff @(posedge clk or negedge rst_n) begin
                if (!rst_n) begin
                    clk_en_reg <= 1'b1;
                    thermal_irq_reg <= 1'b0;
                    power_budget_reg <= 32'b0;
                    c_id_shadow <= 2'b0;
                end else begin
                    clk_en_reg <= clk_en_comb;
                    thermal_irq_reg <= thermal_irq_comb;
                    power_budget_reg <= power_budget_comb;
                    c_id_shadow <= c_id;
                end
            end

            assign clk_en[i] = clk_en_reg;
            assign thermal_irq[i] = thermal_irq_reg;
            assign power_budget_flat[i*32 +: 32] = power_budget_reg;
        end
    endgenerate

endmodule
