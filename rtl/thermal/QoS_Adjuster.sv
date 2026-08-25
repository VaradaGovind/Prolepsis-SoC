`timescale 1ns / 1ps
module QoS_Adjuster (
    input  logic        clk,
    input  logic        rst_n,
    
    // Inputs from Thermal and Phase Detector
    input  logic [9:0]  thermal_state_flat, // 5 x 2-bit (0: Normal, 1: Warning, 2: Critical)
    input  logic [9:0]  phase_flat,         // 5 x 2-bit (0: COMPUTE, 1: MEMORY, 2: BALANCED, 3: IDLE)
    
    // Input Default QoS from Address Map
    input  logic [14:0] default_qos_flat,   // 5 x 3-bit
    
    // Output Adjusted QoS to Arbiter
    output logic [14:0] adjusted_qos_flat   // 5 x 3-bit
);

    genvar i;
    generate
        for (i = 0; i < 5; i = i + 1) begin : G_ADJUSTER
            logic [1:0] t_state;
            assign t_state = thermal_state_flat[i*2 +: 2];
            logic [1:0] t_phase;
            assign t_phase = phase_flat[i*2 +: 2];
            logic [2:0] def_qos;
            assign def_qos = default_qos_flat[i*3 +: 3];
            
            logic [2:0] qos_out;

            always_ff @(posedge clk or negedge rst_n) begin
                if (!rst_n) begin
                    qos_out <= 3'd0;
                end else begin
                    if (t_state == 2'd2) begin
                        qos_out <= (def_qos >= 3'd2) ? (def_qos - 3'd2) : 3'd0; 
                    end else if (t_state == 2'd1) begin
                        if (t_phase == 2'd0) begin
                            qos_out <= (def_qos >= 3'd2) ? (def_qos - 3'd2) : 3'd0;
                        end else begin
                            qos_out <= (def_qos >= 3'd1) ? (def_qos - 3'd1) : 3'd0;
                        end
                    end else if (t_state == 2'd0) begin
                        if (t_phase == 2'd1) begin
                            qos_out <= (def_qos <= 3'd6) ? (def_qos + 3'd1) : 3'd7;
                        end else begin
                            qos_out <= def_qos;
                        end
                    end else begin
                        qos_out <= def_qos;
                    end
                end
            end

            assign adjusted_qos_flat[i*3 +: 3] = qos_out;
        end
    endgenerate

endmodule
