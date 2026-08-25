// ============================================================================
// Prolepsis - Shared thermal and telemetry types
// ============================================================================
package prolepsis_pkg;

    localparam int unsigned Q_FRAC_BITS = 16;
    localparam logic [31:0] Q16_ONE = 32'h0001_0000;
    localparam logic [31:0] Q16_ZERO = 32'h0000_0000;

    typedef enum logic [1:0] {
        PHASE_COMPUTE  = 2'd0,
        PHASE_MEMORY   = 2'd1,
        PHASE_BALANCED = 2'd2,
        PHASE_IDLE     = 2'd3
    } phase_t;

    typedef enum logic [1:0] {
        THERMAL_NORMAL   = 2'd0,
        THERMAL_WARNING  = 2'd1,
        THERMAL_CRITICAL = 2'd2,
        THERMAL_INVALID  = 2'd3
    } thermal_state_t;

    typedef struct packed {
        logic [31:0] temperature_q16_16;
        logic [31:0] retired_delta;
        logic [31:0] cycle_delta;
        logic [31:0] icache_miss;
        logic [31:0] icache_hit;
    } core_telemetry_t;

    typedef struct packed {
        logic [31:0] estimated_q16_16;
        logic [31:0] predicted_q16_16;
        logic [31:0] uncertainty_q16_16;
        logic [31:0] innovation_q16_16;
        thermal_state_t state;
    } thermal_prediction_t;

endpackage
