// ============================================================================
// Prolepsis - Core telemetry interface
// ============================================================================
interface telemetry_if #(parameter int NUM_CORES = 5);
    logic [31:0] temperature_q16_16 [NUM_CORES];
    logic [31:0] retired_delta      [NUM_CORES];
    logic [31:0] cycle_delta        [NUM_CORES];
    logic [31:0] icache_miss        [NUM_CORES];
    logic [31:0] icache_hit         [NUM_CORES];

    modport producer (
        output temperature_q16_16, retired_delta, cycle_delta,
               icache_miss, icache_hit
    );

    modport consumer (
        input temperature_q16_16, retired_delta, cycle_delta,
              icache_miss, icache_hit
    );
endinterface
