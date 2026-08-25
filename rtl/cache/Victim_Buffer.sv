`timescale 1ns / 1ps
//============================================================================
// OrionRV — L1D Victim Buffer (Phase 7.5)
//
// 2-entry fully-associative victim buffer for the direct-mapped L1 data
// cache. Catches conflict misses: when a cache line is evicted, it goes
// here. On a subsequent L1D miss, the victim buffer is checked first —
// a hit triggers a swap (victim → L1D, evictee → victim) with no memory
// access, dramatically cutting conflict-miss penalty.
//
// Coherence: snoop invalidation also checks and clears victim entries.
//============================================================================

module Victim_Buffer #(
    parameter NUM_ENTRIES  = 2,
    parameter LINE_WORDS   = 4,
    parameter TAG_WIDTH    = 20,
    parameter INDEX_WIDTH  = 8
)(
    input  logic        clk,
    input  logic        rst,

    // ------------------------------------------------------------------
    // Eviction port (from Data_Cache on line replacement)
    // ------------------------------------------------------------------
    input  logic                       evict_valid,
    input  logic [TAG_WIDTH-1:0]       evict_tag,
    input  logic [INDEX_WIDTH-1:0]     evict_index,
    input  logic [LINE_WORDS*32-1:0]   evict_data,

    // ------------------------------------------------------------------
    // Lookup port (from Data_Cache on miss, before going to memory)
    // ------------------------------------------------------------------
    input  logic                       lookup_valid,
    input  logic [TAG_WIDTH-1:0]       lookup_tag,
    input  logic [INDEX_WIDTH-1:0]     lookup_index,
    output logic                       lookup_hit,
    output logic [LINE_WORDS*32-1:0]   lookup_data,

    // ------------------------------------------------------------------
    // Swap-in port (after victim hit, the L1D evictee goes here)
    // ------------------------------------------------------------------
    input  logic                       swap_valid,
    input  logic [TAG_WIDTH-1:0]       swap_tag,
    input  logic [INDEX_WIDTH-1:0]     swap_index,
    input  logic [LINE_WORDS*32-1:0]   swap_data,

    // ------------------------------------------------------------------
    // Coherence snoop
    // ------------------------------------------------------------------
    input  logic [TAG_WIDTH-1:0]       snoop_tag,
    input  logic [INDEX_WIDTH-1:0]     snoop_index,
    input  logic                       snoop_valid,

    // ------------------------------------------------------------------
    // Flush
    // ------------------------------------------------------------------
    input  logic                       flush_all
);

    // Storage
    logic                      valid  [0:NUM_ENTRIES-1];
    logic [TAG_WIDTH-1:0]      tag    [0:NUM_ENTRIES-1];
    logic [INDEX_WIDTH-1:0]    idx    [0:NUM_ENTRIES-1];
    logic [LINE_WORDS*32-1:0]  data   [0:NUM_ENTRIES-1];

    // FIFO replacement pointer
    logic [$clog2(NUM_ENTRIES)-1:0] replace_ptr;

    // Lookup — fully associative search
    logic match0;
    assign match0 = valid[0] && (tag[0] == lookup_tag) && (idx[0] == lookup_index);
    logic match1;
    assign match1 = valid[1] && (tag[1] == lookup_tag) && (idx[1] == lookup_index);

    assign lookup_hit  = lookup_valid && (match0 || match1);
    assign lookup_data = match0 ? data[0] :
                         match1 ? data[1] :
                         {(LINE_WORDS*32){1'b0}};

    // Snoop match
    logic snoop_match0;
    assign snoop_match0 = snoop_valid && valid[0] &&
                        (tag[0] == snoop_tag) && (idx[0] == snoop_index);
    logic snoop_match1;
    assign snoop_match1 = snoop_valid && valid[1] &&
                        (tag[1] == snoop_tag) && (idx[1] == snoop_index);

    integer i;

    always_ff @(posedge clk) begin
        if (rst || flush_all) begin
            replace_ptr <= 0;
            for (i = 0; i < NUM_ENTRIES; i = i + 1) begin
                valid[i] <= 1'b0;
                tag[i]   <= {TAG_WIDTH{1'b0}};
                idx[i]   <= {INDEX_WIDTH{1'b0}};
                data[i]  <= {(LINE_WORDS*32){1'b0}};
            end
        end
        else begin
            // Snoop invalidation
            if (snoop_match0) valid[0] <= 1'b0;
            if (snoop_match1) valid[1] <= 1'b0;

            // Swap-in: after a victim hit, the L1D's evicted line replaces
            // the victim entry that was just consumed.
            if (swap_valid) begin
                if (match0) begin
                    tag[0]  <= swap_tag;
                    idx[0]  <= swap_index;
                    data[0] <= swap_data;
                    // valid stays 1
                end
                else if (match1) begin
                    tag[1]  <= swap_tag;
                    idx[1]  <= swap_index;
                    data[1] <= swap_data;
                end
            end
            // Eviction: cache pushes an evicted line here (FIFO)
            else if (evict_valid) begin
                valid[replace_ptr] <= 1'b1;
                tag[replace_ptr]   <= evict_tag;
                idx[replace_ptr]   <= evict_index;
                data[replace_ptr]  <= evict_data;
                replace_ptr        <= replace_ptr + 1'b1;
            end
        end
    end

endmodule
