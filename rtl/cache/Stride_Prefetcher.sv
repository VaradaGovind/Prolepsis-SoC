`timescale 1ns / 1ps
//============================================================================
// OrionRV — Stride Data Prefetcher (Phase 7.5)
//
// Lightweight stride detector with a 4-entry stride table.  Tracks miss
// address patterns per PC hash and issues conservative prefetch hints
// after 3 consecutive stride confirmations.
//
// Only prefetches one stride ahead, and only on stable patterns.
// Prefetch requests are hints — the data cache drops them if busy.
//============================================================================

module Stride_Prefetcher #(
    parameter TABLE_ENTRIES = 4
)(
    input  logic        clk,
    input  logic        rst,

    // ------------------------------------------------------------------
    // Miss notification (from Data_Cache on demand miss)
    // ------------------------------------------------------------------
    input  logic        miss_valid,       // A D-cache demand miss occurred
    input  logic [31:0] miss_addr,        // Address of the miss
    input  logic [31:0] miss_pc,          // PC of the load/store that missed

    // ------------------------------------------------------------------
    // Prefetch hint output (to Data_Cache)
    // ------------------------------------------------------------------
    output logic         pf_req,           // Prefetch request
    output logic  [31:0] pf_addr,          // Prefetch address

    // ------------------------------------------------------------------
    // Backpressure
    // ------------------------------------------------------------------
    input  logic        pf_ack            // Data cache accepted the prefetch
);

    localparam IDX_W = $clog2(TABLE_ENTRIES);  // 2
    localparam CONF_MAX = 2'd3;

    // Stride table
    logic [31:0] st_last_addr [0:TABLE_ENTRIES-1];
    logic [31:0] st_stride    [0:TABLE_ENTRIES-1];
    logic [1:0]  st_conf      [0:TABLE_ENTRIES-1];
    logic        st_valid     [0:TABLE_ENTRIES-1];
    logic [31:0] st_pc_tag    [0:TABLE_ENTRIES-1];  // Full PC for tag match

    // Index into table by PC hash
    logic [IDX_W-1:0] tbl_idx;
    assign tbl_idx = miss_pc[IDX_W+1:2];

    // Lookup
    logic tbl_hit;
    assign tbl_hit = st_valid[tbl_idx] && (st_pc_tag[tbl_idx] == miss_pc);

    integer i;

    always_ff @(posedge clk) begin
        if (rst) begin
            pf_req  <= 1'b0;
            pf_addr <= 32'b0;
            for (i = 0; i < TABLE_ENTRIES; i = i + 1) begin
                st_last_addr[i] <= 32'b0;
                st_stride[i]    <= 32'b0;
                st_conf[i]      <= 2'b0;
                st_valid[i]     <= 1'b0;
                st_pc_tag[i]    <= 32'b0;
            end
        end
        else begin
            // Clear prefetch request when acknowledged
            if (pf_ack)
                pf_req <= 1'b0;

            if (miss_valid) begin
                if (tbl_hit) begin
                    // Existing entry — check stride
                    if ((miss_addr - st_last_addr[tbl_idx]) == st_stride[tbl_idx]) begin
                        // Stride matches — increase confidence
                        if (st_conf[tbl_idx] != CONF_MAX)
                            st_conf[tbl_idx] <= st_conf[tbl_idx] + 2'b01;
                    end else begin
                        // Stride changed — record new stride, reset confidence
                        st_stride[tbl_idx] <= miss_addr - st_last_addr[tbl_idx];
                        st_conf[tbl_idx]   <= 2'b01;
                    end
                    st_last_addr[tbl_idx] <= miss_addr;

                    // Issue prefetch if confidence is at max
                    if (st_conf[tbl_idx] == CONF_MAX && !pf_req) begin
                        pf_req  <= 1'b1;
                        pf_addr <= miss_addr + st_stride[tbl_idx];
                    end
                end
                else begin
                    // New entry — allocate (evict current occupant)
                    st_valid[tbl_idx]     <= 1'b1;
                    st_pc_tag[tbl_idx]    <= miss_pc;
                    st_last_addr[tbl_idx] <= miss_addr;
                    st_stride[tbl_idx]    <= 32'b0;
                    st_conf[tbl_idx]      <= 2'b00;
                end
            end
        end
    end

endmodule
