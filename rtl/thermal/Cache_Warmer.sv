`timescale 1ns / 1ps
//============================================================================
// OrionRV - Cross-Core Cache Warmer (Phase 5.5)
//
// Lightweight warming model that emits a bounded sequence of low-priority
// prefetch addresses prior to migration resume.
//============================================================================

module Cache_Warmer #(
    parameter DEFAULT_WARM_ENTRIES = 32'd16
)(
    input  logic        clk,
    input  logic        rst_n,
    input  logic        start,
    input  logic [2:0]  src_core,
    input  logic [2:0]  dst_core,
    input  logic [7:0]  src_pc,
    input  logic [31:0] warm_entries_cfg,

    output logic  [7:0]  inspect_index,
    input  logic        inspect_valid,
    input  logic [19:0] inspect_tag,

    output logic         busy,
    output logic         done,
    output logic  [31:0] cache_warm_cycles,
    output logic  [31:0] prefetch_count,
    output logic  [31:0] prefetch_addr,
    output logic         prefetch_valid
);

    logic [31:0] warm_target;
    logic [31:0] warm_ctr;
    logic [7:0]  current_index_q;

    always_ff @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            busy             <= 1'b0;
            done             <= 1'b0;
            cache_warm_cycles <= 32'd0;
            prefetch_count   <= 32'd0;
            prefetch_addr    <= 32'd0;
            prefetch_valid   <= 1'b0;
            warm_target      <= DEFAULT_WARM_ENTRIES;
            warm_ctr         <= 32'd0;
            inspect_index    <= 8'd0;
            current_index_q  <= 8'd0;
        end else begin
            done <= 1'b0;

            if (start && !busy) begin
                if (src_core == dst_core) begin
                    busy              <= 1'b0;
                    done              <= 1'b1;
                    cache_warm_cycles <= 32'd0;
                    prefetch_valid    <= 1'b0;
                    prefetch_count    <= 32'd0;
                end else begin
                    busy            <= 1'b1;
                    prefetch_valid  <= 1'b0;
                    warm_ctr        <= 32'd0;
                    prefetch_count  <= 32'd0;
                    warm_target     <= (warm_entries_cfg == 32'd0) ? DEFAULT_WARM_ENTRIES : warm_entries_cfg;
                    inspect_index   <= src_pc;
                    current_index_q <= src_pc;
                end
            end else if (busy) begin
                warm_ctr <= warm_ctr + 32'd1;
                
                if (inspect_valid) begin
                    prefetch_addr  <= {inspect_tag, current_index_q, 4'b0000};
                    prefetch_valid <= 1'b1;
                    prefetch_count <= prefetch_count + 32'd1;
                end else begin
                    prefetch_valid <= 1'b0;
                end
                
                inspect_index   <= inspect_index + 8'd1;
                current_index_q <= inspect_index;

                if (((prefetch_count + (inspect_valid ? 32'd1 : 32'd0)) >= warm_target) || (warm_ctr >= 32'd256)) begin
                    busy              <= 1'b0;
                    done              <= 1'b1;
                    cache_warm_cycles <= warm_ctr + 32'd1;
                end
            end else begin
                prefetch_valid <= 1'b0;
            end
        end
    end

endmodule
