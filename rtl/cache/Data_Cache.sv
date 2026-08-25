`timescale 1ns / 1ps
//============================================================================
// OrionRV - Data Cache (Phase 7 + Phase 7.5 upgrades)
//
// Direct-mapped write-through, write-allocate data cache with:
//   - 1-entry store buffer for non-blocking stores
//   - MSHR for hit-under-miss (non-blocking load miss)
//   - Victim buffer interface (eviction push / miss lookup)
//   - Stride prefetcher interface (hint-driven prefetch fills)
//   - Write-invalidate coherence via external snoop bus
//============================================================================

module Data_Cache #(
    parameter CACHE_LINES = 256,
    parameter LINE_WORDS  = 4,
    parameter ADDR_WIDTH  = 32,
    parameter TAG_WIDTH   = 20,
    parameter INDEX_WIDTH = 8,
    parameter OFFSET_WIDTH = 2,
    parameter CORE_ID = 0
)(
    input  logic        clk,
    input  logic        rst,

    // CPU-side interface
    input  logic [31:0] cpu_addr,
    input  logic [31:0] cpu_wdata,
    input  logic [3:0]  cpu_wstrb,
    input  logic        cpu_we,
    input  logic        cpu_req,
    output logic  [31:0] cpu_rdata,
    output logic         cpu_ready,

    // Memory-side interface
    output logic  [31:0] mem_addr,
    output logic  [31:0] mem_wdata,
    output logic  [3:0]  mem_wstrb,
    output logic         mem_we,
    output logic         mem_req,
    input  logic [31:0] mem_rdata,
    input  logic        mem_ready,

    // Coherence snoop
    input  logic [31:0] snoop_addr,
    input  logic        snoop_we,
    input  logic [2:0]  snoop_src,

    // Cache control
    input  logic        flush_all,

    // Victim buffer interface
    output logic                       victim_evict_valid,
    output logic  [TAG_WIDTH-1:0]      victim_evict_tag,
    output logic  [INDEX_WIDTH-1:0]    victim_evict_index,
    output logic  [LINE_WORDS*32-1:0]  victim_evict_data,
    output logic                      victim_lookup_valid,
    output logic [TAG_WIDTH-1:0]      victim_lookup_tag,
    output logic [INDEX_WIDTH-1:0]    victim_lookup_index,
    input  logic                      victim_hit,
    input  logic [LINE_WORDS*32-1:0]  victim_hit_data,
    output logic                       victim_swap_valid,
    output logic  [TAG_WIDTH-1:0]      victim_swap_tag,
    output logic  [INDEX_WIDTH-1:0]    victim_swap_index,
    output logic  [LINE_WORDS*32-1:0]  victim_swap_data,

    // Stride prefetcher interface
    input  logic        pf_req,
    input  logic [31:0] pf_addr,
    output logic         pf_ack,

    // Miss notification (for stride prefetcher)
    output logic        miss_notify,
    output logic [31:0] miss_notify_addr
);

    localparam ST_IDLE          = 4'd0;
    localparam ST_FILL_REQ      = 4'd1;
    localparam ST_FILL_WAIT     = 4'd2;
    localparam ST_FILL_COMPLETE = 4'd3;
    localparam ST_STORE_REQ     = 4'd4;
    localparam ST_STORE_WAIT    = 4'd5;
    localparam ST_VICTIM_SWAP   = 4'd6;
    localparam ST_PF_FILL_REQ   = 4'd7;
    localparam ST_PF_FILL_WAIT  = 4'd8;
    localparam ST_PF_COMPLETE   = 4'd9;
    localparam ST_MSHR_WAIT     = 4'd10;

    logic [3:0] state;

    // Cache arrays
    logic [TAG_WIDTH-1:0]  tag_array   [0:CACHE_LINES-1];
    logic                  valid_array [0:CACHE_LINES-1];
    logic [31:0]           data_array_0 [0:CACHE_LINES-1];
    logic [31:0]           data_array_1 [0:CACHE_LINES-1];
    logic [31:0]           data_array_2 [0:CACHE_LINES-1];
    logic [31:0]           data_array_3 [0:CACHE_LINES-1];

    // Latched request fields
    logic [31:0] req_addr;
    logic [31:0] req_wdata;
    logic [3:0]  req_wstrb;
    logic        req_we;

    logic [INDEX_WIDTH-1:0]  req_index;
    logic [TAG_WIDTH-1:0]    req_tag;
    logic [OFFSET_WIDTH-1:0] req_word;

    // Fill tracking
    logic [31:0]             fill_base_addr;
    logic [OFFSET_WIDTH-1:0] fill_word;

    // 1-entry store buffer
    logic        sb_valid;
    logic [31:0] sb_addr;
    logic [31:0] sb_wdata;
    logic [3:0]  sb_wstrb;

    // MSHR — tracks one outstanding miss
    logic        mshr_valid;
    logic [31:0] mshr_addr;
    logic [4:0]  mshr_rd;  // Not used in this simple version (in-order pipeline)

    // Indicates ST_STORE_WAIT should enqueue current req_* into the buffer
    // after draining an older buffered store.
    logic        store_wait_rebuffer_new;

    // Registered CPU completion outputs
    logic [31:0] cpu_rdata_reg;
    logic        cpu_ready_reg;

    // Miss notification for stride prefetcher
    logic        miss_notify_r;
    logic [31:0] miss_notify_addr_r;
    assign miss_notify = miss_notify_r;
    assign miss_notify_addr = miss_notify_addr_r;

    integer i;

    function [31:0] apply_wstrb32;
        input [31:0] cur;
        input [31:0] wdata;
        input [3:0]  wstrb;
        begin
            apply_wstrb32 = cur;
            if (wstrb[0]) apply_wstrb32[7:0]   = wdata[7:0];
            if (wstrb[1]) apply_wstrb32[15:8]  = wdata[15:8];
            if (wstrb[2]) apply_wstrb32[23:16] = wdata[23:16];
            if (wstrb[3]) apply_wstrb32[31:24] = wdata[31:24];
        end
    endfunction

    logic busy;
    assign busy = (state != ST_IDLE);

    logic [TAG_WIDTH-1:0]    addr_tag;
    assign addr_tag = cpu_addr[31:12];
    logic [INDEX_WIDTH-1:0]  addr_index;
    assign addr_index = cpu_addr[11:4];
    logic [OFFSET_WIDTH-1:0] addr_word;
    assign addr_word = cpu_addr[3:2];

    logic [TAG_WIDTH-1:0]    snoop_tag;
    assign snoop_tag = snoop_addr[31:12];
    logic [INDEX_WIDTH-1:0]  snoop_index;
    assign snoop_index = snoop_addr[11:4];
    // Note: snoop_addr[3:0] are intentionally not used (word offset bits)
    logic [3:0] snoop_unused;
    assign snoop_unused = snoop_addr[3:0]; // Tie-off to prevent unconnected port warnings
    logic                    snoop_other_core;
    assign snoop_other_core = snoop_we && (snoop_src != CORE_ID[2:0]);

    logic cache_hit;
    assign cache_hit = (!busy) && cpu_req && valid_array[addr_index] && (tag_array[addr_index] == addr_tag);
    logic load_hit;
    assign load_hit = cache_hit && !cpu_we;
    logic [31:0] read_word_hit;
    assign read_word_hit = (addr_word == 2'd0) ? data_array_0[addr_index] :
                                (addr_word == 2'd1) ? data_array_1[addr_index] :
                                (addr_word == 2'd2) ? data_array_2[addr_index] :
                                                      data_array_3[addr_index];

    // Store buffer forwarding: if load addr matches buffered store, forward
    logic sb_fwd_match;
    assign sb_fwd_match = sb_valid && (sb_addr[31:2] == cpu_addr[31:2]);

    // Victim buffer lookup — active during miss handling
    assign victim_lookup_valid = (state == ST_IDLE) && cpu_req && !cache_hit && !cpu_we;
    assign victim_lookup_tag   = addr_tag;
    assign victim_lookup_index = addr_index;

    always_comb begin
        if (load_hit) begin
            if (sb_fwd_match)
                cpu_rdata = apply_wstrb32(read_word_hit, sb_wdata, sb_wstrb);
            else
                cpu_rdata = read_word_hit;
            cpu_ready = 1'b1;
        end else begin
            cpu_rdata = cpu_rdata_reg;
            cpu_ready = cpu_ready_reg;
        end
    end

    always_ff @(posedge clk) begin
        if (rst) begin
            state         <= ST_IDLE;
            req_addr      <= 32'b0;
            req_wdata     <= 32'b0;
            req_wstrb     <= 4'b0;
            req_we        <= 1'b0;
            req_index     <= {INDEX_WIDTH{1'b0}};
            req_tag       <= {TAG_WIDTH{1'b0}};
            req_word      <= {OFFSET_WIDTH{1'b0}};
            fill_base_addr<= 32'b0;
            fill_word     <= {OFFSET_WIDTH{1'b0}};
            cpu_rdata_reg <= 32'b0;
            cpu_ready_reg <= 1'b0;
            sb_valid      <= 1'b0;
            sb_addr       <= 32'b0;
            sb_wdata      <= 32'b0;
            sb_wstrb      <= 4'b0;
            mshr_valid    <= 1'b0;
            mshr_addr     <= 32'b0;
            store_wait_rebuffer_new <= 1'b0;
            miss_notify_r <= 1'b0;
            miss_notify_addr_r <= 32'b0;
            pf_ack        <= 1'b0;

            victim_evict_valid <= 1'b0;
            victim_evict_tag   <= {TAG_WIDTH{1'b0}};
            victim_evict_index <= {INDEX_WIDTH{1'b0}};
            victim_evict_data  <= {(LINE_WORDS*32){1'b0}};
            victim_swap_valid  <= 1'b0;
            victim_swap_tag    <= {TAG_WIDTH{1'b0}};
            victim_swap_index  <= {INDEX_WIDTH{1'b0}};
            victim_swap_data   <= {(LINE_WORDS*32){1'b0}};

            mem_addr      <= 32'b0;
            mem_wdata     <= 32'b0;
            mem_wstrb     <= 4'b0;
            mem_we        <= 1'b0;
            mem_req       <= 1'b0;

            for (i = 0; i < CACHE_LINES; i = i + 1) begin
                valid_array[i] <= 1'b0;
                tag_array[i] <= {TAG_WIDTH{1'b0}};
            end
        end else begin
            cpu_ready_reg <= 1'b0;
            miss_notify_r <= 1'b0;
            victim_evict_valid <= 1'b0;
            victim_swap_valid  <= 1'b0;
            pf_ack <= 1'b0;

            if (flush_all) begin
                state    <= ST_IDLE;
                mem_req  <= 1'b0;
                mem_we   <= 1'b0;
                mem_wstrb<= 4'b0;
                sb_valid <= 1'b0;
                mshr_valid <= 1'b0;
                for (i = 0; i < CACHE_LINES; i = i + 1)
                    valid_array[i] <= 1'b0;
            end else begin
                // Coherence snoop invalidation
                if (snoop_other_core && valid_array[snoop_index] && (tag_array[snoop_index] == snoop_tag))
                    valid_array[snoop_index] <= 1'b0;
                // Also invalidate store buffer if snooped
                if (snoop_other_core && sb_valid && (sb_addr[31:12] == snoop_tag) && (sb_addr[11:4] == snoop_index))
                    sb_valid <= 1'b0;

                case (state)
                    ST_IDLE: begin
                        mem_req <= 1'b0;
                        mem_we  <= 1'b0;

                        // Drain store buffer in background
                        if (sb_valid) begin
                            store_wait_rebuffer_new <= 1'b0;
                            state <= ST_STORE_REQ;
                        end
                        else if (cpu_req) begin
                            req_addr       <= cpu_addr;
                            req_wdata      <= cpu_wdata;
                            req_wstrb      <= cpu_wstrb;
                            req_we         <= cpu_we;
                            req_index      <= addr_index;
                            req_tag        <= addr_tag;
                            req_word       <= addr_word;
                            fill_base_addr <= {cpu_addr[31:4], 4'b0000};
                            fill_word      <= {OFFSET_WIDTH{1'b0}};

                            if (cpu_we) begin
                                if (valid_array[addr_index] && (tag_array[addr_index] == addr_tag)) begin
                                    // Store hit: update cache, buffer write-through
                                    if (addr_word == 2'd0) data_array_0[addr_index] <= apply_wstrb32(data_array_0[addr_index], cpu_wdata, cpu_wstrb);
                                    if (addr_word == 2'd1) data_array_1[addr_index] <= apply_wstrb32(data_array_1[addr_index], cpu_wdata, cpu_wstrb);
                                    if (addr_word == 2'd2) data_array_2[addr_index] <= apply_wstrb32(data_array_2[addr_index], cpu_wdata, cpu_wstrb);
                                    if (addr_word == 2'd3) data_array_3[addr_index] <= apply_wstrb32(data_array_3[addr_index], cpu_wdata, cpu_wstrb);
                                    store_wait_rebuffer_new <= sb_valid;
                                    state <= ST_STORE_REQ;
                                end else begin
                                    // Store miss: fill line first
                                    miss_notify_r <= 1'b1;
                                    miss_notify_addr_r <= cpu_addr;
                                    // Check victim buffer first
                                    if (victim_hit) begin
                                        state <= ST_VICTIM_SWAP;
                                    end else begin
                                        state <= ST_FILL_REQ;
                                    end
                                end
                            end else if (!cache_hit) begin
                                // Load miss
                                miss_notify_r <= 1'b1;
                                miss_notify_addr_r <= cpu_addr;
                                mshr_valid <= 1'b1;
                                mshr_addr  <= cpu_addr;

                                if (victim_hit) begin
                                    state <= ST_VICTIM_SWAP;
                                end else begin
                                    state <= ST_FILL_REQ;
                                end
                            end
                        end
                        // Prefetch hint — only when truly idle
                        else if (pf_req && !sb_valid) begin
                            pf_ack <= 1'b1;
                            req_addr       <= pf_addr;
                            req_we         <= 1'b0;
                            req_index      <= pf_addr[11:4];
                            req_tag        <= pf_addr[31:12];
                            req_word       <= pf_addr[3:2];
                            fill_base_addr <= {pf_addr[31:4], 4'b0000};
                            fill_word      <= {OFFSET_WIDTH{1'b0}};

                            // Only prefetch if not already cached
                            if (!(valid_array[pf_addr[11:4]] && (tag_array[pf_addr[11:4]] == pf_addr[31:12]))) begin
                                state <= ST_PF_FILL_REQ;
                            end
                        end
                    end

                    // Victim buffer swap — install victim line into L1D
                    ST_VICTIM_SWAP: begin
                        // Push current L1D occupant to victim buffer
                        if (valid_array[req_index]) begin
                            victim_evict_valid <= 1'b1;
                            victim_evict_tag   <= tag_array[req_index];
                            victim_evict_index <= req_index;
                            victim_evict_data  <= {data_array_3[req_index],
                                                   data_array_2[req_index],
                                                   data_array_1[req_index],
                                                   data_array_0[req_index]};
                        end

                        // Install victim data into L1D
                        tag_array[req_index]   <= req_tag;
                        valid_array[req_index] <= 1'b1;
                        data_array_0[req_index] <= victim_hit_data[31:0];
                        data_array_1[req_index] <= victim_hit_data[63:32];
                        data_array_2[req_index] <= victim_hit_data[95:64];
                        data_array_3[req_index] <= victim_hit_data[127:96];

                        // Swap: send old line to victim
                        victim_swap_valid <= 1'b1;
                        victim_swap_tag   <= tag_array[req_index];
                        victim_swap_index <= req_index;
                        victim_swap_data  <= {data_array_3[req_index],
                                              data_array_2[req_index],
                                              data_array_1[req_index],
                                              data_array_0[req_index]};

                        if (req_we) begin
                            if (req_word == 2'd0) data_array_0[req_index] <= apply_wstrb32(victim_hit_data[31:0], req_wdata, req_wstrb);
                            if (req_word == 2'd1) data_array_1[req_index] <= apply_wstrb32(victim_hit_data[63:32], req_wdata, req_wstrb);
                            if (req_word == 2'd2) data_array_2[req_index] <= apply_wstrb32(victim_hit_data[95:64], req_wdata, req_wstrb);
                            if (req_word == 2'd3) data_array_3[req_index] <= apply_wstrb32(victim_hit_data[127:96], req_wdata, req_wstrb);
                            state <= ST_STORE_REQ;
                        end else begin
                            cpu_rdata_reg <= victim_hit_data[req_word*32 +: 32];
                            cpu_ready_reg <= 1'b1;
                            mshr_valid    <= 1'b0;
                            state <= ST_IDLE;
                        end
                    end

                    ST_FILL_REQ: begin
                        // Push evicted line to victim buffer before overwriting
                        if (fill_word == {OFFSET_WIDTH{1'b0}} && valid_array[req_index]) begin
                            victim_evict_valid <= 1'b1;
                            victim_evict_tag   <= tag_array[req_index];
                            victim_evict_index <= req_index;
                            victim_evict_data  <= {data_array_3[req_index],
                                                   data_array_2[req_index],
                                                   data_array_1[req_index],
                                                   data_array_0[req_index]};
                        end

                        mem_addr  <= fill_base_addr + {28'b0, fill_word, 2'b00};
                        mem_wdata <= 32'b0;
                        mem_wstrb <= 4'b0000;
                        mem_we    <= 1'b0;
                        mem_req   <= 1'b1;
                        state     <= ST_FILL_WAIT;
                    end

                    ST_FILL_WAIT: begin
                        if (mem_ready) begin
                            mem_req <= 1'b0;
                            if (fill_word == 2'd0) data_array_0[req_index] <= mem_rdata;
                            if (fill_word == 2'd1) data_array_1[req_index] <= mem_rdata;
                            if (fill_word == 2'd2) data_array_2[req_index] <= mem_rdata;
                            if (fill_word == 2'd3) data_array_3[req_index] <= mem_rdata;

                            if (fill_word == LINE_WORDS - 1)
                                state <= ST_FILL_COMPLETE;
                            else begin
                                fill_word <= fill_word + 1'b1;
                                state <= ST_FILL_REQ;
                            end
                        end
                    end

                    ST_FILL_COMPLETE: begin
                        tag_array[req_index]   <= req_tag;
                        valid_array[req_index] <= 1'b1;
                        mshr_valid <= 1'b0;

                        if (req_we) begin
                            if (req_word == 2'd0) data_array_0[req_index] <= apply_wstrb32(data_array_0[req_index], req_wdata, req_wstrb);
                            if (req_word == 2'd1) data_array_1[req_index] <= apply_wstrb32(data_array_1[req_index], req_wdata, req_wstrb);
                            if (req_word == 2'd2) data_array_2[req_index] <= apply_wstrb32(data_array_2[req_index], req_wdata, req_wstrb);
                            if (req_word == 2'd3) data_array_3[req_index] <= apply_wstrb32(data_array_3[req_index], req_wdata, req_wstrb);
                            state <= ST_STORE_REQ;
                        end else begin
                            cpu_rdata_reg <= (req_word == 2'd0) ? data_array_0[req_index] :
                                             (req_word == 2'd1) ? data_array_1[req_index] :
                                             (req_word == 2'd2) ? data_array_2[req_index] :
                                                                  data_array_3[req_index];
                            cpu_ready_reg <= 1'b1;
                            state <= ST_IDLE;
                        end
                    end

                    ST_STORE_REQ: begin
                        // Drain the pending store-buffer or direct store
                        if (sb_valid) begin
                            // Drain old store buffer first
                            mem_addr  <= sb_addr;
                            mem_wdata <= sb_wdata;
                            mem_wstrb <= sb_wstrb;
                        end else begin
                            mem_addr  <= req_addr;
                            mem_wdata <= req_wdata;
                            mem_wstrb <= req_wstrb;
                        end
                        mem_we    <= 1'b1;
                        mem_req   <= 1'b1;
                        state     <= ST_STORE_WAIT;
                    end

                    ST_STORE_WAIT: begin
                        if (mem_ready) begin
                            mem_req <= 1'b0;
                            mem_we  <= 1'b0;

                            if (store_wait_rebuffer_new) begin
                                // Old store buffer drained; now buffer the new one
                                sb_valid <= 1'b1;
                                sb_addr  <= req_addr;
                                sb_wdata <= req_wdata;
                                sb_wstrb <= req_wstrb;
                            end else begin
                                sb_valid <= 1'b0;
                            end

                            cpu_ready_reg <= 1'b1;
                            store_wait_rebuffer_new <= 1'b0;
                            state <= ST_IDLE;
                        end
                    end

                    // Prefetch fill states (abortable)
                    ST_PF_FILL_REQ: begin
                        if (cpu_req) begin
                            state <= ST_IDLE; mem_req <= 1'b0;
                        end else begin
                            mem_addr  <= fill_base_addr + {28'b0, fill_word, 2'b00};
                            mem_wdata <= 32'b0;
                            mem_wstrb <= 4'b0000;
                            mem_we    <= 1'b0;
                            mem_req   <= 1'b1;
                            state     <= ST_PF_FILL_WAIT;
                        end
                    end

                    ST_PF_FILL_WAIT: begin
                        if (cpu_req) begin
                            state <= ST_IDLE; mem_req <= 1'b0;
                        end else if (mem_ready) begin
                            mem_req <= 1'b0;
                            if (fill_word == 2'd0) data_array_0[req_index] <= mem_rdata;
                            if (fill_word == 2'd1) data_array_1[req_index] <= mem_rdata;
                            if (fill_word == 2'd2) data_array_2[req_index] <= mem_rdata;
                            if (fill_word == 2'd3) data_array_3[req_index] <= mem_rdata;
                            if (fill_word == LINE_WORDS - 1)
                                state <= ST_PF_COMPLETE;
                            else begin
                                fill_word <= fill_word + 1'b1;
                                state <= ST_PF_FILL_REQ;
                            end
                        end
                    end

                    ST_PF_COMPLETE: begin
                        tag_array[req_index]   <= req_tag;
                        valid_array[req_index] <= 1'b1;
                        state <= ST_IDLE;
                    end

                    default: begin
                        state   <= ST_IDLE;
                        mem_req <= 1'b0;
                        mem_we  <= 1'b0;
                    end
                endcase
            end
        end
    end

endmodule
