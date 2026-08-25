`timescale 1ns / 1ps
//============================================================================
// OrionRV - Direct-Mapped Instruction Cache
//
// Parameters:
//   CACHE_SIZE_BYTES = 4096 (4 KB default, configurable)
//   LINE_SIZE_WORDS  = 4    (16 bytes per cache line = 4 words)
//
// Structure:
//   4096 / 16 = 256 cache lines
//   Address breakdown (32-bit):
//     [31:12] = Tag          (20 bits)
//     [11:4]  = Index        (8 bits -> 256 lines)
//     [3:2]   = Word offset  (2 bits -> 4 words per line)
//     [1:0]   = Byte offset  (ignored, instructions are 4-byte aligned)
//
// Interface:
//   CPU side: single-cycle hit, stalls on miss
//   Memory side: 4-word burst fill on miss
//
// Designed for DOOM workload: tight inner loops benefit heavily from
// even a small I-cache due to BSP traversal and rendering hot paths.
//============================================================================

module icache #(
    parameter CACHE_LINES    = 256,    // Number of cache lines
    parameter LINE_WORDS     = 4,      // Words per cache line
    parameter ADDR_WIDTH     = 32,
    parameter TAG_WIDTH      = 20,
    parameter INDEX_WIDTH    = 8,
    parameter OFFSET_WIDTH   = 2       // Word offset within line
)(
    input  logic        clk,
    input  logic        rst,

    // -------------------------------------------------------------------
    // CPU-side interface (from pipeline IF stage)
    // -------------------------------------------------------------------
    input  logic [31:0] cpu_addr,
    input  logic        cpu_req,
    output logic [31:0] cpu_rdata,
    output logic        cpu_ready,      // 1 = cache hit (data valid)

    // -------------------------------------------------------------------
    // Memory-side interface (to main memory / AXI interconnect)
    // -------------------------------------------------------------------
    output logic  [31:0] mem_addr,
    input  logic [31:0] mem_rdata,
    output logic         mem_req,
    input  logic        mem_ready,      // 1 = memory data valid this cycle

    // -------------------------------------------------------------------
    // Cache control
    // -------------------------------------------------------------------
    input  logic        flush_all,      // Invalidate entire cache

    // -------------------------------------------------------------------
    // Cache Warming Inspection Interface (Phase 5.5)
    // -------------------------------------------------------------------
    input  logic [INDEX_WIDTH-1:0] inspect_index,
    output logic                   inspect_valid,
    output logic [TAG_WIDTH-1:0]   inspect_tag
);

    // ===================================================================
    // Address field extraction
    // ===================================================================
    logic [TAG_WIDTH-1:0]    addr_tag;
    assign addr_tag = cpu_addr[31:12];
    logic [INDEX_WIDTH-1:0]  addr_index;
    assign addr_index = cpu_addr[11:4];
    logic [OFFSET_WIDTH-1:0] addr_offset;
    assign addr_offset = cpu_addr[3:2];

    // ===================================================================
    // Cache storage
    // ===================================================================
    // Tag array
    logic [TAG_WIDTH-1:0] tag_array   [0:CACHE_LINES-1];
    // Valid bit array
    logic                 valid_array [0:CACHE_LINES-1];
    // Data array: each line has LINE_WORDS words
    logic [31:0]          data_array  [0:CACHE_LINES*LINE_WORDS-1];

    // ===================================================================
    // Cache Inspection (Phase 5.5)
    // ===================================================================
    assign inspect_valid = valid_array[inspect_index];
    assign inspect_tag   = tag_array[inspect_index];

    // ===================================================================
    // Cache lookup (combinational)
    // ===================================================================
    logic tag_match;
    assign tag_match = (tag_array[addr_index] == addr_tag);
    logic addr_word_aligned;
    assign addr_word_aligned = (cpu_addr[1:0] == 2'b00);
    logic cache_hit;
    assign cache_hit = valid_array[addr_index] & tag_match & cpu_req & addr_word_aligned;

    // Data output on hit: select word from cache line
    logic [$clog2(CACHE_LINES*LINE_WORDS)-1:0] data_idx;
    assign data_idx = {addr_index, addr_offset};
    assign cpu_rdata = data_array[data_idx];
    assign cpu_ready = cache_hit;

    // ===================================================================
    // Miss handling state machine (with next-line prefetch)
    // ===================================================================
    localparam IDLE      = 3'b000;
    localparam FILL      = 3'b001;
    localparam FILL_WAIT = 3'b011;  // Deassert mem_req so addr settles
    localparam COMPLETE  = 3'b010;
    localparam PF_FILL      = 3'b100;  // Prefetch fill
    localparam PF_FILL_WAIT = 3'b101;  // Prefetch fill wait
    localparam PF_COMPLETE  = 3'b110;  // Prefetch complete

    logic [2:0]  state;
    logic [OFFSET_WIDTH-1:0] fill_word;   // Which word of the line we're filling
    logic [INDEX_WIDTH-1:0]  fill_index;
    logic [TAG_WIDTH-1:0]    fill_tag;
    logic [31:0]             fill_base_addr;

    // Next-line prefetch detection: accessing last word of a cache line
    logic at_line_end;
    assign at_line_end = (addr_offset == {OFFSET_WIDTH{1'b1}});
    logic [INDEX_WIDTH-1:0] next_index;
    assign next_index = addr_index + 1'b1;
    logic [TAG_WIDTH-1:0]   next_tag;
    assign next_tag = (addr_index == {INDEX_WIDTH{1'b1}}) ?
                                         addr_tag + 1'b1 : addr_tag;
    logic next_line_valid;
    assign next_line_valid = valid_array[next_index] &&
                           (tag_array[next_index] == next_tag);
    logic should_prefetch;
    assign should_prefetch = cpu_req && cache_hit && at_line_end && !next_line_valid;

    integer k;

    always_ff @(posedge clk) begin
        if (rst || flush_all) begin
            state     <= IDLE;
            fill_word <= 0;
            mem_req   <= 1'b0;
            mem_addr  <= 32'b0;
            // Invalidate all cache lines
            for (k = 0; k < CACHE_LINES; k = k + 1) begin
                valid_array[k] <= 1'b0;
            end
        end
        else begin
            case (state)

                // ---------------------------------------------------------
                // IDLE: Wait for a cache miss or prefetch trigger
                // ---------------------------------------------------------
                IDLE: begin
                    if (cpu_req && !cache_hit) begin
                        // Cache miss -- start demand line fill
                        state          <= FILL;
                        fill_index     <= addr_index;
                        fill_tag       <= addr_tag;
                        fill_base_addr <= {cpu_addr[31:4], 4'b0000};
                        fill_word      <= 0;

                        mem_addr <= {cpu_addr[31:4], 4'b0000};
                        mem_req  <= 1'b1;
                    end
                    else if (should_prefetch) begin
                        // Next-line prefetch — only when idle
                        state          <= PF_FILL;
                        fill_index     <= next_index;
                        fill_tag       <= next_tag;
                        fill_base_addr <= {next_tag, next_index, 4'b0000};
                        fill_word      <= 0;

                        mem_addr <= {next_tag, next_index, 4'b0000};
                        mem_req  <= 1'b1;
                    end
                end

                // ---------------------------------------------------------
                // FILL: Wait for current word from memory (demand)
                // ---------------------------------------------------------
                FILL: begin
                    if (mem_ready) begin
                        data_array[{fill_index, fill_word}] <= mem_rdata;
                        mem_req <= 1'b0;

                        if (fill_word == LINE_WORDS - 1) begin
                            tag_array[fill_index]   <= fill_tag;
                            valid_array[fill_index] <= 1'b1;
                            state   <= COMPLETE;
                        end
                        else begin
                            fill_word <= fill_word + 1;
                            mem_addr  <= fill_base_addr + ({fill_word, 2'b00} + 4);
                            state <= FILL_WAIT;
                        end
                    end
                end

                FILL_WAIT: begin
                    mem_req <= 1'b1;
                    state   <= FILL;
                end

                COMPLETE: begin
                    state <= IDLE;
                end

                // ---------------------------------------------------------
                // Prefetch fill states (abortable on demand miss)
                // ---------------------------------------------------------
                PF_FILL: begin
                    // Abort prefetch if a demand miss arrives
                    if (cpu_req && !cache_hit) begin
                        mem_req <= 1'b0;
                        state   <= IDLE;  // Will re-enter IDLE and start demand fill
                    end
                    else if (mem_ready) begin
                        data_array[{fill_index, fill_word}] <= mem_rdata;
                        mem_req <= 1'b0;

                        if (fill_word == LINE_WORDS - 1) begin
                            tag_array[fill_index]   <= fill_tag;
                            valid_array[fill_index] <= 1'b1;
                            state <= PF_COMPLETE;
                        end
                        else begin
                            fill_word <= fill_word + 1;
                            mem_addr  <= fill_base_addr + ({fill_word, 2'b00} + 4);
                            state <= PF_FILL_WAIT;
                        end
                    end
                end

                PF_FILL_WAIT: begin
                    // Abort on demand miss
                    if (cpu_req && !cache_hit) begin
                        mem_req <= 1'b0;
                        state   <= IDLE;
                    end
                    else begin
                        mem_req <= 1'b1;
                        state   <= PF_FILL;
                    end
                end

                PF_COMPLETE: begin
                    state <= IDLE;
                end

                default: begin
                    state <= IDLE;
                    mem_req <= 1'b0;
                end

            endcase
        end
    end

    // ===================================================================
    // Initialization (simulation only)
    // ===================================================================
    initial begin
        for (k = 0; k < CACHE_LINES; k = k + 1) begin
            tag_array[k]   = {TAG_WIDTH{1'b0}};
            valid_array[k] = 1'b0;
        end
    end

endmodule
