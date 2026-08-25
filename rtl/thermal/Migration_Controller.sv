`timescale 1ns / 1ps
//============================================================================
// OrionRV - Task Migration Controller
//
// Transfers architectural state from source core to destination core:
//   1) Pause source + destination cores
//   2) Copy x1..x31 through migration register ports
//   3) Run cache warming window
//   4) Restore destination PC from source checkpoint
//   5) Resume destination and keep source halted
//============================================================================

module Migration_Controller #(
    parameter NUM_CORES  = 5,
    parameter CORE_IDX_W = 3
)(
    input  logic                      clk,
    input  logic                      rst_n,

    // Command interface
    input  logic                      start,
    input  logic                      clear_halt,
    input  logic [CORE_IDX_W-1:0]     src_core,
    input  logic [CORE_IDX_W-1:0]     dst_core,

    // Cache warming handshake
    input  logic                      cache_warm_done,
    output logic                       cache_warm_start,

    // Core state inputs
    input  logic [NUM_CORES*32-1:0]   pc_save_flat,
    input  logic [NUM_CORES*32-1:0]   mig_rdata_flat,
    input  logic [NUM_CORES*32-1:0]   csr_rdata_flat,

    // Per-core migration write ports
    output logic  [NUM_CORES-1:0]      mig_we,
    output logic  [NUM_CORES*5-1:0]    mig_addr_flat,
    output logic  [NUM_CORES*32-1:0]   mig_wdata_flat,

    // Per-core migration CSR ports
    output logic  [NUM_CORES-1:0]      csr_we,
    output logic  [NUM_CORES*4-1:0]    csr_addr_flat,
    output logic  [NUM_CORES*32-1:0]   csr_wdata_flat,

    // Per-core PC restore ports
    output logic  [NUM_CORES-1:0]      pc_load_en,
    output logic  [NUM_CORES*32-1:0]   pc_load_flat,

    // Core enable overrides
    output logic  [NUM_CORES-1:0]      pause_mask,
    output logic  [NUM_CORES-1:0]      halt_mask,

    // Status
    output logic                       busy,
    output logic                       done,
    output logic                       error,
    output logic  [31:0]               last_migration_cycles,
    output logic  [31:0]               migration_count,

    input  logic [NUM_CORES-1:0]  pg_save_req,
    input  logic [NUM_CORES-1:0]  pg_restore_req,
    output logic  [NUM_CORES-1:0]  pg_ack,

    // Active migration route (latched command)
    output logic [CORE_IDX_W-1:0]     active_src_core,
    output logic [CORE_IDX_W-1:0]     active_dst_core
);

    localparam [3:0] ST_IDLE      = 4'd0;
    localparam [3:0] ST_PAUSE     = 4'd1;
    localparam [3:0] ST_READ_GPR  = 4'd2;
    localparam [3:0] ST_WRITE_GPR = 4'd3;
    localparam [3:0] ST_READ_CSR  = 4'd4;
    localparam [3:0] ST_WRITE_CSR = 4'd5;
    localparam [3:0] ST_WARM      = 4'd6;
    localparam [3:0] ST_LOAD_PC   = 4'd7;
    localparam [3:0] ST_DONE      = 4'd8;

    localparam [4:0] CSR_COUNT = 5'd16;

    logic [3:0]                  state;
    logic [CORE_IDX_W-1:0]       src_latched;
    logic [CORE_IDX_W-1:0]       dst_latched;
    logic [5:0]                  reg_index;
    logic [4:0]                  csr_index;
    logic [31:0]                 read_data_q;
    logic [31:0]                 src_pc_q;
    logic [31:0]                 cycle_ctr;
    
    logic [1:0]                  mode; // 0: mig, 1: pg_save, 2: pg_restore
    logic [31:0]                 pwr_save_ram [0:NUM_CORES*64-1];

    logic [CORE_IDX_W-1:0] pg_save_core;
    logic pg_save_valid;
    always_comb begin
        pg_save_core = 0; pg_save_valid = 0;
        if (pg_save_req[0]) begin pg_save_core = 0; pg_save_valid = 1; end
        else if (pg_save_req[1]) begin pg_save_core = 1; pg_save_valid = 1; end
        else if (pg_save_req[2]) begin pg_save_core = 2; pg_save_valid = 1; end
        else if (pg_save_req[3]) begin pg_save_core = 3; pg_save_valid = 1; end
        else if (pg_save_req[4]) begin pg_save_core = 4; pg_save_valid = 1; end
    end
    
    logic [CORE_IDX_W-1:0] pg_restore_core;
    logic pg_restore_valid;
    always_comb begin
        pg_restore_core = 0; pg_restore_valid = 0;
        if (pg_restore_req[0]) begin pg_restore_core = 0; pg_restore_valid = 1; end
        else if (pg_restore_req[1]) begin pg_restore_core = 1; pg_restore_valid = 1; end
        else if (pg_restore_req[2]) begin pg_restore_core = 2; pg_restore_valid = 1; end
        else if (pg_restore_req[3]) begin pg_restore_core = 3; pg_restore_valid = 1; end
        else if (pg_restore_req[4]) begin pg_restore_core = 4; pg_restore_valid = 1; end
    end

    logic src_valid;
    assign src_valid = (src_core < NUM_CORES);
    logic dst_valid;
    assign dst_valid = (dst_core < NUM_CORES);

    logic [31:0] src_mig_rdata;
    assign src_mig_rdata = mig_rdata_flat[(src_latched*32) +: 32];
    logic [31:0] src_csr_rdata;
    assign src_csr_rdata = csr_rdata_flat[(src_latched*32) +: 32];
    assign active_src_core = src_latched;
    assign active_dst_core = dst_latched;

    always_ff @(posedge clk) begin
        if (state == ST_PAUSE && mode == 2'd1) begin
            pwr_save_ram[{src_latched, 6'd32}] <= src_pc_q;
        end else if (state == ST_WRITE_GPR && mode == 2'd1) begin
            pwr_save_ram[{src_latched, {1'b0, reg_index[4:0]}}] <= read_data_q;
        end else if (state == ST_WRITE_CSR && mode == 2'd1) begin
            pwr_save_ram[{src_latched, 6'd33 + {1'b0, csr_index[4:0]}}] <= read_data_q;
        end
    end

    always_ff @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            state                 <= ST_IDLE;
            src_latched           <= {CORE_IDX_W{1'b0}};
            dst_latched           <= {CORE_IDX_W{1'b0}};
            reg_index             <= 6'd1;
            csr_index             <= 5'd0;
            read_data_q           <= 32'b0;
            src_pc_q              <= 32'b0;
            cycle_ctr             <= 32'b0;
            halt_mask             <= {NUM_CORES{1'b0}};
            busy                  <= 1'b0;
            done                  <= 1'b0;
            error                 <= 1'b0;
            cache_warm_start      <= 1'b0;
            last_migration_cycles <= 32'b0;
            migration_count       <= 32'b0;
            mode                  <= 2'b0;
            pg_ack                <= {NUM_CORES{1'b0}};
        end else begin
            done <= 1'b0;
            cache_warm_start <= 1'b0;

            if (clear_halt)
                halt_mask <= {NUM_CORES{1'b0}};

            case (state)
                ST_IDLE: begin
                    busy      <= 1'b0;
                    cycle_ctr <= 32'b0;
                    pg_ack    <= {NUM_CORES{1'b0}};

                    if (start) begin
                        done  <= 1'b0;
                        error <= 1'b0;
                        mode  <= 2'd0;

                        if (!src_valid || !dst_valid || (src_core == dst_core)) begin
                            error <= 1'b1;
                        end else begin
                            src_latched <= src_core;
                            dst_latched <= dst_core;
                            reg_index   <= 6'd1;
                            csr_index   <= 5'd0;
                            src_pc_q    <= pc_save_flat[(src_core*32) +: 32];
                            busy        <= 1'b1;
                            state       <= ST_PAUSE;
                        end
                    end else if (pg_save_valid) begin
                        done  <= 1'b0;
                        mode <= 2'd1;
                        src_latched <= pg_save_core;
                        dst_latched <= pg_save_core;
                        reg_index   <= 6'd1;
                        csr_index   <= 5'd0;
                        src_pc_q    <= pc_save_flat[(pg_save_core*32) +: 32];
                        busy        <= 1'b1;
                        state       <= ST_PAUSE;
                    end else if (pg_restore_valid) begin
                        done  <= 1'b0;
                        mode <= 2'd2;
                        src_latched <= pg_restore_core;
                        dst_latched <= pg_restore_core;
                        reg_index   <= 6'd1;
                        csr_index   <= 5'd0;
                        src_pc_q    <= pwr_save_ram[{pg_restore_core, 6'd32}];
                        busy        <= 1'b1;
                        state       <= ST_PAUSE;
                    end
                end

                ST_PAUSE: begin
                    cycle_ctr <= cycle_ctr + 32'd1;
                    state     <= ST_READ_GPR;
                end

                ST_READ_GPR: begin
                    cycle_ctr   <= cycle_ctr + 32'd1;
                    if (mode == 2'd2) read_data_q <= pwr_save_ram[{src_latched, {1'b0, reg_index[4:0]}}];
                    else              read_data_q <= src_mig_rdata;
                    state       <= ST_WRITE_GPR;
                end

                ST_WRITE_GPR: begin
                    cycle_ctr <= cycle_ctr + 32'd1;

                    if (reg_index == 6'd31) begin
                        csr_index <= 5'd0;
                        state <= ST_READ_CSR;
                    end else begin
                        reg_index <= reg_index + 6'd1;
                        state     <= ST_READ_GPR;
                    end
                end

                ST_READ_CSR: begin
                    cycle_ctr   <= cycle_ctr + 32'd1;
                    if (mode == 2'd2) read_data_q <= pwr_save_ram[{src_latched, 6'd33 + {1'b0, csr_index[4:0]}}];
                    else              read_data_q <= src_csr_rdata;
                    state       <= ST_WRITE_CSR;
                end

                ST_WRITE_CSR: begin
                    cycle_ctr <= cycle_ctr + 32'd1;

                    if (csr_index == (CSR_COUNT - 5'd1)) begin
                        if (mode == 2'd0) begin
                            state     <= ST_WARM;
                            cache_warm_start <= 1'b1;
                        end else begin
                            state     <= ST_LOAD_PC;
                        end
                        csr_index <= 5'd0;
                    end else begin
                        csr_index <= csr_index + 5'd1;
                        state     <= ST_READ_CSR;
                    end
                end

                ST_WARM: begin
                    cycle_ctr <= cycle_ctr + 32'd1;
                    if (cache_warm_done)
                        state <= ST_LOAD_PC;
                end

                ST_LOAD_PC: begin
                    cycle_ctr <= cycle_ctr + 32'd1;
                    state     <= ST_DONE;
                end

                ST_DONE: begin
                    busy                  <= 1'b0;
                    done                  <= 1'b1;
                    error                 <= 1'b0;
                    
                    if (mode == 2'd0) begin
                        halt_mask[src_latched] <= 1'b1;
                        last_migration_cycles <= cycle_ctr;
                        migration_count       <= migration_count + 32'd1;
                    end else begin
                        pg_ack[src_latched] <= 1'b1;
                    end
                    state                 <= ST_IDLE;
                end

                default: begin
                    state <= ST_IDLE;
                end
            endcase
        end
    end

    always_comb begin
        mig_we        = {NUM_CORES{1'b0}};
        mig_addr_flat = {(NUM_CORES*5){1'b0}};
        mig_wdata_flat = {(NUM_CORES*32){1'b0}};
        csr_we        = {NUM_CORES{1'b0}};
        csr_addr_flat = {(NUM_CORES*4){1'b0}};
        csr_wdata_flat = {(NUM_CORES*32){1'b0}};
        pc_load_en    = {NUM_CORES{1'b0}};
        pc_load_flat  = {(NUM_CORES*32){1'b0}};
        pause_mask    = {NUM_CORES{1'b0}};

        if ((state == ST_PAUSE) ||
            (state == ST_READ_GPR) || (state == ST_WRITE_GPR) ||
            (state == ST_READ_CSR) || (state == ST_WRITE_CSR) ||
            (state == ST_WARM) || (state == ST_LOAD_PC)) begin
            pause_mask[src_latched] = 1'b1;
            pause_mask[dst_latched] = 1'b1;
        end

        if ((state == ST_READ_GPR) || (state == ST_WRITE_GPR)) begin
            mig_addr_flat[(src_latched*5) +: 5] = reg_index[4:0];
        end

        if (state == ST_WRITE_GPR) begin
            if (mode != 2'd1) mig_we[dst_latched] = 1'b1;
            mig_addr_flat[(dst_latched*5) +: 5] = reg_index[4:0];
            mig_wdata_flat[(dst_latched*32) +: 32] = read_data_q;
        end

        if ((state == ST_READ_CSR) || (state == ST_WRITE_CSR)) begin
            csr_addr_flat[(src_latched*4) +: 4] = csr_index[3:0];
        end

        if (state == ST_WRITE_CSR) begin
            if (mode != 2'd1) csr_we[dst_latched] = 1'b1;
            csr_addr_flat[(dst_latched*4) +: 4] = csr_index[3:0];
            csr_wdata_flat[(dst_latched*32) +: 32] = read_data_q;
        end

        if (state == ST_LOAD_PC) begin
            if (mode != 2'd1) pc_load_en[dst_latched] = 1'b1;
            pc_load_flat[(dst_latched*32) +: 32] = src_pc_q;
        end
    end

endmodule
