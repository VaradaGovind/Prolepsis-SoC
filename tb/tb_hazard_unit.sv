`timescale 1ns / 1ps

module tb_hazard_unit;
    logic [4:0] id_raw_rs1;
    logic [4:0] id_raw_rs2;
    logic [4:0] ex_rd;
    logic       ex_reg_write;
    logic       ex_mem_read;
    logic [4:0] mem_rd;
    logic       mem_reg_write;
    logic [4:0] wb_rd;
    logic       wb_reg_write;
    logic       is_branch_id;
    logic       branch_taken;
    logic       is_jal;
    logic       is_jalr;
    logic       bp_mispredict;
    logic       atomic_busy;
    logic       alu_busy;
    logic       icache_miss;
    logic       dmem_stall;

    logic stall_if;
    logic stall_id;
    logic stall_ex;
    logic stall_mem;
    logic flush_if;
    logic flush_id;
    logic flush_ex;

    hazard_unit dut (
        .id_raw_rs1(id_raw_rs1),
        .id_raw_rs2(id_raw_rs2),
        .ex_rd(ex_rd),
        .ex_reg_write(ex_reg_write),
        .ex_mem_read(ex_mem_read),
        .mem_rd(mem_rd),
        .mem_reg_write(mem_reg_write),
        .wb_rd(wb_rd),
        .wb_reg_write(wb_reg_write),
        .is_branch_id(is_branch_id),
        .branch_taken(branch_taken),
        .is_jal(is_jal),
        .is_jalr(is_jalr),
        .bp_mispredict(bp_mispredict),
        .atomic_busy(atomic_busy),
        .alu_busy(alu_busy),
        .icache_miss(icache_miss),
        .dmem_stall(dmem_stall),
        .stall_if(stall_if),
        .stall_id(stall_id),
        .stall_ex(stall_ex),
        .stall_mem(stall_mem),
        .flush_if(flush_if),
        .flush_id(flush_id),
        .flush_ex(flush_ex)
    );

    task clear_inputs;
        begin
            id_raw_rs1 = 5'd0;
            id_raw_rs2 = 5'd0;
            ex_rd = 5'd0;
            ex_reg_write = 1'b0;
            ex_mem_read = 1'b0;
            mem_rd = 5'd0;
            mem_reg_write = 1'b0;
            wb_rd = 5'd0;
            wb_reg_write = 1'b0;
            is_branch_id = 1'b0;
            branch_taken = 1'b0;
            is_jal = 1'b0;
            is_jalr = 1'b0;
            bp_mispredict = 1'b0;
            atomic_busy = 1'b0;
            alu_busy = 1'b0;
            icache_miss = 1'b0;
            dmem_stall = 1'b0;
        end
    endtask

    initial begin
        clear_inputs();
        #1;

        // 1) Load-use hazard: EX load destination consumed in ID.
        id_raw_rs1 = 5'd5;
        ex_rd = 5'd5;
        ex_mem_read = 1'b1;
        #1;
        if (!(stall_if && stall_id && flush_ex)) begin
            $display("FAIL: load-use hazard was not detected correctly");
            $finish;
        end

        // 2) Branch dependency hazard: branch in ID depends on EX result.
        clear_inputs();
        is_branch_id = 1'b1;
        id_raw_rs1 = 5'd9;
        ex_reg_write = 1'b1;
        ex_rd = 5'd9;
        #1;
        if (!(stall_if && stall_id && flush_ex)) begin
            $display("FAIL: branch dependency hazard was not detected");
            $finish;
        end

        // 3) Misprediction should flush ID/EX without forcing generic stalls.
        clear_inputs();
        bp_mispredict = 1'b1;
        #1;
        if (!(flush_id && flush_ex && !stall_if && !stall_id)) begin
            $display("FAIL: misprediction flush behavior incorrect");
            $finish;
        end

        // 4) JALR redirect path should flush when not predicted.
        clear_inputs();
        is_jalr = 1'b1;
        #1;
        if (!(flush_id && flush_ex)) begin
            $display("FAIL: JALR control hazard flush missing");
            $finish;
        end

        // 5) I-cache miss is masked for JAL fetch redirection.
        clear_inputs();
        icache_miss = 1'b1;
        is_jal = 1'b1;
        #1;
        if (stall_if || stall_id) begin
            $display("FAIL: JAL should mask icache_miss stall");
            $finish;
        end

        // 6) Data memory stall must hold EX/MEM pipeline.
        clear_inputs();
        dmem_stall = 1'b1;
        #1;
        if (!(stall_if && stall_id && stall_ex && stall_mem)) begin
            $display("FAIL: dmem_stall propagation incorrect");
            $finish;
        end

        $display("PASS: Hazard unit load-use/branch/control stall and flush behavior verified");
        $finish;
    end
endmodule
