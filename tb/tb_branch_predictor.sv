`timescale 1ns / 1ps

module tb_branch_predictor;
    logic         clk;
    logic         rst;
    logic [31:0]  pc_if;
    logic [31:0]  instr_if;
    logic         valid_if;
    logic        predict_taken;
    logic [31:0] predict_target;

    logic [31:0]  pc_ex;
    logic         is_branch_ex;
    logic         branch_taken_ex;
    logic [31:0]  branch_target_ex;
    logic         update_valid;

    Branch_Predictor #(
        .BHT_ENTRIES(64),
        .BTB_ENTRIES(64),
        .RAS_DEPTH(8),
        .GHR_WIDTH(2)
    ) dut (
        .clk(clk),
        .rst(rst),
        .pc_if(pc_if),
        .instr_if(instr_if),
        .valid_if(valid_if),
        .predict_taken(predict_taken),
        .predict_target(predict_target),
        .pc_ex(pc_ex),
        .is_branch_ex(is_branch_ex),
        .branch_taken_ex(branch_taken_ex),
        .branch_target_ex(branch_target_ex),
        .update_valid(update_valid)
    );

    always #5 clk = ~clk;

    task do_update;
        begin
            @(negedge clk);
            update_valid = 1'b1;
            @(posedge clk);
            @(negedge clk);
            update_valid = 1'b0;
        end
    endtask

    initial begin
        clk = 1'b0;
        rst = 1'b1;
        pc_if = 32'b0;
        instr_if = 32'b0;
        valid_if = 1'b0;
        pc_ex = 32'b0;
        is_branch_ex = 1'b0;
        branch_taken_ex = 1'b0;
        branch_target_ex = 32'b0;
        update_valid = 1'b0;

        repeat (2) @(posedge clk);
        rst <= 1'b0;

        // Baseline prediction should be not-taken.
        pc_if = 32'h0000_1000;
        instr_if = 32'h0000_0063;
        valid_if = 1'b1;
        #1;
        if (predict_taken) begin
            $display("FAIL: predictor should start as not-taken");
            $finish;
        end

        // Train taken three times so the active GHR-index entry becomes taken.
        pc_ex = 32'h0000_1000;
        is_branch_ex = 1'b1;
        branch_taken_ex = 1'b1;
        branch_target_ex = 32'h0000_2000;

        do_update();
        do_update();
        do_update();

        #1;
        if (!predict_taken) begin
            $display("FAIL: predictor did not learn taken branch");
            $finish;
        end
        if (predict_target != 32'h0000_2000) begin
            $display("FAIL: BTB target mismatch, got %h", predict_target);
            $finish;
        end

        // Push return address via CALL (JAL x1).
        @(negedge clk);
        pc_if = 32'h0000_3000;
        instr_if = 32'h0000_00EF;
        valid_if = 1'b1;
        @(posedge clk);

        // RET (JALR x0, x1, 0) should predict from RAS.
        @(negedge clk);
        pc_if = 32'h0000_4000;
        instr_if = 32'h0000_8067;
        valid_if = 1'b1;
        #1;

        if (!predict_taken) begin
            $display("FAIL: RET should force taken prediction");
            $finish;
        end
        if (predict_target != 32'h0000_3004) begin
            $display("FAIL: RAS target mismatch, got %h", predict_target);
            $finish;
        end

        $display("PASS: Branch predictor BHT/BTB/RAS behavior verified");
        $finish;
    end
endmodule
