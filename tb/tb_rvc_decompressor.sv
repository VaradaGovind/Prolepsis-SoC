`timescale 1ns / 1ps

module tb_rvc_decompressor;
    logic  [15:0] instr_c;
    logic [31:0] instr_out;
    logic        is_compressed;
    logic        illegal_c;

    RVC_Decompressor dut (
        .instr_c(instr_c),
        .instr_out(instr_out),
        .is_compressed(is_compressed),
        .illegal_c(illegal_c)
    );

    initial begin
        // C.NOP (encoded as C.ADDI x0, 0)
        instr_c = 16'h0001;
        #1;
        if (!is_compressed || illegal_c) begin
            $display("FAIL: C.NOP should be valid compressed instruction");
            $finish;
        end
        if (instr_out != 32'h0000_0013) begin
            $display("FAIL: C.NOP expansion mismatch, got %h", instr_out);
            $finish;
        end

        // C.EBREAK
        instr_c = 16'h9002;
        #1;
        if (!is_compressed || illegal_c) begin
            $display("FAIL: C.EBREAK should be legal compressed instruction");
            $finish;
        end
        if (instr_out != 32'h0010_0073) begin
            $display("FAIL: C.EBREAK expansion mismatch, got %h", instr_out);
            $finish;
        end

        // Invalid quadrant-0 funct3 encoding should flag illegal.
        instr_c = 16'h2000;
        #1;
        if (!is_compressed) begin
            $display("FAIL: 0x2000 should be treated as compressed encoding");
            $finish;
        end
        if (!illegal_c) begin
            $display("FAIL: illegal compressed encoding not flagged");
            $finish;
        end

        // Non-compressed instruction (bits [1:0]=2'b11) should passthrough lower 16b.
        instr_c = 16'hABCF;
        #1;
        if (is_compressed) begin
            $display("FAIL: non-compressed encoding incorrectly flagged as compressed");
            $finish;
        end
        if (illegal_c) begin
            $display("FAIL: non-compressed encoding should not assert illegal_c");
            $finish;
        end
        if (instr_out != 32'h0000_ABCF) begin
            $display("FAIL: passthrough behavior mismatch, got %h", instr_out);
            $finish;
        end

        $display("PASS: RVC decompressor legality and expansion behavior verified");
        $finish;
    end
endmodule
