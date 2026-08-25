`timescale 1ns / 1ps

module tb_qos_arbiter;
    localparam NUM_MASTERS = 4;
    localparam QW = 3;

    logic clk;
    logic rst;
    logic [NUM_MASTERS-1:0] req;
    logic [NUM_MASTERS*QW-1:0] qos_in;
    logic [NUM_MASTERS-1:0] lock;

    logic [NUM_MASTERS-1:0] grant;
    logic [$clog2(NUM_MASTERS)-1:0] grant_idx;
    logic grant_valid;

    integer g0;
    integer g1;
    integer g2;
    integer g3;
    integer cyc;
    integer timeout;

    qos_arbiter #(
        .NUM_MASTERS(NUM_MASTERS),
        .QOS_WIDTH(QW),
        .AGE_BITS(4),
        .AGE_BOOST_THRESH(4)
    ) dut (
        .clk(clk),
        .rst(rst),
        .req(req),
        .qos_in(qos_in),
        .lock(lock),
        .grant(grant),
        .grant_idx(grant_idx),
        .grant_valid(grant_valid)
    );

    always #5 clk = ~clk;

    task set_qos;
        input [2:0] q0;
        input [2:0] q1;
        input [2:0] q2;
        input [2:0] q3;
        begin
            qos_in[(0*QW) +: QW] = q0;
            qos_in[(1*QW) +: QW] = q1;
            qos_in[(2*QW) +: QW] = q2;
            qos_in[(3*QW) +: QW] = q3;
        end
    endtask

    initial begin
        clk = 1'b0;
        rst = 1'b1;
        req = 4'b0000;
        lock = 4'b0000;
        set_qos(3'd0, 3'd0, 3'd0, 3'd0);

        repeat (4) @(posedge clk);
        rst <= 1'b0;

        // 1) Fairness check under equal QoS and continuous requests.
        req <= 4'b1111;
        set_qos(3'd3, 3'd3, 3'd3, 3'd3);
        g0 = 0;
        g1 = 0;
        g2 = 0;
        g3 = 0;

        for (cyc = 0; cyc < 40; cyc = cyc + 1) begin
            @(posedge clk);
            if (grant_valid) begin
                case (grant_idx)
                    2'd0: g0 = g0 + 1;
                    2'd1: g1 = g1 + 1;
                    2'd2: g2 = g2 + 1;
                    2'd3: g3 = g3 + 1;
                    default: ;
                endcase
            end
        end

        if ((g0 == 0) || (g1 == 0) || (g2 == 0) || (g3 == 0)) begin
            $display("FAIL: fairness check failed, grant counts=%0d,%0d,%0d,%0d", g0, g1, g2, g3);
            $finish;
        end

        // 2) Starvation-prevention check: low QoS requester must still get service.
        req <= 4'b0011;
        set_qos(3'd0, 3'd7, 3'd0, 3'd0);
        g0 = 0;

        for (cyc = 0; cyc < 40; cyc = cyc + 1) begin
            @(posedge clk);
            if (grant_valid && grant_idx == 2'd0)
                g0 = g0 + 1;
        end

        if (g0 == 0) begin
            $display("FAIL: low-priority requester was starved despite age boost");
            $finish;
        end

        // 3) Lock check: locked master keeps grant while it continues requesting.
        req <= 4'b1111;
        lock <= 4'b0010; // master1 issues locked request
        set_qos(3'd3, 3'd3, 3'd3, 3'd3);

        timeout = 0;
        while ((!(grant_valid && grant_idx == 2'd1)) && (timeout < 80)) begin
            timeout = timeout + 1;
            @(posedge clk);
        end

        if (!(grant_valid && grant_idx == 2'd1)) begin
            $display("FAIL: lock holder never acquired grant");
            $finish;
        end

        for (cyc = 0; cyc < 8; cyc = cyc + 1) begin
            @(posedge clk);
            if (!(grant_valid && grant_idx == 2'd1)) begin
                $display("FAIL: lock holder lost grant while lock active");
                $finish;
            end
        end

        // Release lock owner request and confirm grant moves away.
        req[1] <= 1'b0;
        lock[1] <= 1'b0;

        timeout = 0;
        while ((grant_valid && grant_idx == 2'd1) && (timeout < 40)) begin
            timeout = timeout + 1;
            @(posedge clk);
        end

        if (grant_valid && grant_idx == 2'd1) begin
            $display("FAIL: grant did not release after lock owner dropped request");
            $finish;
        end

        $display("PASS: QoS arbiter fairness, starvation prevention, and lock behavior verified");
        $finish;
    end
endmodule
