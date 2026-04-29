module cmd_proc_tb();

reg clk, rst_n;
reg [15:0] cmd;
reg snd_cmd;

wire resp_rdy;
wire [7:0] resp;
wire TX_RX, RX_TX;
wire [15:0] cmd_to_proc;
wire cmd_rdy, clr_cmd_rdy, send_resp_wire;
wire strt_cal, strt_hdng, strt_mv, stp_lft, stp_rght, in_cal, cmd_md;
wire [11:0] dsrd_hdng;

// completion signals driven only by the double-flop logic below
logic cal_done, mv_cmplt, sol_cmplt;

// extra outputs so TB matches current module ports
wire cmd_sent, tx_done;

integer timeout_cnt;

initial clk = 0;
always #5 clk = ~clk;

RemoteComm iREMOTE(
    .clk(clk),
    .rst_n(rst_n),
    .cmd(cmd),
    .snd_cmd(snd_cmd),
    .RX(RX_TX),
    .resp(resp),
    .cmd_sent(cmd_sent),
    .resp_rdy(resp_rdy),
    .TX(TX_RX)
);

UART_wrapper iUART(
    .clk(clk),
    .rst_n(rst_n),
    .RX(TX_RX),
    .clr_cmd_rdy(clr_cmd_rdy),
    .trmt(send_resp_wire),
    .resp(8'hA5),
    .cmd_rdy(cmd_rdy),
    .cmd(cmd_to_proc),
    .tx_done(tx_done),
    .TX(RX_TX)
);

cmd_proc iDUT(
    .clk(clk),
    .rst_n(rst_n),
    .cmd(cmd_to_proc),
    .cmd_rdy(cmd_rdy),
    .clr_cmd_rdy(clr_cmd_rdy),
    .send_resp(send_resp_wire),
    .strt_cal(strt_cal),
    .cal_done(cal_done),
    .in_cal(in_cal),
    .strt_hdng(strt_hdng),
    .strt_mv(strt_mv),
    .stp_lft(stp_lft),
    .stp_rght(stp_rght),
    .dsrd_hdng(dsrd_hdng),
    .mv_cmplt(mv_cmplt),
    .sol_cmplt(sol_cmplt),
    .cmd_md(cmd_md)
);

task send_command;
    input [15:0] command;
    begin
        @(negedge clk);
        cmd = command;
        snd_cmd = 1'b1;
        @(negedge clk);
        snd_cmd = 1'b0;
    end
endtask

task wait_for_cmd_rdy;
    begin
        timeout_cnt = 0;
        while (!cmd_rdy && timeout_cnt < 200000) begin
            @(posedge clk);
            timeout_cnt = timeout_cnt + 1;
        end
        if (timeout_cnt >= 200000) begin
            $display("TIMEOUT: cmd_rdy never asserted");
            $stop;
        end
    end
endtask

task wait_for_cal_done;
    begin
        timeout_cnt = 0;
        while (!cal_done && timeout_cnt < 200000) begin
            @(posedge clk);
            timeout_cnt = timeout_cnt + 1;
        end
        if (timeout_cnt >= 200000) begin
            $display("TIMEOUT: cal_done never asserted");
            $stop;
        end
    end
endtask

task wait_for_resp_rdy;
    begin
        timeout_cnt = 0;
        while (!resp_rdy && timeout_cnt < 200000) begin
            @(posedge clk);
            timeout_cnt = timeout_cnt + 1;
        end
        if (timeout_cnt >= 200000) begin
            $display("TIMEOUT: resp_rdy never asserted");
            $stop;
        end
    end
endtask

// ---------------------------
// Double-flopped completion generation
// ---------------------------

// cal_done asserted 2 clocks after strt_cal
logic strt_cal_ff1;
always_ff @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin
        strt_cal_ff1 <= 1'b0;
        cal_done     <= 1'b0;
    end else begin
        strt_cal_ff1 <= strt_cal;
        cal_done     <= strt_cal_ff1;
    end
end

// mv_cmplt asserted 2 clocks after strt_hdng or strt_mv
logic mv_start_ff1;
always_ff @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin
        mv_start_ff1 <= 1'b0;
        mv_cmplt     <= 1'b0;
    end else begin
        mv_start_ff1 <= strt_hdng | strt_mv;
        mv_cmplt     <= mv_start_ff1;
    end
end

// sol_cmplt asserted 2 clocks after cmd_md goes low
logic solve_ff1;
always_ff @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin
        solve_ff1 <= 1'b0;
        sol_cmplt <= 1'b0;
    end else begin
        solve_ff1 <= ~cmd_md;
        sol_cmplt <= solve_ff1;
    end
end

initial begin
    // Sequence 1
    rst_n      = 0;
    snd_cmd    = 0;
    cmd        = 16'h0000;

    repeat(2) @(negedge clk);
    rst_n = 1;
    repeat(2) @(negedge clk);

    // Sequence 2: Calibrate command
    send_command(16'h0000);
    wait_for_cmd_rdy;

    if (strt_cal)
        $display("PASS: strt_cal asserted at cmd_rdy");
    else
        $display("FAIL: strt_cal not asserted at cmd_rdy");

    @(posedge clk);
    if (in_cal)
        $display("PASS: in_cal asserted");
    else
        $display("FAIL: in_cal not asserted");

    wait_for_cal_done;
    $display("PASS: cal_done asserted");

    wait_for_resp_rdy;
    if (resp == 8'hA5)
        $display("PASS: calibrate got 0xA5 response");
    else
        $display("FAIL: calibrate expected 0xA5, got 0x%0h", resp);

    // Sequence 3: Heading command
    send_command(16'h23FF);   // 001 + 12'h3FF
    wait_for_cmd_rdy;

    if (strt_hdng)
        $display("PASS: strt_hdng asserted at cmd_rdy");
    else
        $display("FAIL: strt_hdng not asserted at cmd_rdy");

    @(posedge clk);
    if (dsrd_hdng === 12'h3FF)
        $display("PASS: dsrd_hdng = 12'h3FF");
    else
        $display("FAIL: dsrd_hdng = 12'h%0h, expected 12'h3FF", dsrd_hdng);

    wait_for_resp_rdy;
    if (resp == 8'hA5)
        $display("PASS: heading got 0xA5 response");
    else
        $display("FAIL: heading expected 0xA5, got 0x%0h", resp);

    // Sequence 4: Move command
    send_command(16'h4002);   // 010 with cmd[1]=1, cmd[0]=0
    wait_for_cmd_rdy;

    if (strt_mv)
        $display("PASS: strt_mv asserted at cmd_rdy");
    else
        $display("FAIL: strt_mv not asserted at cmd_rdy");

    @(posedge clk);
    if (stp_lft && !stp_rght)
        $display("PASS: stp_lft asserted correctly");
    else
        $display("FAIL: stp_lft=%0b stp_rght=%0b", stp_lft, stp_rght);

    wait_for_resp_rdy;
    if (resp == 8'hA5)
        $display("PASS: move got 0xA5 response");
    else
        $display("FAIL: move expected 0xA5, got 0x%0h", resp);

    // Sequence 5: Solve command
    send_command(16'h6001);   // 011 ...
    wait_for_cmd_rdy;

    @(posedge clk);
    if (!cmd_md)
        $display("PASS: cmd_md low one clock after cmd_rdy");
    else
        $display("FAIL: cmd_md still high one clock after cmd_rdy");

    $display("All tests completed");
    $stop;
end

endmodule