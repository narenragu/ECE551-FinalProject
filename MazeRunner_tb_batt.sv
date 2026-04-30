`timescale 1ns/1ps

module MazeRunner_tb_batt();

  reg clk, RST_n;
  reg send_cmd;
  reg [15:0] cmd;
  reg [11:0] batt;

  logic cmd_sent;
  logic resp_rdy;
  logic [7:0] resp;
  logic hall_n;

  wire TX_RX, RX_TX;
  wire INRT_SS_n, INRT_SCLK, INRT_MOSI, INRT_MISO, INRT_INT;
  wire lftPWM1, lftPWM2, rghtPWM1, rghtPWM2;
  wire A2D_SS_n, A2D_SCLK, A2D_MOSI, A2D_MISO;
  wire IR_lft_en, IR_cntr_en, IR_rght_en;
  wire piezo, piezo_n;

  MazeRunner iDUT(.clk(clk), .RST_n(RST_n), .INRT_SS_n(INRT_SS_n), .INRT_SCLK(INRT_SCLK),
                  .INRT_MOSI(INRT_MOSI), .INRT_MISO(INRT_MISO), .INRT_INT(INRT_INT),
                  .A2D_SS_n(A2D_SS_n), .A2D_SCLK(A2D_SCLK), .A2D_MOSI(A2D_MOSI),
                  .A2D_MISO(A2D_MISO), .lftPWM1(lftPWM1), .lftPWM2(lftPWM2),
                  .rghtPWM1(rghtPWM1), .rghtPWM2(rghtPWM2), .RX(RX_TX), .TX(TX_RX),
                  .hall_n(hall_n), .piezo(piezo), .piezo_n(piezo_n), .IR_lft_en(IR_lft_en),
                  .IR_rght_en(IR_rght_en), .IR_cntr_en(IR_cntr_en), .LED());

  RemoteComm iCMD(.clk(clk), .rst_n(RST_n), .RX(TX_RX), .TX(RX_TX), .cmd(cmd), .send_cmd(send_cmd),
                  .cmd_sent(cmd_sent), .resp_rdy(resp_rdy), .resp(resp));

  RunnerPhysics iPHYS(.clk(clk), .RST_n(RST_n), .SS_n(INRT_SS_n), .SCLK(INRT_SCLK), .MISO(INRT_MISO),
                      .MOSI(INRT_MOSI), .INT(INRT_INT), .lftPWM1(lftPWM1), .lftPWM2(lftPWM2),
                      .rghtPWM1(rghtPWM1), .rghtPWM2(rghtPWM2),
                      .IR_lft_en(IR_lft_en), .IR_cntr_en(IR_cntr_en), .IR_rght_en(IR_rght_en),
                      .A2D_SS_n(A2D_SS_n), .A2D_SCLK(A2D_SCLK), .A2D_MOSI(A2D_MOSI),
                      .A2D_MISO(A2D_MISO), .hall_n(hall_n), .batt(batt));

  // helper: display battery voltage
  function automatic integer batt_mv(input [7:0] v);
    batt_mv = 4740 + (integer'(v) - 192) * 1560 / 63;
  endfunction

  task automatic display_battery();
    integer mv;
    mv = batt_mv(iDUT.vbatt[11:4]);
    $display("Battery: batt=0x%h vbatt[11:4]=0x%h (%0d.%0dV) batt_low=%b",
             batt, iDUT.vbatt[11:4], mv/1000, (mv%1000)/100, iDUT.batt_low);
  endtask

  // wait for vbatt averaging after changing batt
  task automatic wait_for_battery_to_settle();
    repeat(100_000) @(negedge clk);
  endtask

  task automatic wait_for_ack_silent();
    fork
        begin
            @(posedge resp_rdy);
            if (resp !== 8'hA5)
                $fatal(1, "ERROR: expected 0xA5, got 0x%h", resp);
        end
        begin
            repeat(20_000_000) @(posedge clk);
            $fatal(1, "ERROR: timeout waiting for ACK");
        end
    join_any
    disable fork;
  endtask

  task automatic send_command(input [15:0] command);
    @(negedge clk);
    cmd = command;
    send_cmd = 1'b1;
    @(negedge clk);
    send_cmd = 1'b0;
  endtask

  task automatic send_command_wait_ack_silent(input [15:0] command);
    send_command(command);
    @(posedge cmd_sent);
    wait_for_ack_silent();
  endtask

  // check motor duty symmetry
  task automatic check_motor_duty_match();
    logic [12:0] sum;
    sum = iDUT.iMTR.lft_duty + iDUT.iMTR.rght_duty;
    if (sum < 13'h0F00 || sum > 13'h1100)
        $fatal(1, "ERROR: motor symmetry failed lft=0x%h rght=0x%h sum=0x%h (expected ~0x1000)",
               iDUT.iMTR.lft_duty, iDUT.iMTR.rght_duty, sum);
  endtask

  // check motor duties are symmetric throughout a move
  task automatic check_duty_during_move();
    fork
        begin
            repeat(50) begin
                repeat(10_000) @(posedge clk);
                check_motor_duty_match();
            end
        end
        begin
            wait_for_ack_silent();
        end
    join_any
    disable fork;
    if (!resp_rdy)
        wait_for_ack_silent();
    $display("PASS: Motor duty symmetric during move");
    endtask

  task automatic check_duty_compensation(input [11:0] lft_duty_prev);
    logic [11:0] prev_offset, curr_offset;
    prev_offset = (lft_duty_prev > 12'h800) ? lft_duty_prev - 12'h800 : 12'h800 - lft_duty_prev;
    curr_offset = (iDUT.iMTR.lft_duty > 12'h800) ? iDUT.iMTR.lft_duty - 12'h800 : 12'h800 - iDUT.iMTR.lft_duty;
    if (curr_offset <= prev_offset)
        $display("WARN: duty compensation may not have increased (prev_offset=0x%h curr_offset=0x%h)",
                 prev_offset, curr_offset);
    else
        $display("PASS: Duty compensation increased at lower battery");
  endtask

  initial begin
    $display("================================");
    $display("TEST: MazeRunner battery test");
    $display("================================");

    clk = 0;
    batt = 12'hFFF; // fresh battery
    send_cmd = 0;
    cmd = 0;

    RST_n = 0;
    repeat(5) @(negedge clk);
    RST_n = 1;
    repeat(5) @(negedge clk);
    repeat(200_000) @(negedge clk);

    $display("");

    //-------------------------------------------------------------
    // TEST 1: batt_low should NOT assert at high battery
    //-------------------------------------------------------------
    $display("TEST 1: batt_low deasserted at high battery");
    wait_for_battery_to_settle();
    display_battery();
    if (iDUT.batt_low)
        $fatal(1, "ERROR: batt_low asserted at high battery (batt=0x%h vbatt=0x%h)",
               batt, iDUT.vbatt);
    $display("PASS: batt_low deasserted at high battery");

    $display("");
    //-------------------------------------------------------------
    // TEST 2: piezo should NOT be toggling at high battery (no batt_low)
    //-------------------------------------------------------------
    $display("TEST 2: piezo not toggling at high battery");
    begin
        integer toggle_count;
        logic prev_piezo;
        toggle_count = 0;
        prev_piezo = piezo;
        repeat(100_000) @(posedge clk) begin
            if (piezo !== prev_piezo) begin
                toggle_count = toggle_count + 1;
                prev_piezo = piezo;
            end
        end
        if (toggle_count > 0)
            $fatal("WARN: piezo toggled %0d times at high battery", toggle_count);
        else
            $display("PASS: piezo did not toggle at high battery");
    end

    $display("");
    //-------------------------------------------------------------
    // TEST 3: calibrate and run a forward move at high battery
    //         check motor duty symmetry
    //-------------------------------------------------------------
    $display("TEST 3: Motor duty at high battery");
    send_command_wait_ack_silent(16'h0000); // calibrate
    send_command_wait_ack_silent(16'h27FF); // heading south
    send_command(16'h4002);                 // move forward
    @(posedge cmd_sent);
    // sample duty mid-move
    repeat(50_000) @(posedge clk);
    begin
        logic [11:0] lft_duty_high_batt;
        lft_duty_high_batt = iDUT.iMTR.lft_duty;
        $display("High battery duty: lft=0x%h rght=0x%h", iDUT.iMTR.lft_duty, iDUT.iMTR.rght_duty);
        check_duty_during_move();
    $display("");

        //-------------------------------------------------------------
        // TEST 4: lower battery, duty should increase to compensate
        //-------------------------------------------------------------
        $display("TEST 4: Motor duty compensation at lower battery");
        batt = 12'hD80; // nominal/lower battery
        wait_for_battery_to_settle();
        display_battery();
        send_command_wait_ack_silent(16'h2000); // heading north
        send_command(16'h4002);                 // move forward
        @(posedge cmd_sent);
        repeat(50_000) @(posedge clk);
        $display("Low battery duty: lft=0x%h rght=0x%h", iDUT.iMTR.lft_duty, iDUT.iMTR.rght_duty);
        check_duty_compensation(lft_duty_high_batt);
        check_duty_during_move();
    end
    $display("");

    //-------------------------------------------------------------
    // TEST 5: batt_low asserts at low battery
    //-------------------------------------------------------------
    $display("TEST 5: batt_low asserts at low battery");
    batt = 12'hC00; // below threshold
    wait_for_battery_to_settle();
    display_battery();
    if (!iDUT.batt_low)
        $fatal(1, "ERROR: batt_low not asserted at low battery (batt=0x%h vbatt=0x%h)",
               batt, iDUT.vbatt);
    $display("PASS: batt_low asserted at low battery");
    $display("");

    //-------------------------------------------------------------
    // TEST 6: piezo should toggle when batt_low (low battery tone)
    //-------------------------------------------------------------
    $display("TEST 6: piezo toggles when batt_low");
    begin
        integer toggle_count;
        logic prev_piezo;
        toggle_count = 0;
        prev_piezo = piezo;
        repeat(10_000_000) @(posedge clk) begin
            if (piezo !== prev_piezo) begin
                toggle_count = toggle_count + 1;
                prev_piezo = piezo;
            end
        end
        if (toggle_count == 0)
            $fatal(1, "ERROR: piezo not toggling while batt_low asserted");
        $display("PASS: piezo toggled %0d times with batt_low", toggle_count);
    end
    $display("");

    //-------------------------------------------------------------
    // TEST 7: restore high battery, batt_low should deassert
    //-------------------------------------------------------------
    $display("TEST 7: batt_low deasserts when battery restored");
    batt = 12'hFFF;
    wait_for_battery_to_settle();
    display_battery();
    if (iDUT.batt_low)
        $fatal(1, "ERROR: batt_low still asserted after battery restored");
    $display("PASS: batt_low deasserted after battery restored");

    repeat(5) @(negedge clk);
    $display("");
    $display("All battery tests passed!");
    $stop();
  end

  always #5 clk = ~clk;

endmodule