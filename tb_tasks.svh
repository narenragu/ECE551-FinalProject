//////////////////////
//  Helper tasks   //
////////////////////
task automatic wait_for_ack();
fork
    begin
        @(posedge resp_rdy);
        if (resp !== 8'hA5)
            $fatal(1, "ERROR: expected 0xA5, got 0x%h", resp);
        $display("PASS: ACK received: 0x%h", resp);
    end
    begin
        repeat(20_000_000) @(posedge clk);
        $fatal(1, "ERROR: timeout waiting for ACK");
    end
join_any
disable fork;
endtask

task automatic send_command;
input [15:0] command;
begin
    @(negedge clk);
    cmd = command;
    send_cmd = 1'b1;
    @(negedge clk);
    send_cmd = 1'b0;
end
endtask

task automatic send_command_wait_ack;
input [15:0] command;
begin
    send_command(command);
    @(posedge cmd_sent);
    wait_for_ack();
end
endtask

task automatic wait_for_solve();
fork
    begin
        @(negedge hall_n);
        $display("PASS: Magnet found! Maze solved at time %0t", $time);
        // wait for ACK after sol_cmplt
        @(posedge resp_rdy);
        if (resp !== 8'hA5)
            $fatal(1, "ERROR: expected 0xA5 after solve, got 0x%h", resp);
        $display("PASS: ACK received: 0x%h", resp);
    end
    begin
        repeat(20_000_000) @(posedge clk);
        $fatal(1, "ERROR: maze not solved within 20M cycles");
    end
join_any
disable fork;
endtask

task automatic check_heading(input [11:0] expected_hdng);
if (((iPHYS.heading_robot[19:8] - expected_hdng) & 12'hFFF) > 12'h080 &&
    ((expected_hdng - iPHYS.heading_robot[19:8]) & 12'hFFF) > 12'h080)
    $fatal(1, "ERROR: expected heading 0x%h, got 0x%h", expected_hdng, iPHYS.heading_robot[19:8]);
else
    $display("PASS: Heading correct (expected=0x%h actual=0x%h)", expected_hdng, iPHYS.heading_robot[19:8]);
endtask

task automatic check_position_cell(input [3:0] expected_x, input [3:0] expected_y);
if (iPHYS.xx[14:12] !== expected_x[2:0] || iPHYS.yy[14:12] !== expected_y[2:0]) begin
    $fatal(1, "ERROR: expected cell (%0d, %0d), got (%0d, %0d) [xx=0x%h yy=0x%h]",
            expected_x, expected_y,
            iPHYS.xx[14:12], iPHYS.yy[14:12],
            iPHYS.xx, iPHYS.yy);
end else begin
    $display("PASS: Cell correct: (%0d, %0d) [xx=0x%h yy=0x%h]",
                expected_x, expected_y, iPHYS.xx, iPHYS.yy);
end
endtask

task automatic monitor_maze_solve();
    if (!GATE_LEVEL) begin
        fork
            begin
                logic sol_cmplt_prev;
                sol_cmplt_prev = 0;
                forever @(posedge clk) begin
                    if (iDUT.iSLV.strt_hdng) begin
                        $display("");
                        case (iDUT.iSLV.dsrd_hdng)
                            12'h000: $display("MAZE_SOLVE: strt_hdng > NORTH (0x000) at %0t", $time);
                            12'h3FF: $display("MAZE_SOLVE: strt_hdng > WEST  (0x3FF) at %0t", $time);
                            12'h7FF: $display("MAZE_SOLVE: strt_hdng > SOUTH (0x7FF) at %0t", $time);
                            12'hC00: $display("MAZE_SOLVE: strt_hdng > EAST  (0xC00) at %0t", $time);
                            default: $display("MAZE_SOLVE: strt_hdng > 0x%h at %0t", iDUT.iSLV.dsrd_hdng, $time);
                        endcase
                    end
                    if (iDUT.iSLV.strt_mv) begin
                        $display("MAZE_SOLVE: strt_mv (stp_lft=%b stp_rght=%b) at %0t",
                                 iDUT.iSLV.stp_lft, iDUT.iSLV.stp_rght, $time);
                        $display("POSITION:   xx=0x%h yy=0x%h heading=0x%h",
                                 iPHYS.xx, iPHYS.yy, iPHYS.heading_robot[19:8]);
                        $display("");
                    end
                    if (iDUT.sol_cmplt && !sol_cmplt_prev)
                        $display("MAZE_SOLVE: sol_cmplt! xx=0x%h yy=0x%h at %0t",
                                 iPHYS.xx, iPHYS.yy, $time);
                    sol_cmplt_prev = iDUT.sol_cmplt;
                end
            end
        join_none
    end else begin
        $display("INFO: monitor_maze_solve disabled in gate level mode");
    end
endtask