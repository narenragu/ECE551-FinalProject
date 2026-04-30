module MazeRunner_tb();
  
  reg clk,RST_n;
  reg send_cmd;					// assert to send command to MazeRunner_tb
  reg [15:0] cmd;				// 16-bit command to send
  reg [11:0] batt;				// battery voltage 0xD80 is nominal
  
  logic cmd_sent;				
  logic resp_rdy;				// MazeRunner has sent a pos acknowledge
  logic [7:0] resp;				// resp byte from MazeRunner (hopefully 0xA5)
  logic hall_n;					// magnet found?
  
  /////////////////////////////////////////////////////////////////////////
  // Signals interconnecting MazeRunner to RunnerPhysics and RemoteComm //
  ///////////////////////////////////////////////////////////////////////
  wire TX_RX,RX_TX;
  wire INRT_SS_n,INRT_SCLK,INRT_MOSI,INRT_MISO,INRT_INT;
  wire lftPWM1,lftPWM2,rghtPWM1,rghtPWM2;
  wire A2D_SS_n,A2D_SCLK,A2D_MOSI,A2D_MISO;
  wire IR_lft_en,IR_cntr_en,IR_rght_en;  
  wire piezo;

  ///// Internal registers for testing purposes??? /////////
  logic marker;
  logic [14:0] prev_xx, prev_yy;
  
  //////////////////////
  // Instantiate DUT //
  ////////////////////
  MazeRunner iDUT(.clk(clk),.RST_n(RST_n),.INRT_SS_n(INRT_SS_n),.INRT_SCLK(INRT_SCLK),
                  .INRT_MOSI(INRT_MOSI),.INRT_MISO(INRT_MISO),.INRT_INT(INRT_INT),
				  .A2D_SS_n(A2D_SS_n),.A2D_SCLK(A2D_SCLK),.A2D_MOSI(A2D_MOSI),
				  .A2D_MISO(A2D_MISO),.lftPWM1(lftPWM1),.lftPWM2(lftPWM2),
				  .rghtPWM1(rghtPWM1),.rghtPWM2(rghtPWM2),.RX(RX_TX),.TX(TX_RX),
				  .hall_n(hall_n),.piezo(piezo),.piezo_n(),.IR_lft_en(IR_lft_en),
				  .IR_rght_en(IR_rght_en),.IR_cntr_en(IR_cntr_en),.LED());
	
  ///////////////////////////////////////////////////////////////////////////////////////
  // Instantiate RemoteComm which models bluetooth module receiving & forwarding cmds //
  /////////////////////////////////////////////////////////////////////////////////////
  RemoteComm iCMD(.clk(clk), .rst_n(RST_n), .RX(TX_RX), .TX(RX_TX), .cmd(cmd), .send_cmd(send_cmd),
               .cmd_sent(cmd_sent), .resp_rdy(resp_rdy), .resp(resp));
			   
  ///////////////////////////////////////////////////
  // Instantiate physical model of robot and maze //
  /////////////////////////////////////////////////
  RunnerPhysics iPHYS(.clk(clk),.RST_n(RST_n),.SS_n(INRT_SS_n),.SCLK(INRT_SCLK),.MISO(INRT_MISO),
                      .MOSI(INRT_MOSI),.INT(INRT_INT),.lftPWM1(lftPWM1),.lftPWM2(lftPWM2),
					  .rghtPWM1(rghtPWM1),.rghtPWM2(rghtPWM2),
                     .IR_lft_en(IR_lft_en),.IR_cntr_en(IR_cntr_en),.IR_rght_en(IR_rght_en),
					 .A2D_SS_n(A2D_SS_n),.A2D_SCLK(A2D_SCLK),.A2D_MOSI(A2D_MOSI),
					 .A2D_MISO(A2D_MISO),.hall_n(hall_n),.batt(batt));

  task automatic wait_for_ack();
    fork
        begin
            @(posedge resp_rdy);
            if (resp !== 8'hA5)
                $fatal(1, "ERROR: expected 0xA5, got 0x%h", resp);
            $display("ACK received: 0x%h", resp);
        end
        begin
            repeat(20_000_000) @(posedge clk);
            $fatal(1, "TIMEOUT waiting for ACK");
        end
    join_any
    disable fork;
  endtask

  task send_command;
    input [15:0] command;
    begin
        @(negedge clk);
        cmd = command;
        send_cmd = 1'b1;
        @(negedge clk);
        send_cmd = 1'b0;
    end
  endtask

  task send_command_wait_ack;
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
            $display("Magnet found! Maze solved at time %0t", $time);
            // wait for ACK after sol_cmplt
            @(posedge resp_rdy);
            if (resp !== 8'hA5)
                $fatal(1, "ERROR: expected 0xA5 after solve, got 0x%h", resp);
            $display("Solve ACK received: 0x%h", resp);
        end
        begin
            repeat(20_000_000) @(posedge clk);
            $fatal(1, "TIMEOUT: maze not solved within 10M cycles");
        end
    join_any
    disable fork;
  endtask

  task pulse_marker;
    marker = 1;
    @(negedge clk);
    marker = 0;
  endtask

  task automatic monitor_maze_solve();
    fork
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
            if (iDUT.sol_cmplt) begin
                $display("");
                $display("MAZE_SOLVE: sol_cmplt! xx=0x%h yy=0x%h at %0t",
                         iPHYS.xx, iPHYS.yy, $time);
            end
        end
    join_none
  endtask

  // north 12'h000
  // west  12'h3FF
  // south 12'h7FF
  // east  12'hC00


  // send some commands to the MazeRunner over bluetooth and observe responses
  initial begin
    // initialize signals
    clk = 0;
    batt = 12'hFFF; // nominal battery voltage
    send_cmd = 0;
    cmd = 0;
    marker = 0;
    prev_xx = 15'h3800;
    prev_yy = 15'h3800;

    RST_n = 0; // reset the system
    repeat(5) @(negedge clk);
    RST_n = 1; 
    repeat(5) @(negedge clk);

    repeat(200_000) @(negedge clk);

    // send cmd to calibrate
    $display("CMD: Calibrate");
    send_command_wait_ack(16'h0000);

    /*

    // send cmd to set heading to north
    $display("CMD: Set heading north");
    send_command_wait_ack(16'h2000); // 3'b001 (heading) + 12'h000 (north)

    // send cmd to move
    $display("CMD: Move forward");
    send_command_wait_ack(16'h4002); // 3'b010 (move) + cmd[1] = 1 (stop left)

    // expected to not move since front is blocked
    // expected left right and forward not open
    
    // send cmd to set heading to south
    $display("CMD: Set heading south");
    send_command_wait_ack(16'h27FF); // 3'b001 (heading) + 12'h7FF (south)

    // send cmd to move
    $display("CMD: Move forward");
    send_command_wait_ack(16'h4002); // 3'b010 (move) + cmd[1] = 1 (stop left)

    // expected to move all the way down since no left opening in path
    // expected left right and forward not open

    // send cmd to set heading to north
    $display("CMD: Set heading north");
    send_command_wait_ack(16'h2000); // 3'b001 (heading) + 12'h000 (north)

    // send cmd to move
    $display("CMD: Move forward");
    send_command_wait_ack(16'h4002); // 3'b010 (move) + cmd[1] = 1 (stop left)

    // expected to stop at left opening
    // expected left open, right and forward not open

    // send cmd to set heading to west
    $display("CMD: Set heading west");
    send_command_wait_ack(16'h23FF); // 3'b001 (heading) + 12'h3FF (west)

    // send cmd to move
    $display("CMD: Move forward");
    send_command_wait_ack(16'h4002); // 3'b010 (move) + cmd[1] = 1 (stop left)

    // expected to stop at left opening
    // expected left open, right and forward not open
    // should be above magnet

    */

    // send cmd to move
    $display("CMD: solve");
    send_command(16'h6001); // 3'b011 (solve) + cmd[0] = 1 (left affinity)
    @(posedge cmd_sent);
    monitor_maze_solve();
    wait_for_solve();

    // expected to stop at left opening
    // expected left open, right and forward not open
    // should be above magnet

    repeat(5) @(negedge clk);

    $stop();
  end
  
  always
    #5 clk = ~clk;
	
endmodule
