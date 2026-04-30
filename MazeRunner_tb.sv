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
            repeat(10_000_000) @(posedge clk);
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

    RST_n = 0; // reset the system
    repeat(5) @(negedge clk);
    RST_n = 1; 
    repeat(5) @(negedge clk);

    repeat(200_000) @(negedge clk);

    // send cmd to calibrate
    $display("CMD: Calibrate");
    send_command_wait_ack(16'h0000);

    pulse_marker;

    // send cmd to set heading to north
    $display("CMD: Set heading north");
    send_command_wait_ack(16'h2000); // 3'b001 (heading) + 12'h000 (north)

    // send cmd to move
    $display("CMD: Move forward");
    send_command_wait_ack(16'h4002); // 3'b010 (move) + cmd[1] = 1 (stop left)

    // expected to not move since front is blocked
    // expected left right and forward not open

    pulse_marker;
    
    // send cmd to set heading to south
    $display("CMD: Set heading south");
    send_command_wait_ack(16'h27FF); // 3'b001 (heading) + 12'h7FF (south)

    // send cmd to move
    $display("CMD: Move forward");
    send_command_wait_ack(16'h4002); // 3'b010 (move) + cmd[1] = 1 (stop left)

    // expected to move all the way down since no left opening in path
    // expected left right and forward not open

    pulse_marker;

    // send cmd to set heading to north
    $display("CMD: Set heading north");
    send_command_wait_ack(16'h2000); // 3'b001 (heading) + 12'h000 (north)

    // send cmd to move
    $display("CMD: Move forward");
    send_command_wait_ack(16'h4002); // 3'b010 (move) + cmd[1] = 1 (stop left)

    // expected to stop at left opening
    // expected left open, right and forward not open

    pulse_marker;

    /*
    // send cmd to solve
    $display("CMD: Solve maze: left affinity");
    send_command(16'h6001); // 3'b011 (solve) + cmd[0] = 1 (left affinity)
    @(posedge cmd_sent)
    wait_for_solve();

    $display("Maze solved!");
    */

    repeat(5) @(negedge clk);

    $stop();
  end
  
  always
    #5 clk = ~clk;
	
endmodule
