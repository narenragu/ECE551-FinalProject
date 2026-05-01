`timescale 1ns/1ps

import MazeRunner_helper::*;

module MazeRunner_tb_new();
  
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

  //////////////////////
  //  Helper tasks   //
  ////////////////////
  `include "tb_tasks.svh"

  //////////////////////
  //      Test       //
  ////////////////////

  // north 12'h000
  // west  12'h3FF
  // south 12'h7FF
  // east  12'hC00

  // send some commands to the MazeRunner over bluetooth (RemoteComm) and observe responses
  initial begin
    $display("================================");
    $display("TEST: MazeRunner Testbench ");
    $display("================================");

    // initialize signals
    clk = 0;
    batt = 12'hFFF;
    send_cmd = 0;
    cmd = 0;

    RST_n = 0; // reset the system
    repeat(5) @(negedge clk);
    RST_n = 1; 
    repeat(5) @(negedge clk);

    repeat(200_000) @(negedge clk);

    // send cmd to calibrate
    $display("CMD: Calibrate");
    send_command_wait_ack(CMD_CALIBRATE);
    check_heading(HDG_NORTH);

    // send cmd to set heading to south
    $display("CMD: Set heading south");
    send_command_wait_ack(CMD_HDG_SOUTH);
    check_heading(HDG_SOUTH);

    // send cmd to move
    $display("CMD: Move forward");
    send_command_wait_ack(CMD_MOVE_STOP_R);
    check_position_cell(4'h3, 4'h2);
    $display("");
    
    // send cmd to set heading to west
    $display("CMD: Set heading west");
    send_command_wait_ack(CMD_HDG_WEST);
    check_heading(HDG_WEST);

    // send cmd to move
    $display("CMD: Move forward");
    send_command_wait_ack(CMD_MOVE_STOP_R);
    check_position_cell(4'h1, 4'h2);
    $display("");

    // send cmd to set heading to east
    $display("CMD: Set heading east");
    send_command_wait_ack(CMD_HDG_EAST);
    check_heading(HDG_EAST);

    // send cmd to move
    $display("CMD: Move forward");
    send_command_wait_ack(CMD_MOVE_STOP_R);
    check_position_cell(4'h3, 4'h2);
    $display("");

    repeat(5) @(negedge clk);

    $display("");
    $display("All test passed!");

    $stop();
  end
  
  always
    #5 clk = ~clk;
	
endmodule