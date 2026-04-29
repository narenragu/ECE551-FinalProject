module inert_intf(clk,rst_n,strt_cal,cal_done,heading,rdy,IR_Dtrm,
                  SS_n,SCLK,MOSI,MISO,INT,moving,en_fusion);

  parameter FAST_SIM = 0;

  input clk, rst_n;
  input MISO;
  input INT;
  input strt_cal;
  input moving;
  input en_fusion;
  input [8:0] IR_Dtrm;
  
  output cal_done;
  output signed [11:0] heading;
  output rdy;
  output SS_n, SCLK, MOSI;

  logic [7:0] yawL;			
  logic [7:0] yawH;		
  logic [15:0] cmd;
  logic INT1, INT2;

  logic wrt;
  logic C_Y_H, C_Y_L;
  logic vld;

  wire done;
  wire [15:0] inert_data;
  wire signed [15:0] yaw_rt;
  
  typedef enum logic [2:0] {INIT1, INIT2, INIT3, ENDINIT, WAIT, READL, READH, VLD_ST} state_t;
  state_t state, nxt_state;
  
  SPI_main iSPI(.clk(clk),.rst_n(rst_n),.SS_n(SS_n),.SCLK(SCLK),
                .MISO(MISO),.MOSI(MOSI),.wrt(wrt),.done(done),
                .rd_data(inert_data),.wrt_data(cmd));
				  
  inertial_integrator #(.FAST_SIM(FAST_SIM)) iINT(.clk(clk), .rst_n(rst_n), .strt_cal(strt_cal),
                        .vld(vld),.rdy(rdy),.cal_done(cal_done), .yaw_rt(yaw_rt),.moving(moving),
                        .en_fusion(en_fusion),.IR_Dtrm(IR_Dtrm),.heading(heading));

  always_ff @(posedge clk, negedge rst_n) begin
    if(!rst_n) begin
      INT1 <= 1'b0;
      INT2 <= 1'b0;
    end else begin
      INT1 <= INT;
      INT2 <= INT1;
    end
  end

  logic [15:0] timer;
  always_ff @(posedge clk, negedge rst_n) begin
    if(!rst_n)
      timer <= 16'h0000;
    else if(timer != 16'hFFFF)
      timer <= timer + 1;
  end

  always_ff @(posedge clk, negedge rst_n) begin
    if(!rst_n)
      yawL <= 8'h00;
    else if(C_Y_L)
      yawL <= inert_data[7:0];
  end

  always_ff @(posedge clk, negedge rst_n) begin
    if(!rst_n)
      yawH <= 8'h00;
    else if(C_Y_H)
      yawH <= inert_data[7:0];
  end

  assign yaw_rt = {yawH, yawL};

  always_ff @(posedge clk, negedge rst_n) begin
    if(!rst_n)
      state <= INIT1;
    else
      state <= nxt_state;
  end

  always_comb begin
    wrt       = 0;
    cmd       = 16'h0000;
    C_Y_H     = 0;
    C_Y_L     = 0;
    vld       = 0;
    nxt_state = state;

    case(state)
      INIT1: begin
        cmd = 16'h0D02;
        if(timer == 16'hFFFF) begin
          wrt = 1;
          nxt_state = INIT2;
        end
      end
      INIT2: begin
        cmd = 16'h1160;
        if(done) begin
          wrt = 1;
          nxt_state = INIT3;
        end
      end
      INIT3: begin
        cmd = 16'h1440;
        if(done) begin
          wrt = 1;
          nxt_state = ENDINIT; 
        end
      end
      ENDINIT: begin
        cmd = 16'h1440;
        if(done)
          nxt_state = WAIT;
      end
      WAIT: begin
        if(INT2) begin
          cmd = 16'h2600;
          wrt = 1;
          nxt_state = READL;
        end
      end
      READL: begin
        cmd = 16'h2600;
        if(done) begin
          C_Y_L = 1;
          cmd = 16'h2700;
          wrt = 1;
          nxt_state = READH;
        end
      end
      READH: begin
        cmd = 16'h2700;
        if(done) begin
          C_Y_H = 1;
          nxt_state = VLD_ST;
        end
      end
      VLD_ST: begin
        vld = 1;
        nxt_state = WAIT;
      end
    endcase
  end
 
endmodule