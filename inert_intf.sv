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
  
  // Fix 1: read commands need bit 7 set (0xA6/0xA7 not 0x26/0x27)
  // Fix 2: READL splits into READL_CAP + READH_ST to avoid capturing
  //        yawL on the same cycle as the next wrt (rspns is combinational)
  typedef enum logic [3:0] {INIT1, INIT2, INIT3, ENDINIT,
                             WAIT, READL, READL_CAP, READH_ST,
                             READH, VLD_ST} state_t;
  state_t state, nxt_state;
  
  SPI_main iSPI(.clk(clk),.rst_n(rst_n),.SS_n(SS_n),.SCLK(SCLK),
                .MISO(MISO),.MOSI(MOSI),.wrt(wrt),.done(done),
                .rspns(inert_data),.cmd(cmd));
				  
  inertial_integrator #(.FAST_SIM(FAST_SIM)) iINT(
      .clk(clk), .rst_n(rst_n), .strt_cal(strt_cal),
      .vld(vld), .rdy(rdy), .cal_done(cal_done),
      .yaw_rt(yaw_rt), .moving(moving),
      .en_fusion(en_fusion), .IR_Dtrm(IR_Dtrm), .heading(heading));

  // Double-flop INT for metastability
  always_ff @(posedge clk, negedge rst_n) begin
    if(!rst_n) begin
      INT1 <= 1'b0;
      INT2 <= 1'b0;
    end else begin
      INT1 <= INT;
      INT2 <= INT1;
    end
  end

  // 16-bit timer — saturates at 0xFFFF for init delay
  logic [15:0] timer;
  always_ff @(posedge clk, negedge rst_n) begin
    if(!rst_n)
      timer <= 16'h0000;
    else if(timer != 16'hFFFF)
      timer <= timer + 1;
  end

  // yawL holding register
  always_ff @(posedge clk, negedge rst_n) begin
    if(!rst_n)
      yawL <= 8'h00;
    else if(C_Y_L)
      yawL <= inert_data[7:0];
  end

  // yawH holding register
  always_ff @(posedge clk, negedge rst_n) begin
    if(!rst_n)
      yawH <= 8'h00;
    else if(C_Y_H)
      yawH <= inert_data[7:0];
  end

  assign yaw_rt = {yawH, yawL};

  // State register
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

      // Wait for timer to max out, then send first init write
      INIT1: begin
        cmd = 16'h0D02;
        if(timer == 16'hFFFF) begin
          wrt = 1;
          nxt_state = INIT2;
        end
      end

      // Wait for INIT1 transaction to complete, send second init write
      INIT2: begin
        cmd = 16'h1160;
        if(done) begin
          wrt = 1;
          nxt_state = INIT3;
        end
      end

      // Wait for INIT2 transaction to complete, send third init write
      INIT3: begin
        cmd = 16'h1440;
        if(done) begin
          wrt = 1;
          nxt_state = ENDINIT;
        end
      end

      // Wait for INIT3 transaction to complete, then go to WAIT
      ENDINIT: begin
        cmd = 16'h1440;
        if(done)
          nxt_state = WAIT;
      end

      // Wait for INT (double-flopped), then start reading yawL
      // Fix 1: use 0xA600 (bit7=1 = read) not 0x2600
      WAIT: begin
        if(INT2) begin
          cmd = 16'hA600;
          wrt = 1;
          nxt_state = READL;
        end
      end

      // Wait for yawL read transaction to complete
      // Fix 2: do NOT fire wrt here — capture first, then send next cmd
      READL: begin
        cmd = 16'hA600;
        if(done)
          nxt_state = READL_CAP;
      end

      // Capture yawL now (inert_data stable, no wrt this cycle)
      // then immediately transition to fire the yawH read
      READL_CAP: begin
        C_Y_L = 1;              // safe to capture — no wrt this cycle
        cmd   = 16'hA700;       // Fix 1: bit7=1 = read addr 0x27
        wrt   = 1;
        nxt_state = READH;
      end

      // Wait for yawH read transaction to complete, then capture
      READH: begin
        cmd = 16'hA700;
        if(done) begin
          C_Y_H = 1;
          nxt_state = VLD_ST;
        end
      end

      // Assert vld for one cycle to tell inertial_integrator new data ready
      VLD_ST: begin
        vld = 1;
        nxt_state = WAIT;
      end

      default: nxt_state = INIT1;

    endcase
  end
 
endmodule