module navigate(clk,rst_n,strt_hdng,strt_mv,stp_lft,stp_rght,mv_cmplt,hdng_rdy,moving,
                en_fusion,at_hdng,lft_opn,rght_opn,frwrd_opn,frwrd_spd);
				
  parameter FAST_SIM = 1;		// speeds up incrementing of frwrd register for faster simulation
				
  input clk,rst_n;					// 50MHz clock and asynch active low reset
  input strt_hdng;					// indicates should start a new heading
  input strt_mv;					// indicates should start a new forward move
  input stp_lft;					// indicates should stop at first left opening
  input stp_rght;					// indicates should stop at first right opening
  input hdng_rdy;					// new heading reading ready....used to pace frwrd_spd increments
  output logic mv_cmplt;			// asserted when heading or forward move complete
  output logic moving;				// enables integration in PID and in inertial_integrator
  output en_fusion;					// Only enable fusion (IR reading affect on nav) when moving forward at decent speed.
  input at_hdng;					// from PID, indicates heading close enough to consider heading complete.
  input lft_opn,rght_opn,frwrd_opn;	// from IR sensors, indicates available direction.  Might stop at rise of lft/rght
  output reg [10:0] frwrd_spd;		// unsigned forward speed setting to PID
  
  logic lft_opn_dly, pe_lft_opn, rght_opn_dly, pe_rght_opn;		//delays and edge detectors for lft and rght opn signals
  logic [5:0] frwrd_inc;

  //state machine logic
  typedef enum logic [2:0]{IDLE, HDNG_CHANGE, MOVE, DEC, DEC_FAST} state_t;
  state_t state, nxt_state;
  logic init_frwrd, inc_frwrd, dec_frwrd, dec_frwrd_fast;

  localparam MAX_FRWRD = 11'h2A0;		// max forward speed
  localparam MIN_FRWRD = 11'h0D0;		// minimum duty at which wheels will turn
  
  ////////////////////////////////
  // Now form forward register //
  //////////////////////////////
  always_ff @(posedge clk, negedge rst_n)
    if (!rst_n)
	  frwrd_spd <= 11'h000;
	else if (init_frwrd)
	  frwrd_spd <= MIN_FRWRD;									// min speed to get motors moving
	else if (hdng_rdy && inc_frwrd && (frwrd_spd<MAX_FRWRD))	// max out at 400 of 7FF for control head room
	  frwrd_spd <= frwrd_spd + {5'h00,frwrd_inc};
	else if (hdng_rdy && (frwrd_spd>11'h000) && (dec_frwrd | dec_frwrd_fast))
	  frwrd_spd <= ((dec_frwrd_fast) && (frwrd_spd>{2'h0,frwrd_inc,3'b000})) ? frwrd_spd - {2'h0,frwrd_inc,3'b000} : // 8x accel rate
                    (dec_frwrd_fast) ? 11'h000 :	  // if non zero but smaller than dec amnt set to zero.
	                (frwrd_spd>{4'h0,frwrd_inc,1'b0}) ? frwrd_spd - {4'h0,frwrd_inc,1'b0} : // slow down at 2x accel rate
					11'h000;

  //posedge detectors for left and right opening signals
  always_ff @(posedge clk, negedge rst_n) begin
    if(!rst_n) begin
      lft_opn_dly <= 1'b0;
      rght_opn_dly <= 1'b0;
    end else begin
      lft_opn_dly <= lft_opn;
      rght_opn_dly <= rght_opn;
    end
  end

  //state transition logic
  always_ff @ (posedge clk, negedge rst_n) begin
    if(!rst_n) begin
      state <= IDLE;
    end else begin
      state <= nxt_state;
    end
  end

  //state machine nxt state and output logic
  always_comb begin
    nxt_state = state;
    moving = 0;
    init_frwrd = 0;
    inc_frwrd = 0;
    dec_frwrd = 0;
    dec_frwrd_fast = 0;
    mv_cmplt = 0;

    case(state)
      IDLE: begin
        if(strt_hdng) begin
          nxt_state = HDNG_CHANGE;
        end else if (strt_mv) begin
          nxt_state = MOVE;
          init_frwrd = 1;
        end
      end
      HDNG_CHANGE: begin
        moving = 1;
        if(at_hdng) begin
          nxt_state = IDLE;
          mv_cmplt = 1;
        end
      end
      MOVE: begin
        moving = 1;
        inc_frwrd = 1;
        if(~frwrd_opn) begin
          nxt_state = DEC_FAST;
        end else if(stp_lft && pe_lft_opn || stp_rght && pe_rght_opn) begin
          nxt_state = DEC;
        end
      end
      DEC: begin
        dec_frwrd = 1;
        moving = 1;
        if(frwrd_spd == 0) begin
          nxt_state = IDLE;
          moving = 0;
          mv_cmplt = 1;
        end
      end
      DEC_FAST: begin
        dec_frwrd_fast = 1;
        moving = 1;
        if(frwrd_spd == 0) begin
          nxt_state = IDLE;
          moving = 0;
          mv_cmplt = 1;
        end
      end
      default: begin
        nxt_state = state;
      end
    endcase
  end

  //frwrd_inc based on param FAST_SIM
  generate if(FAST_SIM) begin
    assign frwrd_inc = 6'h18;
  end else begin
    assign frwrd_inc = 6'h02;
  end
  endgenerate

  //assert en_fusion if frwrd_spd is above 1/2 MAX_FRWRD
  assign en_fusion = (frwrd_spd > (MAX_FRWRD >> 1)) ? 1'b1 : 1'b0;

  //assign posedge detect left and right openings
  assign pe_lft_opn = ~lft_opn_dly & lft_opn;
  assign pe_rght_opn = ~rght_opn_dly & rght_opn;

endmodule
  