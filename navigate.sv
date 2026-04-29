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

    logic lft_opn_prev, rght_opn_prev;
    logic lft_opn_edge, rght_opn_edge;
    logic [5:0] frwrd_inc;
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

    // Edge detectors for lft_opn and rght_opn
    always_ff @(posedge clk or negedge rst_n)
        if (!rst_n) begin
            lft_opn_prev <= 1'b0;
            rght_opn_prev <= 1'b0;
        end else begin
            lft_opn_prev <= lft_opn;
            rght_opn_prev <= rght_opn;
    end

    assign lft_opn_edge = lft_opn & ~lft_opn_prev;
    assign rght_opn_edge = rght_opn & ~rght_opn_prev;

    // frwrd_inc value based on FAST_SIM parameter
    generate
        if (FAST_SIM) begin
            assign frwrd_inc = 6'h18;
        end else begin
            assign frwrd_inc = 6'h02;
        end 
    endgenerate

    // en_fusion asserted when speed greater than 1/2 max speed
    assign en_fusion = frwrd_spd > (MAX_FRWRD >> 1);

    // FSM
    typedef enum logic [2:0] {
        IDLE,
        HEADING_CHANGE,
        MOVING_FORWARD,
        DECELERATE,
        DECELERATE_FAST
    } state_t;

    state_t state, next_state;

    always_comb begin
        // default values
        moving = 1'b0;
        init_frwrd = 1'b0;
        inc_frwrd = 1'b0;
        dec_frwrd = 1'b0;
        dec_frwrd_fast = 1'b0;
        mv_cmplt = 1'b0;
        next_state = state;

        case(state)
            IDLE: begin
                if(strt_hdng) begin
                    next_state = HEADING_CHANGE;
                end else if (strt_mv) begin
                    init_frwrd = 1'b1;
                    next_state = MOVING_FORWARD;
                end
                else begin
                    next_state = IDLE;
                end
            end
            HEADING_CHANGE: begin
                moving = 1'b1;
                if (at_hdng) begin
                    mv_cmplt = 1'b1;
                    next_state = IDLE;
                end else begin
                    next_state = HEADING_CHANGE;
                end
            end
            MOVING_FORWARD: begin
                moving = 1'b1;
                inc_frwrd = 1'b1;
                if(~frwrd_opn) begin
                    next_state = DECELERATE_FAST;
                end
                else if((stp_lft && lft_opn_edge) || (stp_rght && rght_opn_edge)) begin
                    next_state = DECELERATE;
                end else begin
                    next_state = MOVING_FORWARD;
                end
            end
            DECELERATE: begin
                moving = 1'b1;
                dec_frwrd = 1'b1;
                if(frwrd_spd == 11'h000) begin
                    mv_cmplt = 1'b1;
                    next_state = IDLE;
                end else begin
                    next_state = DECELERATE;
                end
            end
            DECELERATE_FAST: begin
                moving = 1'b1;
                dec_frwrd_fast = 1'b1;
                if(frwrd_spd == 11'h000) begin
                    mv_cmplt = 1'b1;
                    next_state = IDLE;
                end else begin
                    next_state = DECELERATE_FAST;
                end
            end
            default: begin
                next_state = IDLE;
            end
            
        endcase
    end

    always_ff @(posedge clk or negedge rst_n) begin
        if (!rst_n)
            state <= IDLE;
        else
            state <= next_state;
    end

endmodule
