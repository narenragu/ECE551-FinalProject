module PID(
    input clk, rst_n,
    input moving,                                   //high if turning or moving forward
    input signed [11:0] dsrd_hdng, actl_hdng,       //signed headings
    input hdng_vld,                                 //high for 1 clk every new valid gyro reading
    input [10:0] frwrd_spd,                         //unsigned forward speed                 
    output at_hdng,                                 //asserted if error is small
    output signed [11:0] lft_spd, rght_spd          //signed motor speeds
);
    localparam signed P_coeff = 4'h3;

    logic signed [11:0] err;
    logic signed [9:0] err_sat;
    logic signed [13:0] P_out;
    logic signed [11:0] I_out;
    logic signed [12:0] D_out;
    logic [14:0] P_term, I_term, D_term;
    logic [14:0] PID_add_div8;
    logic [11:0] frwrd_spd_ext;

    assign err = actl_hdng - dsrd_hdng;
    //10 bit sat
    assign err_sat = (err[11:10] == {2{err[9]}}) ? 
        err[9:0] : (err[11] ? 10'h200 : 10'h1FF);
    
    //P Term
    assign P_out = P_coeff * err_sat;
    assign P_term = {P_out[13], P_out};

    //I Term
    I_term iITerm(.clk(clk), .rst_n(rst_n), .hdng_vld(hdng_vld), .moving(moving), .err_sat(err_sat), .I_term(I_out));
    assign I_term = {{3{I_out[11]}}, I_out};

    //D Term
    D_term iDTerm(.clk(clk), .rst_n(rst_n), .err_sat(err_sat), .hdng_vld(hdng_vld), .D_term(D_out));
    assign D_term = {{2{D_out[12]}}, D_out};

    //at hdng
    assign at_hdng = (err_sat < 0 ? -err_sat : err_sat) < 10'sd30; 

    //add PID values and divide by 8
    assign PID_add_div8 = (P_term + I_term + D_term) >>> 3;

    //frwd speed sign extend to 12 bits
    assign frwrd_spd_ext = {1'b0, frwrd_spd};

    //lft speed, rght speed
    assign lft_spd = moving ? (PID_add_div8 + $signed(frwrd_spd_ext)) : 12'h000;
    assign rght_spd = moving ? ($signed(frwrd_spd_ext) - PID_add_div8) : 12'h000;

endmodule