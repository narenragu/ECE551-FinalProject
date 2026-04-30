module PID(
    input clk, rst_n,
    input moving,
    input [11:0] dsrd_hdng, actl_hdng,
    input hdng_vld,
    input [10:0] frwrd_spd,
    output logic at_hdng,
    output logic signed [11:0] lft_spd, rght_spd
);

localparam signed P_COEFF = 4'sh3;
localparam signed D_COEFF = 5'sh0E;

logic signed [11:0] hdng_err;

assign hdng_err = actl_hdng - dsrd_hdng;

// PTERM

logic signed [13:0] P_term;
logic signed [9:0] err_sat;

assign err_sat = (hdng_err[11] == 1'b1 && (hdng_err[10] == 1'b0 || hdng_err[9] == 1'b0)) ? 10'b1000000000 : 
    (hdng_err[11] == 1'b0 && (hdng_err[10] == 1'b1 || hdng_err[9] == 1'b1)) ? 10'b0111111111 : hdng_err[9:0];

assign P_term = err_sat * P_COEFF;

logic signed [9:0] err_abs;
assign err_abs = err_sat[9] ? -err_sat : err_sat;
assign at_hdng = err_abs < 10'd30;

// ITERM

reg signed [15:0] integrator;
logic signed [15:0] err_ext;
logic signed [15:0] sum;
logic signed [15:0] accum_mux;
logic signed [15:0] nxt_integrator;
logic ov;
logic signed [11:0] I_term;

assign err_ext = {{6{err_sat[9]}}, err_sat};
assign sum = integrator + err_ext;
assign ov = (integrator[15] == err_ext[15]) && (sum[15] != integrator[15]);
assign accum_mux = (hdng_vld && !ov) ? sum : integrator;
assign nxt_integrator = moving ? accum_mux : 16'h0000;

always_ff @(posedge clk or negedge rst_n)
    if (!rst_n)
        integrator <= 16'h0000;
    else
        integrator <= nxt_integrator;

assign I_term = integrator[15:4];

// DTERM

logic signed [9:0] prev_err;
logic signed [10:0] D_diff;
logic signed [7:0] D_diff_sat;
logic signed [9:0] q1, q2;
logic signed [12:0] D_term;

always_ff @(posedge clk or negedge rst_n) begin
    if (!rst_n)
        q1 <= 10'b0;
    else if (hdng_vld)
        q1 <= err_sat;
    else
        q1 <= q1;
end

always_ff @(posedge clk or negedge rst_n) begin
    if (!rst_n)
        q2 <= 10'b0;
    else if (hdng_vld)
        q2 <= q1;
    else
        q2 <= q2;
end

assign prev_err = q2;
assign D_diff = err_sat - prev_err;

assign D_diff_sat = (D_diff > 127)  ? 127  :
                    (D_diff < -128) ? -128 :
                    D_diff[7:0];

assign D_term = D_diff_sat * D_COEFF;

// PID SUM

logic [14:0] P_SEXT15;
logic [14:0] I_SEXT15;
logic [14:0] D_SEXT15;
logic [14:0] PID_SUM;
logic signed [14:0] PID_DIV_8;
logic signed [11:0] frwrd_spd_ext;

assign P_SEXT15 = {{1{P_term[13]}}, P_term};
assign I_SEXT15 = {{3{I_term[11]}}, I_term};
assign D_SEXT15 = {{2{D_term[12]}}, D_term};

assign PID_SUM = P_SEXT15 + I_SEXT15 + D_SEXT15;
assign PID_DIV_8 = $signed(PID_SUM) >>> 3;
assign frwrd_spd_ext = {1'b0, frwrd_spd};

assign lft_spd = (moving) ? (PID_DIV_8 + frwrd_spd_ext) : 12'h000;
assign rght_spd = (moving) ? (frwrd_spd_ext - PID_DIV_8) : 12'h000;

endmodule