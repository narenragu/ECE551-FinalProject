module PID(
    input clk, rst_n,
    input moving,
    input [11:0] dsrd_hdng, actl_hdng,
    input hdng_vld,
    input [10:0] frwrd_spd,
    output logic at_hdng,
    output logic signed [11:0] lft_spd, rght_spd
);

localparam signed [3:0] P_COEFF = 4'sh3;
localparam signed [4:0] D_COEFF = 5'sh0E;

// Stage 0: Heading error and saturation

logic signed [11:0] hdng_err;
logic signed [9:0] err_sat;

assign hdng_err = $signed(actl_hdng) - $signed(dsrd_hdng);

assign err_sat =
    (hdng_err[11] && (!hdng_err[10] || !hdng_err[9])) ? 10'sb1000000000 :
    (!hdng_err[11] && (hdng_err[10] || hdng_err[9])) ? 10'sb0111111111 :
    hdng_err[9:0];

// Stage 1: Register error/control values

logic signed [9:0] err_sat_q;
logic moving_q;
logic hdng_vld_q;
logic [10:0] frwrd_spd_q;

always_ff @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin
        err_sat_q   <= 10'sd0;
        moving_q    <= 1'b0;
        hdng_vld_q  <= 1'b0;
        frwrd_spd_q <= 11'd0;
    end else begin
        err_sat_q   <= err_sat;
        moving_q    <= moving;
        hdng_vld_q  <= hdng_vld;
        frwrd_spd_q <= frwrd_spd;
    end
end

// at_hdng from registered error

logic signed [9:0] err_abs_q;

assign err_abs_q = err_sat_q[9] ? -err_sat_q : err_sat_q;

always_ff @(posedge clk or negedge rst_n) begin
    if (!rst_n)
        at_hdng <= 1'b0;
    else
        at_hdng <= (err_abs_q < 10'd30);
end

// I term accumulator

logic signed [15:0] integrator;
logic signed [15:0] err_ext;
logic signed [15:0] sum;
logic ov;

assign err_ext = {{6{err_sat_q[9]}}, err_sat_q};
assign sum = integrator + err_ext;

assign ov = (integrator[15] == err_ext[15]) &&
            (sum[15] != integrator[15]);

always_ff @(posedge clk or negedge rst_n) begin
    if (!rst_n)
        integrator <= 16'sd0;
    else if (!moving_q)
        integrator <= 16'sd0;
    else if (hdng_vld_q && !ov)
        integrator <= sum;
end

// D term delay registers

logic signed [9:0] q1, q2;

always_ff @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin
        q1 <= 10'sd0;
        q2 <= 10'sd0;
    end else if (hdng_vld_q) begin
        q1 <= err_sat_q;
        q2 <= q1;
    end
end

// Stage 2: Compute and register P/I/D terms=

logic signed [13:0] P_term_q;
logic signed [11:0] I_term_q;
logic signed [12:0] D_term_q;

logic signed [10:0] D_diff;
logic signed [7:0] D_diff_sat;

logic moving_qq;
logic [10:0] frwrd_spd_qq;

assign D_diff = err_sat_q - q2;

assign D_diff_sat =
    (D_diff > 11'sd127)  ? 8'sd127  :
    (D_diff < -11'sd128) ? -8'sd128 :
    D_diff[7:0];

always_ff @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin
        P_term_q      <= 14'sd0;
        I_term_q      <= 12'sd0;
        D_term_q      <= 13'sd0;
        moving_qq     <= 1'b0;
        frwrd_spd_qq  <= 11'd0;
    end else begin
        P_term_q      <= err_sat_q * P_COEFF;
        I_term_q      <= integrator[15:4];
        D_term_q      <= D_diff_sat * D_COEFF;
        moving_qq     <= moving_q;
        frwrd_spd_qq  <= frwrd_spd_q;
    end
end

logic signed [14:0] P_SEXT15;
logic signed [14:0] I_SEXT15;
logic signed [14:0] D_SEXT15;
logic signed [14:0] PID_SUM;
logic signed [14:0] PID_DIV_8;
logic signed [11:0] frwrd_spd_ext;

assign P_SEXT15 = {{1{P_term_q[13]}}, P_term_q};
assign I_SEXT15 = {{3{I_term_q[11]}}, I_term_q};
assign D_SEXT15 = {{2{D_term_q[12]}}, D_term_q};

assign PID_SUM = P_SEXT15 + I_SEXT15 + D_SEXT15;
assign PID_DIV_8 = PID_SUM >>> 3;
assign frwrd_spd_ext = {1'b0, frwrd_spd_qq};

always_comb begin
    if (moving_qq) begin
        lft_spd  = frwrd_spd_ext + PID_DIV_8;
        rght_spd = frwrd_spd_ext - PID_DIV_8;
    end else begin
        lft_spd  = 12'sd0;
        rght_spd = 12'sd0;
    end
end

endmodule