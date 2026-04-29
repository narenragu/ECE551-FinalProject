module MtrDrv(
    input clk, rst_n,
    input [11:0] lft_spd, rght_spd,
    input [11:0] vbatt,
    output lftPWM1, lftPWM2, rghtPWM1, rghtPWM2
);
    logic [12:0] scale;
    logic [23:0] lft_prod, rght_prod;
    logic [11:0] lft_mag, rght_mag;
    logic lft_neg, rght_neg;
    logic [11:0] lft_scaled, rght_scaled;
    logic [11:0] lft_duty, rght_duty;

    DutyScaleROM iDSROM(clk, vbatt[9:4], scale);

    always_comb begin
        lft_neg   = lft_spd[11];
        rght_neg  = rght_spd[11];
        lft_mag   = lft_neg  ? (~lft_spd  + 1) : lft_spd;
        rght_mag  = rght_neg ? (~rght_spd + 1) : rght_spd;

        lft_prod  = lft_mag  * scale;
        rght_prod = rght_mag * scale;
    end

    //divide by 2048 & 12-bit saturation
    always_comb begin
        //lft side
        if (lft_prod[23:22] != 0)
            lft_scaled = 12'hFFF;
        else
            lft_scaled = lft_prod[22:11];

        //rght side
        if (rght_prod[23:22] != 0)
            rght_scaled = 12'hFFF;
        else
            rght_scaled = rght_prod[22:11];
    end

    //offset around 12'h800, subtract if negative
    always_comb begin
        lft_duty  = lft_neg  ? (12'h800 - lft_scaled)  : (12'h800 + lft_scaled);
        rght_duty = rght_neg ? (12'h800 - rght_scaled) : (12'h800 + rght_scaled);
    end

    //lft motor PWM
    PWM12 iLftPWM(
        .clk(clk),
        .rst_n(rst_n),
        .duty(lft_duty),
        .PWM1(lftPWM1),
        .PWM2(lftPWM2)
    );

    //rght motor PWM
    PWM12 iRghtPWM(
        .clk(clk),
        .rst_n(rst_n),
        .duty(rght_duty),
        .PWM1(rghtPWM1),
        .PWM2(rghtPWM2)
    );

endmodule