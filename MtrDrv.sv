module MtrDrv(
    input clk, rst_n,
    input signed [11:0] lft_spd, rght_spd,
    input [11:0] vbatt,
    output lftPWM1, lftPWM2, rghtPWM1, rghtPWM2
);

    logic [12:0] scale;
    logic signed [23:0] lft_prod, rght_prod;
    logic signed [11:0] lft_scaled, rght_scaled;
    logic [11:0] lft_duty, rght_duty;

    DutyScaleROM iDSROM(.clk(clk), .batt_level(vbatt[9:4]), .scale(scale));

    assign lft_prod  = lft_spd  * $signed({1'b0, scale});
    assign rght_prod = rght_spd * $signed({1'b0, scale});

    assign lft_scaled  = (lft_prod[23:22]  == 2'b01) ? 12'h7FF :
                         (lft_prod[23:22]  == 2'b10) ? 12'h800 :
                         lft_prod[22:11];

    assign rght_scaled = (rght_prod[23:22] == 2'b01) ? 12'h7FF :
                         (rght_prod[23:22] == 2'b10) ? 12'h800 :
                         rght_prod[22:11];

    assign lft_duty  = 12'h800 + lft_scaled;
    assign rght_duty = 12'h800 - rght_scaled;

    // Left motor PWM
    PWM12 iLftPWM(
        .clk(clk),
        .rst_n(rst_n),
        .duty(lft_duty),
        .PWM1(lftPWM1),
        .PWM2(lftPWM2)
    );

    // Right motor PWM
    PWM12 iRghtPWM(
        .clk(clk),
        .rst_n(rst_n),
        .duty(rght_duty),
        .PWM1(rghtPWM1),
        .PWM2(rghtPWM2)
    );

endmodule