module MtrDrv(
    input clk, rst_n,
    input [11:0] lft_spd, rght_spd,
    input [11:0] vbatt,
    output lftPWM1, lftPWM2, rghtPWM1, rghtPWM2
);
    logic [12:0] scale;
    logic [23:0] lft_prod, rght_prod;
    logic [11:0] lft_scaled, rght_scaled;
    logic [11:0] lft_duty, rght_duty;

    DutyScaleROM iDSROM(clk, vbatt[9:4], scale);

    always_comb begin
        lft_prod = lft_spd * scale;
        rght_prod = rght_spd * scale;
    end

    //divide by 2048 & 12-bit saturation
    always_comb begin
        //lft side
        if(lft_prod[24:23] != 0) begin
            lft_scaled = 12'hFFF;
        end else begin
            lft_scaled = lft_prod[22:11];
        end

        //rght side
        if(rght_prod[24:23] != 0) begin
            rght_scaled = 12'hFFF;
        end else begin
            rght_scaled = rght_prod[22:11];
        end
    end

    //offset around 12'h800
    always_comb begin
        lft_duty = 12'h800 + lft_scaled;
        rght_duty = 12'h800 + rght_scaled;
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