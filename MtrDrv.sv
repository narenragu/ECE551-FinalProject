module MtrDrv(
    input clk, rst_n,
    input signed [11:0] lft_spd, rght_spd,
    input [11:0] vbatt,
    output lftPWM1, lftPWM2, rghtPWM1, rghtPWM2
);

    logic [12:0] scale;
    logic [12:0] scale_q;

    logic signed [11:0] lft_spd_q, rght_spd_q;

    logic signed [23:0] lft_prod, rght_prod;
    logic signed [23:0] lft_prod_q, rght_prod_q;

    logic signed [11:0] lft_scaled, rght_scaled;
    logic [11:0] lft_duty_comb, rght_duty_comb;
    logic [11:0] lft_duty, rght_duty;

<<<<<<< HEAD
    DutyScaleROM iDSROM(.clk(clk), .batt_level(vbatt[9:4]), .scale(scale));
=======
    DutyScaleROM iDSROM(
        .clk(clk),
        .batt_level(vbatt[9:4]),
        .scale(scale)
    );

    // Register multiplier inputs
    always_ff @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            scale_q    <= 13'd0;
            lft_spd_q  <= 12'sd0;
            rght_spd_q <= 12'sd0;
        end else begin
            scale_q    <= scale;
            lft_spd_q  <= lft_spd;
            rght_spd_q <= rght_spd;
        end
    end

    // Full multiply
    assign lft_prod  = lft_spd_q  * $signed({1'b0, scale_q});
    assign rght_prod = rght_spd_q * $signed({1'b0, scale_q});

    // Register multiplier outputs
    always_ff @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            lft_prod_q  <= 24'sd0;
            rght_prod_q <= 24'sd0;
        end else begin
            lft_prod_q  <= lft_prod;
            rght_prod_q <= rght_prod;
        end
    end
>>>>>>> a533a0e7516e4490893e6d328626292a22ed4de1

    // Saturate scaled values
    assign lft_scaled = (lft_prod_q[23:22] == 2'b01) ? 12'sh7FF :
                        (lft_prod_q[23:22] == 2'b10) ? 12'sh800 :
                        lft_prod_q[22:11];

    assign rght_scaled = (rght_prod_q[23:22] == 2'b01) ? 12'sh7FF :
                         (rght_prod_q[23:22] == 2'b10) ? 12'sh800 :
                         rght_prod_q[22:11];

    assign lft_duty_comb  = 12'h800 + lft_scaled;
    assign rght_duty_comb = 12'h800 - rght_scaled;

    // Register duty before PWM
    always_ff @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            lft_duty  <= 12'h800;
            rght_duty <= 12'h800;
        end else begin
            lft_duty  <= lft_duty_comb;
            rght_duty <= rght_duty_comb;
        end
    end

    PWM12 iLftPWM(
        .clk(clk),
        .rst_n(rst_n),
        .duty(lft_duty),
        .PWM1(lftPWM1),
        .PWM2(lftPWM2)
    );

    PWM12 iRghtPWM(
        .clk(clk),
        .rst_n(rst_n),
        .duty(rght_duty),
        .PWM1(rghtPWM1),
        .PWM2(rghtPWM2)
    );

endmodule