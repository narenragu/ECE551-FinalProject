module MtrDrv_tb;

    logic clk, rst_n;
    logic [11:0] lft_spd, rght_spd, vbatt;
    logic lftPWM1, lftPWM2, rghtPWM1, rghtPWM2;

    integer i;
    integer l1, l2, r1, r2;

    MtrDrv dut (
        .clk(clk),
        .rst_n(rst_n),
        .lft_spd(lft_spd),
        .rght_spd(rght_spd),
        .vbatt(vbatt),
        .lftPWM1(lftPWM1),
        .lftPWM2(lftPWM2),
        .rghtPWM1(rghtPWM1),
        .rghtPWM2(rghtPWM2)
    );

    initial clk = 1'b0;
    always #5 clk = ~clk;

    initial begin
        rst_n    = 1'b0;
        lft_spd  = 12'h000;
        rght_spd = 12'h000;
        vbatt    = 12'hDB0;

        repeat(5) @(posedge clk);
        rst_n = 1'b1;
        repeat(3) @(posedge clk);

        // TEST 1: zero input, expect about 50%
        $display("\nTEST 1: zero input @ vbatt[11:4]=DB");
        lft_spd  = 12'h000;
        rght_spd = 12'h000;
        vbatt    = 12'hDB0;
        repeat(3) @(posedge clk);

        l1 = 0; l2 = 0; r1 = 0; r2 = 0;
        for (i = 0; i < 4096; i = i + 1) begin
            @(posedge clk);
            if (lftPWM1)  l1 = l1 + 1;
            if (lftPWM2)  l2 = l2 + 1;
            if (rghtPWM1) r1 = r1 + 1;
            if (rghtPWM2) r2 = r2 + 1;
        end

        $display("LEFT : PWM1 = %0d/4096 = %0f%%, PWM2 = %0d/4096 = %0f%%",
                 l1, 100.0*l1/4096.0, l2, 100.0*l2/4096.0);
        $display("RIGHT: PWM1 = %0d/4096 = %0f%%, PWM2 = %0d/4096 = %0f%%",
                 r1, 100.0*r1/4096.0, r2, 100.0*r2/4096.0);

        // TEST 2: 0x3FF @ DB, expect about 75/25
        $display("\nTEST 2: 3FF input @ vbatt[11:4]=DB");
        lft_spd  = 12'h3FF;
        rght_spd = 12'h3FF;
        vbatt    = 12'hDB0;
        repeat(3) @(posedge clk);

        l1 = 0; l2 = 0; r1 = 0; r2 = 0;
        for (i = 0; i < 4096; i = i + 1) begin
            @(posedge clk);
            if (lftPWM1)  l1 = l1 + 1;
            if (lftPWM2)  l2 = l2 + 1;
            if (rghtPWM1) r1 = r1 + 1;
            if (rghtPWM2) r2 = r2 + 1;
        end

        $display("LEFT : PWM1 = %0d/4096 = %0f%%, PWM2 = %0d/4096 = %0f%%",
                 l1, 100.0*l1/4096.0, l2, 100.0*l2/4096.0);
        $display("RIGHT: PWM1 = %0d/4096 = %0f%%, PWM2 = %0d/4096 = %0f%%",
                 r1, 100.0*r1/4096.0, r2, 100.0*r2/4096.0);

        // TEST 3: 0x3FF @ D0, expect about 76.2/23
        $display("\nTEST 3: 3FF input @ vbatt[11:4]=D0");
        lft_spd  = 12'h3FF;
        rght_spd = 12'h3FF;
        vbatt    = 12'hD00;
        repeat(3) @(posedge clk);

        l1 = 0; l2 = 0; r1 = 0; r2 = 0;
        for (i = 0; i < 4096; i = i + 1) begin
            @(posedge clk);
            if (lftPWM1)  l1 = l1 + 1;
            if (lftPWM2)  l2 = l2 + 1;
            if (rghtPWM1) r1 = r1 + 1;
            if (rghtPWM2) r2 = r2 + 1;
        end

        $display("LEFT : PWM1 = %0d/4096 = %0f%%, PWM2 = %0d/4096 = %0f%%",
                 l1, 100.0*l1/4096.0, l2, 100.0*l2/4096.0);
        $display("RIGHT: PWM1 = %0d/4096 = %0f%%, PWM2 = %0d/4096 = %0f%%",
                 r1, 100.0*r1/4096.0, r2, 100.0*r2/4096.0);

        // TEST 4: 0xC00 @ FF, expect about 28.5/71.4
        $display("\nTEST 4: C00 input @ vbatt[11:4]=FF");
        lft_spd  = 12'hC00;
        rght_spd = 12'hC00;
        vbatt    = 12'hFF0;
        repeat(3) @(posedge clk);

        l1 = 0; l2 = 0; r1 = 0; r2 = 0;
        for (i = 0; i < 4096; i = i + 1) begin
            @(posedge clk);
            if (lftPWM1)  l1 = l1 + 1;
            if (lftPWM2)  l2 = l2 + 1;
            if (rghtPWM1) r1 = r1 + 1;
            if (rghtPWM2) r2 = r2 + 1;
        end

        $display("LEFT : PWM1 = %0d/4096 = %0f%%, PWM2 = %0d/4096 = %0f%%",
                 l1, 100.0*l1/4096.0, l2, 100.0*l2/4096.0);
        $display("RIGHT: PWM1 = %0d/4096 = %0f%%, PWM2 = %0d/4096 = %0f%%",
                 r1, 100.0*r1/4096.0, r2, 100.0*r2/4096.0);

        $stop();
    end

endmodule