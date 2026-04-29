module PWM12_tb();
    logic clk, rst_n;
    logic [11:0] duty;
    logic PWM1, PWM2;
    
    PWM12 iPWM1(.clk(clk), .rst_n(rst_n), .duty(duty), .PWM1(PWM1), .PWM2(PWM2));

    initial begin
        clk = 0;
        rst_n = 0;
        //lower than nonoverlap 12'h02C
        duty = 12'h010;
        @ (posedge clk) rst_n = 1;
        repeat (4096) @ (posedge clk);

        //low but above nonoverlap
        duty = 12'h030;
        rst_n = 0;
        @ (posedge clk) rst_n = 1;
        repeat (4096) @ (posedge clk);

        //half duty cycle
        duty = 12'h800;
        rst_n = 0;
        @ (posedge clk) rst_n = 1;
        repeat (4096) @ (posedge clk);

        //high duty cycle
        duty = 12'hF00;
        rst_n = 0;
        @ (posedge clk) rst_n = 1;
        repeat (4096) @ (posedge clk);

        //max duty cycle
        duty = 12'hFFD;
        rst_n = 0;
        @ (posedge clk) rst_n = 1;
        repeat (4096) @ (posedge clk);

        $display("tb finished");
        $stop();
    end

    always #5 clk = ~clk;

endmodule