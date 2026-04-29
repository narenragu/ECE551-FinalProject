module PWM12_tb();

    reg clk;
    reg rst_n;
    reg [11:0] duty;

    wire PWM1;
    wire PWM2;

    PWM12 iDUT (
        .clk(clk),
        .rst_n(rst_n),
        .duty(duty),
        .PWM1(PWM1),
        .PWM2(PWM2)
    );

    always #5 clk = ~clk;

    initial begin
        clk = 0;

        /*
        duty = 12'd1;
        rst_n = 0;
        repeat(2) @(posedge clk);
        rst_n = 1;
        @(negedge PWM1);
        repeat(2) @(posedge PWM1); // wait for 2 rising edges of PWM1
        */

        
        duty = 12'd1000;
        rst_n = 0;
        repeat(2) @(posedge clk);
        rst_n = 1;
        @(negedge PWM1);
        repeat(2) @(posedge PWM1); // wait for 2 rising edges of PWM1
        
        duty = 12'd2000;
        rst_n = 0;
        repeat(2) @(posedge clk);
        rst_n = 1;
        @(negedge PWM1);
        repeat(2) @(posedge PWM1); // wait for 2 rising edges of PWM1
        
        duty = 12'd3000;
        rst_n = 0;
        repeat(2) @(posedge clk);
        rst_n = 1;
        @(negedge PWM1);
        repeat(2) @(posedge PWM1); // wait for 2 rising edges of PWM1

        $stop;
    end

endmodule