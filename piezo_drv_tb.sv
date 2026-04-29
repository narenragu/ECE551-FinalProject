module piezo_drv_tb();
    logic clk, rst_n;
    logic batt_low;
    logic fanfare;
    logic piezo, piezo_n;

    piezo_drv #(.FAST_SIM(1)) iDUT (
        .clk     (clk),
        .rst_n   (rst_n),
        .batt_low(batt_low),
        .fanfare (fanfare),
        .piezo   (piezo),
        .piezo_n (piezo_n)
    );

    //clk generation
    initial clk = 0;
    always #5 clk = ~clk;

    initial begin
        clk = 0;
        rst_n = 0;
        batt_low = 0;
        fanfare = 0;

        // Reset
        repeat(5) @(posedge clk);
        rst_n = 1;
        @(posedge clk);

        //assert fanfare
        fanfare = 1;
        @(posedge clk);
        fanfare = 0;

        repeat(4000000) @(posedge clk); 

        //assert batt_low
        batt_low = 1;

        repeat(2000000) @(posedge clk);

        //check if batt_low has higher priority than fanfare
        fanfare = 1;
        @(posedge clk);
        fanfare = 0;
        repeat(2000000) @(posedge clk);

        $display("Testbench completed");
        $stop();
    end
endmodule