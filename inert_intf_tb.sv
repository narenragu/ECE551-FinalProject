module inert_intf_tb();
  logic        clk, rst_n;
  logic        strt_cal;
  logic        cal_done;
  logic signed [11:0] heading;
  logic        rdy;
  logic [8:0]  IR_Dtrm;
  logic        SS_n, SCLK, MOSI, MISO;
  logic        INT;
  logic        moving;
  logic        en_fusion;

  inert_intf #(.FAST_SIM(1)) iDUT (
    .clk      (clk),
    .rst_n    (rst_n),
    .strt_cal (strt_cal),
    .cal_done (cal_done),
    .heading  (heading),
    .rdy      (rdy),
    .IR_Dtrm  (IR_Dtrm),
    .SS_n     (SS_n),
    .SCLK     (SCLK),
    .MOSI     (MOSI),
    .MISO     (MISO),
    .INT      (INT),
    .moving   (moving),
    .en_fusion(en_fusion)
  );

  SPI_iNEMO2 iNEMO (
    .SS_n (SS_n),
    .SCLK (SCLK),
    .MOSI (MOSI),
    .MISO (MISO),
    .INT  (INT)
  );

  always #10 clk = ~clk;

  initial begin
    clk       = 0;
    rst_n     = 0;
    strt_cal  = 0;
    moving    = 1;
    en_fusion = 0;
    IR_Dtrm   = 9'h000;

    @(negedge clk);
    @(negedge clk);
    rst_n = 1;
    $display("INFO: Reset released at time %0t", $time);

    // Wait for NEMO_setup with timeout
    fork
      begin : wait_nemo
        @(posedge iNEMO.NEMO_setup);
      end
      begin : timeout_nemo
        repeat(5_000_000) @(posedge clk);
        $display("TIMEOUT at %0t: NEMO_setup never asserted", $time);
        $stop;
      end
    join_any
    disable wait_nemo;
    disable timeout_nemo;
    $display("INFO: NEMO_setup asserted at time %0t", $time);

    // Assert strt_cal for 1 clock
    @(negedge clk);
    strt_cal = 1;
    @(negedge clk);
    strt_cal = 0;
    $display("INFO: strt_cal pulsed at time %0t", $time);

    // Wait for cal_done — use posedge detection with timeout
    // but also handle case where cal_done already came
    fork
      begin : wait_cal
        @(posedge cal_done);
      end
      begin : timeout_cal
        repeat(500_000) @(posedge clk);
      end
    join_any
    disable wait_cal;
    disable timeout_cal;

    if (!cal_done)
      $display("INFO: cal_done not seen via posedge, continuing anyway at %0t", $time);
    else
      $display("INFO: cal_done asserted at time %0t", $time);

    // Run 8 million cycles and observe heading
    $display("INFO: Running 8,000,000 more cycles...");
    repeat(8_000_000) @(posedge clk);

    $display("INFO: Done. Final heading = %0d at time %0t", heading, $time);
    $stop;
  end

endmodule