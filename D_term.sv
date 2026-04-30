module D_term(
    input clk, rst_n,
    input signed [9:0] err_sat,
    input hdng_vld,
    output signed [12:0] D_term
);

    localparam D_COEFF = 5'h0E;
    logic signed [9:0] prev_err;
    logic signed [9:0] ff_out1, ff_out2;
    logic signed [10:0] D_diff;
    logic signed [7:0] D_diff_sat;

    always_ff @ (posedge clk or negedge rst_n) begin
        if(!rst_n) begin
            ff_out1 <= 10'b0;
        end else if(hdng_vld) begin
            ff_out1 <= err_sat;
        end else begin
            ff_out1 <= ff_out1;
        end
    end

    always_ff @ (posedge clk or negedge rst_n) begin
        if(!rst_n) begin
            ff_out2 <= 10'b0;
        end else if(hdng_vld) begin
            ff_out2 <= ff_out1;
        end else begin
            ff_out2 <= ff_out2;
        end
    end

    assign prev_err = ff_out2;
    assign D_diff = err_sat - prev_err;
    //saturate to 8 bits
    assign D_diff_sat = (D_diff[10:8] == {3{D_diff[7]}}) ? D_diff[7:0] :
        ((D_diff[10]) ? 8'h80 : 8'h7F);
    //signed multiply
    assign D_term = $signed(D_COEFF) * D_diff_sat;

endmodule