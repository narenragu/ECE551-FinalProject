module I_term(
    input clk, rst_n,
    input hdng_vld,         //new error signal is valid and should be accumulated 
    input moving,           
    input [9:0] err_sat,
    output signed [11:0] I_term
);

    logic signed [15:0] err_sat_ext;
    logic signed [15:0] accum_out;
    logic signed [15:0] integrator, nxt_integrator;
    logic overflow;

    //accumulator register 
    always_ff @ (posedge clk or negedge rst_n) begin
        if(!rst_n) begin
            integrator <= 16'h0000;
        end else begin
            integrator <= nxt_integrator;
        end
    end

    //sign extend err_sat to 16 bits
    assign err_sat_ext = {{6{err_sat[9]}}, err_sat}; 
    //accumulator logic
    assign accum_out = integrator + err_sat_ext;
    //overflow logic
    assign overflow = (err_sat_ext[15] == integrator[15]) && (accum_out[15] != integrator[15]);

    //2 muxes controlling nxt_integrator
    assign nxt_integrator = moving ? 
        ((hdng_vld && !overflow) ? accum_out : integrator) :
        16'h0000;

    assign I_term = integrator[15:4];

endmodule