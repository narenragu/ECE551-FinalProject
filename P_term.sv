module P_term(
    input [11:0] error,
    output [13:0] P_term
);

    localparam signed P_coeff = 4'h3;

    logic signed [9:0] err_sat;

    assign err_sat = error[11] ? 
        (&error[10:9] ? error[9:0] : {1'b1, 9'b0}) :      //neg case
        (~|error[10:9] ? error[9:0] : 10'sb0111111111);     //pos case

    assign P_term = P_coeff * err_sat;

endmodule