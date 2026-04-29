module IR_math
#(parameter NOM_IR = 12'h900)
(
    input lft_opn, rght_opn,
    input [11:0] lft_IR, rght_IR,
    input signed [8:0] IR_Dtrm,
    input en_fusion,
    input signed [11:0] dsrd_hdng,
    output [11:0] dsrd_hdng_adj
);

logic signed [12:0] IR_diff;
logic signed [12:0] IR_left_nom_diff;
logic signed [12:0] IR_nom_right_diff;

logic signed [11:0] left_ir_mux_out;
logic signed [12:0] second_stage_mux;
logic signed [12:0] third_stage_mux;

logic signed [12:0] div_32_ext_13;

logic signed [12:0] IR_Dtrm_ext_13_mult_4;

logic signed [12:0] third_mux_sum_IR_ext;

logic signed [11:0] out_divide_2;

assign IR_diff = lft_IR - rght_IR;
assign IR_left_nom_diff = lft_IR - NOM_IR;
assign IR_nom_right_diff = NOM_IR - rght_IR;

assign left_ir_mux_out = (rght_opn) ? IR_left_nom_diff : IR_diff >>> 1;
assign second_stage_mux = (lft_opn) ? IR_nom_right_diff : left_ir_mux_out;
assign third_stage_mux = (lft_opn && rght_opn) ? 12'h0000 : second_stage_mux;

// divide third_stage_mux by 32 (shift right by 5) and extend to 13 bits to preserve sign
assign div_32_ext_13 = third_stage_mux >>> 5; // asr 5

// multiply IR_Dtrm by 4 (shift left by 2) and extend to 13 bits to preserve sign
assign IR_Dtrm_ext_13_mult_4 = IR_Dtrm <<< 2; // asl 2

assign third_mux_sum_IR_ext = div_32_ext_13 + IR_Dtrm_ext_13_mult_4;

// divide by 2 (shift right by 1) and extend to 12 bits to preserve sign
assign out_divide_2 = third_mux_sum_IR_ext >>> 1; // asr 1

assign dsrd_hdng_adj = (en_fusion) ? dsrd_hdng + out_divide_2 : dsrd_hdng;

endmodule