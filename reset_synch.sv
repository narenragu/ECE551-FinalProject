
module reset_synch(
    input RST_n,
    input clk,
    output logic rst_n
);
    logic rst_n1;

    always_ff @(negedge clk, negedge RST_n) begin
        if (!RST_n) begin
            rst_n1 <= 1'b0;
            rst_n      <= 1'b0;
        end else begin
            rst_n1 <= 1'b1;
            rst_n      <= rst_n1;
        end
    end

endmodule