module PWM12(
    input clk, rst_n,
    input [11:0] duty,
    output reg PWM1, PWM2
);

    localparam [11:0] NONOVERLAP = 12'h02C;

    logic set1, set2, rst1, rst2;
    logic [11:0] cnt;

    //counter flop
    always_ff @ (posedge clk or negedge rst_n) begin
        if(!rst_n) begin
            cnt <= 0;
        end else begin
            cnt <= cnt + 1;
        end
    end

    //PWM1 flop
    always_ff @ (posedge clk or negedge rst_n) begin
        if(!rst_n) begin
            PWM1 <= 0;
        end else if(rst1) begin
            PWM1 <= 0;
        end else if(set1) begin
            PWM1 <= 1;
        end else begin
            PWM1 <= PWM1;
        end
    end

    //PWM2 flop
    always_ff @ (posedge clk or negedge rst_n) begin
        if(!rst_n) begin
            PWM2 <= 0;
        end else if(rst2) begin
            PWM2 <= 0;
        end else if(set2) begin
            PWM2 <= 1;
        end else begin
            PWM2 <= PWM2;
        end
    end

    assign set1 = cnt >= NONOVERLAP;
    assign rst1 = cnt >= duty;
    assign set2 = cnt >= (duty + NONOVERLAP);
    assign rst2 = &cnt;

endmodule