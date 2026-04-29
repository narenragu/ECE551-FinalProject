module PWM12 (
        input wire clk,
        input wire rst_n,
        input wire [11:0] duty,
        output reg PWM1,
        output reg PWM2
);
    localparam [11:0] NONOVERLAP = 12'h02C;

    reg [11:0] cnt;

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n)
            cnt <= 12'd0;
        else
            cnt <= cnt + 12'd1;
    end

    // PWM1
    always_ff @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            PWM1 <= 0;
        end
        else if(cnt >= duty) begin
            PWM1 <= 0;
        end
        else if(cnt >= NONOVERLAP) begin
            PWM1 <= 1;
        end
        
    end

    // PWM2
    always_ff @(posedge clk or negedge rst_n) begin
        if (!rst_n)
            PWM2 <= 0;
        else if(cnt >= (duty + NONOVERLAP))
            PWM2 <= 1;
        else
            PWM2 <= 0;
end
endmodule