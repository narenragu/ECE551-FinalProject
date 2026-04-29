module UART_tx (
    input logic clk,
    input logic rst_n,
    input logic trmt,
    input logic [7:0] tx_data,
    output logic TX,
    output logic tx_done
);
    localparam BAUD_DIV = 2604;

    typedef enum logic {
        IDLE,
        TRANSMIT
    } state_t;

    state_t state, nxt_state;

    logic [11:0] baud_cnt;
    logic baud_tick;

    logic [3:0] bit_cnt;
    logic [7:0] data_reg;

    always_ff @(posedge clk or negedge rst_n) begin
        if (!rst_n)
            baud_cnt <= 0;
        else if (state == TRANSMIT) begin
            if (baud_cnt == BAUD_DIV-1)
                baud_cnt <= 0;
            else
                baud_cnt <= baud_cnt + 1;
        end
        else
            baud_cnt <= 0;
    end

    assign baud_tick = (baud_cnt == BAUD_DIV-1);

    always_ff @(posedge clk or negedge rst_n) begin
        if (!rst_n)
            state <= IDLE;
        else
            state <= nxt_state;
    end

    always_comb begin
        nxt_state = state;

        case (state)
            IDLE:
                if (trmt)
                    nxt_state = TRANSMIT;

            TRANSMIT:
                if (baud_tick && bit_cnt == 4'd9)
                    nxt_state = IDLE;
            default:
                nxt_state = IDLE;
        endcase
    end

    always_ff @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            bit_cnt  <= 0;
            data_reg <= 0;
        end
        else begin
            case (state)
                IDLE: begin
                    bit_cnt <= 0;
                    if (trmt)
                        data_reg <= tx_data;
                end
                TRANSMIT: begin
                    if (baud_tick)
                        bit_cnt <= bit_cnt + 1;
                end
                default: begin
                    bit_cnt <= 0;
                    data_reg <= 0;
                end
            endcase
        end
    end

    always_comb begin
        if (state == IDLE)
            TX = 1'b1;
        else begin
            unique case (bit_cnt)
                4'd0: TX = 1'b0; // start
                4'd1: TX = data_reg[0];
                4'd2: TX = data_reg[1];
                4'd3: TX = data_reg[2];
                4'd4: TX = data_reg[3];
                4'd5: TX = data_reg[4];
                4'd6: TX = data_reg[5];
                4'd7: TX = data_reg[6];
                4'd8: TX = data_reg[7];
                4'd9: TX = 1'b1; // stop
                default: TX = 1'b1;
            endcase
        end
    end

    always_ff @(posedge clk or negedge rst_n) begin
        if (!rst_n)
            tx_done <= 1'b1;
        else begin
            if (trmt)
                tx_done <= 1'b0;
            else if (state == TRANSMIT && baud_tick && bit_cnt == 4'd9)
                tx_done <= 1'b1;
        end
    end

endmodule