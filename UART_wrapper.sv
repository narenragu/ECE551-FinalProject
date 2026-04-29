module UART_wrapper(
    input logic clk,
    input logic rst_n,
    input logic RX,
    input logic clr_cmd_rdy,
    input logic trmt,
    input logic [7:0] resp,
    output logic cmd_rdy,
    output logic [15:0] cmd,
    output logic tx_done,
    output logic TX
);

    logic rx_rdy;
    logic [7:0] rx_data;
    logic clr_rx_rdy;

    logic [7:0] high_byte;

    typedef enum logic { 
        WAIT_HIGH, 
        WAIT_LOW 
    } state_t;

    state_t state, next_state;

    UART iUART (
        .clk(clk),
        .rst_n(rst_n),
        .RX(RX),
        .TX(TX),
        .rx_rdy(rx_rdy),
        .clr_rx_rdy(clr_rx_rdy),
        .rx_data(rx_data),
        .trmt(trmt),
        .tx_data(resp),
        .tx_done(tx_done)
    );

    // FSM state reg
    always_ff @(posedge clk or negedge rst_n) begin
        if (!rst_n)
            state <= WAIT_HIGH;
        else
            state <= next_state;
    end

    // high byte reg
    always_ff @(posedge clk or negedge rst_n) begin
        if (!rst_n)
            high_byte <= 8'h00;
        else if (rx_rdy && state == WAIT_HIGH)
            high_byte <= rx_data;
    end

    // cmd register
    always_ff @(posedge clk or negedge rst_n) begin
        if (!rst_n)
            cmd <= 16'h0000;
        else if (rx_rdy && state == WAIT_LOW)
            cmd <= {high_byte, rx_data};
    end

    // cmd_rdy register
    always_ff @(posedge clk or negedge rst_n) begin
        if (!rst_n)
            cmd_rdy <= 1'b0;
        else if (rx_rdy && state == WAIT_LOW)
            cmd_rdy <= 1'b1;
        else if (clr_cmd_rdy)
            cmd_rdy <= 1'b0;
    end

    always_comb begin
        next_state = state;
        clr_rx_rdy = 1'b0;

        case (state)
            WAIT_HIGH: begin
                if (rx_rdy) begin
                    clr_rx_rdy = 1'b1; // clear UART rx_rdy
                    next_state = WAIT_LOW;
                end
            end

            WAIT_LOW: begin
                if (rx_rdy) begin
                    clr_rx_rdy = 1'b1; // clear UART rx_rdy
                    next_state = WAIT_HIGH;
                end
            end

            default: next_state = WAIT_HIGH;
        endcase
    end

endmodule