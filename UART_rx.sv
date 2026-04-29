module UART_rx (
    input logic clk,
    input logic rst_n,
    input logic RX,
    input logic clr_rdy,
    output logic [7:0] rx_data,
    output logic rdy
);

    localparam int BAUD_DIV = 2604;
    localparam int HALF_BAUD_DIV = 1302;

    // metastability synchronizer
    logic rx_sync1, rx_sync2;

    always_ff @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            rx_sync1 <= 1'b1;
            rx_sync2 <= 1'b1;
        end else begin
            rx_sync1 <= RX;
            rx_sync2 <= rx_sync1;
        end
    end

    // falling edge detector
    logic rx_prev;
    always_ff @(posedge clk or negedge rst_n) begin
        if (!rst_n)
            rx_prev <= 1'b1;
        else
            rx_prev <= rx_sync2;
    end

    wire start_edge = (rx_prev == 1'b1) && (rx_sync2 == 1'b0);

    logic [11:0] baud_cnt;
    logic [3:0] bit_cnt;
    logic [7:0] shift_reg;

    logic clr_baud, inc_baud;
    logic clr_bit, inc_bit;
    logic shift_en;
    logic set_rdy, clr_rdy_int;

    wire baud_done = (baud_cnt == BAUD_DIV-1);
    wire half_baud_done = (baud_cnt == HALF_BAUD_DIV-1);
    wire bit_done = (bit_cnt == 4'd9);

    // baud counter
    always_ff @(posedge clk or negedge rst_n) begin
        if (!rst_n)
            baud_cnt <= 0;
        else if (clr_baud)
            baud_cnt <= 0;
        else if (inc_baud)
            baud_cnt <= baud_cnt + 1;
    end

    // bit coutner
    always_ff @(posedge clk or negedge rst_n) begin
        if (!rst_n)
            bit_cnt <= 0;
        else if (clr_bit)
            bit_cnt <= 0;
        else if (inc_bit)
            bit_cnt <= bit_cnt + 1;
    end

    // shift reg
    always_ff @(posedge clk or negedge rst_n) begin
        if (!rst_n)
            shift_reg <= 8'd0;
        else if (shift_en)
            shift_reg <= {rx_sync2, shift_reg[7:1]};
    end

    assign rx_data = shift_reg;

    // RX FSM
    typedef enum logic [1:0] {
        IDLE,
        START,
        RECEIVING
    } state_t;

    state_t state, next_state;

    always_comb begin
        clr_baud = 0;
        inc_baud = 0;
        clr_bit = 0;
        inc_bit = 0;
        shift_en = 0;
        set_rdy = 0;
        clr_rdy_int = 0;
        next_state = state;

        case (state)
            IDLE: begin
                clr_bit = 1;
                clr_baud = 1;

                if (start_edge) begin
                    clr_rdy_int = 1;  // clear rdy on new start
                    next_state = START;
                end
            end
            START: begin
                inc_baud = 1;

                if (half_baud_done) begin
                    clr_baud = 1;

                    if (rx_sync2 == 1'b0) begin
                        next_state = RECEIVING;
                        clr_bit = 1;
                    end else begin
                        next_state = IDLE;   // false start?
                    end
                end
            end
            RECEIVING: begin
                inc_baud = 1;

                if (baud_done) begin
                    clr_baud = 1;
                    inc_bit = 1;

                    // shift only during data bits ( 1 – 8 )
                    if (bit_cnt >= 1 && bit_cnt <= 8)
                        shift_en = 1;

                    if (bit_done) begin
                        set_rdy = 1;
                        next_state = IDLE;
                    end
                end
            end
            default: begin
                next_state = IDLE;
            end
        endcase
    end

    // FSM state reg
    always_ff @(posedge clk or negedge rst_n) begin
        if (!rst_n)
            state <= IDLE;
        else
            state <= next_state;
    end

    // SR latch for rdy signal
    always_ff @(posedge clk or negedge rst_n) begin
        if (!rst_n)
            rdy <= 1'b0;
        else if (clr_rdy || clr_rdy_int)
            rdy <= 1'b0;
        else if (set_rdy)
            rdy <= 1'b1;
    end

endmodule