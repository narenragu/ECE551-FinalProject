module RemoteComm(
    input logic clk,
    input logic rst_n,
    input logic [15:0] cmd,
    input logic snd_cmd,
    input logic RX,
    output logic cmd_snt,
    output logic [7:0]  resp,
    output logic resp_rdy,
    output logic TX
);

    logic sel_high;
    logic trmt;
    logic tx_done;
    logic set_cmd_snt;
    logic [7:0] tx_data;
    logic [7:0] low_byte;

    localparam BAUD_DIV = 2604;
    logic [11:0] gap_cnt;
    logic gap_done;

    typedef enum logic [1:0] { 
        IDLE, 
        WAIT_HIGH_DONE,
        GAP,
        WAIT_LOW_DONE 
    } state_t;
    state_t state, next_state;

    UART iUART (
        .clk(clk),
        .rst_n(rst_n),
        .RX(RX),
        .TX(TX),
        .rx_rdy(resp_rdy),
        .clr_rx_rdy(1'b0),
        .rx_data(resp),
        .trmt(trmt),
        .tx_data(tx_data),
        .tx_done(tx_done)
    );

    assign tx_data = sel_high ? cmd[15:8] : low_byte;
    assign gap_done = (gap_cnt == BAUD_DIV - 1);

    // low byte reg
    always_ff @(posedge clk or negedge rst_n) begin
        if (!rst_n)
            low_byte <= 8'h00;
        else if (snd_cmd)
            low_byte <= cmd[7:0];
    end

    // gap counter — runs only in GAP state
    always_ff @(posedge clk or negedge rst_n) begin
        if (!rst_n)
            gap_cnt <= '0;
        else if (state == GAP)
            gap_cnt <= gap_cnt + 1;
        else
            gap_cnt <= '0;
    end

    // cmd_snt SR 
    always_ff @(posedge clk or negedge rst_n) begin
        if (!rst_n)
            cmd_snt <= 1'b0;
        else if (snd_cmd)
            cmd_snt <= 1'b0;
        else if (set_cmd_snt)
            cmd_snt <= 1'b1;
    end

    // FSM state reg
    always_ff @(posedge clk or negedge rst_n) begin
        if (!rst_n)
            state <= IDLE;
        else
            state <= next_state;
    end

    always_comb begin
        next_state = state;
        sel_high = 1'b1;
        trmt = 1'b0;
        set_cmd_snt = 1'b0;

        case (state)
            IDLE: begin
                sel_high = 1'b1;
                if (snd_cmd) begin
                    trmt = 1'b1;
                    next_state = WAIT_HIGH_DONE;
                end
            end

            WAIT_HIGH_DONE: begin
                sel_high = 1'b0;
                if (tx_done)
                    next_state = GAP;
            end

            GAP: begin
                sel_high = 1'b0;
                if (gap_done) begin
                    trmt = 1'b1;
                    next_state = WAIT_LOW_DONE;
                end
            end

            WAIT_LOW_DONE: begin
                sel_high = 1'b0;
                if (tx_done) begin
                    set_cmd_snt = 1'b1;
                    next_state = IDLE;
                end
            end

            default: next_state = IDLE;
        endcase
    end

endmodule