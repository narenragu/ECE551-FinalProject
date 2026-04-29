module RemoteComm(
    input clk, rst_n,
    input [15:0] cmd,
    input snd_cmd,
    input RX,
    output [7:0] resp,
    output logic cmd_sent,
    output resp_rdy,
    output TX
);
    logic clr_rx_rdy;
    logic trmt, tx_done;
    logic [7:0] tx_byte, rx_byte;
    //logic sel; //0 for first byte, 1 for second byte
    logic set_cmd_sent;
    logic load_high, load_low;
    
    UART iUART(.clk(clk), .rst_n(rst_n), .RX(RX), .TX(TX), .rx_rdy(resp_rdy), 
                .clr_rx_rdy(clr_rx_rdy), .rx_data(resp), .trmt(trmt), .tx_data(tx_byte), .tx_done(tx_done));

    typedef enum logic [2:0] {IDLE, LOAD1, SEND1, LOAD2, SEND2} state_t;
    state_t state, nxt_state;

    //clr_rx_rdy control
    always_ff @(posedge clk or negedge rst_n) begin
    if (!rst_n)
        clr_rx_rdy <= 1'b0;
    else
        clr_rx_rdy <= resp_rdy; //clear resp_rdy after one cycle to acknowledge receipt of response
end

    //tx_byte control
    always_ff @(posedge clk or negedge rst_n) begin
    if(!rst_n)
        tx_byte <= 8'h00;
    else if(load_high)
        tx_byte <= cmd[15:8];
    else if(load_low)
        tx_byte <= cmd[7:0];
    end

    //cmd_sent control
    always_ff @ (posedge clk, negedge rst_n) begin
        if(!rst_n) begin
            cmd_sent <= 1'b0;
        end else if(set_cmd_sent) begin
            cmd_sent <= 1'b1; 
        end else if(snd_cmd) begin
            cmd_sent <= 1'b0;
        end
    end

    //state transition
    always_ff @ (posedge clk or negedge rst_n) begin
        if(!rst_n) begin
            state <= IDLE;
        end else begin
            state <= nxt_state;
        end
    end

    //output logic and next state logic
    always_comb begin
    trmt = 0;
    set_cmd_sent = 0;
    load_high = 0;
    load_low = 0;
    nxt_state = state;

    case(state)
        IDLE: begin
            if(snd_cmd) begin
                load_high = 1;
                nxt_state = LOAD1;
            end
        end

        LOAD1: begin
            trmt = 1;
            nxt_state = SEND1;
        end

        SEND1: begin
            if(tx_done) begin
                load_low = 1;
                nxt_state = LOAD2;
            end
        end

        LOAD2: begin
            trmt = 1;
            nxt_state = SEND2;
        end

        SEND2: begin
            if(tx_done) begin
                set_cmd_sent = 1;
                nxt_state = IDLE;
            end
        end
    endcase
end

endmodule