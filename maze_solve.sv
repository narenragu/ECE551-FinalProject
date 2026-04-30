module maze_solve(
    input clk, rst_n,
    input cmd_md,
    input cmd0,
    input lft_opn, rght_opn,
    input mv_cmplt,
    input sol_cmplt,
    output logic strt_hdng,
    output logic [11:0] dsrd_hdng,
    output logic strt_mv,
    output logic stp_lft, stp_rght
);

    function automatic logic [11:0] turn_right(input logic [11:0] h);
        case (h)
            12'h000: turn_right = 12'hC00;
            12'hC00: turn_right = 12'h7FF;
            12'h7FF: turn_right = 12'h3FF;
            12'h3FF: turn_right = 12'h000;
            default: turn_right = 12'h000;
        endcase
    endfunction

    function automatic logic [11:0] turn_left(input logic [11:0] h);
        case (h)
            12'h000: turn_left = 12'h3FF;
            12'h3FF: turn_left = 12'h7FF;
            12'h7FF: turn_left = 12'hC00;
            12'hC00: turn_left = 12'h000;
            default: turn_left = 12'h000;
        endcase
    endfunction

    function automatic logic [11:0] turn_around(input logic [11:0] h);
        case (h)
            12'h000: turn_around = 12'h7FF;
            12'h7FF: turn_around = 12'h000;
            12'h3FF: turn_around = 12'hC00;
            12'hC00: turn_around = 12'h3FF;
            default: turn_around = 12'h000;
        endcase
    endfunction

    typedef enum logic [2:0] {
        IDLE,
        START_MOVE,
        WAIT_MOVE,
        START_TURN,
        WAIT_TURN,
        DONE
    } state_t;

    state_t state, next_state;

    logic [11:0] new_heading;
    logic load_hdng;

    always_ff @(posedge clk or negedge rst_n) begin
        if (!rst_n)
            state <= IDLE;
        else
            state <= next_state;
    end

    always_ff @(posedge clk or negedge rst_n) begin
        if (!rst_n)
            dsrd_hdng <= 12'h000;
        else if (load_hdng)
            dsrd_hdng <= new_heading;
    end

    assign stp_lft  = cmd0;
    assign stp_rght = ~cmd0;

    always_comb begin
        next_state = state;
        new_heading = dsrd_hdng;
        load_hdng = 1'b0;
        strt_mv = 1'b0;
        strt_hdng = 1'b0;

        case (state)
            IDLE: begin
                if (!cmd_md)
                    next_state = START_MOVE;
            end

            START_MOVE: begin
                strt_mv = 1'b1;
                next_state = WAIT_MOVE;
            end

            WAIT_MOVE: begin
                if (sol_cmplt)
                    next_state = DONE;
                else if (mv_cmplt)
                    next_state = START_TURN;
            end

            START_TURN: begin
                if (cmd0) begin
                    if (lft_opn)
                        new_heading = turn_left(dsrd_hdng);
                    else if (rght_opn)
                        new_heading = turn_right(dsrd_hdng);
                    else
                        new_heading = turn_around(dsrd_hdng);
                end else begin
                    if (rght_opn)
                        new_heading = turn_right(dsrd_hdng);
                    else if (lft_opn)
                        new_heading = turn_left(dsrd_hdng);
                    else
                        new_heading = turn_around(dsrd_hdng);
                end
                load_hdng = 1'b1;
                strt_hdng = 1'b1;
                next_state = WAIT_TURN;
            end

            WAIT_TURN: begin
                if (sol_cmplt)
                    next_state = DONE;
                else if (mv_cmplt)
                    next_state = START_MOVE;
            end

            DONE: begin
                next_state = DONE;
            end

            default: next_state = IDLE;
        endcase
    end

endmodule