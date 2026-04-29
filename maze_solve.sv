module maze_solve(
    input  logic        clk,
    input  logic        rst_n,
    input  logic        cmd_md,
    input  logic        cmd0,
    input  logic        lft_opn,
    input  logic        rght_opn,
    input  logic        mv_cmplt,
    input  logic        sol_cmplt,

    output logic        strt_hdng,
    output logic [11:0] dsrd_hdng,
    output logic        strt_mv,
    output logic        stp_lft,
    output logic        stp_rght
);

    function automatic logic [11:0] turn_right(input logic [11:0] h);
        case (h)
            12'h000: turn_right = 12'h3FF;
            12'h3FF: turn_right = 12'h7FF;
            12'h7FF: turn_right = 12'hC00;
            12'hC00: turn_right = 12'h000;
            default: turn_right = 12'h000;
        endcase
    endfunction

    function automatic logic [11:0] turn_left(input logic [11:0] h);
        case (h)
            12'h000: turn_left = 12'hC00;
            12'hC00: turn_left = 12'h7FF;
            12'h7FF: turn_left = 12'h3FF;
            12'h3FF: turn_left = 12'h000;
            default: turn_left = 12'h000;
        endcase
    endfunction

    typedef enum logic [3:0] {
        IDLE,
        START_MOVE,
        MOVE,
        CHOOSE_TURN,
        WAIT_MV_LOW_BEFORE_TURN,
        START_TURN,
        TURNING,
        WAIT_MV_LOW_BEFORE_MOVE,
        DONE
    } state_t;

    state_t state, nxt_state;

    logic [11:0] new_heading;
    logic        load_hdng;

    assign stp_lft  =  cmd0;
    assign stp_rght = ~cmd0;

    always_ff @(posedge clk or negedge rst_n) begin
        if (!rst_n)
            state <= IDLE;
        else
            state <= nxt_state;
    end

    always_ff @(posedge clk or negedge rst_n) begin
        if (!rst_n)
            dsrd_hdng <= 12'h000;
        else if (load_hdng)
            dsrd_hdng <= new_heading;
    end

    always_comb begin
        nxt_state   = state;
        strt_hdng   = 1'b0;
        strt_mv     = 1'b0;
        load_hdng   = 1'b0;
        new_heading = dsrd_hdng;

        case (state)

            IDLE: begin
                if (!cmd_md)
                    nxt_state = START_MOVE;
            end

            START_MOVE: begin
                if (!mv_cmplt) begin
                    strt_mv   = 1'b1;
                    nxt_state = MOVE;
                end
            end

            MOVE: begin
                if (sol_cmplt) begin
                    nxt_state = DONE;
                end else if (mv_cmplt) begin
                    nxt_state = CHOOSE_TURN;
                end
            end

            CHOOSE_TURN: begin
                if (cmd0) begin
                    if (lft_opn)
                        new_heading = turn_left(dsrd_hdng);
                    else if (rght_opn)
                        new_heading = turn_right(dsrd_hdng);
                    else
                        new_heading = turn_right(turn_right(dsrd_hdng));
                end else begin
                    if (rght_opn)
                        new_heading = turn_right(dsrd_hdng);
                    else if (lft_opn)
                        new_heading = turn_left(dsrd_hdng);
                    else
                        new_heading = turn_right(turn_right(dsrd_hdng));
                end

                load_hdng = 1'b1;
                nxt_state = WAIT_MV_LOW_BEFORE_TURN;
            end

            WAIT_MV_LOW_BEFORE_TURN: begin
                if (!mv_cmplt)
                    nxt_state = START_TURN;
            end

            START_TURN: begin
                strt_hdng = 1'b1;
                nxt_state = TURNING;
            end

            TURNING: begin
                if (mv_cmplt)
                    nxt_state = WAIT_MV_LOW_BEFORE_MOVE;
            end

            WAIT_MV_LOW_BEFORE_MOVE: begin
                if (!mv_cmplt)
                    nxt_state = START_MOVE;
            end

            DONE: begin
                if (cmd_md)
                    nxt_state = IDLE;
            end

            default: begin
                nxt_state = IDLE;
            end

        endcase
    end

endmodule