module maze_solve(
    input logic clk, rst_n,
    input logic cmd_md,
    input logic cmd0,
    input logic lft_opn, rght_opn,
    input logic mv_cmplt,
    input logic sol_cmplt,
    output logic strt_hdng,
    output logic [11:0] dsrd_hdng,
    output logic strt_mv,
    output logic stp_lft, stp_rght
);

function automatic logic [11:0] turn_right(input logic [11:0] h);
    case(h)
        12'h000: return 12'hC00;
        12'hC00: return 12'h7FF;
        12'h7FF: return 12'h3FF;
        12'h3FF: return 12'h000;
        default: return 12'h000;
    endcase
endfunction

function automatic logic [11:0] turn_left(input logic [11:0] h);
    case(h)
        12'h000: return 12'h3FF;
        12'h3FF: return 12'h7FF;
        12'h7FF: return 12'hC00;
        12'hC00: return 12'h000;
        default: return 12'h000;
    endcase
endfunction

typedef enum logic [2:0] {
    IDLE, MOVE, LFT_TURN, RGHT_TURN, ROTATE_180, DONE
} state_t;

state_t state, next_state;

logic [11:0] new_heading;
logic load_hdng;

// dsrd_hdng register — resets to north (12'h000)
always_ff @(posedge clk or negedge rst_n) begin
    if (!rst_n)
        dsrd_hdng <= 12'h000;
    else if (load_hdng)
        dsrd_hdng <= new_heading;
end

// State register
always_ff @(posedge clk or negedge rst_n) begin
    if (!rst_n)
        state <= IDLE;
    else
        state <= next_state;
end

// stp_lft/stp_rght: purely combinational from cmd0
assign stp_lft  = cmd0;
assign stp_rght = ~cmd0;

// Next-state and output logic
always_comb begin
    next_state = state;
    strt_hdng  = 1'b0;
    strt_mv    = 1'b0;
    load_hdng  = 1'b0;
    new_heading = dsrd_hdng;

    case (state)

        // ----------------------------------------------------------------
        // Wait for cmd_md to go low to begin solving
        // ----------------------------------------------------------------
        IDLE: begin
            if (!cmd_md) begin
                strt_mv    = 1'b1;
                next_state = MOVE;
            end
        end

        // ----------------------------------------------------------------
        // Move forward until mv_cmplt (stopped at opening or front wall).
        // sol_cmplt takes priority — we're done.
        // On mv_cmplt, apply affinity logic to pick next heading.
        // ----------------------------------------------------------------
        MOVE: begin
            if (sol_cmplt) begin
                next_state = DONE;
            end else if (mv_cmplt) begin
                if (cmd0) begin
                    // ---- LEFT AFFINITY ----
                    // Prefer left; if not open try right; else 180
                    if (lft_opn) begin
                        new_heading = turn_left(dsrd_hdng);
                        load_hdng   = 1'b1;
                        strt_hdng   = 1'b1;
                        next_state  = LFT_TURN;
                    end else if (rght_opn) begin
                        new_heading = turn_right(dsrd_hdng);
                        load_hdng   = 1'b1;
                        strt_hdng   = 1'b1;
                        next_state  = RGHT_TURN;
                    end else begin
                        // Neither open — pull a 180
                        new_heading = turn_right(turn_right(dsrd_hdng));
                        load_hdng   = 1'b1;
                        strt_hdng   = 1'b1;
                        next_state  = ROTATE_180;
                    end
                end else begin
                    // ---- RIGHT AFFINITY ----
                    // Prefer right; if not open try left; else 180
                    if (rght_opn) begin
                        new_heading = turn_right(dsrd_hdng);
                        load_hdng   = 1'b1;
                        strt_hdng   = 1'b1;
                        next_state  = RGHT_TURN;
                    end else if (lft_opn) begin
                        new_heading = turn_left(dsrd_hdng);
                        load_hdng   = 1'b1;
                        strt_hdng   = 1'b1;
                        next_state  = LFT_TURN;
                    end else begin
                        new_heading = turn_right(turn_right(dsrd_hdng));
                        load_hdng   = 1'b1;
                        strt_hdng   = 1'b1;
                        next_state  = ROTATE_180;
                    end
                end
            end
        end

        // ----------------------------------------------------------------
        // Wait for the navigate unit to complete the heading change,
        // then immediately kick off the next forward move.
        // ----------------------------------------------------------------
        LFT_TURN: begin
            if (mv_cmplt) begin
                strt_mv    = 1'b1;
                next_state = MOVE;
            end
        end

        RGHT_TURN: begin
            if (mv_cmplt) begin
                strt_mv    = 1'b1;
                next_state = MOVE;
            end
        end

        ROTATE_180: begin
            if (mv_cmplt) begin
                strt_mv    = 1'b1;
                next_state = MOVE;
            end
        end

        // ----------------------------------------------------------------
        // Terminal state — stay here until reset
        // ----------------------------------------------------------------
        DONE: begin
            next_state = DONE;
        end

        default: next_state = IDLE;

    endcase
end

endmodule