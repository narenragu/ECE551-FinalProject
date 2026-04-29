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
    IDLE, MOVE, TURN, WAIT_TURN, DONE
} state_t;

state_t state, next_state;

logic [11:0] new_heading;
logic load_hdng;

always_ff @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin
        dsrd_hdng <= 12'h000;
    end
    else if (load_hdng) begin
        dsrd_hdng <= new_heading;
    end
end

always_ff @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin 
        state <= IDLE;
    end
    else begin
        state <= next_state;
    end
end

assign stp_lft  = cmd0;
assign stp_rght = ~cmd0;

always_comb begin
    next_state = state;
    strt_hdng = 1'b0;
    strt_mv = 1'b0;
    load_hdng = 1'b0;
    new_heading = dsrd_hdng;

    case (state)
        IDLE: begin
            if (!cmd_md) begin
                strt_mv = 1'b1;
                next_state = MOVE;
            end
        end
        MOVE: begin
            if (mv_cmplt) begin
                if (sol_cmplt) next_state = DONE;
                else next_state = TURN;
            end
        end
        TURN: begin
            strt_hdng = 1'b1;
            load_hdng = 1'b1;
            if (cmd0) begin // left affinity
                if (lft_opn) new_heading = turn_left(dsrd_hdng);
                else if (rght_opn) new_heading = turn_right(dsrd_hdng);
                else new_heading = turn_right(turn_right(dsrd_hdng)); // 180
            end else begin // right affinity
                if (rght_opn) new_heading = turn_right(dsrd_hdng);
                else if (lft_opn) new_heading = turn_left(dsrd_hdng);
                else new_heading = turn_right(turn_right(dsrd_hdng)); // 180
            end
            next_state = WAIT_TURN;
        end
        WAIT_TURN: begin
            if (mv_cmplt) begin
                strt_mv = 1'b1;
                next_state = MOVE;
            end
        end
        DONE: begin            
        end

        default: next_state = IDLE;
    endcase
end

endmodule