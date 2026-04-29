module cmd_proc(
    input logic clk, rst_n,
    input logic [15:0] cmd,
    input logic cmd_rdy,
    output logic clr_cmd_rdy, send_resp,
    output logic strt_cal, in_cal,
    input logic cal_done,
    input logic sol_cmplt,
    output logic strt_hdng, strt_mv, stp_lft, stp_rght,
    output logic [11:0] dsrd_hdng,
    input logic mv_cmplt,
    output logic cmd_md
);
    logic [2:0] opcode;
    logic [11:0] operand;

    //state variables
    typedef enum logic [2:0] {IDLE, CAL, HDNG, MV, SOLVE} state_t;
    state_t state, nxt_state;

    //cmd parsing
    assign opcode = cmd[15:13];
    assign operand = cmd[11:0];

    //dsrd_hdng control
    always_ff @(posedge clk, negedge rst_n) begin
        if(!rst_n) begin
            dsrd_hdng <= 12'h000;
        end else if(strt_hdng) begin
            dsrd_hdng <= operand;
        end
    end

    //stp_lft and stp_rght control
    always_ff @(posedge clk or negedge rst_n) begin
    if(!rst_n) begin
        stp_lft   <= 1'b0;
        stp_rght  <= 1'b0;
    end else if(strt_mv) begin
        stp_lft   <= operand[1];
        stp_rght  <= operand[0];
    end
end

    //SM state transition
    always_ff @(posedge clk, negedge rst_n) begin
        if(!rst_n) begin
            state <= IDLE;
        end else begin
            state <= nxt_state;
        end
     end

    //SM nxt_state and output logic
    always_comb begin
        clr_cmd_rdy = 0;
        send_resp = 0;
        strt_cal = 0;
        in_cal = 0;
        strt_hdng = 0;
        strt_mv = 0;
        cmd_md = 1;
        nxt_state = state;

        case(state) 
            IDLE: begin
                if(cmd_rdy) begin
                    clr_cmd_rdy = 1;
                    case(opcode)
                        3'b000: begin //calibrate cmd
                            strt_cal = 1;
                            nxt_state = CAL;
                        end
                        3'b001: begin //heading cmd
                            strt_hdng = 1;
                            nxt_state = HDNG;
                        end
                        3'b010: begin //move cmd
                            strt_mv = 1;
                            nxt_state = MV;
                        end
                        3'b011: begin //solve cmd
                            cmd_md = 0;
                            nxt_state = SOLVE;                            
                        end
                        default: begin
                            //nuffin
                        end
                    endcase
                end
            end
            CAL: begin
                in_cal = 1;
                if(cal_done) begin
                    send_resp = 1;
                    nxt_state = IDLE;
                end
            end
            HDNG: begin
                if(mv_cmplt) begin
                    send_resp = 1;
                    nxt_state = IDLE;
                end
            end
            MV: begin
                if(mv_cmplt) begin
                    send_resp = 1;
                    nxt_state = IDLE;
                end
            end
            SOLVE: begin
                cmd_md = 0;
                if(sol_cmplt) begin
                    send_resp = 1;
                    nxt_state = IDLE;
                end
            end
        endcase
    end

endmodule