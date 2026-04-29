module piezo_drv #(parameter FAST_SIM = 0)(
    input clk, rst_n,
    input batt_low,
    input fanfare,
    output logic piezo, piezo_n
);

    typedef enum logic [1:0] {G6, C7, E7, G7} notes_t;
    notes_t note;

    logic [15:0] freq_cnt, freq_div;
    logic [24:0] dur_cnt, dur_lim;
    logic [2:0]  note_idx;

    typedef enum logic [1:0] {IDLE, LOW_BATT, FANFARE} state_t;
    state_t state, nxt_state;

    logic [4:0] dur_inc;

    // FAST_SIM duration increment
    generate
        if (FAST_SIM) begin
            always_comb dur_inc = 5'd16;
        end else begin
            always_comb dur_inc = 5'd1;
        end
    endgenerate

    // frequency assignment
    // f = fclk / (2 * freq_div)
    always_comb begin
        case (note)
            G6:      freq_div = 16'd15944;
            C7:      freq_div = 16'd11945;
            E7:      freq_div = 16'd9480;
            G7:      freq_div = 16'd7972;
            default: freq_div = 16'd15944;
        endcase
    end

    // duration assignment
    always_comb begin
        case (state)
            LOW_BATT: begin
                dur_lim = 25'd8388608;   // 2^23
            end

            FANFARE: begin
                case (note_idx)
                    3'd0:    dur_lim = 25'd8388608;   // G6 = 2^23
                    3'd1:    dur_lim = 25'd8388608;   // C7 = 2^23
                    3'd2:    dur_lim = 25'd8388608;   // E7 = 2^23
                    3'd3:    dur_lim = 25'd12582912;  // G7 = 2^23 + 2^22
                    3'd4:    dur_lim = 25'd4194304;   // E7 = 2^22
                    3'd5:    dur_lim = 25'd16777216;  // G7 = 2^24
                    default: dur_lim = 25'd8388608;
                endcase
            end

            default: begin
                dur_lim = 25'd0;
            end
        endcase
    end

    // note assignment
    always_comb begin
        case (state)
            LOW_BATT: begin
                case (note_idx)
                    3'd0:    note = G6;
                    3'd1:    note = C7;
                    3'd2:    note = E7;
                    default: note = G6;
                endcase
            end

            FANFARE: begin
                case (note_idx)
                    3'd0:    note = G6;
                    3'd1:    note = C7;
                    3'd2:    note = E7;
                    3'd3:    note = G7;
                    3'd4:    note = E7;
                    3'd5:    note = G7;
                    default: note = G6;
                endcase
            end

            default: begin
                note = G6;
            end
        endcase
    end

    // frequency counter
    always_ff @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            freq_cnt <= 16'd0;
        end else if (state == IDLE) begin
            freq_cnt <= 16'd0;
        end else if (freq_cnt >= freq_div - 1) begin
            freq_cnt <= 16'd0;
        end else begin
            freq_cnt <= freq_cnt + 16'd1;
        end
    end

    // duration counter
    always_ff @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            dur_cnt <= 25'd0;
        end else if (state == IDLE) begin
            dur_cnt <= 25'd0;
        end else if (dur_cnt + dur_inc >= dur_lim) begin
            dur_cnt <= 25'd0;
        end else begin
            dur_cnt <= dur_cnt + dur_inc;
        end
    end

    // note index update
    always_ff @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            note_idx <= 3'd0;
        end else begin
            case (state)
                IDLE: begin
                    note_idx <= 3'd0;
                end

                LOW_BATT: begin
                    if (dur_cnt + dur_inc >= dur_lim) begin
                        if (note_idx == 3'd2)
                            note_idx <= 3'd0;
                        else
                            note_idx <= note_idx + 3'd1;
                    end
                end

                FANFARE: begin
                    if (dur_cnt + dur_inc >= dur_lim) begin
                        if (note_idx == 3'd5)
                            note_idx <= 3'd0;
                        else
                            note_idx <= note_idx + 3'd1;
                    end
                end
            endcase
        end
    end

    // state transition
    always_ff @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            state <= IDLE;
        end else begin
            state <= nxt_state;
        end
    end

    // next state logic
    always_comb begin
        nxt_state = state;

        case (state)
            IDLE: begin
                if (batt_low)
                    nxt_state = LOW_BATT;
                else if (fanfare)
                    nxt_state = FANFARE;
            end

            LOW_BATT: begin
                if (!batt_low) begin
                    if (fanfare)
                        nxt_state = FANFARE;
                    else
                        nxt_state = IDLE;
                end
            end

            FANFARE: begin
                if ((dur_cnt + dur_inc >= dur_lim) && (note_idx == 3'd5)) begin
                    if (batt_low)
                        nxt_state = LOW_BATT;
                    else
                        nxt_state = IDLE;
                end
            end

            default: begin
                nxt_state = IDLE;
            end
        endcase
    end

    // piezo output generation
    always_ff @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            piezo <= 1'b0;
        end else if (state == IDLE) begin
            piezo <= 1'b0;
        end else if (freq_cnt >= freq_div - 1) begin
            piezo <= ~piezo;
        end
    end

    assign piezo_n = ~piezo;

endmodule