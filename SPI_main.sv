module SPI_main(
    input  logic        clk,
    input  logic        rst_n,

    input  logic        wrt,
    input  logic [15:0] cmd,
    output logic        done,
    output logic [15:0] rspns,

    output logic        SS_n,
    output logic        SCLK,
    output logic        MOSI,
    input  logic        MISO
);

    typedef enum logic [1:0] {
        IDLE,
        FRONT_PORCH,
        TRANSFER
    } state_t;

    state_t state, nxt_state;

    logic [4:0]  SCLK_div;
    logic [15:0] shft_reg;
    logic [3:0]  bit_cnt;
    logic        MISO_smpl;

    logic init;
    logic shft;
    logic smpl;
    logic shft_imm;
    logic done15;

    assign SCLK = SCLK_div[4];
    assign MOSI = shft_reg[15];
    assign rspns = shft_reg;

    // sample before next SCLK rising edge
    assign smpl = (state == TRANSFER) && (SCLK_div == 5'b01111);

    // shift before next SCLK falling edge
    assign shft_imm = (SCLK_div == 5'b11111);

    assign done15 = &bit_cnt;

    // FSM sequential
    always_ff @(posedge clk, negedge rst_n) begin
        if (!rst_n)
            state <= IDLE;
        else
            state <= nxt_state;
    end

    // FSM combinational
    always_comb begin
        nxt_state = state;
        init      = 1'b0;
        shft      = 1'b0;

        case (state)
            IDLE: begin
                if (wrt) begin
                    init      = 1'b1;
                    nxt_state = FRONT_PORCH;
                end
            end

            FRONT_PORCH: begin
                // ignore the first SCLK falling edge
                if (shft_imm)
                    nxt_state = TRANSFER;
            end

            TRANSFER: begin
                if (shft_imm) begin
                    shft = 1'b1;

                    if (done15)
                        nxt_state = IDLE;
                end
            end

            default: begin
                nxt_state = IDLE;
            end
        endcase
    end

    // SCLK divider
    always_ff @(posedge clk, negedge rst_n) begin
        if (!rst_n)
            SCLK_div <= 5'b10111;
        else if (init)
            SCLK_div <= 5'b10111;
        else if (state != IDLE)
            SCLK_div <= SCLK_div + 5'b00001;
    end

    // MISO sample flop
    always_ff @(posedge clk, negedge rst_n) begin
        if (!rst_n)
            MISO_smpl <= 1'b0;
        else if (smpl)
            MISO_smpl <= MISO;
    end

    // Shift register
    always_ff @(posedge clk, negedge rst_n) begin
        if (!rst_n)
            shft_reg <= 16'h0000;
        else if (init)
            shft_reg <= cmd;
        else if (shft)
            shft_reg <= {shft_reg[14:0], MISO_smpl};
    end

    // Bit counter
    always_ff @(posedge clk, negedge rst_n) begin
        if (!rst_n)
            bit_cnt <= 4'h0;
        else if (init)
            bit_cnt <= 4'h0;
        else if (shft)
            bit_cnt <= bit_cnt + 4'h1;
    end

    // SS_n flop
    always_ff @(posedge clk, negedge rst_n) begin
        if (!rst_n)
            SS_n <= 1'b1;
        else if (init)
            SS_n <= 1'b0;
        else if ((state == TRANSFER) && shft_imm && done15)
            SS_n <= 1'b1;
    end

    // Done flop: stays high until next wrt
    always_ff @(posedge clk, negedge rst_n) begin
        if (!rst_n)
            done <= 1'b0;
        else if (init)
            done <= 1'b0;
        else if ((state == TRANSFER) && shft_imm && done15)
            done <= 1'b1;
    end

endmodule