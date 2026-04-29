module SPI_main(
    input clk, rst_n,           //50MHz system clk and async active low reset
    output logic SS_n, SCLK, MOSI,    //SPI protocol signals
    input MISO,                 
    input wrt,                  //SPI transaction indicated by wrt going high for one clk cycle
    input [15:0] cmd,      //data sent to SPI slave
    output logic done,                //asserted when SPI transaction complete; stay asserted until next wrt
    output logic [15:0] rspns       //Data from SPI secondary (only use 7:0 for intertial sensor)
);
    //sclk internal signals
    logic [4:0] sclk_div;
    logic ld_sclk, smpl, shft_imm;
    //shift register internal signals
    logic [15:0] shft_reg;
    logic MISO_smpl;
    logic [1:0] shft_cntrl;
    //bit counter internal signals
    logic [3:0] bit_cnt;
    logic done15;
    //FSM internal signals
    typedef enum logic[1:0] {IDLE, ACTIVE, DONE} state_t;
    state_t state, nxt_state;
    logic init, shft, set_done;
    logic set_done_ff;
    logic init_ff;

    //sclk generation
    //sclk is 1/32 of system clk, sw we count 16 rising edges for 1 half cycle of sclk
    //5-bit counter to count up to 31, load it with 5'b10111 at the beginning of each 
    //sclk cycle so sclk is 1 and resets to 0 after 8 counts (front porch of sclk)
    always_ff @(posedge clk, negedge rst_n) begin
        if(!rst_n) begin
            sclk_div <= 5'b10111;
        end else if(ld_sclk) begin
            sclk_div <= 5'b10111;
        end else begin
            sclk_div <= sclk_div + 1;
        end
    end
    
    //control signals from sclk
    assign SCLK = sclk_div[4];
    //SCLK rising next clk
    assign smpl = (sclk_div == 5'b01111);
    //SCLK falling next clk
    assign shft_imm = (sclk_div == 5'b11111);

    //init_ff
    always_ff @(posedge clk, negedge rst_n) begin
        if(!rst_n) begin
            init_ff <= 1'b0;
        end else begin
            init_ff <= init;
        end
    end
    assign ld_sclk = init_ff; //load sclk divider at beginning of transaction, which is one clk after init is asserted

    //shift register
    //16-bit shift register for both MISO and MOSI
    //sample MISO when smpl asserted
    //shift smpled bit into LSB when shft asserted
    //shift reg loaded with cmd when transaction initiated
    //MISO control
    always_ff @(posedge clk, negedge rst_n) begin
        if(!rst_n) begin
            MISO_smpl <= 1'b0;
        end else if(smpl) begin
            MISO_smpl <= MISO;
        end else begin
            MISO_smpl <= MISO_smpl;
        end
    end

    //shift register control
    always_ff @(posedge clk, negedge rst_n) begin
        if(!rst_n) begin
            shft_reg <= 16'b0;
        end else if(shft_cntrl[1] == 1'b1) begin
            shft_reg <= cmd;
        end else if(shft_cntrl == 2'b01) begin
            shft_reg <= {shft_reg[14:0], MISO_smpl};
        end else if(shft_cntrl == 2'b00) begin
            shft_reg <= shft_reg;
        end
    end
    assign MOSI = shft_reg[15];
    assign shft_cntrl = {init, shft};

    //bit counter
    always_ff @(posedge clk, negedge rst_n) begin
        if(!rst_n) begin
            bit_cnt <= 4'b0;
        end else if(init) begin
            bit_cnt <= 4'b0000; 
        end else if(shft) begin
            bit_cnt <= bit_cnt + 1;
        end 
    end
    assign done15 = &bit_cnt; //done when bit_cnt is 15 (after 16 bits shifted)

    //pipeline set_done to align with end of transaction
    always_ff @(posedge clk, negedge rst_n) begin
        if(!rst_n) begin
            set_done_ff <= 1'b0;
        end else begin
            set_done_ff <= set_done;
        end
    end

    //done control
    always_ff @(posedge clk, negedge rst_n)begin
        if(!rst_n) begin
            done <= 1'b0;
        end else if(init) begin
            done <= 1'b0;
        end else if(set_done_ff) begin
            done <= 1'b1;
        end
    end

    //SS_n control
    always_ff @(posedge clk, negedge rst_n) begin
        if(!rst_n) begin
            SS_n <= 1'b1; //inactive high
        end else if(init) begin
            SS_n <= 1'b0; //active low when transaction initiated
        end else if(set_done_ff) begin
            SS_n <= 1'b1; //inactive high when transaction done
        end
    end

    //rspns control
    always_ff @(posedge clk, negedge rst_n) begin
        if(!rst_n) begin
            rspns <= 16'b0;
        end else if(set_done_ff) begin
            rspns <= shft_reg; //load the received data into rspns when transaction done
        end else begin
            rspns <= rspns; //hold the value of rspns until next transaction
        end
    end

    //FSM 
    always_ff @ (posedge clk, negedge rst_n) begin
        if(!rst_n) begin
            state <= IDLE;
        end else begin
            state <= nxt_state;
        end
    end
    //FSM next state logic and output logic
    always_comb begin
        init = 0;
        shft = 0;
       //ld_sclk = 0;
        set_done = 0;
        nxt_state = state;

        case(state) 
            IDLE: begin
                if(wrt) begin
                    init = 1;
                    // ld_sclk = 1;
                    nxt_state = ACTIVE;
                end
            end   
            ACTIVE: begin
                if(shft_imm) begin
                    shft = 1;
                    if(done15) begin
                        set_done = 1;
                        nxt_state = DONE;
                    end
                end
            end    
            DONE: begin
                set_done = 1; //keep done high until next transaction
                if(wrt) begin
                    set_done = 0;
                    init = 1;
                    // ld_sclk = 1;
                    //shft = 1;
                    nxt_state = ACTIVE;
                end
            end     
        endcase
    end


endmodule