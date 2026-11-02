// uart_tx.sv
// 8N1 UART 送信器（TXのみ）
// 1スタートビット + 8データビット + 1ストップビット、パリティなし

module uart_tx #(
    parameter int CLK_HZ    = 100_000_000,
    parameter int BAUD_RATE = 115_200
)(
    input  logic clk,
    input  logic rst_n,
    input  logic [7:0] tx_data,
    input  logic       tx_valid,
    output logic       tx_busy,
    output logic       txd
);

    localparam int BIT_PERIOD = CLK_HZ / BAUD_RATE;
    localparam int TIMER_W    = $clog2(BIT_PERIOD);

    typedef enum logic [2:0] {
        TX_IDLE,
        TX_START,
        TX_DATA,
        TX_STOP
    } tx_state_t;

    tx_state_t state;
    logic [TIMER_W-1:0] timer;
    logic [7:0] shift_reg;
    logic [2:0] bit_cnt;

    always_ff @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            state    <= TX_IDLE;
            txd      <= 1'b1;  // アイドル時HIGH
            timer    <= '0;
            bit_cnt  <= '0;
            shift_reg <= '0;
        end else begin
            case (state)
                TX_IDLE: begin
                    txd <= 1'b1;
                    if (tx_valid) begin
                        state     <= TX_START;
                        shift_reg <= tx_data;
                        txd       <= 1'b0;  // スタートビット
                        timer     <= '0;
                    end
                end

                TX_START: begin
                    if (timer < BIT_PERIOD - 1) begin
                        timer <= timer + 1'b1;
                    end else begin
                        timer   <= '0;
                        state   <= TX_DATA;
                        txd     <= shift_reg[0];  // LSB first
                        bit_cnt <= '0;
                    end
                end

                TX_DATA: begin
                    if (timer < BIT_PERIOD - 1) begin
                        timer <= timer + 1'b1;
                    end else begin
                        timer <= '0;
                        if (bit_cnt < 7) begin
                            bit_cnt <= bit_cnt + 1'b1;
                            txd <= shift_reg[bit_cnt + 1];
                        end else begin
                            state <= TX_STOP;
                            txd   <= 1'b1;  // ストップビット
                        end
                    end
                end

                TX_STOP: begin
                    if (timer < BIT_PERIOD - 1) begin
                        timer <= timer + 1'b1;
                    end else begin
                        timer <= '0;
                        state <= TX_IDLE;
                    end
                end

                default: state <= TX_IDLE;
            endcase
        end
    end

    assign tx_busy = (state != TX_IDLE);

endmodule
