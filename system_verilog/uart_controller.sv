// uart_controller.sv
// フル機能UARTコントローラ（TX/RX + FIFOバッファ）
// ボーレート生成、スタートビット検出、サンプリング調整を含む

module uart_controller #(
    parameter int CLK_HZ      = 50_000_000,
    parameter int BAUD_RATE   = 115_200,
    parameter int DATA_BITS   = 8,       // 5〜9
    parameter int STOP_BITS   = 1,       // 1, 1.5, 2
    parameter int PARITY      = 0,       // 0:なし, 1:奇数, 2:偶数
    parameter int FIFO_DEPTH  = 16       // TX/RXそれぞれのFIFO深さ
)(
    input  logic clk,
    input  logic rst_n,

    // UART信号
    output logic txd,                    // 送信データ
    input  logic rxd,                    // 受信データ

    // 制御・状態
    input  logic tx_start,               // 送信開始トリガ
    input  logic [DATA_BITS-1:0] tx_data,// 送信データ入力
    output logic tx_busy,                // 送信中フラグ
    output logic tx_fifo_full,

    output logic [DATA_BITS-1:0] rx_data,// 受信データ出力
    output logic rx_valid,               // 受信データ有効
    input  logic rx_ack,                 // 受信データ読み取り確認
    output logic rx_fifo_empty,

    // エラー出力
    output logic rx_framing_error,       // ストップビット異常
    output logic rx_parity_error         // パリティ不一致
);

    // ボーレート分周比
    localparam int BIT_PERIOD = CLK_HZ / BAUD_RATE;
    localparam int TIMER_W    = $clog2(BIT_PERIOD);

    //==========================================================================
    // ボーレートタイマー（TX/RX共通）
    //==========================================================================
    logic [TIMER_W-1:0] baud_timer;
    logic baud_tick;                     // 1ボーレート周期のパルス

    always_ff @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            baud_timer <= '0;
        end else begin
            if (baud_timer < BIT_PERIOD - 1)
                baud_timer <= baud_timer + 1'b1;
            else
                baud_timer <= '0;
        end
    end

    assign baud_tick = (baud_timer == BIT_PERIOD - 1);

    //==========================================================================
    // TX FIFO（簡易実装）
    //==========================================================================
    logic [DATA_BITS-1:0] tx_fifo [FIFO_DEPTH];
    logic [$clog2(FIFO_DEPTH):0] tx_wp, tx_rp;  // 1ビット余分でフル/エンプティ判定
    logic tx_fifo_empty_int, tx_fifo_full_int;

    assign tx_fifo_empty_int = (tx_wp == tx_rp);
    assign tx_fifo_full_int  = (tx_wp[$clog2(FIFO_DEPTH)] != tx_rp[$clog2(FIFO_DEPTH)]) &&
                               (tx_wp[$clog2(FIFO_DEPTH)-1:0] == tx_rp[$clog2(FIFO_DEPTH)-1:0]);
    assign tx_fifo_full = tx_fifo_full_int;

    always_ff @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            tx_wp <= '0;
        end else if (tx_start && !tx_fifo_full_int) begin
            tx_fifo[tx_wp[$clog2(FIFO_DEPTH)-1:0]] <= tx_data;
            tx_wp <= tx_wp + 1'b1;
        end
    end

    //==========================================================================
    // TX状態機械
    //==========================================================================
    typedef enum logic [2:0] {
        TX_IDLE,
        TX_START,
        TX_DATA,
        TX_PARITY,
        TX_STOP
    } tx_state_t;

    tx_state_t tx_state;
    logic [DATA_BITS-1:0] tx_shift;
    logic [$clog2(DATA_BITS):0] tx_bit_cnt;
    logic tx_parity_bit;

    always_ff @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            tx_state   <= TX_IDLE;
            txd        <= 1'b1;          // アイドル時HIGH
            tx_rp      <= '0;
            tx_bit_cnt <= '0;
        end else if (baud_tick) begin
            case (tx_state)
                TX_IDLE: begin
                    txd <= 1'b1;
                    if (!tx_fifo_empty_int) begin
                        tx_state   <= TX_START;
                        tx_shift   <= tx_fifo[tx_rp[$clog2(FIFO_DEPTH)-1:0]];
                        tx_rp      <= tx_rp + 1'b1;
                        txd        <= 1'b0;  // スタートビット
                        tx_parity_bit <= (PARITY == 2) ? ^tx_fifo[tx_rp[$clog2(FIFO_DEPTH)-1:0]] : ~^tx_fifo[tx_rp[$clog2(FIFO_DEPTH)-1:0]];
                    end
                end

                TX_START: begin
                    tx_state   <= TX_DATA;
                    txd        <= tx_shift[0];
                    tx_bit_cnt <= 'd1;
                end

                TX_DATA: begin
                    if (tx_bit_cnt < DATA_BITS) begin
                        txd <= tx_shift[tx_bit_cnt];
                        tx_bit_cnt <= tx_bit_cnt + 1'b1;
                    end else if (PARITY != 0) begin
                        tx_state <= TX_PARITY;
                        txd      <= tx_parity_bit;
                    end else begin
                        tx_state <= TX_STOP;
                        txd      <= 1'b1;  // ストップビット
                    end
                end

                TX_PARITY: begin
                    tx_state <= TX_STOP;
                    txd      <= 1'b1;
                end

                TX_STOP: begin
                    tx_state <= TX_IDLE;
                    txd      <= 1'b1;
                end

                default: tx_state <= TX_IDLE;
            endcase
        end
    end

    assign tx_busy = (tx_state != TX_IDLE);

    //==========================================================================
    // RX同期化（メタステーブル防止）
    //==========================================================================
    logic rxd_sync1, rxd_sync2;

    always_ff @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            rxd_sync1 <= 1'b1;
            rxd_sync2 <= 1'b1;
        end else begin
            rxd_sync1 <= rxd;
            rxd_sync2 <= rxd_sync1;
        end
    end

    //==========================================================================
    // RXスタートビット検出（1/2ビット周期でサンプリング）
    //==========================================================================
    logic [TIMER_W-1:0] rx_timer;
    logic rx_sample_tick;
    logic rx_active;

    // スタートビット検出でタイマーを半周期ずらす
    always_ff @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            rx_timer <= '0;
            rx_active <= 1'b0;
        end else begin
            if (!rx_active && !rxd_sync2) begin
                // スタートビット検出、半周期待機して中央でサンプリング
                rx_timer <= BIT_PERIOD / 2;
                rx_active <= 1'b1;
            end else if (rx_active) begin
                if (rx_timer < BIT_PERIOD - 1)
                    rx_timer <= rx_timer + 1'b1;
                else
                    rx_timer <= '0;
            end
        end
    end

    assign rx_sample_tick = rx_active && (rx_timer == BIT_PERIOD - 1);

    //==========================================================================
    // RX状態機械
    //==========================================================================
    typedef enum logic [2:0] {
        RX_IDLE,
        RX_DATA,
        RX_PARITY,
        RX_STOP
    } rx_state_t;

    rx_state_t rx_state;
    logic [DATA_BITS-1:0] rx_shift;
    logic [$clog2(DATA_BITS):0] rx_bit_cnt;
    logic rx_parity_calc;
    logic rx_parity_received;

    always_ff @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            rx_state        <= RX_IDLE;
            rx_bit_cnt      <= '0;
            rx_framing_error<= 1'b0;
            rx_parity_error <= 1'b0;
            rx_valid        <= 1'b0;
        end else begin
            rx_valid <= 1'b0;  // デフォルトで無効

            if (rx_sample_tick) begin
                case (rx_state)
                    RX_IDLE: begin
                        if (!rxd_sync2) begin  // スタートビット確認
                            rx_state   <= RX_DATA;
                            rx_bit_cnt <= '0;
                            rx_parity_calc <= (PARITY == 2) ? 1'b0 : 1'b1;  // 偶数:0, 奇数:1
                        end
                    end

                    RX_DATA: begin
                        rx_shift <= {rxd_sync2, rx_shift[DATA_BITS-1:1]};
                        rx_parity_calc <= rx_parity_calc ^ rxd_sync2;
                        if (rx_bit_cnt < DATA_BITS - 1) begin
                            rx_bit_cnt <= rx_bit_cnt + 1'b1;
                        end else if (PARITY != 0) begin
                            rx_state <= RX_PARITY;
                        end else begin
                            rx_state <= RX_STOP;
                        end
                    end

                    RX_PARITY: begin
                        rx_parity_received <= rxd_sync2;
                        rx_state <= RX_STOP;
                    end

                    RX_STOP: begin
                        rx_framing_error <= !rxd_sync2;  // ストップビットはHIGHのはず
                        if (PARITY != 0)
                            rx_parity_error <= (rx_parity_received != rx_parity_calc);
                        rx_valid <= rxd_sync2;  // 正常なストップビットなら有効
                        rx_state <= RX_IDLE;
                        rx_active <= 1'b0;
                    end

                    default: rx_state <= RX_IDLE;
                endcase
            end
        end
    end

    //==========================================================================
    // RX FIFO
    //==========================================================================
    logic [DATA_BITS-1:0] rx_fifo [FIFO_DEPTH];
    logic [$clog2(FIFO_DEPTH):0] rx_wp, rx_rp;

    assign rx_fifo_empty = (rx_wp == rx_rp);

    always_ff @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            rx_wp <= '0;
        end else if (rx_valid) begin
            rx_fifo[rx_wp[$clog2(FIFO_DEPTH)-1:0]] <= rx_shift;
            rx_wp <= rx_wp + 1'b1;
        end
    end

    always_ff @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            rx_rp <= '0;
        end else if (rx_ack && !rx_fifo_empty) begin
            rx_rp <= rx_rp + 1'b1;
        end
    end

    assign rx_data = rx_fifo[rx_rp[$clog2(FIFO_DEPTH)-1:0]];

endmodule