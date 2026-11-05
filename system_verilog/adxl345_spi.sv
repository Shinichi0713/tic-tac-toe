// adxl345_spi.sv
// ADXL345 加速度センサ SPI制御モジュール
// 初期化 → 連続読み出し を自動実行
// Pmod ACL (Digilent) 対応

module adxl345_spi #(
    parameter int CLK_HZ = 100_000_000
)(
    input  logic clk,
    input  logic rst_n,

    // 加速度データ出力
    output logic signed [15:0] accel_x,
    output logic signed [15:0] accel_y,
    output logic signed [15:0] accel_z,
    output logic               data_valid,

    // 状態・エラー出力
    output logic [3:0]  status_led,
    output logic        init_done,

    // SPI物理信号
    output logic spi_sck,
    output logic spi_mosi,
    input  logic spi_miso,
    output logic spi_cs_n
);

    //==========================================================================
    // ADXL345 レジスタ定義
    //==========================================================================
    localparam logic [7:0] REG_DEVID       = 8'h00;  // 読み出しで0xE5
    localparam logic [7:0] REG_THRESH_TAP  = 8'h1D;
    localparam logic [7:0] REG_OFSX        = 8'h1E;
    localparam logic [7:0] REG_OFSY        = 8'h1F;
    localparam logic [7:0] REG_OFSZ        = 8'h20;
    localparam logic [7:0] REG_DUR          = 8'h21;
    localparam logic [7:0] REG_LATENT      = 8'h22;
    localparam logic [7:0] REG_WINDOW      = 8'h23;
    localparam logic [7:0] REG_THRESH_ACT  = 8'h24;
    localparam logic [7:0] REG_THRESH_INACT = 8'h25;
    localparam logic [7:0] REG_TIME_INACT  = 8'h26;
    localparam logic [7:0] REG_ACT_INACT_CTL = 8'h27;
    localparam logic [7:0] REG_THRESH_FF   = 8'h28;
    localparam logic [7:0] REG_TIME_FF     = 8'h29;
    localparam logic [7:0] REG_TAP_AXES    = 8'h2A;
    localparam logic [7:0] REG_ACT_TAP_STATUS = 8'h2B;
    localparam logic [7:0] REG_BW_RATE      = 8'h2C;
    localparam logic [7:0] REG_POWER_CTL   = 8'h2D;
    localparam logic [7:0] REG_INT_ENABLE  = 8'h2E;
    localparam logic [7:0] REG_INT_MAP      = 8'h2F;
    localparam logic [7:0] REG_INT_SOURCE  = 8'h30;
    localparam logic [7:0] REG_DATA_FORMAT = 8'h31;
    localparam logic [7:0] REG_DATAX0      = 8'h32;
    localparam logic [7:0] REG_DATAX1      = 8'h33;
    localparam logic [7:0] REG_DATAY0      = 8'h34;
    localparam logic [7:0] REG_DATAY1      = 8'h35;
    localparam logic [7:0] REG_DATAZ0      = 8'h36;
    localparam logic [7:0] REG_DATAZ1      = 8'h37;
    localparam logic [7:0] REG_FIFO_CTL    = 8'h38;
    localparam logic [7:0] REG_FIFO_STATUS = 8'h39;

    // 読み出しビット
    localparam logic READ_BIT = 1'b1;
    // マルチバイトビット
    localparam logic MB_BIT   = 1'b1;

    //==========================================================================
    // 状態機械
    //==========================================================================
    typedef enum logic [4:0] {
        ST_IDLE,
        ST_CHECK_DEVID,
        ST_WAIT_DEVID,
        ST_SET_BW_RATE,
        ST_WAIT_BW,
        ST_SET_DATA_FORMAT,
        ST_WAIT_FORMAT,
        ST_SET_POWER_CTL,
        ST_WAIT_POWER,
        ST_WAIT_STARTUP,      // 測定開始後のスタートアップ待ち
        ST_READ_XYZ_CMD,
        ST_READ_XYZ_DATA0,
        ST_READ_XYZ_DATA1,
        ST_READ_XYZ_DATA2,
        ST_READ_XYZ_DATA3,
        ST_READ_XYZ_DATA4,
        ST_READ_XYZ_DATA5,
        ST_WAIT_NEXT          // 次のサンプリングまで待機
    } state_t;

    state_t state;

    //==========================================================================
    // タイマー
    //==========================================================================
    localparam int STARTUP_US = 2000;   // 測定開始後2ms待ち
    localparam int SAMPLE_US  = 10000;  // 100Hzサンプリング (10ms)
    localparam int TIMER_MAX  = CLK_HZ / 1000_000;  // 1us分のクロック数

    logic [$clog2(STARTUP_US * TIMER_MAX)-1:0] timer;
    logic timer_tick;

    always_ff @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            timer <= '0;
        end else if (state == ST_WAIT_STARTUP || state == ST_WAIT_NEXT) begin
            if (timer < TIMER_MAX - 1) begin
                timer <= timer + 1'b1;
            end else begin
                timer <= '0;
            end
        end else begin
            timer <= '0;
        end
    end

    assign timer_tick = (timer == TIMER_MAX - 1);

    //==========================================================================
    // SPIインターフェース
    //==========================================================================
    logic [7:0]  spi_tx_data;
    logic        spi_tx_valid;
    logic        spi_tx_ready;
    logic [7:0]  spi_rx_data;
    logic        spi_rx_valid;
    logic [4:0]  spi_xfer_len;
    logic        spi_auto_inc;

    spi_master #(
        .CLK_HZ(CLK_HZ),
        .SPI_CLK_HZ(2_500_000)  // ADXL345安全範囲 2.5MHz
    ) u_spi_master (
        .clk(clk),
        .rst_n(rst_n),
        .tx_data(spi_tx_data),
        .tx_valid(spi_tx_valid),
        .tx_ready(spi_tx_ready),
        .rx_data(spi_rx_data),
        .rx_valid(spi_rx_valid),
        .xfer_len(spi_xfer_len),
        .auto_inc(spi_auto_inc),
        .spi_sck(spi_sck),
        .spi_mosi(spi_mosi),
        .spi_miso(spi_miso),
        .spi_cs_n(spi_cs_n)
    );

    //==========================================================================
    // データバッファ
    //==========================================================================
    logic [7:0] rx_buf [6];
    logic [2:0] rx_idx;

    //==========================================================================
    // メイン状態機械
    //==========================================================================
    always_ff @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            state      <= ST_IDLE;
            spi_tx_valid <= 1'b0;
            data_valid <= 1'b0;
            init_done  <= 1'b0;
            accel_x    <= '0;
            accel_y    <= '0;
            accel_z    <= '0;
            rx_idx     <= '0;
            status_led <= 4'b0000;
        end else begin
            data_valid <= 1'b0;
            spi_tx_valid <= 1'b0;

            case (state)
                ST_IDLE: begin
                    state <= ST_CHECK_DEVID;
                end

                //==================================================================
                // 初期化シーケンス
                //==================================================================
                ST_CHECK_DEVID: begin
                    // DEVIDレジスタ読み出し (0x00 → 0xE5)
                    spi_tx_data  <= {READ_BIT, MB_BIT, REG_DEVID[5:0]};
                    spi_xfer_len <= 5'd2;  // コマンド + データ
                    spi_auto_inc <= 1'b0;
                    spi_tx_valid <= 1'b1;
                    if (spi_tx_ready) begin
                        state <= ST_WAIT_DEVID;
                    end
                end

                ST_WAIT_DEVID: begin
                    if (spi_rx_valid) begin
                        if (rx_idx == 0) begin
                            rx_idx <= rx_idx + 1'b1;
                        end else begin
                            rx_idx <= '0;
                            if (spi_rx_data == 8'hE5) begin
                                status_led[0] <= 1'b1;  // DEVID OK
                                state <= ST_SET_BW_RATE;
                            end else begin
                                // エラー: DEVID不一致
                                status_led <= 4'b1110;
                                state <= ST_IDLE;  // リトライ
                            end
                        end
                    end
                end

                ST_SET_BW_RATE: begin
                    // BW_RATE: 100Hzデータレート (0x0A)
                    spi_tx_data  <= REG_BW_RATE;
                    spi_xfer_len <= 5'd2;
                    spi_auto_inc <= 1'b0;
                    spi_tx_valid <= 1'b1;
                    if (spi_tx_ready) begin
                        state <= ST_WAIT_BW;
                    end
                end

                ST_WAIT_BW: begin
                    if (spi_rx_valid && rx_idx == 1) begin
                        rx_idx <= '0;
                        status_led[1] <= 1'b1;
                        state <= ST_SET_DATA_FORMAT;
                    end else if (spi_rx_valid) begin
                        rx_idx <= rx_idx + 1'b1;
                    end
                end

                ST_SET_DATA_FORMAT: begin
                    // DATA_FORMAT: フル解像度、右詰め、±2g (0x00)
                    spi_tx_data  <= REG_DATA_FORMAT;
                    spi_xfer_len <= 5'd2;
                    spi_auto_inc <= 1'b0;
                    spi_tx_valid <= 1'b1;
                    if (spi_tx_ready) begin
                        state <= ST_WAIT_FORMAT;
                    end
                end

                ST_WAIT_FORMAT: begin
                    if (spi_rx_valid && rx_idx == 1) begin
                        rx_idx <= '0;
                        status_led[2] <= 1'b1;
                        state <= ST_SET_POWER_CTL;
                    end else if (spi_rx_valid) begin
                        rx_idx <= rx_idx + 1'b1;
                    end
                end

                ST_SET_POWER_CTL: begin
                    // POWER_CTL: 測定モード開始 (0x08)
                    spi_tx_data  <= REG_POWER_CTL;
                    spi_xfer_len <= 5'd2;
                    spi_auto_inc <= 1'b0;
                    spi_tx_valid <= 1'b1;
                    if (spi_tx_ready) begin
                        state <= ST_WAIT_POWER;
                    end
                end

                ST_WAIT_POWER: begin
                    if (spi_rx_valid && rx_idx == 1) begin
                        rx_idx <= '0;
                        status_led[3] <= 1'b1;
                        init_done <= 1'b1;
                        state <= ST_WAIT_STARTUP;
                    end else if (spi_rx_valid) begin
                        rx_idx <= rx_idx + 1'b1;
                    end
                end

                ST_WAIT_STARTUP: begin
                    // 測定開始後のスタートアップ時間待ち
                    if (timer_tick) begin
                        // timerがusカウントとして動作
                        state <= ST_READ_XYZ_CMD;
                    end
                end

                //==================================================================
                // データ読み出しループ
                //==================================================================
                ST_READ_XYZ_CMD: begin
                    // DATAX0から6バイト連続読み出し
                    // コマンド: READ | MB | 0x32
                    spi_tx_data  <= {READ_BIT, MB_BIT, REG_DATAX0[5:0]};
                    spi_xfer_len <= 5'd7;  // コマンド + 6バイトデータ
                    spi_auto_inc <= 1'b1;
                    spi_tx_valid <= 1'b1;
                    rx_idx <= '0;
                    if (spi_tx_ready) begin
                        state <= ST_READ_XYZ_DATA0;
                    end
                end

                ST_READ_XYZ_DATA0: begin
                    if (spi_rx_valid) begin
                        rx_buf[rx_idx] <= spi_rx_data;
                        if (rx_idx < 5) begin
                            rx_idx <= rx_idx + 1'b1;
                            state <= ST_READ_XYZ_DATA1;  // 実際は連続読み出しなのでrx_validが連続
                        end else begin
                            // 全データ受信完了
                            accel_x <= {rx_buf[1], rx_buf[0]};
                            accel_y <= {rx_buf[3], rx_buf[2]};
                            accel_z <= {rx_buf[5], rx_buf[4]};
                            data_valid <= 1'b1;
                            rx_idx <= '0;
                            state <= ST_WAIT_NEXT;
                        end
                    end
                end

                // 以下は連続読み出しのためのプレースホルダー状態
                // 実際には rx_valid が連続して来るので、ST_READ_XYZ_DATA0 で処理
                ST_READ_XYZ_DATA1: state <= ST_READ_XYZ_DATA0;
                ST_READ_XYZ_DATA2: state <= ST_READ_XYZ_DATA0;
                ST_READ_XYZ_DATA3: state <= ST_READ_XYZ_DATA0;
                ST_READ_XYZ_DATA4: state <= ST_READ_XYZ_DATA0;
                ST_READ_XYZ_DATA5: state <= ST_READ_XYZ_DATA0;

                ST_WAIT_NEXT: begin
                    // 次のサンプリングまで待機
                    if (timer_tick) begin
                        state <= ST_READ_XYZ_CMD;
                    end
                end

                default: state <= ST_IDLE;
            endcase
        end
    end

endmodule
