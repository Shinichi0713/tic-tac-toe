// spi_master.sv
// 汎用SPIマスター (CPOL=1, CPHA=1 対応)
// ADXL345 加速度センサ用に最適化
// 100MHzシステムクロック → SPIクロック最大5MHz

module spi_master #(
    parameter int CLK_HZ      = 100_000_000,
    parameter int SPI_CLK_HZ  = 5_000_000,   // ADXL345 max 5MHz
    parameter int DATA_WIDTH  = 8
)(
    input  logic clk,
    input  logic rst_n,

    // 制御インターフェース
    input  logic [DATA_WIDTH-1:0] tx_data,    // 送信データ
    input  logic                  tx_valid,     // 送信トリガ
    output logic                  tx_ready,     // 受付可能
    output logic [DATA_WIDTH-1:0] rx_data,    // 受信データ
    output logic                  rx_valid,     // 受信有効

    // マルチバイトモード
    input  logic [4:0]            xfer_len,    // 転送バイト数 (1〜32)
    input  logic                  auto_inc,    // アドレス自動インクリメント

    // SPI物理信号 (CPOL=1, CPHA=1)
    output logic spi_sck,
    output logic spi_mosi,
    input  logic spi_miso,
    output logic spi_cs_n
);

    localparam int CLK_DIV = CLK_HZ / (SPI_CLK_HZ * 2);
    localparam int DIV_W   = $clog2(CLK_DIV);

    typedef enum logic [2:0] {
        SPI_IDLE,
        SPI_CS_SETUP,    // CS立ち下げ後のセットアップ (CPHA=1)
        SPI_XFER,
        SPI_CS_HOLD,     // 最終ビット後のホールド
        SPI_CS_DISABLE   // CS無効化
    } spi_state_t;

    spi_state_t state;

    logic [DIV_W-1:0]       clk_div_cnt;
    logic                   sck_toggle;
    logic [4:0]             bit_cnt;       // 現在のバイト内ビットカウンタ
    logic [4:0]             byte_cnt;       // 転送済みバイト数
    logic [4:0]             xfer_len_reg;
    logic                   auto_inc_reg;
    logic [DATA_WIDTH-1:0]  shift_reg;
    logic [DATA_WIDTH-1:0]  rx_shift;
    logic                   sck_phase;     // 0:立ち下げ, 1:立ち上げ

    // SCK生成（CPOL=1: アイドルHIGH）
    always_ff @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            clk_div_cnt <= '0;
            sck_toggle  <= 1'b0;
        end else begin
            if (state == SPI_XFER) begin
                if (clk_div_cnt < CLK_DIV - 1) begin
                    clk_div_cnt <= clk_div_cnt + 1'b1;
                    sck_toggle  <= 1'b0;
                end else begin
                    clk_div_cnt <= '0;
                    sck_toggle  <= 1'b1;
                end
            end else begin
                clk_div_cnt <= '0;
                sck_toggle  <= 1'b0;
            end
        end
    end

    // SCK出力（CPOL=1）
    always_ff @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            spi_sck <= 1'b1;  // CPOL=1: アイドルHIGH
        end else if (state == SPI_XFER && sck_toggle) begin
            spi_sck <= ~spi_sck;
        end else if (state != SPI_XFER) begin
            spi_sck <= 1'b1;
        end
    end

    // メイン状態機械
    always_ff @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            state        <= SPI_IDLE;
            spi_cs_n     <= 1'b1;
            spi_mosi     <= 1'b1;
            bit_cnt      <= '0;
            byte_cnt     <= '0;
            shift_reg    <= '0;
            rx_shift     <= '0;
            rx_valid     <= 1'b0;
            xfer_len_reg <= '0;
            sck_phase    <= 1'b0;
        end else begin
            rx_valid <= 1'b0;

            case (state)
                SPI_IDLE: begin
                    spi_cs_n <= 1'b1;
                    spi_mosi <= 1'b1;
                    bit_cnt  <= '0;
                    byte_cnt <= '0;
                    sck_phase <= 1'b0;
                    if (tx_valid) begin
                        state        <= SPI_CS_SETUP;
                        spi_cs_n     <= 1'b0;          // CS有効化
                        shift_reg    <= tx_data;
                        xfer_len_reg <= xfer_len;
                        auto_inc_reg <= auto_inc;
                    end
                end

                SPI_CS_SETUP: begin
                    // CS立ち下げ後、最初の立ち下げエッジまで待機
                    // CPHA=1: 最初のSCK立ち上げでサンプリング
                    if (sck_toggle) begin
                        state     <= SPI_XFER;
                        sck_phase <= 1'b0;
                    end
                end

                SPI_XFER: begin
                    if (sck_toggle) begin
                        sck_phase <= ~sck_phase;
                        if (!sck_phase) begin
                            // SCK立ち下げ: MOSIにデータをセット
                            spi_mosi <= shift_reg[DATA_WIDTH-1];
                        end else begin
                            // SCK立ち上げ: MISOをサンプリング
                            rx_shift <= {rx_shift[DATA_WIDTH-2:0], spi_miso};
                            bit_cnt  <= bit_cnt + 1'b1;
                            shift_reg <= {shift_reg[DATA_WIDTH-2:0], 1'b0};

                            if (bit_cnt == DATA_WIDTH - 1) begin
                                // 1バイト転送完了
                                rx_valid <= 1'b1;
                                rx_data  <= {rx_shift[DATA_WIDTH-2:0], spi_miso};
                                byte_cnt <= byte_cnt + 1'b1;
                                bit_cnt  <= '0;

                                if (byte_cnt + 1 >= xfer_len_reg) begin
                                    state <= SPI_CS_HOLD;
                                end else begin
                                    // 次のバイトへ
                                    if (auto_inc_reg) begin
                                        shift_reg <= shift_reg;  // ダミー（読み出し時は0x00送信）
                                    end
                                end
                            end
                        end
                    end
                end

                SPI_CS_HOLD: begin
                    // 最終ビット後、CSを一定期間保持
                    if (sck_toggle) begin
                        state    <= SPI_CS_DISABLE;
                        spi_cs_n <= 1'b1;
                    end
                end

                SPI_CS_DISABLE: begin
                    if (sck_toggle) begin
                        state <= SPI_IDLE;
                    end
                end

                default: state <= SPI_IDLE;
            endcase
        end
    end

    assign tx_ready = (state == SPI_IDLE);

endmodule
