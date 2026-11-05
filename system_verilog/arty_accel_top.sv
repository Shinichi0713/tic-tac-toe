// arty_accel_top.sv
// Arty A7 + Pmod ACL (ADXL345) 加速度センサ デモ
// Pmod JA に接続:
//   JA1=CS, JA2=MOSI, JA3=MISO, JA4=SCK
// LED/RGB LED で加速度を表示、UARTで数値送信

module arty_accel_top (
    input  logic CLK100MHZ,
    input  logic [3:0] SW,
    input  logic [3:0] BTN,
    output logic [3:0] LED,
    output logic RGB0_Red,
    output logic RGB0_Green,
    output logic RGB0_Blue,
    output logic RGB1_Red,
    output logic RGB1_Green,
    output logic RGB1_Blue,
    output logic RGB2_Red,
    output logic RGB2_Green,
    output logic RGB2_Blue,
    output logic RGB3_Red,
    output logic RGB3_Green,
    output logic RGB3_Blue,
    output logic UART_TXD,
    // Pmod JA (SPI)
    output logic JA1,   // CS
    output logic JA2,   // MOSI
    input  logic JA3,   // MISO
    output logic JA4    // SCK
);

    localparam int CLK_HZ = 100_000_000;

    // リセット
    logic rst_n;
    assign rst_n = ~BTN[0];  // BTN[0]押下でリセット

    // ADXL345信号
    logic signed [15:0] accel_x, accel_y, accel_z;
    logic               data_valid;
    logic               init_done;
    logic [3:0]         status;

    // SPI信号
    logic spi_sck, spi_mosi, spi_cs_n;

    //==========================================================================
    // ADXL345コントローラ
    //==========================================================================
    adxl345_spi #(
        .CLK_HZ(CLK_HZ)
    ) u_adxl345 (
        .clk(CLK100MHZ),
        .rst_n(rst_n),
        .accel_x(accel_x),
        .accel_y(accel_y),
        .accel_z(accel_z),
        .data_valid(data_valid),
        .status_led(status),
        .init_done(init_done),
        .spi_sck(spi_sck),
        .spi_mosi(spi_mosi),
        .spi_miso(JA3),
        .spi_cs_n(spi_cs_n)
    );

    // Pmod JA ピン割り当て
    assign JA1 = spi_cs_n;
    assign JA2 = spi_mosi;
    assign JA4 = spi_sck;

    //==========================================================================
    // LED表示ロジック
    //==========================================================================
    // SWで表示モード切替
    // SW[1:0] = 00: X軸上位4bit → LED
    // SW[1:0] = 01: Y軸上位4bit → LED
    // SW[1:0] = 10: Z軸上位4bit → LED
    // SW[1:0] = 11: ステータス → LED

    logic [3:0] led_value;
    always_comb begin
        case (SW[1:0])
            2'b00: led_value = accel_x[11:8];   // X軸（上位有意ビット）
            2'b01: led_value = accel_y[11:8];   // Y軸
            2'b10: led_value = accel_z[11:8];   // Z軸
            2'b11: led_value = status;           // 初期化ステータス
        endcase
    end

    assign LED = led_value;

    //==========================================================================
    // RGB LED 表示（傾きインジケータ）
    // RGB0: X軸傾き（赤=負、緑=正）
    // RGB1: Y軸傾き（赤=負、緑=正）
    // RGB2: Z軸（青で常時表示、強度変化）
    //==========================================================================
    // X軸: ±256 LSB/g 程度が1g
    // しきい値で判定
    localparam signed [15:0] TILT_THRESH = 16'sd200;

    assign RGB0_Red   = init_done && (accel_x < -TILT_THRESH);
    assign RGB0_Green = init_done && (accel_x >  TILT_THRESH);
    assign RGB0_Blue  = !init_done;  // 初期化中は青点灯

    assign RGB1_Red   = init_done && (accel_y < -TILT_THRESH);
    assign RGB1_Green = init_done && (accel_y >  TILT_THRESH);
    assign RGB1_Blue  = 1'b0;

    // Z軸: 重力方向（通常は約+1g = +256程度）
    assign RGB2_Red   = 1'b0;
    assign RGB2_Green = 1'b0;
    assign RGB2_Blue  = init_done && (accel_z > TILT_THRESH);

    // RGB3: 未使用
    assign RGB3_Red   = 1'b0;
    assign RGB3_Green = 1'b0;
    assign RGB3_Blue  = 1'b0;

    //==========================================================================
    // UART 送信（加速度値をテキスト送信）
    //==========================================================================
    logic [7:0] uart_tx_data;
    logic       uart_tx_valid;
    logic       uart_tx_busy;

    uart_tx #(
        .CLK_HZ(CLK_HZ),
        .BAUD_RATE(115_200)
    ) u_uart_tx (
        .clk(CLK100MHZ),
        .rst_n(rst_n),
        .tx_data(uart_tx_data),
        .tx_valid(uart_tx_valid),
        .tx_busy(uart_tx_busy),
        .txd(UART_TXD)
    );

    // UART送信フォーマット: "X:#### Y:#### Z:####\n"
    // 16進数4桁で送信
    logic [7:0] hex_digits [6];  // XH, XL, YH, YL, ZH, ZL
    logic [3:0] uart_state;
    logic [3:0] send_idx;
    logic       send_trigger;

    // データ有効時に送信バッファを更新
    always_ff @(posedge CLK100MHZ or negedge rst_n) begin
        if (!rst_n) begin
            hex_digits[0] <= "0";
            hex_digits[1] <= "0";
            hex_digits[2] <= "0";
            hex_digits[3] <= "0";
            hex_digits[4] <= "0";
            hex_digits[5] <= "0";
        end else if (data_valid) begin
            hex_digits[0] <= nibble_to_hex(accel_x[15:12]);
            hex_digits[1] <= nibble_to_hex(accel_x[11:8]);
            hex_digits[2] <= nibble_to_hex(accel_y[15:12]);
            hex_digits[3] <= nibble_to_hex(accel_y[11:8]);
            hex_digits[4] <= nibble_to_hex(accel_z[15:12]);
            hex_digits[5] <= nibble_to_hex(accel_z[11:8]);
        end
    end

    // 4bit → HEX文字変換関数
    function automatic logic [7:0] nibble_to_hex(logic [3:0] nibble);
        case (nibble)
            4'h0: return "0";  4'h1: return "1";
            4'h2: return "2";  4'h3: return "3";
            4'h4: return "4";  4'h5: return "5";
            4'h6: return "6";  4'h7: return "7";
            4'h8: return "8";  4'h9: return "9";
            4'hA: return "A";  4'hB: return "B";
            4'hC: return "C";  4'hD: return "D";
            4'hE: return "E";  4'hF: return "F";
        endcase
    endfunction

    // UART送信状態機械
    always_ff @(posedge CLK100MHZ or negedge rst_n) begin
        if (!rst_n) begin
            uart_state   <= 4'd0;
            send_idx     <= 4'd0;
            uart_tx_valid <= 1'b0;
        end else begin
            uart_tx_valid <= 1'b0;

            case (uart_state)
                4'd0: if (data_valid && SW[3]) uart_state <= 4'd1;  // SW[3]でUART有効
                4'd1: begin uart_tx_data <= "X"; uart_tx_valid <= 1'b1; uart_state <= 4'd2; end
                4'd2: if (!uart_tx_busy) begin uart_tx_data <= ":"; uart_tx_valid <= 1'b1; uart_state <= 4'd3; end
                4'd3: if (!uart_tx_busy) begin uart_tx_data <= hex_digits[0]; uart_tx_valid <= 1'b1; uart_state <= 4'd4; end
                4'd4: if (!uart_tx_busy) begin uart_tx_data <= hex_digits[1]; uart_tx_valid <= 1'b1; uart_state <= 4'd5; end
                4'd5: if (!uart_tx_busy) begin uart_tx_data <= " "; uart_tx_valid <= 1'b1; uart_state <= 4'd6; end
                4'd6: if (!uart_tx_busy) begin uart_tx_data <= "Y"; uart_tx_valid <= 1'b1; uart_state <= 4'd7; end
                4'd7: if (!uart_tx_busy) begin uart_tx_data <= ":"; uart_tx_valid <= 1'b1; uart_state <= 4'd8; end
                4'd8: if (!uart_tx_busy) begin uart_tx_data <= hex_digits[2]; uart_tx_valid <= 1'b1; uart_state <= 4'd9; end
                4'd9: if (!uart_tx_busy) begin uart_tx_data <= hex_digits[3]; uart_tx_valid <= 1'b1; uart_state <= 4'd10; end
                4'd10: if (!uart_tx_busy) begin uart_tx_data <= " "; uart_tx_valid <= 1'b1; uart_state <= 4'd11; end
                4'd11: if (!uart_tx_busy) begin uart_tx_data <= "Z"; uart_tx_valid <= 1'b1; uart_state <= 4'd12; end
                4'd12: if (!uart_tx_busy) begin uart_tx_data <= ":"; uart_tx_valid <= 1'b1; uart_state <= 4'd13; end
                4'd13: if (!uart_tx_busy) begin uart_tx_data <= hex_digits[4]; uart_tx_valid <= 1'b1; uart_state <= 4'd14; end
                4'd14: if (!uart_tx_busy) begin uart_tx_data <= hex_digits[5]; uart_tx_valid <= 1'b1; uart_state <= 4'd15; end
                4'd15: if (!uart_tx_busy) begin uart_tx_data <= 8'h0D; uart_tx_valid <= 1'b1; uart_state <= 4'd0; end  // \r
                default: uart_state <= 4'd0;
            endcase
        end
    end

endmodule
