// arty_a7_top.sv
// Arty A7-35T / A7-100T 用 統合デモプロジェクト
// 機能:
//   - RGB LED0: PWM呼吸LED（SW[0]=0時）
//   - LED[3:0]: BTN[1] 押下回数表示（SW[0]=1時）
//   - UART: 1秒ごとにカウンタ値を送信
//   - 全ボタン入力: チャタリング除去付き
//   - 100MHz システムクロック対応
//
// クロック: 100MHz (E3)
// ボタンは負論理（押すとLOW）

module arty_a7_top (
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
    output logic UART_TXD
);

    //==========================================================================
    // パラメータ
    //==========================================================================
    localparam int CLK_HZ      = 100_000_000;  // 100MHz
    localparam int UART_BAUD   = 115_200;
    localparam int DEBOUNCE_MS = 20;
    localparam int PWM_RES     = 8;             // 8bit PWM

    //==========================================================================
    // 内部信号
    //==========================================================================
    logic rst_n;
    logic [3:0] btn_debounced;
    logic [3:0] btn_posedge;      // 立ち上がり検出（負論理ボタンなので押下時）
    logic [3:0] press_count;
    logic [PWM_RES-1:0] pwm_val;
    logic pwm_out;
    logic [7:0] uart_tx_data;
    logic       uart_tx_valid;
    logic       uart_tx_busy;

    // リセット: BTN[0] をリセットとして使用（負論理、プルアップ）
    assign rst_n = ~BTN[0];  // BTN[0]押下でリセット

    //==========================================================================
    // ボタンチャタリング除去（4チャネル）
    //==========================================================================
    debounce_multi #(
        .NUM_CHANNELS(4),
        .CLK_HZ(CLK_HZ),
        .DEBOUNCE_MS(DEBOUNCE_MS)
    ) u_debounce (
        .clk(CLK100MHZ),
        .rst_n(rst_n),
        .sig_in(~BTN),              // 負論理ボタンを正論理に変換
        .sig_out(btn_debounced),
        .posedge_detect(btn_posedge)
    );

    //==========================================================================
    // ボタン押下回数カウンタ（BTN[1]）
    //==========================================================================
    always_ff @(posedge CLK100MHZ or negedge rst_n) begin
        if (!rst_n) begin
            press_count <= '0;
        end else if (btn_posedge[1]) begin
            press_count <= press_count + 1'b1;
        end
    end

    //==========================================================================
    // PWM 呼吸LED（RGB0_Green）
    //==========================================================================
    pwm_breather #(
        .CLK_HZ(CLK_HZ),
        .PWM_RES(PWM_RES)
    ) u_breather (
        .clk(CLK100MHZ),
        .rst_n(rst_n),
        .speed_sel(SW[3:1]),       // SWで呼吸速度を変更
        .pwm_out(pwm_out)
    );

    //==========================================================================
    // UART 送信（1秒ごとにカウンタ値を送信）
    //==========================================================================
    uart_tx #(
        .CLK_HZ(CLK_HZ),
        .BAUD_RATE(UART_BAUD)
    ) u_uart_tx (
        .clk(CLK100MHZ),
        .rst_n(rst_n),
        .tx_data(uart_tx_data),
        .tx_valid(uart_tx_valid),
        .tx_busy(uart_tx_busy),
        .txd(UART_TXD)
    );

    // 1秒タイマーとUART送信トリガ
    logic [$clog2(CLK_HZ)-1:0] sec_counter;
    logic [7:0] hex_char;

    // 4bit値をASCII HEX文字に変換
    always_comb begin
        case (press_count)
            4'h0: hex_char = "0";
            4'h1: hex_char = "1";
            4'h2: hex_char = "2";
            4'h3: hex_char = "3";
            4'h4: hex_char = "4";
            4'h5: hex_char = "5";
            4'h6: hex_char = "6";
            4'h7: hex_char = "7";
            4'h8: hex_char = "8";
            4'h9: hex_char = "9";
            4'hA: hex_char = "A";
            4'hB: hex_char = "B";
            4'hC: hex_char = "C";
            4'hD: hex_char = "D";
            4'hE: hex_char = "E";
            4'hF: hex_char = "F";
        endcase
    end

    always_ff @(posedge CLK100MHZ or negedge rst_n) begin
        if (!rst_n) begin
            sec_counter  <= '0;
            uart_tx_valid <= 1'b0;
        end else begin
            uart_tx_valid <= 1'b0;  // デフォルト
            if (sec_counter < CLK_HZ - 1) begin
                sec_counter <= sec_counter + 1'b1;
            end else begin
                sec_counter <= '0;
                if (!uart_tx_busy) begin
                    uart_tx_data  <= hex_char;
                    uart_tx_valid <= 1'b1;
                end
            end
        end
    end

    //==========================================================================
    // LED出力マッピング
    //==========================================================================
    // SW[0]でモード切替
    // SW[0]=0: PWM呼吸LEDをRGB0_Greenに出力、LED[3:0]は回転するパターン
    // SW[0]=1: ボタン押下回数をLED[3:0]に表示、RGB0は固定色

    logic [3:0] led_pattern;
    logic [$clog2(CLK_HZ/4)-1:0] led_rot_cnt;

    always_ff @(posedge CLK100MHZ or negedge rst_n) begin
        if (!rst_n) begin
            led_rot_cnt <= '0;
            led_pattern <= 4'b0001;
        end else begin
            if (led_rot_cnt < (CLK_HZ / 4) - 1) begin
                led_rot_cnt <= led_rot_cnt + 1'b1;
            end else begin
                led_rot_cnt <= '0;
                led_pattern <= {led_pattern[2:0], led_pattern[3]};
            end
        end
    end

    assign LED = SW[0] ? press_count : led_pattern;

    // RGB LED制御
    assign RGB0_Red   = SW[0] ? 1'b1 : 1'b0;           // モード1で赤点灯
    assign RGB0_Green = SW[0] ? 1'b0 : pwm_out;        // モード0で呼吸
    assign RGB0_Blue  = 1'b0;

    // 未使用RGB LEDは消灯
    assign RGB1_Red = 1'b0;
    assign RGB1_Green = 1'b0;
    assign RGB1_Blue = 1'b0;
    assign RGB2_Red = 1'b0;
    assign RGB2_Green = 1'b0;
    assign RGB2_Blue = 1'b0;
    assign RGB3_Red = 1'b0;
    assign RGB3_Green = 1'b0;
    assign RGB3_Blue = 1'b0;

endmodule
