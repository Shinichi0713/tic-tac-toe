// debounce_button.sv
// プルアップ/プルダウン入力に対応したボタンチャタリング除去回路
// 押した回数をLEDの点灯パターンで表示する

module debounce_button #(
    parameter CLK_HZ = 50_000_000,      // クロック周波数（デフォルト50MHz）
    parameter DEBOUNCE_MS = 20          // 除去時間20ms
)(
    input  logic clk,                   // システムクロック
    input  logic rst_n,                 // 負論理リセット（LOWでリセット）
    input  logic button,                // ボタン入力（プルアップなら押すとLOW）
    output logic [3:0] led              // LED出力（押した回数を表示）
);

    // 20ms分のクロックカウント数を計算
    localparam COUNTER_MAX = (CLK_HZ / 1000) * DEBOUNCE_MS;
    localparam COUNTER_W = $clog2(COUNTER_MAX);

    logic [COUNTER_W-1:0] timer;
    logic button_sync1, button_sync2; // メタステーブル防止用2段FF
    logic button_stable;                // 安定したボタン状態
    logic button_prev;                  // 1クロック前の状態
    logic [3:0] press_count;            // 押下回数カウンタ

    // 入力の同期化（外部非同期信号をクロック域に持ち込む）
    always_ff @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            button_sync1 <= 1'b1;       // プルアップ前提で初期値HIGH
            button_sync2 <= 1'b1;
        end else begin
            button_sync1 <= button;
            button_sync2 <= button_sync1;
        end
    end

    // チャタリング除去ロジック
    always_ff @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            timer <= '0;
            button_stable <= 1'b1;
        end else begin
            if (button_sync2 != button_stable) begin
                // 状態が変わったらタイマー開始
                timer <= '0;
            end else if (timer < COUNTER_MAX) begin
                // 変化が続く間はカウント
                timer <= timer + 1'b1;
            end else begin
                // 20ms安定したら状態を更新
                button_stable <= button_sync2;
            end
        end
    end

    // 立ち下がりエッジ検出（押された瞬間を1回だけ検出）
    always_ff @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            button_prev <= 1'b1;
            press_count <= '0;
        end else begin
            button_prev <= button_stable;
            // 前回HIGHで今回LOW → プルアップ構成でボタンが押された
            if (button_prev && !button_stable) begin
                press_count <= press_count + 1'b1;
            end
        end
    end

    assign led = press_count;

endmoduleaa


// pwm_fader.sv
// PWMによってLEDの明るさを滑らかに変化させる「ブリージングLED」

module pwm_fader #(
    parameter CLK_HZ = 50_000_000,      // 50MHz
    parameter PWM_RES  = 8,              // PWM分解能 8bit (0〜255)
    parameter FADE_MS  = 10              // 明るさ変化の間隔
)(
    input  logic clk,
    input  logic rst_n,
    output logic led
);

    localparam PWM_MAX = (1 << PWM_RES) - 1;  // 255
    localparam CLK_PER_MS = CLK_HZ / 1000;
    localparam FADE_MAX = CLK_PER_MS * FADE_MS;
    localparam FADE_W = $clog2(FADE_MAX);

    logic [PWM_RES-1:0] pwm_counter;    // PWM用高速カウンタ
    logic [PWM_RES-1:0] brightness;     // 現在の明るさ設定値
    logic [FADE_W-1:0]  fade_timer;     // フェード間隔タイマー
    logic               fade_direction; // 1:明るく 0:暗く

    // PWMカウンタ（高速に0〜255を繰り返し）
    always_ff @(posedge clk or negedge rst_n) begin
        if (!rst_n)
            pwm_counter <= '0;
        else
            pwm_counter <= pwm_counter + 1'b1;
    end

    // PWM比較：brightnessよりカウンタが小さい間だけLED点灯
    assign led = (pwm_counter < brightness);

    // 明るさ値をゆっくり変化させる
    always_ff @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            fade_timer <= '0;
            brightness <= '0;
            fade_direction <= 1'b1;
        end else begin
            if (fade_timer < FADE_MAX) begin
                fade_timer <= fade_timer + 1'b1;
            end else begin
                fade_timer <= '0;
                if (fade_direction) begin
                    if (brightness < PWM_MAX)
                        brightness <= brightness + 1'b1;
                    else
                        fade_direction <= 1'b0;
                end else begin
                    if (brightness > '0)
                        brightness <= brightness - 1'b1;
                    else
                        fade_direction <= 1'b1;
                end
            end
        end
    end

endmodule