#include "pico/stdlib.h"
#include "hardware/adc.h"
#include "hardware/pwm.h"

// --- ピン定義 ---
#define SENSOR_PIN 26       // ADC0（アナログセンサー接続）
#define MOTOR_PIN  15       // PWM出力（モーター駆動用）

// --- パラメータ ---
#define PWM_FREQ      1000  // PWM周波数 1kHz
#define ADC_MAX       4095  // 12ビットADCの最大値
#define DEAD_BAND     200   // モーターが動き始める最小値（ノイズ対策）

int main() {
    stdio_init_all();

    // ========== ADC（センサー）初期化 ==========
    adc_init();
    adc_gpio_init(SENSOR_PIN);      // GPIO26をADCモードに設定
    adc_select_input(0);            // ADC0チャンネルを選択

    // ========== PWM（モーター）初期化 ==========
    gpio_set_function(MOTOR_PIN, GPIO_FUNC_PWM);
    uint slice_num = pwm_gpio_to_slice_num(MOTOR_PIN);

    // システムクロック 150MHz を分周してPWMクロックを作る
    // 目標: PWM_FREQ = 1kHz
    // wrap値を999にする → 1000カウント周期
    // クロック = 1kHz × 1000 = 1MHz
    // 分周比 = 150MHz / 1MHz = 150
    pwm_set_clkdiv(slice_num, 150.0f);
    pwm_set_wrap(slice_num, 999);   // 0〜999の1000段階

    pwm_set_gpio_level(MOTOR_PIN, 0);  // 初期状態: 停止
    pwm_set_enabled(slice_num, true);   // PWM開始

    // ========== メインループ ==========
    while (true) {
        // --- 1. センサー読み取り ---
        uint16_t raw_value = adc_read();  // 0〜4095 の値を取得

        // --- 2. センサー値をPWMデューティ比（0〜1000）に変換 ---
        uint32_t duty = ((uint32_t)raw_value * 1000) / ADC_MAX;

        // --- 3. デッドバンド処理（小さな値は0にする） ---
        if (duty < DEAD_BAND) {
            duty = 0;
        }

        // --- 4. PWM出力を更新（モーター速度制御） ---
        pwm_set_gpio_level(MOTOR_PIN, (uint16_t)duty);

        // --- 5. デバッグ出力（USBシリアル） ---
        printf("ADC: %4d | Duty: %3d%%\n", raw_value, (duty * 100) / 1000);

        // 100ms周期で更新
        sleep_ms(100);
    }

    return 0;
}