#include "pico/stdlib.h"

#define PULSE_PIN 0   // GPIO0からパルス出力

int main() {
    stdio_init_all();
    
    gpio_init(PULSE_PIN);
    gpio_set_dir(PULSE_PIN, GPIO_OUT);

    uint32_t period_us = 1000;   // 周期 1000us = 1kHz
    uint32_t high_us = 500;      // Highの時間 500us = 50%デューティ

    while (true) {
        // High期間
        gpio_put(PULSE_PIN, 1);
        busy_wait_us_32(high_us);  // 正確なマイクロ秒待機
        
        // Low期間
        gpio_put(PULSE_PIN, 0);
        busy_wait_us_32(period_us - high_us);
    }
    
    return 0;
}