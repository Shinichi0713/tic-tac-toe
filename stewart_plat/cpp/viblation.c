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


#include "pico/stdlib.h"
#include "hardware/adc.h"
#include "hardware/timer.h"
#include <math.h>
#include <stdio.h>

// ==================== 行列演算ライブラリ ====================

#define N 3  // 状態次元 (x, y, theta)
#define M 3  // 観測次元 (x_gps, y_gps, theta_imu)
#define U 2  // 制御入力次元 (v, omega)

// 行列構造体
typedef struct {
    uint8_t rows;
    uint8_t cols;
    float data[16];  // 最大4x4程度を想定
} Matrix;

// 行列初期化
void mat_init(Matrix *m, uint8_t r, uint8_t c, const float *vals) {
    m->rows = r; m->cols = c;
    for (int i = 0; i < r * c; i++) m->data[i] = vals ? vals[i] : 0.0f;
}

// ゼロ行列
void mat_zeros(Matrix *m, uint8_t r, uint8_t c) {
    mat_init(m, r, c, NULL);
}

// 単位行列
void mat_eye(Matrix *m, uint8_t n) {
    mat_zeros(m, n, n);
    for (int i = 0; i < n; i++) m->data[i * n + i] = 1.0f;
}

// 要素アクセス
float* mat_at(Matrix *m, uint8_t r, uint8_t c) {
    return &m->data[r * m->cols + c];
}

// 行列加算 C = A + B
void mat_add(const Matrix *A, const Matrix *B, Matrix *C) {
    for (int i = 0; i < A->rows * A->cols; i++)
        C->data[i] = A->data[i] + B->data[i];
}

// 行列減算 C = A - B
void mat_sub(const Matrix *A, const Matrix *B, Matrix *C) {
    for (int i = 0; i < A->rows * A->cols; i++)
        C->data[i] = A->data[i] - B->data[i];
}

// 行列乗算 C = A * B
void mat_mul(const Matrix *A, const Matrix *B, Matrix *C) {
    mat_zeros(C, A->rows, B->cols);
    for (int i = 0; i < A->rows; i++)
        for (int j = 0; j < B->cols; j++)
            for (int k = 0; k < A->cols; k++)
                *mat_at(C, i, j) += *mat_at((Matrix*)A, i, k) * *mat_at((Matrix*)B, k, j);
}

// 行列転置 B = A^T
void mat_transpose(const Matrix *A, Matrix *B) {
    for (int i = 0; i < A->rows; i++)
        for (int j = 0; j < A->cols; j++)
            *mat_at(B, j, i) = *mat_at((Matrix*)A, i, j);
}

// 3x3行列の逆行列（解析的解法）
int mat_inv_3x3(const Matrix *A, Matrix *Ainv) {
    float a = A->data[0], b = A->data[1], c = A->data[2];
    float d = A->data[3], e = A->data[4], f = A->data[5];
    float g = A->data[6], h = A->data[7], i = A->data[8];

    float det = a*(e*i - f*h) - b*(d*i - f*g) + c*(d*h - e*g);
    if (fabsf(det) < 1e-6f) return -1;  // 特異行列

    float inv_det = 1.0f / det;
    Ainv->data[0] = (e*i - f*h) * inv_det;
    Ainv->data[1] = (c*h - b*i) * inv_det;
    Ainv->data[2] = (b*f - c*e) * inv_det;
    Ainv->data[3] = (f*g - d*i) * inv_det;
    Ainv->data[4] = (a*i - c*g) * inv_det;
    Ainv->data[5] = (c*d - a*f) * inv_det;
    Ainv->data[6] = (d*h - e*g) * inv_det;
    Ainv->data[7] = (b*g - a*h) * inv_det;
    Ainv->data[8] = (a*e - b*d) * inv_det;
    Ainv->rows = 3; Ainv->cols = 3;
    return 0;
}

// ==================== EKF構造体 ====================

typedef struct {
    // 状態 x = [x, y, theta]^T
    Matrix x;
    // 共分散 P (3x3)
    Matrix P;
    // 過程ノイズ Q (3x3)
    Matrix Q;
    // 観測ノイズ R (3x3)
    Matrix R;
    // 前回の時刻 [us]
    uint32_t last_time_us;
} EKF;

// EKF初期化
void ekf_init(EKF *ekf, float x0, float y0, float theta0) {
    float x_init[3] = {x0, y0, theta0};
    mat_init(&ekf->x, 3, 1, x_init);

    // 初期共分散（自信がないので大きめ）
    float P_init[9] = {1,0,0, 0,1,0, 0,0,0.5};
    mat_init(&ekf->P, 3, 3, P_init);

    // 過程ノイズ（運動モデルの不確かさ）
    float Q_init[9] = {0.01f,0,0, 0,0.01f,0, 0,0,0.001f};
    mat_init(&ekf->Q, 3, 3, Q_init);

    // 観測ノイズ（センサーの精度）
    float R_init[9] = {0.05f,0,0, 0,0.05f,0, 0,0,0.02f};
    mat_init(&ekf->R, 3, 3, R_init);

    ekf->last_time_us = time_us_32();
}

// ==================== EKF予測ステップ ====================

void ekf_predict(EKF *ekf, float v, float omega) {
    uint32_t now = time_us_32();
    float dt = (now - ekf->last_time_us) * 1e-6f;  // 秒に変換
    ekf->last_time_us = now;

    float x = ekf->x.data[0];
    float y = ekf->x.data[1];
    float theta = ekf->x.data[2];

    // --- 1. 状態予測 x_pred = f(x, u) ---
    float x_pred_vals[3];
    x_pred_vals[0] = x + v * dt * cosf(theta);
    x_pred_vals[1] = y + v * dt * sinf(theta);
    x_pred_vals[2] = theta + omega * dt;
    Matrix x_pred; mat_init(&x_pred, 3, 1, x_pred_vals);

    // 角度を -pi ~ pi に正規化
    while (x_pred.data[2] > M_PI) x_pred.data[2] -= 2*M_PI;
    while (x_pred.data[2] < -M_PI) x_pred.data[2] += 2*M_PI;

    // --- 2. ヤコビアン F = df/dx ---
    float F_vals[9] = {
        1, 0, -v * dt * sinf(theta),
        0, 1,  v * dt * cosf(theta),
        0, 0,  1
    };
    Matrix F; mat_init(&F, 3, 3, F_vals);

    // --- 3. 共分散予測 P = F*P*F^T + Q ---
    Matrix Ft; mat_zeros(&Ft, 3, 3);
    mat_transpose(&F, &Ft);

    Matrix FP; mat_zeros(&FP, 3, 3);
    mat_mul(&F, &ekf->P, &FP);

    Matrix FPFt; mat_zeros(&FPFt, 3, 3);
    mat_mul(&FP, &Ft, &FPFt);

    mat_add(&FPFt, &ekf->Q, &ekf->P);

    // 状態を更新
    for (int i = 0; i < 3; i++) ekf->x.data[i] = x_pred.data[i];
}

// ==================== EKF更新ステップ ====================

void ekf_update(EKF *ekf, float z_x, float z_y, float z_theta) {
    // --- 観測値ベクトル z ---
    float z_vals[3] = {z_x, z_y, z_theta};
    Matrix z; mat_init(&z, 3, 1, z_vals);

    // --- 観測予測 h(x) = x （直接観測を仮定） ---
    // H = I (3x3 単位行列)
    Matrix H; mat_eye(&H, 3);
    Matrix Ht; mat_zeros(&Ht, 3, 3);
    mat_transpose(&H, &Ht);

    // --- イノベーション y = z - h(x) ---
    Matrix y; mat_zeros(&y, 3, 1);
    mat_sub(&z, &ekf->x, &y);

    // 角度差を正規化
    while (y.data[2] > M_PI) y.data[2] -= 2*M_PI;
    while (y.data[2] < -M_PI) y.data[2] += 2*M_PI;

    // --- S = H*P*H^T + R ---
    Matrix PHt; mat_zeros(&PHt, 3, 3);
    mat_mul(&ekf->P, &Ht, &PHt);

    Matrix S; mat_zeros(&S, 3, 3);
    mat_mul(&H, &PHt, &S);
    mat_add(&S, &ekf->R, &S);

    // --- K = P*H^T * S^-1 ---
    Matrix Sinv; mat_zeros(&Sinv, 3, 3);
    if (mat_inv_3x3(&S, &Sinv) != 0) return;  // 逆行列計算失敗

    Matrix K; mat_zeros(&K, 3, 3);
    mat_mul(&PHt, &Sinv, &K);

    // --- x = x + K*y ---
    Matrix Ky; mat_zeros(&Ky, 3, 1);
    mat_mul(&K, &y, &Ky);

    Matrix x_new; mat_zeros(&x_new, 3, 1);
    mat_add(&ekf->x, &Ky, &x_new);
    for (int i = 0; i < 3; i++) ekf->x.data[i] = x_new.data[i];

    // --- P = (I - K*H)*P ---
    Matrix KH; mat_zeros(&KH, 3, 3);
    mat_mul(&K, &H, &KH);

    Matrix I; mat_eye(&I, 3);
    Matrix I_KH; mat_zeros(&I_KH, 3, 3);
    mat_sub(&I, &KH, &I_KH);

    Matrix P_new; mat_zeros(&P_new, 3, 3);
    mat_mul(&I_KH, &ekf->P, &P_new);
    for (int i = 0; i < 9; i++) ekf->P.data[i] = P_new.data[i];
}

// ==================== センサー読み取り（仮実装） ====================

// 実際のセンサーに応じて書き換えてください
void read_encoder(float *v, float *omega) {
    // 例: ADC2, ADC3から左右エンコーダ相当の値を読む
    adc_select_input(2);
    uint16_t left_raw = adc_read();
    adc_select_input(3);
    uint16_t right_raw = adc_read();

    // 仮の変換式（実際のセンサー特性に合わせて調整）
    float v_left  = (left_raw  / 4095.0f) * 1.0f;   // m/s
    float v_right = (right_raw / 4095.0f) * 1.0f;   // m/s

    *v = (v_left + v_right) / 2.0f;
    *omega = (v_right - v_left) / 0.15f;  // 0.15mはトレッド幅（仮）
}

void read_gps_imu(float *x, float *y, float *theta) {
    // 例: ADC0, ADC1からGPS相当のx, yを読む
    adc_select_input(0);
    uint16_t x_raw = adc_read();
    adc_select_input(1);
    uint16_t y_raw = adc_read();

    // 0〜4095 を -10m〜+10m にマッピング（仮）
    *x = (x_raw / 4095.0f) * 20.0f - 10.0f;
    *y = (y_raw / 4095.0f) * 20.0f - 10.0f;

    // thetaはIMU（例: MPU6050のZ軸ジャイロ積分値）から取得する想定
    // ここでは仮に0とする
    *theta = 0.0f;
}

// ==================== メイン ====================

int main() {
    stdio_init_all();
    adc_init();
    adc_gpio_init(26);  // ADC0
    adc_gpio_init(27);  // ADC1
    adc_gpio_init(28);  // ADC2
    adc_gpio_init(29);  // ADC3

    // EKF初期化（初期位置原点、向き0）
    EKF ekf;
    ekf_init(&ekf, 0.0f, 0.0f, 0.0f);

    float v = 0.0f, omega = 0.0f;
    float z_x = 0.0f, z_y = 0.0f, z_theta = 0.0f;

    while (true) {
        // --- センサー読み取り ---
        read_encoder(&v, &omega);
        read_gps_imu(&z_x, &z_y, &z_theta);

        // --- EKF予測（運動モデルによる推定） ---
        ekf_predict(&ekf, v, omega);

        // --- EKF更新（観測値で補正） ---
        ekf_update(&ekf, z_x, z_y, z_theta);

        // --- 結果出力 ---
        printf("EST: x=%6.3f y=%6.3f th=%6.3f | ", 
               ekf.x.data[0], ekf.x.data[1], ekf.x.data[2]);
        printf("OBS: x=%6.3f y=%6.3f th=%6.3f | ", z_x, z_y, z_theta);
        printf("VEL: v=%5.3f w=%5.3f\n", v, omega);

        sleep_ms(50);  // 20Hzで更新
    }

    return 0;
}