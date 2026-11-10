/**
 * @file ik_table_leveling_v2.c
 * @brief IMUオイラー角を用いた4脚テーブル水平制御逆運動学（床傾き対応版）
 *
 * 【問題の整理】
 *   テーブルIMUだけでは「床が傾いているかどうか」がわからないため、
 *   テーブルを「水平（重力座標系で水平面に平行）」に保つための
 *   シリンダ長さを正しく算出するには、床（ベース）の姿勢も必要。
 *
 * 【解法】
 *   パターンA: ベースIMUあり
 *     テーブルIMUとベースIMUの両方から、テーブル座標系→ベース座標系の
 *     相対回転を算出。目標は「テーブルが重力座標系で水平」なので、
 *     目標相対姿勢 = R_base^T * I（水平） を用いてIKを解く。
 *
 *   パターンB: ベースIMUなし（床が水平と仮定）
 *     ベース座標系を重力座標系と同一視。前回版と同じ考え方。
 */

#include <stdio.h>
#include <math.h>

/* ============================================================
 *  ユーザ設定パラメータ（実機の寸法に合わせて変更してください）
 * ============================================================ */
#define T_X1   0.30f
#define T_X2  -0.30f
#define T_Y1   0.20f
#define T_Y2  -0.20f

#define B_X1   0.30f
#define B_X2  -0.30f
#define B_Y1   0.20f
#define B_Y2  -0.20f
#define B_Z   -0.50f

#define NEUTRAL_TABLE_Z  0.50f
#define KEEP_CURRENT_YAW  1

#define DEG2RAD(d) ((d) * (float)M_PI / 180.0f)

/* ============================================================
 *  データ構造
 * ============================================================ */
typedef struct { float x, y, z; } Vec3;

typedef struct {
    float m[3][3];
} Mat3;

/* ============================================================
 *  数学ユーティリティ
 * ============================================================ */
static inline Vec3 vec3_add(Vec3 a, Vec3 b) {
    return (Vec3){ a.x + b.x, a.y + b.y, a.z + b.z };
}

static inline Vec3 vec3_sub(Vec3 a, Vec3 b) {
    return (Vec3){ a.x - b.x, a.y - b.y, a.z - b.z };
}

static inline Vec3 mat3_mul_vec(const Mat3* R, Vec3 v) {
    return (Vec3){
        R->m[0][0]*v.x + R->m[0][1]*v.y + R->m[0][2]*v.z,
        R->m[1][0]*v.x + R->m[1][1]*v.y + R->m[1][2]*v.z,
        R->m[2][0]*v.x + R->m[2][1]*v.y + R->m[2][2]*v.z
    };
}

static inline float vec3_norm(Vec3 v) {
    return sqrtf(v.x*v.x + v.y*v.y + v.z*v.z);
}

/* 回転行列の転置（逆回転 = 逆行列） */
static void mat3_transpose(const Mat3* src, Mat3* dst) {
    for (int i = 0; i < 3; ++i)
        for (int j = 0; j < 3; ++j)
            dst->m[i][j] = src->m[j][i];
}

/* 行列積 C = A * B */
static void mat3_mul(const Mat3* A, const Mat3* B, Mat3* C) {
    for (int i = 0; i < 3; ++i) {
        for (int j = 0; j < 3; ++j) {
            C->m[i][j] = 0.0f;
            for (int k = 0; k < 3; ++k)
                C->m[i][j] += A->m[i][k] * B->m[k][j];
        }
    }
}

/* ZYXオイラー角（roll, pitch, yaw）から回転行列 R_inertial = Rz(yaw)*Ry(pitch)*Rx(roll) */
static void eulerZYX_to_rotmat(float roll, float pitch, float yaw, Mat3* R) {
    float cr = cosf(roll),  sr = sinf(roll);
    float cp = cosf(pitch), sp = sinf(pitch);
    float cy = cosf(yaw),   sy = sinf(yaw);

    R->m[0][0] = cy*cp;
    R->m[0][1] = cy*sp*sr - sy*cr;
    R->m[0][2] = cy*sp*cr + sy*sr;

    R->m[1][0] = sy*cp;
    R->m[1][1] = sy*sp*sr + cy*cr;
    R->m[1][2] = sy*sp*cr - cy*sr;

    R->m[2][0] = -sp;
    R->m[2][1] = cp*sr;
    R->m[2][2] = cp*cr;
}

/* ============================================================
 *  機構定義
 * ============================================================ */
static const Vec3 TABLE_PTS[4] = {
    { T_X1, T_Y2, 0.0f },
    { T_X1, T_Y1, 0.0f },
    { T_X2, T_Y1, 0.0f },
    { T_X2, T_Y2, 0.0f }
};

static const Vec3 BASE_PTS[4] = {
    { B_X1, B_Y2, B_Z },
    { B_X1, B_Y1, B_Z },
    { B_X2, B_Y1, B_Z },
    { B_X2, B_Y2, B_Z }
};

/* ============================================================
 *  コアIK: テーブル位置・姿勢（ベース座標系表現）→ シリンダ長さ
 * ============================================================ */
void compute_lengths(const Vec3* pos, const Mat3* R_table_base, float L[4]) {
    for (int i = 0; i < 4; ++i) {
        Vec3 p = vec3_add(*pos, mat3_mul_vec(R_table_base, TABLE_PTS[i]));
        Vec3 v = vec3_sub(p, BASE_PTS[i]);
        L[i] = vec3_norm(v);
    }
}

/* ============================================================
 *  パターンA: ベースIMUあり（床傾き対応・正確）
 * ============================================================ */

/**
 * @brief ベースIMUとテーブルIMUの両方を使って水平化目標長さを計算
 *
 * @param base_roll, base_pitch, base_yaw  ベースIMUのオイラー角 [rad]
 * @param table_roll, table_pitch, table_yaw テーブルIMUのオイラー角 [rad]
 * @param cur_pos   現在のテーブル中心位置（ベース座標系）[m]
 * @param L_target  出力バッファ（4要素）[m]
 *
 * 【考え方】
 *   目標は「テーブルが重力座標系で水平」＝ テーブルのロール=0, ピッチ=0。
 *   しかしIKに必要なのは「テーブル座標系からベース座標系への回転」。
 *
 *   R_table_base = R_base_inertial^T * R_table_inertial
 *
 *   目標状態では R_table_inertial = I（水平、ヨーは任意）なので、
 *   R_target_table_base = R_base_inertial^T * R_target_inertial
 *
 *   これを用いてシリンダ長さを計算すれば、床が傾いていても正しい長さが得られる。
 */
void compute_leveling_with_base_imu(
    float base_roll, float base_pitch, float base_yaw,
    float table_roll, float table_pitch, float table_yaw,
    const Vec3* cur_pos,
    float L_target[4])
{
    /* ベースの慣性系からの回転 */
    Mat3 R_base_inertial;
    eulerZYX_to_rotmat(base_roll, base_pitch, base_yaw, &R_base_inertial);

    /* 目標テーブル姿勢（水平）の慣性系からの回転 */
    float target_roll  = 0.0f;
    float target_pitch = 0.0f;
    float target_yaw   = KEEP_CURRENT_YAW ? table_yaw : 0.0f;

    Mat3 R_target_inertial;
    eulerZYX_to_rotmat(target_roll, target_pitch, target_yaw, &R_target_inertial);

    /* R_target_table_base = R_base_inertial^T * R_target_inertial */
    Mat3 R_base_inv;
    mat3_transpose(&R_base_inertial, &R_base_inv);

    Mat3 R_target_table_base;
    mat3_mul(&R_base_inv, &R_target_inertial, &R_target_table_base);

    /* Z自動調整: 中立Zで計算 → 現在平均長さとの差分で補正 */
    Vec3 pos = *cur_pos;
    pos.z = NEUTRAL_TABLE_Z;

    float L_tmp[4];
    compute_lengths(&pos, &R_target_table_base, L_tmp);

    /* 現在姿勢の長さ（比較用） */
    Mat3 R_table_inertial;
    eulerZYX_to_rotmat(table_roll, table_pitch, table_yaw, &R_table_inertial);
    Mat3 R_cur_table_base;
    mat3_mul(&R_base_inv, &R_table_inertial, &R_cur_table_base);
    float L_cur[4];
    compute_lengths(cur_pos, &R_cur_table_base, L_cur);

    float avg_cur = 0.0f, avg_tmp = 0.0f;
    for (int i = 0; i < 4; ++i) {
        avg_cur += L_cur[i];
        avg_tmp += L_tmp[i];
    }
    avg_cur *= 0.25f;
    avg_tmp *= 0.25f;
    pos.z = NEUTRAL_TABLE_Z + (avg_cur - avg_tmp);

    compute_lengths(&pos, &R_target_table_base, L_target);
}

/* ============================================================
 *  パターンB: ベースIMUなし（床水平と仮定）
 * ============================================================ */

/**
 * @brief テーブルIMUのみを使って水平化目標長さを計算（床水平仮定）
 *
 * ベース座標系を重力座標系と同一視する。
 * つまり R_base_inertial = I と仮定するので、
 * R_table_base = R_table_inertial（テーブルIMUの値そのまま）。
 */
void compute_leveling_no_base_imu(
    float table_roll, float table_pitch, float table_yaw,
    const Vec3* cur_pos,
    float L_target[4])
{
    /* ベースが水平と仮定 → base_roll = base_pitch = base_yaw = 0 */
    compute_leveling_with_base_imu(
        0.0f, 0.0f, 0.0f,
        table_roll, table_pitch, table_yaw,
        cur_pos, L_target);
}

/* ============================================================
 *  テスト / 使用例
 * ============================================================ */
int main(void) {
    /* 例: テーブルがロール5°、ピッチ-3°傾いている */
    float table_roll  = DEG2RAD(5.0f);
    float table_pitch = DEG2RAD(-3.0f);
    float table_yaw   = DEG2RAD(10.0f);

    Vec3 cur_pos = { 0.0f, 0.0f, NEUTRAL_TABLE_Z };
    float L[4];

    printf("========================================\n");
    printf("  4脚テーブル水平制御 IK（床傾き対応版）\n");
    printf("========================================\n\n");

    /* --- パターンA: 床が傾いている場合（ベースIMUあり） --- */
    printf("【パターンA】ベースIMUあり（床が傾いている場合）\n");
    printf("--------------------------------------------\n");

    /* ケース1: 床が水平 */
    printf("\n[ケース1] 床が水平（base_roll=0, base_pitch=0）\n");
    compute_leveling_with_base_imu(0.0f, 0.0f, 0.0f,
                                    table_roll, table_pitch, table_yaw,
                                    &cur_pos, L);
    for (int i = 0; i < 4; ++i)
        printf("  Cylinder %d: %.4f m\n", i+1, L[i]);

    /* ケース2: 床がロール10°傾いている */
    printf("\n[ケース2] 床が roll=10° 傾いている\n");
    compute_leveling_with_base_imu(DEG2RAD(10.0f), 0.0f, 0.0f,
                                    table_roll, table_pitch, table_yaw,
                                    &cur_pos, L);
    for (int i = 0; i < 4; ++i)
        printf("  Cylinder %d: %.4f m\n", i+1, L[i]);

    /* ケース3: 床が pitch -5°傾いている */
    printf("\n[ケース3] 床が pitch=-5° 傾いている\n");
    compute_leveling_with_base_imu(0.0f, DEG2RAD(-5.0f), 0.0f,
                                    table_roll, table_pitch, table_yaw,
                                    &cur_pos, L);
    for (int i = 0; i < 4; ++i)
        printf("  Cylinder %d: %.4f m\n", i+1, L[i]);

    /* --- パターンB: ベースIMUなし --- */
    printf("\n【パターンB】ベースIMUなし（床水平と仮定）\n");
    printf("--------------------------------------------\n");
    compute_leveling_no_base_imu(table_roll, table_pitch, table_yaw,
                                  &cur_pos, L);
    for (int i = 0; i < 4; ++i)
        printf("  Cylinder %d: %.4f m\n", i+1, L[i]);

    printf("\n========================================\n");
    printf("Note:\n");
    printf("  ・パターンAのケース1とパターンBは同じ結果になる（確認用）。\n");
    printf("  ・床が傾いている場合、パターンBではテーブルが真に水平にならない。\n");
    printf("  ・実機の寸法はマクロ T_X1, B_Z 等を変更してください。\n");
    printf("========================================\n");

    return 0;
}
