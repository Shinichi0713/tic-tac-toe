/**
 * @file ik_table_leveling.c
 * @brief IMUオイラー角を用いた4脚テーブル水平制御逆運動学
 * 
 * 4本のシリンダで支持されたテーブルの姿勢を、IMUから得たロール・ピッチ・ヨー情報に基づき
 * 水平に保つための各シリンダの目標長さを計算します。
 * 
 * 機構仮定:
 * - 上下取り付け点は球面関節（自由回転）
 * - ベース座標系 {B}: Z軸上向き、XY水平面
 * - テーブル座標系 {T}: 原点はテーブル中心、XYはテーブル面
 * - 回転順序: ZYX (yaw -> pitch -> roll)
 */

#include <stdio.h>
#include <math.h>

/* ============================================================
 * ユーザ設定パラメータ（実機の寸法に合わせて変更してください）
 * ============================================================ */

/** テーブル上のシリンダ取り付け点（テーブル座標系、単位: m）
 *  インデックス0~3がそれぞれ脚1~4に対応。
 *  以下は長方形テーブル、中心原点の例。 */
#define T_X1  0.30f   /* 脚1,2のX座標（長手方向片側） */
#define T_X2 -0.30f   /* 脚3,4のX座標（長手方向反対側） */
#define T_Y1  0.20f   /* 脚2,3のY座標（幅方向片側） */
#define T_Y2 -0.20f   /* 脚1,4のY座標（幅方向反対側） */

/** ベース上のシリンダ下端取り付け点（ベース座標系、単位: m）
 *  デフォルトではテーブル取り付け点の真下に配置する例。 */
#define B_X1  0.30f
#define B_X2 -0.30f
#define B_Y1  0.20f
#define B_Y2 -0.20f
#define B_Z  -0.50f   /* ベース取り付け点のZ座標（テーブルより下なら負） */

/** 中立時のテーブル中心Z座標（目標位置の基準） [m] */
#define NEUTRAL_TABLE_Z   0.50f

/** 水平化時のヨー角の扱い
 *  1: 現在のヨー角を維持（水平面上で向いている方向はそのまま）
 *  0: ヨー角を0にする */
#define KEEP_CURRENT_YAW  1

/** 度数法 → ラジアン変換 */
#define DEG2RAD(d) ((d) * (float)M_PI / 180.0f)

/* ============================================================
 * データ構造
 * ============================================================ */

typedef struct {
    float x;
    float y;
    float z;
} Vec3;

typedef struct {
    float m[3][3];
} Mat3;

/* ============================================================
 * 数学ユーティリティ
 * ============================================================ */

static inline Vec3 vec3_add(Vec3 a, Vec3 b) {
    return (Vec3){ a.x + b.x, a.y + b.y, a.z + b.z };
}

static inline Vec3 vec3_sub(Vec3 a, Vec3 b) {
    return (Vec3){ a.x - b.x, a.y - b.y, a.z - b.z };
}

/* 回転行列とベクトルの積 */
static inline Vec3 mat3_mul_vec(const Mat3* R, Vec3 v) {
    return (Vec3){
        R->m[0][0] * v.x + R->m[0][1] * v.y + R->m[0][2] * v.z,
        R->m[1][0] * v.x + R->m[1][1] * v.y + R->m[1][2] * v.z,
        R->m[2][0] * v.x + R->m[2][1] * v.y + R->m[2][2] * v.z
    };
}

static inline float vec3_norm(Vec3 v) {
    return sqrtf(v.x * v.x + v.y * v.y + v.z * v.z);
}

/**
 * @brief オイラー角（ZYX順）から回転行列を生成
 * 
 * 回転順序: Rz(yaw) * Ry(pitch) * Rx(roll)
 * つまり、ベース座標系から見たテーブル座標系の姿勢を表す。
 */
static void eulerZYX_to_rotmat(float roll, float pitch, float yaw, Mat3* R) {
    float cr = cosf(roll);
    float sr = sinf(roll);
    float cp = cosf(pitch);
    float sp = sinf(pitch);
    float cy = cosf(yaw);
    float sy = sinf(yaw);

    R->m[0][0] = cy * cp;
    R->m[0][1] = cy * sp * sr - sy * cr;
    R->m[0][2] = cy * sp * cr + sy * sr;

    R->m[1][0] = sy * cp;
    R->m[1][1] = sy * sp * sr + cy * cr;
    R->m[1][2] = sy * sp * cr - cy * sr;

    R->m[2][0] = -sp;
    R->m[2][1] = cp * sr;
    R->m[2][2] = cp * cr;
}

/* ============================================================
 * 機構定義（取り付け点座標）
 * ============================================================ */

/* テーブル上の取り付け点（テーブル座標系） */
static const Vec3 TABLE_PTS[4] = {
    { T_X1, T_Y2, 0.0f },   /* 脚1 */
    { T_X1, T_Y1, 0.0f },   /* 脚2 */
    { T_X2, T_Y1, 0.0f },   /* 脚3 */
    { T_X2, T_Y2, 0.0f }    /* 脚4 */
};

/* ベース上の取り付け点（ベース座標系） */
static const Vec3 BASE_PTS[4] = {
    { B_X1, B_Y2, B_Z },    /* 脚1 */
    { B_X1, B_Y1, B_Z },    /* 脚2 */
    { B_X2, B_Y1, B_Z },    /* 脚3 */
    { B_X2, B_Y2, B_Z }     /* 脚4 */
};

/* ============================================================
 * 逆運動学（IK）関数
 * ============================================================ */

/**
 * @brief 与えられたテーブル位置・姿勢に対するシリンダ長さを計算
 * @param pos   テーブル中心位置（ベース座標系）[m]
 * @param roll  ロール角 [rad]
 * @param pitch ピッチ角 [rad]
 * @param yaw   ヨー角 [rad]
 * @param L     出力バッファ（4要素）、各シリンダ長さ [m]
 */
void compute_lengths(const Vec3* pos, float roll, float pitch, float yaw, float L[4]) {
    Mat3 R;
    eulerZYX_to_rotmat(roll, pitch, yaw, &R);

    for (int i = 0; i < 4; ++i) {
        /* テーブル上点のベース座標系での位置 */
        Vec3 p = vec3_add(*pos, mat3_mul_vec(&R, TABLE_PTS[i]));
        /* シリンダ軸ベクトル（下端→上端） */
        Vec3 v = vec3_sub(p, BASE_PTS[i]);
        /* 長さ = ノルム */
        L[i] = vec3_norm(v);
    }
}

/**
 * @brief 現在姿勢から「水平化」した際の目標シリンダ長さを計算
 * 
 * 方針:
 * - 目標ロール = 0, 目標ピッチ = 0（水平面に平行）
 * - 目標ヨー   = KEEP_CURRENT_YAW ? 現在のヨー : 0
 * - テーブル中心のXYは現在位置を維持
 * - Zは auto_adjust_z により調整可能
 *
 * @param cur_roll  現在ロール [rad]
 * @param cur_pitch 現在ピッチ [rad]
 * @param cur_yaw   現在ヨー [rad]
 * @param cur_pos   現在のテーブル中心位置（ベース座標系）[m]
 * @param L_target  出力バッファ（4要素）目標長さ [m]
 * @param auto_adjust_z 
 *   0: Zを現在値で固定
 *   1: 水平化前後でシリンダ平均長さが大きく変わらないようZを自動調整
 */
void compute_leveling_lengths(float cur_roll, float cur_pitch, float cur_yaw,
                              const Vec3* cur_pos,
                              float L_target[4],
                              int auto_adjust_z) {
    float tgt_roll  = 0.0f;
    float tgt_pitch = 0.0f;
    float tgt_yaw   = KEEP_CURRENT_YAW ? cur_yaw : 0.0f;

    Vec3 tgt_pos = *cur_pos;  /* XYは維持 */

    if (!auto_adjust_z) {
        /* Zを現在値で固定して計算 */
        compute_lengths(&tgt_pos, tgt_roll, tgt_pitch, tgt_yaw, L_target);
    } else {
        /* Z自動調整: 中立Zで一旦計算し、現在姿勢との平均長さ差分をZにフィードバック */
        tgt_pos.z = NEUTRAL_TABLE_Z;
        float L_tmp[4];
        compute_lengths(&tgt_pos, tgt_roll, tgt_pitch, tgt_yaw, L_tmp);

        float L_cur[4];
        compute_lengths(cur_pos, cur_roll, cur_pitch, cur_yaw, L_cur);

        float avg_cur = 0.0f, avg_tmp = 0.0f;
        for (int i = 0; i < 4; ++i) {
            avg_cur += L_cur[i];
            avg_tmp += L_tmp[i];
        }
        avg_cur *= 0.25f;
        avg_tmp *= 0.25f;

        /* シリンダが垂直に近い場合、Zの変化は平均長さ変化にほぼ等しい */
        tgt_pos.z = NEUTRAL_TABLE_Z + (avg_cur - avg_tmp);
        compute_lengths(&tgt_pos, tgt_roll, tgt_pitch, tgt_yaw, L_target);
    }
}

/* ============================================================
 * 使用例 / 簡テスト
 * ============================================================ */

int main(void) {
    /* 例: IMUから得た現在姿勢（度数法） */
    float roll_deg  = 5.0f;   /* ロール 5° */
    float pitch_deg = -3.0f;  /* ピッチ -3° */
    float yaw_deg   = 10.0f;  /* ヨー 10° */

    float roll  = DEG2RAD(roll_deg);
    float pitch = DEG2RAD(pitch_deg);
    float yaw   = DEG2RAD(yaw_deg);

    /* 現在のテーブル中心位置（例として中立位置） */
    Vec3 cur_pos = { 0.0f, 0.0f, NEUTRAL_TABLE_Z };

    float L[4];

    printf("========================================\n");
    printf("  4脚テーブル水平制御 逆運動学\n");
    printf("========================================\n\n");
    printf("機構パラメータ:\n");
    printf("  テーブル上点: [%.2f,%.2f,0] [%.2f,%.2f,0] [%.2f,%.2f,0] [%.2f,%.2f,0]\n",
           T_X1, T_Y2, T_X1, T_Y1, T_X2, T_Y1, T_X2, T_Y2);
    printf("  ベース下点 : [%.2f,%.2f,%.2f] ... Z=%.2f\n", B_X1, B_Y2, B_Z, B_Z);
    printf("  中立テーブルZ: %.2f m\n\n", NEUTRAL_TABLE_Z);

    printf("現在姿勢: roll=%.2f°, pitch=%.2f°, yaw=%.2f°\n", roll_deg, pitch_deg, yaw_deg);
    printf("現在位置: x=%.2f, y=%.2f, z=%.2f\n\n", cur_pos.x, cur_pos.y, cur_pos.z);

    /* --- 現在姿勢での長さ --- */
    compute_lengths(&cur_pos, roll, pitch, yaw, L);
    printf("【現在姿勢でのシリンダ長さ】\n");
    for (int i = 0; i < 4; ++i) {
        printf("  Cylinder %d: %.4f m\n", i + 1, L[i]);
    }

    /* --- 水平化目（Z固定） --- */
    printf("\n【水平化目標長さ (Z固定)】\n");
    compute_leveling_lengths(roll, pitch, yaw, &cur_pos, L, 0);
    for (int i = 0; i < 4; ++i) {
        printf("  Cylinder %d: %.4f m\n", i + 1, L[i]);
    }

    /* --- 水平化目標（Z自動調整） --- */
    printf("\n【水平化目標長さ (Z自動調整)】\n");
    compute_leveling_lengths(roll, pitch, yaw, &cur_pos, L, 1);
    for (int i = 0; i < 4; ++i) {
        printf("  Cylinder %d: %.4f m\n", i + 1, L[i]);
    }

    printf("\n========================================\n");
    printf("Note: 実機の寸法に合わせてマクロ T_X1,T_Y1,B_Z 等を変更してください。\n");
    printf("========================================\n");

    return 0;
}
