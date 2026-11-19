
"""
ik_table_leveling.py
IMUオイラー角を用いた4脚テーブル水平制御逆運動学（Python版）

【機構パラメータ】
  ・シリンダ間隔: 幅 300 mm, 奥行き 270 mm
  ・中立時シリンダ長さ: 290 mm

【必要なセンサ】
  ・パターンA（推奨）: テーブルIMU + ベースIMU（床傾き対応）
  ・パターンB         : テーブルIMU のみ（床水平と仮定）
"""

import numpy as np
from typing import Tuple

# ============================================================
# 機構パラメータ（単位: m）
# ============================================================

# テーブル上のシンダ取り付け点（テーブル座標系、原点=テーブル中心）
# 奥行き 270 mm → X = ±0.135 m
# 幅     300 mm → Y = ±0.150 m
# Z = 0（テーブル面をXY平面とする）
TABLE_PTS = np.array([
    [ 0.135, -0.150, 0.0],   # 脚1
    [ 0.135,  0.150, 0.0],   # 脚2
    [-0.135,  0.150, 0.0],   # 脚3
    [-0.135, -0.150, 0.0],   # 脚4
])

# ベース上のシリンダ下端取り付け点（ベース座標系）
# 中立時シリンダ長さ 290 mm → 下端Z = -0.290 m（上端Z=0と仮定）
BASE_PTS = np.array([
    [ 0.135, -0.150, -0.290],   # 脚1
    [ 0.135,  0.150, -0.290],   # 脚2
    [-0.135,  0.150, -0.290],   # 脚3
    [-0.135, -0.150, -0.290],   # 脚4
])

# 中立時のテーブル中心Z座標 [m]
NEUTRAL_TABLE_Z = 0.0

# 水平化時のヨー角の扱い
# True : 現在のヨー角を維持
# False: ヨー角を0にする
KEEP_CURRENT_YAW = True


def eulerZYX_to_rotmat(roll: float, pitch: float, yaw: float) -> np.ndarray:
    """
    オイラー角（ZYX順: yaw→pitch→roll）から回転行列を生成。

    Parameters
    ----------
    roll, pitch, yaw : float
        各軸回りの回転角 [rad]

    Returns
    -------
    R : np.ndarray, shape (3, 3)
        回転行列（慣性座標系 → 物体座標系）
    """
    cr, sr = np.cos(roll),  np.sin(roll)
    cp, sp = np.cos(pitch), np.sin(pitch)
    cy, sy = np.cos(yaw),   np.sin(yaw)

    R = np.array([
        [cy*cp,  cy*sp*sr - sy*cr,  cy*sp*cr + sy*sr],
        [sy*cp,  sy*sp*sr + cy*cr,  sy*sp*cr - cy*sr],
        [-sp,    cp*sr,             cp*cr           ]
    ])
    return R


def compute_lengths(pos: np.ndarray, R_table_base: np.ndarray) -> np.ndarray:
    """
    テーブル位置・姿勢（ベース座標系表現）から各シリンダ長さを計算。

    Parameters
    ----------
    pos : np.ndarray, shape (3,)
        テーブル中心位置（ベース座標系）[m]
    R_table_base : np.ndarray, shape (3, 3)
        ベース座標系から見たテーブル座標系の回転行列

    Returns
    -------
    L : np.ndarray, shape (4,)
        各シリンダ長さ [m]
    """
    # テーブル上各点のベース座標系での位置
    table_pts_world = pos + (R_table_base @ TABLE_PTS.T).T

    # シリンダ軸ベクトル（下端→上端）とその長さ
    vectors = table_pts_world - BASE_PTS
    L = np.linalg.norm(vectors, axis=1)
    return L


def compute_leveling_with_base_imu(
    base_roll: float, base_pitch: float, base_yaw: float,
    table_roll: float, table_pitch: float, table_yaw: float,
    cur_pos: np.ndarray,
    keep_yaw: bool = KEEP_CURRENT_YAW,
    neutral_z: float = NEUTRAL_TABLE_Z
) -> np.ndarray:
    """
    ベースIMUとテーブルIMUの両方を使って、水平化目標シリンダ長さを計算。

    目標は「テーブルが重力（慣性）座標系で水平」なので、
    目標のテーブル→ベース相対回転は:
        R_target_table_base = R_base_inertial^T @ R_target_inertial
    ただし R_target_inertial = I（水平）とする。

    Parameters
    ----------
    base_roll, base_pitch, base_yaw : float
        ベースIMUのオイラー角 [rad]
    table_roll, table_pitch, table_yaw : float
        テーブルIMUのオイラー角 [rad]
    cur_pos : np.ndarray, shape (3,)
        現在のテーブル中心位置（ベース座標系）[m]
    keep_yaw : bool
        True の場合、現在のテーブルヨー角を維持
    neutral_z : float
        中立時のテーブル中心Z座標 [m]

    Returns
    -------
    L_target : np.ndarray, shape (4,)
        目標シリンダ長さ [m]
    """
    # ベースの慣性系からの回転
    R_base_inertial = eulerZYX_to_rotmat(base_roll, base_pitch, base_yaw)

    # 目標テーブル姿勢（水平）の慣性系からの回転
    target_roll  = 0.0
    target_pitch = 0.0
    target_yaw   = table_yaw if keep_yaw else 0.0
    R_target_inertial = eulerZYX_to_rotmat(target_roll, target_pitch, target_yaw)

    # R_target_table_base = R_base_inertial^T @ R_target_inertial
    R_target_table_base = R_base_inertial.T @ R_target_inertial

    # --- Z自動調整 ---
    # 中立Zで一旦計算
    pos_tmp = cur_pos.copy()
    pos_tmp[2] = neutral_z
    L_tmp = compute_lengths(pos_tmp, R_target_table_base)

    # 現在姿勢での長さ（比較用）
    R_table_inertial = eulerZYX_to_rotmat(table_roll, table_pitch, table_yaw)
    R_cur_table_base = R_base_inertial.T @ R_table_inertial
    L_cur = compute_lengths(cur_pos, R_cur_table_base)

    # 平均長さの差分でZを補正
    avg_cur = np.mean(L_cur)
    avg_tmp = np.mean(L_tmp)
    pos_target = cur_pos.copy()
    pos_target[2] = neutral_z + (avg_cur - avg_tmp)

    L_target = compute_lengths(pos_target, R_target_table_base)
    return L_target


def compute_leveling_no_base_imu(
    table_roll: float, table_pitch: float, table_yaw: float,
    cur_pos: np.ndarray,
    keep_yaw: bool = KEEP_CURRENT_YAW,
    neutral_z: float = NEUTRAL_TABLE_Z
) -> np.ndarray:
    """
    テーブルIMUのみを使って水平化目標シリンダ長さを算（床水平と仮定）。

    ベース座標系を慣性座標系と同一視する（R_base_inertial = I）。
    そのため、R_table_base = R_table_inertial（テーブルIMU値そのまま）。

    Parameters
    ----------
    table_roll, table_pitch, table_yaw : float
        テーブルIMUのオイラー角 [rad]
    cur_pos : np.ndarray, shape (3,)
        現在のテーブル中心位置（ベース座標系）[m]
    keep_yaw : bool
        True の場合、現在のテーブルヨー角を維持
    neutral_z : float
        中立時のテーブル中心Z座標 [m]

    Returns
    -------
    L_target : np.ndarray, shape (4,)
        目標シリンダ長さ [m]
    """
    # ベースIMUなし → ベースは水平と仮定
    return compute_leveling_with_base_imu(
        0.0, 0.0, 0.0,
        table_roll, table_pitch, table_yaw,
        cur_pos,
        keep_yaw=keep_yaw,
        neutral_z=neutral_z
    )


# ============================================================
# 使用例 / テスト
# ============================================================
if __name__ == "__main__":
    import math

    # テーブルIMUの現在値（例: ロール5度, ピッチ-3度, ヨー10度）
    table_roll  = math.radians(5.0)
    table_pitch = math.radians(-3.0)
    table_yaw   = math.radians(10.0)

    # 現在のテーブル中心位置（中立位置）
    cur_pos = np.array([0.0, 0.0, NEUTRAL_TABLE_Z])

    print("=" * 50)
    print("  4脚テーブル水平制御 逆運動学（Python版）")
    print("=" * 50)
    print("\n機構パラメータ:")
    print("  シリンダ間隔: 幅 300 mm, 奥行き 270 mm")
    print("  中立時シリンダ長さ: 290 mm")
    print("\n現在テーブル姿勢:")
    print("  roll  = %.1f deg" % math.degrees(table_roll))
    print("  pitch = %.1f deg" % math.degrees(table_pitch))
    print("  yaw   = %.1f deg" % math.degrees(table_yaw))

    # --- パターンA: ベースIMUあり ---
    print("\n" + "-" * 50)
    print("【パターンA】ベースIMUあり")
    print("-" * 50)

    # ケース1: 床が水平
    L = compute_leveling_with_base_imu(
        0.0, 0.0, 0.0,
        table_roll, table_pitch, table_yaw,
        cur_pos
    )
    print("\n[ケース1] 床が水平（base_roll=0, base_pitch=0）")
    for i, length in enumerate(L, start=1):
        print("  Cylinder %d: %.2f mm" % (i, length * 1000))

    # ケース2: 床がロール 10度 傾いている
    L = compute_leveling_with_base_imu(
        math.radians(10.0), 0.0, 0.0,
        table_roll, table_pitch, table_yaw,
        cur_pos
    )
    print("\n[ケース2] 床が roll = 10 deg 傾いている")
    for i, length in enumerate(L, start=1):
        print("  Cylinder %d: %.2f mm" % (i, length * 1000))

    # ケース3: が pitch -5度 傾いている
    L = compute_leveling_with_base_imu(
        0.0, math.radians(-5.0), 0.0,
        table_roll, table_pitch, table_yaw,
        cur_pos
    )
    print("\n[ケース3] 床が pitch = -5 deg 傾いている")
    for i, length in enumerate(L, start=1):
        print("  Cylinder %d: %.2f mm" % (i, length * 1000))

    # --- パターンB: ベースIMUなし ---
    print("\n" + "-" * 50)
    print("【パターンB】ベースIMUなし（床水平と仮定）")
    print("-" * 50)
    L = compute_leveling_no_base_imu(
        table_roll, table_pitch, table_yaw,
        cur_pos
    )
    for i, length in enumerate(L, start=1):
        print("  Cylinder %d: %.2f mm" % (i, length * 1000))

    print("\n" + "=" * 50)
    print("Note:")
    print("  ・パターンA ケース1 と パターンB は同じ結果になります。")
    print("  ・床が傾いている場合、ベースIMUがないと真の水平化は不可能です。")
    print("=" * 50)
