#!/usr/bin/env python3
"""
RTKLIB比較ツール

LibGNSS++とRTKLIBの測位結果を比較し、統計情報と比較プロットを生成する。

使い方:
  python3 tools/compare_rtklib.py output/rtk_solution.pos rtklib.pos
"""

import sys
import os
import math
import numpy as np
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt


# ---------------------------------------------------------------------------
# 座標変換
# ---------------------------------------------------------------------------

def geodetic_to_ecef(lat_deg, lon_deg, h):
    a = 6378137.0
    f = 1.0 / 298.257223563
    e2 = 2 * f - f * f
    lat = math.radians(lat_deg)
    lon = math.radians(lon_deg)
    sin_lat = math.sin(lat)
    cos_lat = math.cos(lat)
    N = a / math.sqrt(1.0 - e2 * sin_lat * sin_lat)
    x = (N + h) * cos_lat * math.cos(lon)
    y = (N + h) * cos_lat * math.sin(lon)
    z = (N * (1.0 - e2) + h) * sin_lat
    return x, y, z


def ecef_to_enu(dx, dy, dz, lat_deg, lon_deg):
    lat = math.radians(lat_deg)
    lon = math.radians(lon_deg)
    sin_lat = math.sin(lat)
    cos_lat = math.cos(lat)
    sin_lon = math.sin(lon)
    cos_lon = math.cos(lon)
    e = -sin_lon * dx + cos_lon * dy
    n = -sin_lat * cos_lon * dx - sin_lat * sin_lon * dy + cos_lat * dz
    u = cos_lat * cos_lon * dx + cos_lat * sin_lon * dy + sin_lat * dz
    return e, n, u


# ---------------------------------------------------------------------------
# ファイル読み込み
# ---------------------------------------------------------------------------

RTKLIB_STATUS_MAP = {1: 4, 2: 3, 5: 1}


def read_libgnss_pos(filepath):
    epochs = []
    with open(filepath) as f:
        for line in f:
            line = line.strip()
            if not line or line.startswith("%"):
                continue
            cols = line.split()
            if len(cols) < 11:
                continue
            try:
                epochs.append({
                    "week": int(cols[0]), "tow": float(cols[1]),
                    "lat": float(cols[5]), "lon": float(cols[6]), "hgt": float(cols[7]),
                    "status": int(cols[8]), "nsat": int(cols[9]),
                })
            except (ValueError, IndexError):
                continue
    return epochs


def read_rtklib_pos(filepath):
    epochs = []
    with open(filepath) as f:
        for line in f:
            line = line.strip()
            if not line or line.startswith("%"):
                continue
            cols = line.split()
            if len(cols) < 6:
                continue
            try:
                q = int(cols[5])
                epochs.append({
                    "week": int(cols[0]), "tow": float(cols[1]),
                    "lat": float(cols[2]), "lon": float(cols[3]), "hgt": float(cols[4]),
                    "status": RTKLIB_STATUS_MAP.get(q, 1),
                    "nsat": int(cols[6]) if len(cols) > 6 else 0,
                })
            except (ValueError, IndexError):
                continue
    return epochs


# ---------------------------------------------------------------------------
# ENU計算
# ---------------------------------------------------------------------------

def compute_enu(epochs, ref_lat, ref_lon, ref_hgt):
    rx, ry, rz = geodetic_to_ecef(ref_lat, ref_lon, ref_hgt)
    tow = np.array([ep["tow"] for ep in epochs])
    status = np.array([ep["status"] for ep in epochs])
    e_arr, n_arr, u_arr = [], [], []
    for ep in epochs:
        ex, ey, ez = geodetic_to_ecef(ep["lat"], ep["lon"], ep["hgt"])
        e, n, u = ecef_to_enu(ex - rx, ey - ry, ez - rz, ref_lat, ref_lon)
        e_arr.append(e)
        n_arr.append(n)
        u_arr.append(u)
    return tow, np.array(e_arr), np.array(n_arr), np.array(u_arr), status


def get_ref_position(epochs):
    """Fix解の平均位置を基準点として返す。Fix解が無ければ全平均。"""
    fix = [ep for ep in epochs if ep["status"] == 4]
    src = fix if fix else epochs
    return (
        np.mean([ep["lat"] for ep in src]),
        np.mean([ep["lon"] for ep in src]),
        np.mean([ep["hgt"] for ep in src]),
    )


# ---------------------------------------------------------------------------
# 統計計算
# ---------------------------------------------------------------------------

def calc_stats(label, epochs, tow, e, n, u, status):
    total = len(epochs)
    n_fix = int(np.sum(status == 4))
    n_float = int(np.sum(status == 3))
    n_spp = int(np.sum(status == 1))
    fix_rate = n_fix / total * 100.0 if total > 0 else 0.0

    # RMS (Fix解のみ)
    fix_mask = status == 4
    if np.any(fix_mask):
        h_rms = np.sqrt(np.mean(e[fix_mask] ** 2 + n[fix_mask] ** 2))
        v_rms = np.sqrt(np.mean(u[fix_mask] ** 2))
    else:
        h_rms = float("nan")
        v_rms = float("nan")

    # TTFF
    ttff = float("nan")
    if n_fix > 0:
        first_fix_tow = tow[np.argmax(status == 4)]
        ttff = first_fix_tow - tow[0]

    print(f"\n===== {label} =====")
    print(f"  総エポック数       : {total}")
    print(f"  Fix解              : {n_fix} ({fix_rate:.1f}%)")
    print(f"  Float解            : {n_float} ({n_float / total * 100:.1f}%)" if total > 0 else "")
    print(f"  SPP解              : {n_spp} ({n_spp / total * 100:.1f}%)" if total > 0 else "")
    print(f"  水平RMS (Fix)      : {h_rms * 1000:.1f} mm")
    print(f"  垂直RMS (Fix)      : {v_rms * 1000:.1f} mm")
    print(f"  初回Fix時間 (TTFF) : {ttff:.1f} 秒")

    return {
        "total": total, "n_fix": n_fix, "fix_rate": fix_rate,
        "h_rms": h_rms, "v_rms": v_rms, "ttff": ttff,
    }


# ---------------------------------------------------------------------------
# エポック毎差分
# ---------------------------------------------------------------------------

def epoch_by_epoch_diff(epochs1, epochs2, ref_lat, ref_lon, ref_hgt):
    """共通エポックのFix解同士の差分を計算"""
    tow_map2 = {}
    for ep in epochs2:
        tow_map2[round(ep["tow"], 3)] = ep

    diffs_e, diffs_n, diffs_u, tows = [], [], [], []
    rx, ry, rz = geodetic_to_ecef(ref_lat, ref_lon, ref_hgt)

    for ep1 in epochs1:
        if ep1["status"] != 4:
            continue
        key = round(ep1["tow"], 3)
        ep2 = tow_map2.get(key)
        if ep2 is None or ep2["status"] != 4:
            continue

        x1, y1, z1 = geodetic_to_ecef(ep1["lat"], ep1["lon"], ep1["hgt"])
        x2, y2, z2 = geodetic_to_ecef(ep2["lat"], ep2["lon"], ep2["hgt"])
        e, n, u = ecef_to_enu(x1 - x2, y1 - y2, z1 - z2, ref_lat, ref_lon)
        diffs_e.append(e)
        diffs_n.append(n)
        diffs_u.append(u)
        tows.append(ep1["tow"])

    return np.array(tows), np.array(diffs_e), np.array(diffs_n), np.array(diffs_u)


# ---------------------------------------------------------------------------
# 比較プロット
# ---------------------------------------------------------------------------

def plot_comparison(epochs1, epochs2, ref_lat, ref_lon, ref_hgt, out_png):
    tow1, e1, n1, u1, st1 = compute_enu(epochs1, ref_lat, ref_lon, ref_hgt)
    tow2, e2, n2, u2, st2 = compute_enu(epochs2, ref_lat, ref_lon, ref_hgt)

    t0 = min(tow1[0], tow2[0])
    t1m = (tow1 - t0) / 60.0
    t2m = (tow2 - t0) / 60.0

    dtow, de, dn, du = epoch_by_epoch_diff(epochs1, epochs2, ref_lat, ref_lon, ref_hgt)

    fig, axes = plt.subplots(4, 1, figsize=(14, 18))
    fig.suptitle("LibGNSS++ vs RTKLIB 比較", fontsize=14)

    # --- EN散布図比較 ---
    ax = axes[0]
    fix1 = st1 == 4
    fix2 = st2 == 4
    if np.any(fix1):
        ax.scatter(e1[fix1] * 1000, n1[fix1] * 1000, c="blue", s=4, alpha=0.5, label="LibGNSS++ Fix")
    if np.any(fix2):
        ax.scatter(e2[fix2] * 1000, n2[fix2] * 1000, c="red", s=4, alpha=0.5, label="RTKLIB Fix")
    ax.set_xlabel("East (mm)")
    ax.set_ylabel("North (mm)")
    ax.set_title("(a) Fix解 水平散布図比較")
    ax.set_aspect("equal")
    ax.legend(fontsize=8)
    ax.grid(True, alpha=0.3)

    # --- ステータス比較 ---
    ax = axes[1]
    s_map = {1: 0, 3: 1, 4: 2}
    colors1 = {"blue": "LibGNSS++"}
    colors2 = {"red": "RTKLIB"}
    for s_val in [1, 3, 4]:
        m1 = st1 == s_val
        m2 = st2 == s_val
        if np.any(m1):
            ax.scatter(t1m[m1], [s_map[s_val] + 0.1] * int(np.sum(m1)), c="blue", s=6, alpha=0.5)
        if np.any(m2):
            ax.scatter(t2m[m2], [s_map[s_val] - 0.1] * int(np.sum(m2)), c="red", s=6, alpha=0.5)
    ax.set_yticks([0, 1, 2])
    ax.set_yticklabels(["SPP", "Float", "Fix"])
    ax.set_title("(b) ステータス比較 (青=LibGNSS++, 赤=RTKLIB)")
    ax.grid(True, alpha=0.3)

    # --- Fix解差分 (E, N) ---
    ax = axes[2]
    if len(dtow) > 0:
        dt_min = (dtow - t0) / 60.0
        ax.plot(dt_min, de * 1000, ".", color="blue", markersize=2, label="East差分")
        ax.plot(dt_min, dn * 1000, ".", color="red", markersize=2, label="North差分")
        ax.axhline(0, color="black", linewidth=0.5)
    ax.set_ylabel("差分 (mm)")
    ax.set_title(f"(c) Fix解エポック毎差分 (共通Fix: {len(dtow)}エポック)")
    ax.legend(fontsize=8)
    ax.grid(True, alpha=0.3)

    # --- Fix解差分 (U) ---
    ax = axes[3]
    if len(dtow) > 0:
        dt_min = (dtow - t0) / 60.0
        ax.plot(dt_min, du * 1000, ".", color="green", markersize=2, label="Up差分")
        ax.axhline(0, color="black", linewidth=0.5)
    ax.set_xlabel("時間 (分)")
    ax.set_ylabel("差分 (mm)")
    ax.set_title("(d) Fix解 高さ差分")
    ax.legend(fontsize=8)
    ax.grid(True, alpha=0.3)

    plt.tight_layout(rect=[0, 0, 1, 0.96])
    plt.savefig(out_png, dpi=150)
    print(f"比較プロット保存: {out_png}")
    try:
        plt.show()
    except Exception:
        pass


# ---------------------------------------------------------------------------
# メイン
# ---------------------------------------------------------------------------

def main():
    if len(sys.argv) < 3:
        print("使い方: python3 tools/compare_rtklib.py <libgnss_pos> <rtklib_pos>")
        sys.exit(1)

    file1, file2 = sys.argv[1], sys.argv[2]

    for f in [file1, file2]:
        if not os.path.isfile(f):
            print(f"エラー: ファイルが見つかりません: {f}")
            sys.exit(1)

    epochs1 = read_libgnss_pos(file1)
    epochs2 = read_rtklib_pos(file2)
    print(f"LibGNSS++: {len(epochs1)} エポック ({file1})")
    print(f"RTKLIB   : {len(epochs2)} エポック ({file2})")

    # 共通基準点
    ref_lat, ref_lon, ref_hgt = get_ref_position(epochs1)
    print(f"\n基準位置 (LibGNSS++ Fix平均):")
    print(f"  緯度: {ref_lat:.9f} deg")
    print(f"  経度: {ref_lon:.9f} deg")
    print(f"  高さ: {ref_hgt:.4f} m")

    tow1, e1, n1, u1, st1 = compute_enu(epochs1, ref_lat, ref_lon, ref_hgt)
    tow2, e2, n2, u2, st2 = compute_enu(epochs2, ref_lat, ref_lon, ref_hgt)

    stats1 = calc_stats("LibGNSS++", epochs1, tow1, e1, n1, u1, st1)
    stats2 = calc_stats("RTKLIB", epochs2, tow2, e2, n2, u2, st2)

    # --- 比較サマリ ---
    print("\n===== 比較サマリ =====")
    print(f"  Fix率差分          : {stats1['fix_rate'] - stats2['fix_rate']:+.1f} pp  "
          f"(LibGNSS++ {stats1['fix_rate']:.1f}% vs RTKLIB {stats2['fix_rate']:.1f}%)")
    if not math.isnan(stats1["h_rms"]) and not math.isnan(stats2["h_rms"]):
        print(f"  水平RMS差分        : {(stats1['h_rms'] - stats2['h_rms']) * 1000:+.1f} mm  "
              f"(LibGNSS++ {stats1['h_rms'] * 1000:.1f} mm vs RTKLIB {stats2['h_rms'] * 1000:.1f} mm)")
        print(f"  垂直RMS差分        : {(stats1['v_rms'] - stats2['v_rms']) * 1000:+.1f} mm  "
              f"(LibGNSS++ {stats1['v_rms'] * 1000:.1f} mm vs RTKLIB {stats2['v_rms'] * 1000:.1f} mm)")
    if not math.isnan(stats1["ttff"]) and not math.isnan(stats2["ttff"]):
        print(f"  TTFF差分           : {stats1['ttff'] - stats2['ttff']:+.1f} 秒  "
              f"(LibGNSS++ {stats1['ttff']:.1f}s vs RTKLIB {stats2['ttff']:.1f}s)")

    # ソリューション可用性 (共通エポック)
    tow_set1 = set(round(ep["tow"], 3) for ep in epochs1)
    tow_set2 = set(round(ep["tow"], 3) for ep in epochs2)
    common = tow_set1 & tow_set2
    only1 = tow_set1 - tow_set2
    only2 = tow_set2 - tow_set1
    print(f"\n  ソリューション可用性:")
    print(f"    共通エポック     : {len(common)}")
    print(f"    LibGNSS++のみ   : {len(only1)}")
    print(f"    RTKLIBのみ      : {len(only2)}")

    # エポック毎差分
    dtow, de, dn, du = epoch_by_epoch_diff(epochs1, epochs2, ref_lat, ref_lon, ref_hgt)
    if len(dtow) > 0:
        print(f"\n  エポック毎差分 (共通Fix解: {len(dtow)}エポック):")
        print(f"    East  平均±σ   : {np.mean(de) * 1000:+.2f} ± {np.std(de) * 1000:.2f} mm")
        print(f"    North 平均±σ   : {np.mean(dn) * 1000:+.2f} ± {np.std(dn) * 1000:.2f} mm")
        print(f"    Up    平均±σ   : {np.mean(du) * 1000:+.2f} ± {np.std(du) * 1000:.2f} mm")
        h_diff = np.sqrt(de ** 2 + dn ** 2)
        print(f"    水平差分RMS     : {np.sqrt(np.mean(h_diff ** 2)) * 1000:.2f} mm")
        print(f"    垂直差分RMS     : {np.sqrt(np.mean(du ** 2)) * 1000:.2f} mm")
    else:
        print("\n  共通Fix解が0件のため、エポック毎差分は計算できません。")

    # プロット
    base_name = os.path.splitext(os.path.basename(file1))[0]
    out_png = os.path.join(os.path.dirname(file1) or ".", f"{base_name}_comparison.png")
    plot_comparison(epochs1, epochs2, ref_lat, ref_lon, ref_hgt, out_png)


if __name__ == "__main__":
    main()
