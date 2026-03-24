#!/usr/bin/env python3
"""
RTK測位 簡易統計ツール (テキスト出力のみ、matplotlib不要)

posファイルを読み込み、統計情報を表示する。

使い方:
  python3 tools/rtk_stats.py output/rtk_solution.pos
"""

import sys
import os
import math


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
                    "status": int(cols[8]), "nsat": int(cols[9]), "pdop": float(cols[10]),
                })
            except (ValueError, IndexError):
                continue
    return epochs


# ---------------------------------------------------------------------------
# メイン
# ---------------------------------------------------------------------------

def mean(values):
    return sum(values) / len(values) if values else float("nan")


def stdev(values):
    if len(values) < 2:
        return float("nan")
    m = mean(values)
    return math.sqrt(sum((v - m) ** 2 for v in values) / (len(values) - 1))


def rms(values):
    if not values:
        return float("nan")
    return math.sqrt(sum(v * v for v in values) / len(values))


def percentile(values, p):
    if not values:
        return float("nan")
    s = sorted(values)
    k = (len(s) - 1) * p / 100.0
    f = int(k)
    c = f + 1
    if c >= len(s):
        return s[f]
    return s[f] + (k - f) * (s[c] - s[f])


def main():
    if len(sys.argv) < 2:
        print("使い方: python3 tools/rtk_stats.py <pos_file>")
        sys.exit(1)

    filepath = sys.argv[1]
    if not os.path.isfile(filepath):
        print(f"エラー: ファイルが見つかりません: {filepath}")
        sys.exit(1)

    epochs = read_libgnss_pos(filepath)
    total = len(epochs)

    if total == 0:
        print("エポックが0件です。")
        sys.exit(0)

    n_fix = sum(1 for ep in epochs if ep["status"] == 4)
    n_float = sum(1 for ep in epochs if ep["status"] == 3)
    n_spp = sum(1 for ep in epochs if ep["status"] == 1)

    # 時間範囲
    tow_min = epochs[0]["tow"]
    tow_max = epochs[-1]["tow"]
    duration = tow_max - tow_min

    print("=" * 60)
    print(f"  RTK測位統計: {filepath}")
    print("=" * 60)
    print(f"  GPS Week           : {epochs[0]['week']}")
    print(f"  開始 TOW           : {tow_min:.3f} s")
    print(f"  終了 TOW           : {tow_max:.3f} s")
    print(f"  観測時間           : {duration:.1f} s ({duration / 60:.1f} 分)")
    print(f"  総エポック数       : {total}")
    print()

    # --- ステータス ---
    print("--- ステータス ---")
    print(f"  Fix解              : {n_fix:6d}  ({n_fix / total * 100:5.1f}%)")
    print(f"  Float解            : {n_float:6d}  ({n_float / total * 100:5.1f}%)")
    print(f"  SPP解              : {n_spp:6d}  ({n_spp / total * 100:5.1f}%)")
    other = total - n_fix - n_float - n_spp
    if other > 0:
        print(f"  その他             : {other:6d}  ({other / total * 100:5.1f}%)")
    print()

    # --- TTFF ---
    print("--- 初回Fix (TTFF) ---")
    if n_fix > 0:
        first_fix_tow = None
        for ep in epochs:
            if ep["status"] == 4:
                first_fix_tow = ep["tow"]
                break
        ttff = first_fix_tow - tow_min
        print(f"  初回Fix TOW        : {first_fix_tow:.3f} s")
        print(f"  TTFF               : {ttff:.1f} 秒")
    else:
        print(f"  Fix解なし")
    print()

    # --- 基準位置 & ENU ---
    fix_epochs = [ep for ep in epochs if ep["status"] == 4]
    if fix_epochs:
        ref_lat = mean([ep["lat"] for ep in fix_epochs])
        ref_lon = mean([ep["lon"] for ep in fix_epochs])
        ref_hgt = mean([ep["hgt"] for ep in fix_epochs])
        src_label = "Fix平均"
    else:
        ref_lat = mean([ep["lat"] for ep in epochs])
        ref_lon = mean([ep["lon"] for ep in epochs])
        ref_hgt = mean([ep["hgt"] for ep in epochs])
        src_label = "全エポック平均"

    print(f"--- 基準位置 ({src_label}) ---")
    print(f"  緯度               : {ref_lat:.9f} deg")
    print(f"  経度               : {ref_lon:.9f} deg")
    print(f"  楕円体高           : {ref_hgt:.4f} m")
    print()

    rx, ry, rz = geodetic_to_ecef(ref_lat, ref_lon, ref_hgt)

    # ENU計算 (各ステータス別)
    for label, status_val in [("Fix解", 4), ("Float解", 3), ("全解", None)]:
        subset = [ep for ep in epochs if (status_val is None or ep["status"] == status_val)]
        if not subset:
            continue

        e_list, n_list, u_list = [], [], []
        for ep in subset:
            ex, ey, ez = geodetic_to_ecef(ep["lat"], ep["lon"], ep["hgt"])
            e, n, u = ecef_to_enu(ex - rx, ey - ry, ez - rz, ref_lat, ref_lon)
            e_list.append(e)
            n_list.append(n)
            u_list.append(u)

        h_list = [math.sqrt(e * e + n * n) for e, n in zip(e_list, n_list)]

        print(f"--- 精度統計 ({label}: {len(subset)}エポック) ---")
        print(f"  East   平均±σ     : {mean(e_list) * 1000:+7.2f} ± {stdev(e_list) * 1000:6.2f} mm")
        print(f"  North  平均±σ     : {mean(n_list) * 1000:+7.2f} ± {stdev(n_list) * 1000:6.2f} mm")
        print(f"  Up     平均±σ     : {mean(u_list) * 1000:+7.2f} ± {stdev(u_list) * 1000:6.2f} mm")
        print(f"  水平RMS            : {rms(h_list) * 1000:7.2f} mm")
        print(f"  垂直RMS            : {rms(u_list) * 1000:7.2f} mm")
        print(f"  水平95%            : {percentile(h_list, 95) * 1000:7.2f} mm")
        print(f"  垂直95%            : {percentile([abs(u) for u in u_list], 95) * 1000:7.2f} mm")
        print(f"  水平Max            : {max(h_list) * 1000:7.2f} mm")
        print(f"  垂直Max            : {max(abs(u) for u in u_list) * 1000:7.2f} mm")
        print()

    # --- 衛星数・PDOP統計 ---
    nsats = [ep["nsat"] for ep in epochs]
    pdops = [ep["pdop"] for ep in epochs if ep["pdop"] > 0]

    print("--- 衛星数・PDOP統計 ---")
    print(f"  衛星数 平均        : {mean(nsats):.1f}")
    print(f"  衛星数 最小/最大   : {min(nsats)} / {max(nsats)}")
    if pdops:
        print(f"  PDOP   平均        : {mean(pdops):.2f}")
        print(f"  PDOP   最小/最大   : {min(pdops):.2f} / {max(pdops):.2f}")
    print()

    # --- Fix連続性 ---
    if n_fix > 0:
        print("--- Fix連続性 ---")
        runs = []
        current_run = 0
        for ep in epochs:
            if ep["status"] == 4:
                current_run += 1
            else:
                if current_run > 0:
                    runs.append(current_run)
                current_run = 0
        if current_run > 0:
            runs.append(current_run)

        if runs:
            print(f"  Fix連続区間数      : {len(runs)}")
            print(f"  最長Fix連続        : {max(runs)} エポック")
            print(f"  平均Fix連続        : {mean(runs):.1f} エポック")
            if len(runs) > 1:
                print(f"  最短Fix連続        : {min(runs)} エポック")
        print()

    print("=" * 60)


if __name__ == "__main__":
    main()
