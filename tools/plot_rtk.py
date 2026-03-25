#!/usr/bin/env python3
"""
RTK測位結果 可視化ツール

posファイルを読み込み、以下のプロットを生成する:
  a. ENU位置時系列（Fix平均位置基準）
  b. Fix/Float/SPPステータスタイムライン
  c. 水平(EN)散布図
  d. 高さ時系列
"""

import sys
import os
import numpy as np
from datetime import datetime
import math


# ---------------------------------------------------------------------------
# 座標変換ユーティリティ
# ---------------------------------------------------------------------------

def geodetic_to_ecef(lat_deg, lon_deg, h):
    """WGS84 geodetic -> ECEF (m)"""
    a = 6378137.0
    f = 1.0 / 298.257223563
    e2 = 2 * f - f * f
    lat = math.radians(lat_deg)
    lon = math.radians(lon_deg)
    sin_lat = math.sin(lat)
    cos_lat = math.cos(lat)
    sin_lon = math.sin(lon)
    cos_lon = math.cos(lon)
    N = a / math.sqrt(1.0 - e2 * sin_lat * sin_lat)
    x = (N + h) * cos_lat * cos_lon
    y = (N + h) * cos_lat * sin_lon
    z = (N * (1.0 - e2) + h) * sin_lat
    return x, y, z


def ecef_to_enu(dx, dy, dz, lat_deg, lon_deg):
    """ECEF差分 -> ENU"""
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

STATUS_COLORS = {1: "red", 3: "orange", 4: "green"}
STATUS_LABELS = {1: "SPP", 3: "Float", 4: "Fix"}

RTKLIB_STATUS_MAP = {1: 4, 2: 3, 5: 1}  # RTKLIB Q -> 内部ステータス


def read_libgnss_pos(filepath):
    """LibGNSS++ posファイルを読み込む"""
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
                week = int(cols[0])
                tow = float(cols[1])
                lat = float(cols[5])
                lon = float(cols[6])
                hgt = float(cols[7])
                status = int(cols[8])
                nsat = int(cols[9])
                pdop = float(cols[10])
                epochs.append({
                    "week": week, "tow": tow,
                    "lat": lat, "lon": lon, "hgt": hgt,
                    "status": status, "nsat": nsat, "pdop": pdop,
                })
            except (ValueError, IndexError):
                continue
    return epochs


def read_rtklib_pos(filepath):
    """RTKLIB posファイルを読み込む"""
    epochs = []
    with open(filepath) as f:
        for line in f:
            line = line.strip()
            if not line or line.startswith("%"):
                continue
            cols = line.split()
            try:
                if len(cols) >= 7 and "/" in cols[0] and ":" in cols[1]:
                    dt = datetime.strptime(f"{cols[0]} {cols[1]}", "%Y/%m/%d %H:%M:%S.%f")
                    gps_epoch = datetime(1980, 1, 6)
                    delta = dt - gps_epoch
                    week = delta.days // 7
                    tow = (delta.days % 7) * 86400 + delta.seconds + delta.microseconds / 1e6
                    lat = float(cols[2])
                    lon = float(cols[3])
                    hgt = float(cols[4])
                    q = int(cols[5])
                    nsat = int(cols[6]) if len(cols) > 6 else 0
                elif len(cols) >= 6:
                    week = int(cols[0])
                    tow = float(cols[1])
                    lat = float(cols[2])
                    lon = float(cols[3])
                    hgt = float(cols[4])
                    q = int(cols[5])
                    nsat = int(cols[6]) if len(cols) > 6 else 0
                else:
                    continue
                status = RTKLIB_STATUS_MAP.get(q, 1)
                epochs.append({
                    "week": week, "tow": tow,
                    "lat": lat, "lon": lon, "hgt": hgt,
                    "status": status, "nsat": nsat, "pdop": 0.0,
                })
            except (ValueError, IndexError):
                continue
    return epochs


# ---------------------------------------------------------------------------
# ENU変換
# ---------------------------------------------------------------------------

def compute_enu(epochs, ref_lat=None, ref_lon=None, ref_hgt=None):
    """epochs -> (tow配列, e配列, n配列, u配列, status配列)  Fix平均基準"""
    fix_lats = [ep["lat"] for ep in epochs if ep["status"] == 4]
    fix_lons = [ep["lon"] for ep in epochs if ep["status"] == 4]
    fix_hgts = [ep["hgt"] for ep in epochs if ep["status"] == 4]

    if ref_lat is None:
        if fix_lats:
            ref_lat = np.mean(fix_lats)
            ref_lon = np.mean(fix_lons)
            ref_hgt = np.mean(fix_hgts)
        else:
            ref_lat = np.mean([ep["lat"] for ep in epochs])
            ref_lon = np.mean([ep["lon"] for ep in epochs])
            ref_hgt = np.mean([ep["hgt"] for ep in epochs])

    rx, ry, rz = geodetic_to_ecef(ref_lat, ref_lon, ref_hgt)

    tow_arr = []
    e_arr, n_arr, u_arr = [], [], []
    s_arr = []
    for ep in epochs:
        ex, ey, ez = geodetic_to_ecef(ep["lat"], ep["lon"], ep["hgt"])
        e, n, u = ecef_to_enu(ex - rx, ey - ry, ez - rz, ref_lat, ref_lon)
        tow_arr.append(ep["tow"])
        e_arr.append(e)
        n_arr.append(n)
        u_arr.append(u)
        s_arr.append(ep["status"])

    return (np.array(tow_arr), np.array(e_arr), np.array(n_arr),
            np.array(u_arr), np.array(s_arr), ref_lat, ref_lon, ref_hgt)


# ---------------------------------------------------------------------------
# プロット
# ---------------------------------------------------------------------------

def plot_single(epochs, title_prefix, out_png):
    """単一posファイルのプロット"""
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt
    from matplotlib.lines import Line2D

    if not epochs:
        print("エポックが0件です。プロットをスキップします。")
        return

    tow, e, n, u, st, rlat, rlon, rhgt = compute_enu(epochs)
    t0 = tow[0]
    t_min = (tow - t0) / 60.0  # 分単位

    fig, axes = plt.subplots(4, 1, figsize=(14, 16), sharex=False)
    fig.suptitle(f"{title_prefix}  RTK測位結果", fontsize=14)

    # --- (a) ENU時系列 ---
    ax = axes[0]
    for label, data, offset in [("East", e, 0), ("North", n, 1), ("Up", u, 2)]:
        for s_val, color in STATUS_COLORS.items():
            mask = st == s_val
            if np.any(mask):
                ax.scatter(t_min[mask], data[mask], c=color, s=4, zorder=3)
        ax.plot(t_min, data, color="grey", linewidth=0.3, alpha=0.5, zorder=1)
    ax.set_ylabel("ENU (m)")
    ax.set_title("(a) ENU位置時系列（Fix平均基準）")
    ax.grid(True, alpha=0.3)
    ax.legend(
        handles=[
            Line2D([0], [0], marker="o", color="w", markerfacecolor="green", label="Fix", markersize=6),
            Line2D([0], [0], marker="o", color="w", markerfacecolor="orange", label="Float", markersize=6),
            Line2D([0], [0], marker="o", color="w", markerfacecolor="red", label="SPP", markersize=6),
        ],
        loc="upper right", fontsize=8,
    )

    # --- (b) ステータスタイムライン ---
    ax = axes[1]
    status_y = {1: 0, 3: 1, 4: 2}
    for s_val, color in STATUS_COLORS.items():
        mask = st == s_val
        if np.any(mask):
            y_val = status_y[s_val]
            ax.scatter(t_min[mask], [y_val] * int(np.sum(mask)), c=color, s=10, zorder=3)
    ax.set_yticks([0, 1, 2])
    ax.set_yticklabels(["SPP", "Float", "Fix"])
    ax.set_title("(b) 測位ステータスタイムライン")
    ax.set_ylabel("ステータス")
    ax.grid(True, alpha=0.3)

    # --- (c) EN散布図 ---
    ax = axes[2]
    for s_val, color in STATUS_COLORS.items():
        mask = st == s_val
        if np.any(mask):
            ax.scatter(e[mask], n[mask], c=color, s=6, label=STATUS_LABELS[s_val], alpha=0.7, zorder=3)
    ax.set_xlabel("East (m)")
    ax.set_ylabel("North (m)")
    ax.set_title("(c) 水平散布図 (EN)")
    ax.set_aspect("equal")
    ax.grid(True, alpha=0.3)
    ax.legend(fontsize=8)

    # --- (d) 高さ時系列 ---
    ax = axes[3]
    for s_val, color in STATUS_COLORS.items():
        mask = st == s_val
        if np.any(mask):
            ax.scatter(t_min[mask], u[mask], c=color, s=6, zorder=3)
    ax.plot(t_min, u, color="grey", linewidth=0.3, alpha=0.5, zorder=1)
    ax.set_xlabel("時間 (分)")
    ax.set_ylabel("Up (m)")
    ax.set_title("(d) 高さ時系列")
    ax.grid(True, alpha=0.3)

    plt.tight_layout(rect=[0, 0, 1, 0.96])
    plt.savefig(out_png, dpi=150)
    print(f"プロット保存: {out_png}")


def plot_comparison(epochs1, epochs2, label1, label2, out_png):
    """2つのposファイルの比較プロット"""
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt

    if not epochs1 or not epochs2:
        print("比較データが不足しています。")
        return

    # 共通基準点: epochs1 の Fix 平均
    tow1, e1, n1, u1, st1, rlat, rlon, rhgt = compute_enu(epochs1)
    tow2, e2, n2, u2, st2, _, _, _ = compute_enu(epochs2, rlat, rlon, rhgt)

    t0 = min(tow1[0], tow2[0])
    t1_min = (tow1 - t0) / 60.0
    t2_min = (tow2 - t0) / 60.0

    fig, axes = plt.subplots(3, 1, figsize=(14, 14))
    fig.suptitle(f"比較: {label1} vs {label2}", fontsize=14)

    # East
    ax = axes[0]
    ax.plot(t1_min, e1, ".", color="blue", markersize=2, label=f"{label1} East", alpha=0.7)
    ax.plot(t2_min, e2, ".", color="red", markersize=2, label=f"{label2} East", alpha=0.7)
    ax.set_ylabel("East (m)")
    ax.set_title("East成分比較")
    ax.legend(fontsize=8)
    ax.grid(True, alpha=0.3)

    # North
    ax = axes[1]
    ax.plot(t1_min, n1, ".", color="blue", markersize=2, label=f"{label1} North", alpha=0.7)
    ax.plot(t2_min, n2, ".", color="red", markersize=2, label=f"{label2} North", alpha=0.7)
    ax.set_ylabel("North (m)")
    ax.set_title("North成分比較")
    ax.legend(fontsize=8)
    ax.grid(True, alpha=0.3)

    # Up
    ax = axes[2]
    ax.plot(t1_min, u1, ".", color="blue", markersize=2, label=f"{label1} Up", alpha=0.7)
    ax.plot(t2_min, u2, ".", color="red", markersize=2, label=f"{label2} Up", alpha=0.7)
    ax.set_xlabel("時間 (分)")
    ax.set_ylabel("Up (m)")
    ax.set_title("Up成分比較")
    ax.legend(fontsize=8)
    ax.grid(True, alpha=0.3)

    plt.tight_layout(rect=[0, 0, 1, 0.96])
    plt.savefig(out_png, dpi=150)
    print(f"比較プロット保存: {out_png}")


# ---------------------------------------------------------------------------
# メイン
# ---------------------------------------------------------------------------

def main():
    program = os.environ.get("GNSS_CLI_NAME", os.path.basename(sys.argv[0]))
    if len(sys.argv) < 2 or sys.argv[1] in ("-h", "--help"):
        print(f"使い方: {program} <pos_file> [rtklib_pos_file]")
        sys.exit(0 if len(sys.argv) >= 2 else 1)

    pos_file = sys.argv[1]
    rtklib_file = sys.argv[2] if len(sys.argv) > 2 else None

    if not os.path.isfile(pos_file):
        print(f"エラー: ファイルが見つかりません: {pos_file}")
        sys.exit(1)

    print(f"読み込み中: {pos_file}")
    epochs1 = read_libgnss_pos(pos_file)
    print(f"  エポック数: {len(epochs1)}")

    base_name = os.path.splitext(os.path.basename(pos_file))[0]
    out_png = os.path.join(os.path.dirname(pos_file) or ".", f"{base_name}_plot.png")

    plot_single(epochs1, "LibGNSS++", out_png)

    if rtklib_file:
        if not os.path.isfile(rtklib_file):
            print(f"エラー: RTKLIBファイルが見つかりません: {rtklib_file}")
            sys.exit(1)
        print(f"読み込み中 (RTKLIB): {rtklib_file}")
        epochs2 = read_rtklib_pos(rtklib_file)
        print(f"  エポック数: {len(epochs2)}")

        cmp_png = os.path.join(os.path.dirname(pos_file) or ".", f"{base_name}_vs_rtklib.png")
        plot_comparison(epochs1, epochs2, "LibGNSS++", "RTKLIB", cmp_png)


if __name__ == "__main__":
    main()
