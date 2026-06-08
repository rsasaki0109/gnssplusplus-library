#!/usr/bin/env python3
"""
RTK測位結果の可視化スクリプト
"""

import matplotlib.pyplot as plt
import numpy as np
from datetime import datetime, timedelta

# 日本語フォント設定
plt.rcParams['font.sans-serif'] = ['DejaVu Sans']
plt.rcParams['axes.unicode_minus'] = False

def read_rtklib_pos(filename):
    """RTKLIBの.posファイルを読み込む"""
    data = {
        'week': [], 'tow': [], 'lat': [], 'lon': [], 'height': [],
        'Q': [], 'ns': [], 'sdn': [], 'sde': [], 'sdu': [],
        'sdne': [], 'sdeu': [], 'sdun': [], 'age': [], 'ratio': []
    }

    with open(filename, 'r') as f:
        for line in f:
            if line.startswith('%') or line.strip() == '':
                continue

            parts = line.split()
            if len(parts) >= 15:
                data['week'].append(int(parts[0]))
                data['tow'].append(float(parts[1]))
                data['lat'].append(float(parts[2]))
                data['lon'].append(float(parts[3]))
                data['height'].append(float(parts[4]))
                data['Q'].append(int(parts[5]))
                data['ns'].append(int(parts[6]))
                data['sdn'].append(float(parts[7]))
                data['sde'].append(float(parts[8]))
                data['sdu'].append(float(parts[9]))
                data['sdne'].append(float(parts[10]))
                data['sdeu'].append(float(parts[11]))
                data['sdun'].append(float(parts[12]))
                data['age'].append(float(parts[13]))
                data['ratio'].append(float(parts[14]))

    # numpy配列に変換
    for key in data:
        data[key] = np.array(data[key])

    return data

def plot_position_timeseries(data, output_file='position_timeseries.png'):
    """位置の時系列プロット"""
    fig, axes = plt.subplots(3, 1, figsize=(12, 10))

    # 時刻軸（秒単位）
    time_sec = data['tow'] - data['tow'][0]
    time_min = time_sec / 60.0

    # 平均値からの偏差
    lat_mean = np.mean(data['lat'])
    lon_mean = np.mean(data['lon'])
    height_mean = np.mean(data['height'])

    lat_dev = (data['lat'] - lat_mean) * 111000  # 度からメートルへ（おおよそ）
    lon_dev = (data['lon'] - lon_mean) * 91000   # 度からメートルへ（緯度35度付近）
    height_dev = data['height'] - height_mean

    # 緯度
    axes[0].plot(time_min, lat_dev * 1000, 'b-', linewidth=0.5, label='Latitude deviation')
    axes[0].set_ylabel('Latitude deviation (mm)', fontsize=10)
    axes[0].grid(True, alpha=0.3)
    axes[0].set_title('RTK Positioning Results - Position Time Series', fontsize=12, fontweight='bold')

    # 経度
    axes[1].plot(time_min, lon_dev * 1000, 'g-', linewidth=0.5, label='Longitude deviation')
    axes[1].set_ylabel('Longitude deviation (mm)', fontsize=10)
    axes[1].grid(True, alpha=0.3)

    # 高さ
    axes[2].plot(time_min, height_dev * 1000, 'r-', linewidth=0.5, label='Height deviation')
    axes[2].set_xlabel('Time (minutes)', fontsize=10)
    axes[2].set_ylabel('Height deviation (mm)', fontsize=10)
    axes[2].grid(True, alpha=0.3)

    plt.tight_layout()
    plt.savefig(output_file, dpi=150, bbox_inches='tight')
    print(f"Saved: {output_file}")
    plt.close()

def plot_accuracy(data, output_file='accuracy.png'):
    """精度のプロット"""
    fig, axes = plt.subplots(2, 2, figsize=(12, 10))

    time_min = (data['tow'] - data['tow'][0]) / 60.0

    # 水平精度
    axes[0, 0].plot(time_min, data['sdn'] * 1000, 'b-', linewidth=1, label='North')
    axes[0, 0].plot(time_min, data['sde'] * 1000, 'r-', linewidth=1, label='East')
    axes[0, 0].set_ylabel('Horizontal Std Dev (mm)', fontsize=10)
    axes[0, 0].set_xlabel('Time (minutes)', fontsize=10)
    axes[0, 0].set_title('Horizontal Accuracy', fontsize=11, fontweight='bold')
    axes[0, 0].legend(fontsize=9)
    axes[0, 0].grid(True, alpha=0.3)

    # 垂直精度
    axes[0, 1].plot(time_min, data['sdu'] * 1000, 'g-', linewidth=1)
    axes[0, 1].set_ylabel('Vertical Std Dev (mm)', fontsize=10)
    axes[0, 1].set_xlabel('Time (minutes)', fontsize=10)
    axes[0, 1].set_title('Vertical Accuracy', fontsize=11, fontweight='bold')
    axes[0, 1].grid(True, alpha=0.3)

    # 衛星数
    axes[1, 0].plot(time_min, data['ns'], 'ko-', linewidth=1, markersize=3)
    axes[1, 0].set_ylabel('Number of Satellites', fontsize=10)
    axes[1, 0].set_xlabel('Time (minutes)', fontsize=10)
    axes[1, 0].set_title('Satellite Count', fontsize=11, fontweight='bold')
    axes[1, 0].grid(True, alpha=0.3)
    axes[1, 0].set_ylim([0, 12])

    # Ratio
    axes[1, 1].plot(time_min, data['ratio'], 'm-', linewidth=1)
    axes[1, 1].axhline(y=3.0, color='r', linestyle='--', linewidth=1, label='Threshold (3.0)')
    axes[1, 1].set_ylabel('Ambiguity Ratio', fontsize=10)
    axes[1, 1].set_xlabel('Time (minutes)', fontsize=10)
    axes[1, 1].set_title('Ambiguity Resolution Ratio', fontsize=11, fontweight='bold')
    axes[1, 1].legend(fontsize=9)
    axes[1, 1].grid(True, alpha=0.3)

    plt.tight_layout()
    plt.savefig(output_file, dpi=150, bbox_inches='tight')
    print(f"Saved: {output_file}")
    plt.close()

def plot_trajectory(data, output_file='trajectory.png'):
    """軌跡プロット"""
    fig, ax = plt.subplots(figsize=(10, 10))

    # 平均値からの偏差（メートル）
    lon_mean = np.mean(data['lon'])
    lat_mean = np.mean(data['lat'])

    x = (data['lon'] - lon_mean) * 91000  # 経度差→東西方向（m）
    y = (data['lat'] - lat_mean) * 111000  # 緯度差→南北方向（m）

    # 時間で色分け
    scatter = ax.scatter(x, y, c=range(len(x)), cmap='viridis', s=30, alpha=0.7)

    # 開始点と終了点をマーク
    ax.plot(x[0], y[0], 'go', markersize=12, label='Start', zorder=5)
    ax.plot(x[-1], y[-1], 'ro', markersize=12, label='End', zorder=5)

    # 軌跡線
    ax.plot(x, y, 'k-', linewidth=0.5, alpha=0.3)

    ax.set_xlabel('East-West (m)', fontsize=11)
    ax.set_ylabel('North-South (m)', fontsize=11)
    ax.set_title('RTK Trajectory (relative to mean position)', fontsize=12, fontweight='bold')
    ax.grid(True, alpha=0.3)
    ax.legend(fontsize=10)
    ax.axis('equal')

    # カラーバー
    cbar = plt.colorbar(scatter, ax=ax)
    cbar.set_label('Epoch number', fontsize=10)

    plt.tight_layout()
    plt.savefig(output_file, dpi=150, bbox_inches='tight')
    print(f"Saved: {output_file}")
    plt.close()

def plot_statistics(data, output_file='statistics.png'):
    """統計情報のプロット"""
    fig, axes = plt.subplots(2, 2, figsize=(12, 8))

    # 位置分布（2D）
    lon_mean = np.mean(data['lon'])
    lat_mean = np.mean(data['lat'])
    x = (data['lon'] - lon_mean) * 91000 * 1000  # mm
    y = (data['lat'] - lat_mean) * 111000 * 1000  # mm

    axes[0, 0].scatter(x, y, s=10, alpha=0.5)
    axes[0, 0].set_xlabel('East-West (mm)', fontsize=10)
    axes[0, 0].set_ylabel('North-South (mm)', fontsize=10)
    axes[0, 0].set_title('Horizontal Position Distribution', fontsize=11, fontweight='bold')
    axes[0, 0].grid(True, alpha=0.3)
    axes[0, 0].axis('equal')

    # 標高ヒストグラム
    axes[0, 1].hist(data['height'], bins=30, edgecolor='black', alpha=0.7)
    axes[0, 1].set_xlabel('Height (m)', fontsize=10)
    axes[0, 1].set_ylabel('Frequency', fontsize=10)
    axes[0, 1].set_title('Height Distribution', fontsize=11, fontweight='bold')
    axes[0, 1].grid(True, alpha=0.3, axis='y')

    # 精度ヒストグラム
    horiz_std = np.sqrt(data['sdn']**2 + data['sde']**2) * 1000  # mm
    axes[1, 0].hist(horiz_std, bins=30, edgecolor='black', alpha=0.7, color='blue')
    axes[1, 0].set_xlabel('Horizontal Std Dev (mm)', fontsize=10)
    axes[1, 0].set_ylabel('Frequency', fontsize=10)
    axes[1, 0].set_title('Horizontal Accuracy Distribution', fontsize=11, fontweight='bold')
    axes[1, 0].grid(True, alpha=0.3, axis='y')

    # 垂直精度ヒストグラム
    axes[1, 1].hist(data['sdu'] * 1000, bins=30, edgecolor='black', alpha=0.7, color='green')
    axes[1, 1].set_xlabel('Vertical Std Dev (mm)', fontsize=10)
    axes[1, 1].set_ylabel('Frequency', fontsize=10)
    axes[1, 1].set_title('Vertical Accuracy Distribution', fontsize=11, fontweight='bold')
    axes[1, 1].grid(True, alpha=0.3, axis='y')

    plt.tight_layout()
    plt.savefig(output_file, dpi=150, bbox_inches='tight')
    print(f"Saved: {output_file}")
    plt.close()

def print_summary(data):
    """統計サマリーを表示"""
    print("\n" + "="*60)
    print("RTK POSITIONING SUMMARY")
    print("="*60)

    print(f"\nTotal epochs: {len(data['lat'])}")
    print(f"Duration: {(data['tow'][-1] - data['tow'][0])/60:.1f} minutes")

    print(f"\nSolution Quality:")
    print(f"  Float (Q=1): {np.sum(data['Q'] == 1)} epochs ({np.sum(data['Q'] == 1)/len(data['Q'])*100:.1f}%)")
    print(f"  Fix (Q=2):   {np.sum(data['Q'] == 2)} epochs ({np.sum(data['Q'] == 2)/len(data['Q'])*100:.1f}%)")

    print(f"\nPosition Statistics:")
    print(f"  Mean Latitude:  {np.mean(data['lat']):.8f}°")
    print(f"  Mean Longitude: {np.mean(data['lon']):.8f}°")
    print(f"  Mean Height:    {np.mean(data['height']):.4f} m")
    print(f"  Std Dev Height: {np.std(data['height']):.4f} m")

    print(f"\nAccuracy Statistics:")
    horiz_std = np.sqrt(data['sdn']**2 + data['sde']**2) * 1000
    print(f"  Horizontal (RMS): {np.mean(horiz_std):.2f} mm")
    print(f"  Vertical (RMS):   {np.mean(data['sdu']) * 1000:.2f} mm")

    print(f"\nSatellite Statistics:")
    print(f"  Average: {np.mean(data['ns']):.1f} satellites")
    print(f"  Min: {np.min(data['ns'])} satellites")
    print(f"  Max: {np.max(data['ns'])} satellites")

    print(f"\nAmbiguity Ratio:")
    print(f"  Average: {np.mean(data['ratio']):.1f}")
    print(f"  Min: {np.min(data['ratio']):.1f}")
    print(f"  Max: {np.max(data['ratio']):.1f}")

    print("\n" + "="*60 + "\n")

if __name__ == '__main__':
    # データ読み込み
    data = read_rtklib_pos('rtk_solution.pos')

    # 統計サマリー表示
    print_summary(data)

    # プロット生成
    print("Generating visualizations...")
    plot_position_timeseries(data, 'rtk_position_timeseries.png')
    plot_accuracy(data, 'rtk_accuracy.png')
    plot_trajectory(data, 'rtk_trajectory.png')
    plot_statistics(data, 'rtk_statistics.png')

    print("\nVisualization complete! Generated files:")
    print("  - rtk_position_timeseries.png")
    print("  - rtk_accuracy.png")
    print("  - rtk_trajectory.png")
    print("  - rtk_statistics.png")
