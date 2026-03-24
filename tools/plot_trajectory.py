#!/usr/bin/env python3
"""2D trajectory comparison: libgnss++ vs RTKLIB with status color coding."""
import sys
import math
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
import matplotlib.patches as mpatches
import numpy as np

def read_ours(path):
    """Read libgnss++ pos file. Status: 1=SPP, 3=FLOAT, 4=FIXED"""
    epochs = []
    with open(path) as f:
        for line in f:
            if line.startswith('%'): continue
            p = line.split()
            if len(p) < 11: continue
            tow = float(p[1])
            lat, lon, h = float(p[5]), float(p[6]), float(p[7])
            status = int(p[8])
            epochs.append((tow, lat, lon, h, status))
    return epochs

def read_rtklib(path):
    """Read RTKLIB pos file. Q: 1=fix, 2=float, 4=dgps, 5=single"""
    epochs = []
    with open(path) as f:
        for line in f:
            if line.startswith('%'): continue
            p = line.split()
            if len(p) < 6: continue
            tow = float(p[1])
            lat, lon, h = float(p[2]), float(p[3]), float(p[4])
            q = int(p[5])
            # Map RTKLIB Q to our status: 1->4(fix), 2->3(float), 4->2(dgps), 5->1(spp)
            status_map = {1: 4, 2: 3, 4: 2, 5: 1}
            status = status_map.get(q, 1)
            epochs.append((tow, lat, lon, h, status))
    return epochs

def to_enu(epochs, ref_lat, ref_lon):
    """Convert lat/lon to local ENU (meters) relative to reference."""
    result = []
    for tow, lat, lon, h, status in epochs:
        e = (lon - ref_lon) * math.cos(math.radians(ref_lat)) * 111319.49
        n = (lat - ref_lat) * 111319.49
        result.append((tow, e, n, h, status))
    return result

STATUS_COLORS = {4: '#2ecc71', 3: '#f39c12', 2: '#3498db', 1: '#e74c3c'}
STATUS_NAMES = {4: 'Fix', 3: 'Float', 2: 'DGPS', 1: 'SPP'}

def plot_trajectory(ax, enu, label, alpha=1.0, marker='o', size=8):
    """Plot 2D trajectory with status coloring."""
    for status in [1, 2, 3, 4]:  # draw SPP first, Fix on top
        pts = [(e, n) for _, e, n, _, s in enu if s == status]
        if not pts: continue
        ee, nn = zip(*pts)
        ax.scatter(ee, nn, c=STATUS_COLORS[status], s=size, alpha=alpha,
                   marker=marker, edgecolors='none', zorder=status)

def main():
    if len(sys.argv) < 2:
        print("Usage: python3 plot_trajectory.py <ours.pos> [rtklib.pos] [output.png]")
        sys.exit(1)

    ours_path = sys.argv[1]
    rtklib_path = sys.argv[2] if len(sys.argv) > 2 else None
    out_path = sys.argv[3] if len(sys.argv) > 3 else ours_path.replace('.pos', '_trajectory.png')

    ours = read_ours(ours_path)
    if not ours:
        print("No data in", ours_path)
        sys.exit(1)

    # Reference: mean of fix positions
    fix_pts = [(lat, lon) for _, lat, lon, _, s in ours if s == 4]
    if fix_pts:
        ref_lat = np.mean([p[0] for p in fix_pts])
        ref_lon = np.mean([p[1] for p in fix_pts])
    else:
        ref_lat = np.mean([lat for _, lat, _, _, _ in ours])
        ref_lon = np.mean([lon for _, _, lon, _, _ in ours])

    ours_enu = to_enu(ours, ref_lat, ref_lon)

    has_rtklib = rtklib_path is not None
    if has_rtklib:
        rtklib = read_rtklib(rtklib_path)
        rtklib_enu = to_enu(rtklib, ref_lat, ref_lon)

    # --- Plot ---
    if has_rtklib:
        fig, axes = plt.subplots(1, 2, figsize=(16, 7))
    else:
        fig, axes = plt.subplots(1, 1, figsize=(8, 7))
        axes = [axes]

    # Panel 1: libgnss++
    ax = axes[0]
    plot_trajectory(ax, ours_enu, 'libgnss++')
    ax.set_xlabel('East (m)', fontsize=11)
    ax.set_ylabel('North (m)', fontsize=11)
    ax.set_title('libgnss++', fontsize=13, fontweight='bold')
    ax.set_aspect('equal')
    ax.grid(True, alpha=0.3)

    # Stats text
    n_fix = sum(1 for _, _, _, _, s in ours if s == 4)
    n_float = sum(1 for _, _, _, _, s in ours if s == 3)
    n_spp = sum(1 for _, _, _, _, s in ours if s == 1)
    n_total = len(ours)
    ax.text(0.02, 0.98, f'Fix: {n_fix}/{n_total} ({n_fix*100/n_total:.1f}%)\nFloat: {n_float}\nSPP: {n_spp}',
            transform=ax.transAxes, va='top', fontsize=9, fontfamily='monospace',
            bbox=dict(boxstyle='round,pad=0.3', facecolor='white', alpha=0.8))

    # Panel 2: RTKLIB
    if has_rtklib:
        ax2 = axes[1]
        plot_trajectory(ax2, rtklib_enu, 'RTKLIB')
        ax2.set_xlabel('East (m)', fontsize=11)
        ax2.set_title('RTKLIB', fontsize=13, fontweight='bold')
        ax2.set_aspect('equal')
        ax2.grid(True, alpha=0.3)

        n_fix_r = sum(1 for _, _, _, _, s in rtklib if s == 4)
        n_float_r = sum(1 for _, _, _, _, s in rtklib if s == 3)
        n_spp_r = sum(1 for _, _, _, _, s in rtklib if s == 1)
        n_total_r = len(rtklib)
        ax2.text(0.02, 0.98, f'Fix: {n_fix_r}/{n_total_r} ({n_fix_r*100/n_total_r:.1f}%)\nFloat: {n_float_r}\nSPP: {n_spp_r}',
                transform=ax2.transAxes, va='top', fontsize=9, fontfamily='monospace',
                bbox=dict(boxstyle='round,pad=0.3', facecolor='white', alpha=0.8))

        # Match axis limits
        all_e = [e for _, e, _, _, _ in ours_enu + rtklib_enu]
        all_n = [n for _, _, n, _, _ in ours_enu + rtklib_enu]
        margin = max(max(all_e) - min(all_e), max(all_n) - min(all_n)) * 0.05 + 1
        e_center = (max(all_e) + min(all_e)) / 2
        n_center = (max(all_n) + min(all_n)) / 2
        span = max(max(all_e) - min(all_e), max(all_n) - min(all_n)) / 2 + margin
        for a in axes:
            a.set_xlim(e_center - span, e_center + span)
            a.set_ylim(n_center - span, n_center + span)

    # Legend
    patches = [mpatches.Patch(color=STATUS_COLORS[s], label=STATUS_NAMES[s]) for s in [4, 3, 2, 1]]
    fig.legend(handles=patches, loc='lower center', ncol=4, fontsize=10,
               frameon=True, fancybox=True, shadow=True)

    fig.suptitle('RTK 2D Trajectory Comparison', fontsize=14, fontweight='bold', y=0.98)
    plt.tight_layout(rect=[0, 0.05, 1, 0.95])
    plt.savefig(out_path, dpi=150, bbox_inches='tight')
    print(f'Saved: {out_path}')

if __name__ == '__main__':
    main()
