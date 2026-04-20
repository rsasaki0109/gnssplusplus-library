#!/usr/bin/env python3
"""
Compare LibGNSS++ and RTKLIB RTK output files and generate a README-ready figure.
"""

from pathlib import Path
import math

import matplotlib.pyplot as plt
import numpy as np


ROOT = Path(__file__).resolve().parents[1]
OUTPUT_DIR = ROOT / "output"
LIB_FILE = OUTPUT_DIR / "rtk_solution.pos"
RTKLIB_FILE = OUTPUT_DIR / "rtklib_rtk_result.pos"
OUT_FIG = OUTPUT_DIR / "rtk_comparison.png"


def read_libgnsspp(path: Path) -> dict[str, np.ndarray]:
    rows = {
        "tow": [],
        "lat": [],
        "lon": [],
        "height": [],
        "q": [],
        "ns": [],
    }
    with path.open() as handle:
        for line in handle:
            if not line.strip() or line.startswith("%"):
                continue
            parts = line.split()
            if len(parts) < 11:
                continue
            rows["tow"].append(float(parts[1]))
            rows["lat"].append(float(parts[5]))
            rows["lon"].append(float(parts[6]))
            rows["height"].append(float(parts[7]))
            rows["q"].append(int(parts[8]))
            rows["ns"].append(int(parts[9]))
    return {key: np.array(value) for key, value in rows.items()}


def read_rtklib(path: Path) -> dict[str, np.ndarray]:
    rows = {
        "tow": [],
        "lat": [],
        "lon": [],
        "height": [],
        "q": [],
        "ns": [],
        "sdn": [],
        "sde": [],
        "sdu": [],
    }
    with path.open() as handle:
        for line in handle:
            if not line.strip() or line.startswith("%"):
                continue
            parts = line.split()
            if len(parts) < 15:
                continue
            rows["tow"].append(float(parts[1]))
            rows["lat"].append(float(parts[2]))
            rows["lon"].append(float(parts[3]))
            rows["height"].append(float(parts[4]))
            rows["q"].append(int(parts[5]))
            rows["ns"].append(int(parts[6]))
            rows["sdn"].append(float(parts[7]))
            rows["sde"].append(float(parts[8]))
            rows["sdu"].append(float(parts[9]))
    return {key: np.array(value) for key, value in rows.items()}


def local_offsets(data: dict[str, np.ndarray]) -> tuple[np.ndarray, np.ndarray]:
    lat0 = np.mean(data["lat"])
    lon0 = np.mean(data["lon"])
    north_m = (data["lat"] - lat0) * 111000.0
    east_m = (data["lon"] - lon0) * 91000.0
    return east_m, north_m


def rms(values: np.ndarray) -> float:
    return math.sqrt(np.mean(np.square(values)))


def horizontal_distance(data: dict[str, np.ndarray]) -> np.ndarray:
    east_m, north_m = local_offsets(data)
    return np.sqrt(np.square(east_m) + np.square(north_m))


def summarize(data: dict[str, np.ndarray]) -> dict[str, float]:
    horiz = horizontal_distance(data)
    up = data["height"] - np.mean(data["height"])
    return {
        "epochs": int(len(data["tow"])),
        "mean_ns": float(np.mean(data["ns"])),
        "horiz_rms_m": rms(horiz),
        "vert_rms_m": rms(up),
        "horiz_p95_m": float(np.percentile(horiz, 95)),
        "vert_p95_m": float(np.percentile(np.abs(up), 95)),
    }


def main() -> None:
    lib = read_libgnsspp(LIB_FILE)
    rtk = read_rtklib(RTKLIB_FILE)
    lib_summary = summarize(lib)
    rtk_summary = summarize(rtk)

    fig, axes = plt.subplots(2, 2, figsize=(14, 10))

    for data, title, color in (
        (lib, "LibGNSS++", "tab:orange"),
        (rtk, "RTKLIB", "tab:blue"),
    ):
        east_m, north_m = local_offsets(data)
        axes[0, 0].scatter(
            east_m,
            north_m,
            s=20,
            alpha=0.65,
            label=title,
            color=color,
        )

    axes[0, 0].set_title("Horizontal position scatter")
    axes[0, 0].set_xlabel("East-West offset from mean (m)")
    axes[0, 0].set_ylabel("North-South offset from mean (m)")
    axes[0, 0].grid(alpha=0.3)
    axes[0, 0].axis("equal")
    axes[0, 0].legend()

    lib_t = (lib["tow"] - lib["tow"][0]) / 60.0
    rtk_t = (rtk["tow"] - rtk["tow"][0]) / 60.0
    axes[0, 1].plot(lib_t, lib["height"] - np.mean(lib["height"]), label="LibGNSS++", color="tab:orange")
    axes[0, 1].plot(rtk_t, rtk["height"] - np.mean(rtk["height"]), label="RTKLIB", color="tab:blue")
    axes[0, 1].set_title("Height deviation over time")
    axes[0, 1].set_xlabel("Time (min)")
    axes[0, 1].set_ylabel("Height offset from mean (m)")
    axes[0, 1].grid(alpha=0.3)
    axes[0, 1].legend()

    labels = ["Horiz RMS", "Vert RMS", "Horiz P95", "Vert P95"]
    lib_vals = [
        lib_summary["horiz_rms_m"],
        lib_summary["vert_rms_m"],
        lib_summary["horiz_p95_m"],
        lib_summary["vert_p95_m"],
    ]
    rtk_vals = [
        rtk_summary["horiz_rms_m"],
        rtk_summary["vert_rms_m"],
        rtk_summary["horiz_p95_m"],
        rtk_summary["vert_p95_m"],
    ]
    x = np.arange(len(labels))
    width = 0.36
    axes[1, 0].bar(x - width / 2, lib_vals, width, label="LibGNSS++", color="tab:orange")
    axes[1, 0].bar(x + width / 2, rtk_vals, width, label="RTKLIB", color="tab:blue")
    axes[1, 0].set_title("Dispersion metrics (lower is better)")
    axes[1, 0].set_xticks(x, labels)
    axes[1, 0].set_ylabel("Meters")
    axes[1, 0].grid(axis="y", alpha=0.3)
    axes[1, 0].legend()

    axes[1, 1].axis("off")
    text = "\n".join(
        [
            "Measured on bundled sample outputs",
            "",
            f"LibGNSS++  epochs={lib_summary['epochs']}  mean sats={lib_summary['mean_ns']:.2f}",
            f"  horiz RMS={lib_summary['horiz_rms_m']:.3f} m",
            f"  vert  RMS={lib_summary['vert_rms_m']:.3f} m",
            f"  horiz P95={lib_summary['horiz_p95_m']:.3f} m",
            f"  vert  P95={lib_summary['vert_p95_m']:.3f} m",
            "",
            f"RTKLIB     epochs={rtk_summary['epochs']}  mean sats={rtk_summary['mean_ns']:.2f}",
            f"  horiz RMS={rtk_summary['horiz_rms_m']:.3f} m",
            f"  vert  RMS={rtk_summary['vert_rms_m']:.3f} m",
            f"  horiz P95={rtk_summary['horiz_p95_m']:.3f} m",
            f"  vert  P95={rtk_summary['vert_p95_m']:.3f} m",
        ]
    )
    axes[1, 1].text(
        0.0,
        1.0,
        text,
        va="top",
        ha="left",
        family="monospace",
        fontsize=11,
    )

    fig.suptitle("RTK output comparison: current LibGNSS++ vs RTKLIB baseline", fontsize=16)
    fig.tight_layout()
    fig.savefig(OUT_FIG, dpi=180, bbox_inches="tight")
    print(f"Saved: {OUT_FIG}")


if __name__ == "__main__":
    main()
