#!/usr/bin/env python3
"""
Generate XY and time-series comparison plots for LibGNSS++ vs RTKLIB.
"""

from pathlib import Path
import math

import matplotlib.pyplot as plt
import numpy as np


ROOT = Path(__file__).resolve().parents[1]
OUTPUT_DIR = ROOT / "output"


def read_libgnsspp(path: Path) -> dict[str, np.ndarray]:
    rows = {"tow": [], "lat": [], "lon": [], "height": [], "q": [], "ns": []}
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
    return {k: np.array(v) for k, v in rows.items()}


def read_rtklib(path: Path) -> dict[str, np.ndarray]:
    rows = {"tow": [], "lat": [], "lon": [], "height": [], "q": [], "ns": []}
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
    return {k: np.array(v) for k, v in rows.items()}


def local_offsets(data: dict[str, np.ndarray]) -> tuple[np.ndarray, np.ndarray]:
    lat0 = np.mean(data["lat"])
    lon0 = np.mean(data["lon"])
    north_m = (data["lat"] - lat0) * 111000.0
    east_m = (data["lon"] - lon0) * 91000.0
    return east_m, north_m


def rms(values: np.ndarray) -> float:
    return math.sqrt(np.mean(np.square(values)))


def displacement_from_start(data: dict[str, np.ndarray]) -> tuple[float, float]:
    north = (data["lat"][-1] - data["lat"][0]) * 111000.0
    east = (data["lon"][-1] - data["lon"][0]) * 91000.0
    up = data["height"][-1] - data["height"][0]
    return math.hypot(north, east), up


def summary(data: dict[str, np.ndarray]) -> dict[str, float]:
    east_m, north_m = local_offsets(data)
    up_m = data["height"] - np.mean(data["height"])
    horiz = np.sqrt(np.square(east_m) + np.square(north_m))
    start_end_h, start_end_v = displacement_from_start(data)
    return {
        "epochs": int(len(data["tow"])),
        "mean_ns": float(np.mean(data["ns"])),
        "horiz_rms_m": rms(horiz),
        "vert_rms_m": rms(up_m),
        "horiz_p95_m": float(np.percentile(horiz, 95)),
        "vert_p95_m": float(np.percentile(np.abs(up_m), 95)),
        "start_end_h_m": start_end_h,
        "start_end_v_m": start_end_v,
    }


def plot_xy(lib: dict[str, np.ndarray], rtk: dict[str, np.ndarray], path: Path) -> None:
    lib_e, lib_n = local_offsets(lib)
    rtk_e, rtk_n = local_offsets(rtk)
    plt.figure(figsize=(8, 7))
    plt.scatter(lib_e, lib_n, s=28, alpha=0.7, label="LibGNSS++", color="tab:orange")
    plt.scatter(rtk_e, rtk_n, s=28, alpha=0.7, label="RTKLIB", color="tab:blue")
    plt.xlabel("East-West offset from mean (m)")
    plt.ylabel("North-South offset from mean (m)")
    plt.title("XY comparison of RTK outputs")
    plt.grid(alpha=0.3)
    plt.axis("equal")
    plt.legend()
    plt.tight_layout()
    plt.savefig(path, dpi=180, bbox_inches="tight")
    plt.close()


def plot_timeseries(lib: dict[str, np.ndarray], rtk: dict[str, np.ndarray], path: Path) -> None:
    fig, axes = plt.subplots(3, 1, figsize=(12, 9), sharex=False)
    for ax, key, title in (
        (axes[0], "lat", "Latitude deviation"),
        (axes[1], "lon", "Longitude deviation"),
        (axes[2], "height", "Height deviation"),
    ):
        for data, label, color, scale in (
            (lib, "LibGNSS++", "tab:orange", 111000.0 if key == "lat" else (91000.0 if key == "lon" else 1.0)),
            (rtk, "RTKLIB", "tab:blue", 111000.0 if key == "lat" else (91000.0 if key == "lon" else 1.0)),
        ):
            t = (data["tow"] - data["tow"][0]) / 60.0
            mean = np.mean(data[key])
            dev = (data[key] - mean) * scale
            ax.plot(t, dev, label=label, color=color, linewidth=1.2)
        unit = "m"
        ax.set_ylabel(f"{title} ({unit})")
        ax.set_title(title)
        ax.grid(alpha=0.3)
    axes[2].set_xlabel("Time (min)")
    axes[0].legend()
    fig.tight_layout()
    fig.savefig(path, dpi=180, bbox_inches="tight")
    plt.close(fig)


def main() -> None:
    lib = read_libgnsspp(OUTPUT_DIR / "rtk_solution.pos")
    rtk = read_rtklib(OUTPUT_DIR / "rtklib_rtk_result.pos")
    if len(lib["tow"]) == 0:
        raise SystemExit("LibGNSS++ output is empty; run the RTK example before generating assets.")
    if len(rtk["tow"]) == 0:
        raise SystemExit("RTKLIB output is empty; provide output/rtklib_rtk_result.pos first.")
    plot_xy(lib, rtk, OUTPUT_DIR / "rtk_xy_comparison.png")
    plot_timeseries(lib, rtk, OUTPUT_DIR / "rtk_timeseries_comparison.png")

    for label, data in (("LibGNSS++", lib), ("RTKLIB", rtk)):
        stats = summary(data)
        print(label)
        for key, value in stats.items():
            print(f"  {key}={value}")


if __name__ == "__main__":
    main()
