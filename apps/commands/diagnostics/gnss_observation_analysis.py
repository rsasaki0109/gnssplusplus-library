#!/usr/bin/env python3
"""Export satellite residual, SNR and carrier-continuity diagnostics from RINEX."""
from __future__ import annotations

import argparse
import csv
import hashlib
import importlib
import json
import math
import os
from pathlib import Path
import sys
import time


def file_record(path: Path) -> dict:
    digest = hashlib.sha256()
    with path.open("rb") as handle:
        for block in iter(lambda: handle.read(1048576), b""):
            digest.update(block)
    return {"path": str(path.resolve()), "sha256": digest.hexdigest(), "bytes": path.stat().st_size}


def write_records(path: Path, records: list[dict]) -> None:
    with path.open("w", encoding="utf-8", newline="") as handle:
        writer = csv.DictWriter(handle, fieldnames=list(records[0]))
        writer.writeheader()
        for record in records:
            writer.writerow({**record, "reason_codes": "|".join(record["reason_codes"])})


def plot_satellites(records: list[dict], out: Path, selected: list[str] | None,
                    threshold: float, max_gap_s: float = 2.0) -> list[Path]:
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt

    available = sorted({row["satellite_id"] for row in records})
    if selected is not None and set(selected) - set(available):
        raise ValueError(f"unknown plot satellites: {sorted(set(selected) - set(available))}")
    satellites = selected or available
    origin = min(row["gps_seconds"] for row in records)
    paths = []
    for satellite in satellites:
        rows = [row for row in records if row["satellite_id"] == satellite]
        fig, axes = plt.subplots(4, 1, figsize=(10, 9), sharex=True, layout="constrained")
        keys = ("clock_removed_code_residual_m", "snr_dbhz", "phase_doppler_raw_cycles", "phase_doppler_adjusted_cycles")
        labels = ("Clock-centered code (m)", "SNR (dB-Hz)", "Raw phase + Doppler (cycles)", "Adjusted continuity (cycles)")
        tracks = sorted({(row["signal_id"], row["carrier_observation_type"]) for row in rows})
        for signal, tracking in tracks:
            track = [row for row in rows if (row["signal_id"], row["carrier_observation_type"]) == (signal, tracking)]
            for axis, key in zip(axes, keys):
                if any(row[key] is not None for row in track):
                    times, values = [], []
                    previous_time = None
                    for row in track:
                        if previous_time is not None and row["gps_seconds"] - previous_time > max_gap_s:
                            times.append(row["gps_seconds"] - origin)
                            values.append(math.nan)
                        times.append(row["gps_seconds"] - origin)
                        values.append(row[key] if row[key] is not None else math.nan)
                        previous_time = row["gps_seconds"]
                    axis.plot(times, values, ".-", linewidth=0.7, markersize=2,
                              label=f"signal {signal} {tracking}")
        for axis, label in zip(axes, labels):
            axis.set_ylabel(label)
            axis.grid(alpha=0.25)
            for row in rows:
                if row["slip_suspect"]:
                    axis.axvline(row["gps_seconds"] - origin, color="#dc2626", alpha=0.25, linewidth=0.6)
                elif row["clock_step_candidate"]:
                    axis.axvline(row["gps_seconds"] - origin, color="#d97706", alpha=0.25, linewidth=0.6)
            if axis.lines:
                axis.legend(loc="upper right", fontsize=8)
        for axis in axes[2:]:
            axis.axhline(threshold, color="#6b7280", linestyle="--", linewidth=0.7)
            axis.axhline(-threshold, color="#6b7280", linestyle="--", linewidth=0.7)
        axes[0].set_title(f"{satellite}: diagnostic residuals and carrier continuity\nRed: slip indication; orange: common clock event candidate")
        axes[-1].set_xlabel("Seconds since first analyzed GPS epoch")
        path = out / f"{satellite}.png"
        fig.savefig(path, dpi=130)
        plt.close(fig)
        paths.append(path)
    return paths


def parse_args(argv: list[str] | None = None) -> argparse.Namespace:
    parser = argparse.ArgumentParser(prog=os.environ.get("GNSS_CLI_NAME"), description=__doc__)
    parser.add_argument("--obs", required=True, type=Path)
    parser.add_argument("--nav", required=True, type=Path)
    parser.add_argument("--bindings-dir", type=Path,
                        help="Parent directory of the built libgnsspp package, e.g. <build>/python.")
    parser.add_argument("--runtime-dir", type=Path, action="append", default=[],
                        help="Shared-library directory; on Windows it is registered for DLL loading.")
    parser.add_argument("--max-epochs", type=int, default=300,
                        help="Bounded analysis by default (300); 0 reads the complete file.")
    parser.add_argument("--slip-threshold-cycles", type=float, default=10.0)
    parser.add_argument("--max-gap-s", type=float, default=2.0)
    parser.add_argument("--plot-satellites", nargs="+", help="Plot selected IDs; CSV and summary still include all satellites.")
    parser.add_argument("--output-dir", required=True, type=Path)
    args = parser.parse_args(argv)
    if args.max_epochs < 0:
        parser.error("--max-epochs must be nonnegative")
    return args


def analyze(args: argparse.Namespace) -> dict:
    obs, nav, out = (path.expanduser().resolve() for path in (args.obs, args.nav, args.output_dir))
    for path in (obs, nav):
        if not path.is_file():
            raise ValueError(f"missing RINEX input: {path}")
    if out.exists() and (not out.is_dir() or any(out.iterdir())):
        raise ValueError("output directory must be new or empty")
    if args.bindings_dir is not None:
        package = args.bindings_dir.expanduser().resolve()
        if not (package / "libgnsspp").is_dir():
            raise ValueError("--bindings-dir must contain the built libgnsspp package")
        sys.path.insert(0, str(package))
    dll_handles = []
    try:
        for directory in args.runtime_dir:
            if not directory.is_dir():
                raise ValueError(f"missing runtime directory: {directory}")
            if os.name == "nt":
                dll_handles.append(os.add_dll_directory(str(directory.resolve())))
        try:
            library = importlib.import_module("libgnsspp")
        except ImportError as error:
            raise ValueError("build the public Python bindings and pass --bindings-dir <build>/python") from error
        if not hasattr(library, "observations"):
            raise ValueError("bindings are stale: rebuild the observation-analysis version")
        inputs = [file_record(obs), file_record(nav)]
        script = file_record(Path(__file__))
        package_dir = Path(library.__file__).resolve().parent
        binding_files = [file_record(path) for path in sorted(package_dir.iterdir())
                         if path.is_file() and (path.suffix in (".py", ".pyd", ".so") or ".so." in path.name)]
        started = time.perf_counter()
        epochs = library.preprocess_spp_file(str(obs), str(nav), max_epochs=args.max_epochs)
        records, summary = library.observations.analyze_epochs(
            epochs, slip_threshold_cycles=args.slip_threshold_cycles, max_gap_s=args.max_gap_s)
        out.mkdir(parents=True, exist_ok=True)
        write_records(out / "observations.csv", records)
        plot_dir = out / "satellites"
        plot_dir.mkdir()
        plots = plot_satellites(records, plot_dir, args.plot_satellites, args.slip_threshold_cycles, args.max_gap_s)
        for record in inputs + binding_files + [script]:
            if file_record(Path(record["path"])) != record:
                raise ValueError(f"input or binding changed during analysis: {record['path']}")
        summary.update(state="passed", inputs=inputs, binding_files=binding_files, analysis_script=script,
                       max_epochs=args.max_epochs, evaluation="bounded" if args.max_epochs else "full",
                       wall_s=time.perf_counter() - started,
                       plot_satellites=[path.stem for path in plots],
                       artifacts=[file_record(out / "observations.csv"), *(file_record(path) for path in plots)])
        (out / "summary.json").write_text(json.dumps(summary, indent=2, sort_keys=True, allow_nan=False) + "\n", encoding="utf-8")
        return summary
    finally:
        for handle in dll_handles:
            handle.close()


def main() -> int:
    try:
        result = analyze(parse_args())
    except (OSError, ValueError, AttributeError, ImportError, RuntimeError) as error:
        print(f"Observation analysis failed: {error}", file=sys.stderr)
        return 1
    print(f"Analyzed {result['epochs']} epochs, {result['observation_rows']} rows, {len(result['satellites'])} satellites")
    print(f"Carrier discontinuity indications: {result['slip_suspect_rows']} (diagnostic candidates)")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
