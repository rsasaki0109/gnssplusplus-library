#!/usr/bin/env python3
"""Run the full UrbanNav Tokyo Odaiba benchmark pipeline."""

from __future__ import annotations

import argparse
import concurrent.futures
import csv
import json
import os
from pathlib import Path
import subprocess
import sys
import tempfile

from gnss_runtime import resolve_gnss_command


ROOT_DIR = Path(__file__).resolve().parent.parent
DEFAULT_RTKLIB = "/tmp/RTKLIB/app/rnx2rtkp/gcc/rnx2rtkp"

sys.path.insert(0, str(ROOT_DIR / "scripts"))

import generate_driving_comparison as driving_comparison  # noqa: E402


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(prog=os.environ.get("GNSS_CLI_NAME"))
    parser.add_argument(
        "--rtklib-bin",
        default=os.environ.get("RTKLIB_RNX2RTKP", DEFAULT_RTKLIB),
        help="Path to the RTKLIB rnx2rtkp binary.",
    )
    parser.add_argument(
        "--malib-bin",
        default=os.environ.get("MALIB_RNX2RTKP"),
        help="Optional path to the MALIB rnx2rtkp binary.",
    )
    parser.add_argument(
        "--rover",
        type=Path,
        default=ROOT_DIR / "data/driving/Tokyo_Data/Odaiba/rover_trimble.obs",
        help="Rover RINEX observation file.",
    )
    parser.add_argument(
        "--base",
        type=Path,
        default=ROOT_DIR / "data/driving/Tokyo_Data/Odaiba/base_trimble.obs",
        help="Base RINEX observation file.",
    )
    parser.add_argument(
        "--nav",
        type=Path,
        default=ROOT_DIR / "data/driving/Tokyo_Data/Odaiba/base.nav",
        help="Navigation RINEX file.",
    )
    parser.add_argument(
        "--reference-csv",
        type=Path,
        default=ROOT_DIR / "data/driving/Tokyo_Data/Odaiba/reference.csv",
        help="Ground-truth CSV from UrbanNav.",
    )
    parser.add_argument(
        "--rtklib-config",
        type=Path,
        default=ROOT_DIR / "scripts/rtklib_odaiba.conf",
        help="RTKLIB configuration file.",
    )
    parser.add_argument(
        "--malib-config",
        type=Path,
        default=ROOT_DIR / "scripts/rtklib_odaiba.conf",
        help="Optional MALIB configuration file.",
    )
    parser.add_argument(
        "--lib-pos",
        type=Path,
        default=ROOT_DIR / "output/rtk_solution.pos",
        help="Output path for libgnss++ solution.",
    )
    parser.add_argument(
        "--lib-kml",
        type=Path,
        default=ROOT_DIR / "output/rtk_solution.kml",
        help="Output path for libgnss++ KML.",
    )
    parser.add_argument(
        "--rtklib-pos",
        type=Path,
        default=ROOT_DIR / "output/driving_rtklib_rtk.pos",
        help="Output path for RTKLIB solution.",
    )
    parser.add_argument(
        "--malib-pos",
        type=Path,
        default=ROOT_DIR / "output/driving_malib_rtk.pos",
        help="Output path for optional MALIB solution.",
    )
    parser.add_argument(
        "--comparison-png",
        type=Path,
        default=ROOT_DIR / "docs/driving_odaiba_comparison.png",
        help="Output path for the comparison figure.",
    )
    parser.add_argument(
        "--scorecard-png",
        type=Path,
        default=ROOT_DIR / "docs/driving_odaiba_scorecard.png",
        help="Output path for the scorecard figure.",
    )
    parser.add_argument(
        "--social-card-png",
        type=Path,
        default=ROOT_DIR / "docs/driving_odaiba_social_card.png",
        help="Output path for the social-card figure.",
    )
    parser.add_argument(
        "--comparison-title",
        default="Urban Driving Comparison: libgnss++ vs RTKLIB on UrbanNav Odaiba",
        help="Title for the comparison figure.",
    )
    parser.add_argument(
        "--scorecard-title",
        default="UrbanNav Tokyo Odaiba",
        help="Title for the scorecard figure.",
    )
    parser.add_argument(
        "--social-card-title",
        default="UrbanNav Tokyo Odaiba",
        help="Title for the social-card figure.",
    )
    parser.add_argument(
        "--summary-json",
        type=Path,
        default=ROOT_DIR / "output/odaiba_summary.json",
        help="Output path for a machine-readable benchmark summary JSON.",
    )
    parser.add_argument(
        "--require-all-epochs-min",
        type=int,
        default=0,
        help="Fail if libgnss++ all-epoch matched count is below this value.",
    )
    parser.add_argument(
        "--require-common-epoch-pairs-min",
        type=int,
        default=0,
        help="Fail if common epoch pairs are below this value.",
    )
    parser.add_argument(
        "--require-lib-all-p95-h-max",
        type=float,
        default=None,
        help="Fail if libgnss++ all-epoch p95 horizontal error exceeds this value in meters.",
    )
    parser.add_argument(
        "--require-lib-common-median-h-max",
        type=float,
        default=None,
        help="Fail if libgnss++ common-epoch median horizontal error exceeds this value in meters.",
    )
    parser.add_argument(
        "--require-lib-common-p95-h-max",
        type=float,
        default=None,
        help="Fail if libgnss++ common-epoch p95 horizontal error exceeds this value in meters.",
    )
    parser.add_argument(
        "--mode",
        default="kinematic",
        choices=("auto", "kinematic", "static"),
        help="Mode passed through to gnss solve.",
    )
    parser.add_argument(
        "--glonass-ar",
        default="off",
        choices=("off", "on", "autocal"),
        help="GLONASS ambiguity resolution mode passed through to gnss solve.",
    )
    parser.add_argument(
        "--skip-epochs",
        type=int,
        default=0,
        help="Skip the first N rover epochs before solving with libgnss++.",
    )
    parser.add_argument(
        "--max-epochs",
        type=int,
        default=-1,
        help="Stop libgnss++ after N rover epochs (default: full dataset).",
    )
    parser.add_argument(
        "--segment-epochs",
        type=int,
        default=0,
        help="If > 0, solve libgnss++ in overlapping epoch segments of this size and merge them.",
    )
    parser.add_argument(
        "--warmup-epochs",
        type=int,
        default=300,
        help="Extra epochs before/after each segment for ambiguity warm-up when --segment-epochs is used.",
    )
    parser.add_argument(
        "--jobs",
        type=int,
        default=4,
        help="Parallel worker count for segmented libgnss++ solving.",
    )
    return parser.parse_args()


def ensure_exists(path: Path, description: str) -> None:
    if not path.exists():
        raise SystemExit(f"Missing {description}: {path}")


def run_command(command: list[str]) -> None:
    print("+", " ".join(command))
    subprocess.run(command, check=True)


def rounded_metrics(summary: dict[str, float]) -> dict[str, float | int]:
    rounded: dict[str, float | int] = {}
    for key, value in summary.items():
        if isinstance(value, float):
            rounded[key] = round(value, 6)
        else:
            rounded[key] = value
    return rounded


def write_summary_json(args: argparse.Namespace) -> dict[str, object]:
    reference = driving_comparison.read_reference_csv(args.reference_csv)
    lib_epochs = driving_comparison.read_libgnss_pos(args.lib_pos)
    rtklib_epochs = driving_comparison.read_rtklib_pos(args.rtklib_pos)
    malib_epochs = (
        driving_comparison.read_rtklib_pos(args.malib_pos)
        if args.malib_pos.exists()
        else None
    )

    lib_matched = driving_comparison.match_to_reference(lib_epochs, reference, 0.11)
    rtklib_matched = driving_comparison.match_to_reference(rtklib_epochs, reference, 0.11)
    lib_summary = driving_comparison.summarize(lib_matched, fixed_status=4, label="libgnss++")
    rtklib_summary = driving_comparison.summarize(rtklib_matched, fixed_status=1, label="RTKLIB")
    pairs = driving_comparison.pair_epochs(lib_matched, rtklib_matched, tolerance_s=0.11)
    lib_common_summary, rtklib_common_summary = driving_comparison.summarize_common_epochs(
        pairs,
        lib_fixed_status=4,
        rtklib_fixed_status=1,
    )
    malib_summary = None
    malib_pairs: list[driving_comparison.EpochPair] = []
    malib_common_summary = None
    if malib_epochs is not None:
        malib_matched = driving_comparison.match_to_reference(malib_epochs, reference, 0.11)
        malib_summary = driving_comparison.summarize(malib_matched, fixed_status=1, label="MALIB")
        malib_pairs = driving_comparison.pair_epochs(lib_matched, malib_matched, tolerance_s=0.11)
        lib_malib_common_summary, malib_common_summary = driving_comparison.summarize_common_epochs(
            malib_pairs,
            lib_fixed_status=4,
            rtklib_fixed_status=1,
        )
    else:
        lib_malib_common_summary = None

    payload = {
        "dataset": "UrbanNav Tokyo Odaiba",
        "reference_csv": str(args.reference_csv),
        "lib_pos": str(args.lib_pos),
        "rtklib_pos": str(args.rtklib_pos),
        "malib_pos": str(args.malib_pos) if args.malib_pos.exists() else None,
        "comparison_png": str(args.comparison_png),
        "scorecard_png": str(args.scorecard_png),
        "social_card_png": str(args.social_card_png),
        "common_epoch_pairs": len(pairs),
        "rtklib_common_epoch_pairs": len(pairs),
        "malib_common_epoch_pairs": len(malib_pairs) if malib_summary is not None else None,
        "libgnss_all_epochs": rounded_metrics(lib_summary),
        "rtklib_all_epochs": rounded_metrics(rtklib_summary),
        "libgnss_common_epochs": rounded_metrics(lib_common_summary),
        "rtklib_common_epochs": rounded_metrics(rtklib_common_summary),
        "malib_all_epochs": rounded_metrics(malib_summary) if malib_summary is not None else None,
        "malib_common_epochs": (
            rounded_metrics(malib_common_summary) if malib_common_summary is not None else None
        ),
        "libgnss_vs_malib_common_epochs": (
            rounded_metrics(lib_malib_common_summary)
            if lib_malib_common_summary is not None
            else None
        ),
    }

    args.summary_json.parent.mkdir(parents=True, exist_ok=True)
    args.summary_json.write_text(json.dumps(payload, indent=2, sort_keys=True) + "\n", encoding="utf-8")
    return payload


def enforce_summary_requirements(payload: dict[str, object], args: argparse.Namespace) -> None:
    failures: list[str] = []

    common_epoch_pairs = int(payload["common_epoch_pairs"])
    lib_all = payload["libgnss_all_epochs"]
    lib_common = payload["libgnss_common_epochs"]
    assert isinstance(lib_all, dict)
    assert isinstance(lib_common, dict)

    if args.require_all_epochs_min > 0 and int(lib_all["epochs"]) < args.require_all_epochs_min:
        failures.append(
            f"libgnss++ all-epoch matched count {int(lib_all['epochs'])} < {args.require_all_epochs_min}"
        )
    if (
        args.require_common_epoch_pairs_min > 0
        and common_epoch_pairs < args.require_common_epoch_pairs_min
    ):
        failures.append(
            f"common epoch pairs {common_epoch_pairs} < {args.require_common_epoch_pairs_min}"
        )
    if (
        args.require_lib_all_p95_h_max is not None
        and float(lib_all["p95_h_m"]) > args.require_lib_all_p95_h_max
    ):
        failures.append(
            f"libgnss++ all-epoch p95_h {float(lib_all['p95_h_m']):.6f} m > "
            f"{args.require_lib_all_p95_h_max:.6f} m"
        )
    if (
        args.require_lib_common_median_h_max is not None
        and float(lib_common["median_h_m"]) > args.require_lib_common_median_h_max
    ):
        failures.append(
            f"libgnss++ common-epoch median_h {float(lib_common['median_h_m']):.6f} m > "
            f"{args.require_lib_common_median_h_max:.6f} m"
        )
    if (
        args.require_lib_common_p95_h_max is not None
        and float(lib_common["p95_h_m"]) > args.require_lib_common_p95_h_max
    ):
        failures.append(
            f"libgnss++ common-epoch p95_h {float(lib_common['p95_h_m']):.6f} m > "
            f"{args.require_lib_common_p95_h_max:.6f} m"
        )

    if failures:
        message = "Odaiba benchmark sign-off checks failed:\n" + "\n".join(
            f"  - {failure}" for failure in failures
        )
        raise SystemExit(message)


def read_reference_tows(path: Path) -> list[float]:
    rows: list[float] = []
    with path.open(newline="") as handle:
        reader = csv.reader(handle)
        next(reader)
        for row in reader:
            rows.append(float(row[0]))
    return rows


def run_segmented_lib_solve(args: argparse.Namespace, gnss_command: Path | list[str]) -> None:
    if args.segment_epochs <= 0:
        raise ValueError("segment_epochs must be > 0")
    if args.warmup_epochs < 0:
        raise ValueError("warmup_epochs must be >= 0")
    if args.jobs <= 0:
        raise ValueError("jobs must be > 0")

    tows = read_reference_tows(args.reference_csv)
    if not tows:
        raise SystemExit(f"No reference epochs found in {args.reference_csv}")

    total_epochs = len(tows)
    segments: list[dict[str, object]] = []
    for start in range(0, total_epochs, args.segment_epochs):
        end = min(total_epochs, start + args.segment_epochs)
        warm_start = max(0, start - args.warmup_epochs)
        warm_end = min(total_epochs, end + args.warmup_epochs)
        segments.append(
            {
                "start": start,
                "end": end,
                "warm_start": warm_start,
                "warm_end": warm_end,
                "tow_start": tows[start],
                "tow_end_exclusive": tows[end] if end < total_epochs else None,
            }
        )

    print(
        f"Running segmented libgnss++ solve: {len(segments)} segments, "
        f"segment={args.segment_epochs} epochs, warmup={args.warmup_epochs}, jobs={args.jobs}"
    )

    command_prefix = (
        [sys.executable, str(gnss_command)]
        if isinstance(gnss_command, Path)
        else list(gnss_command)
    )

    with tempfile.TemporaryDirectory(prefix="odaiba_segments_") as temp_dir:
        temp_root = Path(temp_dir)
        for index, segment in enumerate(segments):
            segment["path"] = temp_root / f"segment_{index:03d}.pos"

        def run_one(index: int, segment: dict[str, object]) -> Path:
            out_path = segment["path"]
            command = [
                *command_prefix,
                "solve",
                "--rover",
                str(args.rover),
                "--base",
                str(args.base),
                "--nav",
                str(args.nav),
                "--out",
                str(out_path),
                "--mode",
                args.mode,
                "--glonass-ar",
                args.glonass_ar,
                "--no-kml",
                "--no-kinematic-post-filter",
                "--skip-epochs",
                str(segment["warm_start"]),
                "--max-epochs",
                str(int(segment["warm_end"]) - int(segment["warm_start"])),
            ]
            print(
                f"[segment {index + 1}/{len(segments)}] "
                f"target={segment['start']}-{int(segment['end']) - 1} "
                f"warm={segment['warm_start']}-{int(segment['warm_end']) - 1}"
            )
            subprocess.run(command, check=True)
            return out_path

        with concurrent.futures.ThreadPoolExecutor(max_workers=args.jobs) as executor:
            futures = [
                executor.submit(run_one, index, segment)
                for index, segment in enumerate(segments)
            ]
            for future in concurrent.futures.as_completed(futures):
                future.result()

        header_lines: list[str] = [
            "% LibGNSS++ Position Solution\n",
            "% Format: pos\n",
            "% Columns: GPS_Week GPS_TOW X(m) Y(m) Z(m) Lat(deg) Lon(deg) Height(m) Status Satellites PDOP\n",
            (
                f"% Segmented solve: segment_epochs={args.segment_epochs} "
                f"warmup_epochs={args.warmup_epochs} jobs={args.jobs}\n"
            ),
        ]
        merged_records: list[tuple[int, float, str]] = []
        seen_keys: set[tuple[int, float]] = set()

        for segment in segments:
            tow_start = float(segment["tow_start"])
            tow_end_exclusive = segment["tow_end_exclusive"]
            with Path(segment["path"]).open() as handle:
                for line in handle:
                    if not line.strip() or line.startswith("%"):
                        continue
                    parts = line.split()
                    if len(parts) < 2:
                        continue
                    week = int(float(parts[0]))
                    tow = float(parts[1])
                    if tow < tow_start - 1e-6:
                        continue
                    if tow_end_exclusive is not None and tow >= float(tow_end_exclusive) - 1e-6:
                        continue
                    key = (week, tow)
                    if key in seen_keys:
                        continue
                    merged_records.append((week, tow, line))
                    seen_keys.add(key)

        merged_records.sort(key=lambda row: (row[0], row[1]))
        args.lib_pos.parent.mkdir(parents=True, exist_ok=True)
        with args.lib_pos.open("w") as handle:
            handle.writelines(header_lines)
            for _, _, line in merged_records:
                handle.write(line if line.endswith("\n") else f"{line}\n")

    run_command(
        [
            *command_prefix,
            "pos2kml",
            str(args.lib_pos),
            str(args.lib_kml),
            "--name",
            args.scorecard_title,
        ]
    )


def main() -> int:
    args = parse_args()
    gnss_command = resolve_gnss_command(ROOT_DIR)

    ensure_exists(args.rover, "rover observation file")
    ensure_exists(args.base, "base observation file")
    ensure_exists(args.nav, "navigation file")
    ensure_exists(args.reference_csv, "reference csv")
    ensure_exists(args.rtklib_config, "RTKLIB config")
    if args.malib_bin:
        ensure_exists(args.malib_config, "MALIB config")
    if args.skip_epochs < 0:
        raise SystemExit("--skip-epochs must be >= 0")
    if args.max_epochs == 0:
        raise SystemExit("--max-epochs must be positive or -1")
    if args.segment_epochs < 0:
        raise SystemExit("--segment-epochs must be >= 0")
    if args.warmup_epochs < 0:
        raise SystemExit("--warmup-epochs must be >= 0")
    if args.jobs <= 0:
        raise SystemExit("--jobs must be > 0")

    rtklib_bin = Path(args.rtklib_bin)
    if not rtklib_bin.exists():
        raise SystemExit(f"Missing RTKLIB binary: {rtklib_bin}")
    malib_bin = Path(args.malib_bin) if args.malib_bin else None
    if malib_bin is not None and not malib_bin.exists():
        raise SystemExit(f"Missing MALIB binary: {malib_bin}")

    for output_path in (
        args.lib_pos,
        args.lib_kml,
        args.rtklib_pos,
        args.malib_pos,
        args.comparison_png,
        args.scorecard_png,
        args.summary_json,
    ):
        output_path.parent.mkdir(parents=True, exist_ok=True)

    partial_window = args.skip_epochs > 0 or args.max_epochs > 0

    if not partial_window and args.segment_epochs > 0:
        run_segmented_lib_solve(args, gnss_command)
    else:
        lib_command = [
            *gnss_command,
            "solve",
            "--rover",
            str(args.rover),
            "--base",
            str(args.base),
            "--nav",
            str(args.nav),
            "--out",
            str(args.lib_pos),
            "--mode",
            args.mode,
            "--glonass-ar",
            args.glonass_ar,
        ]
        if partial_window:
            lib_command.append("--no-kml")
        else:
            lib_command.extend(["--kml", str(args.lib_kml)])
        if args.skip_epochs > 0:
            lib_command.extend(["--skip-epochs", str(args.skip_epochs)])
        if args.max_epochs > 0:
            lib_command.extend(["--max-epochs", str(args.max_epochs)])
        run_command(lib_command)
    if partial_window:
        print("Skipping RTKLIB/comparison pipeline for partial epoch window.")
        print(f"  libgnss++: {args.lib_pos}")
        return 0

    run_command(
        [
            str(rtklib_bin),
            "-k",
            str(args.rtklib_config),
            "-o",
            str(args.rtklib_pos),
            str(args.rover),
            str(args.base),
            str(args.nav),
        ]
    )

    if malib_bin is not None:
        run_command(
            [
                str(malib_bin),
                "-k",
                str(args.malib_config),
                "-o",
                str(args.malib_pos),
                str(args.rover),
                str(args.base),
                str(args.nav),
            ]
        )

    run_command(
        [
            *gnss_command,
            "driving-compare",
            "--lib-pos",
            str(args.lib_pos),
            "--rtklib-pos",
            str(args.rtklib_pos),
            "--reference-csv",
            str(args.reference_csv),
            "--output",
            str(args.comparison_png),
            "--title",
            args.comparison_title,
        ]
    )

    run_command(
        [
            *gnss_command,
            "scorecard",
            "--lib-pos",
            str(args.lib_pos),
            "--rtklib-pos",
            str(args.rtklib_pos),
            "--reference-csv",
            str(args.reference_csv),
            "--output",
            str(args.scorecard_png),
            "--title",
            args.scorecard_title,
        ]
    )

    run_command(
        [
            *gnss_command,
            "social-card",
            "--lib-pos",
            str(args.lib_pos),
            "--rtklib-pos",
            str(args.rtklib_pos),
            "--reference-csv",
            str(args.reference_csv),
            "--output",
            str(args.social_card_png),
            "--rtklib-2d",
            str(driving_comparison.default_panel_output(args.comparison_png, "rtklib_2d")),
            "--lib-2d",
            str(driving_comparison.default_panel_output(args.comparison_png, "libgnss_2d")),
            "--title",
            args.social_card_title,
        ]
    )

    summary = write_summary_json(args)
    enforce_summary_requirements(summary, args)

    print("Finished Odaiba benchmark pipeline.")
    print(f"  libgnss++: {args.lib_pos}")
    print(f"  RTKLIB: {args.rtklib_pos}")
    if malib_bin is not None or args.malib_pos.exists():
        print(f"  MALIB: {args.malib_pos}")
    print(f"  comparison: {args.comparison_png}")
    print(f"  scorecard: {args.scorecard_png}")
    print(f"  social card: {args.social_card_png}")
    print(f"  summary: {args.summary_json}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
