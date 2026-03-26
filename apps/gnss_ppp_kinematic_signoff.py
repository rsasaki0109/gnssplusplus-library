#!/usr/bin/env python3
"""Run and validate a bundled kinematic PPP sign-off against an RTK reference."""

from __future__ import annotations

import argparse
from datetime import datetime, timedelta
import json
import math
import os
from pathlib import Path
import subprocess
import sys

from gnss_runtime import ensure_input_exists, resolve_gnss_command


ROOT_DIR = Path(__file__).resolve().parent.parent
PPP_STATUSES = {5, 6}
RTK_FIXED_STATUS = 4
GPS_EPOCH = datetime(1980, 1, 6)


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(prog=os.environ.get("GNSS_CLI_NAME"))
    parser.add_argument(
        "--obs",
        type=Path,
        default=ROOT_DIR / "data/rover_kinematic.obs",
        help="Rover RINEX observation file for PPP.",
    )
    parser.add_argument(
        "--base",
        type=Path,
        default=ROOT_DIR / "data/base_kinematic.obs",
        help="Base RINEX observation file for the RTK reference solution.",
    )
    parser.add_argument(
        "--nav",
        type=Path,
        default=ROOT_DIR / "data/navigation_kinematic.nav",
        help="Navigation RINEX file shared by PPP and RTK reference generation.",
    )
    parser.add_argument(
        "--out",
        type=Path,
        default=ROOT_DIR / "output/ppp_kinematic_solution.pos",
        help="Output .pos path for the PPP solution.",
    )
    parser.add_argument(
        "--reference-pos",
        type=Path,
        default=ROOT_DIR / "output/ppp_kinematic_reference.pos",
        help="Output or existing .pos path for the RTK reference trajectory.",
    )
    parser.add_argument(
        "--summary-json",
        type=Path,
        default=ROOT_DIR / "output/ppp_kinematic_summary.json",
        help="Output path for a machine-readable sign-off summary JSON.",
    )
    parser.add_argument(
        "--max-epochs",
        type=int,
        default=120,
        help="Stop after N epochs (default: 120). Use -1 for full dataset.",
    )
    parser.add_argument(
        "--use-existing-reference",
        action="store_true",
        help="Do not regenerate the RTK reference if --reference-pos already exists.",
    )
    parser.add_argument(
        "--require-common-epoch-pairs-min",
        type=int,
        default=None,
        help="Fail if matched PPP/reference epoch pairs are below this count.",
    )
    parser.add_argument(
        "--require-reference-fix-rate-min",
        type=float,
        default=None,
        help="Fail if the RTK reference fixed-rate percentage is below this value.",
    )
    parser.add_argument(
        "--require-mean-error-max",
        type=float,
        default=None,
        help="Fail if mean 3D PPP-vs-reference error exceeds this value in meters.",
    )
    parser.add_argument(
        "--require-p95-error-max",
        type=float,
        default=None,
        help="Fail if p95 3D PPP-vs-reference error exceeds this value in meters.",
    )
    parser.add_argument(
        "--require-max-error-max",
        type=float,
        default=None,
        help="Fail if max 3D PPP-vs-reference error exceeds this value in meters.",
    )
    parser.add_argument(
        "--require-mean-sats-min",
        type=float,
        default=None,
        help="Fail if mean PPP used-satellite count is below this value.",
    )
    parser.add_argument(
        "--require-ppp-solution-rate-min",
        type=float,
        default=None,
        help="Fail if PPP_FLOAT/PPP_FIXED epochs are below this percentage of valid PPP epochs.",
    )
    parser.add_argument(
        "--malib-bin",
        type=Path,
        default=None,
        help="Optional path to the MALIB rnx2rtkp binary for side-by-side PPP comparison.",
    )
    parser.add_argument(
        "--malib-config",
        type=Path,
        default=ROOT_DIR / "scripts/malib_ppp_kinematic.conf",
        help="MALIB PPP-kinematic configuration file used with --malib-bin.",
    )
    parser.add_argument(
        "--malib-pos",
        type=Path,
        default=ROOT_DIR / "output/malib_ppp_kinematic_solution.pos",
        help="Output .pos path for the optional MALIB PPP solution.",
    )
    parser.add_argument(
        "--use-existing-malib",
        action="store_true",
        help="Do not rerun MALIB if --malib-pos already exists.",
    )
    parser.set_defaults(low_dynamics=True)
    parser.add_argument(
        "--low-dynamics",
        dest="low_dynamics",
        action="store_true",
        help="Run PPP with the quasi-static low-dynamics profile (default for the bundled sample).",
    )
    parser.add_argument(
        "--no-low-dynamics",
        dest="low_dynamics",
        action="store_false",
        help="Disable the quasi-static low-dynamics profile.",
    )
    return parser.parse_args()


def run_command(command: list[str]) -> None:
    print("+", " ".join(command))
    subprocess.run(command, check=True)


def read_pos_records(path: Path) -> list[dict[str, float | int]]:
    records: list[dict[str, float | int]] = []
    with path.open(encoding="ascii") as handle:
        for line in handle:
            if not line.strip() or line.startswith("%"):
                continue
            parts = line.split()
            if len(parts) < 10:
                continue
            records.append(
                {
                    "week": int(float(parts[0])),
                    "tow": float(parts[1]),
                    "x": float(parts[2]),
                    "y": float(parts[3]),
                    "z": float(parts[4]),
                    "status": int(parts[8]),
                    "satellites": int(parts[9]),
                }
            )
    return records


def read_malib_xyz_records(path: Path) -> list[dict[str, float | int]]:
    records: list[dict[str, float | int]] = []
    with path.open(encoding="ascii") as handle:
        for line in handle:
            if not line.strip() or line.startswith("%"):
                continue
            parts = line.split()
            if len(parts) < 7:
                continue
            stamp = datetime.strptime(
                f"{parts[0]} {parts[1]}",
                "%Y/%m/%d %H:%M:%S.%f",
            )
            delta = stamp - GPS_EPOCH
            records.append(
                {
                    "week": delta.days // 7,
                    "tow": round(
                        (delta.days % 7) * 86400
                        + delta.seconds
                        + delta.microseconds / 1e6,
                        3,
                    ),
                    "x": float(parts[2]),
                    "y": float(parts[3]),
                    "z": float(parts[4]),
                    "status": int(parts[5]),
                    "satellites": int(parts[6]),
                }
            )
    return records


def gps_week_tow_to_gpst_tokens(week: int, tow: float) -> tuple[str, str]:
    stamp = GPS_EPOCH + timedelta(weeks=week, seconds=tow)
    return stamp.strftime("%Y/%m/%d"), stamp.strftime("%H:%M:%S.%f")[:-3]


def rounded(value: float) -> float:
    return round(value, 6)


def percentile(sorted_values: list[float], fraction: float) -> float:
    if not sorted_values:
        raise SystemExit("Cannot compute percentile from an empty sample")
    index = max(0, math.ceil(len(sorted_values) * fraction) - 1)
    return sorted_values[index]


def build_summary_payload(args: argparse.Namespace) -> dict[str, object]:
    ppp_records = read_pos_records(args.out)
    reference_records = read_pos_records(args.reference_pos)
    if not ppp_records:
        raise SystemExit(f"No PPP solution epochs found in {args.out}")
    if not reference_records:
        raise SystemExit(f"No reference solution epochs found in {args.reference_pos}")

    reference_by_epoch = {
        (int(record["week"]), round(float(record["tow"]), 3)): record
        for record in reference_records
    }
    paired_errors: list[float] = []
    paired_ppp_satellites: list[int] = []
    paired_reference_satellites: list[int] = []
    for record in ppp_records:
        key = (int(record["week"]), round(float(record["tow"]), 3))
        reference = reference_by_epoch.get(key)
        if reference is None:
            continue
        paired_errors.append(
            math.dist(
                (
                    float(record["x"]),
                    float(record["y"]),
                    float(record["z"]),
                ),
                (
                    float(reference["x"]),
                    float(reference["y"]),
                    float(reference["z"]),
                ),
            )
        )
        paired_ppp_satellites.append(int(record["satellites"]))
        paired_reference_satellites.append(int(reference["satellites"]))

    if not paired_errors:
        raise SystemExit("No common PPP/reference epochs found for kinematic sign-off")

    paired_errors.sort()
    ppp_float_epochs = sum(1 for record in ppp_records if int(record["status"]) == 5)
    ppp_fixed_epochs = sum(1 for record in ppp_records if int(record["status"]) == 6)
    ppp_epochs = sum(1 for record in ppp_records if int(record["status"]) in PPP_STATUSES)
    fallback_epochs = len(ppp_records) - ppp_epochs
    reference_fixed_epochs = sum(
        1 for record in reference_records if int(record["status"]) == RTK_FIXED_STATUS
    )

    payload = {
        "dataset": "sample kinematic PPP",
        "obs": str(args.obs),
        "base": str(args.base),
        "nav": str(args.nav),
        "solution_pos": str(args.out),
        "reference_pos": str(args.reference_pos),
        "ppp_profile": "low_dynamics" if getattr(args, "low_dynamics", False) else "kinematic",
        "epochs": len(ppp_records),
        "reference_epochs": len(reference_records),
        "common_epoch_pairs": len(paired_errors),
        "ppp_float_epochs": ppp_float_epochs,
        "ppp_fixed_epochs": ppp_fixed_epochs,
        "fallback_epochs": fallback_epochs,
        "ppp_solution_rate_pct": rounded(100.0 * ppp_epochs / len(ppp_records)),
        "reference_fixed_epochs": reference_fixed_epochs,
        "reference_fix_rate_pct": rounded(
            100.0 * reference_fixed_epochs / len(reference_records)
        ),
        "mean_position_error_m": rounded(sum(paired_errors) / len(paired_errors)),
        "median_position_error_m": rounded(percentile(paired_errors, 0.5)),
        "p95_position_error_m": rounded(percentile(paired_errors, 0.95)),
        "max_position_error_m": rounded(max(paired_errors)),
        "mean_satellites": rounded(
            sum(paired_ppp_satellites) / len(paired_ppp_satellites)
        ),
        "mean_reference_satellites": rounded(
            sum(paired_reference_satellites) / len(paired_reference_satellites)
        ),
    }
    malib_pos = getattr(args, "malib_pos", None)
    if malib_pos is not None and Path(malib_pos).exists():
        malib_records = read_malib_xyz_records(Path(malib_pos))
        if not malib_records:
            raise SystemExit(f"No MALIB solution epochs found in {malib_pos}")
        malib_errors: list[float] = []
        malib_satellites: list[int] = []
        for record in malib_records:
            key = (int(record["week"]), round(float(record["tow"]), 3))
            reference = reference_by_epoch.get(key)
            if reference is None:
                continue
            malib_errors.append(
                math.dist(
                    (
                        float(record["x"]),
                        float(record["y"]),
                        float(record["z"]),
                    ),
                    (
                        float(reference["x"]),
                        float(reference["y"]),
                        float(reference["z"]),
                    ),
                )
            )
            malib_satellites.append(int(record["satellites"]))
        if not malib_errors:
            raise SystemExit("No common MALIB/reference epochs found for kinematic sign-off")
        malib_errors.sort()
        payload.update(
            {
                "malib_solution_pos": str(malib_pos),
                "malib_epochs": len(malib_records),
                "malib_common_epoch_pairs": len(malib_errors),
                "malib_mean_position_error_m": rounded(
                    sum(malib_errors) / len(malib_errors)
                ),
                "malib_median_position_error_m": rounded(
                    percentile(malib_errors, 0.5)
                ),
                "malib_p95_position_error_m": rounded(
                    percentile(malib_errors, 0.95)
                ),
                "malib_max_position_error_m": rounded(max(malib_errors)),
                "malib_mean_satellites": rounded(
                    sum(malib_satellites) / len(malib_satellites)
                ),
            }
        )
        payload["libgnss_minus_malib_mean_error_m"] = rounded(
            float(payload["mean_position_error_m"])
            - float(payload["malib_mean_position_error_m"])
        )
        payload["libgnss_minus_malib_p95_error_m"] = rounded(
            float(payload["p95_position_error_m"])
            - float(payload["malib_p95_position_error_m"])
        )
        payload["libgnss_minus_malib_max_error_m"] = rounded(
            float(payload["max_position_error_m"])
            - float(payload["malib_max_position_error_m"])
        )
    args.summary_json.parent.mkdir(parents=True, exist_ok=True)
    args.summary_json.write_text(
        json.dumps(payload, indent=2, sort_keys=True) + "\n",
        encoding="utf-8",
    )
    return payload


def enforce_summary_requirements(payload: dict[str, object], args: argparse.Namespace) -> None:
    failures: list[str] = []

    if (
        args.require_common_epoch_pairs_min is not None
        and int(payload["common_epoch_pairs"]) < args.require_common_epoch_pairs_min
    ):
        failures.append(
            f"common epoch pairs {int(payload['common_epoch_pairs'])} < "
            f"{args.require_common_epoch_pairs_min}"
        )
    if (
        args.require_reference_fix_rate_min is not None
        and float(payload["reference_fix_rate_pct"]) < args.require_reference_fix_rate_min
    ):
        failures.append(
            f"reference fix rate {float(payload['reference_fix_rate_pct']):.6f}% < "
            f"{args.require_reference_fix_rate_min:.6f}%"
        )
    if (
        args.require_mean_error_max is not None
        and float(payload["mean_position_error_m"]) > args.require_mean_error_max
    ):
        failures.append(
            f"mean position error {float(payload['mean_position_error_m']):.6f} m > "
            f"{args.require_mean_error_max:.6f} m"
        )
    if (
        args.require_p95_error_max is not None
        and float(payload["p95_position_error_m"]) > args.require_p95_error_max
    ):
        failures.append(
            f"p95 position error {float(payload['p95_position_error_m']):.6f} m > "
            f"{args.require_p95_error_max:.6f} m"
        )
    if (
        args.require_max_error_max is not None
        and float(payload["max_position_error_m"]) > args.require_max_error_max
    ):
        failures.append(
            f"max position error {float(payload['max_position_error_m']):.6f} m > "
            f"{args.require_max_error_max:.6f} m"
        )
    if (
        args.require_mean_sats_min is not None
        and float(payload["mean_satellites"]) < args.require_mean_sats_min
    ):
        failures.append(
            f"mean satellites {float(payload['mean_satellites']):.6f} < "
            f"{args.require_mean_sats_min:.6f}"
        )
    if (
        args.require_ppp_solution_rate_min is not None
        and float(payload["ppp_solution_rate_pct"]) < args.require_ppp_solution_rate_min
    ):
        failures.append(
            f"PPP solution rate {float(payload['ppp_solution_rate_pct']):.6f}% < "
            f"{args.require_ppp_solution_rate_min:.6f}%"
        )

    if failures:
        raise SystemExit(
            "PPP kinematic sign-off checks failed:\n"
            + "\n".join(f"  - {failure}" for failure in failures)
        )


def main() -> int:
    args = parse_args()
    gnss_command = resolve_gnss_command(ROOT_DIR)

    ensure_input_exists(args.obs, "observation file", ROOT_DIR)
    ensure_input_exists(args.base, "base observation file", ROOT_DIR)
    ensure_input_exists(args.nav, "navigation file", ROOT_DIR)
    if args.malib_bin is not None:
        ensure_input_exists(args.malib_bin, "MALIB binary", ROOT_DIR)
        ensure_input_exists(args.malib_config, "MALIB config", ROOT_DIR)
    if args.max_epochs == 0:
        raise SystemExit("--max-epochs must be positive or -1")

    args.out.parent.mkdir(parents=True, exist_ok=True)
    args.reference_pos.parent.mkdir(parents=True, exist_ok=True)
    args.summary_json.parent.mkdir(parents=True, exist_ok=True)
    args.malib_pos.parent.mkdir(parents=True, exist_ok=True)

    if not args.use_existing_reference or not args.reference_pos.exists():
        solve_command = [
            *gnss_command,
            "solve",
            "--rover",
            str(args.obs),
            "--base",
            str(args.base),
            "--nav",
            str(args.nav),
            "--out",
            str(args.reference_pos),
            "--mode",
            "kinematic",
            "--no-kml",
        ]
        if args.max_epochs > 0:
            solve_command.extend(["--max-epochs", str(args.max_epochs)])
        run_command(solve_command)

    ppp_command = [
        *gnss_command,
        "ppp",
        "--kinematic",
        *(["--low-dynamics"] if args.low_dynamics else []),
        "--obs",
        str(args.obs),
        "--nav",
        str(args.nav),
        "--out",
        str(args.out),
    ]
    if args.max_epochs > 0:
        ppp_command.extend(["--max-epochs", str(args.max_epochs)])
    run_command(ppp_command)

    if args.malib_bin is not None and (
        not args.use_existing_malib or not args.malib_pos.exists()
    ):
        lib_records = read_pos_records(args.out)
        start_date, start_time = gps_week_tow_to_gpst_tokens(
            int(lib_records[0]["week"]),
            float(lib_records[0]["tow"]),
        )
        end_date, end_time = gps_week_tow_to_gpst_tokens(
            int(lib_records[-1]["week"]),
            float(lib_records[-1]["tow"]),
        )
        malib_command = [
            str(args.malib_bin),
            "-k",
            str(args.malib_config),
            "-ts",
            start_date,
            start_time,
            "-te",
            end_date,
            end_time,
            "-o",
            str(args.malib_pos),
            str(args.obs),
            str(args.nav),
        ]
        run_command(malib_command)

    payload = build_summary_payload(args)
    enforce_summary_requirements(payload, args)

    print("Finished PPP kinematic sign-off.")
    print(f"  solution: {args.out}")
    print(f"  reference: {args.reference_pos}")
    if args.malib_pos.exists():
        print(f"  MALIB: {args.malib_pos}")
    print(f"  summary: {args.summary_json}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
