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
import tempfile

from gnss_runtime import ensure_input_exists, resolve_gnss_command, run_fetch_products


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
        "--sp3",
        type=Path,
        default=None,
        help="Optional precise SP3 orbit file for the PPP solution.",
    )
    parser.add_argument(
        "--clk",
        type=Path,
        default=None,
        help="Optional precise CLK file for the PPP solution.",
    )
    parser.add_argument(
        "--ionex",
        type=Path,
        default=None,
        help="Optional IONEX TEC map product for the PPP solution.",
    )
    parser.add_argument(
        "--dcb",
        type=Path,
        default=None,
        help="Optional DCB / Bias-SINEX product for the PPP solution.",
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
        "--fetch-products",
        action="store_true",
        help="Fetch SP3/CLK-style products through gnss fetch-products before running PPP.",
    )
    parser.add_argument(
        "--preset",
        action="append",
        default=[],
        help="Product preset passed through to gnss fetch-products. May be repeated.",
    )
    parser.add_argument(
        "--product",
        action="append",
        default=[],
        metavar="KIND=SOURCE",
        help="Product template passed through to gnss fetch-products. May be repeated.",
    )
    parser.add_argument(
        "--product-date",
        default=None,
        help="Optional YYYY-MM-DD date for --fetch-products. Defaults to TIME OF FIRST OBS.",
    )
    parser.add_argument(
        "--product-cache-dir",
        type=Path,
        default=None,
        help="Optional cache directory for gnss fetch-products.",
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
        "--require-converged",
        action="store_true",
        help="Fail unless gnss ppp reports that the run converged.",
    )
    parser.add_argument(
        "--require-convergence-time-max",
        type=float,
        default=None,
        help="Fail if gnss ppp convergence time exceeds this value in seconds.",
    )
    parser.add_argument(
        "--require-ionex-corrections-min",
        type=int,
        default=None,
        help="Fail if gnss ppp reports fewer IONEX-applied corrections than this value.",
    )
    parser.add_argument(
        "--require-dcb-corrections-min",
        type=int,
        default=None,
        help="Fail if gnss ppp reports fewer DCB-applied corrections than this value.",
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
        "sp3": str(getattr(args, "resolved_sp3", None)) if getattr(args, "resolved_sp3", None) is not None else None,
        "clk": str(getattr(args, "resolved_clk", None)) if getattr(args, "resolved_clk", None) is not None else None,
        "ionex": str(getattr(args, "resolved_ionex", None)) if getattr(args, "resolved_ionex", None) is not None else None,
        "dcb": str(getattr(args, "resolved_dcb", None)) if getattr(args, "resolved_dcb", None) is not None else None,
        "fetch_products": bool(getattr(args, "fetch_products", False)),
        "fetched_products": getattr(args, "fetched_products", None),
        "fetched_product_date": getattr(args, "fetched_product_date", None),
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
    ppp_run_summary = getattr(args, "ppp_run_summary", None)
    if isinstance(ppp_run_summary, dict):
        payload.update(
            {
                "ppp_run_summary": ppp_run_summary,
                "ppp_converged": bool(ppp_run_summary.get("converged", False)),
                "ppp_convergence_time_s": rounded(
                    float(ppp_run_summary.get("convergence_time_s", 0.0))
                ),
                "ionex_corrections": int(ppp_run_summary.get("ionex_corrections", 0)),
                "ionex_meters": rounded(float(ppp_run_summary.get("ionex_meters", 0.0))),
                "dcb_corrections": int(ppp_run_summary.get("dcb_corrections", 0)),
                "dcb_meters": rounded(float(ppp_run_summary.get("dcb_meters", 0.0))),
            }
        )
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
    if getattr(args, "require_converged", False) and not bool(payload.get("ppp_converged", False)):
        failures.append("PPP run did not report convergence")
    if (
        getattr(args, "require_convergence_time_max", None) is not None
        and float(payload.get("ppp_convergence_time_s", 0.0)) > args.require_convergence_time_max
    ):
        failures.append(
            f"PPP convergence time {float(payload.get('ppp_convergence_time_s', 0.0)):.6f} s > "
            f"{args.require_convergence_time_max:.6f} s"
        )
    if (
        getattr(args, "require_ionex_corrections_min", None) is not None
        and int(payload.get("ionex_corrections", 0)) < args.require_ionex_corrections_min
    ):
        failures.append(
            f"IONEX corrections {int(payload.get('ionex_corrections', 0))} < "
            f"{args.require_ionex_corrections_min}"
        )
    if (
        getattr(args, "require_dcb_corrections_min", None) is not None
        and int(payload.get("dcb_corrections", 0)) < args.require_dcb_corrections_min
    ):
        failures.append(
            f"DCB corrections {int(payload.get('dcb_corrections', 0))} < "
            f"{args.require_dcb_corrections_min}"
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
    if args.sp3 is not None:
        ensure_input_exists(args.sp3, "SP3 file", ROOT_DIR)
    if args.clk is not None:
        ensure_input_exists(args.clk, "CLK file", ROOT_DIR)
    if args.malib_bin is not None:
        ensure_input_exists(args.malib_bin, "MALIB binary", ROOT_DIR)
        ensure_input_exists(args.malib_config, "MALIB config", ROOT_DIR)
    if args.max_epochs == 0:
        raise SystemExit("--max-epochs must be positive or -1")
    if args.fetch_products and not args.preset and not args.product:
        raise SystemExit("--fetch-products requires at least one --preset or --product KIND=SOURCE")

    args.out.parent.mkdir(parents=True, exist_ok=True)
    args.reference_pos.parent.mkdir(parents=True, exist_ok=True)
    args.summary_json.parent.mkdir(parents=True, exist_ok=True)
    args.malib_pos.parent.mkdir(parents=True, exist_ok=True)
    args.fetched_products = None
    args.fetched_product_date = None

    resolved_sp3 = args.sp3
    resolved_clk = args.clk
    resolved_ionex = args.ionex
    resolved_dcb = args.dcb
    if args.fetch_products:
        fetch_payload = run_fetch_products(
            ROOT_DIR,
            args.obs,
            args.preset,
            args.product,
            product_date_text=args.product_date,
            cache_dir=args.product_cache_dir,
        )
        fetched_products = fetch_payload.get("products", {})
        if isinstance(fetched_products, dict):
            args.fetched_products = fetched_products
            args.fetched_product_date = fetch_payload.get("effective_date")
            if resolved_sp3 is None and "sp3" in fetched_products:
                resolved_sp3 = Path(str(fetched_products["sp3"]))
            if resolved_clk is None and "clk" in fetched_products:
                resolved_clk = Path(str(fetched_products["clk"]))
            if resolved_ionex is None and "ionex" in fetched_products:
                resolved_ionex = Path(str(fetched_products["ionex"]))
            if resolved_dcb is None and "dcb" in fetched_products:
                resolved_dcb = Path(str(fetched_products["dcb"]))
    args.resolved_sp3 = resolved_sp3
    args.resolved_clk = resolved_clk
    args.resolved_ionex = resolved_ionex
    args.resolved_dcb = resolved_dcb
    ppp_run_summary_path = Path(tempfile.mkdtemp(prefix="gnss_ppp_kinematic_signoff_run_")) / "ppp_summary.json"

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
        "--summary-json",
        str(ppp_run_summary_path),
    ]
    if resolved_sp3 is not None:
        ppp_command.extend(["--sp3", str(resolved_sp3)])
    if resolved_clk is not None:
        ppp_command.extend(["--clk", str(resolved_clk)])
    if resolved_ionex is not None:
        ppp_command.extend(["--ionex", str(resolved_ionex)])
    if resolved_dcb is not None:
        ppp_command.extend(["--dcb", str(resolved_dcb)])
    if args.max_epochs > 0:
        ppp_command.extend(["--max-epochs", str(args.max_epochs)])
    try:
        run_command(ppp_command)
        args.ppp_run_summary = json.loads(ppp_run_summary_path.read_text(encoding="utf-8"))
    finally:
        try:
            ppp_run_summary_path.unlink()
            ppp_run_summary_path.parent.rmdir()
        except OSError:
            pass

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
