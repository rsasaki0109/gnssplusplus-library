#!/usr/bin/env python3
"""Run PPP sign-off with fetched real-product style inputs."""

from __future__ import annotations

import argparse
import csv
import json
import math
import os
from pathlib import Path
import subprocess
import tempfile

import gnss_ppc_demo as ppc_demo
from gnss_toml_config import parse_args_with_toml
from gnss_runtime import ensure_input_exists, resolve_gnss_command, run_fetch_products


ROOT_DIR = Path(__file__).resolve().parent.parent
DEFAULT_PRESETS = ["igs-final", "ionex", "dcb"]


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(prog=os.environ.get("GNSS_CLI_NAME"))
    parser.add_argument(
        "--config-toml",
        type=Path,
        default=None,
        help="Optional TOML config. Uses [ppp_products_signoff] or top-level keys.",
    )
    parser.add_argument("--profile", choices=("static", "kinematic", "ppc"), default="static")
    parser.add_argument("--obs", type=Path, default=None)
    parser.add_argument("--base", type=Path, default=None)
    parser.add_argument("--nav", type=Path, default=None)
    parser.add_argument("--dataset-root", type=Path, default=None)
    parser.add_argument("--city", choices=("tokyo", "nagoya"), default=None)
    parser.add_argument("--run", default="run1")
    parser.add_argument("--run-dir", type=Path, default=None)
    parser.add_argument("--reference-csv", type=Path, default=None)
    parser.add_argument("--out", type=Path, default=None)
    parser.add_argument("--summary-json", type=Path, default=None)
    parser.add_argument("--reference-pos", type=Path, default=None)
    parser.add_argument("--comparison-csv", type=Path, default=None)
    parser.add_argument("--comparison-png", type=Path, default=None)
    parser.add_argument("--max-epochs", type=int, default=120)
    parser.add_argument("--match-tolerance-s", type=float, default=0.25)
    parser.add_argument("--use-existing-solution", action="store_true")
    parser.add_argument("--solver-wall-time-s", type=float, default=None)
    parser.add_argument("--ppp-run-summary-json", type=Path, default=None)
    parser.add_argument("--preset", action="append", default=[])
    parser.add_argument("--product", action="append", default=[], metavar="KIND=SOURCE")
    parser.add_argument("--product-date", default=None)
    parser.add_argument("--product-cache-dir", type=Path, default=None)
    parser.add_argument("--enable-ar", action="store_true")
    parser.add_argument("--ar-ratio-threshold", type=float, default=3.0)
    parser.add_argument("--low-dynamics", dest="low_dynamics", action="store_true")
    parser.add_argument("--no-low-dynamics", dest="low_dynamics", action="store_false")
    parser.add_argument("--malib-bin", type=Path, default=None)
    parser.add_argument("--malib-config", type=Path, default=None)
    parser.add_argument("--malib-pos", type=Path, default=None)
    parser.add_argument("--use-existing-malib", action="store_true")
    parser.add_argument("--use-existing-reference", action="store_true")
    parser.add_argument("--require-converged", action="store_true")
    parser.add_argument("--require-convergence-time-max", type=float, default=None)
    parser.add_argument("--require-ionex-corrections-min", type=int, default=None)
    parser.add_argument("--require-dcb-corrections-min", type=int, default=None)
    parser.add_argument("--require-ppp-solution-rate-min", type=float, default=None)
    parser.add_argument("--require-valid-epochs-min", type=int, default=None)
    parser.add_argument("--require-matched-epochs-min", type=int, default=None)
    parser.add_argument("--require-common-epoch-pairs-min", type=int, default=None)
    parser.add_argument("--require-reference-fix-rate-min", type=float, default=None)
    parser.add_argument("--require-mean-error-max", type=float, default=None)
    parser.add_argument("--require-p95-error-max", type=float, default=None)
    parser.add_argument("--require-max-error-max", type=float, default=None)
    parser.add_argument("--require-mean-sats-min", type=float, default=None)
    parser.add_argument("--require-lib-mean-error-vs-malib-max-delta", type=float, default=None)
    parser.add_argument("--require-lib-p95-error-vs-malib-max-delta", type=float, default=None)
    parser.add_argument("--require-lib-max-error-vs-malib-max-delta", type=float, default=None)
    parser.set_defaults(low_dynamics=True)
    return parse_args_with_toml(parser, "ppp_products_signoff")


def default_paths(profile: str) -> tuple[Path, Path, Path | None]:
    if profile == "static":
        return (
            ROOT_DIR / "output/ppp_static_products_solution.pos",
            ROOT_DIR / "output/ppp_static_products_summary.json",
            None,
        )
    if profile == "ppc":
        return (
            ROOT_DIR / "output/ppp_ppc_products_solution.pos",
            ROOT_DIR / "output/ppp_ppc_products_summary.json",
            None,
        )
    return (
        ROOT_DIR / "output/ppp_kinematic_products_solution.pos",
        ROOT_DIR / "output/ppp_kinematic_products_summary.json",
        ROOT_DIR / "output/ppp_kinematic_products_reference.pos",
    )


def default_comparison_artifacts(profile: str) -> tuple[Path, Path]:
    if profile == "ppc":
        stem = "ppp_ppc_products_comparison"
    elif profile == "static":
        stem = "ppp_static_products_comparison"
    else:
        stem = "ppp_kinematic_products_comparison"
    return ROOT_DIR / "output" / f"{stem}.csv", ROOT_DIR / "output" / f"{stem}.png"


def gps_week_tow_to_gpst_tokens(week: int, tow_s: float) -> tuple[str, str]:
    from datetime import datetime, timedelta

    gps_epoch = datetime(1980, 1, 6)
    stamp = gps_epoch + timedelta(weeks=week, seconds=tow_s)
    return stamp.strftime("%Y/%m/%d"), stamp.strftime("%H:%M:%S")


def run_checked(command: list[str]) -> None:
    print("+", " ".join(command))
    subprocess.run(command, cwd=ROOT_DIR, check=True)


def read_lib_pos_records(path: Path) -> list[dict[str, float | int]]:
    records: list[dict[str, float | int]] = []
    with path.open("r", encoding="ascii") as handle:
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
                    "status": int(parts[8]),
                }
            )
    return records


def read_lib_xyz_records(path: Path) -> list[dict[str, float | int]]:
    records: list[dict[str, float | int]] = []
    with path.open("r", encoding="ascii") as handle:
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
    from datetime import datetime, timedelta

    gps_epoch = datetime(1980, 1, 6)
    records: list[dict[str, float | int]] = []
    with path.open("r", encoding="ascii") as handle:
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
            delta: timedelta = stamp - gps_epoch
            records.append(
                {
                    "week": delta.days // 7,
                    "tow": round(
                        (delta.days % 7) * 86400 + delta.seconds + delta.microseconds / 1e6,
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


def resolve_ppc_nav_path(args: argparse.Namespace, run_dir: Path, *, require_exists: bool) -> Path:
    nav_path = args.nav or (run_dir / "base.nav")
    if require_exists:
        ensure_input_exists(nav_path, "PPC navigation file", ROOT_DIR)
    return nav_path


def run_ppc_malib_if_needed(
    args: argparse.Namespace,
    rover_obs: Path,
    nav_path: Path,
    solution_pos: Path,
) -> Path | None:
    if args.malib_bin is None:
        return args.malib_pos
    if args.malib_config is None:
        raise SystemExit("--malib-bin requires --malib-config")
    ensure_input_exists(args.malib_bin, "MALIB binary", ROOT_DIR)
    ensure_input_exists(args.malib_config, "MALIB config", ROOT_DIR)
    if args.malib_pos is None:
        raise SystemExit("--malib-bin requires --malib-pos")
    if args.use_existing_malib and args.malib_pos.exists():
        return args.malib_pos

    lib_records = read_lib_pos_records(solution_pos)
    if not lib_records:
        raise SystemExit(f"No epochs found in libgnss++ PPP solution: {solution_pos}")
    start_date, start_time = gps_week_tow_to_gpst_tokens(
        int(lib_records[0]["week"]),
        float(lib_records[0]["tow"]),
    )
    end_date, end_time = gps_week_tow_to_gpst_tokens(
        int(lib_records[-1]["week"]),
        float(lib_records[-1]["tow"]),
    )
    args.malib_pos.parent.mkdir(parents=True, exist_ok=True)
    run_checked(
        [
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
            str(rover_obs),
            str(nav_path),
        ]
    )
    return args.malib_pos


def classify_comparison_status(*deltas: object) -> str | None:
    numeric = [float(value) for value in deltas if isinstance(value, (int, float))]
    if not numeric:
        return None
    worst = max(numeric)
    if worst <= 0.0:
        return "better"
    if worst <= 0.25:
        return "close"
    return "worse"


def enforce_comparison_requirements(payload: dict[str, object], args: argparse.Namespace) -> None:
    failures: list[str] = []
    checks = [
        ("libgnss_minus_malib_mean_error_m", args.require_lib_mean_error_vs_malib_max_delta, "mean"),
        ("libgnss_minus_malib_p95_error_m", args.require_lib_p95_error_vs_malib_max_delta, "p95"),
        ("libgnss_minus_malib_max_error_m", args.require_lib_max_error_vs_malib_max_delta, "max"),
    ]
    for field, threshold, label in checks:
        if threshold is None:
            continue
        value = payload.get(field)
        if not isinstance(value, (int, float)):
            failures.append(f"missing MALIB comparison metric for {label} error")
            continue
        if float(value) > float(threshold):
            failures.append(
                f"libgnss++ {label} error delta {float(value):.6f} m > {float(threshold):.6f} m"
            )
    if failures:
        raise SystemExit("PPP products comparison checks failed:\n  - " + "\n  - ".join(failures))


def enforce_ppp_requirements(payload: dict[str, object], args: argparse.Namespace) -> None:
    failures: list[str] = []
    if args.require_converged and not bool(payload.get("ppp_converged", False)):
        failures.append("PPP run did not report convergence")
    if (
        args.require_convergence_time_max is not None
        and float(payload.get("ppp_convergence_time_s", 0.0)) > args.require_convergence_time_max
    ):
        failures.append(
            f"PPP convergence time {float(payload.get('ppp_convergence_time_s', 0.0)):.6f} s > "
            f"{args.require_convergence_time_max:.6f} s"
        )
    if (
        args.require_ionex_corrections_min is not None
        and int(payload.get("ionex_corrections", 0)) < args.require_ionex_corrections_min
    ):
        failures.append(
            f"IONEX corrections {int(payload.get('ionex_corrections', 0))} < {args.require_ionex_corrections_min}"
        )
    if (
        args.require_dcb_corrections_min is not None
        and int(payload.get("dcb_corrections", 0)) < args.require_dcb_corrections_min
    ):
        failures.append(
            f"DCB corrections {int(payload.get('dcb_corrections', 0))} < {args.require_dcb_corrections_min}"
        )
    if (
        args.require_ppp_solution_rate_min is not None
        and float(payload.get("ppp_solution_rate_pct", 0.0)) < args.require_ppp_solution_rate_min
    ):
        failures.append(
            f"PPP solution rate {float(payload.get('ppp_solution_rate_pct', 0.0)):.6f}% < "
            f"{args.require_ppp_solution_rate_min:.6f}%"
        )
    if failures:
        raise SystemExit("PPP products checks failed:\n  - " + "\n  - ".join(failures))


def enforce_ppc_metric_requirements(payload: dict[str, object], args: argparse.Namespace) -> None:
    failures: list[str] = []
    if args.require_valid_epochs_min is not None and int(payload.get("valid_epochs", 0)) < args.require_valid_epochs_min:
        failures.append(f"valid epochs {int(payload.get('valid_epochs', 0))} < {args.require_valid_epochs_min}")
    if args.require_matched_epochs_min is not None and int(payload.get("matched_epochs", 0)) < args.require_matched_epochs_min:
        failures.append(f"matched epochs {int(payload.get('matched_epochs', 0))} < {args.require_matched_epochs_min}")
    if (
        args.require_common_epoch_pairs_min is not None
        and int(payload.get("common_epoch_pairs", 0)) < args.require_common_epoch_pairs_min
    ):
        failures.append(
            f"common epoch pairs {int(payload.get('common_epoch_pairs', 0))} < {args.require_common_epoch_pairs_min}"
        )
    if (
        args.require_mean_error_max is not None
        and float(payload.get("mean_position_error_m", 0.0)) > args.require_mean_error_max
    ):
        failures.append(
            f"mean position error {float(payload.get('mean_position_error_m', 0.0)):.6f} m > "
            f"{args.require_mean_error_max:.6f} m"
        )
    if (
        args.require_p95_error_max is not None
        and float(payload.get("p95_position_error_m", 0.0)) > args.require_p95_error_max
    ):
        failures.append(
            f"p95 position error {float(payload.get('p95_position_error_m', 0.0)):.6f} m > "
            f"{args.require_p95_error_max:.6f} m"
        )
    if (
        args.require_max_error_max is not None
        and float(payload.get("max_position_error_m", 0.0)) > args.require_max_error_max
    ):
        failures.append(
            f"max position error {float(payload.get('max_position_error_m', 0.0)):.6f} m > "
            f"{args.require_max_error_max:.6f} m"
        )
    if (
        args.require_mean_sats_min is not None
        and float(payload.get("mean_satellites", 0.0)) < args.require_mean_sats_min
    ):
        failures.append(
            f"mean satellites {float(payload.get('mean_satellites', 0.0)):.6f} < {args.require_mean_sats_min:.6f}"
        )
    if failures:
        raise SystemExit("PPP products PPC checks failed:\n  - " + "\n  - ".join(failures))


def run_checked_json(command: list[str], summary_path: Path) -> dict[str, object]:
    run_checked(command)
    payload = json.loads(summary_path.read_text(encoding="utf-8"))
    if not isinstance(payload, dict):
        raise SystemExit("Expected JSON object summary payload")
    return payload


def resolve_ppc_run_dir(args: argparse.Namespace) -> Path:
    if args.run_dir is not None:
        return args.run_dir
    if args.dataset_root is None or args.city is None:
        raise SystemExit("--profile ppc requires --run-dir or both --dataset-root and --city")
    return args.dataset_root / args.city / args.run


def augment_ppc_malib_comparison(
    payload: dict[str, object],
    reference_csv: Path,
    malib_pos: Path,
    match_tolerance_s: float,
) -> None:
    reference = ppc_demo.read_flexible_reference_csv(reference_csv)
    comparison = ppc_demo.comparison
    malib_epochs = comparison.read_rtklib_pos(malib_pos)
    malib_metrics = ppc_demo.summarize_solution_epochs(
        reference,
        malib_epochs,
        ppc_demo.solver_fixed_status("ppp"),
        "MALIB",
        match_tolerance_s,
        solver_wall_time_s=None,
    )
    payload["malib_solution_pos"] = str(malib_pos)
    payload["comparison_target"] = "MALIB"
    payload["comparison_status"] = classify_comparison_status(
        float(payload.get("mean_position_error_m", 0.0)) - float(malib_metrics["mean_h_m"]),
        float(payload.get("p95_position_error_m", 0.0)) - float(malib_metrics["p95_h_m"]),
        float(payload.get("max_position_error_m", 0.0)) - float(malib_metrics["max_h_m"]),
    )
    payload["malib"] = malib_metrics
    payload["libgnss_minus_malib_mean_error_m"] = round(
        float(payload.get("mean_position_error_m", 0.0)) - float(malib_metrics["mean_h_m"]), 6
    )
    payload["libgnss_minus_malib_p95_error_m"] = round(
        float(payload.get("p95_position_error_m", 0.0)) - float(malib_metrics["p95_h_m"]), 6
    )
    payload["libgnss_minus_malib_max_error_m"] = round(
        float(payload.get("max_position_error_m", 0.0)) - float(malib_metrics["max_h_m"]), 6
    )


def build_ppc_comparison_pairs(
    solution_pos: Path,
    reference_csv: Path,
    malib_pos: Path,
    match_tolerance_s: float,
) -> list[object]:
    reference = ppc_demo.read_flexible_reference_csv(reference_csv)
    comparison = ppc_demo.comparison
    lib_epochs = comparison.read_libgnss_pos(solution_pos)
    malib_epochs = comparison.read_rtklib_pos(malib_pos)
    lib_matched = comparison.match_to_reference(lib_epochs, reference, match_tolerance_s)
    malib_matched = comparison.match_to_reference(malib_epochs, reference, match_tolerance_s)
    return comparison.pair_epochs(lib_matched, malib_matched, match_tolerance_s)


def write_ppc_comparison_csv(path: Path, pairs: list[object]) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    with path.open("w", encoding="utf-8", newline="") as stream:
        writer = csv.writer(stream)
        writer.writerow(
            [
                "gps_tow_s",
                "lib_h_m",
                "malib_h_m",
                "delta_h_m",
                "lib_status",
                "malib_status",
                "lib_east_m",
                "lib_north_m",
                "lib_up_m",
                "malib_east_m",
                "malib_north_m",
                "malib_up_m",
            ]
        )
        for pair in pairs:
            writer.writerow(
                [
                    f"{float(pair.tow):.3f}",
                    f"{float(pair.lib_epoch.horiz_error_m):.6f}",
                    f"{float(pair.rtklib_epoch.horiz_error_m):.6f}",
                    f"{float(pair.gap_m):.6f}",
                    int(pair.lib_epoch.status),
                    int(pair.rtklib_epoch.status),
                    f"{float(pair.lib_epoch.east_m):.6f}",
                    f"{float(pair.lib_epoch.north_m):.6f}",
                    f"{float(pair.lib_epoch.up_m):.6f}",
                    f"{float(pair.rtklib_epoch.east_m):.6f}",
                    f"{float(pair.rtklib_epoch.north_m):.6f}",
                    f"{float(pair.rtklib_epoch.up_m):.6f}",
                ]
            )


def render_ppc_comparison_plot(path: Path, pairs: list[object], title: str) -> None:
    if not pairs:
        raise SystemExit("No common epochs available for PPP products comparison plot")
    import matplotlib

    matplotlib.use("Agg")
    from matplotlib import pyplot as plt  # noqa: WPS433

    indices = list(range(1, len(pairs) + 1))
    lib_h = [float(pair.lib_epoch.horiz_error_m) for pair in pairs]
    malib_h = [float(pair.rtklib_epoch.horiz_error_m) for pair in pairs]
    delta_h = [float(pair.gap_m) for pair in pairs]

    figure, axes = plt.subplots(2, 1, figsize=(10.5, 6.4), sharex=True)
    figure.patch.set_facecolor("#f8f5ec")
    for axis in axes:
        axis.set_facecolor("#fffdf8")
        axis.grid(True, alpha=0.25)

    axes[0].plot(indices, lib_h, color="#d97706", linewidth=1.8, label="libgnss++")
    axes[0].plot(indices, malib_h, color="#2563eb", linewidth=1.6, label="MALIB")
    axes[0].set_ylabel("horizontal error (m)")
    axes[0].set_title(title)
    axes[0].legend(loc="upper right")

    axes[1].plot(indices, delta_h, color="#0f766e", linewidth=1.8, label="MALIB - libgnss++")
    axes[1].axhline(0.0, color="#9ca3af", linewidth=1.0, linestyle="--")
    axes[1].set_ylabel("delta (m)")
    axes[1].set_xlabel("common epoch")
    axes[1].legend(loc="upper right")

    path.parent.mkdir(parents=True, exist_ok=True)
    figure.tight_layout()
    figure.savefig(path, dpi=170)
    plt.close(figure)


def build_kinematic_comparison_rows(
    solution_pos: Path,
    reference_pos: Path,
    malib_pos: Path,
) -> list[dict[str, float | int]]:
    reference_by_epoch = {
        (int(record["week"]), round(float(record["tow"]), 3)): record
        for record in read_lib_xyz_records(reference_pos)
    }
    lib_by_epoch = {
        (int(record["week"]), round(float(record["tow"]), 3)): record
        for record in read_lib_xyz_records(solution_pos)
    }
    malib_by_epoch = {
        (int(record["week"]), round(float(record["tow"]), 3)): record
        for record in read_malib_xyz_records(malib_pos)
    }

    rows: list[dict[str, float | int]] = []
    shared_keys = sorted(set(lib_by_epoch) & set(malib_by_epoch) & set(reference_by_epoch))
    for week, tow in shared_keys:
        reference = reference_by_epoch[(week, tow)]
        lib_record = lib_by_epoch[(week, tow)]
        malib_record = malib_by_epoch[(week, tow)]
        lib_error = math.dist(
            (float(lib_record["x"]), float(lib_record["y"]), float(lib_record["z"])),
            (float(reference["x"]), float(reference["y"]), float(reference["z"])),
        )
        malib_error = math.dist(
            (float(malib_record["x"]), float(malib_record["y"]), float(malib_record["z"])),
            (float(reference["x"]), float(reference["y"]), float(reference["z"])),
        )
        rows.append(
            {
                "gps_week": week,
                "gps_tow_s": tow,
                "lib_error_m": lib_error,
                "malib_error_m": malib_error,
                "delta_error_m": lib_error - malib_error,
                "lib_status": int(lib_record["status"]),
                "malib_status": int(malib_record["status"]),
                "lib_satellites": int(lib_record["satellites"]),
                "malib_satellites": int(malib_record["satellites"]),
            }
        )
    return rows


def write_kinematic_comparison_csv(path: Path, rows: list[dict[str, float | int]]) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    with path.open("w", encoding="utf-8", newline="") as stream:
        writer = csv.writer(stream)
        writer.writerow(
            [
                "gps_week",
                "gps_tow_s",
                "lib_error_m",
                "malib_error_m",
                "delta_error_m",
                "lib_status",
                "malib_status",
                "lib_satellites",
                "malib_satellites",
            ]
        )
        for row in rows:
            writer.writerow(
                [
                    int(row["gps_week"]),
                    f"{float(row['gps_tow_s']):.3f}",
                    f"{float(row['lib_error_m']):.6f}",
                    f"{float(row['malib_error_m']):.6f}",
                    f"{float(row['delta_error_m']):.6f}",
                    int(row["lib_status"]),
                    int(row["malib_status"]),
                    int(row["lib_satellites"]),
                    int(row["malib_satellites"]),
                ]
            )


def render_kinematic_comparison_plot(path: Path, rows: list[dict[str, float | int]], title: str) -> None:
    if not rows:
        raise SystemExit("No common epochs available for PPP kinematic comparison plot")
    import matplotlib

    matplotlib.use("Agg")
    from matplotlib import pyplot as plt  # noqa: WPS433

    indices = list(range(1, len(rows) + 1))
    lib_errors = [float(row["lib_error_m"]) for row in rows]
    malib_errors = [float(row["malib_error_m"]) for row in rows]
    delta_errors = [float(row["delta_error_m"]) for row in rows]

    figure, axes = plt.subplots(2, 1, figsize=(10.5, 6.4), sharex=True)
    figure.patch.set_facecolor("#f8f5ec")
    for axis in axes:
        axis.set_facecolor("#fffdf8")
        axis.grid(True, alpha=0.25)

    axes[0].plot(indices, lib_errors, color="#d97706", linewidth=1.8, label="libgnss++")
    axes[0].plot(indices, malib_errors, color="#2563eb", linewidth=1.6, label="MALIB")
    axes[0].set_ylabel("3D error (m)")
    axes[0].set_title(title)
    axes[0].legend(loc="upper right")

    axes[1].plot(indices, delta_errors, color="#0f766e", linewidth=1.8, label="libgnss++ - MALIB")
    axes[1].axhline(0.0, color="#9ca3af", linewidth=1.0, linestyle="--")
    axes[1].set_ylabel("delta (m)")
    axes[1].set_xlabel("common epoch")
    axes[1].legend(loc="upper right")

    path.parent.mkdir(parents=True, exist_ok=True)
    figure.tight_layout()
    figure.savefig(path, dpi=170)
    plt.close(figure)


def main() -> int:
    args = parse_args()
    out_path, summary_path, reference_path = default_paths(args.profile)
    default_comparison_csv, default_comparison_png = default_comparison_artifacts(args.profile)
    out_path = args.out or out_path
    summary_path = args.summary_json or summary_path
    if args.reference_pos is not None:
        reference_path = args.reference_pos
    comparison_csv = args.comparison_csv or default_comparison_csv
    comparison_png = args.comparison_png or default_comparison_png

    if args.preset:
        presets = list(args.preset)
    elif args.product:
        presets = []
    else:
        presets = list(DEFAULT_PRESETS)

    if args.profile == "ppc":
        run_dir = resolve_ppc_run_dir(args)
        rover_obs = args.obs or (run_dir / "rover.obs")
        nav_path = resolve_ppc_nav_path(
            args,
            run_dir,
            require_exists=(not args.use_existing_solution) or (args.malib_bin is not None),
        )
        reference_csv = args.reference_csv or (run_dir / "reference.csv")
        ensure_input_exists(rover_obs, "PPC rover observation file", ROOT_DIR)
        ensure_input_exists(reference_csv, "PPC reference CSV", ROOT_DIR)
        if args.use_existing_solution:
            ensure_input_exists(out_path, "existing PPC PPP solution file", ROOT_DIR)
            if args.ppp_run_summary_json is None:
                raise SystemExit("--profile ppc with --use-existing-solution requires --ppp-run-summary-json")
            ensure_input_exists(args.ppp_run_summary_json, "existing PPP run summary JSON", ROOT_DIR)

        fetch_payload = run_fetch_products(
            ROOT_DIR,
            rover_obs,
            presets,
            list(args.product),
            product_date_text=args.product_date,
            cache_dir=args.product_cache_dir,
        )
        fetched_products = fetch_payload.get("products", {})
        if not isinstance(fetched_products, dict):
            raise SystemExit("gnss fetch-products did not return a product map")
        if "sp3" not in fetched_products or "clk" not in fetched_products:
            raise SystemExit("--profile ppc requires fetched products to include both sp3 and clk")

        temp_summary_dir: tempfile.TemporaryDirectory[str] | None = None
        if args.ppp_run_summary_json is not None:
            ppp_summary_path = args.ppp_run_summary_json
        else:
            temp_summary_dir = tempfile.TemporaryDirectory(prefix="gnss_ppp_products_ppc_")
            ppp_summary_path = Path(temp_summary_dir.name) / "ppp_summary.json"
        try:
            command = [*resolve_gnss_command(ROOT_DIR), "ppc-demo", "--solver", "ppp"]
            if args.run_dir is not None:
                command.extend(["--run-dir", str(args.run_dir)])
            else:
                command.extend(["--dataset-root", str(args.dataset_root), "--city", str(args.city), "--run", args.run])
            command.extend(
                [
                    "--out",
                    str(out_path),
                    "--summary-json",
                    str(summary_path),
                    "--reference-csv",
                    str(reference_csv),
                    "--match-tolerance-s",
                    str(args.match_tolerance_s),
                    "--ppp-summary-json",
                    str(ppp_summary_path),
                    "--sp3",
                    str(fetched_products["sp3"]),
                    "--clk",
                    str(fetched_products["clk"]),
                ]
            )
            if "ionex" in fetched_products:
                command.extend(["--ionex", str(fetched_products["ionex"])])
            if "dcb" in fetched_products:
                command.extend(["--dcb", str(fetched_products["dcb"])])
            if not args.use_existing_solution or args.nav is not None:
                command.extend(["--nav", str(nav_path)])
            if args.max_epochs > 0:
                command.extend(["--max-epochs", str(args.max_epochs)])
            if args.solver_wall_time_s is not None:
                command.extend(["--solver-wall-time-s", str(args.solver_wall_time_s)])
            if args.use_existing_solution:
                command.append("--use-existing-solution")
            if args.require_valid_epochs_min is not None:
                command.extend(["--require-valid-epochs-min", str(args.require_valid_epochs_min)])
            if args.require_matched_epochs_min is not None:
                command.extend(["--require-matched-epochs-min", str(args.require_matched_epochs_min)])
            payload = run_checked_json(command, summary_path)
        finally:
            if temp_summary_dir is not None:
                temp_summary_dir.cleanup()

        payload["products_signoff_profile"] = "ppc"
        payload["product_presets"] = presets
        payload["product_specs"] = list(args.product)
        payload["fetch_products"] = True
        payload["fetched_products"] = sorted(fetched_products.keys())
        payload["fetched_product_date"] = fetch_payload.get("effective_date")
        payload["sp3"] = str(fetched_products["sp3"]) if "sp3" in fetched_products else None
        payload["clk"] = str(fetched_products["clk"]) if "clk" in fetched_products else None
        payload["ionex"] = str(fetched_products["ionex"]) if "ionex" in fetched_products else None
        payload["dcb"] = str(fetched_products["dcb"]) if "dcb" in fetched_products else None
        payload["reference_csv"] = str(reference_csv)
        payload["run_dir"] = str(run_dir)
        payload["mean_position_error_m"] = payload.get("mean_h_m")
        payload["p95_position_error_m"] = payload.get("p95_h_m")
        payload["max_position_error_m"] = payload.get("max_h_m")

        malib_pos = run_ppc_malib_if_needed(args, rover_obs, nav_path, out_path)
        if malib_pos is not None:
            ensure_input_exists(malib_pos, "MALIB PPP solution file", ROOT_DIR)
            augment_ppc_malib_comparison(payload, reference_csv, malib_pos, args.match_tolerance_s)
            pairs = build_ppc_comparison_pairs(out_path, reference_csv, malib_pos, args.match_tolerance_s)
            if pairs:
                write_ppc_comparison_csv(comparison_csv, pairs)
                render_ppc_comparison_plot(
                    comparison_png,
                    pairs,
                    f"{payload.get('dataset', 'PPC PPP')} vs MALIB",
                )
                payload["common_epoch_pairs"] = len(pairs)
                payload["comparison_csv"] = str(comparison_csv)
                payload["comparison_png"] = str(comparison_png)

        enforce_ppc_metric_requirements(payload, args)
        enforce_ppp_requirements(payload, args)
        enforce_comparison_requirements(payload, args)
        summary_path.write_text(json.dumps(payload, indent=2, sort_keys=True) + "\n", encoding="utf-8")
        print("Finished PPP products sign-off.")
        print("  profile: ppc")
        print(f"  solution: {out_path}")
        print(f"  summary: {summary_path}")
        return 0

    command = [*resolve_gnss_command(ROOT_DIR)]
    if args.profile == "static":
        command.append("ppp-static-signoff")
        if args.obs is not None:
            command.extend(["--obs", str(args.obs)])
        if args.nav is not None:
            command.extend(["--nav", str(args.nav)])
        command.extend(["--out", str(out_path), "--summary-json", str(summary_path), "--max-epochs", str(args.max_epochs)])
        if args.enable_ar:
            command.append("--enable-ar")
        if args.ar_ratio_threshold != 3.0:
            command.extend(["--ar-ratio-threshold", str(args.ar_ratio_threshold)])
        if args.require_valid_epochs_min is not None:
            command.extend(["--require-valid-epochs-min", str(args.require_valid_epochs_min)])
    else:
        command.append("ppp-kinematic-signoff")
        if args.obs is not None:
            command.extend(["--obs", str(args.obs)])
        if args.base is not None:
            command.extend(["--base", str(args.base)])
        if args.nav is not None:
            command.extend(["--nav", str(args.nav)])
        command.extend([
            "--out", str(out_path),
            "--reference-pos", str(reference_path),
            "--summary-json", str(summary_path),
            "--max-epochs", str(args.max_epochs),
        ])
        if args.use_existing_reference:
            command.append("--use-existing-reference")
        if args.low_dynamics:
            command.append("--low-dynamics")
        else:
            command.append("--no-low-dynamics")
        if args.require_common_epoch_pairs_min is not None:
            command.extend(["--require-common-epoch-pairs-min", str(args.require_common_epoch_pairs_min)])
        if args.require_reference_fix_rate_min is not None:
            command.extend(["--require-reference-fix-rate-min", str(args.require_reference_fix_rate_min)])

    command.append("--fetch-products")
    for preset in presets:
        command.extend(["--preset", preset])
    for product in args.product:
        command.extend(["--product", product])
    if args.product_date is not None:
        command.extend(["--product-date", args.product_date])
    if args.product_cache_dir is not None:
        command.extend(["--product-cache-dir", str(args.product_cache_dir)])
    if args.malib_bin is not None:
        command.extend(["--malib-bin", str(args.malib_bin)])
    if args.malib_config is not None:
        command.extend(["--malib-config", str(args.malib_config)])
    if args.malib_pos is not None:
        command.extend(["--malib-pos", str(args.malib_pos)])
    if args.use_existing_malib:
        command.append("--use-existing-malib")
    if args.require_converged:
        command.append("--require-converged")
    if args.require_convergence_time_max is not None:
        command.extend(["--require-convergence-time-max", str(args.require_convergence_time_max)])
    if args.require_ionex_corrections_min is not None:
        command.extend(["--require-ionex-corrections-min", str(args.require_ionex_corrections_min)])
    if args.require_dcb_corrections_min is not None:
        command.extend(["--require-dcb-corrections-min", str(args.require_dcb_corrections_min)])
    if args.require_ppp_solution_rate_min is not None:
        command.extend(["--require-ppp-solution-rate-min", str(args.require_ppp_solution_rate_min)])
    if args.require_mean_error_max is not None:
        command.extend(["--require-mean-error-max", str(args.require_mean_error_max)])
    if args.require_p95_error_max is not None:
        command.extend(["--require-p95-error-max", str(args.require_p95_error_max)])
    if args.require_max_error_max is not None:
        command.extend(["--require-max-error-max", str(args.require_max_error_max)])
    if args.require_mean_sats_min is not None:
        command.extend(["--require-mean-sats-min", str(args.require_mean_sats_min)])

    run_checked(command)

    payload = json.loads(summary_path.read_text(encoding="utf-8"))
    if args.profile == "kinematic" and payload.get("malib_solution_pos"):
        reference_pos_text = payload.get("reference_pos")
        if isinstance(reference_pos_text, str):
            reference_pos = Path(reference_pos_text)
            if not reference_pos.is_absolute():
                reference_pos = (ROOT_DIR / reference_pos).resolve()
            malib_pos = Path(str(payload["malib_solution_pos"]))
            if not malib_pos.is_absolute():
                malib_pos = (ROOT_DIR / malib_pos).resolve()
            comparison_rows = build_kinematic_comparison_rows(out_path, reference_pos, malib_pos)
            if comparison_rows:
                write_kinematic_comparison_csv(comparison_csv, comparison_rows)
                render_kinematic_comparison_plot(
                    comparison_png,
                    comparison_rows,
                    f"{payload.get('dataset', 'PPP kinematic')} vs MALIB",
                )
                payload["reference_common_epoch_pairs"] = payload.get("common_epoch_pairs")
                payload["common_epoch_pairs"] = len(comparison_rows)
                payload["comparison_csv"] = str(comparison_csv)
                payload["comparison_png"] = str(comparison_png)
    payload["products_signoff_profile"] = args.profile
    payload["product_presets"] = presets
    payload["product_specs"] = list(args.product)
    payload["comparison_target"] = "MALIB" if payload.get("malib_solution_pos") else None
    payload["comparison_status"] = classify_comparison_status(
        payload.get("libgnss_minus_malib_mean_error_m"),
        payload.get("libgnss_minus_malib_p95_error_m"),
        payload.get("libgnss_minus_malib_max_error_m"),
    )
    enforce_ppp_requirements(payload, args)
    enforce_comparison_requirements(payload, args)
    summary_path.write_text(json.dumps(payload, indent=2, sort_keys=True) + "\n", encoding="utf-8")

    print("Finished PPP products sign-off.")
    print(f"  profile: {args.profile}")
    print(f"  solution: {out_path}")
    print(f"  summary: {summary_path}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
