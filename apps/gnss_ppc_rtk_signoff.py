#!/usr/bin/env python3
"""Run and validate PPC-Dataset RTK sign-off profiles."""

from __future__ import annotations

import argparse
import json
import os
from pathlib import Path
import subprocess

from gnss_toml_config import parse_args_with_toml
from gnss_runtime import ensure_input_exists, resolve_gnss_command


ROOT_DIR = Path(__file__).resolve().parent.parent

PROFILE_DEFAULTS: dict[str, dict[str, float | int]] = {
    "tokyo": {
        "require_valid_epochs_min": 100,
        "require_matched_epochs_min": 100,
        "require_fix_rate_min": 95.0,
        "require_median_h_max": 0.06,
        "require_p95_h_max": 0.15,
        "require_max_h_max": 0.50,
        "require_p95_up_max": 0.60,
        "require_mean_sats_min": 18.0,
        "require_solver_wall_time_max": 5.0,
        "require_realtime_factor_min": 4.5,
        "require_effective_epoch_rate_min": 24.0,
        "require_lib_fix_rate_vs_rtklib_min_delta": 0.0,
        "require_lib_median_h_vs_rtklib_max_delta": 0.01,
        "require_lib_p95_h_vs_rtklib_max_delta": 0.02,
    },
    "nagoya": {
        "require_valid_epochs_min": 100,
        "require_matched_epochs_min": 100,
        "require_fix_rate_min": 95.0,
        "require_median_h_max": 0.12,
        "require_p95_h_max": 0.12,
        "require_max_h_max": 0.60,
        "require_p95_up_max": 0.40,
        "require_mean_sats_min": 20.0,
        "require_solver_wall_time_max": 5.5,
        "require_realtime_factor_min": 4.4,
        "require_effective_epoch_rate_min": 22.0,
        "require_lib_fix_rate_vs_rtklib_min_delta": 0.0,
        "require_lib_median_h_vs_rtklib_max_delta": 0.05,
        "require_lib_p95_h_vs_rtklib_max_delta": 0.10,
    },
}

PROFILE_TUNING_DEFAULTS: dict[str, dict[str, str | float | int | bool]] = {
    "tokyo": {
        "preset": "low-cost",
        "arfilter": True,
        "arfilter_margin": 0.35,
        "min_hold_count": 8,
        "hold_ratio_threshold": 2.6,
    },
    "nagoya": {
        "preset": "low-cost",
        "arfilter": False,
        "min_hold_count": 7,
        "hold_ratio_threshold": 2.4,
    },
}

REQUIREMENT_NAMES = (
    "require_valid_epochs_min",
    "require_matched_epochs_min",
    "require_positioning_rate_min",
    "require_fix_rate_min",
    "require_ppc_official_score_min",
    "require_ppc_score_3d_50cm_ref_min",
    "require_median_h_max",
    "require_p95_h_max",
    "require_max_h_max",
    "require_p95_up_max",
    "require_mean_sats_min",
    "require_solver_wall_time_max",
    "require_realtime_factor_min",
    "require_effective_epoch_rate_min",
    "require_lib_fix_rate_vs_rtklib_min_delta",
    "require_lib_median_h_vs_rtklib_max_delta",
    "require_lib_p95_h_vs_rtklib_max_delta",
)


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(prog=os.environ.get("GNSS_CLI_NAME"))
    parser.add_argument(
        "--config-toml",
        type=Path,
        default=None,
        help="Optional TOML config. Uses [ppc_rtk_signoff] or top-level keys.",
    )
    parser.add_argument("--dataset-root", type=Path, default=None)
    parser.add_argument("--city", choices=("tokyo", "nagoya"), default=None)
    parser.add_argument("--run", default="run1")
    parser.add_argument("--run-dir", type=Path, default=None)
    parser.add_argument("--rover", type=Path, default=None)
    parser.add_argument("--base", type=Path, default=None)
    parser.add_argument("--nav", type=Path, default=None)
    parser.add_argument("--reference-csv", type=Path, default=None)
    parser.add_argument("--out", type=Path, default=None)
    parser.add_argument("--summary-json", type=Path, default=None)
    parser.add_argument("--max-epochs", type=int, default=120)
    parser.add_argument("--use-existing-solution", action="store_true")
    parser.add_argument("--solver-wall-time-s", type=float, default=None)
    parser.add_argument("--rtklib-bin", type=Path, default=None)
    parser.add_argument(
        "--rtklib-config",
        type=Path,
        default=ROOT_DIR / "scripts/rtklib_odaiba.conf",
    )
    parser.add_argument("--rtklib-pos", type=Path, default=None)
    parser.add_argument("--use-existing-rtklib-solution", action="store_true")
    parser.add_argument("--rtklib-solver-wall-time-s", type=float, default=None)
    parser.add_argument("--commercial-pos", type=Path, default=None)
    parser.add_argument("--commercial-rover", type=Path, default=None)
    parser.add_argument("--commercial-base", type=Path, default=None)
    parser.add_argument("--commercial-nav", type=Path, default=None)
    parser.add_argument("--commercial-out", type=Path, default=None)
    parser.add_argument("--use-existing-commercial-solution", action="store_true")
    parser.add_argument("--commercial-format", choices=("auto", "pos", "csv"), default="auto")
    parser.add_argument("--commercial-label", default="commercial_receiver")
    parser.add_argument("--commercial-matched-csv", type=Path, default=None)
    parser.add_argument("--commercial-solver-wall-time-s", type=float, default=None)
    parser.add_argument("--commercial-preset", choices=("survey", "low-cost", "moving-base"), default=None)
    parser.add_argument("--commercial-arfilter", dest="commercial_arfilter", action="store_true")
    parser.add_argument("--no-commercial-arfilter", dest="commercial_arfilter", action="store_false")
    parser.set_defaults(commercial_arfilter=None)
    parser.add_argument("--commercial-arfilter-margin", type=float, default=None)
    parser.add_argument("--commercial-min-hold-count", type=int, default=None)
    parser.add_argument("--commercial-hold-ratio-threshold", type=float, default=None)
    parser.add_argument("--match-tolerance-s", type=float, default=0.25)
    parser.add_argument("--preset", choices=("survey", "low-cost", "moving-base"), default=None)
    parser.add_argument("--iono", choices=("auto", "off", "iflc", "est"), default=None)
    parser.add_argument("--ratio", type=float, default=None)
    parser.add_argument("--arfilter", dest="arfilter", action="store_true")
    parser.add_argument("--no-arfilter", dest="arfilter", action="store_false")
    parser.set_defaults(arfilter=None)
    parser.add_argument("--arfilter-margin", type=float, default=None)
    parser.add_argument("--min-hold-count", type=int, default=None)
    parser.add_argument("--hold-ratio-threshold", type=float, default=None)
    parser.add_argument("--no-kinematic-post-filter", action="store_true")

    parser.add_argument("--require-valid-epochs-min", type=int, default=None)
    parser.add_argument("--require-matched-epochs-min", type=int, default=None)
    parser.add_argument("--require-positioning-rate-min", type=float, default=None)
    parser.add_argument("--require-fix-rate-min", type=float, default=None)
    parser.add_argument("--require-ppc-official-score-min", type=float, default=None)
    parser.add_argument("--require-ppc-score-3d-50cm-ref-min", type=float, default=None)
    parser.add_argument("--require-median-h-max", type=float, default=None)
    parser.add_argument("--require-p95-h-max", type=float, default=None)
    parser.add_argument("--require-max-h-max", type=float, default=None)
    parser.add_argument("--require-p95-up-max", type=float, default=None)
    parser.add_argument("--require-mean-sats-min", type=float, default=None)
    parser.add_argument("--require-solver-wall-time-max", type=float, default=None)
    parser.add_argument("--require-realtime-factor-min", type=float, default=None)
    parser.add_argument("--require-effective-epoch-rate-min", type=float, default=None)
    parser.add_argument("--require-lib-fix-rate-vs-rtklib-min-delta", type=float, default=None)
    parser.add_argument("--require-lib-median-h-vs-rtklib-max-delta", type=float, default=None)
    parser.add_argument("--require-lib-p95-h-vs-rtklib-max-delta", type=float, default=None)
    return parse_args_with_toml(parser, "ppc_rtk_signoff")


def resolve_run_dir(args: argparse.Namespace) -> Path:
    if args.run_dir is not None:
        return args.run_dir
    if args.dataset_root is None or args.city is None:
        raise SystemExit("Provide either --run-dir or both --dataset-root and --city")
    return args.dataset_root / args.city / args.run


def resolve_city(args: argparse.Namespace, run_dir: Path) -> str:
    city = args.city or run_dir.parent.name.lower()
    if city not in PROFILE_DEFAULTS:
        raise SystemExit(f"Unsupported PPC sign-off city profile: {city}")
    return city


def default_paths(city: str, run_name: str) -> tuple[Path, Path]:
    slug = f"ppc_{city}_{run_name}_rtk"
    return (
        ROOT_DIR / "output" / f"{slug}.pos",
        ROOT_DIR / "output" / f"{slug}_summary.json",
    )


def selected_thresholds(args: argparse.Namespace, city: str, use_rtklib_compare: bool) -> dict[str, float | int]:
    thresholds = dict(PROFILE_DEFAULTS[city])
    if not use_rtklib_compare:
        thresholds.pop("require_lib_fix_rate_vs_rtklib_min_delta", None)
        thresholds.pop("require_lib_median_h_vs_rtklib_max_delta", None)
        thresholds.pop("require_lib_p95_h_vs_rtklib_max_delta", None)
    for name in REQUIREMENT_NAMES:
        value = getattr(args, name)
        if value is not None:
            thresholds[name] = value
    return thresholds


def selected_tuning(args: argparse.Namespace, city: str) -> dict[str, str | float | int | bool]:
    tuning = dict(PROFILE_TUNING_DEFAULTS[city])
    if args.preset is not None:
        tuning["preset"] = args.preset
    if getattr(args, "iono", None) is not None:
        tuning["iono"] = args.iono
    if getattr(args, "ratio", None) is not None:
        tuning["ratio"] = args.ratio
    if args.arfilter is not None:
        tuning["arfilter"] = args.arfilter
    if args.arfilter_margin is not None:
        tuning["arfilter_margin"] = args.arfilter_margin
    if args.min_hold_count is not None:
        tuning["min_hold_count"] = args.min_hold_count
    if args.hold_ratio_threshold is not None:
        tuning["hold_ratio_threshold"] = args.hold_ratio_threshold
    return tuning


def build_ppc_demo_command(args: argparse.Namespace,
                           run_dir: Path,
                           out: Path,
                           summary_json: Path,
                           thresholds: dict[str, float | int],
                           tuning: dict[str, str | float | int | bool]) -> list[str]:
    gnss_command = resolve_gnss_command(ROOT_DIR)
    command = [
        *gnss_command,
        "ppc-demo",
        "--run-dir",
        str(run_dir),
        "--solver",
        "rtk",
        "--out",
        str(out),
        "--summary-json",
        str(summary_json),
        "--match-tolerance-s",
        str(args.match_tolerance_s),
    ]
    if args.city is not None:
        command.extend(["--city", args.city])
    if args.rover is not None:
        command.extend(["--rover", str(args.rover)])
    if args.base is not None:
        command.extend(["--base", str(args.base)])
    if args.nav is not None:
        command.extend(["--nav", str(args.nav)])
    if args.reference_csv is not None:
        command.extend(["--reference-csv", str(args.reference_csv)])
    if args.max_epochs > 0:
        command.extend(["--max-epochs", str(args.max_epochs)])
    if args.use_existing_solution:
        command.append("--use-existing-solution")
    if args.solver_wall_time_s is not None:
        command.extend(["--solver-wall-time-s", str(args.solver_wall_time_s)])
    if args.rtklib_bin is not None:
        command.extend(["--rtklib-bin", str(args.rtklib_bin), "--rtklib-config", str(args.rtklib_config)])
    if args.rtklib_pos is not None:
        command.extend(["--rtklib-pos", str(args.rtklib_pos)])
    if args.use_existing_rtklib_solution:
        command.append("--use-existing-rtklib-solution")
    if args.rtklib_solver_wall_time_s is not None:
        command.extend(["--rtklib-solver-wall-time-s", str(args.rtklib_solver_wall_time_s)])
    if args.commercial_pos is not None:
        command.extend(
            [
                "--commercial-pos",
                str(args.commercial_pos),
                "--commercial-format",
                args.commercial_format,
                "--commercial-label",
                args.commercial_label,
            ]
        )
    if args.commercial_rover is not None:
        command.extend(
            [
                "--commercial-rover",
                str(args.commercial_rover),
                "--commercial-label",
                args.commercial_label,
            ]
        )
    if args.commercial_base is not None:
        command.extend(["--commercial-base", str(args.commercial_base)])
    if args.commercial_nav is not None:
        command.extend(["--commercial-nav", str(args.commercial_nav)])
    if args.commercial_out is not None:
        command.extend(["--commercial-out", str(args.commercial_out)])
    if args.use_existing_commercial_solution:
        command.append("--use-existing-commercial-solution")
    if args.commercial_matched_csv is not None:
        command.extend(["--commercial-matched-csv", str(args.commercial_matched_csv)])
    if args.commercial_solver_wall_time_s is not None:
        command.extend(["--commercial-solver-wall-time-s", str(args.commercial_solver_wall_time_s)])
    if args.commercial_preset is not None:
        command.extend(["--commercial-preset", args.commercial_preset])
    if args.commercial_arfilter is True:
        command.append("--commercial-arfilter")
    elif args.commercial_arfilter is False:
        command.append("--no-commercial-arfilter")
    if args.commercial_arfilter_margin is not None:
        command.extend(["--commercial-arfilter-margin", str(args.commercial_arfilter_margin)])
    if args.commercial_min_hold_count is not None:
        command.extend(["--commercial-min-hold-count", str(args.commercial_min_hold_count)])
    if args.commercial_hold_ratio_threshold is not None:
        command.extend(["--commercial-hold-ratio-threshold", str(args.commercial_hold_ratio_threshold)])

    preset = tuning.get("preset")
    if isinstance(preset, str):
        command.extend(["--preset", preset])
    iono = tuning.get("iono")
    if isinstance(iono, str):
        command.extend(["--iono", iono])
    ratio = tuning.get("ratio")
    if isinstance(ratio, (int, float)):
        command.extend(["--ratio", str(ratio)])
    arfilter = tuning.get("arfilter")
    if arfilter is True:
        command.append("--arfilter")
    elif arfilter is False:
        command.append("--no-arfilter")
    if "arfilter_margin" in tuning:
        command.extend(["--arfilter-margin", str(tuning["arfilter_margin"])])
    if "min_hold_count" in tuning:
        command.extend(["--min-hold-count", str(tuning["min_hold_count"])])
    if "hold_ratio_threshold" in tuning:
        command.extend(["--hold-ratio-threshold", str(tuning["hold_ratio_threshold"])])
    if getattr(args, "no_kinematic_post_filter", False):
        command.append("--no-kinematic-post-filter")

    for name, value in thresholds.items():
        command.extend([f"--{name.replace('_', '-')}", str(value)])
    return command


def run_command(command: list[str]) -> None:
    print("+", " ".join(command))
    subprocess.run(command, check=True)


def main() -> int:
    args = parse_args()
    run_dir = resolve_run_dir(args)
    city = resolve_city(args, run_dir)
    run_name = args.run if args.run_dir is None else run_dir.name
    out, summary_json = default_paths(city, run_name)
    if args.out is not None:
        out = args.out
    if args.summary_json is not None:
        summary_json = args.summary_json

    if args.max_epochs == 0:
        raise SystemExit("--max-epochs must be positive or -1")
    if args.use_existing_rtklib_solution and args.rtklib_pos is None:
        raise SystemExit("--use-existing-rtklib-solution requires --rtklib-pos")
    if args.rtklib_bin is not None:
        ensure_input_exists(args.rtklib_bin, "RTKLIB binary", ROOT_DIR)
        ensure_input_exists(args.rtklib_config, "RTKLIB config", ROOT_DIR)
    if args.use_existing_solution:
        ensure_input_exists(out, "existing PPC RTK solution", ROOT_DIR)
    if args.use_existing_rtklib_solution and args.rtklib_pos is not None:
        ensure_input_exists(args.rtklib_pos, "existing RTKLIB solution", ROOT_DIR)
    if args.commercial_pos is not None and args.commercial_rover is not None:
        raise SystemExit("Use either --commercial-pos or --commercial-rover, not both")
    if args.commercial_pos is not None:
        ensure_input_exists(args.commercial_pos, "commercial receiver solution", ROOT_DIR)
    elif args.commercial_rover is not None:
        ensure_input_exists(args.commercial_rover, "commercial rover observation file", ROOT_DIR)
        if args.commercial_base is not None:
            ensure_input_exists(args.commercial_base, "commercial base observation file", ROOT_DIR)
        if args.commercial_nav is not None:
            ensure_input_exists(args.commercial_nav, "commercial navigation file", ROOT_DIR)
        if args.use_existing_commercial_solution:
            if args.commercial_out is None:
                raise SystemExit("--use-existing-commercial-solution requires --commercial-out")
            ensure_input_exists(args.commercial_out, "existing commercial receiver solution", ROOT_DIR)
    elif args.commercial_matched_csv is not None:
        raise SystemExit("--commercial-matched-csv requires --commercial-pos or --commercial-rover")
    elif args.use_existing_commercial_solution:
        raise SystemExit("--use-existing-commercial-solution requires --commercial-rover")

    thresholds = selected_thresholds(
        args,
        city,
        use_rtklib_compare=(args.rtklib_bin is not None or args.rtklib_pos is not None),
    )
    tuning = selected_tuning(args, city)
    summary_json.parent.mkdir(parents=True, exist_ok=True)
    out.parent.mkdir(parents=True, exist_ok=True)
    command = build_ppc_demo_command(args, run_dir, out, summary_json, thresholds, tuning)
    run_command(command)

    payload = json.loads(summary_json.read_text(encoding="utf-8"))
    payload["signoff_profile"] = f"ppc-rtk-{city}"
    payload["signoff_thresholds"] = thresholds
    payload["tuning_profile"] = tuning
    payload["rtklib_comparison_enabled"] = "delta_vs_rtklib" in payload
    payload["commercial_receiver_comparison_enabled"] = "delta_vs_commercial_receiver" in payload
    summary_json.write_text(json.dumps(payload, indent=2, sort_keys=True) + "\n", encoding="utf-8")

    print("Finished PPC RTK sign-off.")
    print(f"  profile: {payload['signoff_profile']}")
    print(f"  solution: {out}")
    print(f"  summary: {summary_json}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
