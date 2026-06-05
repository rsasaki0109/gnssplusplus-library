#!/usr/bin/env python3
"""Regenerate and verify taroz position/velocity ambiguity PDC artifacts."""

from __future__ import annotations

import argparse
import csv
import glob
import json
import math
import os
from pathlib import Path
import subprocess
import sys
from typing import Any


ROOT_DIR = Path(__file__).resolve().parents[1]
APPS_DIR = Path(__file__).resolve().parent
EXE_SUFFIX = ".exe" if os.name == "nt" else ""
BUILD_CONFIGS = ("Release", "RelWithDebInfo", "Debug", "MinSizeRel")

DEFAULT_DATA_DIR = Path("/tmp/taroz_gtsam_gnss/examples/data")
DEFAULT_TAROZ_ROOT = Path("/tmp/taroz_gtsam_gnss")
DEFAULT_OUT_DIR = ROOT_DIR / "output/dogfood/taroz_pos_vel_amb_pdc_current"
DEFAULT_MATLAB_DIR = (
    ROOT_DIR / "output/dogfood/taroz_matlab_pos_vel_amb_pdc_debug"
)
DEFAULT_MATLAB_DUMP_SCRIPT = (
    ROOT_DIR / "scripts/dump_taroz_pos_vel_ambiguity_pdc_debug.m"
)
DEFAULT_PARITY_TEST = ROOT_DIR / "tests/test_taroz_pos_vel_amb_pdc_factor_parity.py"

POS_NAME = "opt.pos"
SUMMARY_NAME = "summary.json"
EPOCH_DEBUG_NAME = "epoch_debug.csv"
FACTOR_DEBUG_NAME = "factor_debug.csv"
SD_FACTOR_DEBUG_NAME = "sd_factor_debug.csv"
LAMBDA_DEBUG_NAME = "lambda_debug.csv"
COST_TRACE_NAME = "cost_trace.csv"

EXPECTED_FULL_COUNTS = {
    "skip_epochs": 0,
    "max_epochs": 0,
    "seed_matched_epochs": 1141,
    "seed_interpolated_epochs": 34,
    "input_epochs": 1141,
    "optimized_epochs": 1141,
    "valid_solutions": 964,
    "fixed_solutions": 721,
    "float_solutions": 243,
    "single_difference_doppler_factors": 18330,
    "single_difference_tdcp_factors": 13509,
    "double_difference_pseudorange_factors": 15013,
    "double_difference_carrier_factors": 13826,
    "ambiguity_states": 13826,
    "lambda_ambiguity_candidates": 13362,
    "lambda_ambiguity_used_candidates": 10737,
    "lambda_ambiguity_attempts": 964,
    "fixed_ambiguities": 10737,
    "motion_factors": 1140,
    "ambiguity_between_factors": 13398,
    "graph_factors": 76357,
    "graph_values": 21813,
    "double_difference_matched_base_epochs": 1116,
    "double_difference_interpolated_base_epochs": 0,
    "double_difference_candidate_pairs": 13898,
    "double_difference_rejected_no_base_epoch": 0,
    "double_difference_rejected_no_reference": 112,
    "robust_double_difference_pseudorange_factors": 1198,
    "robust_double_difference_carrier_factors": 2941,
    "float_rejected_seed_position_divergence": 0,
    "float_rejected_position_jump": 0,
    "iterations": 24,
}


def _counts_with(
    base: dict[str, int] | None = None,
    **overrides: int,
) -> dict[str, int]:
    counts = dict(EXPECTED_FULL_COUNTS if base is None else base)
    counts.update(overrides)
    return counts


EXPECTED_FIRST_120_COUNTS = _counts_with(
    skip_epochs=0,
    max_epochs=120,
    seed_matched_epochs=120,
    seed_interpolated_epochs=0,
    input_epochs=120,
    optimized_epochs=120,
    valid_solutions=120,
    fixed_solutions=114,
    float_solutions=6,
    single_difference_doppler_factors=2768,
    single_difference_tdcp_factors=2343,
    double_difference_pseudorange_factors=2370,
    double_difference_carrier_factors=2345,
    ambiguity_states=2345,
    lambda_ambiguity_candidates=2345,
    lambda_ambiguity_used_candidates=2223,
    lambda_ambiguity_attempts=120,
    fixed_ambiguities=2223,
    motion_factors=119,
    ambiguity_between_factors=2298,
    graph_factors=12363,
    graph_values=3185,
    double_difference_matched_base_epochs=120,
    double_difference_candidate_pairs=2345,
    double_difference_rejected_no_reference=0,
    robust_double_difference_pseudorange_factors=76,
    robust_double_difference_carrier_factors=554,
    iterations=14,
)

EXPECTED_SHIFTED_120_COUNTS = _counts_with(
    skip_epochs=400,
    max_epochs=120,
    seed_matched_epochs=120,
    seed_interpolated_epochs=0,
    input_epochs=120,
    optimized_epochs=120,
    valid_solutions=57,
    fixed_solutions=26,
    float_solutions=31,
    single_difference_doppler_factors=1198,
    single_difference_tdcp_factors=645,
    double_difference_pseudorange_factors=869,
    double_difference_carrier_factors=712,
    ambiguity_states=712,
    lambda_ambiguity_candidates=492,
    lambda_ambiguity_used_candidates=248,
    lambda_ambiguity_attempts=57,
    fixed_ambiguities=248,
    motion_factors=119,
    ambiguity_between_factors=649,
    graph_factors=4312,
    graph_values=1552,
    double_difference_matched_base_epochs=120,
    double_difference_candidate_pairs=727,
    double_difference_rejected_no_reference=17,
    robust_double_difference_pseudorange_factors=120,
    robust_double_difference_carrier_factors=187,
    iterations=18,
)


EXPECTATION_PROFILES: dict[str, dict[str, Any]] = {
    "default": {
        "counts": EXPECTED_FULL_COUNTS,
        "seed_match_tolerance_s": 0.01,
        "seed_interpolation_max_gap_s": 30.0,
        "lambda_ratio_threshold": 2.0,
        "max_float_seed_position_divergence_m": 0.0,
        "max_float_position_jump_m": 0.0,
        "use_epoch_lambda_fixed_output": True,
        "lambda_ambiguity_fix_used": True,
        "fixed_solution": True,
    },
    "no-epoch-lambda-fixed-output": {
        "counts": _counts_with(
            fixed_solutions=0,
            float_solutions=964,
            lambda_ambiguity_used_candidates=0,
            fixed_ambiguities=0,
        ),
        "seed_match_tolerance_s": 0.01,
        "seed_interpolation_max_gap_s": 30.0,
        "lambda_ratio_threshold": 2.0,
        "max_float_seed_position_divergence_m": 0.0,
        "max_float_position_jump_m": 0.0,
        "use_epoch_lambda_fixed_output": False,
        "lambda_ambiguity_fix_used": False,
        "fixed_solution": False,
    },
    "strict-lambda-ratio": {
        "counts": _counts_with(
            fixed_solutions=0,
            float_solutions=964,
            lambda_ambiguity_used_candidates=0,
            fixed_ambiguities=0,
        ),
        "seed_match_tolerance_s": 0.01,
        "seed_interpolation_max_gap_s": 30.0,
        "lambda_ratio_threshold": 100.0,
        "max_float_seed_position_divergence_m": 0.0,
        "max_float_position_jump_m": 0.0,
        "use_epoch_lambda_fixed_output": True,
        "lambda_ambiguity_fix_used": False,
        "fixed_solution": False,
    },
    "no-seed-interpolation": {
        "counts": _counts_with(
            seed_matched_epochs=1107,
            seed_interpolated_epochs=0,
            robust_double_difference_carrier_factors=2940,
            iterations=25,
        ),
        "seed_match_tolerance_s": 0.01,
        "seed_interpolation_max_gap_s": 0.0,
        "lambda_ratio_threshold": 2.0,
        "max_float_seed_position_divergence_m": 0.0,
        "max_float_position_jump_m": 0.0,
        "use_epoch_lambda_fixed_output": True,
        "lambda_ambiguity_fix_used": True,
        "fixed_solution": True,
    },
    "first-120-window": {
        "counts": EXPECTED_FIRST_120_COUNTS,
        "seed_match_tolerance_s": 0.01,
        "seed_interpolation_max_gap_s": 30.0,
        "lambda_ratio_threshold": 2.0,
        "max_float_seed_position_divergence_m": 0.0,
        "max_float_position_jump_m": 0.0,
        "use_epoch_lambda_fixed_output": True,
        "lambda_ambiguity_fix_used": True,
        "fixed_solution": True,
    },
    "first-120-strict-lambda-ratio": {
        "counts": _counts_with(
            EXPECTED_FIRST_120_COUNTS,
            fixed_solutions=0,
            float_solutions=120,
            lambda_ambiguity_used_candidates=0,
            fixed_ambiguities=0,
        ),
        "seed_match_tolerance_s": 0.01,
        "seed_interpolation_max_gap_s": 30.0,
        "lambda_ratio_threshold": 100.0,
        "max_float_seed_position_divergence_m": 0.0,
        "max_float_position_jump_m": 0.0,
        "use_epoch_lambda_fixed_output": True,
        "lambda_ambiguity_fix_used": False,
        "fixed_solution": False,
    },
    "shifted-120-window": {
        "counts": EXPECTED_SHIFTED_120_COUNTS,
        "matlab_skip_epochs": 412,
        "matlab_max_epochs": 120,
        "seed_match_tolerance_s": 0.01,
        "seed_interpolation_max_gap_s": 30.0,
        "lambda_ratio_threshold": 2.0,
        "max_float_seed_position_divergence_m": 0.0,
        "max_float_position_jump_m": 0.0,
        "use_epoch_lambda_fixed_output": True,
        "lambda_ambiguity_fix_used": True,
        "fixed_solution": True,
    },
    "shifted-120-strict-lambda-ratio": {
        "counts": _counts_with(
            EXPECTED_SHIFTED_120_COUNTS,
            fixed_solutions=0,
            float_solutions=57,
            lambda_ambiguity_used_candidates=0,
            fixed_ambiguities=0,
        ),
        "matlab_skip_epochs": 412,
        "matlab_max_epochs": 120,
        "seed_match_tolerance_s": 0.01,
        "seed_interpolation_max_gap_s": 30.0,
        "lambda_ratio_threshold": 100.0,
        "max_float_seed_position_divergence_m": 0.0,
        "max_float_position_jump_m": 0.0,
        "use_epoch_lambda_fixed_output": True,
        "lambda_ambiguity_fix_used": False,
        "fixed_solution": False,
    },
    "first-120-seed-divergence-guard": {
        "counts": _counts_with(
            EXPECTED_FIRST_120_COUNTS,
            valid_solutions=114,
            fixed_solutions=114,
            float_solutions=0,
            float_rejected_seed_position_divergence=6,
        ),
        "seed_match_tolerance_s": 0.01,
        "seed_interpolation_max_gap_s": 30.0,
        "lambda_ratio_threshold": 2.0,
        "max_float_seed_position_divergence_m": 1.0,
        "max_float_position_jump_m": 0.0,
        "use_epoch_lambda_fixed_output": True,
        "lambda_ambiguity_fix_used": True,
        "fixed_solution": True,
    },
}


def expectation_profile(name: str) -> dict[str, Any]:
    try:
        return EXPECTATION_PROFILES[name]
    except KeyError as exc:
        valid = ", ".join(sorted(EXPECTATION_PROFILES))
        raise SystemExit(
            f"unsupported taroz ambiguity PDC expectation profile {name!r}; "
            f"expected one of: {valid}"
        ) from exc


def default_path(name: str) -> Path:
    return DEFAULT_DATA_DIR / name


def find_binary(target_name: str) -> Path | None:
    filename = target_name + EXE_SUFFIX
    build_roots = [
        Path(path)
        for path in sorted(glob.glob(str(ROOT_DIR / "build*")))
        if Path(path).is_dir()
    ]
    default_build = ROOT_DIR / "build"
    if default_build not in build_roots:
        build_roots.insert(0, default_build)

    candidates = [APPS_DIR / filename]
    for build_root in build_roots:
        candidates.append(build_root / "apps" / filename)
        for config in BUILD_CONFIGS:
            candidates.append(build_root / "apps" / config / filename)
            candidates.append(build_root / config / "apps" / filename)
            candidates.append(build_root / config / filename)

    for candidate in candidates:
        if candidate.is_file():
            return candidate

    recursive_hits: list[Path] = []
    for build_root in build_roots:
        recursive_hits.extend(
            Path(path)
            for path in glob.glob(str(build_root / "**" / filename), recursive=True)
            if Path(path).is_file()
        )
    return sorted(recursive_hits)[0] if recursive_hits else None


def output_paths(out_dir: Path) -> dict[str, Path]:
    return {
        "pos": out_dir / POS_NAME,
        "summary": out_dir / SUMMARY_NAME,
        "epoch_debug": out_dir / EPOCH_DEBUG_NAME,
        "factor_debug": out_dir / FACTOR_DEBUG_NAME,
        "sd_factor_debug": out_dir / SD_FACTOR_DEBUG_NAME,
        "lambda_debug": out_dir / LAMBDA_DEBUG_NAME,
        "cost_trace": out_dir / COST_TRACE_NAME,
    }


def build_fgo_command(args: argparse.Namespace, paths: dict[str, Path]) -> list[str]:
    fgo_bin = Path(args.fgo_bin) if args.fgo_bin else find_binary("gnss_fgo")
    if fgo_bin is None:
        if args.dry_run:
            fgo_bin = Path("gnss_fgo")
        else:
            raise SystemExit(
                "gnss_fgo binary not found. Build it first with "
                "`cmake --build build --target gnss_fgo`, or pass --fgo-bin."
            )

    command = [
        str(fgo_bin),
        "--preset",
        "taroz-amb-pdc",
        "--obs",
        str(args.obs),
        "--base",
        str(args.base),
        "--nav",
        str(args.nav),
        "--seed-pos",
        str(args.seed_pos),
        "--out",
        str(paths["pos"]),
        "--summary-json",
        str(paths["summary"]),
        "--epoch-debug-csv",
        str(paths["epoch_debug"]),
        "--factor-debug-csv",
        str(paths["factor_debug"]),
        "--sd-factor-debug-csv",
        str(paths["sd_factor_debug"]),
        "--lambda-debug-csv",
        str(paths["lambda_debug"]),
        "--cost-trace-csv",
        str(paths["cost_trace"]),
        "--max-float-seed-divergence",
        "0",
        "--max-float-position-jump",
        "0",
        "--quiet",
    ]
    command.extend(args.fgo_extra_arg)
    return command


def _matlab_single_quoted_path(path: Path) -> str:
    return str(path).replace("'", "''")


def repo_relative_path(path: Path) -> Path:
    return path if path.is_absolute() else ROOT_DIR / path


def matlab_example_dir(args: argparse.Namespace) -> Path:
    return args.taroz_example_dir or (args.taroz_root / "examples")


def matlab_output_dir(args: argparse.Namespace) -> Path:
    return repo_relative_path(args.matlab_dir).resolve()


def matlab_window(args: argparse.Namespace) -> tuple[int, int]:
    profile = expectation_profile(args.expectation_profile)
    counts = profile["counts"]
    skip_epochs = (
        args.matlab_skip_epochs
        if args.matlab_skip_epochs is not None
        else int(profile.get("matlab_skip_epochs", counts.get("skip_epochs", 0)))
    )
    max_epochs = (
        args.matlab_max_epochs
        if args.matlab_max_epochs is not None
        else int(profile.get("matlab_max_epochs", counts.get("max_epochs", 0)))
    )
    return skip_epochs, max_epochs


def build_matlab_dump_command(args: argparse.Namespace) -> list[str]:
    script = _matlab_single_quoted_path(repo_relative_path(args.matlab_dump_script))
    return [str(args.matlab_bin), "-batch", f"run('{script}')"]


def matlab_dump_env(args: argparse.Namespace) -> dict[str, str]:
    env = os.environ.copy()
    env["GNSSPP_TAROZ_ROOT"] = str(repo_relative_path(args.taroz_root).resolve())
    env["GNSSPP_TAROZ_POS_VEL_AMB_PDC_EXAMPLE_DIR"] = str(
        repo_relative_path(matlab_example_dir(args)).resolve()
    )
    if args.matlab_data_dir is not None:
        env["GNSSPP_TAROZ_POS_VEL_AMB_PDC_DATA_DIR"] = str(
            repo_relative_path(args.matlab_data_dir).resolve()
        )
    env["GNSSPP_TAROZ_POS_VEL_AMB_PDC_OUT_DIR"] = str(matlab_output_dir(args))
    skip_epochs, max_epochs = matlab_window(args)
    env["GNSSPP_TAROZ_POS_VEL_AMB_PDC_SKIP_EPOCHS"] = str(skip_epochs)
    env["GNSSPP_TAROZ_POS_VEL_AMB_PDC_MAX_EPOCHS"] = str(max_epochs)
    return env


def validate_inputs(args: argparse.Namespace) -> None:
    missing = [
        path
        for path in (args.obs, args.base, args.nav, args.seed_pos)
        if not Path(path).exists()
    ]
    if missing:
        joined = ", ".join(str(path) for path in missing)
        raise SystemExit(f"missing taroz ambiguity PDC input file(s): {joined}")


def validate_matlab_dump_inputs(args: argparse.Namespace) -> None:
    missing = [
        path
        for path in (
            args.matlab_dump_script,
            args.taroz_root,
            matlab_example_dir(args),
        )
        if not repo_relative_path(path).exists()
    ]
    if missing:
        joined = ", ".join(str(path) for path in missing)
        raise SystemExit(f"missing taroz MATLAB oracle input path(s): {joined}")


def run_command(command: list[str], *, env: dict[str, str] | None = None) -> int:
    completed = subprocess.run(command, cwd=ROOT_DIR, env=env, check=False)
    return completed.returncode


def load_native_summary(path: Path) -> dict[str, Any]:
    with path.open(encoding="utf-8") as handle:
        return json.load(handle)


def selected_native_summary(summary: dict[str, Any]) -> dict[str, Any]:
    keys = [
        "preset",
        "backend",
        "optimized_epochs",
        "valid_solutions",
        "fixed_solutions",
        "float_solutions",
        "double_difference_pseudorange_factors",
        "double_difference_carrier_factors",
        "single_difference_doppler_factors",
        "single_difference_tdcp_factors",
        "ambiguity_states",
        "lambda_ambiguity_candidates",
        "lambda_ambiguity_used_candidates",
        "lambda_ambiguity_attempts",
        "fixed_ambiguities",
        "seed_matched_epochs",
        "seed_interpolated_epochs",
        "double_difference_matched_base_epochs",
        "double_difference_interpolated_base_epochs",
        "float_rejected_seed_position_divergence",
        "float_rejected_position_jump",
        "skip_epochs",
        "max_epochs",
        "seed_match_tolerance_s",
        "seed_interpolation_max_gap_s",
        "max_float_seed_position_divergence_m",
        "max_float_position_jump_m",
        "lambda_ratio_threshold",
        "iterations",
        "cost_trace_csv",
        "converged",
        "initial_cost",
        "final_cost",
        "epoch_lambda_processing_time_ms",
        "epoch_lambda_covariance_solve_time_ms",
        "total_processing_time_ms",
    ]
    return {key: summary.get(key) for key in keys if key in summary}


def _close(actual: Any, expected: float, tolerance: float = 1e-9) -> bool:
    return isinstance(actual, (int, float)) and math.isclose(
        float(actual),
        expected,
        rel_tol=tolerance,
        abs_tol=tolerance,
    )


def _is_finite_number(value: Any) -> bool:
    return isinstance(value, (int, float)) and math.isfinite(float(value))


def verify_native_summary(
    summary: dict[str, Any],
    profile_name: str = "default",
) -> list[str]:
    failures: list[str] = []
    profile = expectation_profile(profile_name)
    if summary.get("preset") != "taroz-amb-pdc":
        failures.append("native summary preset is not taroz-amb-pdc")
    if summary.get("backend") != "eigen":
        failures.append("native summary backend is not eigen")
    if summary.get("debug_problem_only") is not False:
        failures.append("taroz-amb-pdc dogfood must run full optimization")
    if summary.get("use_spp_seed") is not False:
        failures.append("taroz-amb-pdc must report use_spp_seed=false")
    if summary.get("use_pseudorange_factors") is not False:
        failures.append("taroz-amb-pdc must disable raw pseudorange factors")
    if summary.get("use_single_difference_doppler_factors") is not True:
        failures.append("taroz-amb-pdc must enable SD Doppler factors")
    if summary.get("use_single_difference_tdcp_factors") is not True:
        failures.append("taroz-amb-pdc must enable SD TDCP factors")
    if summary.get("use_velocity_states") is not True:
        failures.append("taroz-amb-pdc must enable velocity states")
    if summary.get("use_velocity_motion_factors") is not True:
        failures.append("taroz-amb-pdc must enable velocity motion factors")
    if summary.get("use_ambiguity_between_factors") is not True:
        failures.append("taroz-amb-pdc must enable ambiguity between factors")
    if summary.get("linearize_double_difference_factors_at_seed") is not True:
        failures.append("taroz-amb-pdc must seed-linearize DD factors")
    if (
        summary.get("use_epoch_lambda_fixed_output")
        is not profile["use_epoch_lambda_fixed_output"]
    ):
        failures.append(
            "taroz-amb-pdc use_epoch_lambda_fixed_output must be "
            f"{str(profile['use_epoch_lambda_fixed_output']).lower()} "
            f"for profile {profile_name}"
        )
    if summary.get("dd_ambiguity_per_epoch") is not True:
        failures.append("taroz-amb-pdc must use per-epoch DD ambiguity states")
    if summary.get("use_ambiguity_priors") is not False:
        failures.append("taroz-amb-pdc must disable ambiguity priors")
    if summary.get("reject_rover_carrier_lli") is not True:
        failures.append("taroz-amb-pdc must reject rover carrier LLI")
    if summary.get("insert_fixed_interval_gaps") is not True:
        failures.append("taroz-amb-pdc must insert fixed interval gaps")
    if summary.get("exclude_glonass_qzss_sbas") is not True:
        failures.append("taroz-amb-pdc must exclude GLONASS/QZSS/SBAS")

    numeric_expectations = {
        "pseudorange_huber_threshold_sigma": 1.234,
        "carrier_phase_huber_threshold_sigma": 1.234,
        "tdcp_huber_threshold_sigma": 1.234,
        "velocity_motion_sigma_m": 0.01,
        "ambiguity_between_sigma_cycles": 0.001,
        "seed_match_tolerance_s": profile["seed_match_tolerance_s"],
        "seed_interpolation_max_gap_s": profile[
            "seed_interpolation_max_gap_s"
        ],
        "lambda_ratio_threshold": profile["lambda_ratio_threshold"],
        "min_snr_dbhz": 35.0,
        "min_satellites_per_epoch": 0,
        "min_output_dd_carrier_factors_per_epoch": 6,
        "max_float_seed_position_divergence_m": profile[
            "max_float_seed_position_divergence_m"
        ],
        "max_float_position_jump_m": profile["max_float_position_jump_m"],
    }
    for key, expected in numeric_expectations.items():
        if not _close(summary.get(key), expected):
            failures.append(f"taroz-amb-pdc {key} must be {expected}")

    for key, expected in profile["counts"].items():
        if summary.get(key) != expected:
            failures.append(
                f"taroz-amb-pdc {key} must be {expected} "
                f"for profile {profile_name}"
            )

    if summary.get("lambda_ambiguity_fix_solved") is not True:
        failures.append("taroz-amb-pdc LAMBDA must solve")
    if (
        summary.get("lambda_ambiguity_fix_used")
        is not profile["lambda_ambiguity_fix_used"]
    ):
        failures.append(
            "taroz-amb-pdc lambda_ambiguity_fix_used must be "
            f"{str(profile['lambda_ambiguity_fix_used']).lower()} "
            f"for profile {profile_name}"
        )
    if summary.get("partial_lambda_ambiguity_fix_used") is not False:
        failures.append("taroz-amb-pdc must not require partial LAMBDA retry")
    if summary.get("fixed_solution") is not profile["fixed_solution"]:
        failures.append(
            "taroz-amb-pdc fixed_solution must be "
            f"{str(profile['fixed_solution']).lower()} "
            f"for profile {profile_name}"
        )
    if summary.get("converged") is not True:
        failures.append("taroz-amb-pdc optimization must converge")
    if not _is_finite_number(summary.get("final_cost")):
        failures.append("taroz-amb-pdc final_cost must be finite")
    if not _is_finite_number(summary.get("total_processing_time_ms")):
        failures.append("taroz-amb-pdc total_processing_time_ms must be finite")
    return failures


def verify_cost_trace(path: Path, summary: dict[str, Any]) -> list[str]:
    failures: list[str] = []
    if not path.exists():
        return [f"taroz-amb-pdc cost trace is missing: {path}"]

    with path.open(newline="", encoding="utf-8") as handle:
        rows = list(csv.DictReader(handle))
    if not rows:
        return ["taroz-amb-pdc cost trace must not be empty"]

    required_columns = {
        "phase",
        "local_iteration",
        "global_iteration",
        "cost",
    }
    missing_columns = required_columns - set(rows[0])
    if missing_columns:
        failures.append(
            "taroz-amb-pdc cost trace missing columns: "
            + ", ".join(sorted(missing_columns))
        )
        return failures

    if isinstance(summary.get("iterations"), int):
        expected_rows = int(summary["iterations"]) + 1
        if len(rows) != expected_rows:
            failures.append(
                f"taroz-amb-pdc cost trace rows must be {expected_rows}"
            )

    global_iterations: list[int] = []
    costs: list[float] = []
    for index, row in enumerate(rows):
        try:
            global_iterations.append(int(row["global_iteration"]))
            costs.append(float(row["cost"]))
        except ValueError:
            failures.append(f"taroz-amb-pdc cost trace row {index} is not numeric")
            continue
        if row["phase"] not in {"float", "fixed"}:
            failures.append(
                f"taroz-amb-pdc cost trace row {index} has invalid phase "
                f"{row['phase']!r}"
            )

    if global_iterations != sorted(global_iterations):
        failures.append("taroz-amb-pdc cost trace global iterations must be sorted")
    if len(global_iterations) != len(set(global_iterations)):
        failures.append("taroz-amb-pdc cost trace global iterations must be unique")
    if any(not math.isfinite(cost) or cost < 0.0 for cost in costs):
        failures.append("taroz-amb-pdc cost trace costs must be finite non-negative")
    if costs and _is_finite_number(summary.get("initial_cost")):
        if not _close(costs[0], float(summary["initial_cost"])):
            failures.append("taroz-amb-pdc cost trace initial cost must match summary")
    if costs and _is_finite_number(summary.get("final_cost")):
        if not _close(costs[-1], float(summary["final_cost"])):
            failures.append("taroz-amb-pdc cost trace final cost must match summary")
    if len(costs) >= 2 and costs[-1] >= costs[0]:
        failures.append("taroz-amb-pdc cost trace final cost must decrease")
    return failures


def write_run_summary(path: Path, payload: dict[str, Any]) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(json.dumps(payload, indent=2, sort_keys=True) + "\n", encoding="utf-8")


def run_parity(args: argparse.Namespace, paths: dict[str, Path], out_dir: Path) -> int:
    env = os.environ.copy()
    env["GNSSPP_TAROZ_POS_VEL_AMB_PDC_CPP_DIR"] = str(out_dir)
    env["GNSSPP_TAROZ_POS_VEL_AMB_PDC_MATLAB_DIR"] = str(args.matlab_dir)
    env["GNSSPP_TAROZ_POS_VEL_AMB_PDC_CPP_OPT_POS"] = str(paths["pos"])
    env["GNSSPP_TAROZ_POS_VEL_AMB_PDC_CPP_OPT_EPOCH_DEBUG"] = str(
        paths["epoch_debug"]
    )
    env["GNSSPP_TAROZ_POS_VEL_AMB_PDC_CPP_OPT_LAMBDA_DEBUG"] = str(
        paths["lambda_debug"]
    )
    env["GNSSPP_TAROZ_POS_VEL_AMB_PDC_CPP_OPT_COST_TRACE"] = str(
        paths["cost_trace"]
    )
    return run_command([sys.executable, str(args.parity_test)], env=env)


def parse_args(argv: list[str]) -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description=(
            "Regenerate taroz estimate_pos_vel_ambiguity_PDC-equivalent C++ "
            "outputs and verify raw factors, final output, and LAMBDA parity."
        )
    )
    parser.add_argument("--obs", type=Path, default=default_path("rover_1Hz.obs"))
    parser.add_argument("--base", type=Path, default=default_path("base.obs"))
    parser.add_argument("--nav", type=Path, default=default_path("base.nav"))
    parser.add_argument(
        "--seed-pos",
        type=Path,
        default=default_path("rover_1Hz_spp.pos"),
        help="RTKLIB POS used as taroz x_ini seed.",
    )
    parser.add_argument("--out-dir", type=Path, default=DEFAULT_OUT_DIR)
    parser.add_argument(
        "--summary-json",
        type=Path,
        default=None,
        help=(
            "Harness summary JSON path (default: "
            "<out-dir>/taroz_pos_vel_amb_pdc_dogfood_summary.json)."
        ),
    )
    parser.add_argument("--fgo-bin", type=Path, default=None)
    parser.add_argument(
        "--fgo-extra-arg",
        action="append",
        default=[],
        help="Extra single argument passed to gnss_fgo. Repeat for multiple tokens.",
    )
    parser.add_argument("--matlab-dir", type=Path, default=DEFAULT_MATLAB_DIR)
    parser.add_argument(
        "--generate-matlab-dump",
        action="store_true",
        help=(
            "Run the taroz MATLAB dump script before C++ and parity checks, so "
            "--matlab-dir is regenerated from the reference implementation."
        ),
    )
    parser.add_argument(
        "--matlab-bin",
        type=Path,
        default=Path("matlab"),
        help="MATLAB executable used with --generate-matlab-dump.",
    )
    parser.add_argument(
        "--taroz-root",
        type=Path,
        default=DEFAULT_TAROZ_ROOT,
        help="Root of the taroz/PPC-Dataset MATLAB checkout.",
    )
    parser.add_argument(
        "--taroz-example-dir",
        type=Path,
        default=None,
        help=(
            "Directory containing estimate_pos_vel_ambiguity_PDC.m "
            "(default: <taroz-root>/examples)."
        ),
    )
    parser.add_argument(
        "--matlab-dump-script",
        type=Path,
        default=DEFAULT_MATLAB_DUMP_SCRIPT,
        help="MATLAB script that exports taroz ambiguity PDC oracle CSVs.",
    )
    parser.add_argument(
        "--matlab-data-dir",
        type=Path,
        default=None,
        help=(
            "Optional data directory passed to the MATLAB dump script. Use this "
            "for public PPC-Dataset windows prepared in taroz example format."
        ),
    )
    parser.add_argument(
        "--matlab-skip-epochs",
        type=int,
        default=None,
        help=(
            "Override the generated MATLAB oracle window skip. By default, "
            "the selected expectation profile controls the MATLAB window."
        ),
    )
    parser.add_argument(
        "--matlab-max-epochs",
        type=int,
        default=None,
        help=(
            "Override the generated MATLAB oracle window length. By default, "
            "the selected expectation profile controls the MATLAB window."
        ),
    )
    parser.add_argument("--parity-test", type=Path, default=DEFAULT_PARITY_TEST)
    parser.add_argument(
        "--expectation-profile",
        choices=sorted(EXPECTATION_PROFILES),
        default="default",
        help=(
            "Native summary contract to enforce. Use non-default profiles with "
            "matching --fgo-extra-arg overrides."
        ),
    )
    parser.add_argument("--skip-parity", action="store_true")
    parser.add_argument("--dry-run", action="store_true")
    args = parser.parse_args(argv)
    if args.matlab_skip_epochs is not None and args.matlab_skip_epochs < 0:
        raise SystemExit("--matlab-skip-epochs must be non-negative")
    if args.matlab_max_epochs is not None and args.matlab_max_epochs < 0:
        raise SystemExit("--matlab-max-epochs must be non-negative")
    return args


def main(argv: list[str] | None = None) -> int:
    args = parse_args(sys.argv[1:] if argv is None else argv)
    out_dir = args.out_dir
    paths = output_paths(out_dir)
    profile = expectation_profile(args.expectation_profile)
    matlab_skip_epochs, matlab_max_epochs = matlab_window(args)
    harness_summary = args.summary_json or (
        out_dir / "taroz_pos_vel_amb_pdc_dogfood_summary.json"
    )
    payload: dict[str, Any] = {
        "status": "planned",
        "inputs": {
            "obs": str(args.obs),
            "base": str(args.base),
            "nav": str(args.nav),
            "seed_pos": str(args.seed_pos),
        },
        "out_dir": str(out_dir),
        "outputs": {name: str(path) for name, path in paths.items()},
        "harness_summary_json": str(harness_summary),
        "matlab_dir": str(args.matlab_dir),
        "matlab_dump": {
            "enabled": args.generate_matlab_dump,
            "command": build_matlab_dump_command(args)
            if args.generate_matlab_dump
            else None,
            "dump_script": str(args.matlab_dump_script),
            "taroz_root": str(args.taroz_root),
            "example_dir": str(matlab_example_dir(args)),
            "out_dir": str(matlab_output_dir(args)),
            "skip_epochs": matlab_skip_epochs,
            "max_epochs": matlab_max_epochs,
        },
        "expected": {
            "profile": args.expectation_profile,
            "preset": "taroz-amb-pdc",
            "backend": "eigen",
            "counts": profile["counts"],
            "lambda_ratio_threshold": profile["lambda_ratio_threshold"],
            "seed_match_tolerance_s": profile["seed_match_tolerance_s"],
            "seed_interpolation_max_gap_s": profile[
                "seed_interpolation_max_gap_s"
            ],
            "max_float_seed_position_divergence_m": profile[
                "max_float_seed_position_divergence_m"
            ],
            "max_float_position_jump_m": profile["max_float_position_jump_m"],
            "use_epoch_lambda_fixed_output": profile[
                "use_epoch_lambda_fixed_output"
            ],
            "lambda_ambiguity_fix_used": profile["lambda_ambiguity_fix_used"],
            "fixed_solution": profile["fixed_solution"],
        },
    }

    command = build_fgo_command(args, paths)
    payload["fgo_command"] = command

    if args.dry_run:
        payload["status"] = "dry-run"
        write_run_summary(harness_summary, payload)
        print("Dry run. Planned taroz ambiguity PDC dogfood command:")
        if args.generate_matlab_dump:
            print(" ".join(payload["matlab_dump"]["command"]))
        print(" ".join(command))
        print(f"Harness summary: {harness_summary}")
        return 0

    validate_inputs(args)
    if args.generate_matlab_dump:
        validate_matlab_dump_inputs(args)
    out_dir.mkdir(parents=True, exist_ok=True)

    if args.generate_matlab_dump:
        matlab_output_dir(args).mkdir(parents=True, exist_ok=True)
        matlab_command = build_matlab_dump_command(args)
        matlab_returncode = run_command(matlab_command, env=matlab_dump_env(args))
        payload["matlab_dump"]["returncode"] = matlab_returncode
        if matlab_returncode != 0:
            payload["status"] = "failed"
            write_run_summary(harness_summary, payload)
            return matlab_returncode

    fgo_returncode = run_command(command)
    payload["fgo_returncode"] = fgo_returncode
    if fgo_returncode != 0:
        payload["status"] = "failed"
        write_run_summary(harness_summary, payload)
        return fgo_returncode

    native_summary = load_native_summary(paths["summary"])
    payload["native_summary"] = selected_native_summary(native_summary)
    native_failures = verify_native_summary(
        native_summary,
        args.expectation_profile,
    )
    cost_trace_failures = verify_cost_trace(paths["cost_trace"], native_summary)
    payload["cost_trace_failures"] = cost_trace_failures
    native_failures.extend(cost_trace_failures)
    payload["native_summary_failures"] = native_failures
    if native_failures:
        payload["status"] = "failed"
        write_run_summary(harness_summary, payload)
        for failure in native_failures:
            print(f"Error: {failure}", file=sys.stderr)
        return 1

    if args.skip_parity:
        payload["parity"] = {"status": "skipped"}
    else:
        parity_returncode = run_parity(args, paths, out_dir)
        payload["parity"] = {
            "status": "ok" if parity_returncode == 0 else "failed",
            "returncode": parity_returncode,
            "test": str(args.parity_test),
            "matlab_dir": str(args.matlab_dir),
        }
        if parity_returncode != 0:
            payload["status"] = "failed"
            write_run_summary(harness_summary, payload)
            return parity_returncode

    payload["status"] = "ok"
    write_run_summary(harness_summary, payload)
    print(f"Taroz ambiguity PDC dogfood OK: {out_dir}")
    print(f"Harness summary: {harness_summary}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
