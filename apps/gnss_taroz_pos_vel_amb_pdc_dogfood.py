#!/usr/bin/env python3
"""Regenerate and verify taroz position/velocity ambiguity PDC artifacts."""

from __future__ import annotations

import argparse
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
DEFAULT_OUT_DIR = ROOT_DIR / "output/dogfood/taroz_pos_vel_amb_pdc_current"
DEFAULT_MATLAB_DIR = (
    ROOT_DIR / "output/dogfood/taroz_matlab_pos_vel_amb_pdc_debug"
)
DEFAULT_PARITY_TEST = ROOT_DIR / "tests/test_taroz_pos_vel_amb_pdc_factor_parity.py"

POS_NAME = "opt.pos"
SUMMARY_NAME = "summary.json"
EPOCH_DEBUG_NAME = "epoch_debug.csv"
FACTOR_DEBUG_NAME = "factor_debug.csv"
SD_FACTOR_DEBUG_NAME = "sd_factor_debug.csv"
LAMBDA_DEBUG_NAME = "lambda_debug.csv"

EXPECTED_FULL_COUNTS = {
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
    "iterations": 24,
}


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
        "--quiet",
    ]
    command.extend(args.fgo_extra_arg)
    return command


def validate_inputs(args: argparse.Namespace) -> None:
    missing = [
        path
        for path in (args.obs, args.base, args.nav, args.seed_pos)
        if not Path(path).exists()
    ]
    if missing:
        joined = ", ".join(str(path) for path in missing)
        raise SystemExit(f"missing taroz ambiguity PDC input file(s): {joined}")


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
        "iterations",
        "converged",
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


def verify_native_summary(summary: dict[str, Any]) -> list[str]:
    failures: list[str] = []
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
    if summary.get("use_epoch_lambda_fixed_output") is not True:
        failures.append("taroz-amb-pdc must enable epoch LAMBDA fixed output")
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
        "min_snr_dbhz": 35.0,
        "min_satellites_per_epoch": 0,
        "min_output_dd_carrier_factors_per_epoch": 6,
    }
    for key, expected in numeric_expectations.items():
        if not _close(summary.get(key), expected):
            failures.append(f"taroz-amb-pdc {key} must be {expected}")

    for key, expected in EXPECTED_FULL_COUNTS.items():
        if summary.get(key) != expected:
            failures.append(f"taroz-amb-pdc {key} must be {expected}")

    if summary.get("lambda_ambiguity_fix_solved") is not True:
        failures.append("taroz-amb-pdc LAMBDA must solve")
    if summary.get("lambda_ambiguity_fix_used") is not True:
        failures.append("taroz-amb-pdc LAMBDA fixed output must be used")
    if summary.get("partial_lambda_ambiguity_fix_used") is not False:
        failures.append("taroz-amb-pdc must not require partial LAMBDA retry")
    if summary.get("fixed_solution") is not True:
        failures.append("taroz-amb-pdc summary must report fixed_solution=true")
    if summary.get("converged") is not True:
        failures.append("taroz-amb-pdc optimization must converge")
    if not _is_finite_number(summary.get("final_cost")):
        failures.append("taroz-amb-pdc final_cost must be finite")
    if not _is_finite_number(summary.get("total_processing_time_ms")):
        failures.append("taroz-amb-pdc total_processing_time_ms must be finite")
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
    parser.add_argument("--parity-test", type=Path, default=DEFAULT_PARITY_TEST)
    parser.add_argument("--skip-parity", action="store_true")
    parser.add_argument("--dry-run", action="store_true")
    return parser.parse_args(argv)


def main(argv: list[str] | None = None) -> int:
    args = parse_args(sys.argv[1:] if argv is None else argv)
    out_dir = args.out_dir
    paths = output_paths(out_dir)
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
        "expected": {
            "preset": "taroz-amb-pdc",
            "backend": "eigen",
            "counts": EXPECTED_FULL_COUNTS,
        },
    }

    command = build_fgo_command(args, paths)
    payload["fgo_command"] = command

    if args.dry_run:
        payload["status"] = "dry-run"
        write_run_summary(harness_summary, payload)
        print("Dry run. Planned taroz ambiguity PDC dogfood command:")
        print(" ".join(command))
        print(f"Harness summary: {harness_summary}")
        return 0

    validate_inputs(args)
    out_dir.mkdir(parents=True, exist_ok=True)

    fgo_returncode = run_command(command)
    payload["fgo_returncode"] = fgo_returncode
    if fgo_returncode != 0:
        payload["status"] = "failed"
        write_run_summary(harness_summary, payload)
        return fgo_returncode

    native_summary = load_native_summary(paths["summary"])
    payload["native_summary"] = selected_native_summary(native_summary)
    native_failures = verify_native_summary(native_summary)
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
