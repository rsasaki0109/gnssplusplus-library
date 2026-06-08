#!/usr/bin/env python3
"""Regenerate and verify taroz P FGO smoke artifacts."""

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
DEFAULT_TAROZ_ROOT = Path("/tmp/taroz_gtsam_gnss")
DEFAULT_OUT_DIR = ROOT_DIR / "output/dogfood/taroz_p_dogfood_current"
DEFAULT_MATLAB_DIR = ROOT_DIR / "output/dogfood/taroz_matlab_p_debug"
DEFAULT_MATLAB_DUMP_SCRIPT = ROOT_DIR / "scripts/dump_taroz_p_debug.m"
DEFAULT_PARITY_TEST = ROOT_DIR / "tests/test_taroz_p_internal_parity.py"
DEFAULT_MAX_EPOCHS = 80
DEFAULT_MAX_ITERATIONS = 40

POS_NAME = "fgo_taroz_p.pos"
SUMMARY_NAME = "fgo_taroz_p_summary.json"
EPOCH_DEBUG_NAME = "fgo_taroz_p_epoch_debug.csv"
FACTOR_DEBUG_NAME = "fgo_taroz_p_factor_debug.csv"


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
        "taroz-p",
        "--obs",
        str(args.obs),
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
        "--quiet",
    ]
    if args.max_epochs > 0:
        command.extend(["--max-epochs", str(args.max_epochs)])
    if args.max_iterations > 0:
        command.extend(["--max-iterations", str(args.max_iterations)])
    if args.debug_problem_only:
        command.append("--debug-problem-only")
    command.extend(args.fgo_extra_arg)
    return command


def validate_inputs(args: argparse.Namespace) -> None:
    missing = [
        path
        for path in (args.obs, args.nav, args.seed_pos)
        if not Path(path).exists()
    ]
    if missing:
        joined = ", ".join(str(path) for path in missing)
        raise SystemExit(f"missing taroz P input file(s): {joined}")


def _matlab_single_quoted_path(path: Path) -> str:
    return str(path).replace("'", "''")


def repo_relative_path(path: Path) -> Path:
    return path if path.is_absolute() else ROOT_DIR / path


def matlab_example_dir(args: argparse.Namespace) -> Path:
    return args.taroz_example_dir or (args.taroz_root / "examples")


def matlab_output_dir(args: argparse.Namespace) -> Path:
    return repo_relative_path(args.matlab_dir).resolve()


def build_matlab_dump_command(args: argparse.Namespace) -> list[str]:
    script = _matlab_single_quoted_path(repo_relative_path(args.matlab_dump_script))
    return [str(args.matlab_bin), "-batch", f"run('{script}')"]


def matlab_dump_env(args: argparse.Namespace) -> dict[str, str]:
    env = os.environ.copy()
    env["GNSSPP_TAROZ_ROOT"] = str(repo_relative_path(args.taroz_root).resolve())
    env["GNSSPP_TAROZ_P_EXAMPLE_DIR"] = str(
        repo_relative_path(matlab_example_dir(args)).resolve()
    )
    env["GNSSPP_TAROZ_P_OUT_DIR"] = str(matlab_output_dir(args))
    return env


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
        raise SystemExit(f"missing taroz P MATLAB oracle input path(s): {joined}")


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
        "use_spp_seed",
        "optimized_epochs",
        "valid_solutions",
        "pseudorange_factors",
        "tdcp_factors",
        "carrier_phase_factors",
        "double_difference_pseudorange_factors",
        "double_difference_carrier_factors",
        "motion_factors",
        "iterations",
        "converged",
        "final_cost",
        "residual_rms_m",
        "debug_problem_only",
        "graph_factors",
        "graph_values",
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
    *,
    expected_max_epochs: int,
    expected_max_iterations: int,
    debug_problem_only: bool = False,
) -> list[str]:
    failures: list[str] = []
    if bool(summary.get("debug_problem_only", False)) != debug_problem_only:
        failures.append("native summary debug_problem_only does not match request")
    if summary.get("preset") != "taroz-p":
        failures.append("native summary preset is not taroz-p")
    if summary.get("backend") != "eigen":
        failures.append("native summary backend is not eigen")
    if summary.get("use_spp_seed") is not False:
        failures.append("taroz-p must report use_spp_seed=false")
    if summary.get("use_pseudorange_factors") is not True:
        failures.append("taroz-p must enable raw pseudorange factors")
    if not _close(summary.get("pseudorange_sigma_m"), 3.0):
        failures.append("taroz-p pseudorange_sigma_m must be 3")
    if not _close(summary.get("pseudorange_elevation_sigma_power"), 0.5):
        failures.append("taroz-p elevation sigma power must be 0.5")
    if not _close(summary.get("position_prior_sigma_m"), 1000.0):
        failures.append("taroz-p position prior sigma must be 1000")
    if not _close(summary.get("clock_prior_sigma_m"), 1e6):
        failures.append("taroz-p clock prior sigma must be 1e6")
    if not _close(summary.get("clock_motion_sigma_m"), 100.0):
        failures.append("taroz-p clock motion sigma must be 100")
    if summary.get("use_position_motion_factors") is not False:
        failures.append("taroz-p must disable position motion factors")
    if summary.get("use_clock_motion_factors") is not True:
        failures.append("taroz-p must enable clock motion factors")

    zero_factor_keys = [
        "tdcp_factors",
        "carrier_phase_factors",
        "double_difference_pseudorange_factors",
        "double_difference_carrier_factors",
        "ambiguity_states",
    ]
    for key in zero_factor_keys:
        if summary.get(key) != 0:
            failures.append(f"taroz-p must keep {key}=0")

    optimized_epochs = summary.get("optimized_epochs")
    if not isinstance(optimized_epochs, int) or optimized_epochs <= 0:
        failures.append("taroz-p optimized_epochs must be positive")
    elif expected_max_epochs > 0 and optimized_epochs != expected_max_epochs:
        failures.append(
            f"taroz-p optimized_epochs must equal requested max epochs "
            f"({expected_max_epochs})"
        )

    if not isinstance(summary.get("valid_solutions"), int) or summary["valid_solutions"] <= 0:
        failures.append("taroz-p valid_solutions must be positive")
    if not isinstance(summary.get("pseudorange_factors"), int) or summary["pseudorange_factors"] <= 0:
        failures.append("taroz-p pseudorange_factors must be positive")
    if isinstance(optimized_epochs, int) and optimized_epochs > 0:
        expected_motion_factors = max(0, optimized_epochs - 1)
        if summary.get("motion_factors") != expected_motion_factors:
            failures.append(
                f"taroz-p motion_factors must be {expected_motion_factors}"
            )

    iterations = summary.get("iterations")
    if debug_problem_only:
        if iterations != 0:
            failures.append("taroz-p debug-only iterations must be 0")
    elif not isinstance(iterations, int) or iterations <= 0:
        failures.append("taroz-p iterations must be positive")
    elif expected_max_iterations > 0 and iterations > expected_max_iterations:
        failures.append("taroz-p iterations exceeded requested max iterations")
    if not debug_problem_only and summary.get("converged") is not True:
        failures.append("taroz-p smoke should converge")
    if not _is_finite_number(summary.get("final_cost")):
        failures.append("taroz-p final_cost must be finite")
    if not _is_finite_number(summary.get("residual_rms_m")):
        failures.append("taroz-p residual_rms_m must be finite")
    return failures


def write_run_summary(path: Path, payload: dict[str, Any]) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(json.dumps(payload, indent=2, sort_keys=True) + "\n", encoding="utf-8")


def run_parity(args: argparse.Namespace, out_dir: Path) -> int:
    env = os.environ.copy()
    env["GNSSPP_TAROZ_P_CPP_DIR"] = str(out_dir)
    env["GNSSPP_TAROZ_P_MATLAB_DIR"] = str(args.matlab_dir)
    return run_command([sys.executable, str(args.parity_test)], env=env)


def parse_args(argv: list[str]) -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description=(
            "Regenerate taroz estimate_pos_P-equivalent C++ outputs and verify "
            "the raw-pseudorange FGO configuration."
        )
    )
    parser.add_argument("--obs", type=Path, default=default_path("rover_1Hz.obs"))
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
        help="Harness summary JSON path (default: <out-dir>/taroz_p_dogfood_summary.json).",
    )
    parser.add_argument("--fgo-bin", type=Path, default=None)
    parser.add_argument(
        "--max-epochs",
        type=int,
        default=DEFAULT_MAX_EPOCHS,
        help="Epoch cap for the smoke run; 0 keeps the preset default.",
    )
    parser.add_argument(
        "--max-iterations",
        type=int,
        default=DEFAULT_MAX_ITERATIONS,
        help="Iteration cap for the smoke run; 0 keeps the preset default.",
    )
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
        help="Directory containing estimate_pos_P.m (default: <taroz-root>/examples).",
    )
    parser.add_argument(
        "--matlab-dump-script",
        type=Path,
        default=DEFAULT_MATLAB_DUMP_SCRIPT,
        help="MATLAB script that exports taroz P oracle CSVs.",
    )
    parser.add_argument("--parity-test", type=Path, default=DEFAULT_PARITY_TEST)
    parser.add_argument(
        "--debug-problem-only",
        action="store_true",
        help="Build full factor/debug outputs without running FGO optimization.",
    )
    parser.add_argument("--skip-parity", action="store_true")
    parser.add_argument("--dry-run", action="store_true")
    return parser.parse_args(argv)


def main(argv: list[str] | None = None) -> int:
    args = parse_args(sys.argv[1:] if argv is None else argv)
    if args.max_epochs < 0:
        raise SystemExit("--max-epochs must be non-negative")
    if args.max_iterations < 0:
        raise SystemExit("--max-iterations must be non-negative")

    out_dir = args.out_dir
    paths = output_paths(out_dir)
    harness_summary = args.summary_json or (out_dir / "taroz_p_dogfood_summary.json")
    payload: dict[str, Any] = {
        "status": "planned",
        "inputs": {
            "obs": str(args.obs),
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
        },
        "expected": {
            "preset": "taroz-p",
            "backend": "eigen",
            "use_spp_seed": False,
            "max_epochs": args.max_epochs,
            "max_iterations": args.max_iterations,
            "debug_problem_only": args.debug_problem_only,
        },
    }

    command = build_fgo_command(args, paths)
    payload["fgo_command"] = command

    if args.dry_run:
        payload["status"] = "dry-run"
        write_run_summary(harness_summary, payload)
        print("Dry run. Planned taroz P dogfood command:")
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
        expected_max_epochs=args.max_epochs,
        expected_max_iterations=args.max_iterations,
        debug_problem_only=args.debug_problem_only,
    )
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
        parity_returncode = run_parity(args, out_dir)
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
    print(f"Taroz P dogfood OK: {out_dir}")
    print(f"Harness summary: {harness_summary}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
