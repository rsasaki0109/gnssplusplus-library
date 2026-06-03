#!/usr/bin/env python3
"""Regenerate and verify the taroz PC FGO dogfood artifacts."""

from __future__ import annotations

import argparse
import filecmp
import glob
import json
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
DEFAULT_OUT_DIR = ROOT_DIR / "output/dogfood/taroz_pc_dogfood_current"
DEFAULT_BASELINE_CPP_DIR = (
    ROOT_DIR / "output/dogfood/taroz_pc_full_gtsam_fixed_taroz_nocorr"
)
DEFAULT_BASELINE_LAMBDA_DIR = ROOT_DIR / "output/dogfood/taroz_pc_lambda_debug"
DEFAULT_MATLAB_DIR = ROOT_DIR / "output/dogfood/taroz_matlab_pc_debug"
DEFAULT_MATLAB_DUMP_SCRIPT = ROOT_DIR / "scripts/dump_taroz_pc_debug.m"
DEFAULT_PARITY_TEST = ROOT_DIR / "tests/test_taroz_pc_internal_parity.py"

POS_NAME = "fgo_taroz_pc_gtsam_fixed.pos"
SUMMARY_NAME = "fgo_taroz_pc_gtsam_fixed_summary.json"
EPOCH_DEBUG_NAME = "fgo_taroz_pc_gtsam_fixed_epoch_debug.csv"
FACTOR_DEBUG_NAME = "fgo_taroz_pc_gtsam_fixed_factor_debug.csv"
LAMBDA_DEBUG_NAME = "fgo_taroz_pc_gtsam_fixed_lambda_debug.csv"


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
        "taroz-pc",
        "--backend",
        "gtsam-pc",
        "--fix-ambiguities",
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
        raise SystemExit(f"missing taroz PC input file(s): {joined}")


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
    env["GNSSPP_TAROZ_PC_EXAMPLE_DIR"] = str(
        repo_relative_path(matlab_example_dir(args)).resolve()
    )
    env["GNSSPP_TAROZ_PC_OUT_DIR"] = str(matlab_output_dir(args))
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
        raise SystemExit(f"missing taroz PC MATLAB oracle input path(s): {joined}")


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
        "fixed_solutions",
        "float_solutions",
        "double_difference_pseudorange_factors",
        "double_difference_carrier_factors",
        "lambda_ambiguity_candidates",
        "lambda_ambiguity_used_candidates",
        "iterations",
        "initial_cost",
        "final_cost",
    ]
    return {key: summary.get(key) for key in keys if key in summary}


def verify_native_summary(summary: dict[str, Any]) -> list[str]:
    failures: list[str] = []
    if summary.get("preset") != "taroz-pc":
        failures.append("native summary preset is not taroz-pc")
    if summary.get("backend") != "gtsam-pc":
        failures.append("native summary backend is not gtsam-pc")
    if summary.get("use_spp_seed") is not False:
        failures.append("taroz-pc must report use_spp_seed=false")
    return failures


def compare_file_pairs(
    out_dir: Path,
    baseline_cpp_dir: Path,
    baseline_lambda_dir: Path,
) -> dict[str, dict[str, Any]]:
    current = output_paths(out_dir)
    pairs = {
        "pos": (baseline_cpp_dir / POS_NAME, current["pos"]),
        "epoch_debug": (baseline_cpp_dir / EPOCH_DEBUG_NAME, current["epoch_debug"]),
        "factor_debug": (baseline_cpp_dir / FACTOR_DEBUG_NAME, current["factor_debug"]),
        "lambda_debug": (baseline_lambda_dir / LAMBDA_DEBUG_NAME, current["lambda_debug"]),
    }
    results: dict[str, dict[str, Any]] = {}
    for name, (baseline, generated) in pairs.items():
        result: dict[str, Any] = {
            "baseline": str(baseline),
            "generated": str(generated),
        }
        if not baseline.exists() or not generated.exists():
            result["status"] = "missing"
        elif baseline.resolve() == generated.resolve():
            result["status"] = "same-file"
        elif filecmp.cmp(baseline, generated, shallow=False):
            result["status"] = "same"
        else:
            result["status"] = "different"
        results[name] = result
    return results


def compare_summaries(
    generated_summary: Path,
    baseline_summary: Path,
) -> dict[str, Any]:
    ignored = {
        "obs",
        "base",
        "nav",
        "out",
        "epoch_debug_csv",
        "factor_debug_csv",
        "lambda_debug_csv",
        "processing_time_ms",
        "max_iterations",
    }
    result: dict[str, Any] = {
        "baseline": str(baseline_summary),
        "generated": str(generated_summary),
    }
    if not baseline_summary.exists() or not generated_summary.exists():
        result["status"] = "missing"
        return result

    baseline = load_native_summary(baseline_summary)
    generated = load_native_summary(generated_summary)
    keys = sorted(set(baseline) - ignored)
    diffs = [
        {
            "key": key,
            "baseline": baseline.get(key),
            "generated": generated.get(key),
        }
        for key in keys
        if baseline.get(key) != generated.get(key)
    ]
    result["status"] = "same" if not diffs else "different"
    result["checked_keys"] = len(keys)
    result["extra_generated_keys"] = sorted(set(generated) - set(baseline) - ignored)
    result["diffs"] = diffs
    return result


def byte_comparison_failures(results: dict[str, dict[str, Any]]) -> list[str]:
    failures: list[str] = []
    for name, result in results.items():
        status = result.get("status")
        if status == "different":
            failures.append(f"{name} differs from baseline")
    return failures


def run_parity(args: argparse.Namespace, out_dir: Path) -> int:
    env = os.environ.copy()
    env["GNSSPP_TAROZ_PC_CPP_DIR"] = str(out_dir)
    env["GNSSPP_TAROZ_PC_CPP_LAMBDA_DIR"] = str(out_dir)
    env["GNSSPP_TAROZ_PC_MATLAB_DIR"] = str(matlab_output_dir(args))
    return run_command([sys.executable, str(args.parity_test)], env=env)


def write_run_summary(path: Path, payload: dict[str, Any]) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(json.dumps(payload, indent=2, sort_keys=True) + "\n", encoding="utf-8")


def parse_args(argv: list[str]) -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description=(
            "Regenerate taroz estimate_pos_ambiguity_PC-equivalent C++ outputs "
            "and verify intermediate/final parity."
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
        help="Harness summary JSON path (default: <out-dir>/taroz_pc_dogfood_summary.json).",
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
            "Run the taroz MATLAB PC dump script before C++ and parity checks, "
            "so --matlab-dir is regenerated from the reference implementation."
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
        help="Directory containing the taroz MATLAB example (default: <taroz-root>/examples).",
    )
    parser.add_argument(
        "--matlab-dump-script",
        type=Path,
        default=DEFAULT_MATLAB_DUMP_SCRIPT,
        help="MATLAB script that exports the taroz PC oracle CSVs.",
    )
    parser.add_argument("--parity-test", type=Path, default=DEFAULT_PARITY_TEST)
    parser.add_argument("--skip-parity", action="store_true")
    parser.add_argument("--baseline-cpp-dir", type=Path, default=DEFAULT_BASELINE_CPP_DIR)
    parser.add_argument(
        "--baseline-lambda-dir",
        type=Path,
        default=DEFAULT_BASELINE_LAMBDA_DIR,
    )
    parser.add_argument("--no-byte-compare", action="store_true")
    parser.add_argument("--dry-run", action="store_true")
    return parser.parse_args(argv)


def main(argv: list[str] | None = None) -> int:
    args = parse_args(sys.argv[1:] if argv is None else argv)
    out_dir = args.out_dir
    paths = output_paths(out_dir)
    harness_summary = args.summary_json or (out_dir / "taroz_pc_dogfood_summary.json")
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
        },
        "expected": {
            "preset": "taroz-pc",
            "backend": "gtsam-pc",
            "use_spp_seed": False,
        },
    }

    command = build_fgo_command(args, paths)
    payload["fgo_command"] = command

    if args.dry_run:
        payload["status"] = "dry-run"
        write_run_summary(harness_summary, payload)
        print("Dry run. Planned taroz PC dogfood command:")
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
    native_failures = verify_native_summary(native_summary)
    payload["native_summary_failures"] = native_failures
    if native_failures:
        payload["status"] = "failed"
        write_run_summary(harness_summary, payload)
        for failure in native_failures:
            print(f"Error: {failure}", file=sys.stderr)
        return 1

    if args.no_byte_compare or args.generate_matlab_dump:
        reason = (
            "disabled by --no-byte-compare"
            if args.no_byte_compare
            else "generated MATLAB oracle requested"
        )
        payload["byte_comparison"] = {"status": "skipped", "reason": reason}
    else:
        byte_comparison = compare_file_pairs(
            out_dir,
            args.baseline_cpp_dir,
            args.baseline_lambda_dir,
        )
        byte_comparison["summary"] = compare_summaries(
            paths["summary"],
            args.baseline_cpp_dir / SUMMARY_NAME,
        )
        payload["byte_comparison"] = byte_comparison
        byte_failures = byte_comparison_failures(byte_comparison)
        payload["byte_comparison_failures"] = byte_failures
        if byte_failures:
            payload["status"] = "failed"
            write_run_summary(harness_summary, payload)
            for failure in byte_failures:
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
    print(f"Taroz PC dogfood OK: {out_dir}")
    print(f"Harness summary: {harness_summary}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
