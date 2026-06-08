#!/usr/bin/env python3
"""Regenerate taroz observable-mode dogfood artifacts and optional MATLAB parity."""

from __future__ import annotations

import argparse
from dataclasses import dataclass
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
DEFAULT_MAX_EPOCHS = 0
DEFAULT_MAX_ITERATIONS = 1000

FACTOR_DEBUG_NAME = "factor_debug.csv"
GRAPH_DETAIL_NAME = "graph_detail.csv"
SUMMARY_NAME = "summary.json"


@dataclass(frozen=True)
class ModeSpec:
    key: str
    label: str
    target: str
    preset: str
    matlab_prefix: str
    matlab_script: Path
    example_function: str
    out_dir: Path
    matlab_dir: Path
    parity_test: Path
    out_csv_name: str
    factor_keys: tuple[str, ...]
    valid_epoch_keys: tuple[str, ...]


MODE_SPECS = {
    "d": ModeSpec(
        key="d",
        label="taroz D",
        target="gnss_vel_d",
        preset="taroz-d",
        matlab_prefix="D",
        matlab_script=ROOT_DIR / "scripts/dump_taroz_d_debug.m",
        example_function="estimate_vel_D.m",
        out_dir=ROOT_DIR / "output/dogfood/taroz_d_dogfood_current",
        matlab_dir=ROOT_DIR / "output/dogfood/taroz_matlab_d_debug",
        parity_test=ROOT_DIR / "tests/test_taroz_d_internal_parity.py",
        out_csv_name="per_epoch_vel.csv",
        factor_keys=("doppler_factors",),
        valid_epoch_keys=("valid_velocity_epochs",),
    ),
    "pos-pd": ModeSpec(
        key="pos-pd",
        label="taroz position PD",
        target="gnss_pos_pd",
        preset="taroz-pos-pd",
        matlab_prefix="POS_PD",
        matlab_script=ROOT_DIR / "scripts/dump_taroz_pos_pd_debug.m",
        example_function="estimate_pos_PD.m",
        out_dir=ROOT_DIR / "output/dogfood/taroz_pos_pd_dogfood_current",
        matlab_dir=ROOT_DIR / "output/dogfood/taroz_matlab_pos_pd_debug",
        parity_test=ROOT_DIR / "tests/test_taroz_pos_pd_internal_parity.py",
        out_csv_name="per_epoch_state.csv",
        factor_keys=("pseudorange_factors", "doppler_factors"),
        valid_epoch_keys=("valid_position_epochs", "valid_clock_epochs"),
    ),
    "pos-pdc": ModeSpec(
        key="pos-pdc",
        label="taroz position PDC",
        target="gnss_pos_pdc",
        preset="taroz-pos-pdc",
        matlab_prefix="POS_PDC",
        matlab_script=ROOT_DIR / "scripts/dump_taroz_pos_pdc_debug.m",
        example_function="estimate_pos_PDC.m",
        out_dir=ROOT_DIR / "output/dogfood/taroz_pos_pdc_dogfood_current",
        matlab_dir=ROOT_DIR / "output/dogfood/taroz_matlab_pos_pdc_debug",
        parity_test=ROOT_DIR / "tests/test_taroz_pos_pdc_internal_parity.py",
        out_csv_name="per_epoch_state.csv",
        factor_keys=("pseudorange_factors", "doppler_factors", "tdcp_factors"),
        valid_epoch_keys=("valid_position_epochs", "valid_clock_epochs"),
    ),
    "pos-vel-pdc": ModeSpec(
        key="pos-vel-pdc",
        label="taroz position/velocity PDC",
        target="gnss_pos_vel_pdc",
        preset="taroz-pdc",
        matlab_prefix="POS_VEL_PDC",
        matlab_script=ROOT_DIR / "scripts/dump_taroz_pos_vel_pdc_debug.m",
        example_function="estimate_pos_vel_PDC.m",
        out_dir=ROOT_DIR / "output/dogfood/taroz_pos_vel_pdc_dogfood_current",
        matlab_dir=ROOT_DIR / "output/dogfood/taroz_matlab_pos_vel_pdc_debug",
        parity_test=ROOT_DIR / "tests/test_taroz_pos_vel_pdc_internal_parity.py",
        out_csv_name="per_epoch_state.csv",
        factor_keys=("pseudorange_factors", "doppler_factors", "tdcp_factors"),
        valid_epoch_keys=("valid_position_epochs", "valid_velocity_epochs"),
    ),
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


def mode_spec(args: argparse.Namespace) -> ModeSpec:
    return MODE_SPECS[args.mode]


def out_dir(args: argparse.Namespace, spec: ModeSpec) -> Path:
    return args.out_dir or spec.out_dir


def matlab_dir(args: argparse.Namespace, spec: ModeSpec) -> Path:
    return args.matlab_dir or spec.matlab_dir


def parity_test(args: argparse.Namespace, spec: ModeSpec) -> Path:
    return args.parity_test or spec.parity_test


def matlab_dump_script(args: argparse.Namespace, spec: ModeSpec) -> Path:
    return args.matlab_dump_script or spec.matlab_script


def output_paths(directory: Path, spec: ModeSpec) -> dict[str, Path]:
    return {
        "out_csv": directory / spec.out_csv_name,
        "factor_debug": directory / FACTOR_DEBUG_NAME,
        "graph_detail": directory / GRAPH_DETAIL_NAME,
        "summary": directory / SUMMARY_NAME,
    }


def build_native_command(
    args: argparse.Namespace,
    spec: ModeSpec,
    paths: dict[str, Path],
) -> list[str]:
    native_bin = Path(args.native_bin) if args.native_bin else find_binary(spec.target)
    if native_bin is None:
        if args.dry_run:
            native_bin = Path(spec.target)
        else:
            raise SystemExit(
                f"{spec.target} binary not found. Build it first with "
                f"`cmake --build build --target {spec.target}`, or pass --native-bin."
            )

    command = [
        str(native_bin),
        "--obs",
        str(args.obs),
        "--nav",
        str(args.nav),
        "--seed-pos",
        str(args.seed_pos),
        "--out-csv",
        str(paths["out_csv"]),
        "--factor-debug-csv",
        str(paths["factor_debug"]),
        "--graph-csv",
        str(paths["graph_detail"]),
        "--summary-json",
        str(paths["summary"]),
        "--quiet",
    ]
    if args.max_epochs > 0:
        command.extend(["--max-epochs", str(args.max_epochs)])
    if args.max_iterations > 0:
        command.extend(["--max-iterations", str(args.max_iterations)])
    command.extend(args.native_extra_arg)
    return command


def validate_inputs(args: argparse.Namespace) -> None:
    missing = [
        path
        for path in (args.obs, args.nav, args.seed_pos)
        if not Path(path).exists()
    ]
    if missing:
        joined = ", ".join(str(path) for path in missing)
        raise SystemExit(f"missing taroz observable input file(s): {joined}")


def _matlab_single_quoted_path(path: Path) -> str:
    return str(path).replace("'", "''")


def repo_relative_path(path: Path) -> Path:
    return path if path.is_absolute() else ROOT_DIR / path


def matlab_example_dir(args: argparse.Namespace) -> Path:
    return args.taroz_example_dir or (args.taroz_root / "examples")


def matlab_output_dir(args: argparse.Namespace, spec: ModeSpec) -> Path:
    return repo_relative_path(matlab_dir(args, spec)).resolve()


def build_matlab_dump_command(args: argparse.Namespace, spec: ModeSpec) -> list[str]:
    script = _matlab_single_quoted_path(
        repo_relative_path(matlab_dump_script(args, spec))
    )
    return [str(args.matlab_bin), "-batch", f"run('{script}')"]


def matlab_dump_env(args: argparse.Namespace, spec: ModeSpec) -> dict[str, str]:
    env = os.environ.copy()
    env["GNSSPP_TAROZ_ROOT"] = str(repo_relative_path(args.taroz_root).resolve())
    env[f"GNSSPP_TAROZ_{spec.matlab_prefix}_EXAMPLE_DIR"] = str(
        repo_relative_path(matlab_example_dir(args)).resolve()
    )
    env[f"GNSSPP_TAROZ_{spec.matlab_prefix}_OUT_DIR"] = str(
        matlab_output_dir(args, spec)
    )
    return env


def validate_matlab_dump_inputs(args: argparse.Namespace, spec: ModeSpec) -> None:
    missing = [
        path
        for path in (
            matlab_dump_script(args, spec),
            args.taroz_root,
            matlab_example_dir(args),
        )
        if not repo_relative_path(path).exists()
    ]
    if missing:
        joined = ", ".join(str(path) for path in missing)
        raise SystemExit(f"missing {spec.label} MATLAB oracle input path(s): {joined}")


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
        "valid_position_epochs",
        "valid_velocity_epochs",
        "valid_clock_epochs",
        "pseudorange_factors",
        "doppler_factors",
        "tdcp_factors",
        "graph_factors",
        "graph_values",
        "iterations",
        "converged",
        "initial_cost",
        "final_cost",
        "residual_rms_m",
        "residual_rms_mps",
    ]
    return {key: summary.get(key) for key in keys if key in summary}


def _is_finite_number(value: Any) -> bool:
    return isinstance(value, (int, float)) and math.isfinite(float(value))


def verify_native_summary(
    summary: dict[str, Any],
    spec: ModeSpec,
    *,
    expected_max_epochs: int,
    expected_max_iterations: int,
) -> list[str]:
    failures: list[str] = []
    if summary.get("preset") != spec.preset:
        failures.append(f"native summary preset is not {spec.preset}")
    if summary.get("backend") != "eigen":
        failures.append("native summary backend is not eigen")

    optimized_epochs = summary.get("optimized_epochs")
    if not isinstance(optimized_epochs, int) or optimized_epochs <= 0:
        failures.append(f"{spec.label} optimized_epochs must be positive")
    elif expected_max_epochs > 0 and optimized_epochs != expected_max_epochs:
        failures.append(
            f"{spec.label} optimized_epochs must equal requested max epochs "
            f"({expected_max_epochs})"
        )

    if isinstance(optimized_epochs, int) and optimized_epochs > 0:
        for key in spec.valid_epoch_keys:
            if summary.get(key) != optimized_epochs:
                failures.append(f"{spec.label} {key} must match optimized_epochs")

    for key in spec.factor_keys:
        if not isinstance(summary.get(key), int) or summary[key] <= 0:
            failures.append(f"{spec.label} {key} must be positive")

    iterations = summary.get("iterations")
    if not isinstance(iterations, int) or iterations <= 0:
        failures.append(f"{spec.label} iterations must be positive")
    elif expected_max_iterations > 0 and iterations > expected_max_iterations:
        failures.append(f"{spec.label} iterations exceeded requested max iterations")
    if summary.get("converged") is not True:
        failures.append(f"{spec.label} smoke should converge")
    if not _is_finite_number(summary.get("initial_cost")):
        failures.append(f"{spec.label} initial_cost must be finite")
    if not _is_finite_number(summary.get("final_cost")):
        failures.append(f"{spec.label} final_cost must be finite")
    return failures


def run_parity(args: argparse.Namespace, spec: ModeSpec, directory: Path) -> int:
    env = os.environ.copy()
    env[f"GNSSPP_TAROZ_{spec.matlab_prefix}_CPP_DIR"] = str(directory)
    env[f"GNSSPP_TAROZ_{spec.matlab_prefix}_MATLAB_DIR"] = str(matlab_dir(args, spec))
    return run_command([sys.executable, str(parity_test(args, spec))], env=env)


def write_run_summary(path: Path, payload: dict[str, Any]) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(json.dumps(payload, indent=2, sort_keys=True) + "\n", encoding="utf-8")


def parse_args(argv: list[str]) -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description=(
            "Regenerate taroz D/position PD/PDC/position-velocity PDC C++ outputs "
            "and optionally regenerate the matching MATLAB oracle before parity."
        )
    )
    parser.add_argument("--mode", choices=sorted(MODE_SPECS), required=True)
    parser.add_argument("--obs", type=Path, default=default_path("rover_1Hz.obs"))
    parser.add_argument("--nav", type=Path, default=default_path("base.nav"))
    parser.add_argument(
        "--seed-pos",
        type=Path,
        default=default_path("rover_1Hz_spp.pos"),
        help="RTKLIB POS used as taroz x_ini seed.",
    )
    parser.add_argument("--out-dir", type=Path, default=None)
    parser.add_argument(
        "--summary-json",
        type=Path,
        default=None,
        help="Harness summary JSON path (default: <out-dir>/taroz_<mode>_dogfood_summary.json).",
    )
    parser.add_argument("--native-bin", type=Path, default=None)
    parser.add_argument(
        "--max-epochs",
        type=int,
        default=DEFAULT_MAX_EPOCHS,
        help="Epoch cap for the run; 0 keeps the native full-run default.",
    )
    parser.add_argument(
        "--max-iterations",
        type=int,
        default=DEFAULT_MAX_ITERATIONS,
        help="Iteration cap for the run; 0 keeps the native default.",
    )
    parser.add_argument(
        "--native-extra-arg",
        action="append",
        default=[],
        help="Extra single argument passed to the native binary. Repeat for multiple tokens.",
    )
    parser.add_argument("--matlab-dir", type=Path, default=None)
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
        help="Directory containing the taroz MATLAB example (default: <taroz-root>/examples).",
    )
    parser.add_argument(
        "--matlab-dump-script",
        type=Path,
        default=None,
        help="MATLAB script that exports the selected taroz oracle CSVs.",
    )
    parser.add_argument("--parity-test", type=Path, default=None)
    parser.add_argument("--skip-parity", action="store_true")
    parser.add_argument("--dry-run", action="store_true")
    return parser.parse_args(argv)


def main(argv: list[str] | None = None) -> int:
    args = parse_args(sys.argv[1:] if argv is None else argv)
    if args.max_epochs < 0:
        raise SystemExit("--max-epochs must be non-negative")
    if args.max_iterations < 0:
        raise SystemExit("--max-iterations must be non-negative")

    spec = mode_spec(args)
    directory = out_dir(args, spec)
    paths = output_paths(directory, spec)
    harness_summary = args.summary_json or (
        directory / f"taroz_{spec.key.replace('-', '_')}_dogfood_summary.json"
    )
    payload: dict[str, Any] = {
        "status": "planned",
        "mode": spec.key,
        "inputs": {
            "obs": str(args.obs),
            "nav": str(args.nav),
            "seed_pos": str(args.seed_pos),
        },
        "out_dir": str(directory),
        "outputs": {name: str(path) for name, path in paths.items()},
        "harness_summary_json": str(harness_summary),
        "matlab_dir": str(matlab_dir(args, spec)),
        "matlab_dump": {
            "enabled": args.generate_matlab_dump,
            "command": build_matlab_dump_command(args, spec)
            if args.generate_matlab_dump
            else None,
            "dump_script": str(matlab_dump_script(args, spec)),
            "taroz_root": str(args.taroz_root),
            "example_dir": str(matlab_example_dir(args)),
            "out_dir": str(matlab_output_dir(args, spec)),
        },
        "expected": {
            "preset": spec.preset,
            "backend": "eigen",
            "max_epochs": args.max_epochs,
            "max_iterations": args.max_iterations,
        },
    }

    command = build_native_command(args, spec, paths)
    payload["native_command"] = command

    if args.dry_run:
        payload["status"] = "dry-run"
        write_run_summary(harness_summary, payload)
        print(f"Dry run. Planned {spec.label} dogfood command:")
        if args.generate_matlab_dump:
            print(" ".join(payload["matlab_dump"]["command"]))
        print(" ".join(command))
        print(f"Harness summary: {harness_summary}")
        return 0

    validate_inputs(args)
    if args.generate_matlab_dump:
        validate_matlab_dump_inputs(args, spec)
    directory.mkdir(parents=True, exist_ok=True)

    if args.generate_matlab_dump:
        matlab_output_dir(args, spec).mkdir(parents=True, exist_ok=True)
        matlab_command = build_matlab_dump_command(args, spec)
        matlab_returncode = run_command(matlab_command, env=matlab_dump_env(args, spec))
        payload["matlab_dump"]["returncode"] = matlab_returncode
        if matlab_returncode != 0:
            payload["status"] = "failed"
            write_run_summary(harness_summary, payload)
            return matlab_returncode

    native_returncode = run_command(command)
    payload["native_returncode"] = native_returncode
    if native_returncode != 0:
        payload["status"] = "failed"
        write_run_summary(harness_summary, payload)
        return native_returncode

    native_summary = load_native_summary(paths["summary"])
    payload["native_summary"] = selected_native_summary(native_summary)
    native_failures = verify_native_summary(
        native_summary,
        spec,
        expected_max_epochs=args.max_epochs,
        expected_max_iterations=args.max_iterations,
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
        parity_returncode = run_parity(args, spec, directory)
        payload["parity"] = {
            "status": "ok" if parity_returncode == 0 else "failed",
            "returncode": parity_returncode,
            "test": str(parity_test(args, spec)),
            "matlab_dir": str(matlab_dir(args, spec)),
        }
        if parity_returncode != 0:
            payload["status"] = "failed"
            write_run_summary(harness_summary, payload)
            return parity_returncode

    payload["status"] = "ok"
    write_run_summary(harness_summary, payload)
    print(f"{spec.label} dogfood OK: {directory}")
    print(f"Harness summary: {harness_summary}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
