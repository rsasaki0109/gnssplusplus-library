#!/usr/bin/env python3
"""Run the taroz MATLAB-oracle dogfood suite across all ported modes."""

from __future__ import annotations

import argparse
from dataclasses import dataclass
import json
import os
from pathlib import Path
import subprocess
import sys
import time
from typing import Any


ROOT_DIR = Path(__file__).resolve().parents[1]
APPS_DIR = Path(__file__).resolve().parent
GNSS_DISPATCH = APPS_DIR / "gnss.py"
EXE_SUFFIX = ".exe" if os.name == "nt" else ""

DEFAULT_DATA_DIR = Path("/tmp/taroz_gtsam_gnss/examples/data")
DEFAULT_TAROZ_ROOT = Path("/tmp/taroz_gtsam_gnss")
DEFAULT_OUT_ROOT = ROOT_DIR / "output/dogfood/taroz_oracle_suite_current"


@dataclass(frozen=True)
class ModeSpec:
    key: str
    command: str
    matlab_dir: Path
    binary_arg: str | None
    binary_name: str | None
    requires_base: bool = False
    observable_mode: str | None = None
    supports_max_epochs: bool = False
    supports_max_iterations: bool = False
    extra_args: tuple[str, ...] = ()


MODE_SPECS: dict[str, ModeSpec] = {
    "p": ModeSpec(
        key="p",
        command="taroz-p-dogfood",
        matlab_dir=ROOT_DIR / "output/dogfood/taroz_matlab_p_debug",
        binary_arg="--fgo-bin",
        binary_name="gnss_fgo",
        supports_max_epochs=True,
        supports_max_iterations=True,
    ),
    "pd": ModeSpec(
        key="pd",
        command="taroz-pd-dogfood",
        matlab_dir=ROOT_DIR / "output/dogfood/taroz_matlab_pd_debug",
        binary_arg="--pd-bin",
        binary_name="gnss_pos_vel_pd",
        supports_max_epochs=True,
        supports_max_iterations=True,
    ),
    "pc": ModeSpec(
        key="pc",
        command="taroz-pc-dogfood",
        matlab_dir=ROOT_DIR / "output/dogfood/taroz_matlab_pc_debug",
        binary_arg="--fgo-bin",
        binary_name="gnss_fgo",
        requires_base=True,
        extra_args=("--no-byte-compare",),
    ),
    "d": ModeSpec(
        key="d",
        command="taroz-observable-dogfood",
        matlab_dir=ROOT_DIR / "output/dogfood/taroz_matlab_d_debug",
        binary_arg="--native-bin",
        binary_name="gnss_vel_d",
        observable_mode="d",
        supports_max_epochs=True,
        supports_max_iterations=True,
    ),
    "pos-pd": ModeSpec(
        key="pos-pd",
        command="taroz-observable-dogfood",
        matlab_dir=ROOT_DIR / "output/dogfood/taroz_matlab_pos_pd_debug",
        binary_arg="--native-bin",
        binary_name="gnss_pos_pd",
        observable_mode="pos-pd",
        supports_max_epochs=True,
        supports_max_iterations=True,
    ),
    "pos-pdc": ModeSpec(
        key="pos-pdc",
        command="taroz-observable-dogfood",
        matlab_dir=ROOT_DIR / "output/dogfood/taroz_matlab_pos_pdc_debug",
        binary_arg="--native-bin",
        binary_name="gnss_pos_pdc",
        observable_mode="pos-pdc",
        supports_max_epochs=True,
        supports_max_iterations=True,
    ),
    "pos-vel-pdc": ModeSpec(
        key="pos-vel-pdc",
        command="taroz-observable-dogfood",
        matlab_dir=ROOT_DIR / "output/dogfood/taroz_matlab_pos_vel_pdc_debug",
        binary_arg="--native-bin",
        binary_name="gnss_pos_vel_pdc",
        observable_mode="pos-vel-pdc",
        supports_max_epochs=True,
        supports_max_iterations=True,
    ),
    "pos-vel-amb-pdc": ModeSpec(
        key="pos-vel-amb-pdc",
        command="taroz-pos-vel-amb-pdc-dogfood",
        matlab_dir=ROOT_DIR / "output/dogfood/taroz_matlab_pos_vel_amb_pdc_debug",
        binary_arg="--fgo-bin",
        binary_name="gnss_fgo",
        requires_base=True,
    ),
}

MODE_ORDER = tuple(MODE_SPECS)


def default_path(name: str) -> Path:
    return DEFAULT_DATA_DIR / name


def mode_dir_name(mode: str) -> str:
    return mode.replace("-", "_")


def child_summary_name(mode: str) -> str:
    return f"taroz_{mode_dir_name(mode)}_suite_child_summary.json"


def selected_modes(args: argparse.Namespace) -> list[str]:
    requested = args.mode or list(MODE_ORDER)
    seen: set[str] = set()
    modes: list[str] = []
    for mode in requested:
        if mode not in seen:
            modes.append(mode)
            seen.add(mode)
    return modes


def generated_matlab_dir(args: argparse.Namespace, mode: str) -> Path:
    return args.out_root / "matlab" / mode_dir_name(mode)


def matlab_dir_for_mode(args: argparse.Namespace, spec: ModeSpec) -> Path:
    if args.generate_matlab_dump:
        return generated_matlab_dir(args, spec.key)
    return spec.matlab_dir


def native_binary_path(args: argparse.Namespace, spec: ModeSpec) -> Path | None:
    if args.native_bin_dir is None or spec.binary_name is None:
        return None
    return args.native_bin_dir / (spec.binary_name + EXE_SUFFIX)


def build_child_command(args: argparse.Namespace, spec: ModeSpec) -> list[str]:
    out_dir = args.out_root / mode_dir_name(spec.key)
    command = [
        sys.executable,
        str(GNSS_DISPATCH),
        spec.command,
    ]
    if spec.observable_mode is not None:
        command.extend(["--mode", spec.observable_mode])
    command.extend(
        [
            "--obs",
            str(args.obs),
            "--nav",
            str(args.nav),
            "--seed-pos",
            str(args.seed_pos),
        ]
    )
    if spec.requires_base:
        command.extend(["--base", str(args.base)])
    command.extend(
        [
            "--out-dir",
            str(out_dir),
            "--summary-json",
            str(out_dir / child_summary_name(spec.key)),
            "--matlab-dir",
            str(matlab_dir_for_mode(args, spec)),
        ]
    )
    if args.generate_matlab_dump:
        command.append("--generate-matlab-dump")
        command.extend(["--matlab-bin", str(args.matlab_bin)])
        command.extend(["--taroz-root", str(args.taroz_root)])
        if args.taroz_example_dir is not None:
            command.extend(["--taroz-example-dir", str(args.taroz_example_dir)])
    if args.skip_parity:
        command.append("--skip-parity")
    command.extend(spec.extra_args)
    binary_path = native_binary_path(args, spec)
    if binary_path is not None and spec.binary_arg is not None:
        command.extend([spec.binary_arg, str(binary_path)])
    if args.max_epochs is not None and spec.supports_max_epochs:
        command.extend(["--max-epochs", str(args.max_epochs)])
    if args.max_iterations is not None and spec.supports_max_iterations:
        command.extend(["--max-iterations", str(args.max_iterations)])
    return command


def validate_args(args: argparse.Namespace) -> None:
    if args.max_epochs is not None and args.max_epochs < 0:
        raise SystemExit("--max-epochs must be non-negative")
    if args.max_iterations is not None and args.max_iterations < 0:
        raise SystemExit("--max-iterations must be non-negative")


def validate_inputs(args: argparse.Namespace, modes: list[str]) -> None:
    paths = [args.obs, args.nav, args.seed_pos]
    if any(MODE_SPECS[mode].requires_base for mode in modes):
        paths.append(args.base)
    missing = [path for path in paths if not Path(path).exists()]
    if missing:
        joined = ", ".join(str(path) for path in missing)
        raise SystemExit(f"missing taroz oracle suite input file(s): {joined}")


def write_summary(path: Path, payload: dict[str, Any]) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(json.dumps(payload, indent=2, sort_keys=True) + "\n", encoding="utf-8")


def run_command(command: list[str]) -> int:
    completed = subprocess.run(command, cwd=ROOT_DIR, check=False)
    return completed.returncode


def initial_payload(
    args: argparse.Namespace,
    modes: list[str],
    commands: dict[str, list[str]],
    summary_json: Path,
) -> dict[str, Any]:
    return {
        "status": "planned",
        "modes": modes,
        "inputs": {
            "obs": str(args.obs),
            "base": str(args.base),
            "nav": str(args.nav),
            "seed_pos": str(args.seed_pos),
        },
        "out_root": str(args.out_root),
        "summary_json": str(summary_json),
        "generate_matlab_dump": args.generate_matlab_dump,
        "skip_parity": args.skip_parity,
        "keep_going": args.keep_going,
        "max_epochs": args.max_epochs,
        "max_iterations": args.max_iterations,
        "runs": {
            mode: {
                "status": "planned",
                "command": commands[mode],
                "out_dir": str(args.out_root / mode_dir_name(mode)),
                "summary_json": str(
                    args.out_root / mode_dir_name(mode) / child_summary_name(mode)
                ),
                "matlab_dir": str(matlab_dir_for_mode(args, MODE_SPECS[mode])),
            }
            for mode in modes
        },
    }


def parse_args(argv: list[str]) -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description=(
            "Run all taroz MATLAB-oracle dogfood harnesses from one reproducible "
            "entrypoint."
        )
    )
    parser.add_argument(
        "--mode",
        action="append",
        choices=sorted(MODE_SPECS),
        help="Mode to run. Repeat to select a subset; default runs every mode.",
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
    parser.add_argument("--out-root", type=Path, default=DEFAULT_OUT_ROOT)
    parser.add_argument(
        "--summary-json",
        type=Path,
        default=None,
        help="Suite summary JSON path (default: <out-root>/summary.json).",
    )
    parser.add_argument(
        "--native-bin-dir",
        type=Path,
        default=None,
        help=(
            "Directory containing gnss_fgo and taroz native binaries. Omit to let "
            "each dogfood harness discover build outputs."
        ),
    )
    parser.add_argument(
        "--max-epochs",
        type=int,
        default=None,
        help=(
            "Common epoch cap passed to modes that support it. Omit to keep each "
            "dogfood harness default; 0 asks those harnesses for their full run."
        ),
    )
    parser.add_argument(
        "--max-iterations",
        type=int,
        default=None,
        help="Common iteration cap passed to modes that support it.",
    )
    parser.add_argument(
        "--generate-matlab-dump",
        action="store_true",
        help="Regenerate each selected taroz MATLAB oracle under <out-root>/matlab.",
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
        help="Directory containing taroz MATLAB example functions.",
    )
    parser.add_argument("--skip-parity", action="store_true")
    parser.add_argument(
        "--keep-going",
        action="store_true",
        help="Continue running later modes after a failed child harness.",
    )
    parser.add_argument("--dry-run", action="store_true")
    return parser.parse_args(argv)


def main(argv: list[str] | None = None) -> int:
    args = parse_args(sys.argv[1:] if argv is None else argv)
    validate_args(args)
    modes = selected_modes(args)
    commands = {
        mode: build_child_command(args, MODE_SPECS[mode])
        for mode in modes
    }
    summary_json = args.summary_json or (args.out_root / "summary.json")
    payload = initial_payload(args, modes, commands, summary_json)

    if args.dry_run:
        payload["status"] = "dry-run"
        write_summary(summary_json, payload)
        print("Dry run. Planned taroz oracle suite commands:")
        for mode in modes:
            print(f"[{mode}] {' '.join(commands[mode])}")
        print(f"Suite summary: {summary_json}")
        return 0

    validate_inputs(args, modes)
    args.out_root.mkdir(parents=True, exist_ok=True)
    payload["status"] = "running"
    write_summary(summary_json, payload)

    suite_started = time.monotonic()
    failed = False
    for index, mode in enumerate(modes):
        run = payload["runs"][mode]
        run["status"] = "running"
        started = time.monotonic()
        run["started_monotonic_s"] = round(started, 3)
        write_summary(summary_json, payload)

        returncode = run_command(commands[mode])
        elapsed = time.monotonic() - started
        run["returncode"] = returncode
        run["duration_s"] = round(elapsed, 3)
        run["status"] = "passed" if returncode == 0 else "failed"
        write_summary(summary_json, payload)

        if returncode != 0:
            failed = True
            if not args.keep_going:
                for remaining in modes[index + 1 :]:
                    payload["runs"][remaining]["status"] = "skipped"
                    payload["runs"][remaining]["skip_reason"] = (
                        f"stopped after {mode} failed"
                    )
                break

    payload["duration_s"] = round(time.monotonic() - suite_started, 3)
    payload["status"] = "failed" if failed else "passed"
    write_summary(summary_json, payload)
    print(f"Taroz oracle suite {payload['status']}: {summary_json}")
    return 1 if failed else 0


if __name__ == "__main__":
    raise SystemExit(main())
