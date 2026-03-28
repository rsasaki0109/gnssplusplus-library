#!/usr/bin/env python3
"""Run and validate gnss live with realtime/error-handling thresholds."""

from __future__ import annotations

import argparse
import json
import os
from pathlib import Path
import subprocess
import sys

from gnss_runtime import ensure_input_exists, parse_summary_metrics, resolve_gnss_command


ROOT_DIR = Path(__file__).resolve().parent.parent


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(prog=os.environ.get("GNSS_CLI_NAME"))
    parser.add_argument("--rover-rtcm", type=Path, default=None, help="Rover RTCM input for gnss live.")
    parser.add_argument("--rover-ubx", type=Path, default=None, help="Rover UBX input for gnss live.")
    parser.add_argument("--base-rtcm", type=Path, default=None, help="Base RTCM input for gnss live.")
    parser.add_argument("--nav-rinex", type=Path, default=None, help="Optional navigation RINEX for UBX rover mode.")
    parser.add_argument("--out", type=Path, default=ROOT_DIR / "output/live_signoff.pos", help="Output .pos path.")
    parser.add_argument(
        "--summary-json",
        type=Path,
        default=ROOT_DIR / "output/live_signoff_summary.json",
        help="Output path for a machine-readable sign-off summary JSON.",
    )
    parser.add_argument(
        "--log-out",
        type=Path,
        default=None,
        help="Optional captured stdout/stderr log path.",
    )
    parser.add_argument("--max-epochs", type=int, default=3, help="Max written solution epochs (default: 3).")
    parser.add_argument(
        "--use-existing-log",
        type=Path,
        default=None,
        help="Skip execution and summarize an existing gnss live log file.",
    )
    parser.add_argument("--require-termination", default=None, help="Expected termination token.")
    parser.add_argument("--require-aligned-epochs-min", type=int, default=None)
    parser.add_argument("--require-written-solutions-min", type=int, default=None)
    parser.add_argument("--require-fixed-solutions-min", type=int, default=None)
    parser.add_argument("--require-rover-decoder-errors-max", type=int, default=None)
    parser.add_argument("--require-base-decoder-errors-max", type=int, default=None)
    parser.add_argument("--require-realtime-factor-min", type=float, default=None)
    parser.add_argument("--require-effective-epoch-rate-min", type=float, default=None)
    parser.add_argument("--require-solver-wall-time-max", type=float, default=None)
    return parser.parse_args()


def read_summary_line(text: str) -> str:
    for line in reversed(text.splitlines()):
        if line.startswith("summary:"):
            return line.strip()
    raise SystemExit("gnss live log did not contain a `summary:` line")


def build_summary_payload(
    args: argparse.Namespace,
    summary_line: str,
    stdout_text: str,
    stderr_text: str,
    exit_code: int | None,
) -> dict[str, object]:
    metrics = parse_summary_metrics(summary_line)
    payload = {
        "out": str(args.out),
        "summary_json": str(args.summary_json),
        "log_out": str(args.log_out) if args.log_out is not None else None,
        "execution_mode": "existing_log" if args.use_existing_log is not None else "live",
        "exit_code": exit_code,
        "summary_line": summary_line,
        "metrics": metrics,
        "stdout": stdout_text,
        "stderr": stderr_text,
    }
    args.summary_json.parent.mkdir(parents=True, exist_ok=True)
    args.summary_json.write_text(json.dumps(payload, indent=2, sort_keys=True) + "\n", encoding="utf-8")
    return payload


def enforce_requirements(payload: dict[str, object], args: argparse.Namespace) -> None:
    metrics = payload["metrics"]
    assert isinstance(metrics, dict)
    failures: list[str] = []

    def metric(name: str) -> object | None:
        return metrics.get(name)

    def require_min(name: str, threshold: float | int | None) -> None:
        if threshold is None:
            return
        value = metric(name)
        if value is None:
            failures.append(f"missing `{name}` in summary")
        elif float(value) < float(threshold):
            failures.append(f"{name} {float(value):.6f} < {float(threshold):.6f}")

    def require_max(name: str, threshold: float | int | None) -> None:
        if threshold is None:
            return
        value = metric(name)
        if value is None:
            failures.append(f"missing `{name}` in summary")
        elif float(value) > float(threshold):
            failures.append(f"{name} {float(value):.6f} > {float(threshold):.6f}")

    if args.require_termination is not None:
        termination = metric("termination")
        if termination != args.require_termination:
            failures.append(f"termination `{termination}` != `{args.require_termination}`")

    require_min("aligned_epochs", args.require_aligned_epochs_min)
    require_min("written_solutions", args.require_written_solutions_min)
    require_min("fixed_solutions", args.require_fixed_solutions_min)
    require_min("realtime_factor", args.require_realtime_factor_min)
    require_min("effective_epoch_rate_hz", args.require_effective_epoch_rate_min)
    require_max("rover_decoder_errors", args.require_rover_decoder_errors_max)
    require_max("base_decoder_errors", args.require_base_decoder_errors_max)
    require_max("solver_wall_time_s", args.require_solver_wall_time_max)

    if failures:
        raise SystemExit(
            "Live sign-off checks failed:\n" + "\n".join(f"  - {failure}" for failure in failures)
        )


def build_live_command(args: argparse.Namespace) -> list[str]:
    if bool(args.rover_rtcm) == bool(args.rover_ubx):
        raise SystemExit("Specify exactly one of --rover-rtcm or --rover-ubx")
    if args.base_rtcm is None:
        raise SystemExit("--base-rtcm is required unless --use-existing-log is used")
    if args.rover_rtcm is not None:
        ensure_input_exists(args.rover_rtcm, "rover RTCM input", ROOT_DIR)
    if args.rover_ubx is not None:
        ensure_input_exists(args.rover_ubx, "rover UBX input", ROOT_DIR)
    ensure_input_exists(args.base_rtcm, "base RTCM input", ROOT_DIR)
    if args.nav_rinex is not None:
        ensure_input_exists(args.nav_rinex, "navigation RINEX input", ROOT_DIR)

    command = [*resolve_gnss_command(ROOT_DIR), "live"]
    if args.rover_rtcm is not None:
        command += ["--rover-rtcm", str(args.rover_rtcm)]
    if args.rover_ubx is not None:
        command += ["--rover-ubx", str(args.rover_ubx)]
    command += ["--base-rtcm", str(args.base_rtcm)]
    if args.nav_rinex is not None:
        command += ["--nav-rinex", str(args.nav_rinex)]
    command += ["--out", str(args.out), "--max-epochs", str(args.max_epochs), "--quiet"]
    return command


def main() -> int:
    args = parse_args()
    args.summary_json.parent.mkdir(parents=True, exist_ok=True)
    if args.log_out is not None:
        args.log_out.parent.mkdir(parents=True, exist_ok=True)

    stdout_text = ""
    stderr_text = ""
    exit_code: int | None = None

    if args.use_existing_log is not None:
        ensure_input_exists(args.use_existing_log, "existing live log", ROOT_DIR)
        stdout_text = args.use_existing_log.read_text(encoding="utf-8")
        if args.log_out is None:
            args.log_out = args.use_existing_log
    else:
        args.out.parent.mkdir(parents=True, exist_ok=True)
        command = build_live_command(args)
        print("+", " ".join(command))
        completed = subprocess.run(command, cwd=ROOT_DIR, capture_output=True, text=True)
        stdout_text = completed.stdout
        stderr_text = completed.stderr
        exit_code = completed.returncode
        if args.log_out is not None:
            args.log_out.write_text(stdout_text + stderr_text, encoding="utf-8")
        if completed.returncode != 0:
            raise SystemExit(
                f"gnss live failed with exit code {completed.returncode}\n{completed.stdout}{completed.stderr}"
            )

    summary_line = read_summary_line(stdout_text + ("\n" + stderr_text if stderr_text else ""))
    payload = build_summary_payload(args, summary_line, stdout_text, stderr_text, exit_code)
    enforce_requirements(payload, args)

    metrics = payload["metrics"]
    assert isinstance(metrics, dict)
    print("Finished live sign-off.")
    print(f"  summary: {args.summary_json}")
    print(f"  termination: {metrics.get('termination', 'n/a')}")
    print(f"  written_solutions: {metrics.get('written_solutions', 'n/a')}")
    print(f"  fixed_solutions: {metrics.get('fixed_solutions', 'n/a')}")
    print(f"  realtime_factor: {metrics.get('realtime_factor', 'n/a')}")
    print(f"  effective_epoch_rate_hz: {metrics.get('effective_epoch_rate_hz', 'n/a')}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
