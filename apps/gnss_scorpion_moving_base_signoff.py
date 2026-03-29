#!/usr/bin/env python3
"""Prepare and validate the public SCORPION moving-base ROS2 bag workflow."""

from __future__ import annotations

import argparse
import json
import os
from pathlib import Path
import subprocess
from urllib import parse, request

from gnss_runtime import resolve_gnss_command


ROOT_DIR = Path(__file__).resolve().parent.parent
DEFAULT_SCORPION_ZIP_URL = "https://zenodo.org/api/records/8083431/files/2023-06-14T174658Z.zip/content"


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(prog=os.environ.get("GNSS_CLI_NAME"))
    parser.add_argument("--input", type=Path, default=None, help="Local SCORPION bag directory or zip path.")
    parser.add_argument(
        "--input-url",
        default=None,
        help="Optional SCORPION zip URL. If omitted, a local --input is required.",
    )
    parser.add_argument(
        "--download-cache-dir",
        type=Path,
        default=ROOT_DIR / "output" / "downloads",
        help="Directory for downloaded SCORPION zips.",
    )
    parser.add_argument(
        "--work-dir",
        type=Path,
        default=ROOT_DIR / "output" / "scorpion_moving_base",
        help="Directory for generated UBX/reference/intermediate artifacts.",
    )
    parser.add_argument(
        "--out",
        type=Path,
        default=None,
        help="Output .pos path. Defaults to <work-dir>/scorpion_moving_base.pos.",
    )
    parser.add_argument(
        "--summary-json",
        type=Path,
        default=None,
        help="Output JSON summary path. Defaults to <work-dir>/scorpion_moving_base_summary.json.",
    )
    parser.add_argument("--log-out", type=Path, default=None)
    parser.add_argument("--nav-rinex", type=Path, default=None, help="Optional BRDC nav file to skip fetch-products.")
    parser.add_argument("--max-epochs", type=int, default=120)
    parser.add_argument("--match-tolerance-s", type=float, default=0.25)
    parser.add_argument("--use-existing-solution", action="store_true")
    parser.add_argument("--solver-wall-time-s", type=float, default=None)
    parser.add_argument("--require-valid-epochs-min", type=int, default=None)
    parser.add_argument("--require-matched-epochs-min", type=int, default=None)
    parser.add_argument("--require-fix-rate-min", type=float, default=None)
    parser.add_argument("--require-median-baseline-error-max", type=float, default=None)
    parser.add_argument("--require-p95-baseline-error-max", type=float, default=None)
    parser.add_argument("--require-max-baseline-error-max", type=float, default=None)
    parser.add_argument("--require-p95-heading-error-max", type=float, default=None)
    parser.add_argument("--require-heading-samples-min", type=int, default=None)
    parser.add_argument("--require-solver-wall-time-max", type=float, default=None)
    parser.add_argument("--require-realtime-factor-min", type=float, default=None)
    parser.add_argument("--require-effective-epoch-rate-min", type=float, default=None)
    parser.add_argument("--require-termination", default=None)
    parser.add_argument("--require-rover-decoder-errors-max", type=int, default=None)
    parser.add_argument("--require-base-decoder-errors-max", type=int, default=None)
    return parser.parse_args()


def resolve_output_paths(args: argparse.Namespace) -> dict[str, Path]:
    work_dir = args.work_dir
    return {
        "work_dir": work_dir,
        "rover_ubx": work_dir / "rover.ubx",
        "base_ubx": work_dir / "base.ubx",
        "reference_csv": work_dir / "reference.csv",
        "prepare_summary_json": work_dir / "prepare_summary.json",
        "products_summary_json": work_dir / "products_summary.json",
        "out": args.out or (work_dir / "scorpion_moving_base.pos"),
        "summary_json": args.summary_json or (work_dir / "scorpion_moving_base_summary.json"),
    }


def ensure_local_input(args: argparse.Namespace) -> tuple[Path, str | None]:
    if args.input is not None:
        if not args.input.exists():
            raise SystemExit(f"Missing SCORPION moving-base input: {args.input}")
        return args.input, None
    if args.input_url is None:
        raise SystemExit("Provide --input or --input-url for scorpion-moving-base-signoff")

    args.download_cache_dir.mkdir(parents=True, exist_ok=True)
    parsed = parse.urlparse(args.input_url)
    filename = Path(parsed.path).name or "scorpion_moving_base.zip"
    destination = args.download_cache_dir / filename
    if not destination.exists():
        with request.urlopen(args.input_url, timeout=60.0) as response, destination.open("wb") as handle:
            handle.write(response.read())
    return destination, args.input_url


def run_json_command(command: list[str]) -> dict[str, object]:
    print("+", " ".join(command))
    completed = subprocess.run(
        command,
        cwd=ROOT_DIR,
        capture_output=True,
        text=True,
        check=False,
    )
    if completed.returncode != 0:
        raise SystemExit(
            f"{Path(command[1]).name if len(command) > 1 else command[0]} failed with exit code {completed.returncode}\n"
            f"{completed.stdout}{completed.stderr}"
        )
    stdout = completed.stdout.strip()
    if not stdout:
        return {}
    try:
        payload = json.loads(stdout)
    except json.JSONDecodeError as exc:
        raise SystemExit(
            "Expected JSON output from command\n"
            f"{completed.stdout}{completed.stderr}"
        ) from exc
    if not isinstance(payload, dict):
        raise SystemExit("Expected JSON object from command")
    return payload


def run_checked_command(command: list[str], *, capture_output: bool = False) -> subprocess.CompletedProcess[str]:
    print("+", " ".join(command))
    completed = subprocess.run(
        command,
        cwd=ROOT_DIR,
        capture_output=capture_output,
        text=True,
        check=False,
    )
    if completed.returncode != 0:
        raise SystemExit(
            f"command failed with exit code {completed.returncode}\n"
            f"{completed.stdout}{completed.stderr}"
        )
    return completed


def build_signoff_command(
    args: argparse.Namespace,
    paths: dict[str, Path],
    nav_rinex: Path | None,
) -> list[str]:
    command = [
        *resolve_gnss_command(ROOT_DIR),
        "moving-base-signoff",
        "--solver",
        "replay",
        "--rover-ubx",
        str(paths["rover_ubx"]),
        "--base-ubx",
        str(paths["base_ubx"]),
        "--reference-csv",
        str(paths["reference_csv"]),
        "--out",
        str(paths["out"]),
        "--summary-json",
        str(paths["summary_json"]),
        "--max-epochs",
        str(args.max_epochs),
        "--match-tolerance-s",
        str(args.match_tolerance_s),
    ]
    if nav_rinex is not None:
        command.extend(["--nav-rinex", str(nav_rinex)])
    if args.log_out is not None:
        command.extend(["--log-out", str(args.log_out)])
    if args.use_existing_solution:
        command.append("--use-existing-solution")
    if args.solver_wall_time_s is not None:
        command.extend(["--solver-wall-time-s", str(args.solver_wall_time_s)])

    threshold_options = {
        "--require-valid-epochs-min": args.require_valid_epochs_min,
        "--require-matched-epochs-min": args.require_matched_epochs_min,
        "--require-fix-rate-min": args.require_fix_rate_min,
        "--require-median-baseline-error-max": args.require_median_baseline_error_max,
        "--require-p95-baseline-error-max": args.require_p95_baseline_error_max,
        "--require-max-baseline-error-max": args.require_max_baseline_error_max,
        "--require-p95-heading-error-max": args.require_p95_heading_error_max,
        "--require-heading-samples-min": args.require_heading_samples_min,
        "--require-solver-wall-time-max": args.require_solver_wall_time_max,
        "--require-realtime-factor-min": args.require_realtime_factor_min,
        "--require-effective-epoch-rate-min": args.require_effective_epoch_rate_min,
        "--require-rover-decoder-errors-max": args.require_rover_decoder_errors_max,
        "--require-base-decoder-errors-max": args.require_base_decoder_errors_max,
    }
    for option, value in threshold_options.items():
        if value is not None:
            command.extend([option, str(value)])
    if args.require_termination is not None:
        command.extend(["--require-termination", args.require_termination])
    return command


def main() -> int:
    args = parse_args()
    paths = resolve_output_paths(args)
    paths["work_dir"].mkdir(parents=True, exist_ok=True)

    local_input, input_url = ensure_local_input(args)

    prepare_command = [
        *resolve_gnss_command(ROOT_DIR),
        "moving-base-prepare",
        "--input",
        str(local_input),
        "--rover-ubx-out",
        str(paths["rover_ubx"]),
        "--base-ubx-out",
        str(paths["base_ubx"]),
        "--reference-csv",
        str(paths["reference_csv"]),
        "--summary-json",
        str(paths["prepare_summary_json"]),
        "--max-epochs",
        str(args.max_epochs),
        "--quiet",
    ]
    run_checked_command(prepare_command, capture_output=True)
    prepare_payload = json.loads(paths["prepare_summary_json"].read_text(encoding="utf-8"))

    nav_rinex = args.nav_rinex
    products_payload: dict[str, object] | None = None
    if not args.use_existing_solution and nav_rinex is None:
        fetch_command = [
            *resolve_gnss_command(ROOT_DIR),
            "fetch-products",
            "--date",
            str(prepare_payload["date"]),
            "--preset",
            "brdc-nav",
            "--summary-json",
            str(paths["products_summary_json"]),
        ]
        products_payload = run_json_command(fetch_command)
        products = products_payload.get("products")
        if not isinstance(products, dict) or "nav" not in products:
            raise SystemExit("fetch-products did not return a nav product for scorpion-moving-base-signoff")
        nav_rinex = Path(str(products["nav"]))

    signoff_command = build_signoff_command(args, paths, nav_rinex)
    run_checked_command(signoff_command)

    summary_payload = json.loads(paths["summary_json"].read_text(encoding="utf-8"))
    summary_payload["signoff_profile"] = "scorpion-moving-base"
    summary_payload["input"] = str(local_input)
    summary_payload["input_url"] = input_url
    summary_payload["prepare_summary_json"] = str(paths["prepare_summary_json"])
    summary_payload["products_summary_json"] = (
        str(paths["products_summary_json"]) if paths["products_summary_json"].exists() else None
    )
    summary_payload["prepare_summary"] = prepare_payload
    summary_payload["products_summary"] = products_payload
    summary_payload["nav_rinex"] = str(nav_rinex) if nav_rinex is not None else None
    paths["summary_json"].write_text(
        json.dumps(summary_payload, indent=2, sort_keys=True) + "\n",
        encoding="utf-8",
    )

    print("Finished SCORPION moving-base sign-off.")
    print(f"  input: {local_input}")
    print(f"  summary: {paths['summary_json']}")
    print(f"  prepare_summary: {paths['prepare_summary_json']}")
    if nav_rinex is not None:
        print(f"  nav_rinex: {nav_rinex}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
