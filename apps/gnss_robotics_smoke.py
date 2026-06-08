#!/usr/bin/env python3
"""Robotics-oriented RTK realtime smoke wrapper."""

from __future__ import annotations

import argparse
import json
from pathlib import Path
import subprocess
import sys

from gnss_runtime import resolve_gnss_command


ROOT_DIR = Path(__file__).resolve().parents[1]
PROFILE_DEFAULTS = {
    "quick": {
        "max_epochs": 50,
        "positioning_rate_min": 0.0,
        "realtime_factor_min": 0.0,
        "effective_epoch_rate_min": 0.0,
        "solver_wall_time_max": 120.0,
    },
    "realtime": {
        "max_epochs": 200,
        "positioning_rate_min": 0.0,
        "realtime_factor_min": 1.0,
        "effective_epoch_rate_min": 5.0,
        "solver_wall_time_max": 3600.0,
    },
    "full": {
        "max_epochs": None,
        "positioning_rate_min": 70.0,
        "realtime_factor_min": 1.0,
        "effective_epoch_rate_min": 5.0,
        "solver_wall_time_max": 3600.0,
    },
}


def default_dataset_root(root_dir: Path) -> Path:
    for candidate in (
        root_dir / "data" / "PPC-Dataset",
        Path.cwd() / "data" / "PPC-Dataset",
        Path("/datasets/PPC-Dataset"),
    ):
        if candidate.exists():
            return candidate
    return root_dir / "data" / "PPC-Dataset"


def resolve_path(path: Path, root_dir: Path) -> Path:
    path = path.expanduser()
    if path.is_absolute():
        return path
    root_candidate = root_dir / path
    if root_candidate.exists():
        return root_candidate
    return Path.cwd() / path


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description=(
            "Run a short PPC RTK replay with robotics-friendly realtime gates "
            "and write a sign-off summary JSON."
        )
    )
    parser.add_argument("--dataset-root", type=Path, default=None)
    parser.add_argument("--city", choices=("tokyo", "nagoya"), default="tokyo")
    parser.add_argument("--run", default="run1")
    parser.add_argument("--preset", choices=("survey", "low-cost", "moving-base"), default="low-cost")
    parser.add_argument("--ratio", type=float, default=2.4)
    parser.add_argument(
        "--profile",
        choices=tuple(PROFILE_DEFAULTS),
        default="realtime",
        help=(
            "Smoke profile: quick checks wiring, realtime gates 5 Hz replay, "
            "full runs the whole dataset with availability and runtime gates."
        ),
    )
    parser.add_argument(
        "--max-epochs",
        type=int,
        default=None,
        help="Override the selected profile's epoch limit. Use 0 for full replay.",
    )
    parser.add_argument("--out-dir", type=Path, default=Path("output/robotics_smoke"))
    parser.add_argument(
        "--positioning-rate-min",
        type=float,
        default=None,
        help=(
            "Minimum positioning rate over the full reference. Keep the default "
            "for bounded --max-epochs smoke runs; raise it for full-run gates."
        ),
    )
    parser.add_argument("--realtime-factor-min", type=float, default=None)
    parser.add_argument("--effective-epoch-rate-min", type=float, default=None)
    parser.add_argument("--solver-wall-time-max", type=float, default=None)
    parser.add_argument(
        "--dry-run",
        action="store_true",
        help="Print the ppc-rtk-signoff command without running it.",
    )
    return parser.parse_args()


def load_summary(path: Path) -> dict[str, object]:
    try:
        payload = json.loads(path.read_text(encoding="utf-8"))
    except FileNotFoundError as exc:
        raise SystemExit(f"Expected summary JSON was not written: {path}") from exc
    except json.JSONDecodeError as exc:
        raise SystemExit(f"Summary JSON is invalid: {path}") from exc
    if not isinstance(payload, dict):
        raise SystemExit(f"Summary JSON is not an object: {path}")
    return payload


def metric(payload: dict[str, object], key: str) -> object:
    value = payload.get(key)
    return "n/a" if value is None else value


def profile_value(args: argparse.Namespace, name: str) -> float | int | None:
    override = getattr(args, name)
    if override is not None:
        if name == "max_epochs" and override == 0:
            return None
        return override
    return PROFILE_DEFAULTS[args.profile][name]


def append_threshold(
    command: list[str],
    thresholds: dict[str, float | int],
    option: str,
    key: str,
    value: float | int | None,
) -> None:
    if value is None:
        return
    thresholds[key] = value
    command.extend([option, str(value)])


def numeric_metric(payload: dict[str, object], key: str) -> float | None:
    value = payload.get(key)
    if isinstance(value, (int, float)):
        return float(value)
    return None


def build_failure_reasons(payload: dict[str, object],
                          thresholds: dict[str, float | int]) -> list[str]:
    failures: list[str] = []

    checks = (
        ("realtime_factor", "require_realtime_factor_min", ">=", "realtime factor", ""),
        ("effective_epoch_rate_hz", "require_effective_epoch_rate_min", ">=", "effective epoch rate", " Hz"),
        ("positioning_rate_pct", "require_positioning_rate_min", ">=", "positioning rate", "%"),
        ("solver_wall_time_s", "require_solver_wall_time_max", "<=", "solver wall time", " s"),
    )
    for metric_key, threshold_key, direction, label, suffix in checks:
        threshold = thresholds.get(threshold_key)
        if not isinstance(threshold, (int, float)):
            continue
        value = numeric_metric(payload, metric_key)
        if value is None:
            failures.append(f"{label} is unavailable")
            continue
        if direction == ">=" and value < float(threshold):
            failures.append(f"{label} {value:.6f}{suffix} < {float(threshold):.6f}{suffix}")
        if direction == "<=" and value > float(threshold):
            failures.append(f"{label} {value:.6f}{suffix} > {float(threshold):.6f}{suffix}")
    return failures


def annotate_summary(
    path: Path,
    *,
    args: argparse.Namespace,
    command: list[str],
    thresholds: dict[str, float | int],
    completed_returncode: int,
) -> dict[str, object] | None:
    if not path.exists():
        return None
    payload = load_summary(path)
    failures = build_failure_reasons(payload, thresholds)
    if completed_returncode != 0 and not failures:
        failures.append(f"ppc-rtk-signoff exited with code {completed_returncode}")
    status = "passed" if completed_returncode == 0 and not failures else "failed"
    payload.setdefault("signoff_thresholds", thresholds)
    payload["robotics_smoke_profile"] = args.profile
    payload["robotics_smoke_status"] = status
    payload["robotics_smoke_failure_reasons"] = failures
    payload["robotics_smoke_command"] = command
    payload["robotics_smoke_thresholds"] = thresholds
    payload["robotics_smoke"] = {
        "profile": args.profile,
        "status": status,
        "failure_reasons": failures,
        "command": command,
        "thresholds": thresholds,
    }
    path.write_text(json.dumps(payload, indent=2, sort_keys=True) + "\n", encoding="utf-8")
    return payload


def main() -> int:
    args = parse_args()
    root_dir = ROOT_DIR.resolve()
    effective_max_epochs = profile_value(args, "max_epochs")
    positioning_rate_min = profile_value(args, "positioning_rate_min")
    realtime_factor_min = profile_value(args, "realtime_factor_min")
    effective_epoch_rate_min = profile_value(args, "effective_epoch_rate_min")
    solver_wall_time_max = profile_value(args, "solver_wall_time_max")

    dataset_root = (
        resolve_path(args.dataset_root, root_dir)
        if args.dataset_root is not None
        else default_dataset_root(root_dir).resolve()
    )
    out_dir = resolve_path(args.out_dir, root_dir)
    out_dir.mkdir(parents=True, exist_ok=True)

    stem = f"{args.city}_{args.run}_rtk_{args.profile}"
    solution_path = out_dir / f"{stem}.pos"
    summary_path = out_dir / f"{stem}.json"

    thresholds: dict[str, float | int] = {}
    command = [
        *resolve_gnss_command(root_dir),
        "ppc-rtk-signoff",
        "--dataset-root",
        str(dataset_root),
        "--city",
        args.city,
        "--run",
        args.run,
        "--preset",
        args.preset,
        "--ratio",
        str(args.ratio),
        "--out",
        str(solution_path),
        "--summary-json",
        str(summary_path),
        "--require-valid-epochs-min",
        "1",
        "--require-matched-epochs-min",
        "1",
        "--require-fix-rate-min",
        "0",
        "--require-ppc-official-score-min",
        "0",
        "--require-median-h-max",
        "1000000",
        "--require-p95-h-max",
        "1000000",
        "--require-max-h-max",
        "1000000",
        "--require-p95-up-max",
        "1000000",
        "--require-mean-sats-min",
        "0",
    ]
    thresholds.update(
        {
            "require_valid_epochs_min": 1,
            "require_matched_epochs_min": 1,
            "require_fix_rate_min": 0,
            "require_ppc_official_score_min": 0,
            "require_median_h_max": 1000000,
            "require_p95_h_max": 1000000,
            "require_max_h_max": 1000000,
            "require_p95_up_max": 1000000,
            "require_mean_sats_min": 0,
        }
    )
    if effective_max_epochs is not None:
        command.extend(["--max-epochs", str(effective_max_epochs)])
    append_threshold(
        command,
        thresholds,
        "--require-positioning-rate-min",
        "require_positioning_rate_min",
        positioning_rate_min,
    )
    append_threshold(
        command,
        thresholds,
        "--require-solver-wall-time-max",
        "require_solver_wall_time_max",
        solver_wall_time_max,
    )
    append_threshold(
        command,
        thresholds,
        "--require-realtime-factor-min",
        "require_realtime_factor_min",
        realtime_factor_min,
    )
    append_threshold(
        command,
        thresholds,
        "--require-effective-epoch-rate-min",
        "require_effective_epoch_rate_min",
        effective_epoch_rate_min,
    )

    print("+", " ".join(command), flush=True)
    if args.dry_run:
        print(f"profile: {args.profile}")
        print(f"max_epochs: {effective_max_epochs if effective_max_epochs is not None else 'full'}")
        print("thresholds:")
        for key, value in sorted(thresholds.items()):
            print(f"  {key}: {value}")
        return 0

    if not dataset_root.exists():
        raise SystemExit(
            f"Dataset root not found: {dataset_root}\n"
            "Pass --dataset-root /path/to/PPC-Dataset or run gnss doctor."
        )

    completed = subprocess.run(command, cwd=root_dir, check=False)
    payload = annotate_summary(
        summary_path,
        args=args,
        command=command,
        thresholds=thresholds,
        completed_returncode=completed.returncode,
    )
    if completed.returncode != 0:
        if payload is not None:
            reasons = payload.get("robotics_smoke_failure_reasons")
            if isinstance(reasons, list) and reasons:
                print("")
                print("Robotics smoke failed:")
                for reason in reasons:
                    print(f"  - {reason}")
        return completed.returncode

    if payload is None:
        payload = load_summary(summary_path)
    print("")
    print(f"Robotics {args.profile} smoke passed")
    print(f"  solution: {solution_path}")
    print(f"  summary: {summary_path}")
    print(f"  positioning_rate_pct: {metric(payload, 'positioning_rate_pct')}")
    print(f"  fix_rate_pct: {metric(payload, 'fix_rate_pct')}")
    print(f"  solver_wall_time_s: {metric(payload, 'solver_wall_time_s')}")
    print(f"  realtime_factor: {metric(payload, 'realtime_factor')}")
    print(f"  effective_epoch_rate_hz: {metric(payload, 'effective_epoch_rate_hz')}")
    print("")
    print("Inspect artifacts with:")
    print("  python3 apps/gnss.py web --port 8085 --root .")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
