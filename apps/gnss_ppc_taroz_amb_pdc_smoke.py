#!/usr/bin/env python3
"""Run taroz ambiguity PDC FGO checks on PPC-Dataset runs."""

from __future__ import annotations

import argparse
import bisect
import csv
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

DEFAULT_DATASET_ROOT = Path(
    os.environ.get("GNSSPP_PPC_DATASET_ROOT", str(ROOT_DIR / "data/PPC-Dataset"))
)
DEFAULT_OUT_DIR = ROOT_DIR / "output/dogfood/ppc_taroz_amb_pdc_smoke"
REQUIRED_RUN_FILES = ("rover.obs", "base.obs", "base.nav", "reference.csv")


@dataclass(frozen=True)
class PpcRun:
    site: str
    name: str
    path: Path

    @property
    def label(self) -> str:
        return f"{self.site}_{self.name}"

    @property
    def spec(self) -> str:
        return f"{self.site}/{self.name}"


@dataclass(frozen=True)
class PosRecord:
    week: int
    tow: float
    x: float
    y: float
    z: float
    status: int


@dataclass(frozen=True)
class ReferenceRecord:
    week: int
    tow: float
    x: float
    y: float
    z: float


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


def resolve_dataset_root(root: Path) -> Path:
    candidates = [root, root / "PPC-Dataset"]
    for candidate in candidates:
        if (candidate / "nagoya").is_dir() or (candidate / "tokyo").is_dir():
            return candidate
    return root


def discover_runs(dataset_root: Path) -> list[PpcRun]:
    root = resolve_dataset_root(dataset_root)
    runs: list[PpcRun] = []
    for site_dir in sorted(root.iterdir() if root.exists() else []):
        if not site_dir.is_dir() or site_dir.name.startswith("."):
            continue
        for run_dir in sorted(site_dir.glob("run*")):
            if not run_dir.is_dir():
                continue
            if all((run_dir / name).is_file() for name in REQUIRED_RUN_FILES):
                runs.append(PpcRun(site_dir.name, run_dir.name, run_dir))
    return runs


def selected_runs(all_runs: list[PpcRun], specs: list[str]) -> list[PpcRun]:
    if not specs:
        return all_runs
    wanted: list[str] = []
    for raw_spec in specs:
        wanted.extend(token.strip() for token in raw_spec.split(",") if token.strip())
    by_spec = {run.spec: run for run in all_runs}
    by_label = {run.label: run for run in all_runs}
    selected: list[PpcRun] = []
    missing: list[str] = []
    for spec in wanted:
        run = by_spec.get(spec) or by_label.get(spec)
        if run is None:
            missing.append(spec)
        else:
            selected.append(run)
    if missing:
        available = ", ".join(run.spec for run in all_runs)
        raise SystemExit(
            f"unknown PPC run(s): {', '.join(missing)}. Available runs: {available}"
        )
    return selected


def output_paths(out_dir: Path, run: PpcRun) -> dict[str, Path]:
    run_dir = out_dir / run.label
    return {
        "run_dir": run_dir,
        "seed_pos": run_dir / "spp_seed.pos",
        "pos": run_dir / "fgo.pos",
        "summary": run_dir / "summary.json",
        "epoch_debug": run_dir / "epoch_debug.csv",
    }


def build_spp_seed_command(
    args: argparse.Namespace,
    run: PpcRun,
    paths: dict[str, Path],
) -> list[str]:
    spp_bin = Path(args.spp_bin) if args.spp_bin else find_binary("gnss_spp")
    if spp_bin is None:
        if args.dry_run:
            spp_bin = Path("gnss_spp")
        else:
            raise SystemExit(
                "gnss_spp binary not found. Build it first with "
                "`cmake --build build --target gnss_spp`, or pass --spp-bin."
            )

    command = [
        str(spp_bin),
        "--obs",
        str(run.path / "rover.obs"),
        "--nav",
        str(run.path / "base.nav"),
        "--out",
        str(paths["seed_pos"]),
        "--quiet",
    ]
    if args.max_epochs > 0:
        seed_epochs = args.skip_epochs + args.max_epochs
        command.extend(["--max-epochs", str(seed_epochs)])
    return command


def build_fgo_command(
    args: argparse.Namespace,
    run: PpcRun,
    paths: dict[str, Path],
) -> list[str]:
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
        str(run.path / "rover.obs"),
        "--base",
        str(run.path / "base.obs"),
        "--nav",
        str(run.path / "base.nav"),
        "--out",
        str(paths["pos"]),
        "--summary-json",
        str(paths["summary"]),
        "--epoch-debug-csv",
        str(paths["epoch_debug"]),
        "--quiet",
    ]
    if args.skip_epochs > 0:
        command.extend(["--skip-epochs", str(args.skip_epochs)])
    if args.max_epochs > 0:
        command.extend(["--max-epochs", str(args.max_epochs)])
    if args.generate_spp_seed:
        command.extend(["--seed-pos", str(paths["seed_pos"])])
    command.extend(args.fgo_extra_arg)
    return command


def run_command(command: list[str]) -> tuple[int, float]:
    import time

    start = time.monotonic()
    completed = subprocess.run(command, cwd=ROOT_DIR, check=False)
    return completed.returncode, time.monotonic() - start


def load_json(path: Path) -> dict[str, Any]:
    with path.open(encoding="utf-8") as handle:
        return json.load(handle)


def read_pos(path: Path) -> list[PosRecord]:
    rows: list[PosRecord] = []
    with path.open(encoding="ascii", errors="ignore") as handle:
        for line_number, raw_line in enumerate(handle, start=1):
            line = raw_line.strip()
            if not line or line.startswith("%"):
                continue
            parts = line.split()
            if len(parts) < 10:
                raise ValueError(f"{path}:{line_number}: expected at least 10 POS columns")
            rows.append(
                PosRecord(
                    week=int(float(parts[0])),
                    tow=float(parts[1]),
                    x=float(parts[2]),
                    y=float(parts[3]),
                    z=float(parts[4]),
                    status=int(float(parts[8])),
                )
            )
    return rows


def _row_value(row: dict[str, str], column: str) -> str:
    for key, value in row.items():
        if key.strip() == column:
            return value
    raise KeyError(column)


def read_reference(path: Path) -> dict[int, list[ReferenceRecord]]:
    by_week: dict[int, list[ReferenceRecord]] = {}
    with path.open(newline="", encoding="ascii", errors="ignore") as handle:
        for row in csv.DictReader(handle):
            week = int(float(_row_value(row, "GPS Week")))
            record = ReferenceRecord(
                week=week,
                tow=float(_row_value(row, "GPS TOW (s)")),
                x=float(_row_value(row, "ECEF X (m)")),
                y=float(_row_value(row, "ECEF Y (m)")),
                z=float(_row_value(row, "ECEF Z (m)")),
            )
            by_week.setdefault(week, []).append(record)
    for records in by_week.values():
        records.sort(key=lambda record: record.tow)
    return by_week


def percentile(values: list[float], fraction: float) -> float:
    ordered = sorted(values)
    return ordered[int(fraction * (len(ordered) - 1))]


def summarize_reference_errors(
    pos_path: Path,
    reference_csv: Path,
    tolerance_s: float,
) -> dict[str, Any]:
    pos_records = read_pos(pos_path)
    reference_by_week = read_reference(reference_csv)
    reference_tows = {
        week: [record.tow for record in records]
        for week, records in reference_by_week.items()
    }
    valid_errors: list[float] = []
    all_output_errors: list[float] = []
    fixed_errors: list[float] = []
    float_errors: list[float] = []
    no_solution_errors: list[float] = []
    status_counts: dict[str, int] = {}
    for pos in pos_records:
        status_counts[str(pos.status)] = status_counts.get(str(pos.status), 0) + 1
        references = reference_by_week.get(pos.week)
        tows = reference_tows.get(pos.week)
        if not references or not tows:
            continue
        index = bisect.bisect_left(tows, pos.tow)
        candidates = []
        if index < len(references):
            candidates.append(references[index])
        if index > 0:
            candidates.append(references[index - 1])
        if not candidates:
            continue
        reference = min(candidates, key=lambda record: abs(record.tow - pos.tow))
        if abs(reference.tow - pos.tow) > tolerance_s:
            continue
        dx = pos.x - reference.x
        dy = pos.y - reference.y
        dz = pos.z - reference.z
        error = math.sqrt(dx * dx + dy * dy + dz * dz)
        all_output_errors.append(error)
        if pos.status == 4:
            fixed_errors.append(error)
            valid_errors.append(error)
        elif pos.status == 3:
            float_errors.append(error)
            valid_errors.append(error)
        else:
            no_solution_errors.append(error)

    def stats(values: list[float]) -> dict[str, float | int | None]:
        if not values:
            return {
                "matched_epochs": 0,
                "mean_3d_error_m": None,
                "p95_3d_error_m": None,
                "max_3d_error_m": None,
            }
        return {
            "matched_epochs": len(values),
            "mean_3d_error_m": sum(values) / len(values),
            "p95_3d_error_m": percentile(values, 0.95),
            "max_3d_error_m": max(values),
        }

    result = stats(valid_errors)
    result["fixed"] = stats(fixed_errors)
    result["float"] = stats(float_errors)
    result["no_solution"] = stats(no_solution_errors)
    result["all_output"] = stats(all_output_errors)
    result["position_status_counts"] = status_counts
    return rounded(result)


def selected_native_summary(summary: dict[str, Any]) -> dict[str, Any]:
    keys = [
        "preset",
        "backend",
        "input_epochs",
        "optimized_epochs",
        "valid_solutions",
        "fixed_solutions",
        "float_solutions",
        "fix_rate_percent",
        "skip_epochs",
        "seed_pos",
        "seed_matched_epochs",
        "seed_interpolated_epochs",
        "use_spp_seed",
        "single_difference_doppler_factors",
        "single_difference_tdcp_factors",
        "double_difference_pseudorange_factors",
        "double_difference_carrier_factors",
        "lambda_ambiguity_attempts",
        "lambda_ambiguity_candidates",
        "lambda_ambiguity_used_candidates",
        "iterations",
        "converged",
        "final_cost",
        "total_processing_time_ms",
    ]
    return {key: summary.get(key) for key in keys if key in summary}


def validate_native_summary(args: argparse.Namespace, summary: dict[str, Any]) -> list[str]:
    failures: list[str] = []
    if summary.get("preset") != "taroz-amb-pdc":
        failures.append("preset is not taroz-amb-pdc")
    if summary.get("backend") != "eigen":
        failures.append("backend is not eigen")
    if args.max_epochs > 0 and summary.get("optimized_epochs") != args.max_epochs:
        failures.append(
            f"optimized_epochs {summary.get('optimized_epochs')} != {args.max_epochs}"
        )
    if int(summary.get("valid_solutions", 0)) < args.require_valid_min:
        failures.append(
            f"valid_solutions {summary.get('valid_solutions')} < {args.require_valid_min}"
        )
    if int(summary.get("double_difference_carrier_factors", 0)) < args.require_dd_carrier_min:
        failures.append(
            "double_difference_carrier_factors "
            f"{summary.get('double_difference_carrier_factors')} < "
            f"{args.require_dd_carrier_min}"
        )
    if int(summary.get("lambda_ambiguity_attempts", 0)) < args.require_lambda_attempts_min:
        failures.append(
            f"lambda_ambiguity_attempts {summary.get('lambda_ambiguity_attempts')} < "
            f"{args.require_lambda_attempts_min}"
        )
    fix_rate = float(summary.get("fix_rate_percent", 0.0))
    if fix_rate < args.require_fix_rate_min:
        failures.append(f"fix_rate_percent {fix_rate:.6f} < {args.require_fix_rate_min}")
    if not args.allow_not_converged and summary.get("converged") is not True:
        failures.append("optimization did not converge")
    return failures


def _check_metric_max(
    failures: list[str],
    label: str,
    value: Any,
    threshold: float,
) -> None:
    if threshold <= 0.0:
        return
    if value is None:
        failures.append(f"{label} is unavailable")
        return
    numeric = float(value)
    if numeric > threshold:
        failures.append(f"{label} {numeric:.6f} > {threshold}")


def validate_reference_summary(
    args: argparse.Namespace,
    summary: dict[str, Any],
) -> list[str]:
    failures: list[str] = []
    _check_metric_max(
        failures,
        "valid p95_3d_error_m",
        summary.get("p95_3d_error_m"),
        args.require_valid_p95_3d_max,
    )
    _check_metric_max(
        failures,
        "valid max_3d_error_m",
        summary.get("max_3d_error_m"),
        args.require_valid_max_3d_max,
    )
    fixed = summary.get("fixed", {})
    _check_metric_max(
        failures,
        "fixed p95_3d_error_m",
        fixed.get("p95_3d_error_m"),
        args.require_fixed_p95_3d_max,
    )
    return failures


def rounded(value: Any) -> Any:
    if isinstance(value, float):
        return round(value, 9) if math.isfinite(value) else value
    if isinstance(value, dict):
        return {key: rounded(item) for key, item in value.items()}
    if isinstance(value, list):
        return [rounded(item) for item in value]
    return value


def write_summary(path: Path, payload: dict[str, Any]) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(json.dumps(rounded(payload), indent=2, sort_keys=True) + "\n", encoding="utf-8")


def parse_args(argv: list[str]) -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description="Run taroz-amb-pdc FGO checks on PPC-Dataset runs."
    )
    parser.add_argument("--dataset-root", type=Path, default=DEFAULT_DATASET_ROOT)
    parser.add_argument(
        "--run",
        action="append",
        default=[],
        help="Run spec such as nagoya/run1 or tokyo_run2. Repeat or comma-separate.",
    )
    parser.add_argument("--out-dir", type=Path, default=DEFAULT_OUT_DIR)
    parser.add_argument(
        "--summary-json",
        type=Path,
        default=None,
        help="Aggregate summary path (default: <out-dir>/summary.json).",
    )
    parser.add_argument("--fgo-bin", type=Path, default=None)
    parser.add_argument("--spp-bin", type=Path, default=None)
    parser.add_argument("--generate-spp-seed", action="store_true")
    parser.add_argument("--skip-epochs", type=int, default=0)
    parser.add_argument("--max-epochs", type=int, default=20)
    parser.add_argument("--reference-match-tolerance-s", type=float, default=0.05)
    parser.add_argument("--require-valid-min", type=int, default=1)
    parser.add_argument("--require-dd-carrier-min", type=int, default=1)
    parser.add_argument("--require-lambda-attempts-min", type=int, default=1)
    parser.add_argument("--require-fix-rate-min", type=float, default=0.0)
    parser.add_argument("--require-valid-p95-3d-max", type=float, default=0.0)
    parser.add_argument("--require-valid-max-3d-max", type=float, default=0.0)
    parser.add_argument("--require-fixed-p95-3d-max", type=float, default=0.0)
    parser.add_argument("--allow-not-converged", action="store_true")
    parser.add_argument(
        "--fgo-extra-arg",
        action="append",
        default=[],
        help="Extra single argument passed to gnss_fgo. Repeat for multiple tokens.",
    )
    parser.add_argument("--dry-run", action="store_true")
    return parser.parse_args(argv)


def main(argv: list[str] | None = None) -> int:
    args = parse_args(sys.argv[1:] if argv is None else argv)
    if args.max_epochs < 0:
        raise SystemExit("--max-epochs must be non-negative")
    if args.skip_epochs < 0:
        raise SystemExit("--skip-epochs must be non-negative")
    if args.reference_match_tolerance_s < 0:
        raise SystemExit("--reference-match-tolerance-s must be non-negative")
    if args.require_valid_p95_3d_max < 0:
        raise SystemExit("--require-valid-p95-3d-max must be non-negative")
    if args.require_valid_max_3d_max < 0:
        raise SystemExit("--require-valid-max-3d-max must be non-negative")
    if args.require_fixed_p95_3d_max < 0:
        raise SystemExit("--require-fixed-p95-3d-max must be non-negative")

    dataset_root = resolve_dataset_root(args.dataset_root)
    runs = selected_runs(discover_runs(dataset_root), args.run)
    if not runs:
        raise SystemExit(f"no PPC runs found under {dataset_root}")

    summary_path = args.summary_json or (args.out_dir / "summary.json")
    payload: dict[str, Any] = {
        "status": "planned",
        "dataset_root": str(dataset_root),
        "out_dir": str(args.out_dir),
        "skip_epochs": args.skip_epochs,
        "max_epochs": args.max_epochs,
        "generate_spp_seed": args.generate_spp_seed,
        "runs": {},
    }

    planned: dict[str, list[str]] = {}
    planned_seed: dict[str, list[str]] = {}
    for run in runs:
        paths = output_paths(args.out_dir, run)
        if args.generate_spp_seed:
            planned_seed[run.spec] = build_spp_seed_command(args, run, paths)
        planned[run.spec] = build_fgo_command(args, run, paths)
    if planned_seed:
        payload["planned_seed_commands"] = planned_seed
    payload["planned_commands"] = planned

    if args.dry_run:
        payload["status"] = "dry-run"
        write_summary(summary_path, payload)
        print("Dry run. Planned PPC taroz ambiguity PDC smoke runs:")
        for spec, command in planned_seed.items():
            print(f"[{spec} seed] {' '.join(command)}")
        for spec, command in planned.items():
            print(f"[{spec}] {' '.join(command)}")
        print(f"Summary: {summary_path}")
        return 0

    all_failures: list[str] = []
    for run in runs:
        paths = output_paths(args.out_dir, run)
        paths["run_dir"].mkdir(parents=True, exist_ok=True)
        run_payload: dict[str, Any] = {
            "site": run.site,
            "run": run.name,
            "path": str(run.path),
            "outputs": {
                key: str(path)
                for key, path in paths.items()
                if key != "run_dir" and (key != "seed_pos" or args.generate_spp_seed)
            },
        }
        if args.generate_spp_seed:
            seed_command = planned_seed[run.spec]
            seed_returncode, seed_wall_time_s = run_command(seed_command)
            run_payload["seed_command"] = seed_command
            run_payload["seed_returncode"] = seed_returncode
            run_payload["seed_wall_time_s"] = seed_wall_time_s
            if seed_returncode != 0:
                failure = f"{run.spec}: gnss_spp returned {seed_returncode}"
                run_payload["status"] = "failed"
                run_payload["failures"] = [failure]
                all_failures.append(failure)
                payload["runs"][run.spec] = run_payload
                continue

        command = planned[run.spec]
        returncode, wall_time_s = run_command(command)
        run_payload["command"] = command
        run_payload["returncode"] = returncode
        run_payload["solver_wall_time_s"] = wall_time_s
        if returncode != 0:
            failure = f"{run.spec}: gnss_fgo returned {returncode}"
            run_payload["status"] = "failed"
            run_payload["failures"] = [failure]
            all_failures.append(failure)
            payload["runs"][run.spec] = run_payload
            continue

        native_summary = load_json(paths["summary"])
        failures = validate_native_summary(args, native_summary)
        run_payload["native_summary"] = selected_native_summary(native_summary)
        reference_summary = summarize_reference_errors(
            paths["pos"],
            run.path / "reference.csv",
            args.reference_match_tolerance_s,
        )
        failures.extend(validate_reference_summary(args, reference_summary))
        run_payload["reference_summary"] = reference_summary
        run_payload["failures"] = failures
        run_payload["status"] = "ok" if not failures else "failed"
        all_failures.extend(f"{run.spec}: {failure}" for failure in failures)
        payload["runs"][run.spec] = run_payload

    payload["failures"] = all_failures
    payload["status"] = "ok" if not all_failures else "failed"
    write_summary(summary_path, payload)
    if all_failures:
        for failure in all_failures:
            print(f"Error: {failure}", file=sys.stderr)
        return 1
    print(f"PPC taroz ambiguity PDC smoke OK: {len(runs)} run(s)")
    print(f"Summary: {summary_path}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
