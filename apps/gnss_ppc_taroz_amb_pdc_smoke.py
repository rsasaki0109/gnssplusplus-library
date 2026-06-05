#!/usr/bin/env python3
"""Run taroz ambiguity PDC FGO checks on PPC-Dataset runs."""

from __future__ import annotations

import argparse
import bisect
import csv
from dataclasses import dataclass
from datetime import datetime, timedelta
import glob
import json
import math
import os
from pathlib import Path
import shutil
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
ERROR_TAIL_THRESHOLDS_M = (0.5, 1.0, 2.0, 10.0, 100.0, 1000.0)
GPS_EPOCH = datetime(1980, 1, 6)
WGS84_A = 6378137.0
WGS84_F = 1.0 / 298.257223563
WGS84_E2 = WGS84_F * (2.0 - WGS84_F)


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


@dataclass(frozen=True)
class ErrorRecord:
    week: int
    tow: float
    status: int
    error_m: float


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
        "lambda_debug": run_dir / "lambda_debug.csv",
        "cost_trace": run_dir / "cost_trace.csv",
    }


def read_approximate_position_xyz(path: Path) -> tuple[float, float, float]:
    with path.open(encoding="ascii", errors="ignore") as handle:
        for raw_line in handle:
            if "APPROX POSITION XYZ" not in raw_line:
                continue
            parts = raw_line[:60].split()
            if len(parts) >= 3:
                return (float(parts[0]), float(parts[1]), float(parts[2]))
    raise ValueError(f"failed to read APPROX POSITION XYZ from {path}")


def ecef_to_geodetic_deg(
    x: float,
    y: float,
    z: float,
) -> tuple[float, float, float]:
    lon = math.atan2(y, x)
    p = math.hypot(x, y)
    lat = math.atan2(z, p * (1.0 - WGS84_E2))
    for _ in range(12):
        sin_lat = math.sin(lat)
        normal = WGS84_A / math.sqrt(1.0 - WGS84_E2 * sin_lat * sin_lat)
        height = p / math.cos(lat) - normal
        lat = math.atan2(z, p * (1.0 - WGS84_E2 * normal / (normal + height)))
    sin_lat = math.sin(lat)
    normal = WGS84_A / math.sqrt(1.0 - WGS84_E2 * sin_lat * sin_lat)
    height = p / math.cos(lat) - normal
    return (math.degrees(lat), math.degrees(lon), height)


def link_or_copy(source: Path, destination: Path) -> None:
    if destination.exists() or destination.is_symlink():
        destination.unlink()
    try:
        destination.symlink_to(source.resolve())
    except OSError:
        shutil.copyfile(source, destination)


def write_rtklib_seed_pos(input_seed: Path, output_seed: Path) -> int:
    status_to_rtklib_q = {4: 1, 3: 2, 2: 4, 1: 5, 0: 0}
    rows = 0
    with input_seed.open(encoding="ascii", errors="ignore") as src, output_seed.open(
        "w",
        encoding="ascii",
    ) as dst:
        dst.write("% generated from libgnss++ SPP seed ECEF for taroz MATLAB Gsol\n")
        dst.write(
            "%  GPST                  latitude(deg) longitude(deg)  height(m)"
            "   Q  ns   sdn(m)   sde(m)   sdu(m)  sdne(m)  sdeu(m)  sdun(m)"
            " age(s)  ratio    vn(m/s)    ve(m/s)    vu(m/s)"
            "      sdvn     sdve     sdvu    sdvne    sdveu    sdvun\n"
        )
        for raw_line in src:
            line = raw_line.strip()
            if not line or line.startswith("%"):
                continue
            parts = line.split()
            if len(parts) < 10:
                raise ValueError(f"unexpected seed POS row width in {input_seed}: {line}")
            week = int(float(parts[0]))
            tow = float(parts[1])
            x, y, z = (float(parts[2]), float(parts[3]), float(parts[4]))
            status = int(float(parts[8]))
            satellites = int(float(parts[9]))
            lat_deg, lon_deg, height_m = ecef_to_geodetic_deg(x, y, z)
            stamp = GPS_EPOCH + timedelta(weeks=week, seconds=tow)
            dst.write(
                f"{stamp:%Y/%m/%d} {stamp:%H:%M:%S}.{stamp.microsecond // 1000:03d}"
                f"   {lat_deg:17.12f} {lon_deg:17.12f} {height_m:14.7f}"
                f"   {status_to_rtklib_q.get(status, 5):1d} {satellites:3d}"
                "   2.0   2.0   5.0   0.0   0.0   0.0"
                "   0.00    0.0    0.0    0.0    0.0"
                "   0.0   0.0   0.0   0.0   0.0   0.0\n"
            )
            rows += 1
    return rows


def prepare_taroz_matlab_data_dir(run: PpcRun, seed_pos: Path, out_dir: Path) -> dict[str, Any]:
    out_dir.mkdir(parents=True, exist_ok=True)
    link_or_copy(run.path / "rover.obs", out_dir / "rover_1Hz.obs")
    link_or_copy(run.path / "base.obs", out_dir / "base.obs")
    link_or_copy(run.path / "base.nav", out_dir / "base.nav")
    link_or_copy(run.path / "reference.csv", out_dir / "reference.csv")

    x, y, z = read_approximate_position_xyz(run.path / "base.obs")
    lat_deg, lon_deg, height_m = ecef_to_geodetic_deg(x, y, z)
    (out_dir / "base_position.txt").write_text(
        f"{x:.4f} {y:.4f} {z:.4f}\n"
        f"{lat_deg:.8f} {lon_deg:.8f} {height_m:.3f}\n",
        encoding="ascii",
    )
    seed_rows = write_rtklib_seed_pos(seed_pos, out_dir / "rover_1Hz_spp.pos")
    return {
        "path": str(out_dir),
        "seed_rows": seed_rows,
        "base_position_ecef_m": [round(x, 4), round(y, 4), round(z, 4)],
        "base_position_llh_deg_m": [
            round(lat_deg, 8),
            round(lon_deg, 8),
            round(height_m, 3),
        ],
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
        "--lambda-debug-csv",
        str(paths["lambda_debug"]),
        "--cost-trace-csv",
        str(paths["cost_trace"]),
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


def pos_time_key(record: PosRecord) -> tuple[int, int]:
    return (record.week, round(record.tow * 1000))


def read_epoch_debug_time_keys(path: Path) -> list[tuple[int, int]]:
    with path.open(newline="", encoding="utf-8") as handle:
        return [
            (int(float(row["gps_week"])), round(float(row["gps_tow"]) * 1000))
            for row in csv.DictReader(handle)
        ]


def time_key_label(key: tuple[int, int]) -> str:
    week, tow_ms = key
    return f"{week}:{tow_ms / 1000.0:.3f}"


def duplicate_time_keys(keys: list[tuple[int, int]]) -> list[tuple[int, int]]:
    seen: set[tuple[int, int]] = set()
    duplicates: list[tuple[int, int]] = []
    for key in keys:
        if key in seen and key not in duplicates:
            duplicates.append(key)
        seen.add(key)
    return duplicates


def audit_generated_seed_artifacts(
    args: argparse.Namespace,
    seed_pos_path: Path,
    epoch_debug_path: Path,
) -> tuple[dict[str, Any], list[str]]:
    audit: dict[str, Any] = {
        "seed_pos": str(seed_pos_path),
        "epoch_debug": str(epoch_debug_path),
        "seed_rows": None,
        "optimized_epoch_rows": None,
        "expected_seed_rows": (
            args.skip_epochs + args.max_epochs if args.max_epochs > 0 else None
        ),
        "missing_optimized_seed_epochs": 0,
        "window_sequence_matches": None,
    }
    failures: list[str] = []
    if not args.generate_spp_seed:
        return audit, failures
    if not seed_pos_path.exists():
        return audit, [f"generated seed POS is missing: {seed_pos_path}"]
    if not epoch_debug_path.exists():
        return audit, [f"epoch debug CSV is missing: {epoch_debug_path}"]

    seed_keys = [pos_time_key(record) for record in read_pos(seed_pos_path)]
    epoch_keys = read_epoch_debug_time_keys(epoch_debug_path)
    audit["seed_rows"] = len(seed_keys)
    audit["optimized_epoch_rows"] = len(epoch_keys)

    expected_seed_rows = audit["expected_seed_rows"]
    if expected_seed_rows is not None and len(seed_keys) != expected_seed_rows:
        failures.append(
            f"generated seed rows {len(seed_keys)} != "
            f"skip_epochs + max_epochs {expected_seed_rows}"
        )

    duplicate_seed_keys = duplicate_time_keys(seed_keys)
    if duplicate_seed_keys:
        failures.append(
            "generated seed POS has duplicate epoch(s): "
            + ", ".join(time_key_label(key) for key in duplicate_seed_keys[:3])
        )
    duplicate_epoch_keys = duplicate_time_keys(epoch_keys)
    if duplicate_epoch_keys:
        failures.append(
            "epoch debug CSV has duplicate epoch(s): "
            + ", ".join(time_key_label(key) for key in duplicate_epoch_keys[:3])
        )
    if seed_keys != sorted(seed_keys):
        failures.append("generated seed POS epochs are not strictly sorted")
    if epoch_keys != sorted(epoch_keys):
        failures.append("epoch debug CSV epochs are not strictly sorted")

    seed_key_set = set(seed_keys)
    missing_epoch_keys = [key for key in epoch_keys if key not in seed_key_set]
    audit["missing_optimized_seed_epochs"] = len(missing_epoch_keys)
    if missing_epoch_keys:
        failures.append(
            f"generated seed POS is missing {len(missing_epoch_keys)} optimized "
            f"epoch(s), first missing {time_key_label(missing_epoch_keys[0])}"
        )

    if args.max_epochs > 0 and not duplicate_seed_keys and not duplicate_epoch_keys:
        start = args.skip_epochs
        stop = start + len(epoch_keys)
        seed_window = seed_keys[start:stop]
        audit["seed_window_start_index"] = start
        audit["seed_window_rows"] = len(seed_window)
        audit["window_sequence_matches"] = seed_window == epoch_keys
        if seed_window != epoch_keys:
            mismatch_index = next(
                (
                    index
                    for index, pair in enumerate(zip(seed_window, epoch_keys))
                    if pair[0] != pair[1]
                ),
                min(len(seed_window), len(epoch_keys)),
            )
            if mismatch_index < len(seed_window) and mismatch_index < len(epoch_keys):
                detail = (
                    f"seed {time_key_label(seed_window[mismatch_index])} != "
                    f"epoch {time_key_label(epoch_keys[mismatch_index])}"
                )
            else:
                detail = (
                    f"seed window rows {len(seed_window)} != "
                    f"optimized epoch rows {len(epoch_keys)}"
                )
            failures.append(
                "generated seed window does not match optimized epoch sequence "
                f"at skip_epochs {args.skip_epochs}: {detail}"
            )

    return audit, failures


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


def error_tail_key(threshold_m: float) -> str:
    return f"gt_{threshold_m:g}_m".replace(".", "_")


def error_tail_counts(records: list[ErrorRecord]) -> dict[str, int]:
    return {
        error_tail_key(threshold_m): sum(
            1 for record in records if record.error_m > threshold_m
        )
        for threshold_m in ERROR_TAIL_THRESHOLDS_M
    }


def worst_error_epoch(records: list[ErrorRecord]) -> dict[str, float | int] | None:
    if not records:
        return None
    worst = max(records, key=lambda record: record.error_m)
    return {
        "gps_week": worst.week,
        "gps_tow": worst.tow,
        "status": worst.status,
        "error_3d_m": worst.error_m,
    }


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
    valid_records: list[ErrorRecord] = []
    all_output_records: list[ErrorRecord] = []
    fixed_records: list[ErrorRecord] = []
    float_records: list[ErrorRecord] = []
    no_solution_records: list[ErrorRecord] = []
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
        error_record = ErrorRecord(pos.week, pos.tow, pos.status, error)
        all_output_errors.append(error)
        all_output_records.append(error_record)
        if pos.status == 4:
            fixed_errors.append(error)
            fixed_records.append(error_record)
            valid_errors.append(error)
            valid_records.append(error_record)
        elif pos.status == 3:
            float_errors.append(error)
            float_records.append(error_record)
            valid_errors.append(error)
            valid_records.append(error_record)
        else:
            no_solution_errors.append(error)
            no_solution_records.append(error_record)

    def stats(
        values: list[float],
        records: list[ErrorRecord],
    ) -> dict[str, float | int | dict[str, int] | dict[str, float | int] | None]:
        if not values:
            return {
                "matched_epochs": 0,
                "mean_3d_error_m": None,
                "p95_3d_error_m": None,
                "max_3d_error_m": None,
                "tail_counts_3d_error_m": error_tail_counts(records),
                "worst_epoch": None,
            }
        return {
            "matched_epochs": len(values),
            "mean_3d_error_m": sum(values) / len(values),
            "p95_3d_error_m": percentile(values, 0.95),
            "max_3d_error_m": max(values),
            "tail_counts_3d_error_m": error_tail_counts(records),
            "worst_epoch": worst_error_epoch(records),
        }

    result = stats(valid_errors, valid_records)
    result["fixed"] = stats(fixed_errors, fixed_records)
    result["float"] = stats(float_errors, float_records)
    result["no_solution"] = stats(no_solution_errors, no_solution_records)
    result["all_output"] = stats(all_output_errors, all_output_records)
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
        "max_float_seed_position_divergence_m",
        "max_float_position_jump_m",
        "float_rejected_seed_position_divergence",
        "float_rejected_position_jump",
        "single_difference_doppler_factors",
        "single_difference_tdcp_factors",
        "double_difference_pseudorange_factors",
        "double_difference_carrier_factors",
        "lambda_ambiguity_attempts",
        "lambda_ambiguity_candidates",
        "lambda_ambiguity_used_candidates",
        "iterations",
        "cost_trace_csv",
        "converged",
        "initial_cost",
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
    if args.generate_spp_seed:
        optimized_epochs = summary.get("optimized_epochs")
        seed_matched_epochs = summary.get("seed_matched_epochs")
        seed_interpolated_epochs = summary.get("seed_interpolated_epochs")
        if not summary.get("seed_pos"):
            failures.append("generated SPP seed path is missing from summary")
        if isinstance(optimized_epochs, int) and seed_matched_epochs != optimized_epochs:
            failures.append(
                f"seed_matched_epochs {seed_matched_epochs} != {optimized_epochs}"
            )
        if seed_interpolated_epochs != 0:
            failures.append(
                f"seed_interpolated_epochs {seed_interpolated_epochs} != 0"
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


def validate_taroz_matlab_data(
    args: argparse.Namespace,
    data_payload: dict[str, Any],
) -> list[str]:
    failures: list[str] = []
    if args.taroz_matlab_data_dir is None or args.max_epochs <= 0:
        return failures
    expected_seed_rows = args.skip_epochs + args.max_epochs
    seed_rows = data_payload.get("seed_rows")
    if seed_rows != expected_seed_rows:
        failures.append(
            f"taroz MATLAB seed_rows {seed_rows} != "
            f"skip_epochs + max_epochs {expected_seed_rows}"
        )
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
    parser.add_argument(
        "--taroz-matlab-data-dir",
        type=Path,
        default=None,
        help=(
            "Optional output directory for taroz MATLAB-compatible PPC data. "
            "Requires --generate-spp-seed."
        ),
    )
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
    if args.taroz_matlab_data_dir is not None and not args.generate_spp_seed:
        raise SystemExit("--taroz-matlab-data-dir requires --generate-spp-seed")

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
        "taroz_matlab_data_dir": (
            str(args.taroz_matlab_data_dir)
            if args.taroz_matlab_data_dir is not None
            else None
        ),
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
        if args.taroz_matlab_data_dir is not None:
            print(f"Taroz MATLAB data root: {args.taroz_matlab_data_dir}")
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
            if args.taroz_matlab_data_dir is not None:
                data_dir = (
                    args.taroz_matlab_data_dir
                    if len(runs) == 1
                    else args.taroz_matlab_data_dir / run.label
                )
                run_payload["taroz_matlab_data"] = prepare_taroz_matlab_data_dir(
                    run,
                    paths["seed_pos"],
                    data_dir,
                )
                run_payload["taroz_matlab_data_failures"] = (
                    validate_taroz_matlab_data(
                        args,
                        run_payload["taroz_matlab_data"],
                    )
                )

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
        if args.generate_spp_seed:
            seed_audit, seed_audit_failures = audit_generated_seed_artifacts(
                args,
                paths["seed_pos"],
                paths["epoch_debug"],
            )
            run_payload["generated_seed_audit"] = seed_audit
            failures.extend(seed_audit_failures)
        failures.extend(run_payload.get("taroz_matlab_data_failures", []))
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
