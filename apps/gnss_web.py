#!/usr/bin/env python3
"""Local web UI for browsing libgnss++ benchmark artifacts and receiver status."""

from __future__ import annotations

import argparse
import csv
import json
import math
import os
import re
from http import HTTPStatus
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from pathlib import Path
import signal
import sys
import threading
from typing import Any
from urllib.parse import parse_qs, urlparse

from gnss_toml_config import parse_args_with_toml

ROOT_DIR = Path(__file__).resolve().parent.parent
SCRIPTS_DIR = ROOT_DIR / "scripts"

if str(SCRIPTS_DIR) not in sys.path:
    sys.path.insert(0, str(SCRIPTS_DIR))

import generate_driving_comparison as driving_comparison  # noqa: E402


STATUS_COLORS = {
    "FIXED": "#2ecc71",
    "FLOAT": "#f39c12",
    "DGPS": "#3498db",
    "SPP": "#e74c3c",
}

MAX_RENDER_POINTS = 2000
DOCS_SITE_URL = "https://rsasaki0109.github.io/gnssplusplus-library/"
GNSS_GPU_SCORE_COLUMNS = (
    "segment_ppc_pct",
    "honest_ppc_pct",
    "aggregate_ppc_score_pct",
    "ppc_score_pct",
    "score_pct",
    "ppc_pct",
    "rtk_subset_ppc_pct",
    "rtk_honest_ppc_pct",
    "hybrid_subset_ppc_pct",
    "hybrid_ppc_pct",
    "hybrid_honest_ppc_pct",
    "pass_epoch_pct",
)
GNSS_GPU_PASS_COLUMNS = (
    "segment_pass_m",
    "honest_pass_m",
    "ppc_pass_distance_m",
    "pass_m",
    "rtk_subset_pass_m",
    "rtk_honest_pass_m",
    "hybrid_pass_m",
    "hybrid_honest_pass_m",
)
GNSS_GPU_TOTAL_COLUMNS = (
    "segment_total_m",
    "honest_total_m",
    "ppc_total_distance_m",
    "total_m",
    "rtk_subset_total_m",
    "hybrid_total_m",
    "true_arc_length_m",
)
GNSS_GPU_P50_COLUMNS = (
    "p50",
    "median_2d_m",
    "hybrid_2d_median_m",
    "median_error_3d_m",
    "forward_p50_m",
    "smoothed_p50_m",
)
GNSS_GPU_P95_COLUMNS = ("p95", "p95_2d_m", "hybrid_2d_p95_m", "p95_error_3d_m", "mean_p95")
GNSS_GPU_RMS_COLUMNS = ("rms_2d", "rms_2d_m", "mean_rms_2d", "forward_rms_2d_m", "smoothed_rms_2d_m")
GNSS_GPU_MAX_COLUMNS = ("max_2d", "max_2d_m", "max_error_3d_m")
GNSS_GPU_EPOCH_COLUMNS = ("n_epochs", "valid_epochs", "ppc_n_epochs", "count")
GNSS_GPU_TIME_COLUMNS = ("time_ms_per_epoch", "time_ms", "wls_ms")


def default_root_dir() -> Path:
    source_root = Path(__file__).resolve().parent.parent
    if (source_root / "output").exists():
        return source_root
    return Path.cwd()


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        prog=os.environ.get("GNSS_CLI_NAME"),
        description="Serve a local web UI for benchmark artifacts, trajectories, and receiver status.",
    )
    parser.add_argument(
        "--config-toml",
        type=Path,
        default=None,
        help="Optional TOML config. Uses [web] or top-level keys.",
    )
    parser.add_argument("--host", default="127.0.0.1", help="Bind host (default: 127.0.0.1).")
    parser.add_argument("--port", type=int, default=8085, help="Bind port (default: 8085, use 0 for auto).")
    parser.add_argument(
        "--port-file",
        type=Path,
        default=None,
        help="Optional file that receives the bound port after startup.",
    )
    parser.add_argument(
        "--root",
        type=Path,
        default=default_root_dir(),
        help="Artifact root directory (default: source tree root or current working directory).",
    )
    parser.add_argument(
        "--lib-pos",
        type=Path,
        default=None,
        help="libgnss++ .pos path (default: <root>/output/rtk_solution.pos).",
    )
    parser.add_argument(
        "--rtklib-pos",
        type=Path,
        default=None,
        help="RTKLIB .pos path (default: <root>/output/driving_rtklib_rtk.pos).",
    )
    parser.add_argument(
        "--odaiba-summary",
        type=Path,
        default=None,
        help="Odaiba summary JSON path (default: <root>/output/odaiba_summary.json).",
    )
    parser.add_argument(
        "--rcv-status",
        type=Path,
        default=None,
        help="Receiver status JSON path exported by gnss rcv.",
    )
    parser.add_argument(
        "--ppc-summary-glob",
        default="output/ppc_*_summary.json",
        help="Glob under --root for PPC summary JSON files.",
    )
    parser.add_argument(
        "--live-summary-glob",
        default="output/live*_summary.json",
        help="Glob under --root for gnss live-signoff summary JSON files.",
    )
    parser.add_argument(
        "--visibility-summary-glob",
        default="output/visibility*_summary.json",
        help="Glob under --root for gnss visibility summary JSON files.",
    )
    parser.add_argument(
        "--moving-base-summary-glob",
        default="output/*moving_base_summary.json",
        help="Glob under --root for gnss moving-base signoff summary JSON files.",
    )
    parser.add_argument(
        "--ppp-products-summary-glob",
        default="output/*ppp*_products*_summary.json",
        help="Glob under --root for gnss PPP product signoff summary JSON files.",
    )
    parser.add_argument(
        "--dd-residual-summary-glob",
        default="output/*dd_residuals*_summary.json",
        help="Glob under --root for gnss dd-residuals summary JSON files.",
    )
    parser.add_argument(
        "--gnss-gpu-summary-glob",
        default="experiments/results/*_summary.csv",
        help="Glob under --root for gnss_gpu experiment summary CSV files.",
    )
    parser.add_argument(
        "--gnss-gpu-runs-glob",
        default="experiments/results/*_runs.csv",
        help="Glob under --root for gnss_gpu/libgnss++ per-run CSV files.",
    )
    parser.add_argument(
        "--artifact-manifest",
        type=Path,
        default=None,
        help="Artifact manifest JSON path (default: <root>/output/artifact_manifest.json).",
    )
    parser.add_argument(
        "--docs-url",
        default=os.environ.get("GNSSPP_DOCS_URL", DOCS_SITE_URL),
        help="Optional docs site URL shown in the web UI header.",
    )
    return parse_args_with_toml(parser, "web")


def resolve_path(explicit: Path | None, root_dir: Path, relative: str) -> Path:
    if explicit is not None:
        return explicit
    return root_dir / relative


def glob_artifact_paths(root_dir: Path, pattern: str) -> list[Path]:
    paths = set(root_dir.glob(pattern))
    nested_root = root_dir / "third_party/gnssplusplus"
    if nested_root.exists():
        paths.update(nested_root.glob(pattern))
    return sorted(paths)


def resolve_first_existing(explicit: Path | None, root_dir: Path, relatives: list[str]) -> Path:
    if explicit is not None:
        return explicit
    for relative in relatives:
        candidate = root_dir / relative
        if candidate.exists():
            return candidate
    return root_dir / relatives[0]


def relative_display(path: Path, root_dir: Path) -> str:
    try:
        return str(path.resolve().relative_to(root_dir.resolve()))
    except ValueError:
        return str(path)


def load_json(path: Path) -> dict[str, Any] | None:
    if not path.exists():
        return None
    try:
        return json.loads(path.read_text(encoding="utf-8"))
    except (OSError, json.JSONDecodeError):
        return None


def normalize_artifact_path(root_dir: Path, value: Any) -> str | None:
    if not isinstance(value, str) or not value:
        return None
    if value.startswith(("http://", "https://")):
        return value
    try:
        return relative_display(resolve_under_root(root_dir, value), root_dir)
    except ValueError:
        return value
    try:
        return json.loads(path.read_text(encoding="utf-8"))
    except (OSError, json.JSONDecodeError):
        return None


def inferred_sibling_artifact(root_dir: Path, summary_path: Path, suffix: str) -> str | None:
    name = summary_path.name
    if name.endswith("_summary.json"):
        candidate = summary_path.with_name(name[: -len("_summary.json")] + suffix)
    else:
        candidate = summary_path.with_suffix(suffix)
    if candidate.exists():
        return relative_display(candidate, root_dir)
    return None


def normalize_commercial_receiver(root_dir: Path, payload: Any) -> dict[str, Any] | None:
    if not isinstance(payload, dict):
        return None
    normalized = dict(payload)
    for key in ("solution_pos", "matched_csv"):
        normalized[key] = normalize_artifact_path(root_dir, normalized.get(key))
    return normalized


def resolve_under_root(root_dir: Path, path_text: str) -> Path:
    candidate = Path(path_text)
    if not candidate.is_absolute():
        candidate = (root_dir / candidate).resolve()
    else:
        candidate = candidate.resolve()
    candidate.relative_to(root_dir.resolve())
    return candidate


def load_visibility_rows(csv_path: Path) -> list[dict[str, Any]]:
    with csv_path.open("r", encoding="utf-8", newline="") as stream:
        reader = csv.DictReader(stream)
        rows: list[dict[str, Any]] = []
        for row in reader:
            rows.append(
                {
                    "satellite": row.get("satellite"),
                    "system": row.get("system"),
                    "signal": row.get("signal"),
                    "azimuth_deg": float(row["azimuth_deg"]),
                    "elevation_deg": float(row["elevation_deg"]),
                    "snr_dbhz": float(row["snr_dbhz"]) if row.get("snr_dbhz") else None,
                }
            )
        return downsample_points(rows)


def load_moving_base_matches(csv_path: Path) -> list[dict[str, Any]]:
    with csv_path.open("r", encoding="utf-8", newline="") as stream:
        reader = csv.DictReader(stream)
        rows: list[dict[str, Any]] = []
        for row in reader:
            heading_text = row.get("heading_error_deg", "")
            rows.append(
                {
                    "gps_week": int(row["gps_week"]),
                    "gps_tow_s": float(row["gps_tow_s"]),
                    "baseline_error_m": float(row["baseline_error_m"]),
                    "baseline_length_m": float(row["baseline_length_m"]),
                    "heading_error_deg": float(heading_text) if heading_text else None,
                    "status": int(row["status"]),
                    "satellites": int(row["satellites"]),
                }
            )
        return downsample_points(rows)


def classify_realtime_status(realtime_factor: Any) -> str:
    if not isinstance(realtime_factor, (int, float)):
        return "n/a"
    if realtime_factor >= 1.0:
        return "realtime"
    if realtime_factor >= 0.5:
        return "near-realtime"
    return "offline"


def classify_accuracy_status(p95_h_m: Any) -> str:
    if not isinstance(p95_h_m, (int, float)):
        return "n/a"
    if p95_h_m <= 0.5:
        return "excellent"
    if p95_h_m <= 2.0:
        return "good"
    if p95_h_m <= 10.0:
        return "rough"
    return "poor"


def classify_baseline_status(p95_baseline_error_m: Any) -> str:
    if not isinstance(p95_baseline_error_m, (int, float)):
        return "n/a"
    if p95_baseline_error_m <= 0.2:
        return "excellent"
    if p95_baseline_error_m <= 0.5:
        return "good"
    if p95_baseline_error_m <= 2.0:
        return "rough"
    return "poor"


def classify_ppp_products_status(
    converged: Any,
    p95_position_error_m: Any,
    solution_rate_pct: Any,
) -> str:
    if converged is True and isinstance(p95_position_error_m, (int, float)):
        return classify_accuracy_status(p95_position_error_m)
    if converged is True:
        return "converged"
    if isinstance(solution_rate_pct, (int, float)) and solution_rate_pct >= 95.0:
        return "tracking"
    return "warming"


def classify_dd_residual_status(phase_sigma: Any, code_sigma: Any) -> str:
    values = [float(value) for value in (phase_sigma, code_sigma) if isinstance(value, (int, float))]
    if not values:
        return "n/a"
    worst = max(values)
    if worst <= 1.0:
        return "excellent"
    if worst <= 2.0:
        return "good"
    if worst <= 4.0:
        return "rough"
    return "poor"


def classify_comparison_status(*deltas: Any) -> str | None:
    numeric = [float(value) for value in deltas if isinstance(value, (int, float))]
    if not numeric:
        return None
    worst = max(numeric)
    if worst <= 0.0:
        return "better"
    if worst <= 0.25:
        return "close"
    return "worse"


def downsample_points(points: list[dict[str, Any]], limit: int = MAX_RENDER_POINTS) -> list[dict[str, Any]]:
    if len(points) <= limit:
        return points
    stride = max(1, len(points) // limit)
    sampled = points[::stride]
    if sampled[-1] != points[-1]:
        sampled.append(points[-1])
    return sampled


def rounded(value: float | None, digits: int = 6) -> float | None:
    if value is None or not isinstance(value, (int, float)) or not math.isfinite(float(value)):
        return None
    return round(float(value), digits)


def parse_optional_float(value: Any) -> float | None:
    if value is None:
        return None
    if isinstance(value, (int, float)):
        candidate = float(value)
    else:
        text = str(value).strip()
        if not text:
            return None
        try:
            candidate = float(text)
        except ValueError:
            return None
    if not math.isfinite(candidate):
        return None
    return candidate


def first_float(row: dict[str, Any], columns: tuple[str, ...]) -> float | None:
    value, _column = first_float_with_column(row, columns)
    return value


def first_float_with_column(row: dict[str, Any], columns: tuple[str, ...]) -> tuple[float | None, str | None]:
    for column in columns:
        value = parse_optional_float(row.get(column))
        if value is not None:
            return value, column
    return None, None


def first_text(row: dict[str, Any], columns: tuple[str, ...]) -> str | None:
    for column in columns:
        value = row.get(column)
        if value is None:
            continue
        text = str(value).strip()
        if text:
            return text
    return None


def mean_or_none(values: list[float]) -> float | None:
    return sum(values) / len(values) if values else None


def percentile_or_none(values: list[float], q: float) -> float | None:
    if not values:
        return None
    sorted_values = sorted(values)
    if len(sorted_values) == 1:
        return sorted_values[0]
    rank = q * (len(sorted_values) - 1)
    lower = math.floor(rank)
    upper = math.ceil(rank)
    if lower == upper:
        return sorted_values[lower]
    fraction = rank - lower
    return sorted_values[lower] * (1.0 - fraction) + sorted_values[upper] * fraction


def resolve_rtklib_pos_path(explicit: Path | None, root_dir: Path) -> Path:
    return resolve_first_existing(
        explicit,
        root_dir,
        [
            "output/driving_rtklib_rtk.pos",
            "output/rtklib_rtk_result.pos",
            "third_party/gnssplusplus/output/driving_rtklib_rtk.pos",
            "third_party/gnssplusplus/output/rtklib_rtk_result.pos",
        ],
    )


def resolve_libgnss_pos_path(explicit: Path | None, root_dir: Path) -> Path:
    return resolve_first_existing(
        explicit,
        root_dir,
        [
            "output/rtk_solution.pos",
            "third_party/gnssplusplus/output/rtk_solution.pos",
        ],
    )


def classify_gnss_gpu_status(score_pct: Any, p95_m: Any, rms_m: Any) -> str:
    score = parse_optional_float(score_pct)
    p95 = parse_optional_float(p95_m)
    rms = parse_optional_float(rms_m)
    if score is not None:
        if score >= 80.0:
            return "excellent"
        if score >= 60.0:
            return "good"
        if score >= 40.0:
            return "rough"
        return "poor"
    error = p95 if p95 is not None else rms
    if error is None:
        return "n/a"
    if error <= 1.0:
        return "excellent"
    if error <= 5.0:
        return "good"
    if error <= 20.0:
        return "rough"
    return "poor"


def gnss_gpu_family(path: Path, row_label: str | None) -> str:
    text = f"{path.name} {row_label or ''}".lower()
    if "rtklib" in text:
        return "RTKLIB"
    if "libgnss" in text or "gnss++" in text:
        return "libgnss++"
    return "gnss_gpu"


def gnss_gpu_row_label(path: Path, row: dict[str, Any], source_kind: str) -> str:
    suffix = f"_{source_kind}.csv"
    stem = path.name.removesuffix(suffix)
    detail = first_text(row, ("method", "mode", "source", "policy", "run", "config", "strategy", "metric"))
    city = first_text(row, ("city",))
    run = first_text(row, ("run",))
    start_epoch = first_text(row, ("start_epoch",))
    profile = first_text(row, ("profile", "profile_mode"))
    prefix_parts = [value for value in (city, run) if value]
    if start_epoch:
        prefix_parts.append(f"seg{start_epoch}")
    if profile and profile not in prefix_parts:
        prefix_parts.append(profile)
    if prefix_parts:
        prefix = "/".join(prefix_parts)
        if detail and detail != stem and detail not in prefix_parts:
            return f"{stem}:{prefix}:{detail}"
        return f"{stem}:{prefix}"
    if detail and detail != stem:
        return f"{stem}:{detail}"
    return stem


def gnss_gpu_experiment_name(path: Path, source_kind: str) -> str:
    suffix = f"_{source_kind}.csv"
    return path.name.removesuffix(suffix)


def gnss_gpu_dataset_key(path: Path, label: str, row: dict[str, Any]) -> str | None:
    city = first_text(row, ("city",))
    run = first_text(row, ("run",))
    start_epoch = first_text(row, ("start_epoch",))
    segment = first_text(row, ("segment",))
    if city and run:
        normalized_run = run if run.lower().startswith("run") else f"run{run}"
        key = f"{city.lower()}_{normalized_run.lower()}"
        segment_epoch = parse_optional_float(start_epoch)
        if segment_epoch is None and segment:
            match = re.search(r"@(\d+)", segment)
            if match:
                segment_epoch = float(match.group(1))
        if segment_epoch is not None:
            key += f"_seg{int(segment_epoch)}"
        return key

    text = f"{path.stem} {label}".lower()
    city_match = re.search(r"\b(tokyo|nagoya)\b", text)
    run_match = re.search(r"\brun[_-]?(\d+)\b", text)
    seg_match = re.search(r"\b(?:seg|segment|move)[_-]?(\d+)\b", text)
    if seg_match is None and run_match is not None:
        after_run = text[run_match.end() :]
        seg_match = re.search(r"\b(\d{2,5})\b", after_run)
    if not city_match or not run_match:
        return None
    key = f"{city_match.group(1)}_run{int(run_match.group(1))}"
    if seg_match:
        key += f"_seg{int(seg_match.group(1))}"
    return key


def gnss_gpu_score_scope_key(total_m: float | None) -> str | None:
    if total_m is None:
        return None
    return f"distance:{total_m:.1f}m"


def gnss_gpu_score_scope_label(score_column: str | None) -> str | None:
    if score_column is None:
        return None
    if score_column.startswith("segment_") or score_column in {"rtk_subset_ppc_pct", "hybrid_subset_ppc_pct"}:
        return "segment"
    if "honest" in score_column:
        return "honest_full_run"
    if score_column.startswith("aggregate_"):
        return "aggregate"
    if score_column == "pass_epoch_pct":
        return "epoch_pass_rate"
    return "reported"


def gnss_gpu_score_snapshot(row: dict[str, Any]) -> dict[str, float]:
    snapshot = {}
    for column in GNSS_GPU_SCORE_COLUMNS:
        value = parse_optional_float(row.get(column))
        if value is not None:
            snapshot[column] = rounded(value, 6)
    return snapshot


def gnss_gpu_objective(row: dict[str, Any]) -> tuple[int, float]:
    score = parse_optional_float(row.get("score_pct"))
    if score is not None:
        return (3, score)
    p95 = parse_optional_float(row.get("p95_m"))
    if p95 is not None:
        return (2, -p95)
    rms = parse_optional_float(row.get("rms_2d_m"))
    if rms is not None:
        return (1, -rms)
    p50 = parse_optional_float(row.get("p50_m"))
    if p50 is not None:
        return (0, -p50)
    return (-1, 0.0)


def compact_gnss_gpu_run(run: str | None) -> str | None:
    if run is None:
        return None
    run_text = str(run).strip()
    if not run_text:
        return None
    return run_text if run_text.lower().startswith("run") else f"run{run_text}"


def normalize_gnss_gpu_summary_row(
    path: Path,
    root_dir: Path,
    row: dict[str, Any],
    source_kind: str = "summary",
) -> dict[str, Any] | None:
    score_pct, score_column = first_float_with_column(row, GNSS_GPU_SCORE_COLUMNS)
    pass_m = first_float(row, GNSS_GPU_PASS_COLUMNS)
    total_m = first_float(row, GNSS_GPU_TOTAL_COLUMNS)
    p50_m = first_float(row, GNSS_GPU_P50_COLUMNS)
    p95_m = first_float(row, GNSS_GPU_P95_COLUMNS)
    rms_2d_m = first_float(row, GNSS_GPU_RMS_COLUMNS)
    max_2d_m = first_float(row, GNSS_GPU_MAX_COLUMNS)
    n_epochs = first_float(row, GNSS_GPU_EPOCH_COLUMNS)
    time_ms = first_float(row, GNSS_GPU_TIME_COLUMNS)
    if all(value is None for value in (score_pct, pass_m, total_m, p50_m, p95_m, rms_2d_m, max_2d_m)):
        return None
    label = gnss_gpu_row_label(path, row, source_kind)
    experiment = gnss_gpu_experiment_name(path, source_kind)
    city = first_text(row, ("city",))
    run = compact_gnss_gpu_run(first_text(row, ("run",)))
    start_epoch = first_text(row, ("start_epoch",))
    segment = first_text(row, ("segment",))
    method = first_text(row, ("method", "mode", "config", "strategy", "policy", "metric"))
    profile = first_text(row, ("profile", "profile_mode"))
    dataset_key = gnss_gpu_dataset_key(path, label, row)
    score_scope_key = gnss_gpu_score_scope_key(total_m)
    normalized = {
        "_path": relative_display(path, root_dir),
        "source_kind": source_kind,
        "experiment": experiment,
        "label": label,
        "family": gnss_gpu_family(path, label),
        "city": city,
        "run": run,
        "start_epoch": start_epoch,
        "segment": segment,
        "method": method,
        "profile": profile,
        "dataset_key": dataset_key,
        "score_scope_key": score_scope_key,
        "comparison_keys": [
            {"type": key_type, "key": key}
            for key_type, key in (("dataset", dataset_key), ("score_scope", score_scope_key))
            if key is not None
        ],
        "score_pct": rounded(score_pct, 6),
        "score_column": score_column,
        "score_scope": gnss_gpu_score_scope_label(score_column),
        "raw_scores": gnss_gpu_score_snapshot(row),
        "pass_m": rounded(pass_m, 6),
        "total_m": rounded(total_m, 6),
        "p50_m": rounded(p50_m, 6),
        "p95_m": rounded(p95_m, 6),
        "rms_2d_m": rounded(rms_2d_m, 6),
        "max_2d_m": rounded(max_2d_m, 6),
        "n_epochs": int(n_epochs) if n_epochs is not None else None,
        "time_ms": rounded(time_ms, 6),
        "quality_status": classify_gnss_gpu_status(score_pct, p95_m, rms_2d_m),
    }
    normalized["objective_rank"], normalized["objective_value"] = gnss_gpu_objective(normalized)
    return normalized


def gnss_gpu_comparison_delta(
    gpu_row: dict[str, Any],
    lib_row: dict[str, Any],
) -> dict[str, Any]:
    score_delta = None
    if gpu_row.get("score_pct") is not None and lib_row.get("score_pct") is not None:
        score_delta = rounded(float(gpu_row["score_pct"]) - float(lib_row["score_pct"]), 6)
    p95_delta = None
    if gpu_row.get("p95_m") is not None and lib_row.get("p95_m") is not None:
        p95_delta = rounded(float(gpu_row["p95_m"]) - float(lib_row["p95_m"]), 6)
    rms_delta = None
    if gpu_row.get("rms_2d_m") is not None and lib_row.get("rms_2d_m") is not None:
        rms_delta = rounded(float(gpu_row["rms_2d_m"]) - float(lib_row["rms_2d_m"]), 6)
    if score_delta is not None:
        status = "better" if score_delta >= 0.0 else "close" if score_delta >= -0.5 else "worse"
    elif p95_delta is not None:
        status = "better" if p95_delta <= 0.0 else "close" if p95_delta <= 0.25 else "worse"
    elif rms_delta is not None:
        status = "better" if rms_delta <= 0.0 else "close" if rms_delta <= 0.25 else "worse"
    else:
        status = "n/a"
    return {
        "score_delta_pct": score_delta,
        "p95_delta_m": p95_delta,
        "rms_delta_m": rms_delta,
        "status": status,
    }


def build_matched_gnss_gpu_comparisons(rows: list[dict[str, Any]]) -> list[dict[str, Any]]:
    grouped: dict[tuple[str, str], dict[str, list[dict[str, Any]]]] = {}
    for row in rows:
        keys = row.get("comparison_keys")
        if not isinstance(keys, list):
            continue
        family = str(row.get("family", "unknown"))
        if family not in ("gnss_gpu", "libgnss++"):
            continue
        seen_row_keys: set[tuple[str, str]] = set()
        for item in keys:
            if not isinstance(item, dict):
                continue
            key_type = item.get("type")
            key = item.get("key")
            if not isinstance(key_type, str) or not isinstance(key, str):
                continue
            row_key = (key_type, key)
            if row_key in seen_row_keys:
                continue
            seen_row_keys.add(row_key)
            grouped.setdefault(row_key, {}).setdefault(family, []).append(row)

    comparisons: list[dict[str, Any]] = []
    for (key_type, key), families in grouped.items():
        gpu_rows = families.get("gnss_gpu", [])
        lib_rows = families.get("libgnss++", [])
        if not gpu_rows or not lib_rows:
            continue
        gpu_best = max(gpu_rows, key=gnss_gpu_objective)
        lib_best = max(lib_rows, key=gnss_gpu_objective)
        comparison = {
            "key_type": key_type,
            "key": key,
            "gnss_gpu": gpu_best,
            "libgnsspp": lib_best,
            "candidate_counts": {
                "gnss_gpu": len(gpu_rows),
                "libgnss++": len(lib_rows),
            },
        }
        comparison.update(gnss_gpu_comparison_delta(gpu_best, lib_best))
        comparisons.append(comparison)

    key_type_rank = {"dataset": 0, "score_scope": 1}
    comparisons.sort(
        key=lambda row: (
            key_type_rank.get(str(row.get("key_type")), 99),
            -(float(row["score_delta_pct"]) if row.get("score_delta_pct") is not None else -9999.0),
            str(row.get("key", "")),
        )
    )
    return comparisons


def summarize_matched_gnss_gpu_comparisons(
    comparisons: list[dict[str, Any]],
) -> dict[str, Any]:
    status_counts: dict[str, int] = {}
    key_type_counts: dict[str, int] = {}
    source_counts: dict[str, int] = {}
    score_rows: list[dict[str, Any]] = []
    for row in comparisons:
        status = str(row.get("status", "n/a"))
        key_type = str(row.get("key_type", "unknown"))
        status_counts[status] = status_counts.get(status, 0) + 1
        key_type_counts[key_type] = key_type_counts.get(key_type, 0) + 1
        for family_key in ("gnss_gpu", "libgnsspp"):
            family_row = row.get(family_key)
            if isinstance(family_row, dict):
                source = str(family_row.get("source_kind", "unknown"))
                source_counts[source] = source_counts.get(source, 0) + 1
        if row.get("score_delta_pct") is not None:
            score_rows.append(row)

    score_rows.sort(key=lambda row: float(row["score_delta_pct"]))
    dataset_rows = [row for row in comparisons if row.get("key_type") == "dataset"]
    dataset_score_rows = [row for row in dataset_rows if row.get("score_delta_pct") is not None]
    worst_loss = score_rows[0] if score_rows else None
    best_win = score_rows[-1] if score_rows else None
    dataset_worst_loss = min(dataset_score_rows, key=lambda row: float(row["score_delta_pct"])) if dataset_score_rows else None
    dataset_best_win = max(dataset_score_rows, key=lambda row: float(row["score_delta_pct"])) if dataset_score_rows else None
    return {
        "status_counts": status_counts,
        "key_type_counts": key_type_counts,
        "source_counts": source_counts,
        "dataset_count": len(dataset_rows),
        "score_delta_count": len(score_rows),
        "worst_loss": worst_loss,
        "best_win": best_win,
        "dataset_worst_loss": dataset_worst_loss,
        "dataset_best_win": dataset_best_win,
    }


def summarize_gnss_gpu_score_scopes(rows: list[dict[str, Any]]) -> dict[str, Any]:
    scope_counts: dict[str, int] = {}
    score_column_counts: dict[str, int] = {}
    traps: list[dict[str, Any]] = []
    for row in rows:
        scope = str(row.get("score_scope") or "unknown")
        column = str(row.get("score_column") or "unknown")
        scope_counts[scope] = scope_counts.get(scope, 0) + 1
        score_column_counts[column] = score_column_counts.get(column, 0) + 1
        raw_scores = row.get("raw_scores")
        if not isinstance(raw_scores, dict):
            continue
        segment_score = parse_optional_float(raw_scores.get("segment_ppc_pct"))
        honest_score = parse_optional_float(raw_scores.get("honest_ppc_pct"))
        if segment_score is None or honest_score is None:
            continue
        score_gap = segment_score - honest_score
        if segment_score >= 80.0 and score_gap >= 20.0:
            traps.append(
                {
                    "label": row.get("label"),
                    "path": row.get("_path"),
                    "family": row.get("family"),
                    "dataset_key": row.get("dataset_key"),
                    "method": row.get("method") or row.get("profile"),
                    "segment_ppc_pct": rounded(segment_score, 6),
                    "honest_ppc_pct": rounded(honest_score, 6),
                    "score_gap_pct": rounded(score_gap, 6),
                    "segment_total_m": rounded(parse_optional_float(row.get("total_m")), 6),
                    "n_epochs": row.get("n_epochs"),
                    "score_column": row.get("score_column"),
                    "score_scope": row.get("score_scope"),
                }
            )
    traps.sort(
        key=lambda row: (
            float(row.get("score_gap_pct") or 0.0),
            float(row.get("segment_ppc_pct") or 0.0),
            str(row.get("label") or ""),
        ),
        reverse=True,
    )
    return {
        "scope_counts": scope_counts,
        "score_column_counts": score_column_counts,
        "segment_over_honest_trap_count": len(traps),
        "segment_over_honest_traps": traps[:24],
    }


def build_gnss_gpu_method_comparisons(rows: list[dict[str, Any]]) -> list[dict[str, Any]]:
    by_dataset: dict[str, dict[str, list[dict[str, Any]]]] = {}
    for row in rows:
        if row.get("family") != "gnss_gpu":
            continue
        dataset_key = row.get("dataset_key")
        if not isinstance(dataset_key, str) or not dataset_key:
            continue
        score = parse_optional_float(row.get("score_pct"))
        if score is None:
            continue
        method = str(row.get("method") or row.get("label") or "").lower()
        if "hybrid" not in method:
            continue
        bucket = "rtkdiag" if "rtkdiag" in method else "hybrid"
        by_dataset.setdefault(dataset_key, {}).setdefault(bucket, []).append(row)

    comparisons: list[dict[str, Any]] = []
    for dataset_key, groups in by_dataset.items():
        hybrid_rows = groups.get("hybrid", [])
        rtkdiag_rows = groups.get("rtkdiag", [])
        if not hybrid_rows or not rtkdiag_rows:
            continue
        hybrid_best = max(hybrid_rows, key=gnss_gpu_objective)
        rtkdiag_best = max(rtkdiag_rows, key=gnss_gpu_objective)
        score_delta = rounded(float(rtkdiag_best["score_pct"]) - float(hybrid_best["score_pct"]), 6)
        p95_delta = None
        if rtkdiag_best.get("p95_m") is not None and hybrid_best.get("p95_m") is not None:
            p95_delta = rounded(float(rtkdiag_best["p95_m"]) - float(hybrid_best["p95_m"]), 6)
        if score_delta is None:
            status = "n/a"
        elif score_delta >= 1.0:
            status = "better"
        elif score_delta >= -1.0:
            status = "close"
        else:
            status = "worse"
        comparisons.append(
            {
                "dataset_key": dataset_key,
                "hybrid": hybrid_best,
                "rtkdiag": rtkdiag_best,
                "score_delta_pct": score_delta,
                "p95_delta_m": p95_delta,
                "status": status,
                "candidate_counts": {
                    "hybrid": len(hybrid_rows),
                    "rtkdiag": len(rtkdiag_rows),
                },
            }
        )

    comparisons.sort(
        key=lambda row: (
            float(row.get("score_delta_pct") or 0.0),
            str(row.get("dataset_key") or ""),
        )
    )
    return comparisons


def gnss_gpu_city_run_key(row: dict[str, Any]) -> str | None:
    city = row.get("city")
    run = row.get("run")
    if isinstance(city, str) and city and isinstance(run, str) and run:
        return f"{city}/{run}"
    dataset_key = row.get("dataset_key")
    if not isinstance(dataset_key, str):
        return None
    match = re.match(r"([a-z]+)_run(\d+)", dataset_key)
    if match:
        return f"{match.group(1)}/run{match.group(2)}"
    return None


def gnss_gpu_cluster_value(row: dict[str, Any], dimension: str) -> str | None:
    if dimension == "experiment":
        value = row.get("experiment")
    elif dimension == "method":
        value = row.get("method") or row.get("experiment")
    elif dimension == "profile":
        value = row.get("profile") or row.get("method") or row.get("experiment")
    elif dimension == "city_run":
        return gnss_gpu_city_run_key(row)
    elif dimension == "source":
        value = row.get("source_kind")
    else:
        value = None
    if isinstance(value, str) and value.strip():
        return value.strip()
    return None


def build_matched_gnss_gpu_failure_clusters(
    comparisons: list[dict[str, Any]],
) -> list[dict[str, Any]]:
    dataset_rows = [row for row in comparisons if row.get("key_type") == "dataset"]
    source_rows = dataset_rows if dataset_rows else comparisons
    loss_rows = [
        row
        for row in source_rows
        if row.get("score_delta_pct") is not None and float(row["score_delta_pct"]) < -0.5
    ]
    groups: dict[tuple[str, str], list[dict[str, Any]]] = {}
    for row in loss_rows:
        gpu_row = row.get("gnss_gpu")
        lib_row = row.get("libgnsspp")
        if not isinstance(gpu_row, dict) or not isinstance(lib_row, dict):
            continue
        candidates = (
            ("GNSS GPU method", gnss_gpu_cluster_value(gpu_row, "method")),
            ("GNSS GPU experiment", gnss_gpu_cluster_value(gpu_row, "experiment")),
            ("dataset run", gnss_gpu_cluster_value(gpu_row, "city_run")),
            ("libgnss++ experiment", gnss_gpu_cluster_value(lib_row, "experiment")),
            ("libgnss++ profile", gnss_gpu_cluster_value(lib_row, "profile")),
        )
        for dimension, value in candidates:
            if value:
                groups.setdefault((dimension, value), []).append(row)

    clusters: list[dict[str, Any]] = []
    for (dimension, value), rows in groups.items():
        deltas = [float(row["score_delta_pct"]) for row in rows if row.get("score_delta_pct") is not None]
        p95_deltas = [float(row["p95_delta_m"]) for row in rows if row.get("p95_delta_m") is not None]
        if not deltas:
            continue
        worst = min(rows, key=lambda row: float(row["score_delta_pct"]))
        clusters.append(
            {
                "dimension": dimension,
                "value": value,
                "count": len(rows),
                "avg_delta_pct": rounded(sum(deltas) / len(deltas), 6),
                "worst_delta_pct": rounded(min(deltas), 6),
                "best_delta_pct": rounded(max(deltas), 6),
                "avg_p95_delta_m": rounded(sum(p95_deltas) / len(p95_deltas), 6) if p95_deltas else None,
                "worst_key": worst.get("key"),
                "worst_key_type": worst.get("key_type"),
                "worst_gnss_gpu": (worst.get("gnss_gpu") or {}).get("label")
                if isinstance(worst.get("gnss_gpu"), dict)
                else None,
                "worst_libgnsspp": (worst.get("libgnsspp") or {}).get("label")
                if isinstance(worst.get("libgnsspp"), dict)
                else None,
            }
        )

    dimension_rank = {
        "GNSS GPU method": 0,
        "GNSS GPU experiment": 1,
        "dataset run": 2,
        "libgnss++ experiment": 3,
        "libgnss++ profile": 4,
    }
    clusters.sort(
        key=lambda row: (
            dimension_rank.get(str(row.get("dimension", "")), 99),
            -int(row.get("count", 0)),
            float(row.get("avg_delta_pct") or 0.0),
            float(row.get("worst_delta_pct") or 0.0),
            str(row.get("value", "")),
        )
    )
    return clusters


def gnss_gpu_dataset_key_components(dataset_key: Any) -> tuple[str, str, int] | None:
    if not isinstance(dataset_key, str):
        return None
    match = re.fullmatch(r"([a-z]+)_run(\d+)_seg(\d+)", dataset_key)
    if not match:
        return None
    return match.group(1), f"run{int(match.group(2))}", int(match.group(3))


def gnss_gpu_epoch_path_dataset_keys(path: Path) -> set[str]:
    text = path.stem.lower()
    keys: set[str] = set()
    for match in re.finditer(r"(tokyo|nagoya)_run(\d+)[_-](?:seg)?(\d+)", text):
        keys.add(f"{match.group(1)}_run{int(match.group(2))}_seg{int(match.group(3))}")
    for match in re.finditer(r"(?:^|_)([tn])(\d+)[_-](\d+)(?:_|$)", text):
        city = "tokyo" if match.group(1) == "t" else "nagoya"
        keys.add(f"{city}_run{int(match.group(2))}_seg{int(match.group(3))}")
    return keys


def gnss_gpu_epoch_diagnostic_priority(path: Path) -> tuple[int, str]:
    name = path.name.lower()
    if "realtime_fusion" in name:
        rank = 0
    elif "internal_diag" in name:
        rank = 1
    elif "ppc_scan" in name:
        rank = 2
    else:
        rank = 3
    return rank, name


def gnss_gpu_epoch_summary_priority(row: dict[str, Any]) -> tuple[int, int, float, str]:
    path = Path(str(row.get("path", "")))
    file_rank = gnss_gpu_epoch_diagnostic_priority(path)[0]
    epoch_rank = 0 if int(row.get("n_epochs") or 0) >= 50 else 1
    return (
        epoch_rank,
        file_rank,
        -(float(row.get("p95_error_m") or 0.0)),
        str(row.get("path", "")),
    )


def gnss_gpu_epoch_objective(row: dict[str, Any]) -> tuple[int, float]:
    p95 = parse_optional_float(row.get("p95_error_m"))
    if p95 is not None:
        return (2, -p95)
    score = parse_optional_float(row.get("score_pct"))
    if score is not None:
        return (1, score)
    median = parse_optional_float(row.get("median_error_m"))
    if median is not None:
        return (0, -median)
    return (-1, 0.0)


def row_float(row: dict[str, Any], key: str) -> float | None:
    return parse_optional_float(row.get(key))


def boolish(value: Any) -> bool:
    return str(value).strip().lower() in {"1", "true", "t", "yes", "y"}


def first_existing_column(fieldnames: set[str], columns: tuple[str, ...]) -> str | None:
    for column in columns:
        if column in fieldnames:
            return column
    return None


def count_text_values(rows: list[dict[str, Any]], column: str, limit: int = 4) -> list[dict[str, Any]]:
    counts: dict[str, int] = {}
    for row in rows:
        text = str(row.get(column, "")).strip()
        if not text:
            continue
        counts[text] = counts.get(text, 0) + 1
    return [
        {"value": value, "count": count}
        for value, count in sorted(counts.items(), key=lambda item: (-item[1], item[0]))[:limit]
    ]


def rate_for_bool_column(rows: list[dict[str, Any]], column: str) -> float | None:
    if not rows or column not in rows[0]:
        return None
    return rounded(100.0 * sum(1 for row in rows if boolish(row.get(column))) / len(rows), 3)


GNSS_GPU_STAGE_COLUMNS: tuple[tuple[str, str, str], ...] = (
    ("stage_start_m", "start", "pf_epoch_start_to_ref_m"),
    ("stage_predict_m", "predict", "pf_after_predict_to_ref_m"),
    ("stage_pr_m", "PR", "pf_after_pr_to_ref_m"),
    ("stage_doppler_m", "doppler", "pf_after_doppler_to_ref_m"),
    ("stage_dd_carrier_m", "DD carrier", "pf_after_dd_carrier_to_ref_m"),
    ("stage_position_m", "position", "pf_after_position_update_to_ref_m"),
    ("stage_hybrid_m", "hybrid", "pf_after_hybrid_to_ref_m"),
    ("stage_rtkdiag_m", "rtkdiag", "pf_after_rtkdiag_to_ref_m"),
    ("stage_emit_m", "emit", "emit_to_ref_m"),
    ("stage_end_m", "end", "pf_epoch_end_to_ref_m"),
)


def summarize_gnss_gpu_stages(
    rows: list[dict[str, Any]],
    fieldnames: set[str],
    fail_threshold_m: float,
    ppc_threshold_m: float,
) -> list[dict[str, Any]]:
    summaries: list[dict[str, Any]] = []
    for point_key, label, column in GNSS_GPU_STAGE_COLUMNS:
        if column not in fieldnames:
            continue
        values: list[tuple[dict[str, Any], float]] = []
        for row in rows:
            value = row_float(row, column)
            if value is not None:
                values.append((row, value))
        if not values:
            continue
        fail = [(row, value) for row, value in values if value > fail_threshold_m]
        ppc_fail = [(row, value) for row, value in values if value > ppc_threshold_m]
        worst_row, worst_value = max(values, key=lambda item: item[1])
        first_fail_row = fail[0][0] if fail else None
        first_ppc_fail_row = ppc_fail[0][0] if ppc_fail else None
        numeric_values = [value for _, value in values]
        summaries.append(
            {
                "point_key": point_key,
                "label": label,
                "column": column,
                "median_m": rounded(percentile_or_none(numeric_values, 0.5), 6),
                "p95_m": rounded(percentile_or_none(numeric_values, 0.95), 6),
                "max_m": rounded(worst_value, 6),
                "fail_epochs_gt05m": len(ppc_fail),
                "first_fail_epoch_gt05m": rounded(row_float(first_ppc_fail_row, "epoch"), 3)
                if first_ppc_fail_row
                else None,
                "fail_epochs_gt3m": len(fail),
                "first_fail_epoch": rounded(row_float(first_fail_row, "epoch"), 3) if first_fail_row else None,
                "worst_epoch": rounded(row_float(worst_row, "epoch"), 3),
            }
        )
    return summaries


def build_gnss_gpu_stage_transitions(stage_summaries: list[dict[str, Any]]) -> list[dict[str, Any]]:
    transitions: list[dict[str, Any]] = []
    previous: dict[str, Any] | None = None
    for stage in stage_summaries:
        if previous is None:
            previous = stage
            continue
        previous_p95 = parse_optional_float(previous.get("p95_m"))
        current_p95 = parse_optional_float(stage.get("p95_m"))
        previous_median = parse_optional_float(previous.get("median_m"))
        current_median = parse_optional_float(stage.get("median_m"))
        if previous_p95 is not None and current_p95 is not None:
            transitions.append(
                {
                    "from_label": previous.get("label"),
                    "to_label": stage.get("label"),
                    "from_point_key": previous.get("point_key"),
                    "to_point_key": stage.get("point_key"),
                    "delta_p95_m": rounded(current_p95 - previous_p95, 6),
                    "from_p95_m": rounded(previous_p95, 6),
                    "to_p95_m": rounded(current_p95, 6),
                    "delta_median_m": rounded(current_median - previous_median, 6)
                    if previous_median is not None and current_median is not None
                    else None,
                    "from_fail_epochs_gt3m": previous.get("fail_epochs_gt3m"),
                    "to_fail_epochs_gt3m": stage.get("fail_epochs_gt3m"),
                    "from_fail_epochs_gt05m": previous.get("fail_epochs_gt05m"),
                    "to_fail_epochs_gt05m": stage.get("fail_epochs_gt05m"),
                }
            )
        previous = stage
    return transitions


def summarize_gnss_gpu_doppler(rows: list[dict[str, Any]], fieldnames: set[str]) -> dict[str, Any] | None:
    if "pf_after_pr_to_ref_m" not in fieldnames or "pf_after_doppler_to_ref_m" not in fieldnames:
        return None
    deltas: list[tuple[dict[str, Any], float]] = []
    for row in rows:
        before = row_float(row, "pf_after_pr_to_ref_m")
        after = row_float(row, "pf_after_doppler_to_ref_m")
        if before is None or after is None:
            continue
        deltas.append((row, after - before))
    if not deltas:
        return None

    values = [delta for _, delta in deltas]
    positive_values = [delta for delta in values if delta > 0.0]
    worst_row, worst_delta = max(deltas, key=lambda item: item[1])
    update_rate = rate_for_bool_column(rows, "doppler_update_applied")
    skip_counts = count_text_values(rows, "doppler_gate_skip_reason") if "doppler_gate_skip_reason" in fieldnames else []
    p95_delta = percentile_or_none(positive_values or values, 0.95)
    if worst_delta >= 20.0:
        diagnosis = "doppler shock"
    elif p95_delta is not None and p95_delta >= 2.0:
        diagnosis = "doppler drift amplifier"
    elif update_rate == 0.0:
        diagnosis = "doppler gated off"
    else:
        diagnosis = "doppler stable"

    return {
        "diagnosis": diagnosis,
        "update_rate_pct": update_rate,
        "max_error_delta_m": rounded(worst_delta, 6),
        "p95_positive_error_delta_m": rounded(p95_delta, 6),
        "positive_epoch_count": sum(1 for delta in values if delta > 0.0),
        "shock_epoch_count": sum(1 for delta in values if delta >= 10.0),
        "worst_epoch": rounded(row_float(worst_row, "epoch"), 3),
        "worst_tow": rounded(row_float(worst_row, "tow"), 3),
        "worst_before_m": rounded(row_float(worst_row, "pf_after_pr_to_ref_m"), 6),
        "worst_after_m": rounded(row_float(worst_row, "pf_after_doppler_to_ref_m"), 6),
        "worst_pfvel_rms_mps": rounded(row_float(worst_row, "doppler_current_pfvel_rms_mps"), 6),
        "worst_refvel_rms_mps": rounded(row_float(worst_row, "doppler_current_refvel_rms_mps"), 6),
        "worst_wls_rms_mps": rounded(row_float(worst_row, "doppler_current_wls_rms_mps"), 6),
        "worst_wls_speed_mps": rounded(row_float(worst_row, "doppler_current_wls_speed_mps"), 6),
        "worst_wls_to_refvel_mps": rounded(row_float(worst_row, "doppler_current_wls_to_refvel_mps"), 6),
        "skip_counts": skip_counts,
    }


def summarize_gnss_gpu_position_update(rows: list[dict[str, Any]], fieldnames: set[str]) -> dict[str, Any] | None:
    if "pf_after_position_update_to_ref_m" not in fieldnames:
        return None
    before_column = first_existing_column(
        fieldnames,
        (
            "pf_after_dd_pr_ls_anchor_to_ref_m",
            "pf_after_dd_carrier_to_ref_m",
            "pf_after_doppler_to_ref_m",
            "pf_after_pr_to_ref_m",
        ),
    )
    deltas: list[tuple[dict[str, Any], float]] = []
    for row in rows:
        after = row_float(row, "pf_after_position_update_to_ref_m")
        before = row_float(row, before_column) if before_column else None
        if before is not None and after is not None:
            deltas.append((row, after - before))

    wls_to_ref_values = [
        value for value in (row_float(row, "position_update_wls_to_ref_m") for row in rows) if value is not None
    ]
    wls_to_pf_values = [
        value for value in (row_float(row, "position_update_wls_to_pf_before_m") for row in rows) if value is not None
    ]
    rms_values = [
        value for value in (row_float(row, "position_update_wls_postfit_rms_m") for row in rows) if value is not None
    ]
    skip_min_epoch_rate = rate_for_bool_column(rows, "position_update_skipped_min_epoch")
    skip_rms_rate = rate_for_bool_column(rows, "position_update_skipped_wls_rms")
    skip_pdop_rate = rate_for_bool_column(rows, "position_update_skipped_wls_pdop")
    skip_wls_to_pf_rate = rate_for_bool_column(rows, "position_update_skipped_wls_to_pf")
    if not deltas and not wls_to_ref_values and not rms_values:
        return None

    worst_row: dict[str, Any] | None = None
    worst_delta: float | None = None
    if deltas:
        worst_row, worst_delta = max(deltas, key=lambda item: item[1])
    p95_delta = percentile_or_none([delta for _, delta in deltas if delta > 0.0] or [delta for _, delta in deltas], 0.95)
    median_wls_to_ref = percentile_or_none(wls_to_ref_values, 0.5)
    if worst_delta is not None and worst_delta >= 5.0:
        diagnosis = "position update shock"
    elif p95_delta is not None and p95_delta >= 2.0:
        diagnosis = "position update amplifier"
    elif median_wls_to_ref is not None and median_wls_to_ref > 3.0:
        diagnosis = "weak WLS update target"
    else:
        diagnosis = "position update stable"

    return {
        "diagnosis": diagnosis,
        "before_column": before_column,
        "max_error_delta_m": rounded(worst_delta, 6),
        "p95_positive_error_delta_m": rounded(p95_delta, 6),
        "positive_epoch_count": sum(1 for _, delta in deltas if delta > 0.0),
        "worst_epoch": rounded(row_float(worst_row, "epoch"), 3) if worst_row else None,
        "worst_before_m": rounded(row_float(worst_row, before_column), 6) if worst_row and before_column else None,
        "worst_after_m": rounded(row_float(worst_row, "pf_after_position_update_to_ref_m"), 6) if worst_row else None,
        "median_wls_to_ref_m": rounded(median_wls_to_ref, 6),
        "p95_wls_to_ref_m": rounded(percentile_or_none(wls_to_ref_values, 0.95), 6),
        "median_wls_to_pf_before_m": rounded(percentile_or_none(wls_to_pf_values, 0.5), 6),
        "median_wls_postfit_rms_m": rounded(percentile_or_none(rms_values, 0.5), 6),
        "p95_wls_postfit_rms_m": rounded(percentile_or_none(rms_values, 0.95), 6),
        "skip_min_epoch_rate_pct": skip_min_epoch_rate,
        "skip_wls_rms_rate_pct": skip_rms_rate,
        "skip_wls_pdop_rate_pct": skip_pdop_rate,
        "skip_wls_to_pf_rate_pct": skip_wls_to_pf_rate,
    }


def stage_diagnosis_label(
    first_bad_stage: dict[str, Any] | None,
    largest_transition: dict[str, Any] | None,
) -> str:
    first_label = str((first_bad_stage or {}).get("label", ""))
    transition_delta = parse_optional_float((largest_transition or {}).get("delta_p95_m"))
    if first_label == "start":
        if transition_delta is not None and transition_delta >= 2.0:
            return "bad initial state plus stage amplification"
        return "bad initial state"
    if transition_delta is not None and transition_delta >= 2.0:
        return "stage amplification"
    if first_label:
        return f"first failure at {first_label}"
    return "no stage failure"


def abs_values(rows: list[dict[str, Any]], column: str) -> list[float]:
    values: list[float] = []
    for row in rows:
        value = row_float(row, column)
        if value is not None:
            values.append(abs(value))
    return values


def first_epoch_row(rows: list[dict[str, Any]]) -> dict[str, Any]:
    return min(rows, key=lambda row: row_float(row, "epoch") if row_float(row, "epoch") is not None else float("inf"))


def initial_state_diagnosis(epoch0: dict[str, Any]) -> str:
    start = parse_optional_float(epoch0.get("start_to_ref_m"))
    init = parse_optional_float(epoch0.get("init_to_ref_m"))
    if start is None:
        start = init
    east = abs(parse_optional_float(epoch0.get("err_e_m")) or 0.0)
    north = abs(parse_optional_float(epoch0.get("err_n_m")) or 0.0)
    up = abs(parse_optional_float(epoch0.get("err_u_m")) or 0.0)
    spread = parse_optional_float(epoch0.get("init_spread_pos_m"))
    if spread is None:
        spread = parse_optional_float(epoch0.get("spread_m"))
    if start is None or start <= 3.0:
        return "initial state near reference"
    horizontal = math.hypot(east, north)
    if up > 3.0 and up > horizontal * 1.5:
        return "vertical-biased initial error"
    if horizontal > 3.0 and horizontal > up * 1.2:
        return "horizontal initial error"
    if spread is not None and spread > 20.0:
        return "diffuse initial particle cloud"
    if spread is not None and spread < 5.0:
        return "overconfident wrong initial state"
    return "mixed initial error"


def summarize_initial_state(rows: list[dict[str, Any]], fieldnames: set[str]) -> dict[str, Any] | None:
    if "pf_epoch_start_to_ref_m" not in fieldnames:
        return None
    start_values = [value for value in (row_float(row, "pf_epoch_start_to_ref_m") for row in rows) if value is not None]
    if not start_values:
        return None
    epoch0_row = first_epoch_row(rows)
    epoch0 = {
        "epoch": rounded(row_float(epoch0_row, "epoch"), 3),
        "tow": rounded(row_float(epoch0_row, "tow"), 3),
        "init_source": first_text(epoch0_row, ("pf_init_source",)),
        "init_spread_pos_m": rounded(row_float(epoch0_row, "pf_init_spread_pos_m"), 6),
        "init_spread_cb_m": rounded(row_float(epoch0_row, "pf_init_spread_cb_m"), 6),
        "init_clock_bias_m": rounded(row_float(epoch0_row, "pf_init_clock_bias_m"), 6),
        "init_to_ref_m": rounded(row_float(epoch0_row, "pf_init_to_ref_m"), 6),
        "init_err_e_m": rounded(row_float(epoch0_row, "pf_init_err_e_m"), 6),
        "init_err_n_m": rounded(row_float(epoch0_row, "pf_init_err_n_m"), 6),
        "init_err_u_m": rounded(row_float(epoch0_row, "pf_init_err_u_m"), 6),
        "wls_init_to_ref_m": rounded(row_float(epoch0_row, "wls_init_to_ref_m"), 6),
        "wls_init_err_e_m": rounded(row_float(epoch0_row, "wls_init_err_e_m"), 6),
        "wls_init_err_n_m": rounded(row_float(epoch0_row, "wls_init_err_n_m"), 6),
        "wls_init_err_u_m": rounded(row_float(epoch0_row, "wls_init_err_u_m"), 6),
        "hybrid_init_available": boolish(epoch0_row.get("hybrid_init_available")),
        "hybrid_init_to_ref_m": rounded(row_float(epoch0_row, "hybrid_init_to_ref_m"), 6),
        "hybrid_init_err_e_m": rounded(row_float(epoch0_row, "hybrid_init_err_e_m"), 6),
        "hybrid_init_err_n_m": rounded(row_float(epoch0_row, "hybrid_init_err_n_m"), 6),
        "hybrid_init_err_u_m": rounded(row_float(epoch0_row, "hybrid_init_err_u_m"), 6),
        "start_to_ref_m": rounded(row_float(epoch0_row, "pf_epoch_start_to_ref_m"), 6),
        "err_e_m": rounded(row_float(epoch0_row, "pf_epoch_start_err_e_m"), 6),
        "err_n_m": rounded(row_float(epoch0_row, "pf_epoch_start_err_n_m"), 6),
        "err_u_m": rounded(row_float(epoch0_row, "pf_epoch_start_err_u_m"), 6),
        "spread_m": rounded(row_float(epoch0_row, "pf_epoch_start_spread_m"), 6),
        "ess_ratio": rounded(row_float(epoch0_row, "pf_epoch_start_ess_ratio"), 6),
        "predict_source": first_text(epoch0_row, ("predict_source",)),
        "pr_update_mode": first_text(epoch0_row, ("pr_update_mode",)),
        "n_sat_raw": rounded(row_float(epoch0_row, "n_sat_raw"), 3),
        "n_sat_used_pr": rounded(row_float(epoch0_row, "n_sat_used_pr"), 3),
        "emitted_source": first_text(epoch0_row, ("emitted_source",)),
        "resampled_epoch_end": boolish(epoch0_row.get("resampled_epoch_end")),
    }
    return {
        "median_start_to_ref_m": rounded(percentile_or_none(start_values, 0.5), 6),
        "p95_start_to_ref_m": rounded(percentile_or_none(start_values, 0.95), 6),
        "max_start_to_ref_m": rounded(max(start_values), 6),
        "p95_abs_err_e_m": rounded(percentile_or_none(abs_values(rows, "pf_epoch_start_err_e_m"), 0.95), 6),
        "p95_abs_err_n_m": rounded(percentile_or_none(abs_values(rows, "pf_epoch_start_err_n_m"), 0.95), 6),
        "p95_abs_err_u_m": rounded(percentile_or_none(abs_values(rows, "pf_epoch_start_err_u_m"), 0.95), 6),
        "median_spread_m": rounded(
            percentile_or_none(
                [value for value in (row_float(row, "pf_epoch_start_spread_m") for row in rows) if value is not None],
                0.5,
            ),
            6,
        ),
        "median_ess_ratio": rounded(
            percentile_or_none(
                [value for value in (row_float(row, "pf_epoch_start_ess_ratio") for row in rows) if value is not None],
                0.5,
            ),
            6,
        ),
        "init_source_counts": count_text_values(rows, "pf_init_source") if "pf_init_source" in fieldnames else [],
        "epoch0": epoch0,
        "diagnosis": initial_state_diagnosis(epoch0),
    }


def stage_summary_by_key(row: dict[str, Any]) -> dict[str, dict[str, Any]]:
    out: dict[str, dict[str, Any]] = {}
    for stage in row.get("stage_summaries") or []:
        if isinstance(stage, dict) and isinstance(stage.get("point_key"), str):
            out[str(stage["point_key"])] = stage
    return out


def compact_epoch_diagnostic(row: dict[str, Any]) -> dict[str, Any]:
    return {
        "path": row.get("path"),
        "source": row.get("source"),
        "kind": row.get("kind"),
        "n_epochs": row.get("n_epochs"),
        "score_pct": row.get("score_pct"),
        "median_error_m": row.get("median_error_m"),
        "p95_error_m": row.get("p95_error_m"),
        "max_error_m": row.get("max_error_m"),
        "first_fail_epoch": row.get("first_fail_epoch"),
        "first_ppc_fail_epoch": row.get("first_ppc_fail_epoch"),
        "worst_epoch": row.get("worst_epoch"),
        "first_bad_stage": row.get("first_bad_stage"),
        "first_ppc_bad_stage": row.get("first_ppc_bad_stage"),
        "worst_stage": row.get("worst_stage"),
        "largest_stage_jump": row.get("largest_stage_jump"),
        "stage_diagnosis": row.get("stage_diagnosis"),
        "ppc_stage_diagnosis": row.get("ppc_stage_diagnosis"),
        "initial_state": row.get("initial_state"),
        "doppler_diagnostics": row.get("doppler_diagnostics"),
        "position_update_diagnostics": row.get("position_update_diagnostics"),
        "dd_anchor_rate_pct": row.get("dd_anchor_rate_pct"),
        "tdcp_rate_pct": row.get("tdcp_rate_pct"),
        "doppler_update_rate_pct": row.get("doppler_update_rate_pct"),
        "dd_carrier_update_rate_pct": row.get("dd_carrier_update_rate_pct"),
        "hybrid_pu_rate_pct": row.get("hybrid_pu_rate_pct"),
        "resampled_end_rate_pct": row.get("resampled_end_rate_pct"),
    }


def build_gnss_gpu_epoch_comparisons(
    by_dataset: dict[str, list[dict[str, Any]]],
) -> list[dict[str, Any]]:
    comparisons: list[dict[str, Any]] = []
    for dataset_key, rows in by_dataset.items():
        if len(rows) < 2:
            continue
        best = max(rows, key=gnss_gpu_epoch_objective)
        worst = min(rows, key=gnss_gpu_epoch_objective)
        if best is worst:
            continue
        score_gap = None
        if best.get("score_pct") is not None and worst.get("score_pct") is not None:
            score_gap = rounded(float(best["score_pct"]) - float(worst["score_pct"]), 6)
        p95_gap = None
        if best.get("p95_error_m") is not None and worst.get("p95_error_m") is not None:
            p95_gap = rounded(float(worst["p95_error_m"]) - float(best["p95_error_m"]), 6)
        stage_gaps: list[dict[str, Any]] = []
        for point_key, label, _ in GNSS_GPU_STAGE_COLUMNS:
            stage_items: list[tuple[dict[str, Any], dict[str, Any], float]] = []
            for candidate in rows:
                stage = stage_summary_by_key(candidate).get(point_key)
                if not stage:
                    continue
                p95 = parse_optional_float(stage.get("p95_m"))
                if p95 is not None:
                    stage_items.append((candidate, stage, p95))
            if len(stage_items) < 2:
                continue
            best_candidate, best_stage, best_p95 = min(stage_items, key=lambda item: item[2])
            worst_candidate, worst_stage, worst_p95 = max(stage_items, key=lambda item: item[2])
            stage_gaps.append(
                {
                    "point_key": point_key,
                    "label": label,
                    "best_p95_m": rounded(best_p95, 6),
                    "worst_p95_m": rounded(worst_p95, 6),
                    "delta_p95_m": rounded(worst_p95 - best_p95, 6),
                    "best_source": best_candidate.get("source"),
                    "worst_source": worst_candidate.get("source"),
                    "best_fail_epochs_gt3m": best_stage.get("fail_epochs_gt3m"),
                    "worst_fail_epochs_gt3m": worst_stage.get("fail_epochs_gt3m"),
                    "best_fail_epochs_gt05m": best_stage.get("fail_epochs_gt05m"),
                    "worst_fail_epochs_gt05m": worst_stage.get("fail_epochs_gt05m"),
                }
            )
        dominant_stage = max(stage_gaps, key=lambda row: float(row["delta_p95_m"])) if stage_gaps else None
        start_stage = next((row for row in stage_gaps if row.get("point_key") == "stage_start_m"), None)
        comparisons.append(
            {
                "dataset_key": dataset_key,
                "candidate_count": len(rows),
                "best": compact_epoch_diagnostic(best),
                "worst": compact_epoch_diagnostic(worst),
                "score_gap_pct": score_gap,
                "p95_gap_m": p95_gap,
                "dominant_stage_gap": dominant_stage,
                "start_stage_gap": start_stage,
                "stage_gaps": stage_gaps,
            }
        )
    comparisons.sort(
        key=lambda row: (
            -(float(row.get("score_gap_pct") or 0.0)),
            -(float(row.get("p95_gap_m") or 0.0)),
            str(row.get("dataset_key", "")),
        )
    )
    return comparisons


def summarize_gnss_gpu_epoch_file(
    path: Path,
    root_dir: Path,
    dataset_key: str,
) -> dict[str, Any] | None:
    try:
        with path.open("r", encoding="utf-8", newline="") as stream:
            rows = list(csv.DictReader(stream))
    except OSError:
        return None
    if not rows:
        return None
    fieldnames = set(rows[0])
    error_column = first_existing_column(
        fieldnames,
        (
            "fused_error_3d_m",
            "emit_to_ref_m",
            "pf_epoch_end_to_ref_m",
            "pf_before_emit_to_ref_m",
            "smoothed_error_3d",
            "error_3d",
        ),
    )
    error_2d_column = first_existing_column(
        fieldnames,
        ("fused_error_2d_m", "smoothed_error_2d", "forward_error_2d", "error_2d"),
    )
    if error_column is None and error_2d_column is None:
        return None
    primary_error_column = error_column or error_2d_column
    assert primary_error_column is not None
    error_values = [value for value in (row_float(row, primary_error_column) for row in rows) if value is not None]
    if not error_values:
        return None
    pass_threshold_m = 0.5
    fail_threshold_m = 3.0
    pass_flags = [
        row_float(row, primary_error_column) is not None and row_float(row, primary_error_column) <= pass_threshold_m
        for row in rows
    ]
    pass_rows = [row for index, row in enumerate(rows) if pass_flags[index]]
    ppc_fail_rows = [
        row
        for row in rows
        if (row_float(row, primary_error_column) is not None and row_float(row, primary_error_column) > pass_threshold_m)
    ]
    fail_rows = [
        row
        for row in rows
        if (row_float(row, primary_error_column) is not None and row_float(row, primary_error_column) > fail_threshold_m)
    ]
    first_ppc_fail = ppc_fail_rows[0] if ppc_fail_rows else None
    first_fail = fail_rows[0] if fail_rows else None
    worst_row = max(rows, key=lambda row: row_float(row, primary_error_column) or float("-inf"))
    distance_values = [row_float(row, "ppc_segment_distance_m") for row in rows]
    if any(value is not None for value in distance_values):
        total_m = sum(value or 0.0 for value in distance_values)
        pass_m = sum((distance_values[index] or 0.0) for index, passed in enumerate(pass_flags) if passed)
        score_pct = 100.0 * pass_m / total_m if total_m > 0.0 else None
    else:
        total_m = None
        pass_m = None
        score_pct = 100.0 * len(pass_rows) / len(rows)

    stage_summaries = summarize_gnss_gpu_stages(rows, fieldnames, fail_threshold_m, pass_threshold_m)
    first_ppc_bad_stage = next((stage for stage in stage_summaries if stage["fail_epochs_gt05m"] > 0), None)
    first_bad_stage = next((stage for stage in stage_summaries if stage["fail_epochs_gt3m"] > 0), None)
    worst_stage = max(stage_summaries, key=lambda stage: float(stage["p95_m"] or 0.0)) if stage_summaries else None
    stage_transitions = build_gnss_gpu_stage_transitions(stage_summaries)
    positive_transitions = [
        transition for transition in stage_transitions if parse_optional_float(transition.get("delta_p95_m")) is not None
    ]
    largest_stage_jump = (
        max(positive_transitions, key=lambda transition: float(transition["delta_p95_m"]))
        if positive_transitions
        else None
    )
    initial_state = summarize_initial_state(rows, fieldnames)
    doppler_diagnostics = summarize_gnss_gpu_doppler(rows, fieldnames)
    position_update_diagnostics = summarize_gnss_gpu_position_update(rows, fieldnames)

    points: list[dict[str, Any]] = []
    for row in rows:
        error_value = row_float(row, primary_error_column)
        if error_value is None:
            continue
        point = {
            "epoch": rounded(row_float(row, "epoch"), 3),
            "tow": rounded(row_float(row, "tow"), 3),
            "error_m": rounded(error_value, 6),
            "error_2d_m": rounded(row_float(row, error_2d_column), 6) if error_2d_column else None,
            "wls_error_m": rounded(first_float(row, ("wls_error_3d_m", "wls_error_2d_m")), 6),
            "dd_shift_m": rounded(row_float(row, "dd_pr_shift_m"), 6),
            "dd_rms_m": rounded(row_float(row, "dd_pr_robust_rms_m"), 6),
            "tdcp_postfit_rms_m": rounded(row_float(row, "tdcp_postfit_rms_m"), 6),
            "height_hold_correction_m": rounded(row_float(row, "height_hold_correction_m"), 6),
            "pf_spread_m": rounded(first_float(row, ("pf_epoch_end_spread_m", "pf_before_emit_spread_m")), 6),
            "ess_ratio": rounded(first_float(row, ("pf_epoch_end_ess_ratio", "pf_before_emit_ess_ratio")), 6),
            "dd_pr_anchor_used": boolish(row.get("dd_pr_anchor_used")),
            "tdcp_used": boolish(row.get("tdcp_used")),
            "height_hold_used": boolish(row.get("height_hold_used")),
            "widelane_anchor_used": boolish(row.get("widelane_anchor_used")),
            "doppler_update_applied": boolish(row.get("doppler_update_applied")),
            "dd_carrier_update_applied": boolish(row.get("dd_carrier_update_applied")),
            "hybrid_pu_applied": boolish(row.get("hybrid_pu_applied")),
            "resampled_epoch_end": boolish(row.get("resampled_epoch_end")),
        }
        for point_key, _, column in GNSS_GPU_STAGE_COLUMNS:
            point[point_key] = rounded(row_float(row, column), 6)
        points.append(point)

    text_counts: dict[str, list[dict[str, Any]]] = {}
    for column in ("emitted_source", "rtkdiag_selected_label", "doppler_gate_skip_reason", "widelane_reason", "predict_source"):
        if column in fieldnames:
            text_counts[column] = count_text_values(rows, column)

    return {
        "dataset_key": dataset_key,
        "path": relative_display(path, root_dir),
        "source": path.name.removesuffix(".csv"),
        "kind": "internal" if "internal" in path.name else "epoch",
        "n_epochs": len(rows),
        "error_column": primary_error_column,
        "score_pct": rounded(score_pct, 6),
        "pass_threshold_m": rounded(pass_threshold_m, 6),
        "pass_epochs": len(pass_rows),
        "fail_epochs_gt05m": len(ppc_fail_rows),
        "fail_epochs_gt3m": len(fail_rows),
        "pass_m": rounded(pass_m, 6),
        "total_m": rounded(total_m, 6),
        "median_error_m": rounded(percentile_or_none(error_values, 0.5), 6),
        "p95_error_m": rounded(percentile_or_none(error_values, 0.95), 6),
        "max_error_m": rounded(max(error_values), 6),
        "first_ppc_fail_epoch": rounded(row_float(first_ppc_fail, "epoch"), 3) if first_ppc_fail else None,
        "first_ppc_fail_tow": rounded(row_float(first_ppc_fail, "tow"), 3) if first_ppc_fail else None,
        "first_fail_epoch": rounded(row_float(first_fail, "epoch"), 3) if first_fail else None,
        "first_fail_tow": rounded(row_float(first_fail, "tow"), 3) if first_fail else None,
        "worst_epoch": rounded(row_float(worst_row, "epoch"), 3),
        "worst_tow": rounded(row_float(worst_row, "tow"), 3),
        "dd_anchor_rate_pct": rate_for_bool_column(rows, "dd_pr_anchor_used"),
        "tdcp_rate_pct": rate_for_bool_column(rows, "tdcp_used"),
        "height_hold_rate_pct": rate_for_bool_column(rows, "height_hold_used"),
        "widelane_rate_pct": rate_for_bool_column(rows, "widelane_anchor_used"),
        "doppler_update_rate_pct": rate_for_bool_column(rows, "doppler_update_applied"),
        "dd_carrier_update_rate_pct": rate_for_bool_column(rows, "dd_carrier_update_applied"),
        "hybrid_pu_rate_pct": rate_for_bool_column(rows, "hybrid_pu_applied"),
        "resampled_end_rate_pct": rate_for_bool_column(rows, "resampled_epoch_end"),
        "stage_summaries": stage_summaries,
        "stage_transitions": stage_transitions,
        "first_ppc_bad_stage": first_ppc_bad_stage,
        "first_bad_stage": first_bad_stage,
        "worst_stage": worst_stage,
        "largest_stage_jump": largest_stage_jump,
        "stage_diagnosis": stage_diagnosis_label(first_bad_stage, largest_stage_jump),
        "ppc_stage_diagnosis": stage_diagnosis_label(first_ppc_bad_stage, largest_stage_jump),
        "initial_state": initial_state,
        "doppler_diagnostics": doppler_diagnostics,
        "position_update_diagnostics": position_update_diagnostics,
        "text_counts": text_counts,
        "points": downsample_points(points, 320),
    }


def load_gnss_gpu_epoch_diagnostics(
    root_dir: Path,
    matched_by_loss: list[dict[str, Any]],
) -> dict[str, Any]:
    targets: list[str] = []
    for row in matched_by_loss:
        if row.get("key_type") != "dataset":
            continue
        key = row.get("key")
        if gnss_gpu_dataset_key_components(key) is None:
            continue
        if key not in targets:
            targets.append(str(key))
        if len(targets) >= 8:
            break

    by_dataset: dict[str, list[dict[str, Any]]] = {key: [] for key in targets}
    if not targets:
        return {"targets": [], "files": [], "by_dataset": {}, "top": []}

    target_set = set(targets)
    candidates: dict[str, list[Path]] = {key: [] for key in targets}
    for path in glob_artifact_paths(root_dir, "experiments/results/*_epochs.csv"):
        keys = gnss_gpu_epoch_path_dataset_keys(path)
        for key in target_set.intersection(keys):
            candidates[key].append(path)

    files: list[dict[str, Any]] = []
    top: list[dict[str, Any]] = []
    for key in targets:
        paths = sorted(candidates.get(key, []), key=gnss_gpu_epoch_diagnostic_priority)[:24]
        for path in paths:
            summary = summarize_gnss_gpu_epoch_file(path, root_dir, key)
            if summary is None:
                continue
            by_dataset[key].append(summary)
            files.append({"dataset_key": key, "path": summary["path"], "rows": summary["n_epochs"]})
        by_dataset[key].sort(
            key=gnss_gpu_epoch_summary_priority
        )
        if by_dataset[key]:
            top.extend(by_dataset[key][:2])

    comparisons = build_gnss_gpu_epoch_comparisons(by_dataset)
    top.sort(
        key=lambda row: (
            targets.index(str(row.get("dataset_key"))) if row.get("dataset_key") in targets else 999,
            -(float(row.get("p95_error_m") or 0.0)),
        )
    )
    return {
        "targets": targets,
        "files": files,
        "by_dataset": by_dataset,
        "top": top[:12],
        "comparisons": comparisons[:16],
    }


def load_gnss_gpu_summaries(
    root_dir: Path,
    pattern: str,
    runs_pattern: str | None = None,
) -> dict[str, Any]:
    rows: list[dict[str, Any]] = []
    files: list[dict[str, Any]] = []
    csv_sources = [("summary", path) for path in glob_artifact_paths(root_dir, pattern)]
    if runs_pattern:
        csv_sources.extend(("runs", path) for path in glob_artifact_paths(root_dir, runs_pattern))
    seen_paths: set[tuple[str, Path]] = set()
    for source_kind, path in csv_sources:
        source_key = (source_kind, path.resolve())
        if source_key in seen_paths:
            continue
        seen_paths.add(source_key)
        file_rows = 0
        try:
            with path.open("r", encoding="utf-8", newline="") as stream:
                reader = csv.DictReader(stream)
                for raw_row in reader:
                    file_rows += 1
                    normalized = normalize_gnss_gpu_summary_row(path, root_dir, raw_row, source_kind)
                    if normalized is not None:
                        rows.append(normalized)
        except OSError:
            continue
        files.append({"path": relative_display(path, root_dir), "rows": file_rows, "kind": source_kind})

    rows.sort(
        key=lambda row: (
            int(row.get("objective_rank", -1)),
            float(row.get("objective_value", 0.0)),
            str(row.get("label", "")),
        ),
        reverse=True,
    )
    family_counts: dict[str, int] = {}
    for row in rows:
        family = str(row.get("family", "unknown"))
        family_counts[family] = family_counts.get(family, 0) + 1

    best_by_family: dict[str, dict[str, Any]] = {}
    for row in rows:
        family = str(row.get("family", "unknown"))
        if family not in best_by_family:
            best_by_family[family] = row

    gpu_best = best_by_family.get("gnss_gpu")
    lib_best = best_by_family.get("libgnss++")
    comparison: dict[str, Any] | None = None
    if gpu_best is not None and lib_best is not None:
        comparison = {
            "gnss_gpu": gpu_best,
            "libgnsspp": lib_best,
        }
        comparison.update(gnss_gpu_comparison_delta(gpu_best, lib_best))

    matched_comparisons = build_matched_gnss_gpu_comparisons(rows)
    matched_by_loss = sorted(
        [row for row in matched_comparisons if row.get("score_delta_pct") is not None],
        key=lambda row: float(row["score_delta_pct"]),
    )
    matched_by_win = list(reversed(matched_by_loss))
    matched_failure_clusters = build_matched_gnss_gpu_failure_clusters(matched_comparisons)
    matched_epoch_diagnostics = load_gnss_gpu_epoch_diagnostics(root_dir, matched_by_loss)
    method_comparisons = build_gnss_gpu_method_comparisons(rows)
    method_regressions = [row for row in method_comparisons if row.get("status") == "worse"]

    return {
        "files": files,
        "row_count": len(rows),
        "rows": rows[:120],
        "top_rows": rows[:24],
        "family_counts": family_counts,
        "best_by_family": best_by_family,
        "comparison": comparison,
        "matched_comparisons": matched_comparisons[:40],
        "matched_worst_losses": matched_by_loss[:24],
        "matched_best_wins": matched_by_win[:24],
        "matched_failure_clusters": matched_failure_clusters[:30],
        "matched_epoch_diagnostics": matched_epoch_diagnostics,
        "matched_comparison_count": len(matched_comparisons),
        "matched_summary": summarize_matched_gnss_gpu_comparisons(matched_comparisons),
        "score_scope_summary": summarize_gnss_gpu_score_scopes(rows),
        "method_comparisons": method_comparisons[:40],
        "method_regressions": method_regressions[:24],
    }


def solution_analysis(
    epochs: list[driving_comparison.SolutionEpoch],
    solver_label: str,
) -> dict[str, Any]:
    status_counts: dict[str, int] = {}
    status_runs: list[dict[str, Any]] = []
    previous_status: str | None = None
    run_start = 0
    for index, epoch in enumerate(epochs):
        status_name, _ = driving_comparison.status_style(solver_label, epoch.status)
        status_counts[status_name] = status_counts.get(status_name, 0) + 1
        if previous_status is None:
            previous_status = status_name
            run_start = index
        elif status_name != previous_status:
            start_epoch = epochs[run_start]
            end_epoch = epochs[index - 1]
            status_runs.append(
                {
                    "status": previous_status,
                    "start_tow": rounded(start_epoch.tow, 3),
                    "end_tow": rounded(end_epoch.tow, 3),
                    "epochs": index - run_start,
                    "duration_s": rounded(end_epoch.tow - start_epoch.tow, 3),
                }
            )
            previous_status = status_name
            run_start = index
    if previous_status is not None:
        start_epoch = epochs[run_start]
        end_epoch = epochs[-1]
        status_runs.append(
            {
                "status": previous_status,
                "start_tow": rounded(start_epoch.tow, 3),
                "end_tow": rounded(end_epoch.tow, 3),
                "epochs": len(epochs) - run_start,
                "duration_s": rounded(end_epoch.tow - start_epoch.tow, 3),
            }
        )

    tows = [float(epoch.tow) for epoch in epochs]
    gaps = [tows[index] - tows[index - 1] for index in range(1, len(tows))]
    cadence = percentile_or_none([gap for gap in gaps if gap > 0.0], 0.5)
    gap_threshold = max((cadence or 0.0) * 1.5, (cadence or 0.0) + 1.0)
    large_gaps = [
        {
            "before_tow": rounded(tows[index - 1], 3),
            "after_tow": rounded(tows[index], 3),
            "gap_s": rounded(tows[index] - tows[index - 1], 3),
        }
        for index in range(1, len(tows))
        if cadence is not None and tows[index] - tows[index - 1] > gap_threshold
    ]
    ratio_values = [
        float(epoch.ratio)
        for epoch in epochs
        if epoch.ratio is not None and math.isfinite(float(epoch.ratio))
    ]
    satellite_values = [float(epoch.num_satellites) for epoch in epochs]
    first_fixed = next(
        (
            epoch
            for epoch in epochs
            if driving_comparison.status_style(solver_label, epoch.status)[0] == "FIXED"
        ),
        None,
    )
    fixed_runs = [run for run in status_runs if run["status"] == "FIXED"]
    longest_fixed_run = max(fixed_runs, key=lambda run: int(run["epochs"]), default=None)
    analysis_points = []
    for epoch in epochs:
        status_name, color = driving_comparison.status_style(solver_label, epoch.status)
        analysis_points.append(
            {
                "tow": rounded(epoch.tow, 3),
                "elapsed_s": rounded(epoch.tow - epochs[0].tow, 3),
                "status": status_name,
                "color": color,
                "satellites": epoch.num_satellites,
                "ratio": rounded(epoch.ratio, 3),
            }
        )
    return {
        "epochs": len(epochs),
        "gps_week": epochs[0].week,
        "start_tow": rounded(epochs[0].tow, 3),
        "end_tow": rounded(epochs[-1].tow, 3),
        "duration_s": rounded(epochs[-1].tow - epochs[0].tow, 3),
        "median_cadence_s": rounded(cadence, 3),
        "large_gaps": large_gaps[:20],
        "large_gap_count": len(large_gaps),
        "status_counts": status_counts,
        "status_runs": status_runs[:40],
        "status_transition_count": max(0, len(status_runs) - 1),
        "fix_rate_pct": rounded(100.0 * status_counts.get("FIXED", 0) / len(epochs), 3),
        "float_rate_pct": rounded(100.0 * status_counts.get("FLOAT", 0) / len(epochs), 3),
        "spp_rate_pct": rounded(100.0 * status_counts.get("SPP", 0) / len(epochs), 3),
        "ttff_s": rounded(first_fixed.tow - epochs[0].tow, 3) if first_fixed else None,
        "longest_fixed_run_epochs": longest_fixed_run["epochs"] if longest_fixed_run else 0,
        "longest_fixed_run_s": longest_fixed_run["duration_s"] if longest_fixed_run else None,
        "mean_satellites": rounded(mean_or_none(satellite_values), 3),
        "min_satellites": int(min(satellite_values)) if satellite_values else None,
        "max_satellites": int(max(satellite_values)) if satellite_values else None,
        "median_ratio": rounded(percentile_or_none(ratio_values, 0.5), 3),
        "p95_ratio": rounded(percentile_or_none(ratio_values, 0.95), 3),
        "max_ratio": rounded(max(ratio_values) if ratio_values else None, 3),
        "points": downsample_points(analysis_points),
    }


def build_solution_payload(name: str, path: Path, root_dir: Path, solver_label: str) -> dict[str, Any]:
    if not path.exists():
        return {
            "name": name,
            "available": False,
            "path": relative_display(path, root_dir),
            "error": "file not found",
        }

    if name == "libgnsspp":
        epochs = driving_comparison.read_libgnss_pos(path)
    elif name == "rtklib":
        epochs = driving_comparison.read_rtklib_pos(path)
    else:
        raise ValueError(f"unsupported solution name: {name}")

    if not epochs:
        return {
            "name": name,
            "available": False,
            "path": relative_display(path, root_dir),
            "error": "no epochs",
        }

    origin = driving_comparison.ReferenceEpoch(
        week=epochs[0].week,
        tow=epochs[0].tow,
        lat_deg=epochs[0].lat_deg,
        lon_deg=epochs[0].lon_deg,
        height_m=epochs[0].height_m,
        ecef=epochs[0].ecef,
    )
    trajectory = driving_comparison.trajectory_enu(epochs, origin)
    points: list[dict[str, Any]] = []
    status_counts: dict[str, int] = {}
    for epoch, enu in zip(epochs, trajectory):
        status_name, color = driving_comparison.status_style(solver_label, epoch.status)
        status_counts[status_name] = status_counts.get(status_name, 0) + 1
        points.append(
            {
                "tow": round(epoch.tow, 3),
                "east_m": round(float(enu[0]), 3),
                "north_m": round(float(enu[1]), 3),
                "up_m": round(float(enu[2]), 3),
                "status": status_name,
                "color": color,
                "satellites": epoch.num_satellites,
            }
        )

    return {
        "name": name,
        "available": True,
        "path": relative_display(path, root_dir),
        "epoch_count": len(epochs),
        "status_counts": status_counts,
        "analysis": solution_analysis(epochs, solver_label),
        "points": downsample_points(points),
    }


def build_overview(args: argparse.Namespace) -> dict[str, Any]:
    root_dir = args.root.resolve()
    odaiba_summary_path = resolve_first_existing(
        args.odaiba_summary,
        root_dir,
        ["output/odaiba_summary.json", "third_party/gnssplusplus/output/odaiba_summary.json"],
    )
    lib_pos_path = resolve_libgnss_pos_path(args.lib_pos, root_dir)
    rtklib_pos_path = resolve_rtklib_pos_path(args.rtklib_pos, root_dir)
    artifact_manifest_path = resolve_first_existing(
        args.artifact_manifest,
        root_dir,
        ["output/artifact_manifest.json", "third_party/gnssplusplus/output/artifact_manifest.json"],
    )
    rcv_status = load_json(args.rcv_status) if args.rcv_status is not None else None

    ppc_summaries: list[dict[str, Any]] = []
    for path in glob_artifact_paths(root_dir, args.ppc_summary_glob):
        payload = load_json(path)
        if payload is None:
            continue
        payload = dict(payload)
        payload["_path"] = relative_display(path, root_dir)
        payload["runtime_status"] = classify_realtime_status(payload.get("realtime_factor"))
        payload["quality_status"] = classify_accuracy_status(payload.get("p95_h_m"))
        commercial_receiver = normalize_commercial_receiver(root_dir, payload.get("commercial_receiver"))
        commercial_delta = payload.get("delta_vs_commercial_receiver")
        if not isinstance(commercial_delta, dict):
            commercial_delta = None
        if commercial_receiver is not None:
            payload["commercial_receiver"] = commercial_receiver
            payload["commercial_comparison_status"] = classify_comparison_status(
                commercial_delta.get("median_h_m") if commercial_delta else None,
                commercial_delta.get("p95_h_m") if commercial_delta else None,
                commercial_delta.get("max_h_m") if commercial_delta else None,
                commercial_delta.get("p95_abs_up_m") if commercial_delta else None,
            )
        ppc_summaries.append(payload)

    live_summaries: list[dict[str, Any]] = []
    for path in glob_artifact_paths(root_dir, args.live_summary_glob):
        payload = load_json(path)
        if payload is None:
            continue
        metrics = payload.get("metrics")
        if not isinstance(metrics, dict):
            continue
        live_summaries.append(
            {
                "_path": relative_display(path, root_dir),
                "execution_mode": payload.get("execution_mode"),
                "exit_code": payload.get("exit_code"),
                "termination": metrics.get("termination"),
                "aligned_epochs": metrics.get("aligned_epochs"),
                "written_solutions": metrics.get("written_solutions"),
                "fixed_solutions": metrics.get("fixed_solutions"),
                "realtime_factor": metrics.get("realtime_factor"),
                "effective_epoch_rate_hz": metrics.get("effective_epoch_rate_hz"),
                "rover_decoder_errors": metrics.get("rover_decoder_errors"),
                "base_decoder_errors": metrics.get("base_decoder_errors"),
                "runtime_status": classify_realtime_status(metrics.get("realtime_factor")),
            }
        )

    visibility_summaries: list[dict[str, Any]] = []
    for path in glob_artifact_paths(root_dir, args.visibility_summary_glob):
        payload = load_json(path)
        if payload is None:
            continue
        csv_path = None
        png_path = None
        csv_value = payload.get("csv")
        if isinstance(csv_value, str):
            try:
                resolved_csv = resolve_under_root(root_dir, csv_value)
                csv_path = relative_display(resolved_csv, root_dir)
                sibling_png = resolved_csv.with_suffix(".png")
                if sibling_png.exists():
                    png_path = relative_display(sibling_png, root_dir)
            except ValueError:
                csv_path = csv_value
        png_value = payload.get("png")
        if isinstance(png_value, str):
            try:
                png_path = relative_display(resolve_under_root(root_dir, png_value), root_dir)
            except ValueError:
                png_path = png_value
        visibility_summaries.append(
            {
                "_path": relative_display(path, root_dir),
                "csv_path": csv_path,
                "png_path": png_path,
                "epochs_processed": payload.get("epochs_processed"),
                "epochs_with_rows": payload.get("epochs_with_rows"),
                "rows_written": payload.get("rows_written"),
                "unique_satellites": payload.get("unique_satellites"),
                "mean_satellites_per_epoch": payload.get("mean_satellites_per_epoch"),
                "max_satellites_per_epoch": payload.get("max_satellites_per_epoch"),
                "mean_elevation_deg": payload.get("mean_elevation_deg"),
                "mean_snr_dbhz": payload.get("mean_snr_dbhz"),
            }
        )

    moving_base_summaries: list[dict[str, Any]] = []
    for path in glob_artifact_paths(root_dir, args.moving_base_summary_glob):
        payload = load_json(path)
        if payload is None:
            continue
        commercial_receiver = normalize_commercial_receiver(root_dir, payload.get("commercial_receiver"))
        commercial_delta = payload.get("libgnss_vs_commercial_receiver")
        if not isinstance(commercial_delta, dict):
            commercial_delta = None
        moving_base_summaries.append(
            {
                "_path": relative_display(path, root_dir),
                "summary_path": relative_display(path, root_dir),
                "execution_mode": payload.get("execution_mode"),
                "solver": payload.get("solver"),
                "matched_epochs": payload.get("matched_epochs"),
                "valid_epochs": payload.get("valid_epochs"),
                "fix_rate_pct": payload.get("fix_rate_pct"),
                "median_baseline_error_m": payload.get("median_baseline_error_m"),
                "p95_baseline_error_m": payload.get("p95_baseline_error_m"),
                "p95_heading_error_deg": payload.get("p95_heading_error_deg"),
                "termination": payload.get("termination"),
                "realtime_factor": payload.get("realtime_factor"),
                "effective_epoch_rate_hz": payload.get("effective_epoch_rate_hz"),
                "solution_pos": normalize_artifact_path(root_dir, payload.get("solution_pos")),
                "matched_csv": normalize_artifact_path(root_dir, payload.get("matched_csv")),
                "prepare_summary_json": normalize_artifact_path(root_dir, payload.get("prepare_summary_json")),
                "products_summary_json": normalize_artifact_path(root_dir, payload.get("products_summary_json")),
                "plot_png": normalize_artifact_path(root_dir, payload.get("plot_png")),
                "commercial_receiver_csv": normalize_artifact_path(
                    root_dir,
                    payload.get("commercial_receiver_csv"),
                ),
                "commercial_receiver_matched_csv": normalize_artifact_path(
                    root_dir,
                    payload.get("commercial_receiver_matched_csv"),
                ),
                "commercial_receiver": commercial_receiver,
                "libgnss_vs_commercial_receiver": commercial_delta,
                "commercial_comparison_status": classify_comparison_status(
                    commercial_delta.get("median_baseline_error_m_delta") if commercial_delta else None,
                    commercial_delta.get("p95_baseline_error_m_delta") if commercial_delta else None,
                    commercial_delta.get("max_baseline_error_m_delta") if commercial_delta else None,
                    commercial_delta.get("p95_heading_error_deg_delta") if commercial_delta else None,
                ),
                "signoff_profile": payload.get("signoff_profile"),
                "nav_rinex": normalize_artifact_path(root_dir, payload.get("nav_rinex")),
                "input": normalize_artifact_path(root_dir, payload.get("input")),
                "input_url": normalize_artifact_path(root_dir, payload.get("input_url")),
                "prepare_summary": payload.get("prepare_summary"),
                "products_summary": payload.get("products_summary"),
                "runtime_status": classify_realtime_status(payload.get("realtime_factor")),
                "quality_status": classify_baseline_status(payload.get("p95_baseline_error_m")),
            }
        )

    ppp_products_summaries: list[dict[str, Any]] = []
    for path in glob_artifact_paths(root_dir, args.ppp_products_summary_glob):
        payload = load_json(path)
        if payload is None:
            continue
        p95_error = payload.get("p95_position_error_m")
        if not isinstance(p95_error, (int, float)):
            p95_error = payload.get("max_position_error_m")
        ppp_products_summaries.append(
            {
                "_path": relative_display(path, root_dir),
                "summary_path": relative_display(path, root_dir),
                "dataset": payload.get("dataset"),
                "run_dir": normalize_artifact_path(root_dir, payload.get("run_dir")),
                "reference_csv": normalize_artifact_path(root_dir, payload.get("reference_csv")),
                "profile": payload.get("products_signoff_profile"),
                "product_presets": payload.get("product_presets"),
                "fetched_product_date": payload.get("fetched_product_date"),
                "ppp_solution_rate_pct": payload.get("ppp_solution_rate_pct"),
                "ppp_converged": payload.get("ppp_converged"),
                "ppp_convergence_time_s": payload.get("ppp_convergence_time_s"),
                "mean_position_error_m": payload.get("mean_position_error_m"),
                "p95_position_error_m": payload.get("p95_position_error_m"),
                "max_position_error_m": payload.get("max_position_error_m"),
                "ionex_corrections": payload.get("ionex_corrections"),
                "dcb_corrections": payload.get("dcb_corrections"),
                "solution_pos": normalize_artifact_path(root_dir, payload.get("solution_pos")),
                "sp3": normalize_artifact_path(root_dir, payload.get("sp3")),
                "clk": normalize_artifact_path(root_dir, payload.get("clk")),
                "ionex": normalize_artifact_path(root_dir, payload.get("ionex")),
                "dcb": normalize_artifact_path(root_dir, payload.get("dcb")),
                "malib_solution_pos": normalize_artifact_path(root_dir, payload.get("malib_solution_pos")),
                "comparison_target": payload.get("comparison_target"),
                "comparison_status": payload.get("comparison_status"),
                "comparison_csv": normalize_artifact_path(root_dir, payload.get("comparison_csv")),
                "comparison_png": normalize_artifact_path(root_dir, payload.get("comparison_png")),
                "common_epoch_pairs": payload.get("common_epoch_pairs"),
                "libgnss_minus_malib_mean_error_m": payload.get("libgnss_minus_malib_mean_error_m"),
                "libgnss_minus_malib_p95_error_m": payload.get("libgnss_minus_malib_p95_error_m"),
                "libgnss_minus_malib_max_error_m": payload.get("libgnss_minus_malib_max_error_m"),
                "quality_status": classify_ppp_products_status(
                    payload.get("ppp_converged"),
                    p95_error,
                    payload.get("ppp_solution_rate_pct"),
                ),
            }
        )

    dd_residual_summaries: list[dict[str, Any]] = []
    for path in glob_artifact_paths(root_dir, args.dd_residual_summary_glob):
        payload = load_json(path)
        if payload is None:
            continue
        coverage = payload.get("coverage")
        if not isinstance(coverage, dict):
            coverage = {}
        by_kind = payload.get("by_kind")
        if not isinstance(by_kind, dict):
            by_kind = {}
        phase = by_kind.get("phase")
        if not isinstance(phase, dict):
            phase = {}
        code = by_kind.get("code")
        if not isinstance(code, dict):
            code = {}
        overall = payload.get("overall")
        if not isinstance(overall, dict):
            overall = {}
        dd_residual_summaries.append(
            {
                "_path": relative_display(path, root_dir),
                "summary_path": relative_display(path, root_dir),
                "csv_path": inferred_sibling_artifact(root_dir, path, ".csv"),
                "html_report": inferred_sibling_artifact(root_dir, path, ".html"),
                "top_pairs_csv": inferred_sibling_artifact(root_dir, path, "_top_pairs.csv"),
                "rows": overall.get("rows", payload.get("rows")),
                "epochs": coverage.get("epochs", overall.get("epochs")),
                "satellite_pairs": coverage.get("satellite_pairs"),
                "pair_frequency_kind_tracks": coverage.get("pair_frequency_kind_tracks"),
                "phase_p95_abs_residual_m": phase.get("p95_abs_residual_m"),
                "code_p95_abs_residual_m": code.get("p95_abs_residual_m"),
                "phase_p95_abs_normalized_residual": phase.get("p95_abs_normalized_residual"),
                "code_p95_abs_normalized_residual": code.get("p95_abs_normalized_residual"),
                "max_abs_residual_m": overall.get("max_abs_residual_m"),
                "max_abs_normalized_residual": overall.get("max_abs_normalized_residual"),
                "suppressed_rows": overall.get("suppressed_rows"),
                "quality_status": classify_dd_residual_status(
                    phase.get("p95_abs_normalized_residual"),
                    code.get("p95_abs_normalized_residual"),
                ),
            }
        )

    artifact_manifest_payload = load_json(artifact_manifest_path)
    artifact_manifest = []
    if artifact_manifest_payload is not None:
        bundles = artifact_manifest_payload.get("bundles")
        if isinstance(bundles, list):
            artifact_manifest = bundles

    return {
        "title": "libgnss++ web",
        "root": str(root_dir),
        "docs_url": args.docs_url,
        "artifacts": {
            "odaiba_summary": relative_display(odaiba_summary_path, root_dir),
            "lib_pos": relative_display(lib_pos_path, root_dir),
            "rtklib_pos": relative_display(rtklib_pos_path, root_dir),
            "rcv_status": relative_display(args.rcv_status, root_dir) if args.rcv_status else None,
            "artifact_manifest": relative_display(artifact_manifest_path, root_dir) if artifact_manifest else None,
        },
        "odaiba_summary": load_json(odaiba_summary_path),
        "ppc_summaries": ppc_summaries,
        "live_summaries": live_summaries,
        "visibility_summaries": visibility_summaries,
        "moving_base_summaries": moving_base_summaries,
        "ppp_products_summaries": ppp_products_summaries,
        "dd_residual_summaries": dd_residual_summaries,
        "gnss_gpu_summaries": load_gnss_gpu_summaries(
            root_dir,
            args.gnss_gpu_summary_glob,
            args.gnss_gpu_runs_glob,
        ),
        "artifact_manifest": artifact_manifest,
        "receiver_status": rcv_status,
    }


def render_html() -> str:
    return """<!doctype html>
<html lang="en">
<head>
  <meta charset="utf-8">
  <meta name="viewport" content="width=device-width, initial-scale=1">
  <title>libgnss++ web</title>
  <style>
    :root {
      --bg: #f4f1e8;
      --paper: #fffdf8;
      --ink: #1f2937;
      --muted: #6b7280;
      --line: #d6d0c4;
      --accent: #0f766e;
    }
    * { box-sizing: border-box; }
    body {
      margin: 0;
      font-family: "IBM Plex Sans", "Segoe UI", sans-serif;
      color: var(--ink);
      background:
        radial-gradient(circle at top left, rgba(15,118,110,0.08), transparent 35%),
        linear-gradient(180deg, #f8f5ec 0%, var(--bg) 100%);
    }
    main { max-width: 1280px; margin: 0 auto; padding: 28px 20px 40px; }
    h1, h2, h3 { margin: 0; }
    p { margin: 0; color: var(--muted); }
    .hero { display: grid; gap: 10px; margin-bottom: 24px; }
    .hero-top { display: flex; justify-content: space-between; gap: 16px; align-items: start; flex-wrap: wrap; }
    .hero h1 { font-size: 2rem; letter-spacing: -0.03em; }
    .hero-actions { display: flex; gap: 10px; flex-wrap: wrap; }
    .hero-link {
      display: inline-flex;
      align-items: center;
      padding: 9px 14px;
      border-radius: 999px;
      border: 1px solid var(--line);
      background: rgba(255,255,255,0.82);
      color: var(--ink);
      text-decoration: none;
      font-size: 0.9rem;
      font-weight: 600;
    }
    .hero-link:hover { border-color: var(--accent); color: var(--accent); }
    .chips { display: flex; flex-wrap: wrap; gap: 10px; }
    .chip {
      border: 1px solid var(--line);
      background: rgba(255,255,255,0.75);
      padding: 6px 10px;
      border-radius: 999px;
      font-size: 0.9rem;
      color: var(--muted);
    }
    .grid { display: grid; gap: 18px; }
    .grid.two { grid-template-columns: repeat(auto-fit, minmax(320px, 1fr)); }
    .card {
      background: rgba(255,255,255,0.85);
      border: 1px solid var(--line);
      border-radius: 20px;
      padding: 18px;
      backdrop-filter: blur(8px);
      box-shadow: 0 16px 30px rgba(31,41,55,0.06);
    }
    .metrics {
      display: grid;
      grid-template-columns: repeat(auto-fit, minmax(140px, 1fr));
      gap: 12px;
      margin-top: 14px;
    }
    .metric {
      background: var(--paper);
      border: 1px solid var(--line);
      border-radius: 14px;
      padding: 12px;
      min-height: 78px;
    }
    .metric .k { font-size: 0.78rem; color: var(--muted); margin-bottom: 6px; }
    .metric .v { font-size: 1.05rem; font-weight: 700; }
    .plot-wrap { display: grid; gap: 12px; }
    .plot-card canvas {
      width: 100%;
      height: 360px;
      border-radius: 14px;
      background: #fff;
      border: 1px solid var(--line);
    }
    .artifact-frame {
      width: 100%;
      min-height: 820px;
      border: 1px solid var(--line);
      border-radius: 14px;
      background: #fff;
    }
    .legend { display: flex; flex-wrap: wrap; gap: 10px; margin-top: 10px; }
    .legend-item { display: inline-flex; align-items: center; gap: 8px; font-size: 0.85rem; color: var(--muted); }
    .swatch { width: 12px; height: 12px; border-radius: 999px; border: 1px solid rgba(0,0,0,0.1); }
    .badge {
      display: inline-flex;
      align-items: center;
      padding: 4px 8px;
      border-radius: 999px;
      font-size: 0.78rem;
      font-weight: 700;
      letter-spacing: 0.02em;
      border: 1px solid transparent;
      white-space: nowrap;
    }
    .badge.realtime, .badge.excellent {
      background: rgba(46, 204, 113, 0.14);
      color: #0f7a43;
      border-color: rgba(46, 204, 113, 0.28);
    }
    .badge.near-realtime, .badge.good, .badge.close, .badge.available {
      background: rgba(243, 156, 18, 0.16);
      color: #9a5f00;
      border-color: rgba(243, 156, 18, 0.26);
    }
    .badge.offline, .badge.rough, .badge.poor, .badge.worse {
      background: rgba(231, 76, 60, 0.12);
      color: #b03a2e;
      border-color: rgba(231, 76, 60, 0.24);
    }
    .badge.better {
      background: rgba(46, 204, 113, 0.14);
      color: #0f7a43;
      border-color: rgba(46, 204, 113, 0.28);
    }
    .badge.na {
      background: rgba(107, 114, 128, 0.12);
      color: #4b5563;
      border-color: rgba(107, 114, 128, 0.18);
    }
    .status-pre {
      margin-top: 12px;
      padding: 12px;
      border-radius: 14px;
      background: #111827;
      color: #f3f4f6;
      overflow: auto;
      font-size: 0.85rem;
    }
    .section-title { margin-bottom: 12px; display: flex; justify-content: space-between; gap: 12px; align-items: baseline; }
    .tiny { font-size: 0.85rem; color: var(--muted); }
    table {
      width: 100%;
      border-collapse: collapse;
      margin-top: 10px;
      font-size: 0.92rem;
    }
    th, td {
      text-align: left;
      padding: 10px 8px;
      border-bottom: 1px solid var(--line);
    }
    th { color: var(--muted); font-weight: 600; }
  </style>
</head>
<body>
  <main>
    <section class="hero">
      <div class="hero-top">
        <div>
          <h1>libgnss++ local web UI</h1>
          <p>Benchmark snapshot, live sign-offs, 2D trajectories, PPC summaries, and receiver status from the existing non-GUI stack.</p>
        </div>
        <div class="hero-actions">
          <a class="hero-link" id="docs-link" href="https://rsasaki0109.github.io/gnssplusplus-library/" target="_blank" rel="noreferrer">Open docs site</a>
        </div>
      </div>
      <div class="chips" id="artifact-chips"></div>
    </section>

    <section class="grid two">
      <div class="card">
        <div class="section-title">
          <h2>Odaiba snapshot</h2>
          <span class="tiny" id="odaiba-source"></span>
        </div>
        <div class="metrics" id="odaiba-metrics"></div>
      </div>
      <div class="card">
        <div class="section-title">
          <h2>Receiver status</h2>
          <span class="tiny">auto-refresh 2s</span>
        </div>
        <div class="metrics" id="receiver-metrics"></div>
        <pre class="status-pre" id="receiver-json">No receiver status configured.</pre>
      </div>
    </section>

    <section class="card" style="margin-top: 18px;">
      <div class="section-title">
        <h2>2D trajectories</h2>
        <span class="tiny">status-colored ENU points from .pos files</span>
      </div>
      <div class="grid two plot-wrap">
        <div class="plot-card">
          <h3>libgnss++</h3>
          <canvas id="lib-canvas" width="600" height="360"></canvas>
        </div>
        <div class="plot-card">
          <h3>RTKLIB</h3>
          <canvas id="rtk-canvas" width="600" height="360"></canvas>
        </div>
      </div>
      <div class="legend" id="status-legend"></div>
    </section>

    <section class="card" style="margin-top: 18px;">
      <div class="section-title">
        <h2>RTKLIB analysis</h2>
        <span class="tiny">status, ratio, satellites, gaps, and FIX continuity from RTKLIB .pos</span>
      </div>
      <div class="metrics" id="rtklib-analysis-metrics"></div>
      <p class="tiny" id="rtklib-analysis-source" style="margin-top:10px;"></p>
      <div class="grid two plot-wrap" style="margin-top: 12px;">
        <div class="plot-card">
          <h3>Status timeline</h3>
          <canvas id="rtklib-status-canvas" width="600" height="260"></canvas>
        </div>
        <div class="plot-card">
          <h3>Ratio and satellites</h3>
          <canvas id="rtklib-ratio-canvas" width="600" height="260"></canvas>
        </div>
      </div>
    </section>

    <section class="card" style="margin-top: 18px;">
      <div class="section-title">
        <h2>GNSS GPU comparison</h2>
        <span class="tiny">auto-discovered from experiments/results/*_summary.csv and *_runs.csv</span>
      </div>
      <div class="metrics" id="gnss-gpu-metrics"></div>
      <p class="tiny" id="gnss-gpu-comparison-note" style="margin-top:10px;"></p>
      <table id="gnss-gpu-score-scope-table">
        <thead>
          <tr>
            <th>Score scope trap</th>
            <th>Segment score</th>
            <th>Honest score</th>
            <th>Gap</th>
            <th>Segment</th>
            <th>Method</th>
          </tr>
        </thead>
        <tbody></tbody>
      </table>
      <table id="gnss-gpu-method-compare-table">
        <thead>
          <tr>
            <th>Method comparison</th>
            <th>Hybrid</th>
            <th>RTKDiag</th>
            <th>Δ score</th>
            <th>Δ P95</th>
            <th>Status</th>
          </tr>
        </thead>
        <tbody></tbody>
      </table>
      <table id="gnss-gpu-cluster-table">
        <thead>
          <tr>
            <th>Failure cluster</th>
            <th>Losses</th>
            <th>Avg Δ score</th>
            <th>Worst Δ score</th>
            <th>Avg Δ P95</th>
            <th>Worst match</th>
          </tr>
        </thead>
        <tbody></tbody>
      </table>
      <table id="gnss-gpu-loss-table">
        <thead>
          <tr>
            <th>Worst match</th>
            <th>GNSS GPU best</th>
            <th>libgnss++ best</th>
            <th>Δ score</th>
            <th>Δ P95/RMS</th>
            <th>Candidates</th>
            <th>Status</th>
          </tr>
        </thead>
        <tbody></tbody>
      </table>
      <div class="grid two plot-wrap" style="margin-top: 12px;">
        <div class="plot-card">
          <h3>Worst segment epoch error</h3>
          <canvas id="gnss-gpu-epoch-error-canvas" width="600" height="320"></canvas>
        </div>
        <div class="plot-card">
          <h3>Worst segment GNSS cues</h3>
          <canvas id="gnss-gpu-epoch-cue-canvas" width="600" height="320"></canvas>
        </div>
        <div class="plot-card">
          <h3>Worst segment stage error</h3>
          <canvas id="gnss-gpu-stage-error-canvas" width="600" height="320"></canvas>
        </div>
      </div>
      <table id="gnss-gpu-epoch-table">
        <thead>
          <tr>
            <th>Epoch diagnostics</th>
            <th>Score / errors</th>
            <th>PPC / 3m fail</th>
            <th>Stage diagnosis</th>
            <th>Signals</th>
            <th>Top states</th>
          </tr>
        </thead>
        <tbody></tbody>
      </table>
      <table id="gnss-gpu-initial-table">
        <thead>
          <tr>
            <th>Initial state</th>
            <th>Start error</th>
            <th>Epoch 0 ENU</th>
            <th>Spread / ESS</th>
            <th>Epoch 0 mode</th>
            <th>Diagnosis</th>
          </tr>
        </thead>
        <tbody></tbody>
      </table>
      <table id="gnss-gpu-epoch-compare-table">
        <thead>
          <tr>
            <th>Dataset</th>
            <th>Best diagnostic</th>
            <th>Worst diagnostic</th>
            <th>Score / P95 gap</th>
            <th>Start gap</th>
            <th>Dominant stage gap</th>
          </tr>
        </thead>
        <tbody></tbody>
      </table>
      <table id="gnss-gpu-matched-table">
        <thead>
          <tr>
            <th>Best match</th>
            <th>GNSS GPU best</th>
            <th>libgnss++ best</th>
            <th>Δ score</th>
            <th>Δ P95/RMS</th>
            <th>Candidates</th>
            <th>Status</th>
          </tr>
        </thead>
        <tbody></tbody>
      </table>
      <div class="grid two plot-wrap" style="margin-top: 12px;">
        <div class="plot-card">
          <h3>Top score rows</h3>
          <canvas id="gnss-gpu-score-canvas" width="600" height="320"></canvas>
        </div>
        <div class="plot-card">
          <h3>Error rows</h3>
          <canvas id="gnss-gpu-error-canvas" width="600" height="320"></canvas>
        </div>
      </div>
      <table id="gnss-gpu-table">
        <thead>
          <tr>
            <th>Run</th>
            <th>Family</th>
            <th>Score</th>
            <th>P50/P95/RMS</th>
            <th>Pass distance</th>
            <th>Epochs</th>
            <th>Time</th>
            <th>Status</th>
          </tr>
        </thead>
        <tbody></tbody>
      </table>
    </section>

    <section class="card" style="margin-top: 18px;">
      <div class="section-title">
        <h2>Live sign-offs</h2>
        <span class="tiny">auto-discovered from output/live*_summary.json</span>
      </div>
      <table id="live-table">
        <thead>
          <tr>
            <th>Path</th>
            <th>Termination</th>
            <th>Written</th>
            <th>Fixed</th>
            <th>Realtime</th>
            <th>Epoch rate</th>
            <th>Decoder errors</th>
          </tr>
        </thead>
        <tbody></tbody>
      </table>
    </section>

    <section class="card" style="margin-top: 18px;">
      <div class="section-title">
        <h2>PPC summaries</h2>
        <span class="tiny">auto-discovered from output/ppc_*_summary.json</span>
      </div>
      <table id="ppc-table">
        <thead>
          <tr>
            <th>Path</th>
            <th>Matched</th>
            <th>Positioning</th>
            <th>Fix rate</th>
            <th>3D50/ref</th>
            <th>Median H</th>
            <th>P95 H</th>
            <th>Receiver</th>
            <th>Δ receiver</th>
            <th>Quality</th>
            <th>Realtime</th>
            <th>Wall</th>
            <th>Rate</th>
          </tr>
        </thead>
        <tbody></tbody>
      </table>
    </section>

    <section class="card" style="margin-top: 18px;">
      <div class="section-title">
        <h2>Moving-base sign-offs</h2>
        <span class="tiny">auto-discovered from output/*moving_base_summary.json</span>
      </div>
      <div class="metrics" id="moving-base-metrics"></div>
      <p class="tiny" id="moving-base-provenance" style="margin-top: 10px;"></p>
      <div class="grid two plot-wrap">
        <div>
          <table id="moving-base-table">
            <thead>
              <tr>
                <th>Path</th>
                <th>Matched</th>
                <th>Fix rate</th>
                <th>P95 baseline</th>
                <th>P95 heading</th>
                <th>Receiver</th>
                <th>Δ receiver</th>
                <th>Quality</th>
                <th>Realtime</th>
                <th>Rate</th>
                <th>Termination</th>
              </tr>
            </thead>
            <tbody></tbody>
          </table>
        </div>
        <div class="plot-card">
          <h3>Moving-base view</h3>
          <img id="moving-base-image" alt="Moving-base artifact" style="display:none; width:100%; border:1px solid var(--line); border-radius:10px;" />
        </div>
        <div class="plot-card">
          <h3>Moving-base history</h3>
          <canvas id="moving-base-history-canvas" width="600" height="360"></canvas>
        </div>
        <div class="plot-card">
          <h3>Heading history</h3>
          <canvas id="moving-base-heading-canvas" width="600" height="360"></canvas>
        </div>
      </div>
    </section>

    <section class="card" style="margin-top: 18px;">
      <div class="section-title">
        <h2>PPP products sign-offs</h2>
        <span class="tiny">auto-discovered from output/*ppp*_products*_summary.json</span>
      </div>
      <table id="ppp-products-table">
        <thead>
          <tr>
            <th>Path</th>
            <th>Profile</th>
            <th>Date</th>
            <th>PPP rate</th>
            <th>Convergence</th>
            <th>Mean err</th>
            <th>P95 / max</th>
            <th>Corrections</th>
            <th>Compare</th>
            <th>Status</th>
          </tr>
        </thead>
        <tbody></tbody>
      </table>
    </section>

    <section class="card" style="margin-top: 18px;">
      <div class="section-title">
        <h2>DD residual diagnostics</h2>
        <span class="tiny">auto-discovered from output/*dd_residuals*_summary.json</span>
      </div>
      <div class="metrics" id="dd-residual-metrics"></div>
      <div class="plot-card" style="margin-top: 14px;">
        <h3>DD residual report preview</h3>
        <iframe id="dd-residual-frame" class="artifact-frame" title="DD residual report preview"></iframe>
        <p class="tiny" id="dd-residual-empty" style="margin-top: 10px;"></p>
      </div>
      <table id="dd-residual-table">
        <thead>
          <tr>
            <th>Path</th>
            <th>Coverage</th>
            <th>Phase p95</th>
            <th>Code p95</th>
            <th>Max</th>
            <th>Suppressed</th>
            <th>Status</th>
            <th>Artifacts</th>
          </tr>
        </thead>
        <tbody></tbody>
      </table>
    </section>

    <section class="card" style="margin-top: 18px;">
      <div class="section-title">
        <h2>Visibility summaries</h2>
        <span class="tiny">summary rows plus a quick polar view from the first CSV artifact</span>
      </div>
      <div class="grid two plot-wrap">
        <div class="plot-card">
          <h3>Visibility view</h3>
          <img id="visibility-image" alt="Visibility artifact" style="display:none; width:100%; border:1px solid var(--line); border-radius:10px; margin-bottom:12px;" />
          <canvas id="visibility-canvas" width="600" height="360"></canvas>
        </div>
        <div>
          <table id="visibility-table">
            <thead>
              <tr>
                <th>Path</th>
                <th>Epochs</th>
                <th>Rows</th>
                <th>Unique sats</th>
                <th>Mean sats</th>
                <th>Mean elev</th>
                <th>Mean SNR</th>
              </tr>
            </thead>
            <tbody></tbody>
          </table>
        </div>
      </div>
    </section>

    <section class="card" style="margin-top: 18px;">
      <div class="section-title">
        <h2>Artifact bundles</h2>
        <span class="tiny"><a id="artifact-manifest-link" href="#" target="_blank" rel="noreferrer">manifest</a></span>
      </div>
      <table id="artifact-manifest-table">
        <thead>
          <tr>
            <th>Category</th>
            <th>Label</th>
            <th>Status</th>
            <th>Summary</th>
            <th>Artifacts</th>
          </tr>
        </thead>
        <tbody></tbody>
      </table>
    </section>
  </main>
  <script>
    const STATUS_COLORS = {"FIXED":"#2ecc71","FLOAT":"#f39c12","DGPS":"#3498db","SPP":"#e74c3c"};

    async function fetchJson(path) {
      const response = await fetch(path);
      if (!response.ok) {
        const payload = await response.json().catch(() => ({error: response.statusText}));
        throw new Error(payload.error || response.statusText);
      }
      return response.json();
    }

    function renderMetricGrid(target, entries) {
      target.innerHTML = "";
      for (const [key, value] of entries) {
        const el = document.createElement("div");
        el.className = "metric";
        el.innerHTML = `<div class="k">${key}</div><div class="v">${value}</div>`;
        target.appendChild(el);
      }
    }

    function drawTrajectory(canvas, payload) {
      const ctx = canvas.getContext("2d");
      ctx.clearRect(0, 0, canvas.width, canvas.height);
      ctx.fillStyle = "#fff";
      ctx.fillRect(0, 0, canvas.width, canvas.height);

      if (!payload.available || !payload.points.length) {
        ctx.fillStyle = "#6b7280";
        ctx.font = "16px IBM Plex Sans, sans-serif";
        ctx.fillText(payload.error || "No data", 24, 40);
        return;
      }

      const pad = 28;
      const xs = payload.points.map((point) => point.east_m);
      const ys = payload.points.map((point) => point.north_m);
      const minX = Math.min(...xs);
      const maxX = Math.max(...xs);
      const minY = Math.min(...ys);
      const maxY = Math.max(...ys);
      const spanX = Math.max(maxX - minX, 1);
      const spanY = Math.max(maxY - minY, 1);
      const scale = Math.min((canvas.width - pad * 2) / spanX, (canvas.height - pad * 2) / spanY);

      const mapX = (value) => pad + (value - minX) * scale;
      const mapY = (value) => canvas.height - pad - (value - minY) * scale;

      ctx.strokeStyle = "#d1d5db";
      ctx.lineWidth = 1.5;
      ctx.beginPath();
      payload.points.forEach((point, index) => {
        const x = mapX(point.east_m);
        const y = mapY(point.north_m);
        if (index === 0) ctx.moveTo(x, y);
        else ctx.lineTo(x, y);
      });
      ctx.stroke();

      payload.points.forEach((point) => {
        ctx.fillStyle = point.color || STATUS_COLORS[point.status] || "#64748b";
        ctx.beginPath();
        ctx.arc(mapX(point.east_m), mapY(point.north_m), 2.4, 0, Math.PI * 2);
        ctx.fill();
      });

      ctx.fillStyle = "#6b7280";
      ctx.font = "12px IBM Plex Sans, sans-serif";
      ctx.fillText(`epochs: ${payload.epoch_count}`, 14, 18);
      ctx.fillText(payload.path, 14, canvas.height - 10);
    }

    function renderLegend() {
      const legend = document.getElementById("status-legend");
      legend.innerHTML = "";
      for (const [name, color] of Object.entries(STATUS_COLORS)) {
        const item = document.createElement("div");
        item.className = "legend-item";
        item.innerHTML = `<span class="swatch" style="background:${color}"></span>${name}`;
        legend.appendChild(item);
      }
    }

    function drawVisibility(canvas, payload) {
      const ctx = canvas.getContext("2d");
      ctx.clearRect(0, 0, canvas.width, canvas.height);
      ctx.fillStyle = "#fff";
      ctx.fillRect(0, 0, canvas.width, canvas.height);

      if (!payload.available || !payload.rows.length) {
        ctx.fillStyle = "#6b7280";
        ctx.font = "16px IBM Plex Sans, sans-serif";
        ctx.fillText(payload.error || "No visibility data", 24, 40);
        return;
      }

      const centerX = canvas.width / 2;
      const centerY = canvas.height / 2;
      const radius = Math.min(canvas.width, canvas.height) * 0.38;

      ctx.strokeStyle = "#d1d5db";
      ctx.lineWidth = 1;
      for (const frac of [0.25, 0.5, 0.75, 1.0]) {
        ctx.beginPath();
        ctx.arc(centerX, centerY, radius * frac, 0, Math.PI * 2);
        ctx.stroke();
      }
      ctx.beginPath();
      ctx.moveTo(centerX, centerY - radius);
      ctx.lineTo(centerX, centerY + radius);
      ctx.moveTo(centerX - radius, centerY);
      ctx.lineTo(centerX + radius, centerY);
      ctx.stroke();

      ctx.fillStyle = "#6b7280";
      ctx.font = "12px IBM Plex Sans, sans-serif";
      ctx.fillText("N", centerX - 4, centerY - radius - 8);
      ctx.fillText("E", centerX + radius + 8, centerY + 4);
      ctx.fillText("S", centerX - 4, centerY + radius + 16);
      ctx.fillText("W", centerX - radius - 16, centerY + 4);

      for (const row of payload.rows) {
        const az = row.azimuth_deg * Math.PI / 180.0;
        const r = radius * (90.0 - row.elevation_deg) / 90.0;
        const x = centerX + r * Math.sin(az);
        const y = centerY - r * Math.cos(az);
        const snr = row.snr_dbhz ?? 0;
        const alpha = Math.max(0.35, Math.min(1.0, snr / 50.0));
        ctx.fillStyle = `rgba(15, 118, 110, ${alpha})`;
        ctx.beginPath();
        ctx.arc(x, y, 3.2, 0, Math.PI * 2);
        ctx.fill();
      }

      ctx.fillStyle = "#6b7280";
      ctx.font = "12px IBM Plex Sans, sans-serif";
      ctx.fillText(`rows: ${payload.rows.length}`, 14, 18);
      ctx.fillText(payload.path || "", 14, canvas.height - 10);
    }

    function drawMovingBaseHistory(canvas, payload) {
      const ctx = canvas.getContext("2d");
      ctx.clearRect(0, 0, canvas.width, canvas.height);
      ctx.fillStyle = "#fff";
      ctx.fillRect(0, 0, canvas.width, canvas.height);

      if (!payload.available || !payload.rows.length) {
        ctx.fillStyle = "#6b7280";
        ctx.font = "16px IBM Plex Sans, sans-serif";
        ctx.fillText(payload.error || "No moving-base matches", 24, 40);
        return;
      }

      const padLeft = 52;
      const padRight = 18;
      const padTop = 20;
      const padBottom = 30;
      const plotWidth = canvas.width - padLeft - padRight;
      const plotHeight = canvas.height - padTop - padBottom;

      const baselineValues = payload.rows.map((row) => row.baseline_error_m);
      const headingValues = payload.rows
        .map((row) => row.heading_error_deg)
        .filter((value) => value !== null && value !== undefined);
      const maxBaseline = Math.max(...baselineValues, 0.1);
      const maxHeading = headingValues.length ? Math.max(...headingValues, 1.0) : 1.0;
      const maxValue = Math.max(maxBaseline, maxHeading);

      const mapX = (index) =>
        padLeft + (plotWidth * index) / Math.max(payload.rows.length - 1, 1);
      const mapY = (value) =>
        padTop + plotHeight - (plotHeight * value) / Math.max(maxValue, 1e-6);

      ctx.strokeStyle = "#d1d5db";
      ctx.lineWidth = 1;
      for (let i = 0; i <= 4; i += 1) {
        const y = padTop + (plotHeight * i) / 4;
        ctx.beginPath();
        ctx.moveTo(padLeft, y);
        ctx.lineTo(canvas.width - padRight, y);
        ctx.stroke();
      }

      ctx.strokeStyle = "#0f766e";
      ctx.lineWidth = 1.8;
      ctx.beginPath();
      payload.rows.forEach((row, index) => {
        const x = mapX(index);
        const y = mapY(row.baseline_error_m);
        if (index === 0) ctx.moveTo(x, y);
        else ctx.lineTo(x, y);
      });
      ctx.stroke();

      if (headingValues.length) {
        ctx.strokeStyle = "#b45309";
        ctx.lineWidth = 1.5;
        ctx.beginPath();
        let started = false;
        payload.rows.forEach((row, index) => {
          if (row.heading_error_deg === null || row.heading_error_deg === undefined) {
            return;
          }
          const x = mapX(index);
          const y = mapY(row.heading_error_deg);
          if (!started) {
            ctx.moveTo(x, y);
            started = true;
          } else {
            ctx.lineTo(x, y);
          }
        });
        if (started) ctx.stroke();
      }

      ctx.fillStyle = "#6b7280";
      ctx.font = "12px IBM Plex Sans, sans-serif";
      ctx.fillText("baseline m", 12, 18);
      ctx.fillText("heading deg", 98, 18);
      ctx.fillStyle = "#0f766e";
      ctx.fillRect(74, 9, 14, 3);
      ctx.fillStyle = "#b45309";
      ctx.fillRect(166, 9, 14, 3);
      ctx.fillStyle = "#6b7280";
      ctx.fillText(`rows: ${payload.rows.length}`, 14, canvas.height - 10);
      ctx.fillText(payload.path || "", 94, canvas.height - 10);
    }

    function drawMovingBaseSeries(canvas, payload, field, label, color, suffix) {
      const ctx = canvas.getContext("2d");
      ctx.clearRect(0, 0, canvas.width, canvas.height);
      ctx.fillStyle = "#fff";
      ctx.fillRect(0, 0, canvas.width, canvas.height);

      if (!payload.available || !payload.rows.length) {
        ctx.fillStyle = "#6b7280";
        ctx.font = "16px IBM Plex Sans, sans-serif";
        ctx.fillText(payload.error || "No moving-base matches", 24, 40);
        return;
      }

      const rows = payload.rows.filter((row) => row[field] !== null && row[field] !== undefined);
      if (!rows.length) {
        ctx.fillStyle = "#6b7280";
        ctx.font = "16px IBM Plex Sans, sans-serif";
        ctx.fillText(`No ${label.toLowerCase()} data`, 24, 40);
        return;
      }

      const padLeft = 52;
      const padRight = 18;
      const padTop = 20;
      const padBottom = 30;
      const plotWidth = canvas.width - padLeft - padRight;
      const plotHeight = canvas.height - padTop - padBottom;
      const values = rows.map((row) => row[field]);
      const maxValue = Math.max(...values, 0.1);

      const mapX = (index) =>
        padLeft + (plotWidth * index) / Math.max(rows.length - 1, 1);
      const mapY = (value) =>
        padTop + plotHeight - (plotHeight * value) / Math.max(maxValue, 1e-6);

      ctx.strokeStyle = "#d1d5db";
      ctx.lineWidth = 1;
      for (let i = 0; i <= 4; i += 1) {
        const y = padTop + (plotHeight * i) / 4;
        ctx.beginPath();
        ctx.moveTo(padLeft, y);
        ctx.lineTo(canvas.width - padRight, y);
        ctx.stroke();
      }

      ctx.strokeStyle = color;
      ctx.lineWidth = 1.8;
      ctx.beginPath();
      rows.forEach((row, index) => {
        const x = mapX(index);
        const y = mapY(row[field]);
        if (index === 0) ctx.moveTo(x, y);
        else ctx.lineTo(x, y);
      });
      ctx.stroke();

      ctx.fillStyle = "#6b7280";
      ctx.font = "12px IBM Plex Sans, sans-serif";
      ctx.fillText(label, 12, 18);
      ctx.fillStyle = color;
      ctx.fillRect(12, 24, 18, 3);
      ctx.fillStyle = "#6b7280";
      ctx.fillText(`rows: ${rows.length}`, 14, canvas.height - 10);
      ctx.fillText(
        `max ${formatMaybeNumber(maxValue, field === "heading_error_deg" ? 2 : 3, suffix)}`,
        94,
        canvas.height - 10
      );
    }

    function drawSolutionStatusTimeline(canvas, payload) {
      const ctx = canvas.getContext("2d");
      ctx.clearRect(0, 0, canvas.width, canvas.height);
      ctx.fillStyle = "#fff";
      ctx.fillRect(0, 0, canvas.width, canvas.height);

      const analysis = payload.analysis || {};
      const points = analysis.points || [];
      if (!payload.available || !points.length) {
        ctx.fillStyle = "#6b7280";
        ctx.font = "16px IBM Plex Sans, sans-serif";
        ctx.fillText(payload.error || "No RTKLIB epochs", 24, 40);
        return;
      }

      const left = 54;
      const right = 18;
      const top = 28;
      const stripHeight = 58;
      const plotWidth = canvas.width - left - right;
      const duration = Math.max(analysis.duration_s || points[points.length - 1].elapsed_s || 1, 1);
      const mapX = (elapsed) => left + (plotWidth * elapsed) / duration;

      ctx.strokeStyle = "#d1d5db";
      ctx.lineWidth = 1;
      ctx.strokeRect(left, top, plotWidth, stripHeight);
      for (const point of points) {
        const x = mapX(point.elapsed_s || 0);
        const nextWidth = Math.max(2, plotWidth / Math.max(points.length, 1));
        ctx.fillStyle = point.color || STATUS_COLORS[point.status] || "#64748b";
        ctx.fillRect(x, top, nextWidth, stripHeight);
      }

      const runRows = (analysis.status_runs || []).slice(0, 5);
      ctx.fillStyle = "#374151";
      ctx.font = "12px IBM Plex Sans, sans-serif";
      ctx.fillText(`epochs ${analysis.epochs ?? "n/a"} / transitions ${analysis.status_transition_count ?? "n/a"} / gaps ${analysis.large_gap_count ?? "n/a"}`, 12, 16);
      ctx.fillStyle = "#6b7280";
      ctx.fillText(`TOW ${formatMaybeNumber(analysis.start_tow, 1, "s")} - ${formatMaybeNumber(analysis.end_tow, 1, "s")}`, left, top + stripHeight + 18);
      runRows.forEach((run, index) => {
        const y = top + stripHeight + 38 + index * 17;
        ctx.fillStyle = STATUS_COLORS[run.status] || "#64748b";
        ctx.fillRect(left, y - 9, 10, 10);
        ctx.fillStyle = "#6b7280";
        ctx.fillText(`${run.status}: ${run.epochs} epochs, ${formatMaybeNumber(run.duration_s, 1, "s")}`, left + 16, y);
      });
      if ((analysis.status_runs || []).length > runRows.length) {
        ctx.fillText(`+${analysis.status_runs.length - runRows.length} more runs`, left + 16, top + stripHeight + 38 + runRows.length * 17);
      }
    }

    function drawSolutionRatioSatellites(canvas, payload) {
      const ctx = canvas.getContext("2d");
      ctx.clearRect(0, 0, canvas.width, canvas.height);
      ctx.fillStyle = "#fff";
      ctx.fillRect(0, 0, canvas.width, canvas.height);

      const analysis = payload.analysis || {};
      const points = (analysis.points || []).filter((point) => point.satellites !== null && point.satellites !== undefined);
      if (!payload.available || !points.length) {
        ctx.fillStyle = "#6b7280";
        ctx.font = "16px IBM Plex Sans, sans-serif";
        ctx.fillText(payload.error || "No RTKLIB epochs", 24, 40);
        return;
      }

      const left = 54;
      const right = 18;
      const top = 24;
      const bottom = 34;
      const plotWidth = canvas.width - left - right;
      const plotHeight = canvas.height - top - bottom;
      const duration = Math.max(analysis.duration_s || points[points.length - 1].elapsed_s || 1, 1);
      const satMax = Math.max(...points.map((point) => point.satellites), 1);
      const ratioValues = points.map((point) => point.ratio).filter((value) => value !== null && value !== undefined);
      const ratioMax = Math.max(...ratioValues, 1);
      const yMax = Math.max(satMax, ratioMax);
      const mapX = (elapsed) => left + (plotWidth * elapsed) / duration;
      const mapY = (value) => top + plotHeight - (plotHeight * value) / Math.max(yMax, 1e-6);

      ctx.strokeStyle = "#d1d5db";
      ctx.lineWidth = 1;
      for (let i = 0; i <= 4; i += 1) {
        const y = top + (plotHeight * i) / 4;
        ctx.beginPath();
        ctx.moveTo(left, y);
        ctx.lineTo(canvas.width - right, y);
        ctx.stroke();
      }

      function strokeSeries(field, color) {
        ctx.strokeStyle = color;
        ctx.lineWidth = 1.7;
        ctx.beginPath();
        let started = false;
        points.forEach((point) => {
          const value = point[field];
          if (value === null || value === undefined) return;
          const x = mapX(point.elapsed_s || 0);
          const y = mapY(value);
          if (!started) {
            ctx.moveTo(x, y);
            started = true;
          } else {
            ctx.lineTo(x, y);
          }
        });
        if (started) ctx.stroke();
      }

      strokeSeries("ratio", "#7c3aed");
      strokeSeries("satellites", "#0f766e");
      ctx.fillStyle = "#6b7280";
      ctx.font = "12px IBM Plex Sans, sans-serif";
      ctx.fillText(`ratio p50 ${formatMaybeNumber(analysis.median_ratio, 2)} / p95 ${formatMaybeNumber(analysis.p95_ratio, 2)}`, 12, 16);
      ctx.fillStyle = "#7c3aed";
      ctx.fillRect(12, 24, 18, 3);
      ctx.fillStyle = "#0f766e";
      ctx.fillRect(94, 24, 18, 3);
      ctx.fillStyle = "#6b7280";
      ctx.fillText("ratio", 34, 28);
      ctx.fillText("satellites", 116, 28);
      ctx.fillText(`sat ${analysis.min_satellites ?? "n/a"}-${analysis.max_satellites ?? "n/a"}`, 14, canvas.height - 10);
      ctx.fillText(`max ratio ${formatMaybeNumber(analysis.max_ratio, 2)}`, 120, canvas.height - 10);
    }

    function drawSummaryBars(canvas, rows, field, label, color, lowerIsBetter = false) {
      const ctx = canvas.getContext("2d");
      ctx.clearRect(0, 0, canvas.width, canvas.height);
      ctx.fillStyle = "#fff";
      ctx.fillRect(0, 0, canvas.width, canvas.height);

      const points = (rows || [])
        .filter((row) => row[field] !== null && row[field] !== undefined)
        .slice()
        .sort((a, b) => lowerIsBetter ? a[field] - b[field] : b[field] - a[field])
        .slice(0, 10);
      if (!points.length) {
        ctx.fillStyle = "#6b7280";
        ctx.font = "16px IBM Plex Sans, sans-serif";
        ctx.fillText(`No ${label} data`, 24, 40);
        return;
      }

      const left = 150;
      const right = 18;
      const top = 24;
      const bottom = 24;
      const plotWidth = canvas.width - left - right;
      const plotHeight = canvas.height - top - bottom;
      const rowHeight = plotHeight / points.length;
      const maxValue = Math.max(...points.map((row) => Math.abs(row[field])), 1e-6);

      ctx.font = "11px IBM Plex Sans, sans-serif";
      points.forEach((row, index) => {
        const y = top + index * rowHeight;
        const barWidth = (plotWidth * Math.abs(row[field])) / maxValue;
        ctx.fillStyle = row.family === "libgnss++" ? "#7c3aed" : color;
        ctx.fillRect(left, y + 4, barWidth, Math.max(8, rowHeight - 8));
        ctx.fillStyle = "#374151";
        const name = String(row.label || row._path || "row");
        ctx.fillText(name.length > 22 ? `${name.slice(0, 21)}...` : name, 8, y + rowHeight * 0.62);
        ctx.fillText(formatMaybeNumber(row[field], field.endsWith("_pct") ? 2 : 3, field.endsWith("_pct") ? "%" : " m"), left + barWidth + 6, y + rowHeight * 0.62);
      });
      ctx.fillStyle = "#6b7280";
      ctx.fillText(label, 12, 14);
    }

    function drawEpochDiagnosticSeries(canvas, diagnostic, mode) {
      const ctx = canvas.getContext("2d");
      ctx.clearRect(0, 0, canvas.width, canvas.height);
      ctx.fillStyle = "#fff";
      ctx.fillRect(0, 0, canvas.width, canvas.height);

      const points = (diagnostic && diagnostic.points) || [];
      if (!points.length) {
        ctx.fillStyle = "#6b7280";
        ctx.font = "16px IBM Plex Sans, sans-serif";
        ctx.fillText("No matched epoch diagnostics", 24, 40);
        return;
      }

      const left = 52;
      const right = 18;
      const top = 28;
      const bottom = 34;
      const plotWidth = canvas.width - left - right;
      const plotHeight = canvas.height - top - bottom;
      const xs = points.map((point, index) => point.epoch ?? index);
      const minX = Math.min(...xs);
      const maxX = Math.max(...xs);
      const series = mode === "cues"
        ? [
            ["dd_shift_m", "#b45309", "DD shift"],
            ["dd_rms_m", "#7c3aed", "DD rms"],
            ["tdcp_postfit_rms_m", "#0f766e", "TDCP"],
          ]
        : mode === "stages"
        ? [
            ["stage_start_m", "#64748b", "start"],
            ["stage_pr_m", "#b45309", "PR"],
            ["stage_doppler_m", "#0f766e", "doppler"],
            ["stage_dd_carrier_m", "#7c3aed", "DD carrier"],
            ["stage_position_m", "#0369a1", "position"],
            ["stage_emit_m", "#b91c1c", "emit"],
            ["stage_end_m", "#111827", "end"],
          ]
        : [
            ["error_m", "#b91c1c", diagnostic.error_column || "error"],
            ["wls_error_m", "#64748b", "WLS"],
          ];
      const values = [];
      for (const [field] of series) {
        for (const point of points) {
          if (point[field] !== null && point[field] !== undefined) values.push(point[field]);
        }
      }
      const maxY = Math.max(...values, 1.0);
      const mapX = (value) => left + (plotWidth * (value - minX)) / Math.max(maxX - minX, 1);
      const mapY = (value) => top + plotHeight - (plotHeight * value) / Math.max(maxY, 1e-6);

      ctx.strokeStyle = "#d1d5db";
      ctx.lineWidth = 1;
      for (let i = 0; i <= 4; i += 1) {
        const y = top + (plotHeight * i) / 4;
        ctx.beginPath();
        ctx.moveTo(left, y);
        ctx.lineTo(canvas.width - right, y);
        ctx.stroke();
      }
      if (maxY >= 0.5) {
        const thresholdY = mapY(0.5);
        ctx.setLineDash([4, 4]);
        ctx.strokeStyle = "#dc2626";
        ctx.beginPath();
        ctx.moveTo(left, thresholdY);
        ctx.lineTo(canvas.width - right, thresholdY);
        ctx.stroke();
        ctx.setLineDash([]);
        ctx.fillStyle = "#dc2626";
        ctx.font = "11px IBM Plex Sans, sans-serif";
        ctx.fillText("0.5 m PPC", left + 6, Math.max(top + 12, thresholdY - 4));
      }

      for (const [field, color] of series) {
        ctx.strokeStyle = color;
        ctx.lineWidth = 1.7;
        ctx.beginPath();
        let started = false;
        points.forEach((point, index) => {
          const value = point[field];
          if (value === null || value === undefined) return;
          const x = mapX(point.epoch ?? index);
          const y = mapY(value);
          if (!started) {
            ctx.moveTo(x, y);
            started = true;
          } else {
            ctx.lineTo(x, y);
          }
        });
        if (started) ctx.stroke();
      }

      ctx.fillStyle = "#6b7280";
      ctx.font = "12px IBM Plex Sans, sans-serif";
      ctx.fillText(`${diagnostic.dataset_key || "dataset"} / ${diagnostic.source || ""}`, 12, 16);
      series.slice(0, 5).forEach(([_, color, label], index) => {
        const x = 12 + index * 108;
        ctx.fillStyle = color;
        ctx.fillRect(x, 24, 18, 3);
        ctx.fillStyle = "#6b7280";
        ctx.fillText(label, x + 24, 28);
      });
      ctx.fillText(`p95 ${formatMaybeNumber(diagnostic.p95_error_m, 3, " m")} / max ${formatMaybeNumber(diagnostic.max_error_m, 3, " m")}`, 14, canvas.height - 10);
    }

    function formatMaybeNumber(value, digits = 2, suffix = "") {
      if (value === null || value === undefined) return "n/a";
      if (typeof value === "number") return `${value.toFixed(digits)}${suffix}`;
      return String(value);
    }

    function renderBadge(status) {
      const normalized = (status || "n/a");
      const className = normalized === "n/a" ? "na" : normalized;
      return `<span class="badge ${className}">${normalized}</span>`;
    }

    function commercialReceiverArtifactLinks(row) {
      const receiver = row.commercial_receiver || {};
      const solution = receiver.solution_pos || row.commercial_receiver_csv;
      const matches = receiver.matched_csv || row.commercial_receiver_matched_csv;
      return [
        solution ? artifactLink(solution, "receiver") : null,
        matches ? artifactLink(matches, "receiver-matches") : null,
      ].filter(Boolean);
    }

    function commercialReceiverSummary(row, mode) {
      const receiver = row.commercial_receiver;
      if (!receiver) return "n/a";
      const metric = mode === "moving-base"
        ? `p95 ${formatMaybeNumber(receiver.p95_baseline_error_m, 3, " m")}`
        : `p95 H ${formatMaybeNumber(receiver.p95_h_m, 3, " m")}`;
      const bits = [
        receiver.label || "receiver",
        `${receiver.matched_epochs ?? "n/a"} matched`,
        `fix ${formatMaybeNumber(receiver.fix_rate_pct, 2, "%")}`,
        metric,
      ].filter(Boolean);
      const links = commercialReceiverArtifactLinks(row);
      return `${bits.join(" / ")}${links.length ? `<br><span class="tiny">${links.join(" / ")}</span>` : ""}`;
    }

    function commercialDeltaSummary(delta, mode, status) {
      if (!delta) return "n/a";
      const bits = [];
      if (mode === "moving-base") {
        if (delta.fix_rate_pct_delta !== null && delta.fix_rate_pct_delta !== undefined) {
          bits.push(`Δfix ${formatMaybeNumber(delta.fix_rate_pct_delta, 2, "%")}`);
        }
        if (delta.median_baseline_error_m_delta !== null && delta.median_baseline_error_m_delta !== undefined) {
          bits.push(`Δmedian ${formatMaybeNumber(delta.median_baseline_error_m_delta, 3, " m")}`);
        }
        if (delta.p95_baseline_error_m_delta !== null && delta.p95_baseline_error_m_delta !== undefined) {
          bits.push(`Δp95 ${formatMaybeNumber(delta.p95_baseline_error_m_delta, 3, " m")}`);
        }
        if (delta.p95_heading_error_deg_delta !== null && delta.p95_heading_error_deg_delta !== undefined) {
          bits.push(`Δhead ${formatMaybeNumber(delta.p95_heading_error_deg_delta, 2, " deg")}`);
        }
      } else {
        if (delta.fix_rate_pct !== null && delta.fix_rate_pct !== undefined) {
          bits.push(`Δfix ${formatMaybeNumber(delta.fix_rate_pct, 2, "%")}`);
        }
        if (delta.median_h_m !== null && delta.median_h_m !== undefined) {
          bits.push(`Δmedian ${formatMaybeNumber(delta.median_h_m, 3, " m")}`);
        }
        if (delta.p95_h_m !== null && delta.p95_h_m !== undefined) {
          bits.push(`Δp95 ${formatMaybeNumber(delta.p95_h_m, 3, " m")}`);
        }
        if (delta.p95_abs_up_m !== null && delta.p95_abs_up_m !== undefined) {
          bits.push(`Δup95 ${formatMaybeNumber(delta.p95_abs_up_m, 3, " m")}`);
        }
      }
      if (status) bits.push(renderBadge(status));
      return bits.length ? bits.join(" / ") : "n/a";
    }

    function bundleMetricSummary(bundle) {
      if (!bundle || !bundle.metrics) return "";
      const metrics = bundle.metrics;
      switch (bundle.category) {
        case "ppc":
          return [
            metrics.matched_epochs ?? "n/a",
            "matched",
            "/",
            formatMaybeNumber(metrics.p95_h_m, 2, " m"),
            "p95",
            "/",
            formatMaybeNumber(metrics.realtime_factor, 2, "x"),
          ].join(" ");
        case "live":
          return [
            metrics.termination || "n/a",
            "/",
            metrics.written_solutions ?? "n/a",
            "written",
            "/",
            formatMaybeNumber(metrics.realtime_factor, 2, "x"),
          ].join(" ");
        case "visibility":
          return [
            metrics.rows_written ?? "n/a",
            "rows",
            "/",
            metrics.unique_satellites ?? "n/a",
            "sats",
            "/",
            formatMaybeNumber(metrics.mean_elevation_deg, 1, " deg"),
          ].join(" ");
        case "moving-base":
          return [
            metrics.matched_epochs ?? "n/a",
            "matched",
            "/",
            formatMaybeNumber(metrics.p95_baseline_error_m, 3, " m"),
            "p95",
            "/",
            formatMaybeNumber(metrics.realtime_factor, 2, "x"),
          ].join(" ");
        case "ppp-products":
          return [
            formatMaybeNumber(metrics.ppp_solution_rate_pct, 1, "%"),
            "rate",
            "/",
            formatMaybeNumber(metrics.ppp_convergence_time_s, 1, " s"),
            "conv",
            "/",
            formatMaybeNumber(metrics.p95_position_error_m, 3, " m"),
          ].join(" ");
        default:
          return "";
      }
    }

    function artifactLink(path, label) {
      if (!path) return label || "n/a";
      return `<a href="/artifact?path=${encodeURIComponent(path)}" target="_blank" rel="noreferrer">${label || path}</a>`;
    }

    function smartLink(value, label) {
      if (!value) return label || "n/a";
      if (String(value).startsWith("http://") || String(value).startsWith("https://")) {
        return `<a href="${value}" target="_blank" rel="noreferrer">${label || value}</a>`;
      }
      return artifactLink(value, label);
    }

    async function refreshStatus() {
      const metrics = document.getElementById("receiver-metrics");
      const pre = document.getElementById("receiver-json");
      try {
        const payload = await fetchJson("/api/status");
        if (!payload.available) {
          renderMetricGrid(metrics, [["state", "n/a"], ["path", "not configured"]]);
          pre.textContent = payload.message || "No receiver status configured.";
          return;
        }
        renderMetricGrid(metrics, [
          ["state", payload.state || "unknown"],
          ["pid", payload.pid_running ? `running (${payload.pid || "n/a"})` : "stopped"],
          ["uptime", payload.uptime_seconds !== null ? `${payload.uptime_seconds.toFixed(1)} s` : "n/a"],
          ["restarts", payload.restart_count ?? 0],
        ]);
        pre.textContent = JSON.stringify(payload.raw, null, 2);
      } catch (error) {
        renderMetricGrid(metrics, [["state", "error"]]);
        pre.textContent = String(error);
      }
    }

    async function init() {
      renderLegend();
      const overview = await fetchJson("/api/overview");
      const docsLink = document.getElementById("docs-link");
      if (overview.docs_url) docsLink.href = overview.docs_url;
      const manifestLink = document.getElementById("artifact-manifest-link");
      if (overview.artifacts.artifact_manifest) {
        manifestLink.href = `/artifact?path=${encodeURIComponent(overview.artifacts.artifact_manifest)}`;
        manifestLink.textContent = overview.artifacts.artifact_manifest;
      } else {
        manifestLink.removeAttribute("href");
        manifestLink.textContent = "manifest unavailable";
      }
      const chips = document.getElementById("artifact-chips");
      for (const [key, value] of Object.entries(overview.artifacts)) {
        if (!value) continue;
        const chip = document.createElement("div");
        chip.className = "chip";
        chip.textContent = `${key}: ${value}`;
        chips.appendChild(chip);
      }

      const odaiba = overview.odaiba_summary || {};
      document.getElementById("odaiba-source").textContent = overview.artifacts.odaiba_summary || "";
      renderMetricGrid(document.getElementById("odaiba-metrics"), [
        ["lib epochs", odaiba.all_epochs?.libgnsspp?.epochs ?? "n/a"],
        ["RTKLIB epochs", odaiba.all_epochs?.rtklib?.epochs ?? "n/a"],
        ["lib fix rate", formatMaybeNumber(odaiba.all_epochs?.libgnsspp?.fix_rate_pct, 2, "%")],
        ["RTKLIB fix rate", formatMaybeNumber(odaiba.all_epochs?.rtklib?.fix_rate_pct, 2, "%")],
        ["common median H", formatMaybeNumber(odaiba.common_epochs?.libgnsspp?.median_h_m, 3, " m")],
        ["common p95 H", formatMaybeNumber(odaiba.common_epochs?.libgnsspp?.p95_h_m, 2, " m")],
      ]);

      const gnssGpu = overview.gnss_gpu_summaries || {};
      const gnssGpuRows = gnssGpu.top_rows || [];
      const gnssGpuComparison = gnssGpu.comparison || {};
      const gnssGpuMatched = gnssGpu.matched_comparisons || [];
      const gnssGpuWorstLosses = gnssGpu.matched_worst_losses || [];
      const gnssGpuBestWins = gnssGpu.matched_best_wins || [];
      const gnssGpuFailureClusters = gnssGpu.matched_failure_clusters || [];
      const gnssGpuEpochDiagnostics = gnssGpu.matched_epoch_diagnostics || {};
      const gnssGpuEpochRows = gnssGpuEpochDiagnostics.top || [];
      const gnssGpuEpochComparisons = gnssGpuEpochDiagnostics.comparisons || [];
      const gnssGpuMatchedSummary = gnssGpu.matched_summary || {};
      const gnssGpuScoreScope = gnssGpu.score_scope_summary || {};
      const gnssGpuScopeCounts = gnssGpuScoreScope.scope_counts || {};
      const gnssGpuScopeTraps = gnssGpuScoreScope.segment_over_honest_traps || [];
      const gnssGpuMethodComparisons = gnssGpu.method_comparisons || [];
      const gnssGpuMethodRegressions = gnssGpu.method_regressions || [];
      const gnssGpuStatusCounts = gnssGpuMatchedSummary.status_counts || {};
      const topFailureCluster = gnssGpuFailureClusters[0] || {};
      const primaryMatched = gnssGpuMatched[0] || {};
      const worstMatched = gnssGpuMatchedSummary.dataset_worst_loss || gnssGpuMatchedSummary.worst_loss || gnssGpuWorstLosses[0] || {};
      const bestMatched = gnssGpuMatchedSummary.dataset_best_win || gnssGpuMatchedSummary.best_win || gnssGpuBestWins[0] || {};
      const gnssGpuBest = gnssGpuComparison.gnss_gpu || gnssGpu.best_by_family?.gnss_gpu || {};
      const libgnssBest = gnssGpuComparison.libgnsspp || gnssGpu.best_by_family?.["libgnss++"] || {};
      renderMetricGrid(document.getElementById("gnss-gpu-metrics"), [
        ["CSV files", (gnssGpu.files || []).length],
        ["rows", gnssGpu.row_count ?? (gnssGpu.rows || []).length],
        ["matched keys", gnssGpu.matched_comparison_count ?? gnssGpuMatched.length],
        ["dataset matches", gnssGpuMatchedSummary.dataset_count ?? "n/a"],
        ["score scopes", Object.entries(gnssGpuScopeCounts).map(([key, value]) => `${key}:${value}`).join(" / ") || "n/a"],
        ["scope traps", gnssGpuScoreScope.segment_over_honest_trap_count ?? 0],
        ["method regressions", gnssGpuMethodRegressions.length],
        ["wins / losses", `${gnssGpuStatusCounts.better ?? 0} / ${gnssGpuStatusCounts.worse ?? 0}`],
        ["top loss cluster", topFailureCluster.value ? `${topFailureCluster.value} (${topFailureCluster.count})` : "n/a"],
        ["epoch diag files", (gnssGpuEpochDiagnostics.files || []).length],
        ["epoch diff sets", gnssGpuEpochComparisons.length],
        ["GNSS GPU best", gnssGpuBest.score_pct !== null && gnssGpuBest.score_pct !== undefined ? formatMaybeNumber(gnssGpuBest.score_pct, 2, "%") : formatMaybeNumber(gnssGpuBest.p95_m, 3, " m")],
        ["libgnss++ best", libgnssBest.score_pct !== null && libgnssBest.score_pct !== undefined ? formatMaybeNumber(libgnssBest.score_pct, 2, "%") : formatMaybeNumber(libgnssBest.p95_m, 3, " m")],
        ["worst Δ score", formatMaybeNumber(worstMatched.score_delta_pct, 3, "%")],
        ["best Δ score", formatMaybeNumber(bestMatched.score_delta_pct, 3, "%")],
        ["worst Δ p95", formatMaybeNumber(worstMatched.p95_delta_m, 3, " m")],
        ["global Δ score", formatMaybeNumber(gnssGpuComparison.score_delta_pct, 3, "%")],
      ]);
      document.getElementById("gnss-gpu-comparison-note").innerHTML = worstMatched.status
        ? `${renderBadge(worstMatched.status)} worst matched by ${worstMatched.key_type}: ${worstMatched.key}; ${worstMatched.gnss_gpu?.label || "gnss_gpu"} vs ${worstMatched.libgnsspp?.label || "libgnss++"}`
        : (gnssGpuComparison.status
          ? `${renderBadge(gnssGpuComparison.status)} global best only: ${gnssGpuBest.label || "gnss_gpu"} vs ${libgnssBest.label || "libgnss++"}`
          : "No GNSS GPU vs libgnss++ comparison row found yet.");
      function renderScoreScopeTrapRows(rows) {
        const body = document.querySelector("#gnss-gpu-score-scope-table tbody");
        body.innerHTML = "";
        for (const row of rows.slice(0, 12)) {
          const tr = document.createElement("tr");
          tr.innerHTML = `
            <td>${artifactLink(row.path, row.label || row.path || "n/a")}<br><span class="tiny">${row.score_column || "n/a"} (${row.score_scope || "n/a"})</span></td>
            <td>${formatMaybeNumber(row.segment_ppc_pct, 3, "%")}</td>
            <td>${formatMaybeNumber(row.honest_ppc_pct, 3, "%")}</td>
            <td>${formatMaybeNumber(row.score_gap_pct, 3, "%")}</td>
            <td>${row.dataset_key || "n/a"}<br><span class="tiny">${formatMaybeNumber(row.segment_total_m, 2, " m")} / ${row.n_epochs ?? "n/a"} epochs</span></td>
            <td>${row.method || "n/a"}<br><span class="tiny">${row.family || "n/a"}</span></td>`;
          body.appendChild(tr);
        }
      }
      function renderMethodComparisonRows(rows) {
        const body = document.querySelector("#gnss-gpu-method-compare-table tbody");
        body.innerHTML = "";
        for (const row of rows.slice(0, 12)) {
          const hybrid = row.hybrid || {};
          const rtkdiag = row.rtkdiag || {};
          const counts = row.candidate_counts || {};
          const tr = document.createElement("tr");
          tr.innerHTML = `
            <td>${row.dataset_key || "n/a"}<br><span class="tiny">hybrid ${counts.hybrid ?? "n/a"} / rtkdiag ${counts.rtkdiag ?? "n/a"}</span></td>
            <td>${artifactLink(hybrid._path, hybrid.label || "hybrid")}<br><span class="tiny">${formatMaybeNumber(hybrid.score_pct, 3, "%")} (${hybrid.score_scope || "n/a"})</span></td>
            <td>${artifactLink(rtkdiag._path, rtkdiag.label || "rtkdiag")}<br><span class="tiny">${formatMaybeNumber(rtkdiag.score_pct, 3, "%")} (${rtkdiag.score_scope || "n/a"})</span></td>
            <td>${formatMaybeNumber(row.score_delta_pct, 3, "%")}</td>
            <td>${formatMaybeNumber(row.p95_delta_m, 3, " m")}</td>
            <td>${renderBadge(row.status)}</td>`;
          body.appendChild(tr);
        }
      }
      function renderFailureClusterRows(rows) {
        const body = document.querySelector("#gnss-gpu-cluster-table tbody");
        body.innerHTML = "";
        for (const row of rows.slice(0, 12)) {
          const tr = document.createElement("tr");
          tr.innerHTML = `
            <td>${row.dimension || "n/a"}<br><span class="tiny">${row.value || "n/a"}</span></td>
            <td>${row.count ?? "n/a"}</td>
            <td>${formatMaybeNumber(row.avg_delta_pct, 3, "%")}</td>
            <td>${formatMaybeNumber(row.worst_delta_pct, 3, "%")}</td>
            <td>${formatMaybeNumber(row.avg_p95_delta_m, 3, " m")}</td>
            <td>${row.worst_key_type || "n/a"}<br><span class="tiny">${row.worst_key || "n/a"}</span><br><span class="tiny">${row.worst_gnss_gpu || "gnss_gpu"} vs ${row.worst_libgnsspp || "libgnss++"}</span></td>`;
          body.appendChild(tr);
        }
      }
      function renderEpochDiagnosticRows(rows) {
        const body = document.querySelector("#gnss-gpu-epoch-table tbody");
        body.innerHTML = "";
        for (const row of rows.slice(0, 10)) {
          const textBits = [];
          for (const [key, counts] of Object.entries(row.text_counts || {})) {
            const top = (counts || []).slice(0, 2).map((item) => `${item.value}:${item.count}`).join(", ");
            if (top) textBits.push(`${key} ${top}`);
          }
          const tr = document.createElement("tr");
          const firstBad = row.first_bad_stage || {};
          const firstPpcBad = row.first_ppc_bad_stage || {};
          const worstStage = row.worst_stage || {};
          const largestJump = row.largest_stage_jump || {};
          const doppler = row.doppler_diagnostics || {};
          const pu = row.position_update_diagnostics || {};
          tr.innerHTML = `
            <td>${artifactLink(row.path, row.dataset_key || "dataset")}<br><span class="tiny">${row.source || row.path || "n/a"}</span></td>
            <td>${formatMaybeNumber(row.score_pct, 3, "%")}<br><span class="tiny">pass <=${formatMaybeNumber(row.pass_threshold_m, 2, " m")} ${row.pass_epochs ?? "n/a"}/${row.n_epochs ?? "n/a"}</span><br><span class="tiny">med ${formatMaybeNumber(row.median_error_m, 3, " m")} / p95 ${formatMaybeNumber(row.p95_error_m, 3, " m")} / max ${formatMaybeNumber(row.max_error_m, 3, " m")}</span></td>
            <td>${row.first_ppc_fail_epoch ?? "n/a"} / ${row.worst_epoch ?? "n/a"}<br><span class="tiny">fail>0.5m ${row.fail_epochs_gt05m ?? "n/a"} epochs</span><br><span class="tiny">fail>3m ${row.fail_epochs_gt3m ?? "n/a"} @ ${row.first_fail_epoch ?? "n/a"}</span></td>
            <td>${row.ppc_stage_diagnosis || row.stage_diagnosis || "n/a"}<br><span class="tiny">PPC first ${firstPpcBad.label || "n/a"} @ ${firstPpcBad.first_fail_epoch_gt05m ?? "n/a"} (${firstPpcBad.fail_epochs_gt05m ?? "n/a"} epochs)</span><br><span class="tiny">3m first ${firstBad.label || "n/a"} @ ${firstBad.first_fail_epoch ?? "n/a"} / worst ${worstStage.label || "n/a"} p95 ${formatMaybeNumber(worstStage.p95_m, 3, " m")}</span><br><span class="tiny">jump ${largestJump.from_label || "n/a"} -> ${largestJump.to_label || "n/a"} ${formatMaybeNumber(largestJump.delta_p95_m, 3, " m")}</span><br><span class="tiny">${doppler.diagnosis || "doppler n/a"}: max +${formatMaybeNumber(doppler.max_error_delta_m, 3, " m")} @ ${doppler.worst_epoch ?? "n/a"}</span><br><span class="tiny">${pu.diagnosis || "PU n/a"}: max +${formatMaybeNumber(pu.max_error_delta_m, 3, " m")} @ ${pu.worst_epoch ?? "n/a"}</span></td>
            <td>DD ${formatMaybeNumber(row.dd_anchor_rate_pct, 1, "%")} / TDCP ${formatMaybeNumber(row.tdcp_rate_pct, 1, "%")}<br><span class="tiny">dop ${formatMaybeNumber(row.doppler_update_rate_pct, 1, "%")} / carrier ${formatMaybeNumber(row.dd_carrier_update_rate_pct, 1, "%")} / HH ${formatMaybeNumber(row.height_hold_rate_pct, 1, "%")}</span><br><span class="tiny">dop WLS speed ${formatMaybeNumber(doppler.worst_wls_speed_mps, 2, " m/s")} / rms ${formatMaybeNumber(doppler.worst_wls_rms_mps, 2, " m/s")}</span><br><span class="tiny">PU WLS med ${formatMaybeNumber(pu.median_wls_to_ref_m, 2, " m")} / rms ${formatMaybeNumber(pu.median_wls_postfit_rms_m, 2, " m")}</span></td>
            <td><span class="tiny">${textBits.join("<br>") || "n/a"}</span></td>`;
          body.appendChild(tr);
        }
      }
      function renderInitialStateRows(rows) {
        const body = document.querySelector("#gnss-gpu-initial-table tbody");
        body.innerHTML = "";
        for (const row of rows.filter((item) => item.initial_state).slice(0, 10)) {
          const initial = row.initial_state || {};
          const epoch0 = initial.epoch0 || {};
          const initSourceCounts = (initial.init_source_counts || []).map((item) => `${item.value}:${item.count}`).join(" / ");
          const tr = document.createElement("tr");
          tr.innerHTML = `
            <td>${artifactLink(row.path, row.dataset_key || "dataset")}<br><span class="tiny">${row.source || row.path || "n/a"}</span></td>
            <td>p50 ${formatMaybeNumber(initial.median_start_to_ref_m, 3, " m")} / p95 ${formatMaybeNumber(initial.p95_start_to_ref_m, 3, " m")}<br><span class="tiny">max ${formatMaybeNumber(initial.max_start_to_ref_m, 3, " m")}; init ${formatMaybeNumber(epoch0.init_to_ref_m, 3, " m")}</span></td>
            <td>E ${formatMaybeNumber(epoch0.err_e_m, 3, " m")} / N ${formatMaybeNumber(epoch0.err_n_m, 3, " m")} / U ${formatMaybeNumber(epoch0.err_u_m, 3, " m")}<br><span class="tiny">init E ${formatMaybeNumber(epoch0.init_err_e_m, 3, " m")} / N ${formatMaybeNumber(epoch0.init_err_n_m, 3, " m")} / U ${formatMaybeNumber(epoch0.init_err_u_m, 3, " m")}</span><br><span class="tiny">p95 abs ${formatMaybeNumber(initial.p95_abs_err_e_m, 3, " m")} / ${formatMaybeNumber(initial.p95_abs_err_n_m, 3, " m")} / ${formatMaybeNumber(initial.p95_abs_err_u_m, 3, " m")}</span></td>
            <td>${formatMaybeNumber(epoch0.spread_m, 3, " m")} / ${formatMaybeNumber(epoch0.ess_ratio, 3)}<br><span class="tiny">seed ${epoch0.init_source || "n/a"} ${formatMaybeNumber(epoch0.init_spread_pos_m, 3, " m")}; counts ${initSourceCounts || "n/a"}</span><br><span class="tiny">median ${formatMaybeNumber(initial.median_spread_m, 3, " m")} / ${formatMaybeNumber(initial.median_ess_ratio, 3)}</span></td>
            <td>${epoch0.predict_source || "n/a"} / ${epoch0.pr_update_mode || "n/a"}<br><span class="tiny">sat ${epoch0.n_sat_used_pr ?? "n/a"} / ${epoch0.n_sat_raw ?? "n/a"}; emit ${epoch0.emitted_source || "n/a"}; resample ${epoch0.resampled_epoch_end ? "yes" : "no"}</span></td>
            <td>${initial.diagnosis || "n/a"}<br><span class="tiny">WLS ${formatMaybeNumber(epoch0.wls_init_to_ref_m, 3, " m")} / hybrid ${formatMaybeNumber(epoch0.hybrid_init_to_ref_m, 3, " m")}</span></td>`;
          body.appendChild(tr);
        }
      }
      function diagnosticShort(row) {
        if (!row) return "n/a";
        return `${artifactLink(row.path, row.source || row.path || "diagnostic")}<br><span class="tiny">${formatMaybeNumber(row.score_pct, 2, "%")} / p95 ${formatMaybeNumber(row.p95_error_m, 3, " m")} / ${row.n_epochs ?? "n/a"} epochs</span>`;
      }
      function scoreShort(row) {
        if (!row) return "n/a";
        const raw = row.raw_scores || {};
        const details = [];
        if (raw.segment_ppc_pct !== undefined && raw.segment_ppc_pct !== null) details.push(`segment ${formatMaybeNumber(raw.segment_ppc_pct, 2, "%")}`);
        if (raw.honest_ppc_pct !== undefined && raw.honest_ppc_pct !== null) details.push(`honest ${formatMaybeNumber(raw.honest_ppc_pct, 2, "%")}`);
        if (raw.ppc_score_pct !== undefined && raw.ppc_score_pct !== null && row.score_column !== "ppc_score_pct") details.push(`ppc ${formatMaybeNumber(raw.ppc_score_pct, 2, "%")}`);
        const source = row.score_scope || row.score_column || "score";
        const suffix = details.length ? details.slice(0, 3).join(" / ") : source;
        return `${formatMaybeNumber(row.score_pct, 3, "%")}<br><span class="tiny">${source}; ${suffix}</span>`;
      }
      function stageGapShort(gap) {
        if (!gap) return "n/a";
        return `${gap.label || "stage"} ${formatMaybeNumber(gap.delta_p95_m, 3, " m")}<br><span class="tiny">${formatMaybeNumber(gap.best_p95_m, 3, " m")} -> ${formatMaybeNumber(gap.worst_p95_m, 3, " m")}</span><br><span class="tiny">${gap.best_source || "best"} -> ${gap.worst_source || "worst"}</span>`;
      }
      function renderEpochComparisonRows(rows) {
        const body = document.querySelector("#gnss-gpu-epoch-compare-table tbody");
        body.innerHTML = "";
        for (const row of rows.slice(0, 10)) {
          const tr = document.createElement("tr");
          tr.innerHTML = `
            <td>${row.dataset_key || "n/a"}<br><span class="tiny">${row.candidate_count ?? "n/a"} diagnostics</span></td>
            <td>${diagnosticShort(row.best)}</td>
            <td>${diagnosticShort(row.worst)}</td>
            <td>${formatMaybeNumber(row.score_gap_pct, 3, "%")}<br><span class="tiny">p95 ${formatMaybeNumber(row.p95_gap_m, 3, " m")}</span></td>
            <td>${stageGapShort(row.start_stage_gap)}</td>
            <td>${stageGapShort(row.dominant_stage_gap)}</td>`;
          body.appendChild(tr);
        }
      }
      function renderMatchedComparisonRows(selector, rows) {
        const body = document.querySelector(`${selector} tbody`);
        body.innerHTML = "";
        for (const row of rows.slice(0, 12)) {
          const counts = row.candidate_counts || {};
          const tr = document.createElement("tr");
          tr.innerHTML = `
            <td>${row.key_type || "n/a"}<br><span class="tiny">${row.key || "n/a"}</span></td>
            <td>${artifactLink(row.gnss_gpu?._path, row.gnss_gpu?.label || "n/a")}<br><span class="tiny">${formatMaybeNumber(row.gnss_gpu?.score_pct, 3, "%")} (${row.gnss_gpu?.score_scope || "n/a"}) / p95 ${formatMaybeNumber(row.gnss_gpu?.p95_m, 3, " m")}</span></td>
            <td>${artifactLink(row.libgnsspp?._path, row.libgnsspp?.label || "n/a")}<br><span class="tiny">${formatMaybeNumber(row.libgnsspp?.score_pct, 3, "%")} (${row.libgnsspp?.score_scope || "n/a"}) / p95 ${formatMaybeNumber(row.libgnsspp?.p95_m, 3, " m")}</span></td>
            <td>${formatMaybeNumber(row.score_delta_pct, 3, "%")}</td>
            <td>${formatMaybeNumber(row.p95_delta_m, 3, " m")} / ${formatMaybeNumber(row.rms_delta_m, 3, " m")}</td>
            <td>${counts.gnss_gpu ?? "n/a"} / ${counts["libgnss++"] ?? "n/a"}</td>
            <td>${renderBadge(row.status)}</td>`;
          body.appendChild(tr);
        }
      }
      renderScoreScopeTrapRows(gnssGpuScopeTraps);
      renderMethodComparisonRows(gnssGpuMethodRegressions.length ? gnssGpuMethodRegressions : gnssGpuMethodComparisons);
      renderFailureClusterRows(gnssGpuFailureClusters);
      renderEpochDiagnosticRows(gnssGpuEpochRows);
      renderInitialStateRows(gnssGpuEpochRows);
      renderEpochComparisonRows(gnssGpuEpochComparisons);
      drawEpochDiagnosticSeries(document.getElementById("gnss-gpu-epoch-error-canvas"), gnssGpuEpochRows[0], "error");
      drawEpochDiagnosticSeries(document.getElementById("gnss-gpu-epoch-cue-canvas"), gnssGpuEpochRows[0], "cues");
      drawEpochDiagnosticSeries(document.getElementById("gnss-gpu-stage-error-canvas"), gnssGpuEpochRows[0], "stages");
      renderMatchedComparisonRows("#gnss-gpu-loss-table", gnssGpuWorstLosses);
      renderMatchedComparisonRows("#gnss-gpu-matched-table", gnssGpuBestWins.length ? gnssGpuBestWins : gnssGpuMatched);
      drawSummaryBars(document.getElementById("gnss-gpu-score-canvas"), gnssGpuRows, "score_pct", "score pct", "#0f766e", false);
      drawSummaryBars(document.getElementById("gnss-gpu-error-canvas"), gnssGpuRows, "p95_m", "p95 error", "#b45309", true);
      const gnssGpuBody = document.querySelector("#gnss-gpu-table tbody");
      gnssGpuBody.innerHTML = "";
      for (const row of gnssGpuRows) {
        const passDistance = row.pass_m !== null && row.pass_m !== undefined
          ? `${formatMaybeNumber(row.pass_m, 2, " m")} / ${formatMaybeNumber(row.total_m, 2, " m")}`
          : "n/a";
        const tr = document.createElement("tr");
        tr.innerHTML = `
          <td>${artifactLink(row._path, row.label || row._path || "n/a")}</td>
          <td>${row.family || "n/a"}</td>
          <td>${scoreShort(row)}</td>
          <td>${formatMaybeNumber(row.p50_m, 3, " m")} / ${formatMaybeNumber(row.p95_m, 3, " m")} / ${formatMaybeNumber(row.rms_2d_m, 3, " m")}</td>
          <td>${passDistance}</td>
          <td>${row.n_epochs ?? "n/a"}</td>
          <td>${formatMaybeNumber(row.time_ms, 3, " ms")}</td>
          <td>${renderBadge(row.quality_status)}</td>`;
        gnssGpuBody.appendChild(tr);
      }

      const liveBody = document.querySelector("#live-table tbody");
      liveBody.innerHTML = "";
      for (const row of overview.live_summaries || []) {
        const tr = document.createElement("tr");
        tr.innerHTML = `
          <td>${row._path || "n/a"}</td>
          <td>${row.termination || "n/a"}</td>
          <td>${row.written_solutions ?? "n/a"}</td>
          <td>${row.fixed_solutions ?? "n/a"}</td>
          <td>${renderBadge(row.runtime_status)} ${formatMaybeNumber(row.realtime_factor, 2, "x")}</td>
          <td>${formatMaybeNumber(row.effective_epoch_rate_hz, 2, " Hz")}</td>
          <td>${(row.rover_decoder_errors ?? "n/a")}/${(row.base_decoder_errors ?? "n/a")}</td>`;
        liveBody.appendChild(tr);
      }

      const ppcBody = document.querySelector("#ppc-table tbody");
      ppcBody.innerHTML = "";
      for (const row of overview.ppc_summaries || []) {
        const tr = document.createElement("tr");
        tr.innerHTML = `
          <td>${row._path || "n/a"}</td>
          <td>${row.matched_epochs ?? "n/a"}</td>
          <td>${formatMaybeNumber(row.positioning_rate_pct, 2, "%")}</td>
          <td>${formatMaybeNumber(row.fix_rate_pct, 2, "%")}</td>
          <td>${formatMaybeNumber(row.ppc_score_3d_50cm_ref_pct, 2, "%")}</td>
          <td>${formatMaybeNumber(row.median_h_m, 3, " m")}</td>
          <td>${formatMaybeNumber(row.p95_h_m, 2, " m")}</td>
          <td>${commercialReceiverSummary(row, "ppc")}</td>
          <td>${commercialDeltaSummary(row.delta_vs_commercial_receiver, "ppc", row.commercial_comparison_status)}</td>
          <td>${renderBadge(row.quality_status)}</td>
          <td>${renderBadge(row.runtime_status)} ${formatMaybeNumber(row.realtime_factor, 2, "x")}</td>
          <td>${formatMaybeNumber(row.solver_wall_time_s, 2, " s")}</td>
          <td>${formatMaybeNumber(row.effective_epoch_rate_hz, 2, " Hz")}</td>`;
        ppcBody.appendChild(tr);
      }

      const movingBaseBody = document.querySelector("#moving-base-table tbody");
      movingBaseBody.innerHTML = "";
      for (const row of overview.moving_base_summaries || []) {
        const provenance = [
          row.solution_pos ? artifactLink(row.solution_pos, "pos") : null,
          row.matched_csv ? artifactLink(row.matched_csv, "matches") : null,
          row.prepare_summary_json ? artifactLink(row.prepare_summary_json, "prepare") : null,
          row.products_summary_json ? artifactLink(row.products_summary_json, "products") : null,
          row.nav_rinex ? artifactLink(row.nav_rinex, "nav") : null,
          row.input ? artifactLink(row.input, "input") : null,
          row.input_url ? smartLink(row.input_url, "source") : null,
          row.plot_png ? artifactLink(row.plot_png, "plot") : null,
          ...commercialReceiverArtifactLinks(row),
        ].filter(Boolean).join(" / ");
        const tr = document.createElement("tr");
        tr.innerHTML = `
          <td>${artifactLink(row.summary_path || row._path, row._path || "n/a")}<br><span class="tiny">${provenance || (row.signoff_profile || "n/a")}</span></td>
          <td>${row.matched_epochs ?? "n/a"}</td>
          <td>${formatMaybeNumber(row.fix_rate_pct, 2, "%")}</td>
          <td>${formatMaybeNumber(row.p95_baseline_error_m, 3, " m")}</td>
          <td>${formatMaybeNumber(row.p95_heading_error_deg, 2, " deg")}</td>
          <td>${commercialReceiverSummary(row, "moving-base")}</td>
          <td>${commercialDeltaSummary(row.libgnss_vs_commercial_receiver, "moving-base", row.commercial_comparison_status)}</td>
          <td>${renderBadge(row.quality_status)}</td>
          <td>${renderBadge(row.runtime_status)} ${formatMaybeNumber(row.realtime_factor, 2, "x")}</td>
          <td>${formatMaybeNumber(row.effective_epoch_rate_hz, 2, " Hz")}</td>
          <td>${row.termination || "n/a"}</td>`;
        movingBaseBody.appendChild(tr);
      }
      const movingBaseImage = document.getElementById("moving-base-image");
      const movingBaseMetrics = document.getElementById("moving-base-metrics");
      const movingBaseProvenance = document.getElementById("moving-base-provenance");
      const firstMovingBase = (overview.moving_base_summaries || []).find((row) => row.plot_png);
      if (firstMovingBase) {
        const metricEntries = [
          ["matched", firstMovingBase.matched_epochs ?? "n/a"],
          ["fix rate", formatMaybeNumber(firstMovingBase.fix_rate_pct, 2, "%")],
          ["p95 baseline", formatMaybeNumber(firstMovingBase.p95_baseline_error_m, 3, " m")],
          ["p95 heading", formatMaybeNumber(firstMovingBase.p95_heading_error_deg, 2, " deg")],
          ["realtime", formatMaybeNumber(firstMovingBase.realtime_factor, 2, "x")],
          ["epoch rate", formatMaybeNumber(firstMovingBase.effective_epoch_rate_hz, 2, " Hz")],
        ];
        if (firstMovingBase.commercial_receiver) {
          metricEntries.push(
            ["receiver fix", formatMaybeNumber(firstMovingBase.commercial_receiver.fix_rate_pct, 2, "%")],
            ["receiver p95", formatMaybeNumber(firstMovingBase.commercial_receiver.p95_baseline_error_m, 3, " m")]
          );
        }
        if (firstMovingBase.libgnss_vs_commercial_receiver) {
          metricEntries.push(
            [
              "Δ receiver p95",
              formatMaybeNumber(firstMovingBase.libgnss_vs_commercial_receiver.p95_baseline_error_m_delta, 3, " m"),
            ]
          );
        }
        renderMetricGrid(movingBaseMetrics, metricEntries);
        movingBaseProvenance.innerHTML = [
          firstMovingBase.summary_path ? artifactLink(firstMovingBase.summary_path, "summary") : null,
          firstMovingBase.prepare_summary_json ? artifactLink(firstMovingBase.prepare_summary_json, "prepare") : null,
          firstMovingBase.products_summary_json ? artifactLink(firstMovingBase.products_summary_json, "products") : null,
          firstMovingBase.nav_rinex ? artifactLink(firstMovingBase.nav_rinex, "nav") : null,
          firstMovingBase.input ? artifactLink(firstMovingBase.input, "input") : null,
          firstMovingBase.input_url ? smartLink(firstMovingBase.input_url, "source") : null,
          ...commercialReceiverArtifactLinks(firstMovingBase),
        ].filter(Boolean).join(" / ");
      } else {
        renderMetricGrid(movingBaseMetrics, [["status", "n/a"], ["summary", "unavailable"]]);
        movingBaseProvenance.textContent = "No moving-base summaries found.";
      }
      if (firstMovingBase && firstMovingBase.plot_png) {
        movingBaseImage.src = `/artifact?path=${encodeURIComponent(firstMovingBase.plot_png)}`;
        movingBaseImage.style.display = "block";
      } else {
        movingBaseImage.removeAttribute("src");
        movingBaseImage.style.display = "none";
      }
      if (firstMovingBase && firstMovingBase.matched_csv) {
        const movingBasePayload = await fetchJson(
          `/api/moving-base-matches?path=${encodeURIComponent(firstMovingBase.matched_csv)}`
        );
        drawMovingBaseHistory(
          document.getElementById("moving-base-history-canvas"),
          movingBasePayload
        );
        drawMovingBaseSeries(
          document.getElementById("moving-base-heading-canvas"),
          movingBasePayload,
          "heading_error_deg",
          "heading deg",
          "#b45309",
          " deg"
        );
      } else {
        drawMovingBaseHistory(
          document.getElementById("moving-base-history-canvas"),
          {available: false, rows: [], error: "No moving-base matches CSV"}
        );
        drawMovingBaseSeries(
          document.getElementById("moving-base-heading-canvas"),
          {available: false, rows: [], error: "No moving-base matches CSV"},
          "heading_error_deg",
          "heading deg",
          "#b45309",
          " deg"
        );
      }

      const pppProductsBody = document.querySelector("#ppp-products-table tbody");
      pppProductsBody.innerHTML = "";
      for (const row of overview.ppp_products_summaries || []) {
        const presets = Array.isArray(row.product_presets) && row.product_presets.length
          ? row.product_presets.join(", ")
          : "custom";
        const solutionLink = row.solution_pos ? artifactLink(row.solution_pos, "pos") : null;
        const productLinks = [
          row.sp3 ? artifactLink(row.sp3, "sp3") : null,
          row.clk ? artifactLink(row.clk, "clk") : null,
          row.ionex ? artifactLink(row.ionex, "ionex") : null,
          row.dcb ? artifactLink(row.dcb, "dcb") : null,
          row.malib_solution_pos ? artifactLink(row.malib_solution_pos, "malib") : null,
          row.comparison_csv ? artifactLink(row.comparison_csv, "compare-csv") : null,
          row.comparison_png ? artifactLink(row.comparison_png, "compare-png") : null,
        ].filter(Boolean).join(" / ");
        const provenance = [
          row.reference_csv ? artifactLink(row.reference_csv, "reference") : null,
          row.run_dir ? artifactLink(row.run_dir, "run") : null,
        ].filter(Boolean).join(" / ");
        const compareBits = [];
        if (row.comparison_target) compareBits.push(row.comparison_target);
        if (row.common_epoch_pairs !== null && row.common_epoch_pairs !== undefined) {
          compareBits.push(`${row.common_epoch_pairs} paired`);
        }
        if (row.libgnss_minus_malib_mean_error_m !== null && row.libgnss_minus_malib_mean_error_m !== undefined) {
          compareBits.push(`Δmean ${formatMaybeNumber(row.libgnss_minus_malib_mean_error_m, 3, " m")}`);
        }
        if (row.libgnss_minus_malib_p95_error_m !== null && row.libgnss_minus_malib_p95_error_m !== undefined) {
          compareBits.push(`Δp95 ${formatMaybeNumber(row.libgnss_minus_malib_p95_error_m, 3, " m")}`);
        }
        if (row.libgnss_minus_malib_max_error_m !== null && row.libgnss_minus_malib_max_error_m !== undefined) {
          compareBits.push(`Δmax ${formatMaybeNumber(row.libgnss_minus_malib_max_error_m, 3, " m")}`);
        }
        const tr = document.createElement("tr");
        tr.innerHTML = `
          <td>${artifactLink(row.summary_path || row._path, row._path || "n/a")}<br><span class="tiny">${[row.dataset, solutionLink, provenance || null, productLinks || presets].filter(Boolean).join(" / ")}</span></td>
          <td>${[row.profile, row.dataset].filter(Boolean).join(" / ") || "n/a"}</td>
          <td>${row.fetched_product_date || "n/a"}</td>
          <td>${formatMaybeNumber(row.ppp_solution_rate_pct, 2, "%")}</td>
          <td>${row.ppp_converged ? "yes" : "no"} / ${formatMaybeNumber(row.ppp_convergence_time_s, 1, " s")}</td>
          <td>${formatMaybeNumber(row.mean_position_error_m, 3, " m")}</td>
          <td>${formatMaybeNumber(row.p95_position_error_m, 3, " m")} / ${formatMaybeNumber(row.max_position_error_m, 3, " m")}</td>
          <td>I ${row.ionex_corrections ?? "n/a"} / D ${row.dcb_corrections ?? "n/a"}</td>
          <td>${compareBits.length ? compareBits.join(" / ") : "n/a"}</td>
          <td>${renderBadge(row.quality_status)} ${row.comparison_status ? renderBadge(row.comparison_status) : ""}</td>`;
        pppProductsBody.appendChild(tr);
      }

      const ddResidualBody = document.querySelector("#dd-residual-table tbody");
      ddResidualBody.innerHTML = "";
      const firstDDResidual = (overview.dd_residual_summaries || []).find((row) => row.html_report);
      const ddResidualFrame = document.getElementById("dd-residual-frame");
      const ddResidualEmpty = document.getElementById("dd-residual-empty");
      const ddResidualMetrics = document.getElementById("dd-residual-metrics");
      if (firstDDResidual) {
        renderMetricGrid(ddResidualMetrics, [
          ["epochs", firstDDResidual.epochs ?? "n/a"],
          ["satellite pairs", firstDDResidual.satellite_pairs ?? "n/a"],
          ["tracks", firstDDResidual.pair_frequency_kind_tracks ?? "n/a"],
          ["phase p95", `${formatMaybeNumber(firstDDResidual.phase_p95_abs_residual_m, 6, " m")} / ${formatMaybeNumber(firstDDResidual.phase_p95_abs_normalized_residual, 3, " sigma")}`],
          ["code p95", `${formatMaybeNumber(firstDDResidual.code_p95_abs_residual_m, 6, " m")} / ${formatMaybeNumber(firstDDResidual.code_p95_abs_normalized_residual, 3, " sigma")}`],
          ["quality", firstDDResidual.quality_status || "n/a"],
        ]);
        ddResidualFrame.src = `/artifact?path=${encodeURIComponent(firstDDResidual.html_report)}`;
        ddResidualFrame.style.display = "block";
        ddResidualEmpty.innerHTML = [
          artifactLink(firstDDResidual.html_report, "open full report"),
          firstDDResidual.csv_path ? artifactLink(firstDDResidual.csv_path, "csv") : null,
          firstDDResidual.top_pairs_csv ? artifactLink(firstDDResidual.top_pairs_csv, "top-pairs") : null,
          firstDDResidual.summary_path ? artifactLink(firstDDResidual.summary_path, "summary") : null,
        ].filter(Boolean).join(" / ");
      } else {
        renderMetricGrid(ddResidualMetrics, [["status", "n/a"], ["summary", "unavailable"]]);
        ddResidualFrame.removeAttribute("src");
        ddResidualFrame.style.display = "none";
        ddResidualEmpty.textContent = "No DD residual HTML report found. Run gnss dd-residuals with --html-report.";
      }
      for (const row of overview.dd_residual_summaries || []) {
        const artifacts = [
          row.html_report ? artifactLink(row.html_report, "html") : null,
          row.csv_path ? artifactLink(row.csv_path, "csv") : null,
          row.top_pairs_csv ? artifactLink(row.top_pairs_csv, "top-pairs") : null,
          row.summary_path ? artifactLink(row.summary_path, "summary") : null,
        ].filter(Boolean).join(" / ");
        const coverage = [
          `${row.epochs ?? "n/a"} epochs`,
          `${row.satellite_pairs ?? "n/a"} pairs`,
          `${row.pair_frequency_kind_tracks ?? "n/a"} tracks`,
          `${row.rows ?? "n/a"} rows`,
        ].join(" / ");
        const tr = document.createElement("tr");
        tr.innerHTML = `
          <td>${artifactLink(row.summary_path || row._path, row._path || "n/a")}</td>
          <td>${coverage}</td>
          <td>${formatMaybeNumber(row.phase_p95_abs_residual_m, 6, " m")}<br><span class="tiny">${formatMaybeNumber(row.phase_p95_abs_normalized_residual, 3, " sigma")}</span></td>
          <td>${formatMaybeNumber(row.code_p95_abs_residual_m, 6, " m")}<br><span class="tiny">${formatMaybeNumber(row.code_p95_abs_normalized_residual, 3, " sigma")}</span></td>
          <td>${formatMaybeNumber(row.max_abs_residual_m, 6, " m")}<br><span class="tiny">${formatMaybeNumber(row.max_abs_normalized_residual, 3, " sigma")}</span></td>
          <td>${row.suppressed_rows ?? "n/a"}</td>
          <td>${renderBadge(row.quality_status)}</td>
          <td>${artifacts || "n/a"}</td>`;
        ddResidualBody.appendChild(tr);
      }

      const visibilityBody = document.querySelector("#visibility-table tbody");
      visibilityBody.innerHTML = "";
      for (const row of overview.visibility_summaries || []) {
        const tr = document.createElement("tr");
        tr.innerHTML = `
          <td>${row._path || "n/a"}</td>
          <td>${row.epochs_processed ?? "n/a"}</td>
          <td>${row.rows_written ?? "n/a"}</td>
          <td>${row.unique_satellites ?? "n/a"}</td>
          <td>${formatMaybeNumber(row.mean_satellites_per_epoch, 2)}</td>
          <td>${formatMaybeNumber(row.mean_elevation_deg, 2, " deg")}</td>
          <td>${formatMaybeNumber(row.mean_snr_dbhz, 2, " dB-Hz")}</td>`;
        visibilityBody.appendChild(tr);
      }

      const visibilityImage = document.getElementById("visibility-image");
      const firstVisibility = (overview.visibility_summaries || []).find((row) => row.csv_path);
      if (firstVisibility && firstVisibility.png_path) {
        visibilityImage.src = `/artifact?path=${encodeURIComponent(firstVisibility.png_path)}`;
        visibilityImage.style.display = "block";
      } else {
        visibilityImage.removeAttribute("src");
        visibilityImage.style.display = "none";
      }
      if (firstVisibility) {
        const visibilityPayload = await fetchJson(`/api/visibility?path=${encodeURIComponent(firstVisibility.csv_path)}`);
        drawVisibility(document.getElementById("visibility-canvas"), visibilityPayload);
      } else {
        drawVisibility(document.getElementById("visibility-canvas"), {available: false, rows: [], error: "No visibility CSV"});
      }

      const manifestBody = document.querySelector("#artifact-manifest-table tbody");
      manifestBody.innerHTML = "";
      for (const bundle of overview.artifact_manifest || []) {
        const artifactLinks = Object.entries(bundle.artifacts || {})
          .filter(([, value]) => value)
          .map(([key, value]) => smartLink(value, key))
          .join(" / ");
        const metricSummary = bundleMetricSummary(bundle);
        const statusParts = [];
        if (bundle.quality_status) statusParts.push(renderBadge(bundle.quality_status));
        if (bundle.runtime_status) statusParts.push(renderBadge(bundle.runtime_status));
        if (bundle.comparison_status) statusParts.push(renderBadge(bundle.comparison_status));
        if (!statusParts.length && bundle.status) statusParts.push(renderBadge(bundle.status));
        const tr = document.createElement("tr");
        tr.innerHTML = `
          <td>${bundle.category || "n/a"}</td>
          <td>${bundle.label || "n/a"}<br><span class="tiny">${bundle.headline || ""}</span></td>
          <td>${statusParts.join(" ") || "n/a"}</td>
          <td>${smartLink(bundle.summary_json, bundle.summary_json || "n/a")}</td>
          <td>${artifactLinks || "n/a"}${metricSummary ? `<br><span class="tiny">${metricSummary}</span>` : ""}</td>`;
        manifestBody.appendChild(tr);
      }

      const libSolution = await fetchJson("/api/solution?name=libgnsspp");
      const rtklibSolution = await fetchJson("/api/solution?name=rtklib");
      drawTrajectory(document.getElementById("lib-canvas"), libSolution);
      drawTrajectory(document.getElementById("rtk-canvas"), rtklibSolution);
      const rtklibAnalysis = rtklibSolution.analysis || {};
      renderMetricGrid(document.getElementById("rtklib-analysis-metrics"), [
        ["epochs", rtklibAnalysis.epochs ?? "n/a"],
        ["duration", formatMaybeNumber(rtklibAnalysis.duration_s, 1, " s")],
        ["cadence", formatMaybeNumber(rtklibAnalysis.median_cadence_s, 2, " s")],
        ["FIX rate", formatMaybeNumber(rtklibAnalysis.fix_rate_pct, 2, "%")],
        ["TTFF", formatMaybeNumber(rtklibAnalysis.ttff_s, 1, " s")],
        ["ratio p95", formatMaybeNumber(rtklibAnalysis.p95_ratio, 2)],
        ["satellites", `${rtklibAnalysis.min_satellites ?? "n/a"}-${rtklibAnalysis.max_satellites ?? "n/a"}`],
        ["gaps/transitions", `${rtklibAnalysis.large_gap_count ?? "n/a"} / ${rtklibAnalysis.status_transition_count ?? "n/a"}`],
      ]);
      document.getElementById("rtklib-analysis-source").innerHTML = rtklibSolution.path
        ? artifactLink(rtklibSolution.path, rtklibSolution.path)
        : (rtklibSolution.error || "RTKLIB .pos unavailable");
      drawSolutionStatusTimeline(document.getElementById("rtklib-status-canvas"), rtklibSolution);
      drawSolutionRatioSatellites(document.getElementById("rtklib-ratio-canvas"), rtklibSolution);
      await refreshStatus();
      setInterval(refreshStatus, 2000);
    }

    init().catch((error) => {
      document.body.innerHTML = `<main><pre class="status-pre">${String(error)}</pre></main>`;
    });
  </script>
</body>
</html>
"""


def make_handler(args: argparse.Namespace):
    root_dir = args.root.resolve()
    lib_pos_path = resolve_libgnss_pos_path(args.lib_pos, root_dir)
    rtklib_pos_path = resolve_rtklib_pos_path(args.rtklib_pos, root_dir)

    class Handler(BaseHTTPRequestHandler):
        def log_message(self, fmt: str, *values: Any) -> None:
            return

        def _write(self, body: bytes, content_type: str, status: HTTPStatus = HTTPStatus.OK) -> None:
            self.send_response(status)
            self.send_header("Content-Type", content_type)
            self.send_header("Content-Length", str(len(body)))
            self.send_header("Cache-Control", "no-store")
            self.end_headers()
            self.wfile.write(body)

        def _write_json(self, payload: dict[str, Any], status: HTTPStatus = HTTPStatus.OK) -> None:
            self._write(json.dumps(payload, indent=2, sort_keys=True).encode("utf-8"),
                        "application/json; charset=utf-8", status)

        def do_GET(self) -> None:
            parsed = urlparse(self.path)
            query = parse_qs(parsed.query)

            try:
                if parsed.path == "/":
                    self._write(render_html().encode("utf-8"), "text/html; charset=utf-8")
                    return
                if parsed.path == "/api/health":
                    self._write_json({"ok": True})
                    return
                if parsed.path == "/api/overview":
                    self._write_json(build_overview(args))
                    return
                if parsed.path == "/api/status":
                    if args.rcv_status is None:
                        self._write_json({"available": False, "message": "No --rcv-status configured."})
                        return
                    raw = load_json(args.rcv_status)
                    if raw is None:
                        self._write_json(
                            {
                                "available": False,
                                "message": f"Status file unavailable: {args.rcv_status}",
                            }
                        )
                        return
                    self._write_json(
                        {
                            "available": True,
                            "state": raw.get("state"),
                            "pid": raw.get("pid"),
                            "pid_running": raw.get("pid_running"),
                            "uptime_seconds": raw.get("uptime_seconds"),
                            "restart_count": raw.get("restart_count"),
                            "raw": raw,
                        }
                    )
                    return
                if parsed.path == "/api/solution":
                    name = query.get("name", [""])[0]
                    if name == "libgnsspp":
                        self._write_json(build_solution_payload(name, lib_pos_path, root_dir, "libgnss++"))
                        return
                    if name == "rtklib":
                        self._write_json(build_solution_payload(name, rtklib_pos_path, root_dir, "RTKLIB"))
                        return
                    self._write_json({"error": "unknown solution name"}, HTTPStatus.BAD_REQUEST)
                    return
                if parsed.path == "/api/visibility":
                    csv_arg = query.get("path", [""])[0]
                    if not csv_arg:
                        self._write_json({"error": "missing visibility CSV path"}, HTTPStatus.BAD_REQUEST)
                        return
                    try:
                        csv_path = resolve_under_root(root_dir, csv_arg)
                    except ValueError:
                        self._write_json({"error": "visibility CSV path escapes artifact root"}, HTTPStatus.BAD_REQUEST)
                        return
                    if not csv_path.exists():
                        self._write_json({"error": f"visibility CSV not found: {csv_arg}"}, HTTPStatus.NOT_FOUND)
                        return
                    self._write_json(
                        {
                            "available": True,
                            "path": relative_display(csv_path, root_dir),
                            "rows": load_visibility_rows(csv_path),
                        }
                    )
                    return
                if parsed.path == "/api/moving-base-matches":
                    csv_arg = query.get("path", [""])[0]
                    if not csv_arg:
                        self._write_json({"error": "missing moving-base CSV path"}, HTTPStatus.BAD_REQUEST)
                        return
                    try:
                        csv_path = resolve_under_root(root_dir, csv_arg)
                    except ValueError:
                        self._write_json(
                            {"error": "moving-base CSV path escapes artifact root"},
                            HTTPStatus.BAD_REQUEST,
                        )
                        return
                    if not csv_path.exists():
                        self._write_json(
                            {"error": f"moving-base CSV not found: {csv_arg}"},
                            HTTPStatus.NOT_FOUND,
                        )
                        return
                    self._write_json(
                        {
                            "available": True,
                            "path": relative_display(csv_path, root_dir),
                            "rows": load_moving_base_matches(csv_path),
                        }
                    )
                    return
                if parsed.path == "/artifact":
                    artifact_arg = query.get("path", [""])[0]
                    if not artifact_arg:
                        self._write_json({"error": "missing artifact path"}, HTTPStatus.BAD_REQUEST)
                        return
                    try:
                        artifact_path = resolve_under_root(root_dir, artifact_arg)
                    except ValueError:
                        self._write_json({"error": "artifact path escapes artifact root"}, HTTPStatus.BAD_REQUEST)
                        return
                    if not artifact_path.exists():
                        self._write_json({"error": f"artifact not found: {artifact_arg}"}, HTTPStatus.NOT_FOUND)
                        return
                    suffix = artifact_path.suffix.lower()
                    content_type = {
                        ".png": "image/png",
                        ".jpg": "image/jpeg",
                        ".jpeg": "image/jpeg",
                        ".svg": "image/svg+xml",
                        ".json": "application/json; charset=utf-8",
                        ".html": "text/html; charset=utf-8",
                        ".pos": "text/plain; charset=utf-8",
                        ".csv": "text/plain; charset=utf-8",
                        ".txt": "text/plain; charset=utf-8",
                    }.get(suffix, "application/octet-stream")
                    self._write(artifact_path.read_bytes(), content_type)
                    return

                self._write_json({"error": "not found"}, HTTPStatus.NOT_FOUND)
            except Exception as exc:  # pragma: no cover - tested through API failure shape
                self._write_json({"error": str(exc)}, HTTPStatus.INTERNAL_SERVER_ERROR)

    return Handler


def main() -> int:
    args = parse_args()
    server = ThreadingHTTPServer((args.host, args.port), make_handler(args))
    shutdown_started = False
    previous_handlers: dict[int, Any] = {}

    def request_shutdown(signum: int, _frame: Any) -> None:
        nonlocal shutdown_started
        if shutdown_started:
            return
        shutdown_started = True
        threading.Thread(target=server.shutdown, daemon=True).start()

    for sig in (signal.SIGINT, signal.SIGTERM):
        previous_handlers[sig] = signal.getsignal(sig)
        signal.signal(sig, request_shutdown)

    bound_host, bound_port = server.server_address[:2]
    if args.port_file is not None:
        args.port_file.parent.mkdir(parents=True, exist_ok=True)
        args.port_file.write_text(f"{bound_port}\n", encoding="utf-8")
    print(f"Serving gnss web at http://{bound_host}:{bound_port}", flush=True)
    try:
        server.serve_forever(poll_interval=0.2)
    except KeyboardInterrupt:
        pass
    finally:
        for sig, handler in previous_handlers.items():
            signal.signal(sig, handler)
        server.server_close()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
