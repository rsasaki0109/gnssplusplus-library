#!/usr/bin/env python3
"""Local web UI for browsing libgnss++ benchmark artifacts and receiver status."""

from __future__ import annotations

import argparse
import csv
import json
import os
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
        "points": downsample_points(points),
    }


def build_overview(args: argparse.Namespace) -> dict[str, Any]:
    root_dir = args.root.resolve()
    odaiba_summary_path = resolve_path(args.odaiba_summary, root_dir, "output/odaiba_summary.json")
    lib_pos_path = resolve_path(args.lib_pos, root_dir, "output/rtk_solution.pos")
    rtklib_pos_path = resolve_path(args.rtklib_pos, root_dir, "output/driving_rtklib_rtk.pos")
    artifact_manifest_path = resolve_path(args.artifact_manifest, root_dir, "output/artifact_manifest.json")
    rcv_status = load_json(args.rcv_status) if args.rcv_status is not None else None

    ppc_summaries: list[dict[str, Any]] = []
    for path in sorted(root_dir.glob(args.ppc_summary_glob)):
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
    for path in sorted(root_dir.glob(args.live_summary_glob)):
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
    for path in sorted(root_dir.glob(args.visibility_summary_glob)):
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
    for path in sorted(root_dir.glob(args.moving_base_summary_glob)):
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
    for path in sorted(root_dir.glob(args.ppp_products_summary_glob)):
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

      drawTrajectory(document.getElementById("lib-canvas"), await fetchJson("/api/solution?name=libgnsspp"));
      drawTrajectory(document.getElementById("rtk-canvas"), await fetchJson("/api/solution?name=rtklib"));
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
    lib_pos_path = resolve_path(args.lib_pos, root_dir, "output/rtk_solution.pos")
    rtklib_pos_path = resolve_path(args.rtklib_pos, root_dir, "output/driving_rtklib_rtk.pos")

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
