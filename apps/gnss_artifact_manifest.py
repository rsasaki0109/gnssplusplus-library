#!/usr/bin/env python3
"""Build a compact artifact manifest from generated summary JSON files."""

from __future__ import annotations

import argparse
import json
import os
from datetime import datetime, timezone
from pathlib import Path
from typing import Any


ROOT_DIR = Path(__file__).resolve().parent.parent


def default_root_dir() -> Path:
    if (ROOT_DIR / "output").exists():
        return ROOT_DIR
    return Path.cwd()


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        prog=os.environ.get("GNSS_CLI_NAME"),
        description="Build an artifact manifest from generated summary JSON files.",
    )
    parser.add_argument("--root", type=Path, default=default_root_dir(), help="Artifact root directory.")
    parser.add_argument(
        "--output",
        type=Path,
        default=None,
        help="Output manifest JSON path (default: <root>/output/artifact_manifest.json).",
    )
    parser.add_argument("--ppc-summary-glob", default="output/ppc_*_summary.json")
    parser.add_argument("--live-summary-glob", default="output/live*_summary.json")
    parser.add_argument("--visibility-summary-glob", default="output/visibility*_summary.json")
    parser.add_argument("--moving-base-summary-glob", default="output/*moving_base_summary.json")
    parser.add_argument("--ppp-products-summary-glob", default="output/*ppp*_products*_summary.json")
    parser.add_argument("--ppc-spp-policy-suite-glob", default="output/**/*_suite_summary.json")
    parser.add_argument("--quiet", action="store_true", help="Suppress non-summary prints.")
    return parser.parse_args()


def load_json(path: Path) -> dict[str, Any] | None:
    if not path.exists():
        return None
    try:
        payload = json.loads(path.read_text(encoding="utf-8"))
    except (OSError, json.JSONDecodeError):
        return None
    if not isinstance(payload, dict):
        return None
    return payload


def relative_display(path: Path, root_dir: Path) -> str:
    try:
        return str(path.resolve().relative_to(root_dir.resolve()))
    except ValueError:
        return str(path)


def resolve_under_root(root_dir: Path, value: str) -> Path:
    candidate = Path(value)
    if candidate.is_absolute():
        candidate = candidate.resolve()
    else:
        candidate = (root_dir / candidate).resolve()
    candidate.relative_to(root_dir.resolve())
    return candidate


def normalize_artifact_path(root_dir: Path, value: Any) -> str | None:
    if not isinstance(value, str) or not value:
        return None
    if value.startswith(("http://", "https://")):
        return value
    try:
        return relative_display(resolve_under_root(root_dir, value), root_dir)
    except ValueError:
        return value


def normalize_commercial_receiver(root_dir: Path, payload: Any) -> dict[str, Any] | None:
    if not isinstance(payload, dict):
        return None
    normalized = dict(payload)
    for key in ("solution_pos", "matched_csv"):
        normalized[key] = normalize_artifact_path(root_dir, normalized.get(key))
    return normalized


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


def classify_ppp_products_status(converged: Any, p95_position_error_m: Any, solution_rate_pct: Any) -> str:
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


def build_ppc_entries(root_dir: Path, pattern: str) -> list[dict[str, Any]]:
    entries: list[dict[str, Any]] = []
    for path in sorted(root_dir.glob(pattern)):
        payload = load_json(path)
        if payload is None:
            continue
        quality_status = classify_accuracy_status(payload.get("p95_h_m"))
        runtime_status = classify_realtime_status(payload.get("realtime_factor"))
        commercial_receiver = normalize_commercial_receiver(root_dir, payload.get("commercial_receiver"))
        commercial_delta = payload.get("delta_vs_commercial_receiver")
        if not isinstance(commercial_delta, dict):
            commercial_delta = None
        comparison_status = classify_comparison_status(
            commercial_delta.get("median_h_m") if commercial_delta else None,
            commercial_delta.get("p95_h_m") if commercial_delta else None,
            commercial_delta.get("max_h_m") if commercial_delta else None,
            commercial_delta.get("p95_abs_up_m") if commercial_delta else None,
        )
        entries.append(
            {
                "category": "ppc",
                "label": path.name,
                "summary_json": relative_display(path, root_dir),
                "status": quality_status,
                "quality_status": quality_status,
                "runtime_status": runtime_status,
                "comparison_status": comparison_status,
                "headline": (
                    f"{payload.get('matched_epochs', 'n/a')} matched / "
                    f"{payload.get('fix_rate_pct', 'n/a')} fix / "
                    f"{payload.get('p95_h_m', 'n/a')} p95 H"
                ),
                "artifacts": {
                    "summary": relative_display(path, root_dir),
                    "solution": normalize_artifact_path(root_dir, payload.get("solution_pos")),
                    "reference": normalize_artifact_path(root_dir, payload.get("reference_pos")),
                    "rtklib": normalize_artifact_path(root_dir, payload.get("rtklib_solution_pos")),
                    "commercial_solution": (
                        commercial_receiver.get("solution_pos") if commercial_receiver else None
                    ),
                    "commercial_matches": (
                        commercial_receiver.get("matched_csv") if commercial_receiver else None
                    ),
                },
                "metrics": {
                    "matched_epochs": payload.get("matched_epochs"),
                    "fix_rate_pct": payload.get("fix_rate_pct"),
                    "median_h_m": payload.get("median_h_m"),
                    "p95_h_m": payload.get("p95_h_m"),
                    "solver_wall_time_s": payload.get("solver_wall_time_s"),
                    "realtime_factor": payload.get("realtime_factor"),
                    "effective_epoch_rate_hz": payload.get("effective_epoch_rate_hz"),
                    "commercial_receiver": commercial_receiver,
                    "delta_vs_commercial_receiver": commercial_delta,
                },
            }
        )
    return entries


def build_live_entries(root_dir: Path, pattern: str) -> list[dict[str, Any]]:
    entries: list[dict[str, Any]] = []
    for path in sorted(root_dir.glob(pattern)):
        payload = load_json(path)
        if payload is None:
            continue
        metrics = payload.get("metrics")
        if not isinstance(metrics, dict):
            continue
        runtime_status = classify_realtime_status(metrics.get("realtime_factor"))
        entries.append(
            {
                "category": "live",
                "label": path.name,
                "summary_json": relative_display(path, root_dir),
                "status": runtime_status,
                "runtime_status": runtime_status,
                "headline": (
                    f"{metrics.get('termination', 'n/a')} / "
                    f"{metrics.get('written_solutions', 'n/a')} written / "
                    f"{metrics.get('realtime_factor', 'n/a')}x"
                ),
                "artifacts": {
                    "summary": relative_display(path, root_dir),
                    "solution": normalize_artifact_path(root_dir, metrics.get("solution_pos")),
                    "log": normalize_artifact_path(root_dir, metrics.get("log_out")),
                },
                "metrics": {
                    "termination": metrics.get("termination"),
                    "aligned_epochs": metrics.get("aligned_epochs"),
                    "written_solutions": metrics.get("written_solutions"),
                    "fixed_solutions": metrics.get("fixed_solutions"),
                    "realtime_factor": metrics.get("realtime_factor"),
                    "effective_epoch_rate_hz": metrics.get("effective_epoch_rate_hz"),
                    "rover_decoder_errors": metrics.get("rover_decoder_errors"),
                    "base_decoder_errors": metrics.get("base_decoder_errors"),
                },
            }
        )
    return entries


def build_visibility_entries(root_dir: Path, pattern: str) -> list[dict[str, Any]]:
    entries: list[dict[str, Any]] = []
    for path in sorted(root_dir.glob(pattern)):
        payload = load_json(path)
        if payload is None:
            continue
        entries.append(
            {
                "category": "visibility",
                "label": path.name,
                "summary_json": relative_display(path, root_dir),
                "status": "available",
                "headline": (
                    f"{payload.get('rows_written', 'n/a')} rows / "
                    f"{payload.get('unique_satellites', 'n/a')} sats / "
                    f"{payload.get('mean_elevation_deg', 'n/a')} deg mean elev"
                ),
                "artifacts": {
                    "summary": relative_display(path, root_dir),
                    "csv": normalize_artifact_path(root_dir, payload.get("csv")),
                    "png": normalize_artifact_path(root_dir, payload.get("png")),
                },
                "metrics": {
                    "epochs_processed": payload.get("epochs_processed"),
                    "epochs_with_rows": payload.get("epochs_with_rows"),
                    "rows_written": payload.get("rows_written"),
                    "unique_satellites": payload.get("unique_satellites"),
                    "mean_satellites_per_epoch": payload.get("mean_satellites_per_epoch"),
                    "mean_elevation_deg": payload.get("mean_elevation_deg"),
                    "mean_snr_dbhz": payload.get("mean_snr_dbhz"),
                },
            }
        )
    return entries


def build_moving_base_entries(root_dir: Path, pattern: str) -> list[dict[str, Any]]:
    entries: list[dict[str, Any]] = []
    for path in sorted(root_dir.glob(pattern)):
        payload = load_json(path)
        if payload is None:
            continue
        quality_status = classify_baseline_status(payload.get("p95_baseline_error_m"))
        runtime_status = classify_realtime_status(payload.get("realtime_factor"))
        commercial_receiver = normalize_commercial_receiver(root_dir, payload.get("commercial_receiver"))
        commercial_delta = payload.get("libgnss_vs_commercial_receiver")
        if not isinstance(commercial_delta, dict):
            commercial_delta = None
        comparison_status = classify_comparison_status(
            commercial_delta.get("median_baseline_error_m_delta") if commercial_delta else None,
            commercial_delta.get("p95_baseline_error_m_delta") if commercial_delta else None,
            commercial_delta.get("max_baseline_error_m_delta") if commercial_delta else None,
            commercial_delta.get("p95_heading_error_deg_delta") if commercial_delta else None,
        )
        entries.append(
            {
                "category": "moving-base",
                "label": path.name,
                "summary_json": relative_display(path, root_dir),
                "status": quality_status,
                "quality_status": quality_status,
                "runtime_status": runtime_status,
                "comparison_status": comparison_status,
                "headline": (
                    f"{payload.get('matched_epochs', 'n/a')} matched / "
                    f"{payload.get('fix_rate_pct', 'n/a')} fix / "
                    f"{payload.get('p95_baseline_error_m', 'n/a')} p95 baseline"
                ),
                "artifacts": {
                    "summary": relative_display(path, root_dir),
                    "solution": normalize_artifact_path(root_dir, payload.get("solution_pos")),
                    "matched_csv": normalize_artifact_path(root_dir, payload.get("matched_csv")),
                    "prepare": normalize_artifact_path(root_dir, payload.get("prepare_summary_json")),
                    "products": normalize_artifact_path(root_dir, payload.get("products_summary_json")),
                    "plot": normalize_artifact_path(root_dir, payload.get("plot_png")),
                    "nav_rinex": normalize_artifact_path(root_dir, payload.get("nav_rinex")),
                    "input": normalize_artifact_path(root_dir, payload.get("input")),
                    "input_url": normalize_artifact_path(root_dir, payload.get("input_url")),
                    "commercial_solution": (
                        commercial_receiver.get("solution_pos")
                        if commercial_receiver
                        else normalize_artifact_path(root_dir, payload.get("commercial_receiver_csv"))
                    ),
                    "commercial_matches": (
                        commercial_receiver.get("matched_csv")
                        if commercial_receiver
                        else normalize_artifact_path(root_dir, payload.get("commercial_receiver_matched_csv"))
                    ),
                },
                "metrics": {
                    "matched_epochs": payload.get("matched_epochs"),
                    "valid_epochs": payload.get("valid_epochs"),
                    "fix_rate_pct": payload.get("fix_rate_pct"),
                    "median_baseline_error_m": payload.get("median_baseline_error_m"),
                    "p95_baseline_error_m": payload.get("p95_baseline_error_m"),
                    "p95_heading_error_deg": payload.get("p95_heading_error_deg"),
                    "termination": payload.get("termination"),
                    "realtime_factor": payload.get("realtime_factor"),
                    "effective_epoch_rate_hz": payload.get("effective_epoch_rate_hz"),
                    "commercial_receiver": commercial_receiver,
                    "libgnss_vs_commercial_receiver": commercial_delta,
                },
            }
        )
    return entries


def build_ppp_products_entries(root_dir: Path, pattern: str) -> list[dict[str, Any]]:
    entries: list[dict[str, Any]] = []
    for path in sorted(root_dir.glob(pattern)):
        payload = load_json(path)
        if payload is None:
            continue
        p95_error = payload.get("p95_position_error_m")
        if not isinstance(p95_error, (int, float)):
            p95_error = payload.get("max_position_error_m")
        quality_status = classify_ppp_products_status(
            payload.get("ppp_converged"),
            p95_error,
            payload.get("ppp_solution_rate_pct"),
        )
        comparison_status = classify_comparison_status(
            payload.get("libgnss_minus_malib_mean_error_m"),
            payload.get("libgnss_minus_malib_p95_error_m"),
            payload.get("libgnss_minus_malib_max_error_m"),
        )
        comparison_target = payload.get("comparison_target")
        if comparison_target is None and payload.get("malib_solution_pos"):
            comparison_target = "MALIB"
        entries.append(
            {
                "category": "ppp-products",
                "label": path.name,
                "summary_json": relative_display(path, root_dir),
                "status": quality_status,
                "quality_status": quality_status,
                "comparison_status": comparison_status,
                "headline": (
                    f"{payload.get('dataset', payload.get('products_signoff_profile', 'n/a'))} / "
                    f"{payload.get('ppp_solution_rate_pct', 'n/a')} rate / "
                    f"{payload.get('ppp_convergence_time_s', 'n/a')} s conv"
                ),
                "artifacts": {
                    "summary": relative_display(path, root_dir),
                    "solution": normalize_artifact_path(root_dir, payload.get("solution_pos")),
                    "reference": normalize_artifact_path(root_dir, payload.get("reference_csv")),
                    "run_dir": normalize_artifact_path(root_dir, payload.get("run_dir")),
                    "comparison_csv": normalize_artifact_path(root_dir, payload.get("comparison_csv")),
                    "comparison_png": normalize_artifact_path(root_dir, payload.get("comparison_png")),
                    "sp3": normalize_artifact_path(root_dir, payload.get("sp3")),
                    "clk": normalize_artifact_path(root_dir, payload.get("clk")),
                    "ionex": normalize_artifact_path(root_dir, payload.get("ionex")),
                    "dcb": normalize_artifact_path(root_dir, payload.get("dcb")),
                    "malib": normalize_artifact_path(root_dir, payload.get("malib_solution_pos")),
                },
                "metrics": {
                    "dataset": payload.get("dataset"),
                    "profile": payload.get("products_signoff_profile"),
                    "fetched_product_date": payload.get("fetched_product_date"),
                    "ppp_solution_rate_pct": payload.get("ppp_solution_rate_pct"),
                    "ppp_converged": payload.get("ppp_converged"),
                    "ppp_convergence_time_s": payload.get("ppp_convergence_time_s"),
                    "mean_position_error_m": payload.get("mean_position_error_m"),
                    "p95_position_error_m": payload.get("p95_position_error_m"),
                    "max_position_error_m": payload.get("max_position_error_m"),
                    "ionex_corrections": payload.get("ionex_corrections"),
                    "dcb_corrections": payload.get("dcb_corrections"),
                    "common_epoch_pairs": payload.get("common_epoch_pairs"),
                    "comparison_target": comparison_target,
                    "libgnss_minus_malib_mean_error_m": payload.get("libgnss_minus_malib_mean_error_m"),
                    "libgnss_minus_malib_p95_error_m": payload.get("libgnss_minus_malib_p95_error_m"),
                    "libgnss_minus_malib_max_error_m": payload.get("libgnss_minus_malib_max_error_m"),
                },
            }
        )
    return entries


def policy_suite_status(policy_report: dict[str, Any] | None) -> str:
    checks = policy_report.get("checks") if policy_report else None
    if not isinstance(checks, dict):
        return "n/a"
    return "passed" if checks.get("passed") is True else "failed"


def policy_suite_runs(policy_report: dict[str, Any] | None) -> list[dict[str, Any]]:
    runs = policy_report.get("runs") if policy_report else None
    if not isinstance(runs, list):
        return []
    rows: list[dict[str, Any]] = []
    for run in runs:
        if not isinstance(run, dict):
            continue
        rows.append(
            {
                "label": run.get("label"),
                "selected_rate_mps": run.get("selected_rate_mps"),
                "selected_min_jump_m": run.get("selected_min_jump_m"),
                "policy_positioning_rate_pct": run.get("policy_positioning_rate_pct"),
                "positioning_drop_pct": run.get("positioning_drop_pct"),
                "policy_p95_h_m": run.get("policy_p95_h_m"),
                "p95_h_delta_m": run.get("p95_h_delta_m"),
                "jump_gate_rejected_epochs": run.get("jump_gate_rejected_epochs"),
                "bridge_inserted_epochs": run.get("bridge_inserted_epochs"),
            }
        )
    return rows


def numeric_values(rows: list[dict[str, Any]], key: str) -> list[float]:
    values: list[float] = []
    for row in rows:
        value = row.get(key)
        if isinstance(value, (int, float)):
            values.append(float(value))
    return values


def normalize_policy_suite_run_artifacts(root_dir: Path, runs: Any) -> list[dict[str, Any]]:
    if not isinstance(runs, list):
        return []
    normalized_runs: list[dict[str, Any]] = []
    for run in runs:
        if not isinstance(run, dict):
            continue
        normalized_runs.append(
            {
                "label": run.get("label"),
                "reference": normalize_artifact_path(root_dir, run.get("reference_csv")),
                "input": normalize_artifact_path(root_dir, run.get("input_pos")),
                "baseline": normalize_artifact_path(root_dir, run.get("baseline_pos")),
                "sweep_json": normalize_artifact_path(root_dir, run.get("sweep_json")),
                "sweep_csv": normalize_artifact_path(root_dir, run.get("sweep_csv")),
                "filtered_pos": normalize_artifact_path(root_dir, run.get("filtered_pos")),
                "compare_summary": normalize_artifact_path(root_dir, run.get("compare_summary_json")),
                "compare_csv": normalize_artifact_path(root_dir, run.get("compare_csv")),
                "compare_matches": normalize_artifact_path(root_dir, run.get("compare_matched_csv")),
                "compare_png": normalize_artifact_path(root_dir, run.get("compare_png")),
            }
        )
    return normalized_runs


def build_ppc_spp_policy_suite_entries(root_dir: Path, pattern: str) -> list[dict[str, Any]]:
    entries: list[dict[str, Any]] = []
    for path in sorted(root_dir.glob(pattern)):
        payload = load_json(path)
        if payload is None:
            continue
        if payload.get("dry_run") is True:
            continue
        if not payload.get("policy_report_json") or not isinstance(payload.get("runs"), list):
            continue

        policy_report_path_text = payload.get("policy_report_json")
        policy_report_path: Path | None = None
        policy_report: dict[str, Any] | None = None
        if isinstance(policy_report_path_text, str) and policy_report_path_text:
            try:
                policy_report_path = resolve_under_root(root_dir, policy_report_path_text)
            except ValueError:
                policy_report_path = Path(policy_report_path_text)
            policy_report = load_json(policy_report_path)

        policy_runs = policy_suite_runs(policy_report)
        if not policy_runs and isinstance(payload.get("policy_runs"), list):
            policy_runs = [
                run for run in payload["policy_runs"]
                if isinstance(run, dict)
            ]
        p95_deltas = numeric_values(policy_runs, "p95_h_delta_m")
        positioning_drops = numeric_values(policy_runs, "positioning_drop_pct")
        worst_p95_delta = payload.get("worst_p95_h_delta_m")
        if not isinstance(worst_p95_delta, (int, float)):
            worst_p95_delta = max(p95_deltas) if p95_deltas else None
        worst_positioning_drop = payload.get("worst_positioning_drop_pct")
        if not isinstance(worst_positioning_drop, (int, float)):
            worst_positioning_drop = max(positioning_drops) if positioning_drops else None
        checks = payload.get("checks")
        if not isinstance(checks, dict):
            checks = policy_report.get("checks") if isinstance(policy_report, dict) else None
        if not isinstance(checks, dict):
            checks = None
        if checks is not None:
            status = "passed" if checks.get("passed") is True else "failed"
        else:
            status = policy_suite_status(policy_report)
        comparison_status = classify_comparison_status(*p95_deltas)
        label = payload.get("prefix") if isinstance(payload.get("prefix"), str) else path.stem

        entries.append(
            {
                "category": "ppc-spp-policy-suite",
                "label": label,
                "summary_json": relative_display(path, root_dir),
                "status": status,
                "comparison_status": comparison_status,
                "headline": (
                    f"{payload.get('run_count', len(policy_runs))} runs / "
                    f"{status} / worst p95 delta "
                    f"{worst_p95_delta if worst_p95_delta is not None else 'n/a'} m"
                ),
                "artifacts": {
                    "summary": relative_display(path, root_dir),
                    "policy_report": normalize_artifact_path(root_dir, payload.get("policy_report_json")),
                    "policy_csv": normalize_artifact_path(root_dir, payload.get("policy_report_csv")),
                    "runs": normalize_policy_suite_run_artifacts(root_dir, payload.get("runs")),
                },
                "metrics": {
                    "run_count": payload.get("run_count"),
                    "prefix": payload.get("prefix"),
                    "rates_mps": payload.get("rates_mps"),
                    "min_jumps_m": payload.get("min_jumps_m"),
                    "bridge_max_gap_s": payload.get("bridge_max_gap_s"),
                    "bridge_max_anchor_speed_mps": payload.get("bridge_max_anchor_speed_mps"),
                    "max_positioning_drop_pct": payload.get("max_positioning_drop_pct"),
                    "min_positioning_rate_pct": payload.get("min_positioning_rate_pct"),
                    "max_p95_delta_m": payload.get("max_p95_delta_m"),
                    "checks": checks,
                    "worst_p95_h_delta_m": worst_p95_delta,
                    "worst_positioning_drop_pct": worst_positioning_drop,
                    "runs": policy_runs,
                },
            }
        )
    return entries


def main() -> int:
    args = parse_args()
    root_dir = args.root.resolve()
    output_path = (args.output or (root_dir / "output" / "artifact_manifest.json")).resolve()
    output_path.parent.mkdir(parents=True, exist_ok=True)

    bundles: list[dict[str, Any]] = []
    bundles.extend(build_ppc_entries(root_dir, args.ppc_summary_glob))
    bundles.extend(build_live_entries(root_dir, args.live_summary_glob))
    bundles.extend(build_visibility_entries(root_dir, args.visibility_summary_glob))
    bundles.extend(build_moving_base_entries(root_dir, args.moving_base_summary_glob))
    bundles.extend(build_ppp_products_entries(root_dir, args.ppp_products_summary_glob))
    bundles.extend(build_ppc_spp_policy_suite_entries(root_dir, args.ppc_spp_policy_suite_glob))

    payload = {
        "root": str(root_dir),
        "generated_at_utc": datetime.now(timezone.utc).isoformat(),
        "bundle_count": len(bundles),
        "bundles": bundles,
    }
    output_path.write_text(json.dumps(payload, indent=2, sort_keys=True) + "\n", encoding="utf-8")

    if not args.quiet:
        print("Wrote artifact manifest.")
        print(f"  output: {output_path}")
        print(f"  bundles: {len(bundles)}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
