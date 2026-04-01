#!/usr/bin/env python3
"""Run CLAS-oriented PPP-AR experiments with a comparable interface."""

from __future__ import annotations

import argparse
from dataclasses import asdict, dataclass
from datetime import datetime, timezone
import json
import math
import os
from pathlib import Path
import subprocess
import sys
import time
import tomllib
from typing import Any


ROOT_DIR = Path(__file__).resolve().parents[2]
APPS_DIR = ROOT_DIR / "apps"
DEFAULT_STRATEGIES_TOML = ROOT_DIR / "experiments" / "ppp_ar" / "strategies.toml"
DEFAULT_RESULTS_JSON = ROOT_DIR / "output" / "ppp_ar_experiments" / "results.json"


@dataclass(frozen=True)
class ExperimentInput:
    case_key: str
    dataset_label: str
    obs: Path
    nav: Path
    qzss_l6: Path | None
    ssr_csv: Path | None
    qzss_gps_week: int | None
    reference_ecef: tuple[float, float, float]
    max_epochs: int
    output_dir: Path
    mode: str
    estimate_troposphere: bool
    strategies: tuple[str, ...]


@dataclass(frozen=True)
class ExperimentSuite:
    label: str
    cases: tuple[ExperimentInput, ...]


@dataclass(frozen=True)
class ExperimentStrategy:
    key: str
    label: str
    design_style: str
    status: str
    description: str
    claslib_reference: str
    extra_args: tuple[str, ...]
    compact_flush_policy: str | None
    compact_atmos_merge_policy: str | None
    compact_atmos_subtype_merge_policy: str | None
    implementation_files: tuple[Path, ...]


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser()
    parser.add_argument("--config-toml", type=Path, required=True)
    parser.add_argument("--strategies-toml", type=Path, default=DEFAULT_STRATEGIES_TOML)
    parser.add_argument("--output-json", type=Path, default=DEFAULT_RESULTS_JSON)
    parser.add_argument("--markdown-out", type=Path, default=None)
    parser.add_argument("--strategy", action="append", default=[])
    parser.add_argument("--dry-run", action="store_true")
    return parser.parse_args()


def _load_toml(path: Path) -> dict[str, Any]:
    with path.open("rb") as handle:
        payload = tomllib.load(handle)
    if not isinstance(payload, dict):
        raise SystemExit(f"TOML file must be a table: {path}")
    return payload


def _resolve_path(base_dir: Path, value: str | None) -> Path | None:
    if value is None:
        return None
    path = Path(value)
    if not path.is_absolute():
        path = (base_dir / path).resolve()
    return path


def load_rinex_approx_position(path: Path) -> tuple[float, float, float]:
    with path.open("r", encoding="ascii", errors="ignore") as handle:
        for line in handle:
            if "APPROX POSITION XYZ" in line:
                return tuple(float(value) for value in line[:60].split()[:3])  # type: ignore[return-value]
            if "END OF HEADER" in line:
                break
    raise SystemExit(f"reference_ecef is missing and RINEX header has no APPROX POSITION XYZ: {path}")


def _parse_input_section(
    *,
    base_dir: Path,
    section: dict[str, Any],
    fallback_case_key: str,
) -> ExperimentInput:
    obs = _resolve_path(base_dir, str(section["obs"]))
    nav = _resolve_path(base_dir, str(section["nav"]))
    qzss_l6 = _resolve_path(base_dir, section.get("qzss_l6"))
    ssr_csv = _resolve_path(base_dir, section.get("ssr_csv"))
    output_dir = _resolve_path(base_dir, str(section.get("output_dir", "output/ppp_ar_experiments")))
    if section.get("reference_ecef") is None:
        reference = load_rinex_approx_position(obs)
    else:
        reference = tuple(float(value) for value in section["reference_ecef"])
        if len(reference) != 3:
            raise SystemExit("reference_ecef must have exactly 3 values")
    strategies = tuple(str(value) for value in section.get("strategies", ()))
    if not strategies:
        raise SystemExit("strategies must contain at least one strategy key")
    return ExperimentInput(
        case_key=str(section.get("case_key", fallback_case_key)),
        dataset_label=str(section.get("dataset_label", "CLAS PPP experiment")),
        obs=obs,
        nav=nav,
        qzss_l6=qzss_l6,
        ssr_csv=ssr_csv,
        qzss_gps_week=int(section["qzss_gps_week"]) if section.get("qzss_gps_week") is not None else None,
        reference_ecef=reference,
        max_epochs=int(section.get("max_epochs", 120)),
        output_dir=output_dir,
        mode=str(section.get("mode", "static")),
        estimate_troposphere=bool(section.get("estimate_troposphere", True)),
        strategies=strategies,
    )


def load_input_config(path: Path) -> ExperimentInput:
    payload = _load_toml(path)
    section = payload.get("clas_ppp_experiment", payload)
    if not isinstance(section, dict):
        raise SystemExit(f"[clas_ppp_experiment] must be a table in {path}")
    return _parse_input_section(
        base_dir=path.resolve().parent,
        section=section,
        fallback_case_key="default",
    )


def load_suite_config(path: Path) -> ExperimentSuite:
    payload = _load_toml(path)
    suite_section = payload.get("clas_ppp_suite")
    if suite_section is None:
        return ExperimentSuite(label="PPP-AR experiment suite", cases=(load_input_config(path),))
    if not isinstance(suite_section, dict):
        raise SystemExit(f"[clas_ppp_suite] must be a table in {path}")

    raw_cases = suite_section.get("cases")
    if not isinstance(raw_cases, list) or not raw_cases:
        raise SystemExit(f"[clas_ppp_suite] must contain a non-empty cases array in {path}")

    base_dir = path.resolve().parent
    defaults = {
        key: value
        for key, value in suite_section.items()
        if key not in {"label", "cases"}
    }
    default_output_dir = _resolve_path(base_dir, str(defaults.get("output_dir", "output/ppp_ar_experiments")))
    cases: list[ExperimentInput] = []
    for index, raw_case in enumerate(raw_cases, start=1):
        if not isinstance(raw_case, dict):
            raise SystemExit(f"clas_ppp_suite.cases[{index - 1}] must be an inline table")
        merged = {**defaults, **raw_case}
        fallback_case_key = str(raw_case.get("case_key", f"case_{index}"))
        if "output_dir" not in raw_case:
            merged["output_dir"] = str(default_output_dir / fallback_case_key)
        cases.append(
            _parse_input_section(
                base_dir=base_dir,
                section=merged,
                fallback_case_key=fallback_case_key,
            )
        )
    return ExperimentSuite(
        label=str(suite_section.get("label", "PPP-AR experiment suite")),
        cases=tuple(cases),
    )


def load_strategies(path: Path) -> dict[str, ExperimentStrategy]:
    payload = _load_toml(path)
    raw_strategies = payload.get("strategies")
    if not isinstance(raw_strategies, dict):
        raise SystemExit(f"Missing [strategies.*] tables in {path}")
    strategies: dict[str, ExperimentStrategy] = {}
    for key, raw in raw_strategies.items():
        if not isinstance(raw, dict):
            raise SystemExit(f"[strategies.{key}] must be a table")
        implementation_files = tuple(
            (ROOT_DIR / str(relative_path)).resolve()
            for relative_path in raw.get("implementation_files", [])
        )
        strategies[key] = ExperimentStrategy(
            key=key,
            label=str(raw.get("label", key)),
            design_style=str(raw.get("design_style", "unspecified")),
            status=str(raw.get("status", "candidate")),
            description=str(raw.get("description", "")),
            claslib_reference=str(raw.get("claslib_reference", "")),
            extra_args=tuple(str(value) for value in raw.get("extra_args", [])),
            compact_flush_policy=(
                str(raw["compact_flush_policy"]) if raw.get("compact_flush_policy") is not None else None
            ),
            compact_atmos_merge_policy=(
                str(raw["compact_atmos_merge_policy"])
                if raw.get("compact_atmos_merge_policy") is not None
                else None
            ),
            compact_atmos_subtype_merge_policy=(
                str(raw["compact_atmos_subtype_merge_policy"])
                if raw.get("compact_atmos_subtype_merge_policy") is not None
                else None
            ),
            implementation_files=implementation_files,
        )
    return strategies


def ensure_input_exists(path: Path | None, description: str) -> None:
    if path is None or not path.exists():
        raise SystemExit(f"Missing {description}: {path}")


def ensure_ssr_csv(input_config: ExperimentInput, strategy: ExperimentStrategy) -> Path:
    if input_config.ssr_csv is not None:
        ensure_input_exists(input_config.ssr_csv, "SSR CSV")
        return input_config.ssr_csv
    ensure_input_exists(input_config.qzss_l6, "QZSS L6 source")
    if input_config.qzss_gps_week is None:
        raise SystemExit("qzss_gps_week is required when generating SSR CSV from raw L6")

    if str(APPS_DIR) not in sys.path:
        sys.path.insert(0, str(APPS_DIR))
    from gnss_clas_ppp import expand_compact_ssr_text  # type: ignore
    from gnss_qzss_l6_info import decode_cssr_messages, decode_source, write_compact_corrections  # type: ignore

    input_config.output_dir.mkdir(parents=True, exist_ok=True)
    flush_policy = strategy.compact_flush_policy or "lag-tolerant-union"
    atmos_merge_policy = strategy.compact_atmos_merge_policy or "stec-coeff-carry"
    atmos_subtype_merge_policy = strategy.compact_atmos_subtype_merge_policy or "union"
    compact_path = input_config.output_dir / (
        f"compact_ssr_{flush_policy}__{atmos_merge_policy}__{atmos_subtype_merge_policy}.csv"
    )
    expanded_path = input_config.output_dir / (
        f"expanded_ssr_{flush_policy}__{atmos_merge_policy}__{atmos_subtype_merge_policy}.csv"
    )
    if compact_path.exists() and expanded_path.exists():
        return expanded_path
    _, subframes, _ = decode_source(str(input_config.qzss_l6))
    _, corrections, _ = decode_cssr_messages(
        subframes,
        gps_week=input_config.qzss_gps_week,
        flush_policy=flush_policy,
        atmos_merge_policy=atmos_merge_policy,
        atmos_subtype_merge_policy=atmos_subtype_merge_policy,
    )
    write_compact_corrections(compact_path, corrections)
    expand_compact_ssr_text(compact_path.read_text(encoding="ascii"), expanded_path)
    return expanded_path


def percentile(values: list[float], fraction: float) -> float:
    if not values:
        return float("nan")
    ordered = sorted(values)
    if len(ordered) == 1:
        return ordered[0]
    position = (len(ordered) - 1) * fraction
    lower = math.floor(position)
    upper = math.ceil(position)
    if lower == upper:
        return ordered[lower]
    weight = position - lower
    return ordered[lower] * (1.0 - weight) + ordered[upper] * weight


def summarize_solution(pos_path: Path, reference_ecef: tuple[float, float, float]) -> dict[str, Any]:
    rows: list[dict[str, float | int]] = []
    with pos_path.open("r", encoding="ascii") as handle:
        for line in handle:
            if not line.strip() or line.startswith("%"):
                continue
            parts = line.split()
            if len(parts) < 10:
                continue
            rows.append(
                {
                    "x": float(parts[2]),
                    "y": float(parts[3]),
                    "z": float(parts[4]),
                    "status": int(parts[8]),
                    "satellites": int(parts[9]),
                }
            )
    if not rows:
        raise SystemExit(f"No PPP solution epochs found in {pos_path}")

    def error_m(row: dict[str, float | int]) -> float:
        return math.sqrt(
            (float(row["x"]) - reference_ecef[0]) ** 2
            + (float(row["y"]) - reference_ecef[1]) ** 2
            + (float(row["z"]) - reference_ecef[2]) ** 2
        )

    errors = [error_m(row) for row in rows]
    fixed_errors = [error_m(row) for row in rows if int(row["status"]) == 6]
    return {
        "epochs": len(rows),
        "fixed_epochs": sum(1 for row in rows if int(row["status"]) == 6),
        "float_epochs": sum(1 for row in rows if int(row["status"]) == 5),
        "fallback_epochs": sum(1 for row in rows if int(row["status"]) < 5),
        "mean_3d_error_m": round(sum(errors) / len(errors), 6),
        "median_3d_error_m": round(percentile(errors, 0.5), 6),
        "p95_3d_error_m": round(percentile(errors, 0.95), 6),
        "max_3d_error_m": round(max(errors), 6),
        "fixed_mean_3d_error_m": round(sum(fixed_errors) / len(fixed_errors), 6)
        if fixed_errors
        else None,
        "fixed_p95_3d_error_m": round(percentile(fixed_errors, 0.95), 6)
        if fixed_errors
        else None,
    }


def line_count(path: Path) -> int:
    with path.open("r", encoding="utf-8") as handle:
        return sum(1 for _ in handle)


def classify_band(score: float) -> str:
    if score >= 80.0:
        return "high"
    if score >= 60.0:
        return "medium"
    return "low"


def compute_structure_metrics(strategy: ExperimentStrategy) -> dict[str, Any]:
    loc_by_file = {str(path.relative_to(ROOT_DIR)): line_count(path) for path in strategy.implementation_files}
    module_count = len(loc_by_file)
    total_loc = sum(loc_by_file.values())
    max_file_loc = max(loc_by_file.values()) if loc_by_file else 0
    arg_count = len(strategy.extra_args)
    readability_score = max(
        0.0,
        min(100.0, 95.0 - max_file_loc / 45.0 - module_count * 2.0 - arg_count * 0.5),
    )
    extensibility_score = max(
        0.0,
        min(
            100.0,
            30.0
            + module_count * 15.0
            + (10.0 if "pipeline" in strategy.design_style else 0.0)
            - max_file_loc / 100.0,
        ),
    )
    return {
        "implementation_files": list(loc_by_file.keys()),
        "implementation_loc": total_loc,
        "max_file_loc": max_file_loc,
        "module_count": module_count,
        "command_arg_count": arg_count,
        "readability_score": round(readability_score, 1),
        "readability_rating": classify_band(readability_score),
        "extensibility_score": round(extensibility_score, 1),
        "extensibility_rating": classify_band(extensibility_score),
    }


def evaluate_promotion_readiness(results: list[dict[str, Any]]) -> list[dict[str, Any]]:
    baseline = next((result for result in results if result.get("status") == "baseline"), None)
    if baseline is None:
        for result in results:
            result["promotion_stage"] = "unclassified"
            result["promotion_gate_pass"] = False
            result["promotion_rationale"] = "No baseline strategy defined."
        return results

    baseline_solution_rate = float(baseline.get("ppp_solution_rate_pct", 0.0))
    baseline_mean = float(baseline.get("mean_3d_error_m", float("inf")))
    baseline_p95 = float(baseline.get("p95_3d_error_m", float("inf")))
    baseline_fixed = int(baseline.get("ppp_fixed_solutions", 0))
    baseline_readability = float(baseline.get("readability_score", 0.0))
    baseline_extensibility = float(baseline.get("extensibility_score", 0.0))

    for result in results:
        if result is baseline:
            result["promotion_stage"] = "stable_control"
            result["promotion_gate_pass"] = False
            result["promotion_rationale"] = "Stable control arm used to judge candidate strategies."
            result["comparison_to_baseline"] = {
                "solution_rate_pct_delta": 0.0,
                "mean_3d_error_m_delta": 0.0,
                "p95_3d_error_m_delta": 0.0,
                "fixed_solution_delta": 0,
                "readability_score_delta": 0.0,
                "extensibility_score_delta": 0.0,
            }
            continue

        solution_rate_delta = float(result.get("ppp_solution_rate_pct", 0.0)) - baseline_solution_rate
        mean_improvement = baseline_mean - float(result.get("mean_3d_error_m", float("inf")))
        p95_improvement = baseline_p95 - float(result.get("p95_3d_error_m", float("inf")))
        fixed_delta = int(result.get("ppp_fixed_solutions", 0)) - baseline_fixed
        readability_delta = float(result.get("readability_score", 0.0)) - baseline_readability
        extensibility_delta = float(result.get("extensibility_score", 0.0)) - baseline_extensibility

        gate_checks = {
            "solution_rate_non_regression": solution_rate_delta >= -0.05,
            "mean_error_improved": mean_improvement >= 0.25,
            "p95_error_improved": p95_improvement >= 0.25,
            "readability_non_regression": readability_delta >= 0.0,
            "extensibility_non_regression": extensibility_delta >= 0.0,
        }
        if "solver" in str(result.get("design_style", "")):
            gate_checks["fixed_epochs_improved"] = fixed_delta > 0

        gate_pass = all(gate_checks.values())
        if gate_pass and fixed_delta > 0:
            stage = "promotion_candidate"
            rationale = "Beats the stable control on accuracy, preserves solution rate, and adds fixed epochs."
        elif gate_pass:
            stage = "trial_candidate"
            rationale = "Beats the stable control on the shared accuracy gates and is ready for wider trials."
        else:
            stage = "exploratory"
            failed = [name for name, passed in gate_checks.items() if not passed]
            rationale = "Still exploratory because these gates are not met: " + ", ".join(failed)

        result["comparison_to_baseline"] = {
            "solution_rate_pct_delta": round(solution_rate_delta, 4),
            "mean_3d_error_m_delta": round(mean_improvement, 6),
            "p95_3d_error_m_delta": round(p95_improvement, 6),
            "fixed_solution_delta": fixed_delta,
            "readability_score_delta": round(readability_delta, 1),
            "extensibility_score_delta": round(extensibility_delta, 1),
        }
        result["promotion_gate_checks"] = gate_checks
        result["promotion_gate_pass"] = gate_pass
        result["promotion_stage"] = stage
        result["promotion_rationale"] = rationale
    return results


def summarize_suite_results(case_payloads: list[dict[str, Any]]) -> list[dict[str, Any]]:
    by_strategy: dict[str, list[dict[str, Any]]] = {}
    for payload in case_payloads:
        for result in payload["results"]:
            by_strategy.setdefault(str(result["strategy"]), []).append(result)

    summaries: list[dict[str, Any]] = []
    for strategy_key, entries in by_strategy.items():
        first = entries[0]
        cases_run = len(entries)
        cases_passed = sum(1 for entry in entries if bool(entry.get("promotion_gate_pass", False)))
        fixed_gain_cases = sum(
            1
            for entry in entries
            if int(entry.get("comparison_to_baseline", {}).get("fixed_solution_delta", 0)) > 0
        )
        gate_pass_all = cases_passed == cases_run
        gate_pass_majority = cases_passed >= math.ceil(cases_run / 2.0)
        comparison_solution = [
            float(entry.get("comparison_to_baseline", {}).get("solution_rate_pct_delta", 0.0))
            for entry in entries
        ]
        comparison_mean = [
            float(entry.get("comparison_to_baseline", {}).get("mean_3d_error_m_delta", 0.0))
            for entry in entries
        ]
        comparison_p95 = [
            float(entry.get("comparison_to_baseline", {}).get("p95_3d_error_m_delta", 0.0))
            for entry in entries
        ]
        hybrid_fallback_total = sum(
            int(entry.get("clas_hybrid_fallback_epochs", 0))
            for entry in entries
        )

        if first.get("status") == "baseline":
            promotion_stage = "stable_control"
            rationale = "Stable control arm used to judge candidate strategies across the suite."
        elif gate_pass_all and (
            "solver" not in str(first.get("design_style", "")) or fixed_gain_cases == cases_run
        ):
            promotion_stage = "promotion_candidate"
            rationale = "Wins the shared gates across every suite case and preserves the solver-specific gains."
        elif gate_pass_all:
            promotion_stage = "trial_candidate"
            rationale = "Passes the shared gates across every suite case and is ready for broader trials."
        elif gate_pass_majority:
            promotion_stage = "extended_trial"
            rationale = "Passes in most suite cases but is not yet stable enough for promotion."
        else:
            promotion_stage = "exploratory"
            rationale = "Still exploratory because it does not repeatedly beat the control arm across the suite."

        summaries.append(
            {
                "strategy": strategy_key,
                "label": first["label"],
                "design_style": first["design_style"],
                "status": first["status"],
                "cases_run": cases_run,
                "cases_passed": cases_passed,
                "fixed_gain_cases": fixed_gain_cases,
                "mean_solution_rate_pct": round(
                    sum(float(entry.get("ppp_solution_rate_pct", 0.0)) for entry in entries) / cases_run,
                    4,
                ),
                "mean_3d_error_m": round(
                    sum(float(entry.get("mean_3d_error_m", 0.0)) for entry in entries) / cases_run,
                    6,
                ),
                "p95_3d_error_m": round(
                    sum(float(entry.get("p95_3d_error_m", 0.0)) for entry in entries) / cases_run,
                    6,
                ),
                "mean_wall_time_s": round(
                    sum(float(entry.get("wall_time_s", 0.0)) for entry in entries) / cases_run,
                    3,
                ),
                "total_fixed_solutions": sum(int(entry.get("ppp_fixed_solutions", 0)) for entry in entries),
                "total_clas_hybrid_fallback_epochs": hybrid_fallback_total,
                "module_count": int(first.get("module_count", 0)),
                "implementation_loc": int(first.get("implementation_loc", 0)),
                "max_file_loc": int(first.get("max_file_loc", 0)),
                "command_arg_count": int(first.get("command_arg_count", 0)),
                "claslib_reference": str(first.get("claslib_reference", "")),
                "readability_score": float(first.get("readability_score", 0.0)),
                "extensibility_score": float(first.get("extensibility_score", 0.0)),
                "readability_rating": str(first.get("readability_rating", "low")),
                "extensibility_rating": str(first.get("extensibility_rating", "low")),
                "suite_solution_rate_pct_delta": round(sum(comparison_solution) / cases_run, 4),
                "suite_mean_3d_error_m_delta": round(sum(comparison_mean) / cases_run, 6),
                "suite_p95_3d_error_m_delta": round(sum(comparison_p95) / cases_run, 6),
                "promotion_stage": promotion_stage,
                "promotion_gate_pass": gate_pass_all,
                "promotion_rationale": rationale,
            }
        )
    order = {entry["strategy"]: index for index, payload in enumerate(case_payloads) for entry in payload["results"]}
    return sorted(summaries, key=lambda entry: order.get(str(entry["strategy"]), 999))


def build_command(
    input_config: ExperimentInput,
    strategy: ExperimentStrategy,
    ssr_csv: Path,
    pos_path: Path,
    summary_path: Path,
) -> list[str]:
    command = [
        sys.executable,
        str(ROOT_DIR / "apps" / "gnss.py"),
        "ppp",
        "--obs",
        str(input_config.obs),
        "--nav",
        str(input_config.nav),
        "--ssr",
        str(ssr_csv),
        "--out",
        str(pos_path),
        "--summary-json",
        str(summary_path),
        "--max-epochs",
        str(input_config.max_epochs),
    ]
    command.append("--kinematic" if input_config.mode == "kinematic" else "--static")
    command.append("--estimate-troposphere" if input_config.estimate_troposphere else "--no-estimate-troposphere")
    command.extend(strategy.extra_args)
    return command


def run_command(command: list[str]) -> tuple[float, subprocess.CompletedProcess[str]]:
    started = time.perf_counter()
    completed = subprocess.run(
        command,
        cwd=ROOT_DIR,
        check=True,
        capture_output=True,
        text=True,
    )
    return time.perf_counter() - started, completed


def run_strategy(
    input_config: ExperimentInput,
    strategy: ExperimentStrategy,
    ssr_csv: Path,
    *,
    dry_run: bool,
) -> dict[str, Any]:
    output_dir = input_config.output_dir / strategy.key
    output_dir.mkdir(parents=True, exist_ok=True)
    pos_path = output_dir / f"{strategy.key}.pos"
    summary_path = output_dir / f"{strategy.key}_summary.json"
    stdout_path = output_dir / f"{strategy.key}.stdout.log"
    stderr_path = output_dir / f"{strategy.key}.stderr.log"
    command = build_command(input_config, strategy, ssr_csv, pos_path, summary_path)
    result: dict[str, Any] = {
        "strategy": strategy.key,
        "label": strategy.label,
        "design_style": strategy.design_style,
        "status": strategy.status,
        "description": strategy.description,
        "claslib_reference": strategy.claslib_reference,
        "command": command,
        "solution_pos": str(pos_path),
        "summary_json": str(summary_path),
        **compute_structure_metrics(strategy),
    }
    if dry_run:
        result["dry_run"] = True
        return result

    wall_time_s, completed = run_command(command)
    stdout_path.write_text(completed.stdout, encoding="utf-8")
    stderr_path.write_text(completed.stderr, encoding="utf-8")
    summary = json.loads(summary_path.read_text(encoding="utf-8"))
    result.update(
        {
            "wall_time_s": round(wall_time_s, 3),
            "ppp_solution_rate_pct": float(summary.get("ppp_solution_rate_pct", 0.0)),
            "ppp_fixed_solutions": int(summary.get("ppp_fixed_solutions", 0)),
            "ppp_float_solutions": int(summary.get("ppp_float_solutions", 0)),
            "fallback_solutions": int(summary.get("fallback_solutions", 0)),
            "clas_hybrid_fallback_epochs": int(summary.get("clas_hybrid_fallback_epochs", 0)),
            "converged": bool(summary.get("converged", False)),
            "convergence_time_s": float(summary.get("convergence_time_s", 0.0)),
            "mean_satellites": float(summary.get("mean_satellites", 0.0)),
            "atmospheric_trop_corrections": int(summary.get("atmospheric_trop_corrections", 0)),
            "atmospheric_iono_corrections": int(summary.get("atmospheric_iono_corrections", 0)),
            "atmospheric_trop_meters": float(summary.get("atmospheric_trop_meters", 0.0)),
            "atmospheric_iono_meters": float(summary.get("atmospheric_iono_meters", 0.0)),
            "average_processing_time_ms": float(summary.get("average_processing_time_ms", 0.0)),
        }
    )
    result.update(summarize_solution(pos_path, input_config.reference_ecef))
    return result


def render_markdown(
    suite: ExperimentSuite,
    strategy_path: Path,
    case_payloads: list[dict[str, Any]],
    suite_summary: list[dict[str, Any]],
) -> str:
    generated = datetime.now(timezone.utc).strftime("%Y-%m-%d %H:%M UTC")
    first_case = suite.cases[0]
    lines = [
        "# PPP-AR Experiments",
        "",
        f"Generated: {generated}",
        "",
        "This page is the current externalized state of the PPP-AR experiment lane.",
        "All rows below use the same strategy catalog, the same metrics, and comparable CLAS inputs.",
        "",
        "## Shared Interface",
        "",
        f"- Suite: `{suite.label}`",
        f"- Cases: `{len(suite.cases)}`",
        f"- Primary mode: `{first_case.mode}`",
        f"- Shared strategy catalog: `{strategy_path.relative_to(ROOT_DIR)}`",
        f"- Shared output schema: `wall_time_s`, `ppp_solution_rate_pct`, `ppp_fixed_solutions`, `fallback_solutions`, `clas_hybrid_fallback_epochs`, `mean_3d_error_m`, `median_3d_error_m`, `p95_3d_error_m`, `max_3d_error_m`, `readability_score`, `extensibility_score`",
        "",
        "## Suite Summary",
        "",
        "| Strategy | Cases passed | Mean solution rate | Mean 3D (m) | P95 3D (m) | Mean delta vs baseline (m) | Fixed total | Hybrid fallback total | Promotion |",
        "|---|---:|---:|---:|---:|---:|---:|---:|---|",
    ]
    for result in suite_summary:
        lines.append(
            "| "
            + " | ".join(
                [
                    str(result["label"]),
                    f"{int(result['cases_passed'])}/{int(result['cases_run'])}",
                    f"{float(result['mean_solution_rate_pct']):.2f}",
                    f"{float(result['mean_3d_error_m']):.3f}",
                    f"{float(result['p95_3d_error_m']):.3f}",
                    f"{float(result['suite_mean_3d_error_m_delta']):.3f}",
                    str(int(result["total_fixed_solutions"])),
                    str(int(result.get("total_clas_hybrid_fallback_epochs", 0))),
                    str(result["promotion_stage"]),
                ]
            )
            + " |"
        )
    lines.extend(
        [
            "",
            "## Case Results",
            "",
        ]
    )
    for case_payload in case_payloads:
        case = next(case for case in suite.cases if case.case_key == case_payload["case_key"])
        results = case_payload["results"]
        lines.extend(
            [
                f"### {case.dataset_label}",
                "",
                f"- Case key: `{case.case_key}`",
                f"- Observation file: `{case.obs.name}`",
                f"- Navigation file: `{case.nav.name}`",
                f"- QZSS L6 source: `{case.qzss_l6.name if case.qzss_l6 is not None else 'existing SSR CSV'}`",
                f"- Max epochs: `{case.max_epochs}`",
                f"- Mode: `{case.mode}`",
                "",
                "#### Strategy Comparison",
                "",
                "| Strategy | Style | Solution rate | Fixed | Fallback | Hybrid fallback | Mean 3D (m) | P95 3D (m) | Wall (s) | Readability | Extensibility | Promotion |",
                "|---|---|---:|---:|---:|---:|---:|---:|---:|---|---|---|",
            ]
        )
        for result in results:
            lines.append(
                "| "
                + " | ".join(
                    [
                        result["label"],
                        result["design_style"],
                        f"{float(result.get('ppp_solution_rate_pct', 0.0)):.2f}",
                        str(int(result.get("ppp_fixed_solutions", 0))),
                        str(int(result.get("fallback_solutions", 0))),
                        str(int(result.get("clas_hybrid_fallback_epochs", 0))),
                        f"{float(result.get('mean_3d_error_m', float('nan'))):.3f}",
                        f"{float(result.get('p95_3d_error_m', float('nan'))):.3f}",
                        f"{float(result.get('wall_time_s', 0.0)):.2f}",
                        str(result["readability_rating"]),
                        str(result["extensibility_rating"]),
                        str(result.get("promotion_stage", "unclassified")),
                    ]
                )
                + " |"
            )
        lines.extend(
            [
                "",
                "#### Promotion Gate",
                "",
                "| Strategy | Mean delta vs baseline (m) | P95 delta vs baseline (m) | Fixed delta | Solution-rate delta | Gate | Rationale |",
                "|---|---:|---:|---:|---:|---|---|",
            ]
        )
        for result in results:
            baseline_compare = result.get("comparison_to_baseline", {})
            lines.append(
                "| "
                + " | ".join(
                    [
                        result["label"],
                        f"{float(baseline_compare.get('mean_3d_error_m_delta', 0.0)):.3f}",
                        f"{float(baseline_compare.get('p95_3d_error_m_delta', 0.0)):.3f}",
                        str(int(baseline_compare.get("fixed_solution_delta", 0))),
                        f"{float(baseline_compare.get('solution_rate_pct_delta', 0.0)):.2f}",
                        "pass" if bool(result.get("promotion_gate_pass", False)) else "hold",
                        str(result.get("promotion_rationale", "")),
                    ]
                )
                + " |"
            )
        lines.extend(
            [
                "",
            ]
        )
    lines.extend(
        [
            "",
            "## Structural Heuristics",
            "",
            "| Strategy | Files | LOC | Max file LOC | Args | CLASLIB reference |",
            "|---|---:|---:|---:|---:|---|",
        ]
    )
    for result in suite_summary:
        lines.append(
            "| "
            + " | ".join(
                [
                    result["label"],
                    str(int(result["module_count"])),
                    str(int(result["implementation_loc"])),
                    str(int(result["max_file_loc"])),
                    str(int(result["command_arg_count"])),
                    str(result["claslib_reference"]),
                ]
            )
            + " |"
        )
    lines.extend(
        [
            "",
            "## Notes",
            "",
            "- `IFLC Float Baseline` is the control arm and remains the fallback path.",
            "- `OSR * Pipeline` arms keep the CLASLIB-inspired observation-space boundary and differ in how epoch fallback, atmosphere-token selection, and ambiguity fixing are staged.",
            "- `Strict` arms stay inside the CLAS epoch path. `Hybrid` arms restore the pre-epoch state and re-run the stable standard PPP path when the CLAS OSR stage cannot complete.",
            "- `Guarded` arms keep the nearest grid but reject stale atmosphere tokens completely once they age out.",
            "- `Balanced` arms are stale-aware: they keep the nearest grid while corrections are fresh, then prefer freshness once the current network ages out.",
            "- `Freshness` arms prioritize correction recency over grid proximity so selector failures can be isolated from the cssr2osr-style measurement path.",
            "- `Clock-Bound-*` arms keep the same accepted CLAS update path but tighten expanded-SSR timing acceptance relative to the selected clock epoch.",
            "- `Orbit-* Source` arms change the compact-to-expanded row emission policy before the solver sees any CLAS SSR values.",
            "- `*Atmos-Merge*` arms change how compact STEC polynomial terms are carried or reset before expanded SSR rows are materialized.",
            "- `*Subtype-Merge*` arms change how subtype 8/9/12 atmosphere families replace or coexist before expanded SSR rows are materialized.",
            "- `*Value*` arms change how expanded atmosphere values are constructed from polynomial terms and residual lists before residual formation.",
            "- `*Residual-Sampling*` arms keep the same composed atmosphere model but change whether expanded residual lists use indexed samples, mean fallback, or mean-only sampling.",
            "- `*Subtype12-Value*` arms keep residual sampling fixed and only change how subtype-12 rows retain constant, linear, or higher-order surface terms before residual formation.",
            "- Promotion is decided against the control arm and only becomes stable after repeated suite wins.",
            "- Readability and extensibility are heuristic scores generated from experiment-owned files, file size, and command surface. They are not code-review substitutes.",
        ]
    )
    return "\n".join(lines) + "\n"


def select_strategies(
    input_config: ExperimentInput,
    strategies: dict[str, ExperimentStrategy],
    selected: list[str],
) -> list[ExperimentStrategy]:
    keys = selected or list(input_config.strategies)
    missing = [key for key in keys if key not in strategies]
    if missing:
        raise SystemExit(f"Unknown strategy keys: {', '.join(missing)}")
    return [strategies[key] for key in keys]


def main() -> int:
    args = parse_args()
    suite = load_suite_config(args.config_toml)
    strategies = load_strategies(args.strategies_toml)
    case_payloads: list[dict[str, Any]] = []
    for input_config in suite.cases:
        ensure_input_exists(input_config.obs, "observation file")
        ensure_input_exists(input_config.nav, "navigation file")
        selected = select_strategies(input_config, strategies, args.strategy)
        ssr_cache: dict[str, Path] = {}
        results = []
        for strategy in selected:
            flush_policy = strategy.compact_flush_policy or "lag-tolerant-union"
            ssr_csv = ssr_cache.get(flush_policy)
            if ssr_csv is None:
                ssr_csv = ensure_ssr_csv(input_config, strategy)
                ssr_cache[flush_policy] = ssr_csv
            results.append(run_strategy(input_config, strategy, ssr_csv, dry_run=args.dry_run))
        case_payloads.append(
            {
                "case_key": input_config.case_key,
                "dataset_label": input_config.dataset_label,
                "obs": input_config.obs.name,
                "nav": input_config.nav.name,
                "qzss_l6": input_config.qzss_l6.name if input_config.qzss_l6 is not None else None,
                "ssr_csv": next(iter(ssr_cache.values())).name if ssr_cache else None,
                "max_epochs": input_config.max_epochs,
                "mode": input_config.mode,
                "estimate_troposphere": input_config.estimate_troposphere,
                "results": evaluate_promotion_readiness(results),
            }
        )
    suite_summary = summarize_suite_results(case_payloads)
    payload = {
        "generated_at": datetime.now(timezone.utc).isoformat(),
        "suite_label": suite.label,
        "case_count": len(suite.cases),
        "cases": case_payloads,
        "suite_summary": suite_summary,
    }
    args.output_json.parent.mkdir(parents=True, exist_ok=True)
    args.output_json.write_text(json.dumps(payload, indent=2, sort_keys=True) + "\n", encoding="utf-8")
    if args.markdown_out is not None:
        args.markdown_out.parent.mkdir(parents=True, exist_ok=True)
        args.markdown_out.write_text(
            render_markdown(suite, args.strategies_toml, case_payloads, suite_summary),
            encoding="utf-8",
        )
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
