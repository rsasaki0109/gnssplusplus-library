#!/usr/bin/env python3
"""Run optional MADOCA residual-component diff and persist a CI summary."""

from __future__ import annotations

import argparse
from dataclasses import asdict, dataclass
import json
import os
from pathlib import Path
import subprocess
import sys
import time
from typing import Mapping


SUMMARY_SCHEMA = "ci_optional_madoca_residual_component_diff.v1"


@dataclass(frozen=True)
class DiffStep:
    name: str
    slug: str
    command: list[str] | None
    outputs: list[str]
    summary_json: str
    skip_reason: str | None = None


@dataclass(frozen=True)
class DiffContext:
    repo_root: Path
    output_dir: Path
    python_executable: str
    base_csv: Path | None
    candidate_csv: Path | None
    components: list[str]
    row_type: str | None
    iteration: int | None
    threshold: float | None
    fail_on_diff: bool


def repo_root_from_script() -> Path:
    return Path(__file__).resolve().parents[2]


def resolve_optional_path(repo_root: Path, value: str) -> Path | None:
    text = value.strip()
    if not text:
        return None
    path = Path(text)
    if path.is_absolute():
        return path
    return repo_root / path


def parse_components(value: str) -> list[str]:
    components = [item.strip() for item in value.replace(";", ",").split(",")]
    return [item for item in components if item]


def parse_optional_int(value: str) -> int | None:
    text = value.strip()
    if not text:
        return None
    return int(text)


def parse_optional_float(value: str) -> float | None:
    text = value.strip()
    if not text:
        return None
    return float(text)


def truthy(value: str) -> bool:
    return value.strip().lower() in {"1", "true", "yes", "y", "on"}


def build_context(
    repo_root: Path,
    output_dir: Path,
    environ: Mapping[str, str],
    *,
    python_executable: str = sys.executable,
) -> DiffContext:
    return DiffContext(
        repo_root=repo_root,
        output_dir=output_dir,
        python_executable=python_executable,
        base_csv=resolve_optional_path(
            repo_root,
            environ.get("GNSSPP_MADOCA_RESIDUAL_BASE_CSV", ""),
        ),
        candidate_csv=resolve_optional_path(
            repo_root,
            environ.get("GNSSPP_MADOCA_RESIDUAL_CANDIDATE_CSV", "")
            or environ.get("GNSSPP_MADOCA_RESIDUAL_NATIVE_CSV", ""),
        ),
        components=parse_components(
            environ.get("GNSSPP_MADOCA_RESIDUAL_COMPONENTS", "residual_m")
        ),
        row_type=environ.get("GNSSPP_MADOCA_RESIDUAL_ROW_TYPE", "").strip() or None,
        iteration=parse_optional_int(environ.get("GNSSPP_MADOCA_RESIDUAL_ITERATION", "")),
        threshold=parse_optional_float(environ.get("GNSSPP_MADOCA_RESIDUAL_THRESHOLD", "")),
        fail_on_diff=truthy(environ.get("GNSSPP_MADOCA_RESIDUAL_FAIL_ON_DIFF", "")),
    )


def make_step(context: DiffContext) -> DiffStep:
    name = "MADOCA residual-component diff"
    slug = "madoca_residual_component_diff"
    report_json = context.output_dir / f"{slug}.json"
    details_csv = context.output_dir / f"{slug}.csv"
    summary_json = context.output_dir / f"{slug}_summary.json"
    outputs = [report_json, details_csv, summary_json]

    missing = [
        (label, path)
        for label, path in (
            ("base residual CSV", context.base_csv),
            ("candidate residual CSV", context.candidate_csv),
        )
        if path is None or not path.is_file()
    ]
    if missing:
        missing_text = ", ".join(f"{label}: {path}" for label, path in missing)
        return DiffStep(
            name=name,
            slug=slug,
            command=None,
            outputs=[str(path) for path in outputs],
            summary_json=str(summary_json),
            skip_reason=f"MADOCA residual-component input is unavailable ({missing_text}).",
        )

    assert context.base_csv is not None
    assert context.candidate_csv is not None
    command = [
        context.python_executable,
        str(context.repo_root / "scripts" / "analysis" / "madoca_residual_component_diff.py"),
        str(context.base_csv),
        str(context.candidate_csv),
        "--base-label",
        "madocalib",
        "--candidate-label",
        "native",
        "--json-out",
        str(report_json),
        "--details-csv",
        str(details_csv),
    ]
    for component in context.components or ["residual_m"]:
        command.extend(["--component", component])
    if context.row_type is not None:
        command.extend(["--row-type", context.row_type])
    if context.iteration is not None:
        command.extend(["--iteration", str(context.iteration)])
    if context.threshold is not None:
        command.extend(["--component-threshold", str(context.threshold)])
    if context.fail_on_diff:
        command.append("--fail-on-diff")

    return DiffStep(
        name=name,
        slug=slug,
        command=command,
        outputs=[str(path) for path in outputs],
        summary_json=str(summary_json),
    )


def build_step_plan(
    repo_root: Path,
    output_dir: Path,
    environ: Mapping[str, str],
    *,
    python_executable: str = sys.executable,
) -> list[DiffStep]:
    context = build_context(repo_root, output_dir, environ, python_executable=python_executable)
    return [make_step(context)]


def load_diff_metrics(report_json: Path) -> dict[str, object]:
    if not report_json.is_file():
        return {}
    try:
        payload = json.loads(report_json.read_text(encoding="utf-8"))
    except json.JSONDecodeError:
        return {}
    if not isinstance(payload, dict):
        return {}
    metrics: dict[str, object] = {}
    for key in [
        "schema",
        "base_rows",
        "candidate_rows",
        "common_rows",
        "base_only_rows",
        "candidate_only_rows",
        "components_compared",
        "threshold_exceedances",
        "row_set_complete",
    ]:
        if key in payload:
            metrics[key] = payload[key]
    per_component = payload.get("per_component")
    if isinstance(per_component, list):
        max_abs_delta = 0.0
        for item in per_component:
            if not isinstance(item, dict):
                continue
            value = item.get("max_abs_delta")
            if isinstance(value, (int, float)):
                max_abs_delta = max(max_abs_delta, float(value))
        metrics["max_abs_delta"] = max_abs_delta
    return metrics


def run_step(step: DiffStep, repo_root: Path, log_dir: Path) -> dict[str, object]:
    log_path = log_dir / f"{step.slug}.log"
    summary_json = Path(step.summary_json)
    report_json = summary_json.with_name(f"{step.slug}.json")
    record: dict[str, object] = {
        "name": step.name,
        "slug": step.slug,
        "outputs": step.outputs,
        "summary_json": step.summary_json,
        "log_path": str(log_path),
    }
    if step.skip_reason is not None:
        record["status"] = "skipped"
        record["skip_reason"] = step.skip_reason
        log_path.write_text(step.skip_reason + "\n", encoding="utf-8")
        print(f"Skipping {step.name}: {step.skip_reason}")
        return record

    assert step.command is not None
    command_display = " ".join(step.command)
    print(f"+ {command_display}")
    started = time.monotonic()
    completed = subprocess.run(
        step.command,
        cwd=repo_root,
        text=True,
        capture_output=True,
        check=False,
    )
    elapsed = time.monotonic() - started
    combined_log = completed.stdout
    if completed.stderr:
        if combined_log and not combined_log.endswith("\n"):
            combined_log += "\n"
        combined_log += completed.stderr
    log_path.write_text(f"$ {command_display}\n\n{combined_log}", encoding="utf-8")
    record["command"] = step.command
    record["elapsed_s"] = elapsed
    record["returncode"] = completed.returncode
    metrics = load_diff_metrics(report_json)
    if metrics:
        record["metrics"] = metrics
    if completed.returncode == 0:
        record["status"] = "passed"
        print(f"Passed {step.name} in {elapsed:.2f}s")
    else:
        record["status"] = "failed"
        lines = combined_log.splitlines()
        tail = "\n".join(lines[-20:])
        if tail:
            print(tail)
        print(f"Failed {step.name} in {elapsed:.2f}s; see {log_path}")
    return record


def format_metric(value: object) -> str:
    if isinstance(value, float):
        return f"{value:.6g}"
    return str(value)


def render_result_detail(result: dict[str, object]) -> str:
    status = str(result["status"])
    if status == "skipped":
        return str(result.get("skip_reason", ""))
    if status == "failed":
        log_path = result.get("log_path")
        return f"see `{log_path}`" if log_path else "failed"

    detail_parts: list[str] = []
    elapsed = result.get("elapsed_s")
    if isinstance(elapsed, (float, int)):
        detail_parts.append(f"{elapsed:.2f}s")
    metrics = result.get("metrics")
    if isinstance(metrics, dict):
        common_rows = metrics.get("common_rows")
        if common_rows is not None:
            detail_parts.append(f"common rows `{common_rows}`")
        base_only = metrics.get("base_only_rows")
        candidate_only = metrics.get("candidate_only_rows")
        if base_only is not None or candidate_only is not None:
            detail_parts.append(f"unmatched `{base_only}/{candidate_only}`")
        compared = metrics.get("components_compared")
        if compared is not None:
            detail_parts.append(f"components `{compared}`")
        max_abs_delta = metrics.get("max_abs_delta")
        if max_abs_delta is not None:
            detail_parts.append(f"max |delta| {format_metric(max_abs_delta)}")
        threshold_exceedances = metrics.get("threshold_exceedances")
        if threshold_exceedances is not None:
            detail_parts.append(f"threshold exceedances `{threshold_exceedances}`")
    return ", ".join(detail_parts)


def render_markdown_summary(results: list[dict[str, object]]) -> str:
    passed = sum(1 for result in results if result["status"] == "passed")
    failed = sum(1 for result in results if result["status"] == "failed")
    skipped = sum(1 for result in results if result["status"] == "skipped")
    lines = [
        "## Optional MADOCA Residual-Component Diff",
        "",
        f"- `passed`: `{passed}`",
        f"- `failed`: `{failed}`",
        f"- `skipped`: `{skipped}`",
        "",
        "| Step | Status | Detail |",
        "| --- | --- | --- |",
    ]
    for result in results:
        lines.append(f"| {result['name']} | `{result['status']}` | {render_result_detail(result)} |")
    lines.append("")
    return "\n".join(lines)


def write_summary(summary_path: Path, steps: list[DiffStep], results: list[dict[str, object]]) -> None:
    summary_path.parent.mkdir(parents=True, exist_ok=True)
    payload = {
        "summary_schema": SUMMARY_SCHEMA,
        "steps": [asdict(step) for step in steps],
        "results": results,
        "counts": {
            "passed": sum(1 for result in results if result["status"] == "passed"),
            "failed": sum(1 for result in results if result["status"] == "failed"),
            "skipped": sum(1 for result in results if result["status"] == "skipped"),
        },
    }
    summary_path.write_text(json.dumps(payload, indent=2) + "\n", encoding="utf-8")


def append_github_step_summary(path: Path, results: list[dict[str, object]]) -> None:
    with path.open("a", encoding="utf-8") as handle:
        handle.write(render_markdown_summary(results))
        handle.write("\n")


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser()
    parser.add_argument("--repo-root", type=Path, default=repo_root_from_script())
    parser.add_argument("--output-dir", type=Path, default=None)
    parser.add_argument("--summary-json", type=Path, default=None)
    parser.add_argument("--github-step-summary", type=Path, default=None)
    return parser.parse_args()


def main() -> int:
    args = parse_args()
    repo_root = args.repo_root.resolve()
    output_dir = (args.output_dir if args.output_dir is not None else repo_root / "output").resolve()
    summary_json = (
        args.summary_json
        if args.summary_json is not None
        else output_dir / "ci_optional_madoca_residual_component_summary.json"
    ).resolve()
    log_dir = output_dir / "ci_optional_madoca_residual_component"
    output_dir.mkdir(parents=True, exist_ok=True)
    log_dir.mkdir(parents=True, exist_ok=True)

    steps = build_step_plan(repo_root, output_dir, os.environ)
    results = [run_step(step, repo_root, log_dir) for step in steps]
    write_summary(summary_json, steps, results)
    summary_path = args.github_step_summary
    if summary_path is None:
        summary_env = os.environ.get("GITHUB_STEP_SUMMARY", "").strip()
        summary_path = Path(summary_env) if summary_env else None
    if summary_path is not None:
        append_github_step_summary(summary_path, results)
    return 1 if any(result["status"] == "failed" for result in results) else 0


if __name__ == "__main__":
    raise SystemExit(main())
