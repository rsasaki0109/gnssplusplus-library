#!/usr/bin/env python3
"""Run optional PPP products sign-off and persist a diagnostic summary."""

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


@dataclass(frozen=True)
class SignoffStep:
    name: str
    slug: str
    command: list[str] | None
    outputs: list[str]
    summary_json: str
    skip_reason: str | None = None


@dataclass(frozen=True)
class SignoffContext:
    repo_root: Path
    output_dir: Path
    gnss_command: list[str]
    malib_bin: Path | None
    malib_config: Path | None
    obs: Path
    base: Path
    nav: Path


def repo_root_from_script() -> Path:
    return Path(__file__).resolve().parents[2]


def resolve_optional_path(repo_root: Path, value: str, default: Path | None = None) -> Path | None:
    text = value.strip()
    if not text:
        return default
    path = Path(text)
    if path.is_absolute():
        return path
    return repo_root / path


def is_executable_file(path: Path | None) -> bool:
    return path is not None and path.is_file() and os.access(path, os.X_OK)


def build_context(
    repo_root: Path,
    output_dir: Path,
    environ: Mapping[str, str],
    *,
    python_executable: str = sys.executable,
) -> SignoffContext:
    return SignoffContext(
        repo_root=repo_root,
        output_dir=output_dir,
        gnss_command=[python_executable, str(repo_root / "apps" / "gnss.py")],
        malib_bin=resolve_optional_path(repo_root, environ.get("GNSSPP_MALIB_BIN", "")),
        malib_config=resolve_optional_path(repo_root, environ.get("GNSSPP_PPP_PRODUCTS_MALIB_CONFIG", "")),
        obs=resolve_optional_path(
            repo_root,
            environ.get("GNSSPP_PPP_PRODUCTS_OBS", ""),
            repo_root / "data" / "rover_kinematic.obs",
        ),
        base=resolve_optional_path(
            repo_root,
            environ.get("GNSSPP_PPP_PRODUCTS_BASE", ""),
            repo_root / "data" / "base_kinematic.obs",
        ),
        nav=resolve_optional_path(
            repo_root,
            environ.get("GNSSPP_PPP_PRODUCTS_NAV", ""),
            repo_root / "data" / "navigation_kinematic.nav",
        ),
    )


def make_step(context: SignoffContext) -> SignoffStep:
    name = "PPP products sign-off with MALIB comparison"
    slug = "ppp_kinematic_products"
    summary_json = context.output_dir / f"{slug}_summary.json"
    solution_pos = context.output_dir / f"{slug}_solution.pos"
    reference_pos = context.output_dir / f"{slug}_reference.pos"
    comparison_csv = context.output_dir / f"{slug}_comparison.csv"
    comparison_png = context.output_dir / f"{slug}_comparison.png"
    malib_pos = context.output_dir / "malib_ppp_kinematic_solution.pos"
    outputs = [
        summary_json,
        solution_pos,
        reference_pos,
        comparison_csv,
        comparison_png,
        malib_pos,
    ]

    if not is_executable_file(context.malib_bin):
        return SignoffStep(
            name=name,
            slug=slug,
            command=None,
            outputs=[str(path) for path in outputs],
            summary_json=str(summary_json),
            skip_reason="MALIB binary is unavailable.",
        )

    missing_inputs = [
        (label, path)
        for label, path in (
            ("rover observation", context.obs),
            ("base observation", context.base),
            ("navigation", context.nav),
        )
        if not path.is_file()
    ]
    if missing_inputs:
        missing_text = ", ".join(f"{label}: {path}" for label, path in missing_inputs)
        return SignoffStep(
            name=name,
            slug=slug,
            command=None,
            outputs=[str(path) for path in outputs],
            summary_json=str(summary_json),
            skip_reason=f"PPP products input is unavailable ({missing_text}).",
        )

    command = [
        *context.gnss_command,
        "ppp-products-signoff",
        "--profile",
        "kinematic",
        "--obs",
        str(context.obs),
        "--base",
        str(context.base),
        "--nav",
        str(context.nav),
        "--out",
        str(solution_pos),
        "--reference-pos",
        str(reference_pos),
        "--summary-json",
        str(summary_json),
        "--comparison-csv",
        str(comparison_csv),
        "--comparison-png",
        str(comparison_png),
        "--max-epochs",
        "60",
        "--malib-bin",
        str(context.malib_bin),
        "--malib-pos",
        str(malib_pos),
        "--require-common-epoch-pairs-min",
        "20",
        "--require-ppp-solution-rate-min",
        "100",
    ]
    if context.malib_config is not None:
        command.extend(["--malib-config", str(context.malib_config)])
    return SignoffStep(
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
) -> list[SignoffStep]:
    context = build_context(repo_root, output_dir, environ, python_executable=python_executable)
    return [make_step(context)]


def load_summary_metrics(summary_json: Path) -> dict[str, object]:
    if not summary_json.is_file():
        return {}
    try:
        payload = json.loads(summary_json.read_text(encoding="utf-8"))
    except json.JSONDecodeError:
        return {}
    if not isinstance(payload, dict):
        return {}
    keys = [
        "products_signoff_profile",
        "dataset",
        "ppp_solution_rate_pct",
        "common_epoch_pairs",
        "reference_common_epoch_pairs",
        "mean_position_error_m",
        "p95_position_error_m",
        "max_position_error_m",
        "comparison_target",
        "comparison_status",
        "libgnss_minus_malib_mean_error_m",
        "libgnss_minus_malib_p95_error_m",
        "libgnss_minus_malib_max_error_m",
    ]
    return {key: payload[key] for key in keys if key in payload}


def run_step(step: SignoffStep, repo_root: Path, log_dir: Path) -> dict[str, object]:
    log_path = log_dir / f"{step.slug}.log"
    summary_json = Path(step.summary_json)
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
    metrics = load_summary_metrics(summary_json)
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


def format_metric_value(value: object, *, suffix: str = "") -> str:
    if isinstance(value, float):
        text = f"{value:.3f}".rstrip("0").rstrip(".")
    else:
        text = str(value)
    return text + suffix


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
        profile = metrics.get("products_signoff_profile")
        if profile is not None:
            detail_parts.append(f"profile `{profile}`")
        solution_rate = metrics.get("ppp_solution_rate_pct")
        if solution_rate is not None:
            detail_parts.append(f"PPP solution {format_metric_value(solution_rate, suffix='%')}")
        common_pairs = metrics.get("common_epoch_pairs")
        if common_pairs is not None:
            detail_parts.append(f"common pairs `{common_pairs}`")
        p95 = metrics.get("p95_position_error_m")
        if p95 is not None:
            detail_parts.append(f"p95 {format_metric_value(p95, suffix=' m')}")
        comparison_status = metrics.get("comparison_status")
        if comparison_status is not None:
            detail_parts.append(f"comparison `{comparison_status}`")
    return ", ".join(detail_parts)


def render_markdown_summary(results: list[dict[str, object]]) -> str:
    passed = sum(1 for result in results if result["status"] == "passed")
    failed = sum(1 for result in results if result["status"] == "failed")
    skipped = sum(1 for result in results if result["status"] == "skipped")
    lines = [
        "## Optional PPP Products Sign-off",
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


def write_summary(summary_path: Path, steps: list[SignoffStep], results: list[dict[str, object]]) -> None:
    summary_path.parent.mkdir(parents=True, exist_ok=True)
    payload = {
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
        else output_dir / "ci_optional_ppp_products_summary.json"
    ).resolve()
    log_dir = output_dir / "ci_optional_ppp_products"
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
