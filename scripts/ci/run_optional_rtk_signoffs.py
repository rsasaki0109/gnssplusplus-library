#!/usr/bin/env python3
"""Run optional RTK-related sign-offs and persist a diagnostic summary."""

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
    skip_reason: str | None = None


@dataclass(frozen=True)
class SignoffContext:
    repo_root: Path
    output_dir: Path
    gnss_command: list[str]
    dataset_root: Path | None
    rtklib_bin: Path | None
    scorpion_input: Path | None
    scorpion_url: str


def repo_root_from_script() -> Path:
    return Path(__file__).resolve().parents[2]


def is_executable_file(path: Path | None) -> bool:
    return path is not None and path.is_file() and os.access(path, os.X_OK)


def build_context(
    repo_root: Path,
    output_dir: Path,
    environ: Mapping[str, str],
    *,
    python_executable: str = sys.executable,
) -> SignoffContext:
    dataset_root_value = environ.get("GNSSPP_PPC_DATASET_ROOT", "").strip()
    rtklib_bin_value = environ.get("GNSSPP_RTKLIB_BIN", "").strip()
    scorpion_input_value = environ.get("GNSSPP_SCORPION_MOVING_BASE_INPUT", "").strip()
    return SignoffContext(
        repo_root=repo_root,
        output_dir=output_dir,
        gnss_command=[python_executable, str(repo_root / "apps" / "gnss.py")],
        dataset_root=Path(dataset_root_value) if dataset_root_value else None,
        rtklib_bin=Path(rtklib_bin_value) if rtklib_bin_value else None,
        scorpion_input=Path(scorpion_input_value) if scorpion_input_value else None,
        scorpion_url=environ.get("GNSSPP_SCORPION_MOVING_BASE_URL", "").strip(),
    )


def make_step(name: str, slug: str, command: list[str], outputs: list[Path]) -> SignoffStep:
    return SignoffStep(name=name, slug=slug, command=command, outputs=[str(path) for path in outputs])


def skip_step(name: str, slug: str, outputs: list[Path], reason: str) -> SignoffStep:
    return SignoffStep(
        name=name,
        slug=slug,
        command=None,
        outputs=[str(path) for path in outputs],
        skip_reason=reason,
    )


def make_ppc_rtk_step(context: SignoffContext, city: str, *, require_rtklib: bool) -> SignoffStep:
    step_name = (
        f"PPC {city.capitalize()} RTK sign-off with RTKLIB comparison"
        if require_rtklib
        else f"PPC {city.capitalize()} RTK sign-off"
    )
    slug = f"ppc_{city}_run1_rtk"
    outputs = [
        context.output_dir / f"{slug}_summary.json",
        context.output_dir / f"{slug}.pos",
        context.output_dir / f"{slug}_events.csv",
    ]
    if require_rtklib:
        outputs.append(context.output_dir / f"{slug}_rtklib.pos")

    if context.dataset_root is None or not context.dataset_root.is_dir():
        return skip_step(step_name, slug, outputs, "PPC-Dataset root is unavailable.")
    if require_rtklib and not is_executable_file(context.rtklib_bin):
        return skip_step(step_name, slug, outputs, "RTKLIB binary is unavailable.")

    command = [
        *context.gnss_command,
        "ppc-rtk-signoff",
        "--dataset-root",
        str(context.dataset_root),
        "--city",
        city,
        "--max-epochs",
        "120",
        "--out",
        str(context.output_dir / f"{slug}.pos"),
        "--debug-epoch-log",
        str(context.output_dir / f"{slug}_events.csv"),
        "--summary-json",
        str(context.output_dir / f"{slug}_summary.json"),
    ]
    if require_rtklib:
        command.extend(
            [
                "--rtklib-bin",
                str(context.rtklib_bin),
                "--rtklib-pos",
                str(context.output_dir / f"{slug}_rtklib.pos"),
            ]
        )
    return make_step(step_name, slug, command, outputs)


def make_scorpion_step(context: SignoffContext) -> SignoffStep:
    slug = "scorpion_moving_base"
    name = "SCORPION moving-base sign-off"
    outputs = [
        context.output_dir / "scorpion_moving_base_summary.json",
        context.output_dir / "scorpion_moving_base" / "prepare_summary.json",
        context.output_dir / "scorpion_moving_base" / "products_summary.json",
        context.output_dir / "scorpion_moving_base" / "scorpion_moving_base.pos",
        context.output_dir / "scorpion_moving_base" / "scorpion_moving_base_matches.csv",
        context.output_dir / "scorpion_moving_base" / "scorpion_moving_base.png",
        context.output_dir / "scorpion_moving_base" / "reference.csv",
    ]

    input_args: list[str]
    if context.scorpion_input is not None and context.scorpion_input.exists():
        input_args = ["--input", str(context.scorpion_input)]
    elif context.scorpion_url:
        input_args = ["--input-url", context.scorpion_url]
    else:
        return skip_step(name, slug, outputs, "SCORPION moving-base input is unavailable.")

    command = [
        *context.gnss_command,
        "scorpion-moving-base-signoff",
        *input_args,
        "--summary-json",
        str(context.output_dir / "scorpion_moving_base_summary.json"),
        "--max-epochs",
        "120",
        "--require-matched-epochs-min",
        "80",
        "--require-fix-rate-min",
        "90",
        "--require-p95-baseline-error-max",
        "0.20",
        "--require-p95-heading-error-max",
        "10",
        "--require-realtime-factor-min",
        "1.0",
    ]
    return make_step(name, slug, command, outputs)


def build_step_plan(
    repo_root: Path,
    output_dir: Path,
    environ: Mapping[str, str],
    *,
    python_executable: str = sys.executable,
) -> list[SignoffStep]:
    context = build_context(
        repo_root,
        output_dir,
        environ,
        python_executable=python_executable,
    )
    return [
        make_ppc_rtk_step(context, "nagoya", require_rtklib=False),
        make_ppc_rtk_step(context, "tokyo", require_rtklib=True),
        make_scorpion_step(context),
    ]


def run_step(step: SignoffStep, repo_root: Path, log_dir: Path) -> dict[str, object]:
    log_path = log_dir / f"{step.slug}.log"
    record: dict[str, object] = {
        "name": step.name,
        "slug": step.slug,
        "outputs": step.outputs,
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


def render_markdown_summary(results: list[dict[str, object]]) -> str:
    passed = sum(1 for result in results if result["status"] == "passed")
    failed = sum(1 for result in results if result["status"] == "failed")
    skipped = sum(1 for result in results if result["status"] == "skipped")
    lines = [
        "## Optional RTK Sign-offs",
        "",
        f"- `passed`: `{passed}`",
        f"- `failed`: `{failed}`",
        f"- `skipped`: `{skipped}`",
        "",
        "| Step | Status | Detail |",
        "| --- | --- | --- |",
    ]
    for result in results:
        status = str(result["status"])
        detail = ""
        if status == "skipped":
            detail = str(result.get("skip_reason", ""))
        elif status == "passed":
            elapsed = result.get("elapsed_s")
            detail = f"{elapsed:.2f}s" if isinstance(elapsed, (float, int)) else ""
        elif status == "failed":
            log_path = result.get("log_path")
            detail = f"see `{log_path}`" if log_path else "failed"
        lines.append(f"| {result['name']} | `{status}` | {detail} |")
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
        args.summary_json if args.summary_json is not None else output_dir / "ci_optional_rtk_signoffs_summary.json"
    ).resolve()
    log_dir = output_dir / "ci_optional_rtk_signoffs"
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
