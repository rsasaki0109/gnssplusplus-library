#!/usr/bin/env python3
"""Run optional MADOCA materialization diff and persist a CI summary."""

from __future__ import annotations

from pathlib import Path
import sys
from typing import Mapping


sys.path.insert(0, str(Path(__file__).resolve().parent))

import optional_diff_runner as runner  # noqa: E402


SUMMARY_SCHEMA = "ci_optional_madoca_materialization_diff.v2"

CONFIG = runner.DiffRunnerConfig(
    summary_schema=SUMMARY_SCHEMA,
    name="MADOCA materialization diff",
    markdown_title="Optional MADOCA Materialization Diff",
    slug="madoca_materialization_diff",
    analysis_script="madoca_materialization_diff.py",
    base_label="madocalib",
    candidate_label="native",
    base_env="GNSSPP_MADOCA_MATERIALIZATION_BASE_CSV",
    candidate_envs=(
        "GNSSPP_MADOCA_MATERIALIZATION_CANDIDATE_CSV",
        "GNSSPP_MADOCA_MATERIALIZATION_NATIVE_CSV",
    ),
    components_env="GNSSPP_MADOCA_MATERIALIZATION_COMPONENTS",
    fail_on_diff_env="GNSSPP_MADOCA_MATERIALIZATION_FAIL_ON_DIFF",
    default_components=(),
    skip_subject="MADOCA materialization",
    base_missing_label="base materialization CSV",
    candidate_missing_label="candidate materialization CSV",
    summary_filename="ci_optional_madoca_materialization_summary.json",
    log_dir_name="ci_optional_madoca_materialization",
    per_component_max_key="max_abs_delta",
    component_flag=None,
    extra_options=(
        runner.EnvOption(
            (
                "GNSSPP_MADOCA_MATERIALIZATION_NUMERIC_THRESHOLD",
                "GNSSPP_MADOCA_MATERIALIZATION_THRESHOLD",
            ),
            "--numeric-threshold",
            "float",
        ),
    ),
    input_summary_script="madoca_materialization_summary.py",
    input_summary_schema="madoca_materialization_summary.v1",
    input_summary_fail_on_issue=True,
)


DiffStep = runner.DiffStep


def repo_root_from_script() -> Path:
    return runner.repo_root_from_script()


def build_step_plan(
    repo_root: Path,
    output_dir: Path,
    environ: Mapping[str, str],
    *,
    python_executable: str = sys.executable,
) -> list[DiffStep]:
    return runner.build_step_plan(CONFIG, repo_root, output_dir, environ, python_executable=python_executable)


def run_step(step: DiffStep, repo_root: Path, log_dir: Path) -> dict[str, object]:
    return runner.run_step(CONFIG, step, repo_root, log_dir)


def render_markdown_summary(results: list[dict[str, object]]) -> str:
    return runner.render_markdown_summary(CONFIG, results)


def write_summary(summary_path: Path, steps: list[DiffStep], results: list[dict[str, object]]) -> None:
    runner.write_summary(CONFIG, summary_path, steps, results)


def main() -> int:
    return runner.main(CONFIG)


if __name__ == "__main__":
    raise SystemExit(main())
