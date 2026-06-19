#!/usr/bin/env python3
"""Run optional CLAS ZD component diff and persist a CI summary."""

from __future__ import annotations

from pathlib import Path
import sys
from typing import Mapping


sys.path.insert(0, str(Path(__file__).resolve().parent))

import optional_diff_runner as runner  # noqa: E402


SUMMARY_SCHEMA = "ci_optional_clas_zd_component_diff.v2"

CONFIG = runner.DiffRunnerConfig(
    summary_schema=SUMMARY_SCHEMA,
    name="CLAS ZD component diff",
    markdown_title="Optional CLAS ZD Component Diff",
    slug="clas_zd_component_diff",
    analysis_script="clas_zd_component_diff.py",
    base_label="claslib",
    candidate_label="native",
    base_env="GNSSPP_CLAS_ZD_BASE_CSV",
    candidate_envs=(
        "GNSSPP_CLAS_ZD_CANDIDATE_CSV",
        "GNSSPP_CLAS_ZD_NATIVE_CSV",
    ),
    components_env="GNSSPP_CLAS_ZD_COMPONENTS",
    fail_on_diff_env="GNSSPP_CLAS_ZD_FAIL_ON_DIFF",
    default_components=(),
    skip_subject="CLAS ZD component",
    base_missing_label="base ZD component CSV",
    candidate_missing_label="candidate ZD component CSV",
    summary_filename="ci_optional_clas_zd_component_summary.json",
    log_dir_name="ci_optional_clas_zd_component",
    per_component_max_key="max_abs_delta_m",
    extra_options=(
        runner.EnvOption(("GNSSPP_CLAS_ZD_STAGE",), "--stage"),
        runner.EnvOption(("GNSSPP_CLAS_ZD_ROW_TYPE",), "--row-type"),
        runner.EnvOption(("GNSSPP_CLAS_ZD_SAT",), "--sat"),
        runner.EnvOption(("GNSSPP_CLAS_ZD_FREQ",), "--freq"),
        runner.EnvOption(("GNSSPP_CLAS_ZD_RINEX_CODE",), "--rinex-code"),
        runner.EnvOption(("GNSSPP_CLAS_ZD_DUPLICATE_POLICY",), "--duplicate-policy"),
        runner.EnvOption(
            ("GNSSPP_CLAS_ZD_THRESHOLD_M", "GNSSPP_CLAS_ZD_THRESHOLD"),
            "--component-threshold-m",
            "float",
        ),
    ),
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
