#!/usr/bin/env python3
"""Shared runner for optional CI diff artifacts."""

from __future__ import annotations

import argparse
from dataclasses import asdict, dataclass
import hashlib
import json
import os
from pathlib import Path
import subprocess
import sys
import time
from typing import Mapping, Sequence


@dataclass(frozen=True)
class EnvOption:
    env_names: tuple[str, ...]
    flag: str
    value_type: str = "str"


@dataclass(frozen=True)
class DiffRunnerConfig:
    summary_schema: str
    name: str
    markdown_title: str
    slug: str
    analysis_script: str
    base_label: str
    candidate_label: str
    base_env: str
    candidate_envs: tuple[str, ...]
    components_env: str
    fail_on_diff_env: str
    default_components: tuple[str, ...]
    skip_subject: str
    base_missing_label: str
    candidate_missing_label: str
    summary_filename: str
    log_dir_name: str
    per_component_max_key: str
    extra_options: tuple[EnvOption, ...] = ()
    component_flag: str | None = "--component"
    input_summary_script: str | None = None
    input_summary_schema: str | None = None
    input_summary_fail_on_issue: bool = False
    input_summary_extra_options: tuple[EnvOption, ...] = ()
    input_summary_require_components: bool = False
    highlight_components: tuple[str, ...] = ()


@dataclass(frozen=True)
class DiffStep:
    name: str
    slug: str
    command: list[str] | None
    outputs: list[str]
    summary_json: str
    skip_reason: str | None = None
    pre_commands: tuple[tuple[str, ...], ...] = ()


@dataclass(frozen=True)
class DiffContext:
    repo_root: Path
    output_dir: Path
    python_executable: str
    base_csv: Path | None
    candidate_csv: Path | None
    components: list[str]
    option_values: tuple[tuple[str, str], ...]
    input_summary_option_values: tuple[tuple[str, str], ...]
    fail_on_diff: bool


IDENTITY_PROVENANCE_METRIC_KEYS = (
    "gps_l2w_rows",
    "gps_l2w_bias_exact_identity_rows",
    "gps_l2w_observation_exact_match_rows",
    "gps_l2w_observation_family_fallback_rows",
    "gps_l2w_code_bias_fallback_rows",
)

STATUS_ORDER = ("passed", "failed", "blocked_infrastructure", "not_applicable", "skipped")


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


def optional_env(environ: Mapping[str, str], names: Sequence[str]) -> str:
    for name in names:
        value = environ.get(name, "").strip()
        if value:
            return value
    return ""


def parse_option_values(option: EnvOption, environ: Mapping[str, str]) -> list[tuple[str, str]]:
    value = optional_env(environ, option.env_names)
    if not value:
        return []
    if option.value_type == "csv":
        return [(option.flag, item) for item in parse_components(value)]
    if option.value_type == "int_csv":
        return [(option.flag, str(int(item))) for item in parse_components(value)]
    if option.value_type == "float_csv":
        return [(option.flag, str(float(item))) for item in parse_components(value)]
    if option.value_type == "int":
        return [(option.flag, str(int(value)))]
    if option.value_type == "float":
        return [(option.flag, str(float(value)))]
    if option.value_type != "str":
        raise ValueError(f"unsupported option value type: {option.value_type}")
    return [(option.flag, value)]


def truthy(value: str) -> bool:
    return value.strip().lower() in {"1", "true", "yes", "y", "on"}


def file_sha256(path: Path) -> str | None:
    if not path.is_file():
        return None
    digest = hashlib.sha256()
    with path.open("rb") as handle:
        for chunk in iter(lambda: handle.read(1024 * 1024), b""):
            digest.update(chunk)
    return digest.hexdigest()


def artifact_record(path: Path, *, role: str, required: bool) -> dict[str, object]:
    exists = path.is_file()
    record: dict[str, object] = {
        "role": role,
        "path": str(path),
        "required": required,
        "exists": exists,
    }
    if exists:
        record["bytes"] = path.stat().st_size
        record["sha256"] = file_sha256(path)
    return record


def metric_label(value: object) -> str:
    text = str(value).strip().lower()
    return "".join(character if character.isalnum() else "_" for character in text).strip("_")


def build_context(
    config: DiffRunnerConfig,
    repo_root: Path,
    output_dir: Path,
    environ: Mapping[str, str],
    *,
    python_executable: str = sys.executable,
) -> DiffContext:
    components = parse_components(environ.get(config.components_env, ""))
    if not components:
        components = list(config.default_components)
    return DiffContext(
        repo_root=repo_root,
        output_dir=output_dir,
        python_executable=python_executable,
        base_csv=resolve_optional_path(repo_root, environ.get(config.base_env, "")),
        candidate_csv=resolve_optional_path(
            repo_root,
            optional_env(environ, config.candidate_envs),
        ),
        components=components,
        option_values=tuple(
            pair
            for option in config.extra_options
            for pair in parse_option_values(option, environ)
        ),
        input_summary_option_values=tuple(
            pair
            for option in config.input_summary_extra_options
            for pair in parse_option_values(option, environ)
        ),
        fail_on_diff=truthy(environ.get(config.fail_on_diff_env, "")),
    )


def make_step(config: DiffRunnerConfig, context: DiffContext) -> DiffStep:
    report_json = context.output_dir / f"{config.slug}.json"
    details_csv = context.output_dir / f"{config.slug}.csv"
    summary_json = context.output_dir / f"{config.slug}_summary.json"
    outputs = [report_json, details_csv]
    pre_commands: list[tuple[str, ...]] = []

    missing = [
        (label, path)
        for label, path in (
            (config.base_missing_label, context.base_csv),
            (config.candidate_missing_label, context.candidate_csv),
        )
        if path is None or not path.is_file()
    ]
    if missing:
        missing_text = ", ".join(f"{label}: {path}" for label, path in missing)
        return DiffStep(
            name=config.name,
            slug=config.slug,
            command=None,
            outputs=[str(path) for path in outputs],
            summary_json=str(summary_json),
            skip_reason=f"{config.skip_subject} input is unavailable ({missing_text}).",
        )

    assert context.base_csv is not None
    assert context.candidate_csv is not None
    if config.input_summary_script is not None:
        for label, snapshot_csv in (
            (config.base_label, context.base_csv),
            (config.candidate_label, context.candidate_csv),
        ):
            normalized_label = metric_label(label) or label
            snapshot_summary = context.output_dir / f"{config.slug}_{normalized_label}_snapshot_summary.json"
            outputs.append(snapshot_summary)
            command = [
                context.python_executable,
                str(context.repo_root / "scripts" / "analysis" / config.input_summary_script),
                str(snapshot_csv),
                "--json-out",
                str(snapshot_summary),
            ]
            if config.input_summary_fail_on_issue:
                command.append("--fail-on-issue")
            for flag, value in context.input_summary_option_values:
                command.extend([flag, value])
            if config.input_summary_require_components:
                for component in context.components:
                    command.extend(["--require-component", component])
            pre_commands.append(tuple(command))

    command = [
        context.python_executable,
        str(context.repo_root / "scripts" / "analysis" / config.analysis_script),
        str(context.base_csv),
        str(context.candidate_csv),
        "--base-label",
        config.base_label,
        "--candidate-label",
        config.candidate_label,
        "--json-out",
        str(report_json),
        "--details-csv",
        str(details_csv),
    ]
    if config.component_flag is not None:
        for component in context.components:
            command.extend([config.component_flag, component])
    for flag, value in context.option_values:
        command.extend([flag, value])
    if context.fail_on_diff:
        command.append("--fail-on-diff")

    return DiffStep(
        name=config.name,
        slug=config.slug,
        command=command,
        outputs=[str(path) for path in outputs],
        summary_json=str(summary_json),
        pre_commands=tuple(pre_commands),
    )


def build_step_plan(
    config: DiffRunnerConfig,
    repo_root: Path,
    output_dir: Path,
    environ: Mapping[str, str],
    *,
    python_executable: str = sys.executable,
) -> list[DiffStep]:
    context = build_context(config, repo_root, output_dir, environ, python_executable=python_executable)
    return [make_step(config, context)]


def load_diff_metrics(config: DiffRunnerConfig, report_json: Path) -> dict[str, object]:
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
        "base_duplicate_keys",
        "candidate_duplicate_keys",
        "components_compared",
        "numeric_components_compared",
        "discrete_mismatches",
        "threshold_exceedances",
        "numeric_threshold_exceedances",
        "row_set_complete",
        "sat_filter",
        "freq_filter",
        "rinex_code_filter",
        "duplicate_policy",
    ]:
        if key in payload:
            metrics[key] = payload[key]
    if "components_compared" not in metrics and "numeric_components_compared" in payload:
        metrics["components_compared"] = payload["numeric_components_compared"]
    if "threshold_exceedances" not in metrics and "numeric_threshold_exceedances" in payload:
        metrics["threshold_exceedances"] = payload["numeric_threshold_exceedances"]
    per_component = payload.get("per_component")
    if not isinstance(per_component, list):
        per_component = payload.get("per_numeric_component")
    if isinstance(per_component, list):
        max_abs_delta = 0.0
        per_component_max_abs_delta: dict[str, float] = {}
        for item in per_component:
            if not isinstance(item, dict):
                continue
            component = item.get("component")
            value = item.get(config.per_component_max_key)
            if isinstance(value, (int, float)):
                parsed_value = float(value)
                max_abs_delta = max(max_abs_delta, parsed_value)
                if isinstance(component, str) and component:
                    per_component_max_abs_delta[component] = parsed_value
        metrics["max_abs_delta"] = max_abs_delta
        if per_component_max_abs_delta:
            metrics["per_component_max_abs_delta"] = per_component_max_abs_delta
    top_component_deltas = payload.get("top_component_deltas")
    if isinstance(top_component_deltas, list) and top_component_deltas:
        first_delta = top_component_deltas[0]
        if isinstance(first_delta, dict):
            component = first_delta.get("component")
            delta = first_delta.get("delta_m")
            if not isinstance(delta, (int, float)):
                delta = first_delta.get("delta")
            abs_delta = first_delta.get("abs_delta_m")
            if not isinstance(abs_delta, (int, float)):
                abs_delta = first_delta.get("abs_delta")
            if isinstance(component, str) and component:
                metrics["top_delta_component"] = component
            if isinstance(delta, (int, float)):
                metrics["top_delta_delta"] = float(delta)
            if isinstance(abs_delta, (int, float)):
                metrics["top_delta_abs_delta"] = float(abs_delta)
            key_parts = []
            for key in ("week", "tow", "iteration", "stage", "row_type", "sat", "freq", "rinex_code"):
                value = first_delta.get(key)
                if value is not None:
                    metrics[f"top_delta_{key}"] = value
                    key_parts.append(str(value))
            if key_parts:
                metrics["top_delta_key"] = "/".join(key_parts)
    top_row_breakdowns = payload.get("top_row_component_breakdowns")
    if isinstance(top_row_breakdowns, list) and top_row_breakdowns:
        first_row = top_row_breakdowns[0]
        if isinstance(first_row, dict):
            sum_abs_delta = first_row.get("sum_abs_delta_m")
            max_abs_delta = first_row.get("max_abs_delta_m")
            if isinstance(sum_abs_delta, (int, float)):
                metrics["top_row_sum_abs_delta"] = float(sum_abs_delta)
            if isinstance(max_abs_delta, (int, float)):
                metrics["top_row_max_abs_delta"] = float(max_abs_delta)
            key_parts = []
            for key in ("week", "tow", "row_type", "sat", "freq", "rinex_code"):
                value = first_row.get(key)
                if value is not None:
                    metrics[f"top_row_{key}"] = value
                    key_parts.append(str(value))
            if key_parts:
                metrics["top_row_key"] = "/".join(key_parts)
            components = first_row.get("components")
            if isinstance(components, list) and components:
                first_component = components[0]
                if isinstance(first_component, dict):
                    component = first_component.get("component")
                    delta = first_component.get("delta_m")
                    abs_delta = first_component.get("abs_delta_m")
                    if isinstance(component, str) and component:
                        metrics["top_row_dominant_component"] = component
                    if isinstance(delta, (int, float)):
                        metrics["top_row_dominant_delta"] = float(delta)
                    if isinstance(abs_delta, (int, float)):
                        metrics["top_row_dominant_abs_delta"] = float(abs_delta)
    identity_provenance = payload.get("identity_provenance")
    if isinstance(identity_provenance, dict):
        for label, summary in identity_provenance.items():
            if not isinstance(summary, dict):
                continue
            normalized_label = metric_label(label)
            if not normalized_label:
                continue
            for key in IDENTITY_PROVENANCE_METRIC_KEYS:
                value = summary.get(key)
                if isinstance(value, (int, float)):
                    metrics[f"identity_{normalized_label}_{key}"] = int(value)
    return metrics


def load_input_summary_metrics(config: DiffRunnerConfig, outputs: Sequence[str]) -> dict[str, object]:
    if config.input_summary_schema is None:
        return {}
    metrics: dict[str, object] = {}
    for label in (config.base_label, config.candidate_label):
        normalized_label = metric_label(label)
        if not normalized_label:
            continue
        suffix = f"{config.slug}_{normalized_label}_snapshot_summary.json"
        summary_path = next((Path(output) for output in outputs if output.endswith(suffix)), None)
        if summary_path is None or not summary_path.is_file():
            continue
        try:
            payload = json.loads(summary_path.read_text(encoding="utf-8"))
        except json.JSONDecodeError:
            continue
        if not isinstance(payload, dict):
            continue
        metrics[f"{normalized_label}_snapshot_schema"] = payload.get("schema")
        metrics[f"{normalized_label}_snapshot_status"] = payload.get("status")
        rows = payload.get("rows")
        if isinstance(rows, int):
            metrics[f"{normalized_label}_snapshot_rows"] = rows
        systems = payload.get("systems")
        if isinstance(systems, dict):
            metrics[f"{normalized_label}_snapshot_systems"] = systems
        row_key = payload.get("row_key")
        if isinstance(row_key, dict):
            duplicate_groups = row_key.get("duplicate_groups")
            if isinstance(duplicate_groups, int):
                metrics[f"{normalized_label}_snapshot_duplicate_groups"] = duplicate_groups
            max_duplicate_occurrences = row_key.get("max_duplicate_occurrences")
            if isinstance(max_duplicate_occurrences, int):
                metrics[f"{normalized_label}_snapshot_max_duplicate_occurrences"] = max_duplicate_occurrences
        bias_identity = payload.get("bias_identity")
        if isinstance(bias_identity, dict):
            for key in ("rows_with_code_bias", "rows_with_phase_bias"):
                value = bias_identity.get(key)
                if isinstance(value, int):
                    metrics[f"{normalized_label}_snapshot_{key}"] = value
        issue_counts = payload.get("issue_counts")
        if isinstance(issue_counts, dict):
            metrics[f"{normalized_label}_snapshot_issue_counts"] = issue_counts
    return metrics


def run_step(config: DiffRunnerConfig, step: DiffStep, repo_root: Path, log_dir: Path) -> dict[str, object]:
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
        record["status"] = "blocked_infrastructure"
        record["block_reason"] = step.skip_reason
        record["skip_reason"] = step.skip_reason
        log_path.write_text(step.skip_reason + "\n", encoding="utf-8")
        record["artifacts"] = [
            artifact_record(log_path, role="log", required=True),
            *[
                artifact_record(Path(output), role="declared_output", required=False)
                for output in step.outputs
            ],
        ]
        print(f"Blocked {step.name}: {step.skip_reason}")
        return record

    assert step.command is not None
    commands = [list(command) for command in step.pre_commands]
    commands.append(step.command)
    command_display = " && ".join(" ".join(command) for command in commands)
    for command in commands:
        print("+ " + " ".join(command))
    started = time.monotonic()
    combined_log = ""
    returncode = 0
    failed_command: list[str] | None = None
    for command in commands:
        single_display = " ".join(command)
        completed = subprocess.run(
            command,
            cwd=repo_root,
            text=True,
            capture_output=True,
            check=False,
        )
        if combined_log and not combined_log.endswith("\n"):
            combined_log += "\n"
        combined_log += f"$ {single_display}\n\n"
        combined_log += completed.stdout
        if completed.stderr:
            if combined_log and not combined_log.endswith("\n"):
                combined_log += "\n"
            combined_log += completed.stderr
        if completed.returncode != 0:
            returncode = completed.returncode
            failed_command = command
            break
    elapsed = time.monotonic() - started
    log_path.write_text(combined_log, encoding="utf-8")
    record["command"] = step.command
    if step.pre_commands:
        record["pre_commands"] = [list(command) for command in step.pre_commands]
    record["elapsed_s"] = elapsed
    record["returncode"] = returncode
    if failed_command is not None:
        record["failed_command"] = failed_command
    metrics = load_diff_metrics(config, report_json)
    metrics.update(load_input_summary_metrics(config, step.outputs))
    if metrics:
        record["metrics"] = metrics
    if returncode == 0:
        missing_outputs = [output for output in step.outputs if not Path(output).exists()]
        record["missing_outputs"] = missing_outputs
        if missing_outputs:
            record["status"] = "failed"
            print(
                f"Failed {step.name} in {elapsed:.2f}s; "
                f"missing expected outputs: {', '.join(missing_outputs)}"
            )
        else:
            record["status"] = "passed"
            print(f"Passed {step.name} in {elapsed:.2f}s")
    else:
        record["status"] = "failed"
        lines = combined_log.splitlines()
        tail = "\n".join(lines[-20:])
        if tail:
            print(tail)
        failed_display = " ".join(failed_command) if failed_command is not None else command_display
        print(f"Failed {step.name} in {elapsed:.2f}s during `{failed_display}`; see {log_path}")
    record["artifacts"] = [
        artifact_record(log_path, role="log", required=True),
        *[
            artifact_record(Path(output), role="declared_output", required=returncode == 0)
            for output in step.outputs
        ],
    ]
    return record


def format_metric(value: object) -> str:
    if isinstance(value, float):
        return f"{value:.6g}"
    return str(value)


def render_result_detail(config: DiffRunnerConfig, result: dict[str, object]) -> str:
    status = str(result["status"])
    if status in {"blocked_infrastructure", "not_applicable", "skipped"}:
        return str(result.get("block_reason") or result.get("skip_reason", ""))
    if status == "failed":
        missing_outputs = result.get("missing_outputs")
        if isinstance(missing_outputs, list) and missing_outputs:
            return "missing " + ", ".join(f"`{output}`" for output in missing_outputs)
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
        per_component = metrics.get("per_component_max_abs_delta")
        if isinstance(per_component, dict):
            for component in config.highlight_components:
                value = per_component.get(component)
                if isinstance(value, (int, float)):
                    detail_parts.append(f"`{component}` |delta| {format_metric(value)}")
        top_delta_component = metrics.get("top_delta_component")
        top_delta_component_abs = metrics.get("top_delta_abs_delta")
        if top_delta_component is not None:
            if top_delta_component_abs is not None:
                detail_parts.append(
                    "top delta "
                    f"`{top_delta_component}` |delta| {format_metric(top_delta_component_abs)}"
                )
            else:
                detail_parts.append(f"top delta `{top_delta_component}`")
        top_row_sum = metrics.get("top_row_sum_abs_delta")
        if top_row_sum is not None:
            detail_parts.append(f"top row sum |delta| {format_metric(top_row_sum)}")
        top_row_component = metrics.get("top_row_dominant_component")
        top_row_component_abs = metrics.get("top_row_dominant_abs_delta")
        if top_row_component is not None:
            if top_row_component_abs is not None:
                detail_parts.append(
                    "top component "
                    f"`{top_row_component}` |delta| {format_metric(top_row_component_abs)}"
                )
            else:
                detail_parts.append(f"top component `{top_row_component}`")
        threshold_exceedances = metrics.get("threshold_exceedances")
        if threshold_exceedances is not None:
            detail_parts.append(f"threshold exceedances `{threshold_exceedances}`")
        discrete_mismatches = metrics.get("discrete_mismatches")
        if discrete_mismatches is not None:
            detail_parts.append(f"discrete mismatches `{discrete_mismatches}`")
        base_duplicates = metrics.get("base_duplicate_keys")
        candidate_duplicates = metrics.get("candidate_duplicate_keys")
        if base_duplicates is not None or candidate_duplicates is not None:
            detail_parts.append(f"duplicate keys `{base_duplicates}/{candidate_duplicates}`")
        snapshot_rows = [
            (str(key).removesuffix("_snapshot_rows"), value)
            for key, value in metrics.items()
            if str(key).endswith("_snapshot_rows")
        ]
        if snapshot_rows:
            preferred_order = {"madocalib": 0, "claslib": 0, "native": 1}
            ordered = sorted(
                snapshot_rows,
                key=lambda item: (preferred_order.get(item[0], 10), item[0]),
            )
            detail_parts.append(
                "snapshot rows `"
                + "/".join(str(value) for _, value in ordered[:2])
                + "`"
            )
        native_l2w_rows = metrics.get("identity_native_gps_l2w_rows")
        if native_l2w_rows is not None:
            detail_parts.append(f"native L2W `{native_l2w_rows}`")
            exact_bias = metrics.get("identity_native_gps_l2w_bias_exact_identity_rows")
            exact_match = metrics.get("identity_native_gps_l2w_observation_exact_match_rows")
            if exact_bias is not None or exact_match is not None:
                detail_parts.append(f"native L2W exact bias/match `{exact_bias}/{exact_match}`")
            obs_fallback = metrics.get(
                "identity_native_gps_l2w_observation_family_fallback_rows"
            )
            code_fallback = metrics.get("identity_native_gps_l2w_code_bias_fallback_rows")
            if obs_fallback is not None or code_fallback is not None:
                detail_parts.append(f"native L2W fallback obs/code `{obs_fallback}/{code_fallback}`")
    return ", ".join(detail_parts)


def render_markdown_summary(config: DiffRunnerConfig, results: list[dict[str, object]]) -> str:
    counts = status_counts(results)
    lines = [
        f"## {config.markdown_title}",
        "",
        *[f"- `{status}`: `{count}`" for status, count in counts.items()],
        "",
        "| Step | Status | Detail |",
        "| --- | --- | --- |",
    ]
    for result in results:
        lines.append(
            f"| {result['name']} | `{result['status']}` | {render_result_detail(config, result)} |"
        )
    lines.append("")
    return "\n".join(lines)


def status_counts(results: list[dict[str, object]]) -> dict[str, int]:
    counts = {status: 0 for status in STATUS_ORDER}
    for result in results:
        status = str(result["status"])
        counts.setdefault(status, 0)
        counts[status] += 1
    return counts


def aggregate_status(results: list[dict[str, object]]) -> str:
    if any(result["status"] == "failed" for result in results):
        return "failed"
    if any(result["status"] == "blocked_infrastructure" for result in results):
        return "blocked_infrastructure"
    if any(result["status"] == "passed" for result in results):
        return "passed"
    return "not_applicable"


def next_actions(results: list[dict[str, object]]) -> list[str]:
    if any(result["status"] == "blocked_infrastructure" for result in results):
        return [
            "Provide the missing configured CSV inputs, then rerun the optional diff.",
            "Treat blocked_infrastructure as missing evidence, not as a passing oracle sign-off.",
        ]
    if any(result["status"] == "failed" for result in results):
        return ["Inspect the per-step log and declared output artifacts before changing solver behavior."]
    return []


def write_summary(
    config: DiffRunnerConfig,
    summary_path: Path,
    steps: list[DiffStep],
    results: list[dict[str, object]],
) -> None:
    summary_path.parent.mkdir(parents=True, exist_ok=True)
    payload = {
        "summary_schema": config.summary_schema,
        "contract": "optional_diff_artifact_contract.v1",
        "status": aggregate_status(results),
        "next_actions": next_actions(results),
        "steps": [asdict(step) for step in steps],
        "results": results,
        "counts": status_counts(results),
    }
    summary_path.write_text(json.dumps(payload, indent=2) + "\n", encoding="utf-8")


def append_github_step_summary(config: DiffRunnerConfig, path: Path, results: list[dict[str, object]]) -> None:
    with path.open("a", encoding="utf-8") as handle:
        handle.write(render_markdown_summary(config, results))
        handle.write("\n")


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser()
    parser.add_argument("--repo-root", type=Path, default=repo_root_from_script())
    parser.add_argument("--output-dir", type=Path, default=None)
    parser.add_argument("--summary-json", type=Path, default=None)
    parser.add_argument("--github-step-summary", type=Path, default=None)
    return parser.parse_args()


def main(config: DiffRunnerConfig) -> int:
    args = parse_args()
    repo_root = args.repo_root.resolve()
    output_dir = (args.output_dir if args.output_dir is not None else repo_root / "output").resolve()
    summary_json = (
        args.summary_json
        if args.summary_json is not None
        else output_dir / config.summary_filename
    ).resolve()
    log_dir = output_dir / config.log_dir_name
    output_dir.mkdir(parents=True, exist_ok=True)
    log_dir.mkdir(parents=True, exist_ok=True)

    steps = build_step_plan(config, repo_root, output_dir, os.environ)
    results = [run_step(config, step, repo_root, log_dir) for step in steps]
    write_summary(config, summary_json, steps, results)
    summary_path = args.github_step_summary
    if summary_path is None:
        summary_env = os.environ.get("GITHUB_STEP_SUMMARY", "").strip()
        summary_path = Path(summary_env) if summary_env else None
    if summary_path is not None:
        append_github_step_summary(config, summary_path, results)
    return 1 if any(result["status"] == "failed" for result in results) else 0
