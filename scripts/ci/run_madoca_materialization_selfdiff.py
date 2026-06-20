#!/usr/bin/env python3
"""Generate a public MADOCA materialization dump and self-diff it in CI."""

from __future__ import annotations

import argparse
from dataclasses import dataclass
import json
import os
from pathlib import Path
import shutil
import subprocess
import sys
import time
from typing import Mapping, Sequence


sys.path.insert(0, str(Path(__file__).resolve().parent))

from optional_diff_runner import artifact_record, truthy  # noqa: E402


SUMMARY_SCHEMA = "ci_madoca_materialization_selfdiff.v2"
CONTRACT = "madoca_materialization_selfdiff.v1"
DEFAULT_MADOCALIB_REPO = "https://github.com/QZSS-Strategy-Office/madocalib.git"
DEFAULT_MADOCALIB_REF = "0089f7dc97e8e2ba283a40be2edf4b73a140df6c"
DEFAULT_MIN_ROWS = 1000
DEFAULT_REQUIRED_SYSTEMS = ("GPS", "GLONASS", "Galileo", "QZSS", "BeiDou")
DEFAULT_REQUIRED_BIAS_IDS = ("2", "8", "9", "14", "22")
REQUIRED_DATA_FILES = (
    "sample_data/data/rinex/BRDM00DLR_S_20250910000_01D_MN.rnx",
    "sample_data/data/l6/2025/091/2025091A.204.l6",
)
TINY_OBS_TEXT = """\
     3.04           OBSERVATION DATA    M                   RINEX VERSION / TYPE
GNSSPP              CI                  20250620 000000 UTC PGM / RUN BY / DATE
  2025     4     1     0     0    0.0000000     GPS         TIME OF FIRST OBS
                                                            END OF HEADER
"""


@dataclass(frozen=True)
class RunPaths:
    output_dir: Path
    work_dir: Path
    log_path: Path
    summary_json: Path
    checkout_dir: Path
    tiny_obs: Path
    native_dump: Path
    native_summary_json: Path
    native_materialization_summary_json: Path
    selfdiff_json: Path
    selfdiff_csv: Path


@dataclass(frozen=True)
class RunConfig:
    repo_root: Path
    output_dir: Path
    data_root: Path | None
    auto_fetch: bool
    fail_on_blocked: bool
    madocalib_repo: str
    madocalib_ref: str
    min_rows: int
    required_systems: tuple[str, ...]
    required_code_bias_ids: tuple[str, ...]
    required_phase_bias_ids: tuple[str, ...]
    python_executable: str


@dataclass(frozen=True)
class Evaluation:
    status: str
    failures: list[str]
    metrics: dict[str, object]


def repo_root_from_script() -> Path:
    return Path(__file__).resolve().parents[2]


def default_paths(output_dir: Path) -> RunPaths:
    work_dir = output_dir / "madoca_materialization_selfdiff"
    return RunPaths(
        output_dir=output_dir,
        work_dir=work_dir,
        log_path=output_dir / "ci_madoca_materialization_selfdiff" / "madoca_materialization_selfdiff.log",
        summary_json=output_dir / "ci_madoca_materialization_selfdiff_summary.json",
        checkout_dir=work_dir / "madocalib",
        tiny_obs=work_dir / "tiny.obs",
        native_dump=work_dir / "native_materialization.csv",
        native_summary_json=work_dir / "native_summary.json",
        native_materialization_summary_json=work_dir / "native_materialization_summary.json",
        selfdiff_json=work_dir / "selfdiff.json",
        selfdiff_csv=work_dir / "selfdiff.csv",
    )


def resolve_optional_path(repo_root: Path, value: str) -> Path | None:
    text = value.strip()
    if not text:
        return None
    path = Path(text)
    return path if path.is_absolute() else repo_root / path


def env_or(environ: Mapping[str, str], name: str, default: str) -> str:
    value = environ.get(name, "").strip()
    return value if value else default


def parse_csv_list(text: str) -> tuple[str, ...]:
    return tuple(item.strip() for item in text.split(",") if item.strip())


def parse_config(args: argparse.Namespace, environ: Mapping[str, str]) -> RunConfig:
    repo_root = args.repo_root.resolve()
    output_dir = (args.output_dir if args.output_dir is not None else repo_root / "output").resolve()
    return RunConfig(
        repo_root=repo_root,
        output_dir=output_dir,
        data_root=resolve_optional_path(
            repo_root,
            environ.get("GNSSPP_MADOCA_MATERIALIZATION_DATA_ROOT", ""),
        ),
        auto_fetch=truthy(env_or(environ, "GNSSPP_MADOCA_MATERIALIZATION_AUTO_FETCH", "1")),
        fail_on_blocked=truthy(env_or(environ, "GNSSPP_MADOCA_MATERIALIZATION_FAIL_ON_BLOCKED", "1")),
        madocalib_repo=env_or(
            environ,
            "GNSSPP_MADOCA_MATERIALIZATION_MADOCALIB_REPO",
            DEFAULT_MADOCALIB_REPO,
        ),
        madocalib_ref=env_or(
            environ,
            "GNSSPP_MADOCA_MATERIALIZATION_MADOCALIB_REF",
            DEFAULT_MADOCALIB_REF,
        ),
        min_rows=int(env_or(environ, "GNSSPP_MADOCA_MATERIALIZATION_ROWS_MIN", str(DEFAULT_MIN_ROWS))),
        required_systems=parse_csv_list(
            env_or(
                environ,
                "GNSSPP_MADOCA_MATERIALIZATION_REQUIRED_SYSTEMS",
                ",".join(DEFAULT_REQUIRED_SYSTEMS),
            )
        ),
        required_code_bias_ids=parse_csv_list(
            env_or(
                environ,
                "GNSSPP_MADOCA_MATERIALIZATION_REQUIRED_CODE_BIAS_IDS",
                ",".join(DEFAULT_REQUIRED_BIAS_IDS),
            )
        ),
        required_phase_bias_ids=parse_csv_list(
            env_or(
                environ,
                "GNSSPP_MADOCA_MATERIALIZATION_REQUIRED_PHASE_BIAS_IDS",
                ",".join(DEFAULT_REQUIRED_BIAS_IDS),
            )
        ),
        python_executable=sys.executable,
    )


def data_root_failures(data_root: Path) -> list[str]:
    return [name for name in REQUIRED_DATA_FILES if not (data_root / name).is_file()]


def run_logged(
    command: Sequence[str],
    *,
    cwd: Path,
    log_path: Path,
    env: Mapping[str, str] | None = None,
) -> subprocess.CompletedProcess[str]:
    display = " ".join(command)
    with log_path.open("a", encoding="utf-8") as handle:
        handle.write(f"$ {display}\n")
    started = time.monotonic()
    completed = subprocess.run(
        list(command),
        cwd=cwd,
        env=dict(env) if env is not None else None,
        text=True,
        capture_output=True,
        check=False,
    )
    elapsed = time.monotonic() - started
    combined = completed.stdout
    if completed.stderr:
        if combined and not combined.endswith("\n"):
            combined += "\n"
        combined += completed.stderr
    with log_path.open("a", encoding="utf-8") as handle:
        handle.write(combined)
        if combined and not combined.endswith("\n"):
            handle.write("\n")
        handle.write(f"# exit={completed.returncode} elapsed_s={elapsed:.3f}\n\n")
    return completed


def checkout_madocalib_data(config: RunConfig, paths: RunPaths) -> tuple[Path | None, str | None]:
    if config.data_root is not None:
        missing = data_root_failures(config.data_root)
        if not missing:
            return config.data_root, None
        return None, f"configured MADOCALIB data root is missing required files: {', '.join(missing)}"

    if not config.auto_fetch:
        return None, "GNSSPP_MADOCA_MATERIALIZATION_DATA_ROOT is unset and auto-fetch is disabled"

    if paths.checkout_dir.exists():
        shutil.rmtree(paths.checkout_dir)
    paths.checkout_dir.parent.mkdir(parents=True, exist_ok=True)
    commands = [
        ["git", "init", str(paths.checkout_dir)],
        ["git", "-C", str(paths.checkout_dir), "remote", "add", "origin", config.madocalib_repo],
        ["git", "-C", str(paths.checkout_dir), "sparse-checkout", "init", "--no-cone"],
        [
            "git",
            "-C",
            str(paths.checkout_dir),
            "sparse-checkout",
            "set",
            *REQUIRED_DATA_FILES,
        ],
        ["git", "-C", str(paths.checkout_dir), "fetch", "--depth", "1", "origin", config.madocalib_ref],
        ["git", "-C", str(paths.checkout_dir), "checkout", "--detach", "FETCH_HEAD"],
    ]
    for command in commands:
        completed = run_logged(command, cwd=config.repo_root, log_path=paths.log_path)
        if completed.returncode != 0:
            return None, f"failed to fetch MADOCALIB public data: {' '.join(command)}"

    missing = data_root_failures(paths.checkout_dir)
    if missing:
        return None, f"fetched MADOCALIB data is missing required files: {', '.join(missing)}"
    return paths.checkout_dir, None


def build_dump_command(config: RunConfig, paths: RunPaths, data_root: Path) -> list[str]:
    binary = config.repo_root / "build" / "apps" / "gnss_ppp"
    return [
        str(binary),
        "--obs",
        str(paths.tiny_obs),
        "--nav",
        str(data_root / REQUIRED_DATA_FILES[0]),
        "--madoca-l6",
        str(data_root / REQUIRED_DATA_FILES[1]),
        "--madoca-materialization-dump",
        str(paths.native_dump),
        "--madoca-materialization-dump-only",
        "--summary-json",
        str(paths.native_summary_json),
        "--quiet",
    ]


def build_selfdiff_command(config: RunConfig, paths: RunPaths) -> list[str]:
    return [
        config.python_executable,
        str(config.repo_root / "scripts" / "analysis" / "madoca_materialization_diff.py"),
        str(paths.native_dump),
        str(paths.native_dump),
        "--base-label",
        "native",
        "--candidate-label",
        "native",
        "--json-out",
        str(paths.selfdiff_json),
        "--details-csv",
        str(paths.selfdiff_csv),
        "--fail-on-diff",
    ]


def build_materialization_summary_command(config: RunConfig, paths: RunPaths) -> list[str]:
    command = [
        config.python_executable,
        str(config.repo_root / "scripts" / "analysis" / "madoca_materialization_summary.py"),
        str(paths.native_dump),
        "--json-out",
        str(paths.native_materialization_summary_json),
        "--require-rows-min",
        str(config.min_rows),
        "--require-no-duplicate-keys",
        "--fail-on-issue",
    ]
    for system in config.required_systems:
        command.extend(["--require-system", system])
    for signal_id in config.required_code_bias_ids:
        command.extend(["--require-code-bias-id", signal_id])
    for signal_id in config.required_phase_bias_ids:
        command.extend(["--require-phase-bias-id", signal_id])
    return command


def load_json(path: Path) -> dict[str, object]:
    payload = json.loads(path.read_text(encoding="utf-8"))
    if not isinstance(payload, dict):
        raise ValueError(f"{path}: expected JSON object")
    return payload


def count_csv_data_rows(path: Path) -> int:
    if not path.is_file():
        return 0
    with path.open(encoding="utf-8") as handle:
        return max(sum(1 for _line in handle) - 1, 0)


def positive_count(mapping: Mapping[object, object], key: str) -> bool:
    try:
        return int(mapping.get(key, 0)) > 0
    except (TypeError, ValueError):
        return False


def evaluate(
    *,
    config: RunConfig,
    paths: RunPaths,
    dump_summary: dict[str, object],
    materialization_summary: dict[str, object],
    selfdiff_report: dict[str, object],
) -> Evaluation:
    failures: list[str] = []
    csv_rows = count_csv_data_rows(paths.native_dump)
    summary_rows = dump_summary.get("madoca_materialization_rows")
    if not isinstance(summary_rows, int):
        failures.append("native summary missing integer madoca_materialization_rows")
        summary_rows = 0
    if csv_rows != summary_rows:
        failures.append(f"CSV rows {csv_rows} != summary rows {summary_rows}")
    if summary_rows < config.min_rows:
        failures.append(f"materialization rows {summary_rows} < minimum {config.min_rows}")
    if materialization_summary.get("schema") != "madoca_materialization_summary.v1":
        failures.append("materialization summary schema mismatch")
    if materialization_summary.get("status") != "passed":
        failures.append(f"materialization summary status {materialization_summary.get('status')} != passed")
    if materialization_summary.get("rows") != summary_rows:
        failures.append(
            f"materialization summary rows {materialization_summary.get('rows')} != summary rows {summary_rows}"
        )
    systems = materialization_summary.get("systems")
    if not isinstance(systems, dict):
        systems = {}
        failures.append("materialization summary systems is not an object")
    for system in config.required_systems:
        if not positive_count(systems, system):
            failures.append(f"materialization summary missing required system {system}")
    row_key = materialization_summary.get("row_key")
    if not isinstance(row_key, dict):
        row_key = {}
        failures.append("materialization summary row_key is not an object")
    duplicate_groups = row_key.get("duplicate_groups")
    if duplicate_groups != 0:
        failures.append(f"materialization summary duplicate groups {duplicate_groups} != 0")
    bias_identity = materialization_summary.get("bias_identity")
    if not isinstance(bias_identity, dict):
        bias_identity = {}
        failures.append("materialization summary bias_identity is not an object")
    code_bias_ids = bias_identity.get("code_bias_ids")
    if not isinstance(code_bias_ids, dict):
        code_bias_ids = {}
        failures.append("materialization summary code_bias_ids is not an object")
    phase_bias_ids = bias_identity.get("phase_bias_ids")
    if not isinstance(phase_bias_ids, dict):
        phase_bias_ids = {}
        failures.append("materialization summary phase_bias_ids is not an object")
    for signal_id in config.required_code_bias_ids:
        if not positive_count(code_bias_ids, signal_id):
            failures.append(f"materialization summary missing required code bias id {signal_id}")
    for signal_id in config.required_phase_bias_ids:
        if not positive_count(phase_bias_ids, signal_id):
            failures.append(f"materialization summary missing required phase bias id {signal_id}")
    if selfdiff_report.get("schema") != "madoca_materialization_diff.v1":
        failures.append("self-diff schema mismatch")
    for key in ("base_only_rows", "candidate_only_rows", "discrete_mismatches", "numeric_threshold_exceedances"):
        value = selfdiff_report.get(key)
        if value != 0:
            failures.append(f"self-diff {key} {value} != 0")
    if selfdiff_report.get("row_set_complete") is not True:
        failures.append("self-diff row_set_complete is not true")
    max_delta = 0.0
    for item in selfdiff_report.get("per_numeric_component", []):
        if isinstance(item, dict) and isinstance(item.get("max_abs_delta"), (float, int)):
            max_delta = max(max_delta, float(item["max_abs_delta"]))
    if max_delta != 0.0:
        failures.append(f"self-diff max numeric delta {max_delta} != 0")

    metrics = {
        "materialization_rows": summary_rows,
        "csv_rows": csv_rows,
        "materialization_systems": materialization_summary.get("systems"),
        "materialization_row_key": materialization_summary.get("row_key"),
        "materialization_bias_identity": materialization_summary.get("bias_identity"),
        "selfdiff_common_rows": selfdiff_report.get("common_rows"),
        "selfdiff_numeric_components_compared": selfdiff_report.get("numeric_components_compared"),
        "selfdiff_max_abs_delta": max_delta,
    }
    return Evaluation(status="failed" if failures else "passed", failures=failures, metrics=metrics)


def artifact_records(paths: RunPaths, *, blocked: bool = False) -> list[dict[str, object]]:
    return [
        artifact_record(paths.log_path, role="log", required=True),
        artifact_record(paths.native_dump, role="native_materialization_csv", required=not blocked),
        artifact_record(paths.native_summary_json, role="native_summary_json", required=not blocked),
        artifact_record(
            paths.native_materialization_summary_json,
            role="native_materialization_summary_json",
            required=not blocked,
        ),
        artifact_record(paths.selfdiff_json, role="selfdiff_json", required=not blocked),
        artifact_record(paths.selfdiff_csv, role="selfdiff_csv", required=not blocked),
    ]


def write_summary(
    paths: RunPaths,
    *,
    config: RunConfig,
    status: str,
    block_reason: str | None = None,
    failures: list[str] | None = None,
    metrics: dict[str, object] | None = None,
) -> None:
    payload = {
        "summary_schema": SUMMARY_SCHEMA,
        "contract": CONTRACT,
        "status": status,
        "configuration": {
            "madocalib_repo": config.madocalib_repo,
            "madocalib_ref": config.madocalib_ref,
            "min_rows": config.min_rows,
            "required_systems": list(config.required_systems),
            "required_code_bias_ids": list(config.required_code_bias_ids),
            "required_phase_bias_ids": list(config.required_phase_bias_ids),
            "auto_fetch": config.auto_fetch,
            "data_root": str(config.data_root) if config.data_root is not None else None,
        },
        "input_provenance": {
            "required_data_files": list(REQUIRED_DATA_FILES),
        },
        "metrics": metrics or {},
        "failures": failures or [],
        "block_reason": block_reason,
        "artifacts": artifact_records(paths, blocked=status == "blocked_infrastructure"),
        "next_actions": [],
    }
    if status == "blocked_infrastructure":
        payload["next_actions"] = [
            "Provide GNSSPP_MADOCA_MATERIALIZATION_DATA_ROOT or allow public MADOCALIB auto-fetch.",
            "Treat blocked_infrastructure as missing native MADOCA materialization evidence.",
        ]
    elif status == "failed":
        payload["next_actions"] = [
            "Inspect the native materialization log, summary JSON, and self-diff artifacts before changing solver behavior."
        ]
    paths.summary_json.parent.mkdir(parents=True, exist_ok=True)
    paths.summary_json.write_text(json.dumps(payload, indent=2) + "\n", encoding="utf-8")


def parse_args(argv: Sequence[str] | None = None) -> argparse.Namespace:
    parser = argparse.ArgumentParser()
    parser.add_argument("--repo-root", type=Path, default=repo_root_from_script())
    parser.add_argument("--output-dir", type=Path, default=None)
    return parser.parse_args(argv)


def main(argv: Sequence[str] | None = None) -> int:
    args = parse_args(argv)
    config = parse_config(args, os.environ)
    paths = default_paths(config.output_dir)
    paths.log_path.parent.mkdir(parents=True, exist_ok=True)
    paths.work_dir.mkdir(parents=True, exist_ok=True)
    if paths.log_path.exists():
        paths.log_path.unlink()
    paths.tiny_obs.write_text(TINY_OBS_TEXT, encoding="ascii")

    data_root, block_reason = checkout_madocalib_data(config, paths)
    if data_root is None:
        write_summary(paths, config=config, status="blocked_infrastructure", block_reason=block_reason)
        print(f"MADOCA materialization self-diff blocked: {block_reason}")
        return 1 if config.fail_on_blocked else 0

    dump_result = run_logged(build_dump_command(config, paths, data_root), cwd=config.repo_root, log_path=paths.log_path)
    if dump_result.returncode != 0:
        write_summary(
            paths,
            config=config,
            status="failed",
            failures=["native MADOCA materialization dump failed"],
        )
        print(f"MADOCA materialization dump failed; see {paths.log_path}")
        return 1

    materialization_summary_result = run_logged(
        build_materialization_summary_command(config, paths),
        cwd=config.repo_root,
        log_path=paths.log_path,
    )
    if materialization_summary_result.returncode != 0:
        write_summary(
            paths,
            config=config,
            status="failed",
            failures=["native MADOCA materialization summary validation failed"],
        )
        print(f"MADOCA materialization summary validation failed; see {paths.log_path}")
        return 1

    selfdiff_result = run_logged(build_selfdiff_command(config, paths), cwd=config.repo_root, log_path=paths.log_path)
    if selfdiff_result.returncode != 0:
        write_summary(
            paths,
            config=config,
            status="failed",
            failures=["native MADOCA materialization self-diff failed"],
        )
        print(f"MADOCA materialization self-diff failed; see {paths.log_path}")
        return 1

    evaluation = evaluate(
        config=config,
        paths=paths,
        dump_summary=load_json(paths.native_summary_json),
        materialization_summary=load_json(paths.native_materialization_summary_json),
        selfdiff_report=load_json(paths.selfdiff_json),
    )
    write_summary(
        paths,
        config=config,
        status=evaluation.status,
        failures=evaluation.failures,
        metrics=evaluation.metrics,
    )
    print("MADOCA materialization self-diff:", evaluation.status)
    print(f"  rows: {evaluation.metrics.get('materialization_rows')}")
    print(f"  dump: {paths.native_dump}")
    print(f"  selfdiff: {paths.selfdiff_json}")
    return 1 if evaluation.status != "passed" else 0


if __name__ == "__main__":
    raise SystemExit(main())
