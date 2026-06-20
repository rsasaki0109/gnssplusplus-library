#!/usr/bin/env python3
"""Generate the public CLAS A4b native ZD dump and self-diff it in CI."""

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


SUMMARY_SCHEMA = "ci_clas_a4b_native_selfdiff.v3"
CONTRACT = "clas_a4b_native_selfdiff.v1"
DEFAULT_CLASLIB_REPO = "https://github.com/QZSS-Strategy-Office/claslib.git"
DEFAULT_CLASLIB_REF = "23cfd363a2db6d8d8144e292c82e9d97ca2d3015"
DEFAULT_MAX_EPOCHS = 300
DEFAULT_RECEIVER_ANTENNA_TYPE = "TRM59800.80     NONE"
REQUIRED_DATA_FILES = (
    "0627239Q.obs",
    "sept_2019239.nav",
    "2019239Q.l6",
    "igs14_L5copy.atx",
)
ENV_OVERRIDES = {
    "GNSS_PPP_CLAS_DD_FILTER": "1",
    "GNSS_PPP_CLAS_CODE_ROW_PARITY": "bias,full-prc",
    "GNSS_PPP_CLAS_RX_ANTENNA": "1",
    "GNSS_PPP_CLAS_ATMOS_GRID_MATRIX": "1",
    "GNSS_PPP_CLAS_ATMOS_LIFECYCLE": "1",
}


@dataclass(frozen=True)
class RunPaths:
    output_dir: Path
    work_dir: Path
    log_path: Path
    summary_json: Path
    checkout_dir: Path
    native_pos: Path
    native_summary_json: Path
    native_code_dump: Path
    native_code_dump_summary_json: Path
    selfdiff_json: Path
    selfdiff_csv: Path


@dataclass(frozen=True)
class RunConfig:
    repo_root: Path
    output_dir: Path
    data_root: Path | None
    auto_fetch: bool
    fail_on_blocked: bool
    claslib_repo: str
    claslib_ref: str
    max_epochs: int
    gps_l2w_rows_min: int
    receiver_antenna_type: str
    python_executable: str


def repo_root_from_script() -> Path:
    return Path(__file__).resolve().parents[2]


def default_paths(output_dir: Path) -> RunPaths:
    work_dir = output_dir / "clas_a4b_native_selfdiff"
    return RunPaths(
        output_dir=output_dir,
        work_dir=work_dir,
        log_path=output_dir / "ci_clas_a4b_native_selfdiff" / "clas_a4b_native_selfdiff.log",
        summary_json=output_dir / "ci_clas_a4b_native_selfdiff_summary.json",
        checkout_dir=work_dir / "claslib",
        native_pos=work_dir / "native.pos",
        native_summary_json=work_dir / "native_summary.json",
        native_code_dump=work_dir / "native_code_dump.csv",
        native_code_dump_summary_json=work_dir / "native_code_dump_summary.json",
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


def parse_config(args: argparse.Namespace, environ: Mapping[str, str]) -> RunConfig:
    repo_root = args.repo_root.resolve()
    output_dir = (args.output_dir if args.output_dir is not None else repo_root / "output").resolve()
    max_epochs = int(env_or(environ, "GNSSPP_CLAS_A4B_MAX_EPOCHS", str(DEFAULT_MAX_EPOCHS)))
    return RunConfig(
        repo_root=repo_root,
        output_dir=output_dir,
        data_root=resolve_optional_path(repo_root, environ.get("GNSSPP_CLAS_A4B_DATA_ROOT", "")),
        auto_fetch=truthy(env_or(environ, "GNSSPP_CLAS_A4B_AUTO_FETCH", "1")),
        fail_on_blocked=truthy(env_or(environ, "GNSSPP_CLAS_A4B_FAIL_ON_BLOCKED", "1")),
        claslib_repo=env_or(environ, "GNSSPP_CLAS_A4B_CLASLIB_REPO", DEFAULT_CLASLIB_REPO),
        claslib_ref=env_or(environ, "GNSSPP_CLAS_A4B_CLASLIB_REF", DEFAULT_CLASLIB_REF),
        max_epochs=max_epochs,
        gps_l2w_rows_min=int(env_or(environ, "GNSSPP_CLAS_A4B_GPS_L2W_ROWS_MIN", str(max_epochs))),
        receiver_antenna_type=environ.get(
            "GNSSPP_CLAS_A4B_RECEIVER_ANTENNA_TYPE",
            DEFAULT_RECEIVER_ANTENNA_TYPE,
        ).strip()
        or DEFAULT_RECEIVER_ANTENNA_TYPE,
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


def checkout_claslib_data(config: RunConfig, paths: RunPaths) -> tuple[Path | None, str | None]:
    if config.data_root is not None:
        missing = data_root_failures(config.data_root)
        if not missing:
            return config.data_root, None
        return None, f"configured CLAS data root is missing required files: {', '.join(missing)}"

    if not config.auto_fetch:
        return None, "GNSSPP_CLAS_A4B_DATA_ROOT is unset and auto-fetch is disabled"

    if paths.checkout_dir.exists():
        shutil.rmtree(paths.checkout_dir)
    paths.checkout_dir.parent.mkdir(parents=True, exist_ok=True)
    commands = [
        ["git", "init", str(paths.checkout_dir)],
        ["git", "-C", str(paths.checkout_dir), "remote", "add", "origin", config.claslib_repo],
        ["git", "-C", str(paths.checkout_dir), "sparse-checkout", "init", "--cone"],
        ["git", "-C", str(paths.checkout_dir), "sparse-checkout", "set", "data"],
        ["git", "-C", str(paths.checkout_dir), "fetch", "--depth", "1", "origin", config.claslib_ref],
        ["git", "-C", str(paths.checkout_dir), "checkout", "--detach", "FETCH_HEAD"],
    ]
    for command in commands:
        completed = run_logged(command, cwd=config.repo_root, log_path=paths.log_path)
        if completed.returncode != 0:
            return None, f"failed to fetch CLASLIB public data: {' '.join(command)}"

    data_root = paths.checkout_dir / "data"
    missing = data_root_failures(data_root)
    if missing:
        return None, f"fetched CLASLIB data is missing required files: {', '.join(missing)}"
    return data_root, None


def build_native_command(
    config: RunConfig,
    paths: RunPaths,
    data_root: Path,
) -> tuple[list[str], dict[str, str]]:
    command = [
        config.python_executable,
        str(config.repo_root / "apps" / "gnss.py"),
        "clas-ppp",
        "--profile",
        "clas",
        "--obs",
        str(data_root / "0627239Q.obs"),
        "--nav",
        str(data_root / "sept_2019239.nav"),
        "--qzss-l6",
        str(data_root / "2019239Q.l6"),
        "--qzss-gps-week",
        "2068",
        "--antex",
        str(data_root / "igs14_L5copy.atx"),
        "--receiver-antenna-type",
        config.receiver_antenna_type,
        "--compact-code-bias-composition-policy",
        "base-only-if-present",
        "--compact-code-bias-bank-policy",
        "latest-preceding-bank",
        "--compact-bias-row-materialization",
        "selected-satellite-base-extend",
        "--out",
        str(paths.native_pos),
        "--summary-json",
        str(paths.native_summary_json),
        "--max-epochs",
        str(config.max_epochs),
    ]
    env = dict(os.environ)
    env.update(ENV_OVERRIDES)
    env["GNSS_PPP_CLAS_CODE_DUMP"] = str(paths.native_code_dump)
    return command, env


def build_selfdiff_command(config: RunConfig, paths: RunPaths) -> list[str]:
    return [
        config.python_executable,
        str(config.repo_root / "scripts" / "analysis" / "clas_zd_component_diff.py"),
        str(paths.native_code_dump),
        str(paths.native_code_dump),
        "--base-label",
        "native",
        "--candidate-label",
        "native",
        "--row-type",
        "code",
        "--sat",
        "G14",
        "--freq",
        "1",
        "--rinex-code",
        "C2W",
        "--duplicate-policy",
        "mean",
        "--json-out",
        str(paths.selfdiff_json),
        "--details-csv",
        str(paths.selfdiff_csv),
        "--fail-on-diff",
    ]


def build_code_dump_summary_command(config: RunConfig, paths: RunPaths) -> list[str]:
    return [
        config.python_executable,
        str(config.repo_root / "scripts" / "analysis" / "clas_zd_component_summary.py"),
        str(paths.native_code_dump),
        "--json-out",
        str(paths.native_code_dump_summary_json),
        "--fail-on-issue",
        "--require-system",
        "GPS",
        "--require-row-type",
        "code",
        "--require-rinex-code",
        "C2W",
        "--require-component",
        "prc_m",
        "--require-gps-l2w-rows-min",
        str(config.gps_l2w_rows_min),
        "--require-gps-l2w-exact-bias",
        "--require-gps-l2w-observation-exact-match",
        "--require-gps-l2w-no-observation-family-fallback",
        "--require-gps-l2w-no-code-bias-fallback",
        "--require-gps-l2w-no-phase-bias-fallback",
        "--require-no-duplicate-keys",
    ]


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


def max_component_delta(report: Mapping[str, object]) -> float:
    per_component = report.get("per_component")
    if not isinstance(per_component, list):
        return 0.0
    maximum = 0.0
    for item in per_component:
        if isinstance(item, dict):
            value = item.get("max_abs_delta_m")
            if isinstance(value, (int, float)):
                maximum = max(maximum, float(value))
    return maximum


def native_identity(report: Mapping[str, object]) -> dict[str, object]:
    identity = report.get("identity_provenance")
    if not isinstance(identity, dict):
        return {}
    summary = identity.get("native")
    return summary if isinstance(summary, dict) else {}


def evaluate(
    config: RunConfig,
    native_summary: Mapping[str, object],
    code_dump_summary: Mapping[str, object],
    report: Mapping[str, object],
    dump_rows: int,
) -> tuple[dict[str, object], dict[str, object], list[str]]:
    identity = native_identity(report)
    gps_l2w_rows = int(identity.get("gps_l2w_rows", 0))
    exact_bias_rows = int(identity.get("gps_l2w_bias_exact_identity_rows", 0))
    exact_match_rows = int(identity.get("gps_l2w_observation_exact_match_rows", 0))
    obs_fallback_rows = int(identity.get("gps_l2w_observation_family_fallback_rows", 0))
    code_fallback_rows = int(identity.get("gps_l2w_code_bias_fallback_rows", 0))
    max_delta = max_component_delta(report)
    metrics = {
        "native_epochs": int(native_summary.get("epochs", 0)),
        "native_ppp_solution_rate_pct": native_summary.get("ppp_solution_rate_pct"),
        "native_code_dump_rows": dump_rows,
        "native_code_dump_summary_schema": code_dump_summary.get("schema"),
        "native_code_dump_summary_status": code_dump_summary.get("status"),
        "native_code_dump_summary_rows": code_dump_summary.get("rows"),
        "native_code_dump_summary_duplicate_groups": (
            code_dump_summary.get("row_key", {}).get("duplicate_groups")
            if isinstance(code_dump_summary.get("row_key"), dict)
            else None
        ),
        "common_rows": int(report.get("common_rows", 0)),
        "base_only_rows": int(report.get("base_only_rows", 0)),
        "candidate_only_rows": int(report.get("candidate_only_rows", 0)),
        "components_compared": int(report.get("components_compared", 0)),
        "threshold_exceedances": int(report.get("threshold_exceedances", 0)),
        "max_abs_delta_m": max_delta,
        "gps_l2w_rows": gps_l2w_rows,
        "gps_l2w_bias_exact_identity_rows": exact_bias_rows,
        "gps_l2w_observation_exact_match_rows": exact_match_rows,
        "gps_l2w_observation_family_fallback_rows": obs_fallback_rows,
        "gps_l2w_code_bias_fallback_rows": code_fallback_rows,
    }
    thresholds = {
        "native_epochs_min": config.max_epochs,
        "gps_l2w_rows_min": config.gps_l2w_rows_min,
        "base_only_rows_max": 0,
        "candidate_only_rows_max": 0,
        "fallback_rows_max": 0,
        "max_abs_delta_m_max": 0.0,
    }
    failures: list[str] = []
    if metrics["native_epochs"] < config.max_epochs:
        failures.append(f"native epochs {metrics['native_epochs']} < {config.max_epochs}")
    if metrics["native_code_dump_summary_schema"] != "clas_zd_component_summary.v2":
        failures.append(
            f"native code dump summary schema {metrics['native_code_dump_summary_schema']} "
            "!= clas_zd_component_summary.v2"
        )
    if metrics["native_code_dump_summary_status"] != "passed":
        failures.append(
            f"native code dump summary status {metrics['native_code_dump_summary_status']} != passed"
        )
    if metrics["native_code_dump_summary_rows"] != dump_rows:
        failures.append(
            f"native code dump summary rows {metrics['native_code_dump_summary_rows']} != dump rows {dump_rows}"
        )
    if gps_l2w_rows < config.gps_l2w_rows_min:
        failures.append(f"GPS L2W rows {gps_l2w_rows} < {config.gps_l2w_rows_min}")
    if exact_bias_rows != gps_l2w_rows:
        failures.append(f"exact bias rows {exact_bias_rows} != GPS L2W rows {gps_l2w_rows}")
    if exact_match_rows != gps_l2w_rows:
        failures.append(f"exact observation-match rows {exact_match_rows} != GPS L2W rows {gps_l2w_rows}")
    if obs_fallback_rows != 0:
        failures.append(f"observation-family fallback rows {obs_fallback_rows} != 0")
    if code_fallback_rows != 0:
        failures.append(f"code-bias fallback rows {code_fallback_rows} != 0")
    if metrics["base_only_rows"] != 0 or metrics["candidate_only_rows"] != 0:
        failures.append(
            f"self-diff unmatched rows {metrics['base_only_rows']}/{metrics['candidate_only_rows']} != 0/0"
        )
    if metrics["threshold_exceedances"] != 0:
        failures.append(f"threshold exceedances {metrics['threshold_exceedances']} != 0")
    if max_delta != 0.0:
        failures.append(f"self-diff max component delta {max_delta} != 0")
    return metrics, thresholds, failures


def summarize_artifacts(paths: RunPaths) -> list[dict[str, object]]:
    return [
        artifact_record(paths.log_path, role="log", required=True),
        artifact_record(paths.native_pos, role="native_solution", required=True),
        artifact_record(paths.native_summary_json, role="native_summary", required=True),
        artifact_record(paths.native_code_dump, role="native_zd_component_dump", required=True),
        artifact_record(paths.native_code_dump_summary_json, role="native_zd_component_summary", required=True),
        artifact_record(paths.selfdiff_json, role="selfdiff_json", required=True),
        artifact_record(paths.selfdiff_csv, role="selfdiff_csv", required=True),
    ]


def write_summary(paths: RunPaths, payload: dict[str, object]) -> None:
    paths.summary_json.parent.mkdir(parents=True, exist_ok=True)
    paths.summary_json.write_text(json.dumps(payload, indent=2, sort_keys=True) + "\n", encoding="utf-8")


def build_payload(
    *,
    config: RunConfig,
    paths: RunPaths,
    status: str,
    data_root: Path | None,
    metrics: dict[str, object] | None = None,
    thresholds: dict[str, object] | None = None,
    failures: list[str] | None = None,
    block_reason: str | None = None,
) -> dict[str, object]:
    next_actions: list[str] = []
    if status == "blocked_infrastructure":
        next_actions.append("Provide CLAS public data with GNSSPP_CLAS_A4B_DATA_ROOT or allow sparse auto-fetch.")
        next_actions.append("Treat blocked_infrastructure as missing native A4b evidence, not as a passing sign-off.")
        if config.fail_on_blocked:
            next_actions.append("Set GNSSPP_CLAS_A4B_FAIL_ON_BLOCKED=0 only for diagnostic-only local runs.")
    elif status == "failed":
        next_actions.append("Inspect the native run log, self-diff JSON, and component dump before changing CLAS model behavior.")
    return {
        "summary_schema": SUMMARY_SCHEMA,
        "contract": CONTRACT,
        "status": status,
        "block_reason": block_reason,
        "failures": failures or [],
        "next_actions": next_actions,
        "input_provenance": {
            "claslib_repo": config.claslib_repo,
            "claslib_ref": config.claslib_ref,
            "data_root": str(data_root) if data_root is not None else None,
            "required_files": list(REQUIRED_DATA_FILES),
        },
        "configuration": {
            "max_epochs": config.max_epochs,
            "fail_on_blocked": config.fail_on_blocked,
            "receiver_antenna_type": config.receiver_antenna_type,
            "env_overrides": {**ENV_OVERRIDES, "GNSS_PPP_CLAS_CODE_DUMP": str(paths.native_code_dump)},
            "selfdiff_filter": {"sat": "G14", "freq": 1, "rinex_code": "C2W", "row_type": "code"},
        },
        "metrics": metrics or {},
        "thresholds": thresholds or {},
        "artifacts": summarize_artifacts(paths),
    }


def parse_args(argv: Sequence[str] | None = None) -> argparse.Namespace:
    parser = argparse.ArgumentParser()
    parser.add_argument("--repo-root", type=Path, default=repo_root_from_script())
    parser.add_argument("--output-dir", type=Path, default=None)
    return parser.parse_args(argv)


def main(argv: Sequence[str] | None = None, environ: Mapping[str, str] | None = None) -> int:
    args = parse_args(argv)
    env = os.environ if environ is None else environ
    config = parse_config(args, env)
    paths = default_paths(config.output_dir)
    paths.work_dir.mkdir(parents=True, exist_ok=True)
    paths.log_path.parent.mkdir(parents=True, exist_ok=True)
    paths.log_path.write_text("", encoding="utf-8")

    data_root, block_reason = checkout_claslib_data(config, paths)
    if data_root is None:
        payload = build_payload(
            config=config,
            paths=paths,
            status="blocked_infrastructure",
            data_root=None,
            block_reason=block_reason,
        )
        write_summary(paths, payload)
        print(f"CLAS A4b native self-diff blocked: {block_reason}")
        print(f"  summary: {paths.summary_json}")
        return 1 if config.fail_on_blocked else 0

    native_command, native_env = build_native_command(config, paths, data_root)
    native_result = run_logged(native_command, cwd=config.repo_root, log_path=paths.log_path, env=native_env)
    if native_result.returncode != 0:
        payload = build_payload(
            config=config,
            paths=paths,
            status="failed",
            data_root=data_root,
            failures=["native CLAS A4b run failed"],
        )
        write_summary(paths, payload)
        print(f"CLAS A4b native run failed; see {paths.log_path}")
        return 1

    code_dump_summary_command = build_code_dump_summary_command(config, paths)
    code_dump_summary_result = run_logged(
        code_dump_summary_command,
        cwd=config.repo_root,
        log_path=paths.log_path,
    )
    if code_dump_summary_result.returncode != 0:
        payload = build_payload(
            config=config,
            paths=paths,
            status="failed",
            data_root=data_root,
            failures=["native CLAS A4b code dump summary failed"],
        )
        write_summary(paths, payload)
        print(f"CLAS A4b native code dump summary failed; see {paths.log_path}")
        return 1

    selfdiff_command = build_selfdiff_command(config, paths)
    selfdiff_result = run_logged(selfdiff_command, cwd=config.repo_root, log_path=paths.log_path)
    if selfdiff_result.returncode != 0:
        payload = build_payload(
            config=config,
            paths=paths,
            status="failed",
            data_root=data_root,
            failures=["native CLAS A4b self-diff failed"],
        )
        write_summary(paths, payload)
        print(f"CLAS A4b native self-diff failed; see {paths.log_path}")
        return 1

    native_summary = load_json(paths.native_summary_json)
    code_dump_summary = load_json(paths.native_code_dump_summary_json)
    selfdiff_report = load_json(paths.selfdiff_json)
    metrics, thresholds, failures = evaluate(
        config,
        native_summary,
        code_dump_summary,
        selfdiff_report,
        count_csv_data_rows(paths.native_code_dump),
    )
    status = "failed" if failures else "passed"
    payload = build_payload(
        config=config,
        paths=paths,
        status=status,
        data_root=data_root,
        metrics=metrics,
        thresholds=thresholds,
        failures=failures,
    )
    write_summary(paths, payload)
    print("CLAS A4b native self-diff:", status)
    print(f"  summary: {paths.summary_json}")
    print(f"  native dump: {paths.native_code_dump}")
    print(f"  selfdiff: {paths.selfdiff_json}")
    if failures:
        for failure in failures:
            print(f"  - {failure}")
        return 1
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
