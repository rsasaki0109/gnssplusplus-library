#!/usr/bin/env python3
"""Run exactly one authorized Phase108 truth-only evaluator subprocess.

Phase107 native outputs are immutable and already exist.  This launcher never
invokes the native solver and never opens raw/base/truth/candidate files.  It
checks the pre-truth contract and the independent authorization, then starts
one evaluator process whose only data inputs are the two sealed candidate
CSVs and the two pinned official truth files.
"""

from __future__ import annotations

import argparse
import importlib.util
import json
import os
from pathlib import Path
import subprocess
import sys
import tempfile
from typing import Any


ROOT = Path(__file__).resolve().parents[3]
EVALUATOR_PATH = ROOT / "apps/commands/benchmarks/gnss_smartphone_phase108_phase107_solution_accuracy.py"
OUTPUT_ROOT = ROOT / "output/smartphone-r5/phase108-phase107-solution-accuracy-v1"
RESULT_JSON = ROOT / "docs/use_cases/records/smartphone_r5_phase108_phase107_solution_accuracy_result_v1.json"


class Phase108ExecutionError(ValueError):
    """Raised when the one-shot truth-only boundary cannot be established."""


def fail(message: str) -> Phase108ExecutionError:
    return Phase108ExecutionError(message)


def atomic_json(path: Path, value: dict[str, Any]) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    descriptor, temporary = tempfile.mkstemp(prefix=f".{path.name}.", suffix=".tmp", dir=path.parent)
    try:
        with os.fdopen(descriptor, "w", encoding="utf-8") as handle:
            json.dump(value, handle, indent=2, sort_keys=True, allow_nan=False)
            handle.write("\n")
            handle.flush()
            os.fsync(handle.fileno())
        os.replace(temporary, path)
    except BaseException:
        try:
            os.unlink(temporary)
        except OSError:
            pass
        raise


def load_evaluator() -> Any:
    spec = importlib.util.spec_from_file_location("phase108_truth_only_contract", EVALUATOR_PATH)
    if spec is None or spec.loader is None:
        raise fail(f"unable to load Phase108 evaluator: {EVALUATOR_PATH}")
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


def safe_environment() -> dict[str, str]:
    # No inherited truth, MAT, raw, base, coordinate, token, or Kaggle path is
    # forwarded to the evaluator subprocess.
    return {
        "PATH": os.environ.get("PATH", "/usr/bin:/bin"),
        "LANG": "C",
        "LC_ALL": "C",
        "TZ": "UTC",
    }


def run_once() -> int:
    evaluator = load_evaluator()
    try:
        pre = evaluator.verify_pre_truth()
        auth = evaluator.verify_authorization()
    except Exception as exc:
        raise fail(f"Phase108 pre-truth/authorization verification failed: {exc}") from exc
    for key, expected in (("native_solver_invocations", 0), ("raw_reads", 0), ("base_reads", 0), ("truth_reads", 0), ("accuracy_calculations", 0), ("kaggle_or_token_access", 0)):
        if pre.get(key) != expected:
            raise fail(f"pre-truth accounting changed: {key}={pre.get(key)!r}")
    if auth.get("status") != "authorized-for-phase108-truth-only-accuracy-evaluation":
        raise fail("authorization status is not the exact Phase108 truth-only status")
    if RESULT_JSON.exists():
        raise fail(f"refusing a second Phase108 evaluator run: {RESULT_JSON}")
    if OUTPUT_ROOT.exists() and any(OUTPUT_ROOT.iterdir()):
        raise fail(f"refusing to overwrite nonempty Phase108 log directory: {OUTPUT_ROOT}")
    OUTPUT_ROOT.mkdir(parents=True, exist_ok=True)
    stdout_path = OUTPUT_ROOT / "evaluator.stdout.log"
    stderr_path = OUTPUT_ROOT / "evaluator.stderr.log"
    metadata_path = OUTPUT_ROOT / "execution_metadata.json"
    command = [sys.executable, str(EVALUATOR_PATH), "--evaluate", "--result-json", str(RESULT_JSON)]
    if any("truth" in token.lower() or "ground_truth" in token.lower() for token in command):
        raise fail("truth path/token leaked into evaluator command")
    with stdout_path.open("wb") as stdout, stderr_path.open("wb") as stderr:
        completed = subprocess.run(command, cwd=ROOT, env=safe_environment(), stdout=stdout, stderr=stderr, check=False)
    atomic_json(metadata_path, {
        "phase": 108,
        "status": "evaluator-completed",
        "evaluator_process_count": 1,
        "return_code": completed.returncode,
        "command": [sys.executable, str(EVALUATOR_PATH), "--evaluate", "--result-json", "relative-result-path"],
        "truth_path_in_command": False,
        "truth_read_by_launcher": False,
        "raw_or_base_read_by_launcher": False,
        "native_solver_invocations": 0,
        "reruns": 0,
        "fallbacks": 0,
        "solution_output_published": False,
        "logs": {"stdout": str(stdout_path.relative_to(ROOT)), "stderr": str(stderr_path.relative_to(ROOT))},
    })
    return completed.returncode


def main(argv: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--run-once", action="store_true")
    args = parser.parse_args(argv)
    if not args.run_once:
        parser.error("--run-once is required")
    try:
        return run_once()
    except Exception as exc:
        print(f"phase108 launcher: fail-closed: {exc}", file=sys.stderr)
        return 2


if __name__ == "__main__":
    raise SystemExit(main())
