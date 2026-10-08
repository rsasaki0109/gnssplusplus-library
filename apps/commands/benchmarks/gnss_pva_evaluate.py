#!/usr/bin/env python3
"""Replay raw PPC with received-event semantics, then score truth offline."""
from __future__ import annotations

import argparse
import hashlib
import json
import os
from pathlib import Path
import shutil
import subprocess
import sys
import time

import gnss_pva_metrics as metrics


def pin(path):
    path = Path(path).resolve()
    digest = hashlib.sha256()
    with path.open("rb") as stream:
        for block in iter(lambda: stream.read(1024*1024), b""): digest.update(block)
    return dict(path=str(path), bytes=path.stat().st_size, sha256=digest.hexdigest())


def dump(path, value):
    Path(path).write_text(json.dumps(value, ensure_ascii=False, indent=2, allow_nan=False)+"\n", encoding="utf-8")


def source_identity():
    parents = Path(__file__).resolve().parents
    root = parents[3] if len(parents) > 3 else None
    if root is None or not (root/".git").exists(): return None
    head = subprocess.check_output(["git", "rev-parse", "HEAD"], cwd=root, text=True).strip()
    diff = subprocess.check_output(["git", "diff", "--binary", "HEAD"], cwd=root, stderr=subprocess.DEVNULL)
    untracked = subprocess.check_output(["git", "ls-files", "--others", "--exclude-standard"], cwd=root, text=True).splitlines()
    return dict(commit=head, tracked_diff_sha256=hashlib.sha256(diff).hexdigest(),
                untracked={name: pin(root/name) for name in untracked})


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__, prog=os.environ.get("GNSS_CLI_NAME"))
    source = parser.add_mutually_exclusive_group(required=True)
    source.add_argument("--run-dir", type=Path, help="External PPC run with rover.obs/base.obs/base.nav/imu.csv/reference.csv")
    source.add_argument("--estimate", type=Path, help="Existing online CSV; score only, no solver execution")
    parser.add_argument("--reference", type=Path, help="Truth CSV for score-only mode")
    parser.add_argument("--replay-binary", type=Path)
    parser.add_argument("--output-dir", type=Path, required=True, help="Must not exist")
    parser.add_argument("--max-epochs", type=int, default=0, help="0=full input; nonzero reports a bounded development prefix")
    parser.add_argument("--scenario", choices=("normal", "gnss_outage", "imu_gap", "loose_only"), default="normal")
    parser.add_argument("--start-s", type=float, default=60)
    parser.add_argument("--duration-s", type=float, default=10)
    parser.add_argument("--plot", action="store_true", help="Needs matplotlib; metrics need only Python standard library")
    parser.add_argument("--candidate", choices=("none", "vehicle_nhc_latched_v1", "velocity_consistency_v1", "velocity_consistency_v2"), default="none", help="Opt-in frozen development experiment; does not alter default inference")
    args = parser.parse_args(argv)
    if args.output_dir.exists(): parser.error("output directory already exists")
    if args.max_epochs < 0: parser.error("max epochs must be nonnegative")
    if args.run_dir and args.reference: parser.error("run-dir uses its reference.csv only after replay")
    if args.estimate and not args.reference: parser.error("estimate requires reference")
    args.output_dir.mkdir(parents=True)
    manifest = dict(schema="libgnsspp.pva_evaluation.v1", state="running", reference_used_for_estimation=False,
                    dataset_role="development/regression; no heldout claim", scoring_source=pin(metrics.__file__),
                    workflow_source=pin(__file__), inputs={}, argv=None)
    try:
        manifest["repository"] = source_identity()
        if args.run_dir:
            manifest["inputs"] = {n: pin(args.run_dir/n) for n in ("rover.obs", "base.obs", "base.nav", "imu.csv", "reference.csv")}
            binary = args.replay_binary or Path(shutil.which("gnss_pva_replay") or "")
            if not binary.is_file(): raise ValueError("provide --replay-binary or install gnss_pva_replay on PATH")
            manifest["binary"] = pin(binary)
            command = [str(binary.resolve()), str(args.run_dir.resolve()), str((args.output_dir/"replay").resolve()),
                       str(args.max_epochs), args.scenario, str(args.start_s), str(args.duration_s)]
            if args.candidate != "none": command += ["--candidate", args.candidate]
            manifest["argv"] = command
            started = time.monotonic()
            result = subprocess.run(command, capture_output=True, text=True)
            manifest["wall_s"] = time.monotonic()-started
            (args.output_dir/"replay.log").write_text(result.stdout+result.stderr, encoding="utf-8")
            if result.returncode: raise ValueError(f"replay failed with exit {result.returncode}; see replay.log")
            estimate, reference = args.output_dir/"replay/pva.csv", args.run_dir/"reference.csv"
            replay = json.loads((args.output_dir/"replay/replay.json").read_text(encoding="utf-8"))
            manifest["replay"] = replay
            if manifest["binary"] != pin(binary) or any(p != pin(args.run_dir/n) for n, p in manifest["inputs"].items()):
                raise ValueError("input/binary changed during replay")
        else:
            estimate, reference = args.estimate, args.reference
            manifest["inputs"] = dict(estimate=pin(estimate), reference=pin(reference))
        report, rows = metrics.score(estimate, reference)
        if args.run_dir:
            replay = manifest["replay"]
            if args.scenario in ("gnss_outage", "imu_gap"):
                report["scenario"] = metrics.scenario_summary(rows, args.scenario, replay["scenario_start_s"], replay["scenario_duration_s"])
        metrics.write_errors(args.output_dir/"errors.csv", rows)
        if args.plot: metrics.plot_errors(args.output_dir/"errors.png", rows)
        dump(args.output_dir/"score.json", report)
        manifest["estimate"] = pin(estimate)
        manifest["outputs"] = {p.name: pin(p) for p in args.output_dir.iterdir() if p.is_file()}
        manifest["state"] = "passed"
        dump(args.output_dir/"manifest.json", manifest)
        print(json.dumps(dict(output=str(args.output_dir), epochs=report["epochs"], matched=report["match_fraction"], coverage=report["coverage"])))
        return 0
    except (ValueError, OSError, KeyError, ImportError) as error:
        manifest.update(state="failed", error=str(error))
        dump(args.output_dir/"manifest.json", manifest)
        print(f"pva-evaluate: {error}", file=sys.stderr)
        return 2


if __name__ == "__main__":
    raise SystemExit(main())
