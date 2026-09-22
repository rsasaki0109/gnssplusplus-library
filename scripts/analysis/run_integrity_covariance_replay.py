#!/usr/bin/env python3
"""Replay the frozen low-cost RTK gate using existing PPC FGO shadow exports.

Reference coordinates are consumed only after the native process exits.
This is development regression evidence, not independent-data validation.
"""
from __future__ import annotations

import argparse
from collections import Counter
import csv
import json
from pathlib import Path
import re
import subprocess
import time

import analyze_ppc_wrong_fix_residuals as audit
from analyze_fgo_external_dr_witness import ecef_delta_to_enu, ecef_lat_lon
import compare_fixed_lag_covariance as comparison
from evaluate_ppc_residual_integrity_policy import build_reference_index, nearest_reference
from run_fixed_lag_covariance_regression import save, sha256


def score(path: Path, reference: dict, expected: int) -> tuple[dict, dict]:
    rows, epochs = [], audit.load_solution(path)
    for epoch in epochs:
        truth = nearest_reference(reference, epoch.week, epoch.tow_s, 0.11)
        if truth is None:
            continue
        delta = tuple(a - b for a, b in zip(epoch.ecef, truth))
        enu = ecef_delta_to_enu(delta, *ecef_lat_lon(*truth))
        rows.append(dict(tow=str(epoch.tow_s), status="FIXED" if epoch.status == 4 else "FLOAT",
                         e_err_m=str(enu[0]), n_err_m=str(enu[1]), u_err_m=str(enu[2])))
    if not rows:
        raise ValueError(f"no truth-matched positions: {path}")
    summary = comparison.summarize(rows, 2.0, expected)
    summary["native_output_epochs"] = len(epochs)
    summary["native_none_epochs"] = sum(e.status == 0 for e in epochs)
    for key in list(summary):
        if "covariance" in key:
            del summary[key]  # POS has no exported covariance; do not imply zero coverage.
    labeled = {(e.week, e.tow_s): e for e in epochs}
    return summary, labeled


def execute(argv: list[str], prefix: Path, inputs: dict, resume: bool) -> dict:
    record = prefix.with_suffix(".run.json")
    fingerprint = dict(argv=argv, binary_sha256=sha256(Path(argv[0])), inputs=inputs)
    if record.exists():
        previous = json.loads(record.read_text())
        if (resume and previous.get("state") == "complete" and
                previous["fingerprint"] == fingerprint and all(
                    sha256(prefix.with_suffix(suffix)) == value
                    for suffix, value in previous["output_sha256"].items())):
            if "expected_epochs" in previous:
                # Upgrade the early report schema; native outputs remain
                # immutable and hash-verified. The old field counted outputs.
                log = prefix.with_suffix(".log").read_text(encoding="utf-8", errors="replace")
                previous["output_epochs"] = len(audit.load_solution(prefix.with_suffix(".pos")))
                previous["processed_rover_epochs"] = comparison.native_input_epochs(log, "rtk")
                del previous["expected_epochs"]
                previous["report_schema_correction"] = "input count is measured before post-filtering"
                save(record, previous)
            return previous
        raise RuntimeError(f"existing run is not a verified completed match: {record}")
    prefix.parent.mkdir(parents=True, exist_ok=True)
    started = time.perf_counter()
    with prefix.with_suffix(".log").open("wb") as log:
        process = subprocess.Popen(argv, stdout=log, stderr=subprocess.STDOUT)
        payload = dict(fingerprint=fingerprint, state="running", pid=process.pid)
        save(record, payload)
        code = process.wait()
    payload.update(returncode=code, wall_s=time.perf_counter() - started, state="failed")
    if code == 0 and sha256(Path(argv[0])) == fingerprint["binary_sha256"]:
        log = prefix.with_suffix(".log").read_text(encoding="utf-8", errors="replace")
        total = re.search(r"total solutions:\s*(\d+)", log)
        skipped = re.search(r"skipped rover epochs:\s*(\d+)", log)
        if total and skipped:
            payload.update(state="complete", output_epochs=int(total[1]),
                           processed_rover_epochs=comparison.native_input_epochs(log, "rtk"),
                           skipped_rover_epochs=int(skipped[1]), output_sha256={
                               s: sha256(prefix.with_suffix(s)) for s in (".pos", ".integrity.csv", ".log")})
    save(record, payload)
    if payload["state"] != "complete":
        raise RuntimeError(f"native replay failed: {record}")
    return payload


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    for option in ("baseline-bin", "candidate-bin", "dataset-root", "shadow-root", "output-dir"):
        parser.add_argument("--" + option, type=Path, required=True)
    parser.add_argument("--runs", nargs="+", default=[f"{c}/run{i}" for c in ("tokyo", "nagoya") for i in (1, 2, 3)])
    parser.add_argument("--max-epochs", type=int, default=-1)
    parser.add_argument("--resume", action="store_true")
    args = parser.parse_args()
    for run in args.runs:
        if run not in {relative for _, relative in audit.RUNS}:
            parser.error(f"unsupported PPC run: {run}")
        data = (args.dataset_root / run).resolve()
        shadow_dir = args.shadow_root / run.replace("/", "_")
        shadow = (shadow_dir / "covariance.shadow.csv").resolve()
        shadow_record = json.loads((shadow_dir / "covariance.run.json").read_text())
        if shadow_record.get("state") != "complete" or shadow_record["shadow_sha256"] != sha256(shadow):
            raise ValueError(f"shadow generation must be complete and hash verified: {shadow}")
        inputs = {name: sha256(data / name) for name in ("rover.obs", "base.obs", "base.nav", "reference.csv")}
        inputs["shadow"] = sha256(shadow)
        reference = build_reference_index(audit.load_reference(data / "reference.csv"))
        variants, solutions = {}, {}
        output = (args.output_dir / run.replace("/", "_")).resolve()
        for name, binary, with_shadow in (("baseline_plain", args.baseline_bin, False),
                ("candidate_plain", args.candidate_bin, False),
                ("baseline_shadow", args.baseline_bin, True),
                ("candidate_shadow", args.candidate_bin, True)):
            prefix = output / name
            argv = [str(binary.resolve()), "--rover", str(data / "rover.obs"),
                    "--base", str(data / "base.obs"), "--nav", str(data / "base.nav"),
                    "--preset", "low-cost", "--max-epochs", str(args.max_epochs), "--no-kml",
                    "--out", str(prefix.with_suffix(".pos")), "--realtime-fix-integrity",
                    "--integrity-log", str(prefix.with_suffix(".integrity.csv"))]
            if with_shadow:
                argv += ["--integrity-shadow-csv", str(shadow)]
            print(f"starting {run} {name}", flush=True)
            record = execute(argv, prefix, inputs, args.resume)
            native_log = prefix.with_suffix(".log").read_text(encoding="utf-8", errors="replace")
            metrics, solutions[name] = score(prefix.with_suffix(".pos"), reference,
                comparison.native_input_epochs(native_log, "rtk"))
            with prefix.with_suffix(".integrity.csv").open(newline="") as handle:
                telemetry = list(csv.DictReader(handle))
            health = re.search(r"integrity shadow health:\s*(\d+)/(\d+)",
                prefix.with_suffix(".log").read_text(encoding="utf-8", errors="replace"))
            if with_shadow and health is None:
                raise ValueError(f"missing native shadow health summary: {prefix}")
            variants[name] = dict(metrics=metrics, wall_s=record["wall_s"],
                states=dict(Counter(r["state"] for r in telemetry)),
                independent_position_valid=sum(r["independent_valid"] == "1" for r in telemetry),
                shadow_health_qualified=int(health[1]) if health else 0,
                shadow_health_lookups=int(health[2]) if health else 0,
                consensus_demoted=sum(r["consensus_demoted"] == "1" for r in telemetry))
            print(f"finished {run} {name}: {record['wall_s']:.3f}s", flush=True)
        comparisons = {}
        for before, after in (("baseline_plain", "candidate_plain"),
                              ("baseline_shadow", "candidate_shadow"),
                              ("candidate_plain", "candidate_shadow")):
            a, b = solutions[before], solutions[after]
            caught = harmed = 0
            for key in a.keys() & b.keys():
                truth = nearest_reference(reference, *key, 0.11)
                if truth is not None and a[key].status == 4 and b[key].status != 4:
                    if audit.error_3d_m(a[key].ecef, truth) > 2.0:
                        caught += 1
                    else:
                        harmed += 1
            comparisons[f"{before}_to_{after}"] = dict(caught_wrong_fix=caught,
                harmed_correct_fix=harmed, epoch_grid_identical=a.keys() == b.keys(),
                positions_identical=a.keys() == b.keys() and all(a[k].ecef == b[k].ecef for k in a),
                statuses_identical=a.keys() == b.keys() and all(a[k].status == b[k].status for k in a))
        save(output / "comparison.json", dict(run=run, variants=variants, comparisons=comparisons,
             reference_match_tolerance_s=0.11, wrong_fix_threshold_3d_m=2.0,
             expected_epoch_basis="native processed rover epochs: exact base + interpolated base + skipped"))


if __name__ == "__main__":
    main()
