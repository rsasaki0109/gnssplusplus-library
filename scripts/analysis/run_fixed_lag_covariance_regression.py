#!/usr/bin/env python3
"""Run pinned PPC baseline/candidate covariance comparisons serially.

Both executables must support --fixed-lag-lever-arm. The baseline solver is
unchanged; its CLI-only calibration patch is recorded separately in the report.
"""
from __future__ import annotations

import argparse
import hashlib
import json
from pathlib import Path
import re
import subprocess
import time

import compare_fixed_lag_covariance as comparison


def sha256(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as handle:
        for block in iter(lambda: handle.read(1024 * 1024), b""):
            digest.update(block)
    return digest.hexdigest()


def save(path: Path, payload: dict) -> None:
    pending = path.with_suffix(".pending.json")
    pending.write_text(json.dumps(payload, indent=2, allow_nan=False) + "\n", encoding="utf-8")
    pending.replace(path)


def execute(argv: list[str], prefix: Path, inputs: dict, resume: bool) -> dict:
    record = prefix.with_suffix(".run.json")
    csv_path = prefix.with_suffix(".csv")
    fingerprint = {"argv": argv, "binary_sha256": sha256(Path(argv[0])), "inputs": inputs}
    if record.exists():
        previous = json.loads(record.read_text(encoding="utf-8"))
        shadow = prefix.with_suffix(".shadow.csv")
        shadow_matches = "shadow_sha256" not in previous or (
            shadow.exists() and previous["shadow_sha256"] == sha256(shadow))
        if (resume and previous.get("state") == "complete" and
                previous.get("fingerprint") == fingerprint and csv_path.exists() and
                previous.get("csv_sha256") == sha256(csv_path) and shadow_matches):
            return previous
        raise RuntimeError(f"existing run is not a verified completed match: {record}; inspect its PID/state")
    prefix.parent.mkdir(parents=True, exist_ok=True)
    started = time.perf_counter()
    with prefix.with_suffix(".log").open("wb") as log:
        process = subprocess.Popen(argv, stdout=log, stderr=subprocess.STDOUT)
        payload = {"fingerprint": fingerprint, "state": "running", "pid": process.pid}
        save(record, payload)
        code = process.wait()
    payload.update(returncode=code, wall_s=time.perf_counter() - started)
    if sha256(Path(argv[0])) != fingerprint["binary_sha256"]:
        payload.update(state="invalid", reason="binary changed during run")
    elif code != 0 or not csv_path.exists():
        payload.update(state="failed")
    else:
        log_text = prefix.with_suffix(".log").read_text(encoding="utf-8", errors="replace")
        count = re.search(r"lag=.*?epochs=(\d+)", log_text)
        health = re.search(r"wall_clock=([\d.eE+-]+) s.*nonfinite=(\d+), NONE_epochs=(\d+)", log_text)
        payload.update(state="complete", csv_sha256=sha256(csv_path),
                       output_epochs=int(count[1]) if count else None)
        if health:
            payload.update(solver_wall_s=float(health[1]), nonfinite_epochs=int(health[2]),
                           none_epochs=int(health[3]))
        else:
            payload.update(state="invalid", reason="missing native final health summary")
        shadow = prefix.with_suffix(".shadow.csv")
        if shadow.exists():
            payload["shadow_sha256"] = sha256(shadow)
    save(record, payload)
    if payload["state"] != "complete":
        raise RuntimeError(f"native run did not complete successfully: {record}")
    return payload


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--baseline-bin", required=True, type=Path)
    parser.add_argument("--candidate-bin", required=True, type=Path)
    parser.add_argument("--dataset-root", required=True, type=Path)
    parser.add_argument("--output-dir", required=True, type=Path)
    parser.add_argument("--max-epochs", type=int, default=-1)
    parser.add_argument("--start-epoch", type=int, default=0)
    parser.add_argument("--runs", nargs="+", default=[f"{c}/run{i}" for c in ("tokyo", "nagoya") for i in (1, 2, 3)])
    parser.add_argument("--resume", action="store_true")
    args = parser.parse_args()
    for run in args.runs:
        if run not in {f"{c}/run{i}" for c in ("tokyo", "nagoya") for i in (1, 2, 3)}:
            parser.error(f"unsupported PPC run: {run}")
        city, _ = run.split("/")
        data = (args.dataset_root / run).resolve()
        inputs = {name: {"path": str(data / name), "sha256": sha256(data / name)}
                  for name in ("rover.obs", "base.obs", "base.nav", "imu.csv", "reference.csv")}
        common = ["--rover", str(data / "rover.obs"), "--base", str(data / "base.obs"),
                  "--nav", str(data / "base.nav"), "--imu", str(data / "imu.csv"),
                  "--ref", str(data / "reference.csv"), "--max-epochs", str(args.max_epochs),
                  "--start-epoch", str(args.start_epoch), "--fixed-lag", "5",
                  "--fixed-lag-lever-arm", "0.31,0,0.55" if city == "tokyo" else "0.593,-0.670,-1.216",
                  "--multi-freq", "--partial-ar", "--hold", "--elev-mask", "25", "--snr-mask", "30",
                  "--imu-preset-tactical", "--cmc", "--cmc-level", "0.75", "--cp-hold",
                  "--cp-hold-res", "2.0", "--exc-recovery", "--ddpr-anchor", "--fde", "--varerr",
                  "--fix-demote", "--fix-demote-dist", "5", "--fix-demote-res", "25",
                  "--fix-demote-posthold", "5"]
        records, rows = {}, {}
        output = (args.output_dir / run.replace("/", "_")).resolve()
        for variant, binary in (("baseline", args.baseline_bin), ("off", args.candidate_bin), ("covariance", args.candidate_bin)):
            prefix = output / variant
            argv = [str(binary.resolve()), *common, "--dump-csv", str(prefix.with_suffix(".csv"))]
            if variant != "baseline":
                argv += ["--dump-shadow-csv", str(prefix.with_suffix(".shadow.csv"))]
            if variant == "covariance":
                argv += ["--fixed-lag-covariance"]
            print(f"starting {run} {variant}", flush=True)
            records[variant] = execute(argv, prefix, inputs, args.resume)
            rows[variant] = comparison.load(prefix.with_suffix(".csv"))
            print(f"finished {run} {variant}: {records[variant]['wall_s']:.3f}s", flush=True)
        columns = ("tow", "status", "x_ecef_m", "y_ecef_m", "z_ecef_m", "ratio", "nfixed")
        same = lambda a, b: len(a) == len(b) and all(
            all(x.get(c) == y.get(c) for c in columns) for x, y in zip(a, b))
        payload = {"run": run, "max_epochs": args.max_epochs, "start_epoch": args.start_epoch,
                   "baseline_off_solution_text_identical": same(rows["baseline"], rows["off"]),
                   "off_covariance_solution_text_identical": same(rows["off"], rows["covariance"]),
                   "expected_epoch_basis": "native rover reader inputs, before problem construction and output filtering",
                   "metrics": {v: comparison.summarize(r, 2.0, comparison.native_input_epochs(
                       (output / f"{v}.log").read_text(encoding="utf-8", errors="replace"), "fgo"))
                       for v, r in rows.items()},
                   "wall_seconds": {v: r["wall_s"] for v, r in records.items()}}
        payload["solver_wall_seconds"] = {v: r["solver_wall_s"] for v, r in records.items()}
        payload["native_none_epochs"] = {v: r["none_epochs"] for v, r in records.items()}
        payload["native_nonfinite_epochs"] = {v: r["nonfinite_epochs"] for v, r in records.items()}
        # Full-precision truth-free exports give a stronger off/on position check.
        def shadow_positions(variant: str) -> list:
            import csv
            with (output / f"{variant}.shadow.csv").open(newline="", encoding="utf-8") as handle:
                return [tuple(row[k] for k in ("gps_week", "tow", "status", "x_ecef_m", "y_ecef_m", "z_ecef_m"))
                        for row in csv.DictReader(handle)]
        payload["off_covariance_full_precision_positions_identical"] = shadow_positions("off") == shadow_positions("covariance")
        save(output / "comparison.json", payload)


if __name__ == "__main__":
    main()
