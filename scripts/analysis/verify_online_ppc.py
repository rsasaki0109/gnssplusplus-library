#!/usr/bin/env python3
"""Stage raw PPC received events, verify numerical prefixes and outage behavior.

This is a simulated reception regression, not an observed network-latency or
accuracy benchmark. Native online code sees only the events already delivered.
"""
import argparse
import csv
import hashlib
import json
import math
from pathlib import Path
import queue
import subprocess
import threading
import time


def record(path):
    path = Path(path).resolve()
    digest = hashlib.sha256()
    with path.open("rb") as handle:
        for block in iter(lambda: handle.read(1048576), b""):
            digest.update(block)
    return {"path": str(path), "sha256": digest.hexdigest(), "bytes": path.stat().st_size}


def rows(path):
    with path.open(encoding="utf-8") as handle:
        return list(csv.DictReader(line for line in handle if not line.startswith("%")))


def numerical(row):
    return {key: value for key, value in row.items() if key != "processing_ms"}


def valid(row, prefix):
    return all(math.isfinite(float(row[prefix + axis + "_m"])) for axis in ("x", "y", "z"))


def statistics(data):
    durations = sorted(float(row["processing_ms"]) for row in data)
    return {
        "epochs": len(data), "valid_rtk": sum(valid(row, "rtk_") for row in data),
        "valid_fused": sum(valid(row, "fused_") for row in data),
        "exact_base": sum(int(row["exact_base"]) for row in data),
        "tight_updates": sum(int(row["tight_update"]) for row in data),
        "max_reset_generation": max(int(row["reset_generation"]) for row in data),
        "processing_p95_ms": durations[min(len(durations) - 1, math.ceil(.95 * len(durations)) - 1)],
        "processing_max_ms": durations[-1],
        "rtk_status_counts": {key: sum(row["rtk_status"] == key for row in data)
                              for key in sorted({row["rtk_status"] for row in data})},
        "reason_counts": {key: sum(row["reason"] == key for row in data)
                          for key in sorted({row["reason"] for row in data})},
    }


def streamed(exe, fixture, output, metadata, prefix, perturb_suffix=False):
    command = [str(exe), "--base-ecef", *map(str, metadata["base_ecef"]),
               "--lever-arm", *map(str, metadata["lever_arm"])]
    lines = queue.Queue()
    log_path = output.with_suffix(".stderr.log")
    with log_path.open("w", encoding="utf-8") as log:
        process = subprocess.Popen(command, stdin=subprocess.PIPE, stdout=subprocess.PIPE,
                                   stderr=log, text=True, encoding="utf-8", bufsize=1)
        def read():
            for line in process.stdout:
                lines.put(line)
            lines.put(None)
        thread = threading.Thread(target=read, daemon=True)
        thread.start()
        def get():
            try:
                line = lines.get(timeout=60)
            except queue.Empty as exc:
                raise RuntimeError("no streamed output within 60 seconds") from exc
            if line is None:
                raise RuntimeError("online process exited before output: " + log_path.read_text(encoding="utf-8"))
            return line
        emitted = 0
        waits = []
        try:
            with output.open("w", encoding="utf-8", newline="") as handle:
                handle.write(get())  # causal provenance declaration
                handle.write(get())  # CSV header
                with fixture.open(encoding="utf-8") as source:
                    for line in source:
                        if perturb_suffix and emitted == prefix and line.startswith("ROVER "):
                            # Change the suffix through a real state reset. Its
                            # timestamp is the next received rover event.
                            fields = line.split()
                            process.stdin.write(f"RESET {fields[1]} {fields[2]}\n")
                        process.stdin.write(line)
                        if line.startswith("ROVER "):
                            started = time.perf_counter()
                            process.stdin.flush()
                            handle.write(get())
                            waits.append((time.perf_counter() - started) * 1000)
                            emitted += 1
                process.stdin.close()
                code = process.wait(timeout=60)
                if code:
                    raise RuntimeError(f"online process failed ({code}): " + log_path.read_text(encoding="utf-8"))
        finally:
            if process.poll() is None:
                process.kill()
                process.wait()
            process.stdout.close()
        return {"argv": command, "outputs_received_before_next_rover_input": emitted,
                "stdin_remained_open_until_all_outputs_received": True,
                "delivery_to_output_max_ms": max(waits), "stderr": record(log_path)}


def verify(args):
    if args.output_dir.exists():
        raise ValueError("output directory must be new")
    args.output_dir.mkdir(parents=True)
    inputs = [record(args.run_dir / name) for name in ("rover.obs", "base.obs", "base.nav", "imu.csv")]
    binaries = [record(args.fixture_exe), record(args.online_exe)]
    staged = args.output_dir / "staged"
    command = [str(args.fixture_exe), str(args.run_dir), str(staged), str(args.epochs)]
    with (args.output_dir / "staging.log").open("w", encoding="utf-8") as log:
        subprocess.run(command, stdout=log, stderr=subprocess.STDOUT, check=True)
    metadata = json.loads((staged / "staging.json").read_text(encoding="utf-8"))
    prefix = metadata["prefix_epochs"]
    typed = {name: rows(staged / (name + ".csv"))
             for name in ("normal", "missing_base", "late_base", "imu_gap", "delayed_rover")}
    if any(len(data) != args.epochs for data in typed.values()):
        raise AssertionError("typed fixture did not emit all epochs")
    for name, data in typed.items():
        if [numerical(row) for row in data[:prefix]] != [numerical(row) for row in typed["normal"][:prefix]]:
            raise AssertionError("typed numerical prefix differs: " + name)
    stats = {name: statistics(data) for name, data in typed.items()}
    for actual, expected in zip(typed["delayed_rover"][prefix:], typed["normal"][prefix:]):
        excluded = {"processing_ms", "received_week", "received_tow", "input_age_s"}
        if {k: v for k, v in actual.items() if k not in excluded} != {k: v for k, v in expected.items() if k not in excluded}:
            raise AssertionError("delayed delivery changed chronological numerical output")
        if not math.isclose(float(actual["input_age_s"]), .15, abs_tol=1e-8):
            raise AssertionError("delayed reception age is incorrect")
    normal = stats["normal"]
    if not normal["valid_rtk"] or not normal["valid_fused"] or not normal["tight_updates"]:
        raise AssertionError("real typed RTK/fusion/tight time update was not exercised")
    if stats["missing_base"]["exact_base"] >= normal["exact_base"]:
        raise AssertionError("missing base was not exercised")
    if stats["late_base"]["exact_base"] != stats["missing_base"]["exact_base"]:
        raise AssertionError("late base revised an emitted epoch")
    if stats["imu_gap"]["max_reset_generation"] <= normal["max_reset_generation"]:
        raise AssertionError("IMU gap did not reset")
    if not any(row["fusion_initialized"] == "1" and valid(row, "fused_")
               for row in typed["imu_gap"][prefix + 30:]):
        raise AssertionError("fusion did not reinitialize after IMU gap")
    stream_reports = {}
    for name, perturb in (("normal", False), ("suffix_reset", True)):
        stream_reports[name] = streamed(args.online_exe, staged / "events.txt",
            args.output_dir / ("stream_" + name + ".csv"), metadata, prefix, perturb)
    stream = rows(args.output_dir / "stream_normal.csv")
    changed = rows(args.output_dir / "stream_suffix_reset.csv")
    if [numerical(row) for row in stream[:prefix]] != [numerical(row) for row in changed[:prefix]]:
        raise AssertionError("streamed numerical prefix differs")
    if not any(valid(row, "rtk_") for row in stream[:prefix]):
        raise AssertionError("transport prefix has no actual positioning outputs")
    if [numerical(row) for row in stream[prefix:]] == [numerical(row) for row in changed[prefix:]]:
        raise AssertionError("suffix perturbation did not change later state")
    for item in inputs + binaries:
        if record(item["path"]) != item:
            raise AssertionError("input/binary changed during verification")
    report = {"schema": "online_ppc_verification.v1", "state": "passed", "staging": metadata,
              "input_artifacts": inputs, "binaries": binaries, "staging_argv": command,
              "prefix_epochs_exactly_equal": prefix, "prefix_comparison_excludes": ["processing_ms"],
              "delayed_rover_numeric_parity": True,
              "typed_api": stats, "streaming": stream_reports, "transport": statistics(stream),
              "claims": "prefix causality and outage behavior for this declared received-event simulation; no full-run accuracy or real network latency claim"}
    report["artifacts"] = [record(path) for path in sorted(args.output_dir.rglob("*")) if path.is_file()]
    (args.output_dir / "report.json").write_text(json.dumps(report, indent=2, allow_nan=False) + "\n", encoding="utf-8")
    print(json.dumps({"state": "passed", "prefix_epochs": prefix, "typed_api": stats,
                      "transport": report["transport"]}, indent=2))


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--run-dir", type=Path, required=True)
    parser.add_argument("--fixture-exe", type=Path, required=True)
    parser.add_argument("--online-exe", type=Path, required=True)
    parser.add_argument("--output-dir", type=Path, required=True)
    parser.add_argument("--epochs", type=int, default=600)
    verify(parser.parse_args())


if __name__ == "__main__":
    main()
