#!/usr/bin/env python3
"""Replay two frozen binaries/settings on UrbanNav; reference is scoring only."""

import argparse
from concurrent.futures import ThreadPoolExecutor
import hashlib
import json
from pathlib import Path
import subprocess
import sys
import time

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from generate_driving_comparison import (  # noqa: E402
    ecef_to_enu,
    read_libgnss_pos,
    read_reference_csv,
)


def sha(path):
    return hashlib.sha256(Path(path).read_bytes()).hexdigest()


def stats(values):
    return (
        {
            "median": float(np.median(values)),
            "p95": float(np.percentile(values, 95)),
            "max": float(np.max(values)),
        }
        if values
        else None
    )


def score(path, reference):
    rows = {}
    unmatched = 0
    for row in read_libgnss_pos(path):
        key = (row.week, round(row.tow, 3))
        if key in rows:
            raise ValueError(f"duplicate epoch: {path}: {key}")
        truth = reference.get(key)
        if truth is None:
            unmatched += 1
            continue
        enu = ecef_to_enu(row.ecef - truth.ecef, truth.lat_deg, truth.lon_deg)
        if not np.all(np.isfinite(enu)):
            raise ValueError(f"nonfinite position: {path}: {key}")
        rows[key] = (row.status, float(np.hypot(*enu[:2])), float(np.linalg.norm(enu)))
    fixed = {k: v for k, v in rows.items() if v[0] == 4}
    result = {
        "reference_epochs": len(reference),
        "position_epochs": len(rows),
        "missing_epochs": len(reference) - len(rows),
        "unmatched_epochs": unmatched,
        "fix_epochs": len(fixed),
        "horizontal_m": stats([v[1] for v in rows.values()]),
        "fix_3d_m": stats([v[2] for v in fixed.values()]),
        "gross_fix_epochs": [k for k, v in fixed.items() if v[2] > 3],
    }
    good_sets = {}
    for threshold in (0.5, 2.0, 3.0):
        good = {k for k, v in fixed.items() if v[2] <= threshold}
        good_sets[str(threshold)] = good
        # Outages start after the first accurate FIX. Missing/FLOAT/wrong FIX all
        # count; a terminal outage is retained as right-censored, never dropped.
        start = None
        seen = False
        first = None
        delays = []
        times = sorted(reference)
        for week, tow in times:
            stamp = week * 604800 + tow
            if (week, tow) in good:
                if first is None:
                    first = stamp
                if start is not None and stamp - start >= 1:
                    delays.append(stamp - start)
                start = None
                seen = True
            elif seen and start is None:
                start = stamp
        first_stamp = times[0][0] * 604800 + times[0][1]
        end_stamp = times[-1][0] * 604800 + times[-1][1]
        result[f"threshold_{threshold}m"] = {
            "correct_fix": len(good),
            "wrong_fix": len(fixed) - len(good),
            "first_correct_fix_s": None if first is None else first - first_stamp,
            "completed_outages_ge_1s": len(delays),
            "recovery_s": stats(delays),
            "terminal_outage_right_censored": start is not None,
            "terminal_outage_elapsed_s": None if start is None else end_stamp - start,
        }
    return result, good_sets, rows


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--before", required=True, type=Path)
    parser.add_argument("--after", required=True, type=Path)
    parser.add_argument("--data-root", required=True, type=Path)
    parser.add_argument("--output", required=True, type=Path)
    parser.add_argument("--jobs", type=int, default=2)
    args = parser.parse_args()
    if args.jobs < 1:
        parser.error("--jobs must be positive")
    out = args.output.resolve()
    out.mkdir(parents=True, exist_ok=True)
    if any(out.iterdir()):
        parser.error("--output must be empty to preserve previous experiments")
    binaries = {"before": args.before.resolve(), "after": args.after.resolve()}
    sources = {}
    tasks = []
    for dataset in ("Odaiba", "Shinjuku"):
        root = (args.data_root / dataset).resolve()
        sources[dataset] = {
            name: {"path": str(root / name), "sha256": sha(root / name)}
            for name in (
                "rover_trimble.obs",
                "base_trimble.obs",
                "base.nav",
                "reference.csv",
            )
        }
        for version, binary in binaries.items():
            for recipe in ("default", "age5_doppler3"):
                name = f"{dataset}-{version}-{recipe}"
                argv = [
                    str(binary),
                    "--rover",
                    str(root / "rover_trimble.obs"),
                    "--base",
                    str(root / "base_trimble.obs"),
                    "--nav",
                    str(root / "base.nav"),
                    "--mode",
                    "kinematic",
                    "--glonass-ar",
                    "off",
                    "--base-ecef",
                    "-3961904.9530",
                    "3348993.7578",
                    "3698211.7553",
                    "--no-kml",
                    "--out",
                    str(out / (name + ".pos")),
                ]
                if recipe != "default":
                    argv += [
                        "--max-fixed-anchor-age",
                        "5",
                        "--max-fixed-doppler-consensus",
                        "3",
                    ]
                tasks.append((name, argv))
    manifest = {
        "binaries": {
            k: {"path": str(p), "sha256": sha(p)} for k, p in binaries.items()
        },
        "inputs": sources,
        "jobs": args.jobs,
        "runtime_note": "Concurrent wall times include contention; no speed claim.",
        "scoring": "Exact GPS week/TOW rounded to 1 ms; no interpolation. FIX status=4. "
        "3D error thresholds describe position accuracy, not integer correctness.",
        "source_diff_sha256": hashlib.sha256(
            subprocess.check_output(["git", "diff", "HEAD"])
        ).hexdigest(),
        "source_head": subprocess.check_output(
            ["git", "rev-parse", "HEAD"], text=True
        ).strip(),
        "runs": {},
    }
    (out / "provenance.json").write_text(json.dumps(manifest, indent=2) + "\n")

    def run(task):
        name, argv = task
        start = time.monotonic()
        with (out / (name + ".log")).open("w") as log:
            process = subprocess.run(argv, stdout=log, stderr=subprocess.STDOUT)
        result = {
            "argv": argv,
            "returncode": process.returncode,
            "wall_seconds": time.monotonic() - start,
        }
        path = out / (name + ".pos")
        if (
            process.returncode
            or not path.exists()
            or "output written:" not in (out / (name + ".log")).read_text()
        ):
            raise RuntimeError(f"incomplete run: {name}")
        result["sha256"] = sha(path)
        print(f"completed {name}: {result['wall_seconds']:.1f}s", flush=True)
        return name, result

    with ThreadPoolExecutor(max_workers=args.jobs) as pool:
        manifest["runs"] = dict(pool.map(run, tasks))
    for k, p in binaries.items():
        if sha(p) != manifest["binaries"][k]["sha256"]:
            raise RuntimeError("binary changed during evaluation")
    for files in sources.values():
        for item in files.values():
            if sha(item["path"]) != item["sha256"]:
                raise RuntimeError("input changed during evaluation")
    report = {"provenance": manifest, "datasets": {}}
    for dataset in sources:
        refs = read_reference_csv(args.data_root / dataset / "reference.csv")
        reference = {(r.week, round(r.tow, 3)): r for r in refs}
        if not reference or len(reference) != len(refs):
            raise ValueError("empty or duplicate reference epochs")
        scores, good, rows = {}, {}, {}
        for version in binaries:
            for recipe in ("default", "age5_doppler3"):
                key = version + "-" + recipe
                scores[key], good[key], rows[key] = score(
                    out / f"{dataset}-{key}.pos", reference
                )
        common = set.intersection(*(set(r) for r in rows.values()))
        for key, score_result in scores.items():
            score_result["common_position_epochs"] = len(common)
            score_result["common_horizontal_m"] = stats(
                [rows[key][k][1] for k in common]
            )
            for threshold, population in good["before-default"].items():
                score_result[f"threshold_{threshold}m"]["baseline_correct_fix_lost"] = (
                    len(population - good[key][threshold])
                )
        report["datasets"][dataset] = {
            "default_scored_rows_identical": rows["before-default"]
            == rows["after-default"],
            "default_pos_bytes_identical": sha(out / f"{dataset}-before-default.pos")
            == sha(out / f"{dataset}-after-default.pos"),
            "scores": scores,
        }
    (out / "provenance.json").write_text(json.dumps(manifest, indent=2) + "\n")
    (out / "evaluation.json").write_text(json.dumps(report, indent=2) + "\n")


if __name__ == "__main__":
    main()
