#!/usr/bin/env python3
"""Run the PPC `--navi776-tc` replay nine times per run and report the spread.

Moving the lever arm by 2-4 cm moves the correct FIX of a single Tokyo run by
up to 5 pp (docs/ppc_rtk_vs_gnss_imu_fusion.md#run-to-run-spread), so one run
per arm cannot compare two RTK+IMU settings. This driver runs `gnss_fuse` on
each PPC run with the body-FLU lever arm and eight perturbations of it, scores
the coupled RTK stream, and prints the mean ± sample SD per run and the six-run
mean ± its standard error.

Scoring follows the lever-arm comparison: raw PPC reference, 0.25 s matching,
60-epoch skip, FIX = status 4, correct FIX = 3D error below 0.5 m.
`--truth-offset-body` moves the reference to the antenna first, for data whose
reference point is not the antenna (UrbanNav Hong Kong).

Example (the "both" column of the wrong-fix-guard table):

    python3 scripts/analysis/ppc_fuse_spread.py \\
        --dataset-root data/PPC-Dataset --out-dir output/ppc_fuse_spread/both \\
        -- --hold-kalman-update --min-fix-satellites 4
"""

from __future__ import annotations

import argparse
import bisect
from concurrent.futures import ThreadPoolExecutor
import csv
import dataclasses
import json
import math
import os
from pathlib import Path
import statistics
import subprocess
import sys

ROOT_DIR = Path(__file__).resolve().parents[2]
SCRIPTS_DIR = ROOT_DIR / "scripts"
if str(SCRIPTS_DIR) not in sys.path:
    sys.path.insert(0, str(SCRIPTS_DIR))

import generate_driving_comparison as comparison  # noqa: E402

RUNS = tuple(f"{city}/run{number}" for city in ("tokyo", "nagoya") for number in (1, 2, 3))
# Same command and body-FLU lever arms as gnss_ppc_native_replay.py.
COMMON = ["--preset", "low-cost", "--ratio", "2.4", "--max-subset-ar-drop-steps", "18",
          "--rtk-snr-weighting", "--no-arfilter"]
PRESET = ["--navi776-tc"]
LEVER_ARMS = {"tokyo": (0.31, 0.0, 0.55), "nagoya": (0.593, 0.670, 1.216)}
# The PPC README vectors (FRD), passed as FLU before the lever-arm fix.
PRE_FIX_LEVER_ARMS = {"tokyo": (0.31, 0.0, -0.55), "nagoya": (0.593, -0.670, -1.216)}
# Member 0 is the lever arm itself; the others move one axis by 2 or 4 cm.
PERTURBATIONS = ((0.0, 0.0, 0.0),
                 (0.0, 0.0, -0.04), (0.0, 0.0, -0.02), (0.0, 0.0, 0.02), (0.0, 0.0, 0.04),
                 (0.0, -0.02, 0.0), (0.0, 0.02, 0.0), (-0.02, 0.0, 0.0), (0.02, 0.0, 0.0))
MATCH_TOLERANCE_S = 0.25
SKIP_EPOCHS = 60
RTK_FIXED_STATUS = 4
CORRECT_3D_M = 0.5


def member_lever_arms(base: tuple[float, float, float]) -> list[str]:
    """Lever-arm arguments of the nine members, in the order of PERTURBATIONS."""
    return [",".join(f"{value + delta:.3f}" for value, delta in zip(base, offset))
            for offset in PERTURBATIONS]


def score(matched: list[comparison.MatchedEpoch],
          correct_3d_m: float = CORRECT_3D_M) -> dict[str, float]:
    """Correct FIX (% of epochs), wrong FIX (% of FIX), FIX rate and FIX above 3 m."""
    if not matched:
        raise ValueError("no epochs matched the reference")
    fixed = [epoch for epoch in matched if epoch.status == RTK_FIXED_STATUS]
    correct = sum(math.sqrt(e.east_m**2 + e.north_m**2 + e.up_m**2) < correct_3d_m for e in fixed)
    return {
        "correct_pct": 100.0 * correct / len(matched),
        "wrong_pct": 100.0 * (len(fixed) - correct) / len(fixed) if fixed else 0.0,
        "fix_pct": 100.0 * len(fixed) / len(matched),
        "fix_above_3m": float(sum(e.horiz_error_m > 3.0 for e in fixed)),
    }


def read_headings(reference_csv: Path) -> list[tuple[float, float]]:
    """(GPS TOW, heading in rad clockwise from north) of each reference row."""
    with reference_csv.open(newline="") as handle:
        reader = csv.reader(handle)
        column = next(reader).index("Heading (deg)")
        return [(float(row[0]), math.radians(float(row[column]))) for row in reader]


def move_truth_to_antenna(matched: list[comparison.MatchedEpoch],
                          headings: list[tuple[float, float]],
                          offset_body: tuple[float, float, float]) -> list[comparison.MatchedEpoch]:
    """Errors against the reference moved by a constant body-frame antenna offset.

    `offset_body` is the antenna minus the reference point, x right, y forward,
    z up, rotated to ENU with the heading of the nearest reference row.
    """
    times = [t for t, _ in headings]
    right, forward, up = offset_body
    moved = []
    for epoch in matched:
        i = bisect.bisect_left(times, epoch.tow)
        j = min((k for k in (i - 1, i) if 0 <= k < len(times)), key=lambda k: abs(times[k] - epoch.tow))
        h = headings[j][1]
        east = epoch.east_m - (forward * math.sin(h) + right * math.cos(h))
        north = epoch.north_m - (forward * math.cos(h) - right * math.sin(h))
        moved.append(dataclasses.replace(epoch, east_m=east, north_m=north, up_m=epoch.up_m - up,
                                         horiz_error_m=math.hypot(east, north)))
    return moved


def score_file(pos: Path, reference_csv: Path,
               correct_3d_m: float = CORRECT_3D_M,
               truth_offset_body: tuple[float, float, float] | None = None) -> dict[str, float]:
    reference = comparison.read_reference_csv(reference_csv)
    solution = comparison.read_libgnss_pos(pos)
    matched = comparison.match_to_reference(solution, reference, MATCH_TOLERANCE_S)[SKIP_EPOCHS:]
    if truth_offset_body is not None:
        matched = move_truth_to_antenna(matched, read_headings(reference_csv), truth_offset_body)
    return score(matched, correct_3d_m)


def summarize(per_run: dict[str, list[dict[str, float]]]) -> dict:
    """Per-run mean and sample SD, and the mean over runs with its standard error."""
    runs = {}
    for run, members in per_run.items():
        if len(members) < 2:
            raise ValueError(f"{run}: need at least two members for a spread")
        runs[run] = {key: {"mean": statistics.fmean(m[key] for m in members),
                           "sd": statistics.stdev(m[key] for m in members)}
                     for key in members[0]}
        runs[run]["members"] = len(members)
    overall = {}
    for key in ("correct_pct", "wrong_pct", "fix_pct", "fix_above_3m"):
        means = [runs[run][key]["mean"] for run in runs]
        variance = sum(runs[run][key]["sd"] ** 2 / runs[run]["members"] for run in runs)
        overall[key] = {"mean": statistics.fmean(means), "se": math.sqrt(variance) / len(runs)}
    return {"runs": runs, "mean": overall}


def format_table(summary: dict) -> str:
    lines = ["| Run | Correct FIX (% of epochs) | Wrong FIX (% of FIX) | FIX > 3 m |",
             "| --- | ---: | ---: | ---: |"]
    for run, row in summary["runs"].items():
        lines.append(f"| {run} | {row['correct_pct']['mean']:.2f} ± {row['correct_pct']['sd']:.2f} "
                     f"| {row['wrong_pct']['mean']:.2f} ± {row['wrong_pct']['sd']:.2f} "
                     f"| {row['fix_above_3m']['mean']:.0f} |")
    mean = summary["mean"]
    lines.append(f"| **Mean** | **{mean['correct_pct']['mean']:.2f} ± {mean['correct_pct']['se']:.2f}** "
                 f"| **{mean['wrong_pct']['mean']:.2f} ± {mean['wrong_pct']['se']:.2f}** "
                 f"| **{mean['fix_above_3m']['mean']:.0f}** |")
    return "\n".join(lines)


def find_gnss_fuse(explicit: Path | None) -> Path:
    candidates = [explicit] if explicit else []
    if os.environ.get("GNSSPP_BUILD_DIR"):
        candidates.append(Path(os.environ["GNSSPP_BUILD_DIR"]) / "apps" / "gnss_fuse")
    candidates.append(ROOT_DIR / "build" / "apps" / "gnss_fuse")
    for candidate in candidates:
        if candidate and candidate.is_file():
            return candidate
    raise FileNotFoundError("gnss_fuse not found; pass --gnss-fuse or set GNSSPP_BUILD_DIR")


def run_member(binary: Path, run_dir: Path, lever_arm: str, rtk_pos: Path,
               extra: list[str]) -> None:
    fused = rtk_pos.with_name(rtk_pos.stem + "_fused.pos")
    argv = [str(binary), "--data-dir", str(run_dir), "--lever-arm", lever_arm, *COMMON, *extra,
            "--out", str(fused), "--rtk-pos-out", str(rtk_pos)]
    with rtk_pos.with_suffix(".log").open("w", encoding="utf-8") as log:
        subprocess.run(argv, check=True, stdout=log, stderr=subprocess.STDOUT)
    fused.unlink(missing_ok=True)


def parse_args(argv: list[str] | None = None) -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__,
                                     formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("--dataset-root", type=Path, default=ROOT_DIR / "data" / "PPC-Dataset")
    parser.add_argument("--out-dir", type=Path, required=True)
    parser.add_argument("--gnss-fuse", type=Path, help="gnss_fuse binary (default: build tree)")
    parser.add_argument("--runs", nargs="+", default=list(RUNS),
                        help="run directories under --dataset-root (default: the six PPC runs)")
    parser.add_argument("--lever-arm", help="body-FLU lever arm x,y,z for every run, for runs "
                        "outside PPC (default: the PPC lever arm of each city)")
    parser.add_argument("--jobs", type=int, default=4)
    parser.add_argument("--pre-fix-lever-arms", action="store_true",
                        help="perturb the pre-fix (FRD) lever arms instead, for comparison only")
    parser.add_argument("--without-navi776-tc", action="store_true",
                        help="omit --navi776-tc so that the arguments after -- set the coupling")
    parser.add_argument("--correct-3d-m", type=float, default=CORRECT_3D_M,
                        help="3D error below which a FIX counts as correct; wrong FIX are "
                             "the others (default: 0.5)")
    parser.add_argument("--truth-offset-body", metavar="RIGHT,FORWARD,UP",
                        help="antenna minus reference point in the body frame (m), applied with "
                             "the reference heading before scoring; write it with '=' when the "
                             "first value is negative (default: none)")
    parser.add_argument("--score-only", action="store_true",
                        help="score the RTK streams already in --out-dir")
    parser.add_argument("extra", nargs=argparse.REMAINDER,
                        help="arguments after -- are appended to every gnss_fuse command")
    args = parser.parse_args(argv)
    if args.extra and args.extra[0] == "--":
        args.extra = args.extra[1:]
    if not args.without_navi776_tc:
        args.extra = PRESET + args.extra
    if args.jobs < 1:
        parser.error("--jobs must be >= 1")
    if not args.correct_3d_m > 0.0:
        parser.error("--correct-3d-m must be > 0")
    if args.truth_offset_body is not None:
        try:
            args.truth_offset_body = tuple(float(v) for v in args.truth_offset_body.split(","))
        except ValueError:
            parser.error("--truth-offset-body must be right,forward,up")
        if len(args.truth_offset_body) != 3 or not all(math.isfinite(v) for v in args.truth_offset_body):
            parser.error("--truth-offset-body must be three finite numbers right,forward,up")
    if args.lever_arm is not None:
        try:
            args.lever_arm = tuple(float(value) for value in args.lever_arm.split(","))
        except ValueError:
            parser.error("--lever-arm must be x,y,z")
        if len(args.lever_arm) != 3 or not all(math.isfinite(v) for v in args.lever_arm):
            parser.error("--lever-arm must be three finite numbers x,y,z")
    else:
        unknown = [run for run in args.runs if run.split("/")[0] not in LEVER_ARMS]
        if unknown:
            parser.error(f"--lever-arm is required for runs outside PPC: {', '.join(unknown)}")
    return args


def main(argv: list[str] | None = None) -> int:
    args = parse_args(argv)
    args.out_dir.mkdir(parents=True, exist_ok=True)
    bases = PRE_FIX_LEVER_ARMS if args.pre_fix_lever_arms else LEVER_ARMS
    if args.lever_arm is not None:
        bases = {run.split("/")[0]: args.lever_arm for run in args.runs}
    plan = []
    for run in args.runs:
        city = run.split("/")[0]
        for index, lever_arm in enumerate(member_lever_arms(bases[city])):
            pos = args.out_dir / f"{run.replace('/', '_')}_{index}.pos"
            plan.append((run, lever_arm, pos))
    if not args.score_only:
        binary = find_gnss_fuse(args.gnss_fuse)
        with ThreadPoolExecutor(max_workers=args.jobs) as pool:
            jobs = [pool.submit(run_member, binary, args.dataset_root / run, lever_arm, pos, args.extra)
                    for run, lever_arm, pos in plan]
            for job in jobs:
                job.result()
    per_run: dict[str, list[dict[str, float]]] = {run: [] for run in args.runs}
    for run, _, pos in plan:
        per_run[run].append(score_file(pos, args.dataset_root / run / "reference.csv",
                                       args.correct_3d_m, args.truth_offset_body))
    summary = summarize(per_run)
    summary["correct_3d_m"] = args.correct_3d_m
    if args.truth_offset_body is not None:
        summary["truth_offset_body"] = list(args.truth_offset_body)
    summary["command"] = {"common": COMMON, "arguments": args.extra,
                          "lever_arms": {run: member_lever_arms(bases[run.split("/")[0]])
                                         for run in args.runs}}
    (args.out_dir / "summary.json").write_text(json.dumps(summary, indent=2) + "\n", encoding="utf-8")
    print(format_table(summary))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
