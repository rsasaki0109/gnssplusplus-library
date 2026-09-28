#!/usr/bin/env python3
"""Run and score the README GNSS/IMU FGO vs tightly-coupled-gnss-imu-fgo tables.

Two subcommands back the ``gnss reproduce fgo-tokyo`` lane:

``run``
    Launch one ``gnss_fgo_parity`` replay, echo its stdout (the lane runner
    keeps it in the step log), and write a JSON summary of the headline
    numbers the binary prints: wall time, FIXED/FLOAT horizontal RMS, the
    all-epoch horizontal <50 cm rate, the per-epoch LAMBDA fix rate, and the
    geometry-free slip reset counters.  On Windows the GTSAM DLL directory is
    prepended to ``PATH`` from ``--dll-dir`` or ``$GTSAM_BIN_DIR``.

``score``
    Score the ``--dump-csv`` outputs of the GF-reset preset and its baseline
    (the same preset without ``--gf-slip-reset``) against the PPC reference.
    It reports the README comparison table (all-epoch horizontal <50 cm,
    fix rate, and fixed-only horizontal RMS, computed as in
    ``scripts/plot_fgo_parity_runs.py``), the PPC distance-weighted
    correct/wrong FIX and official score (0.11 s matching, 0.5 m 3D
    threshold), and the published tightly-coupled-gnss-imu-fgo values with
    the per-run deltas and README claim counts.
"""

from __future__ import annotations

from _paths import ROOT_DIR  # noqa: F401  (sets sys.path for the shared helpers)

import argparse
import csv
import json
import math
import os
from pathlib import Path
import re
import subprocess
import sys
import threading
import time

import generate_driving_comparison as comparison  # noqa: E402
import gnss_ppc_metrics as metrics  # noqa: E402


# Published tightly-coupled-gnss-imu-fgo results on the same PPC Tokyo
# rover/base/IMU data (README comparison table; run1 is also printed by
# gnss_fgo_parity as the "inuex35 truth target").
REFERENCE_TC_FGO = {
    "tokyo_run1": {"under50_pct": 56.7, "fix_rate_pct": 49.5, "fixed_rms_h_m": 0.815},
    "tokyo_run2": {"under50_pct": 69.9, "fix_rate_pct": 60.8, "fixed_rms_h_m": 0.277},
    "tokyo_run3": {"under50_pct": 67.9, "fix_rate_pct": 59.4, "fixed_rms_h_m": 0.211},
}

STDOUT_PATTERNS = {
    "epochs": re.compile(r"lag=\S+ s, epochs=(\d+)"),
    "solver_wall_clock_s": re.compile(r"wall_clock=([0-9.eE+-]+) s"),
    "lambda_fixed_epochs": re.compile(r"fixed_epochs=(\d+)/\d+ \("),
    "lambda_fix_rate_pct": re.compile(r"fixed_epochs=\d+/\d+ \(([0-9.eE+-]+)% fix-rate\)"),
    "gf_confirmed_resets": re.compile(r"Geometry-free slip reset: \w+ \(confirmed_resets=(\d+)"),
    "gf_guard_demotions": re.compile(r"gross_spp_demotions=(\d+)"),
    "ref_float_epochs": re.compile(r"FLOAT: n=(\d+) rms="),
    "ref_float_rms_h_m": re.compile(r"FLOAT: n=\d+ rms=([0-9.eE+-]+) m"),
    "ref_fixed_epochs": re.compile(r"FIXED: n=(\d+) rms="),
    "ref_fixed_rms_h_m": re.compile(r"FIXED: n=\d+ rms=([0-9.eE+-]+) m"),
    "ref_under50_pct": re.compile(r"ALL \(float\+fixed\) <50cm rate=([0-9.eE+-]+)%"),
}


def rounded(value: float | None, digits: int = 6) -> float | None:
    if value is None or not math.isfinite(value):
        return None
    return round(value, digits)


# --------------------------------------------------------------------------
# run
# --------------------------------------------------------------------------


def parse_parity_stdout(text: str) -> dict[str, float | int | None]:
    parsed: dict[str, float | int | None] = {}
    for key, pattern in STDOUT_PATTERNS.items():
        matches = pattern.findall(text)
        if not matches:
            parsed[key] = None
            continue
        value = matches[-1]
        parsed[key] = int(value) if value.isdigit() else float(value)
    return parsed


def _monitor_peak_rss(pid: int, stop: threading.Event, peak: list[int]) -> None:
    try:
        import psutil  # type: ignore[import-not-found]
    except ImportError:
        return
    try:
        process = psutil.Process(pid)
        while not stop.is_set():
            peak[0] = max(peak[0], process.memory_info().rss)
            stop.wait(1.0)
    except psutil.Error:
        return


def command_run(args: argparse.Namespace) -> int:
    command = list(args.command)
    if command and command[0] == "--":
        command = command[1:]
    if not command:
        raise SystemExit("run: pass the gnss_fgo_parity command after `--`")
    env = os.environ.copy()
    dll_dir = args.dll_dir or os.environ.get("GTSAM_BIN_DIR")
    if dll_dir:
        env["PATH"] = str(Path(dll_dir)) + os.pathsep + env.get("PATH", "")
    args.log.parent.mkdir(parents=True, exist_ok=True)
    started = time.monotonic()
    peak = [0]
    stop = threading.Event()
    with args.log.open("w", encoding="utf-8", errors="replace") as log:
        process = subprocess.Popen(
            command,
            stdout=subprocess.PIPE,
            stderr=subprocess.STDOUT,
            text=True,
            encoding="utf-8",
            errors="replace",
            env=env,
        )
        monitor = threading.Thread(target=_monitor_peak_rss, args=(process.pid, stop, peak), daemon=True)
        monitor.start()
        assert process.stdout is not None
        for line in process.stdout:
            log.write(line)
            sys.stdout.write(line)
        returncode = process.wait()
        stop.set()
        monitor.join(timeout=5.0)
    elapsed = time.monotonic() - started
    summary = {
        "label": args.label,
        "returncode": returncode,
        "wall_time_s": round(elapsed, 3),
        "peak_rss_mb": round(peak[0] / (1024.0 * 1024.0), 1) if peak[0] else None,
        "log": str(args.log),
        "command": command,
        **parse_parity_stdout(args.log.read_text(encoding="utf-8", errors="replace")),
    }
    args.summary_json.parent.mkdir(parents=True, exist_ok=True)
    args.summary_json.write_text(json.dumps(summary, indent=2) + "\n", encoding="utf-8")
    print(
        f"[{args.label}] exit {returncode} in {elapsed:.1f} s"
        + (f", peak RSS {summary['peak_rss_mb']} MB" if summary["peak_rss_mb"] else "")
    )
    return returncode


# --------------------------------------------------------------------------
# score
# --------------------------------------------------------------------------


def dump_csv_table_metrics(path: Path) -> dict[str, float | int | None]:
    """README table metrics from a dump CSV (same definitions as plot_fgo_parity_runs.py)."""
    epochs = fixed = under50 = 0
    fixed_sq = 0.0
    with path.open(newline="", encoding="utf-8") as handle:
        for row in csv.DictReader(handle):
            status = row["status"].strip().upper()
            horiz = float(row["horiz_err_m"])
            epochs += 1
            if horiz < 0.5:
                under50 += 1
            if status == "FIXED":
                fixed += 1
                fixed_sq += horiz * horiz
    return {
        "epochs": epochs,
        "fixed_epochs": fixed,
        "under50_pct": rounded(100.0 * under50 / epochs) if epochs else None,
        "fix_rate_pct": rounded(100.0 * fixed / epochs) if epochs else None,
        "fixed_rms_h_m": rounded(math.sqrt(fixed_sq / fixed)) if fixed else None,
    }


def distance_fix_metrics(
    reference: list[comparison.ReferenceEpoch],
    solution: list[comparison.SolutionEpoch],
    match_tolerance_s: float,
) -> dict[str, float]:
    records = metrics.ppc_official_segment_records(reference, solution, match_tolerance_s, threshold_m=0.50)
    total = sum(float(record["segment_distance_m"]) for record in records)
    matched = sum(float(record["matched_distance_m"]) for record in records)
    scored = sum(float(record["score_distance_m"]) for record in records)
    correct_fix = sum(
        float(record["segment_distance_m"])
        for record in records
        if record["status"] == 4 and record["scored"]
    )
    wrong_fix = sum(
        float(record["segment_distance_m"])
        for record in records
        if record["status"] == 4 and record["matched"] and not record["scored"]
    )
    return {
        "total_distance_m": total,
        "matched_distance_m": matched,
        "score_distance_m": scored,
        "correct_fix_distance_m": correct_fix,
        "wrong_fix_distance_m": wrong_fix,
    }


def distance_percentages(values: dict[str, float]) -> dict[str, float | None]:
    total = values["total_distance_m"]
    fix = values["correct_fix_distance_m"] + values["wrong_fix_distance_m"]

    def pct(numerator: float, denominator: float) -> float | None:
        return rounded(100.0 * numerator / denominator) if denominator > 0.0 else None

    return {
        "correct_fix_distance_pct": pct(values["correct_fix_distance_m"], total),
        "wrong_fix_distance_pct": pct(values["wrong_fix_distance_m"], total),
        "official_score_pct": pct(values["score_distance_m"], total),
        "matched_distance_pct": pct(values["matched_distance_m"], total),
        "wrong_fix_over_fix_distance_pct": pct(values["wrong_fix_distance_m"], fix),
    }


def load_json(path: Path) -> dict[str, object] | None:
    if not path.is_file():
        return None
    return json.loads(path.read_text(encoding="utf-8"))


def score_variant(
    variant: str,
    runs: list[str],
    args: argparse.Namespace,
) -> dict[str, object]:
    rows: list[dict[str, object]] = []
    totals = {
        "total_distance_m": 0.0,
        "matched_distance_m": 0.0,
        "score_distance_m": 0.0,
        "correct_fix_distance_m": 0.0,
        "wrong_fix_distance_m": 0.0,
    }
    for run in runs:
        label = f"{args.city}_{run}"
        csv_path = args.work_dir / variant / f"{label}.csv"
        reference = comparison.read_reference_csv(args.dataset_root / args.city / run / "reference.csv")
        solution = metrics.load_fgo_parity_csv(csv_path, reference)
        distances = distance_fix_metrics(reference, solution, args.match_tolerance_s)
        for key in totals:
            totals[key] += distances[key]
        epoch_metrics = metrics.summarize_solution_epochs(
            reference,
            solution,
            fixed_status=4,
            label=f"FGO {variant} {label}",
            match_tolerance_s=args.match_tolerance_s,
            solver_wall_time_s=None,
        )
        parity = load_json(args.work_dir / variant / f"{label}_summary.json") or {}
        row: dict[str, object] = {
            "label": label,
            "csv": str(csv_path),
            **dump_csv_table_metrics(csv_path),
            **{key: rounded(value) for key, value in distances.items()},
            **distance_percentages(distances),
            "p95_h_m": epoch_metrics["p95_h_m"],
            "p95_abs_up_m": epoch_metrics["p95_abs_up_m"],
            "gf_confirmed_resets": parity.get("gf_confirmed_resets"),
            "gf_guard_demotions": parity.get("gf_guard_demotions"),
            "wall_time_s": parity.get("wall_time_s"),
            "peak_rss_mb": parity.get("peak_rss_mb"),
            "stdout": {
                key: parity.get(key)
                for key in (
                    "lambda_fix_rate_pct",
                    "ref_fixed_rms_h_m",
                    "ref_under50_pct",
                    "ref_fixed_epochs",
                    "ref_float_epochs",
                )
            },
        }
        rows.append(row)
    demotions = [row["gf_guard_demotions"] for row in rows]
    aggregate: dict[str, object] = {
        **{key: rounded(value) for key, value in totals.items()},
        **distance_percentages(totals),
        "gf_guard_demotions": sum(demotions) if all(isinstance(v, int) for v in demotions) else None,
    }
    return {"runs": rows, "aggregate": aggregate}


def compare_with_reference(rows: list[dict[str, object]]) -> dict[str, object]:
    comparisons: list[dict[str, object]] = []
    wins = {"under50_pct": 0, "fix_rate_pct": 0, "fixed_rms_h_m": 0}
    for row in rows:
        reference = REFERENCE_TC_FGO.get(str(row["label"]))
        if reference is None:
            continue
        entry: dict[str, object] = {"label": row["label"], "reference": reference}
        for key in wins:
            ours = row.get(key)
            if not isinstance(ours, (int, float)):
                entry[f"{key}_delta"] = None
                continue
            delta = float(ours) - reference[key]
            entry[f"{key}_delta"] = rounded(delta)
            better = delta < 0.0 if key == "fixed_rms_h_m" else delta > 0.0
            entry[f"{key}_better"] = better
            wins[key] += int(better)
        comparisons.append(entry)
    mean_delta: dict[str, float | None] = {}
    for key in wins:
        deltas = [entry.get(f"{key}_delta") for entry in comparisons]
        valid = [float(value) for value in deltas if isinstance(value, (int, float))]
        mean_delta[key] = rounded(sum(valid) / len(valid)) if valid and len(valid) == len(deltas) else None
    return {
        "runs": comparisons,
        "mean_delta": mean_delta,
        "runs_better": {
            "under50_pct": wins["under50_pct"],
            "fix_rate_pct": wins["fix_rate_pct"],
            "fixed_rms_h_m": wins["fixed_rms_h_m"],
        },
        "run_count": len(comparisons),
    }


def command_score(args: argparse.Namespace) -> int:
    runs = list(args.runs)
    payload: dict[str, object] = {
        "schema": "gnss_fgo_tokyo_reproduce.v1",
        "city": args.city,
        "match_tolerance_s": args.match_tolerance_s,
        "reference_tc_fgo": REFERENCE_TC_FGO,
        "variants": {},
    }
    variants: dict[str, object] = {}
    for variant in args.variants:
        variants[variant] = score_variant(variant, runs, args)
    payload["variants"] = variants
    primary = variants[args.variants[0]]
    assert isinstance(primary, dict)
    payload["vs_reference"] = compare_with_reference(primary["runs"])  # type: ignore[arg-type]
    args.output_json.parent.mkdir(parents=True, exist_ok=True)
    args.output_json.write_text(json.dumps(payload, indent=2) + "\n", encoding="utf-8")
    if args.markdown_output is not None:
        args.markdown_output.parent.mkdir(parents=True, exist_ok=True)
        args.markdown_output.write_text(render_markdown(payload, args.variants), encoding="utf-8")
    print(render_markdown(payload, args.variants))
    return 0


def _fmt(value: object, digits: int, suffix: str = "") -> str:
    if isinstance(value, (int, float)) and not isinstance(value, bool):
        return f"{value:.{digits}f}{suffix}"
    return "n/a"


def render_markdown(payload: dict[str, object], variant_names: list[str]) -> str:
    variants = payload["variants"]
    assert isinstance(variants, dict)
    primary = variants[variant_names[0]]
    lines = [
        f"### `{variant_names[0]}` vs tightly-coupled-gnss-imu-fgo",
        "",
        "| Run | libgnss++ <50cm | Reference <50cm | libgnss++ fix | Reference fix | libgnss++ fixed RMS | Reference fixed RMS | Wall time |",
        "|---|---:|---:|---:|---:|---:|---:|---:|",
    ]
    for row in primary["runs"]:
        reference = REFERENCE_TC_FGO.get(row["label"], {})
        lines.append(
            f"| {row['label']} | {_fmt(row['under50_pct'], 1, '%')} | {_fmt(reference.get('under50_pct'), 1, '%')} | "
            f"{_fmt(row['fix_rate_pct'], 1, '%')} | {_fmt(reference.get('fix_rate_pct'), 1, '%')} | "
            f"{_fmt(row['fixed_rms_h_m'], 3, ' m')} | {_fmt(reference.get('fixed_rms_h_m'), 3, ' m')} | "
            f"{_fmt(row['wall_time_s'], 1, ' s')} |"
        )
    if len(variant_names) > 1:
        base = variants[variant_names[1]]
        lines += [
            "",
            f"### `{variant_names[1]}` -> `{variant_names[0]}` (PPC distance-weighted)",
            "",
            "| Run | Correct FIX | Wrong FIX | Official score | GF guard demotions |",
            "|---|---:|---:|---:|---:|",
        ]
        pairs = list(zip(base["runs"], primary["runs"])) + [(base["aggregate"], primary["aggregate"])]
        for before, after in pairs:
            label = after.get("label", "aggregate")
            lines.append(
                f"| {label} | {_fmt(before['correct_fix_distance_pct'], 3, '%')} -> "
                f"{_fmt(after['correct_fix_distance_pct'], 3, '%')} | "
                f"{_fmt(before['wrong_fix_distance_pct'], 3, '%')} -> "
                f"{_fmt(after['wrong_fix_distance_pct'], 3, '%')} | "
                f"{_fmt(before['official_score_pct'], 3, '%')} -> {_fmt(after['official_score_pct'], 3, '%')} | "
                f"{after.get('gf_guard_demotions', 'n/a')} |"
            )
        lines.append(
            "\nWrong FIX/FIX: "
            f"{_fmt(base['aggregate']['wrong_fix_over_fix_distance_pct'], 3, '%')} -> "
            f"{_fmt(primary['aggregate']['wrong_fix_over_fix_distance_pct'], 3, '%')}; matched distance "
            f"{_fmt(base['aggregate']['matched_distance_pct'], 3, '%')} -> "
            f"{_fmt(primary['aggregate']['matched_distance_pct'], 3, '%')}"
        )
    return "\n".join(lines) + "\n"


# --------------------------------------------------------------------------
# CLI
# --------------------------------------------------------------------------


def parse_args(argv: list[str] | None = None) -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    subparsers = parser.add_subparsers(dest="command_name", required=True)

    run = subparsers.add_parser("run", help="Run one gnss_fgo_parity replay and summarize its stdout.")
    run.add_argument("--label", required=True)
    run.add_argument("--log", type=Path, required=True, help="Where to write the full stdout.")
    run.add_argument("--summary-json", type=Path, required=True)
    run.add_argument("--dll-dir", default=None,
                     help="Directory prepended to PATH (GTSAM DLLs on Windows; default $GTSAM_BIN_DIR).")
    run.add_argument("command", nargs=argparse.REMAINDER, help="-- gnss_fgo_parity ARGS...")

    score = subparsers.add_parser("score", help="Score dump CSVs and compare with the reference.")
    score.add_argument("--dataset-root", type=Path, required=True)
    score.add_argument("--work-dir", type=Path, required=True,
                       help="Holds <variant>/<city>_<run>.csv and <variant>/<city>_<run>_summary.json.")
    score.add_argument("--city", default="tokyo")
    score.add_argument("--runs", nargs="+", default=["run1", "run2", "run3"])
    score.add_argument("--variants", nargs="+", default=["gf_reset", "baseline"],
                       help="First variant is compared with the reference; the second is its baseline.")
    score.add_argument("--match-tolerance-s", type=float, default=0.11)
    score.add_argument("--output-json", type=Path, required=True)
    score.add_argument("--markdown-output", type=Path, default=None)
    return parser.parse_args(argv)


def main(argv: list[str] | None = None) -> int:
    args = parse_args(argv)
    if args.command_name == "run":
        return command_run(args)
    return command_score(args)


if __name__ == "__main__":
    raise SystemExit(main())
