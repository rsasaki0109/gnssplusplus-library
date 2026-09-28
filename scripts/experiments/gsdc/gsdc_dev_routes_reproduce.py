#!/usr/bin/env python3
"""Run and score the README GSDC base-surveyed dev-route table.

Two subcommands back the ``gnss reproduce gsdc-dev-routes`` lane:

``run``
    Launch one ``gnss_fgo_imu_no_base`` replay, keep its stdout/stderr in
    ``--log``, and write a JSON summary with the return code, wall time and
    peak resident memory (with ``psutil``).  On Windows the GTSAM DLL
    directory is prepended to ``PATH`` from ``--dll-dir`` or
    ``$GTSAM_BIN_DIR``.

``score``
    Score each route's ``--out`` CSV against the route's ``ground_truth.csv``
    with the metric used for the README table and the research record:
    horizontal error by spherical Haversine (R = 6371008.8 m) on an exact
    ``UnixTimeMillis`` join (one phone per route), then ``(P50 + P95) / 2``
    with linearly interpolated percentiles (``numpy.percentile`` default,
    as in the GSDC Kaggle metric).
"""

from __future__ import annotations

import argparse
import csv
import json
import math
import os
from pathlib import Path
import subprocess
import sys
import threading
import time
from typing import Any

EARTH_RADIUS_M = 6371008.8


def rounded(value: float | None, digits: int = 6) -> float | None:
    if value is None or not math.isfinite(value):
        return None
    return round(value, digits)


# --------------------------------------------------------------------------
# run
# --------------------------------------------------------------------------


def _monitor_peak_rss(pid: int, stop: threading.Event, peak: list[int]) -> None:
    try:
        import psutil  # type: ignore[import-not-found]
    except ImportError:
        return
    try:
        process = psutil.Process(pid)
        while not stop.is_set():
            peak[0] = max(peak[0], process.memory_info().rss)
            stop.wait(0.5)
    except psutil.Error:
        return


def command_run(args: argparse.Namespace) -> int:
    command = list(args.command)
    if command and command[0] == "--":
        command = command[1:]
    if not command:
        raise SystemExit("run: pass the gnss_fgo_imu_no_base command after `--`")
    env = os.environ.copy()
    dll_dir = args.dll_dir or os.environ.get("GTSAM_BIN_DIR")
    if dll_dir:
        env["PATH"] = str(Path(dll_dir)) + os.pathsep + env.get("PATH", "")
    for flag in ("--out", "--summary-json"):
        if flag in command:
            Path(command[command.index(flag) + 1]).parent.mkdir(parents=True, exist_ok=True)
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
            if args.echo:
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
    }
    args.summary_json.parent.mkdir(parents=True, exist_ok=True)
    args.summary_json.write_text(json.dumps(summary, indent=2) + "\n", encoding="utf-8")
    print(
        f"[{args.label}] exit {returncode} in {elapsed:.1f} s"
        + (f", peak RSS {summary['peak_rss_mb']} MB" if summary["peak_rss_mb"] else "")
    )
    if returncode != 0:
        tail = args.log.read_text(encoding="utf-8", errors="replace").splitlines()[-30:]
        sys.stdout.write("\n".join(tail) + "\n")
    return returncode


# --------------------------------------------------------------------------
# score
# --------------------------------------------------------------------------


def haversine_m(lat1: float, lon1: float, lat2: float, lon2: float) -> float:
    p1, p2 = math.radians(lat1), math.radians(lat2)
    dp = p2 - p1
    dl = math.radians(lon2 - lon1)
    a = math.sin(dp / 2.0) ** 2 + math.cos(p1) * math.cos(p2) * math.sin(dl / 2.0) ** 2
    return 2.0 * EARTH_RADIUS_M * math.asin(min(1.0, math.sqrt(a)))


def percentile(values: list[float], q: float) -> float:
    """Linear-interpolation percentile (``numpy.percentile`` default)."""
    if not values:
        raise ValueError("percentile of an empty list")
    ordered = sorted(values)
    rank = (len(ordered) - 1) * q / 100.0
    lower = math.floor(rank)
    upper = math.ceil(rank)
    if lower == upper:
        return ordered[int(rank)]
    return ordered[lower] + (ordered[upper] - ordered[lower]) * (rank - lower)


def read_positions(path: Path) -> dict[int, tuple[float, float]]:
    rows: dict[int, tuple[float, float]] = {}
    with path.open(newline="", encoding="utf-8") as handle:
        for row in csv.DictReader(handle):
            lat = row.get("LatitudeDegrees", "").strip()
            lon = row.get("LongitudeDegrees", "").strip()
            if not lat or not lon:
                continue
            key = int(float(row["UnixTimeMillis"]))
            if key in rows:
                raise ValueError(f"{path}: duplicate UnixTimeMillis {key}")
            rows[key] = (float(lat), float(lon))
    return rows


def score_route(prediction: Path, truth: Path) -> dict[str, Any]:
    predicted = read_positions(prediction)
    reference = read_positions(truth)
    keys = sorted(set(predicted) & set(reference))
    if not keys:
        raise ValueError(f"{prediction}: no UnixTimeMillis in common with {truth}")
    errors = [haversine_m(*predicted[key], *reference[key]) for key in keys]
    p50 = percentile(errors, 50.0)
    p95 = percentile(errors, 95.0)
    return {
        "prediction": str(prediction),
        "truth": str(truth),
        "prediction_rows": len(predicted),
        "truth_rows": len(reference),
        "matched_rows": len(keys),
        "unmatched_prediction_rows": len(predicted) - len(keys),
        "unmatched_truth_rows": len(reference) - len(keys),
        "p50_m": rounded(p50),
        "p95_m": rounded(p95),
        "score_m": rounded((p50 + p95) / 2.0),
        "mean_m": rounded(sum(errors) / len(errors)),
        "max_m": rounded(max(errors)),
    }


def load_json(path: Path) -> dict[str, Any]:
    if not path.is_file():
        return {}
    return json.loads(path.read_text(encoding="utf-8"))


def command_score(args: argparse.Namespace) -> int:
    rows: list[dict[str, Any]] = []
    for route in args.routes:
        run_dir = args.work_dir / "runs" / route
        row: dict[str, Any] = {"route": route}
        row.update(score_route(run_dir / "solution.csv", args.inputs_dir / route / "ground_truth.csv"))
        run = load_json(run_dir / "run_summary.json")
        native = load_json(run_dir / "native_summary.json")
        row["wall_time_s"] = run.get("wall_time_s")
        row["peak_rss_mb"] = run.get("peak_rss_mb")
        row["dataset_id"] = native.get("dataset_id")
        rows.append(row)
    scores = [row["score_m"] for row in rows]
    payload = {
        "schema": "gnss_gsdc_dev_routes_reproduce.v1",
        "metric": "(P50+P95)/2 horizontal, exact UnixTimeMillis join, Haversine R=6371008.8 m",
        "routes": rows,
        "mean_score_m": rounded(sum(scores) / len(scores)) if scores else None,
    }
    args.output_json.parent.mkdir(parents=True, exist_ok=True)
    args.output_json.write_text(json.dumps(payload, indent=2) + "\n", encoding="utf-8")
    markdown = render_markdown(payload)
    if args.markdown_output is not None:
        args.markdown_output.parent.mkdir(parents=True, exist_ok=True)
        args.markdown_output.write_text(markdown, encoding="utf-8")
    print(markdown)
    return 0


def _fmt(value: object, digits: int) -> str:
    if isinstance(value, (int, float)) and not isinstance(value, bool):
        return f"{value:.{digits}f}"
    return "n/a"


def render_markdown(payload: dict[str, Any]) -> str:
    lines = [
        "| route | (P50+P95)/2 m | P50 m | P95 m | mean m | matched rows | wall time s | peak RSS MB |",
        "|---|---:|---:|---:|---:|---:|---:|---:|",
    ]
    for row in payload["routes"]:
        lines.append(
            f"| {row['route']} | {_fmt(row['score_m'], 5)} | {_fmt(row['p50_m'], 4)} | {_fmt(row['p95_m'], 4)} | "
            f"{_fmt(row['mean_m'], 4)} | {row['matched_rows']}/{row['truth_rows']} | "
            f"{_fmt(row['wall_time_s'], 1)} | {_fmt(row['peak_rss_mb'], 0)} |"
        )
    return "\n".join(lines) + "\n"


def command_figure(args: argparse.Namespace) -> int:
    """Draw docs/gsdc_base_surveyed_osm.png from the scored routes."""
    payload = json.loads(args.metrics_json.read_text(encoding="utf-8"))
    specs = [
        f"{row['route']} ({row['score_m']:.3f} m):{row['prediction']}:{row['truth']}"
        for row in payload["routes"]
    ]
    scripts_dir = Path(__file__).resolve().parents[2]
    sys.path.insert(0, str(scripts_dir))
    import plot_gsdc_base_surveyed_osm as plot  # noqa: E402

    return plot.main(["--routes", *specs, "--output", str(args.output)]) or 0


# --------------------------------------------------------------------------
# CLI
# --------------------------------------------------------------------------


def parse_args(argv: list[str] | None = None) -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    subparsers = parser.add_subparsers(dest="command_name", required=True)

    run = subparsers.add_parser("run", help="Run one gnss_fgo_imu_no_base replay.")
    run.add_argument("--label", required=True)
    run.add_argument("--log", type=Path, required=True, help="Where to write the solver stdout/stderr.")
    run.add_argument("--summary-json", type=Path, required=True)
    run.add_argument("--dll-dir", default=None,
                     help="Directory prepended to PATH (GTSAM DLLs on Windows; default $GTSAM_BIN_DIR).")
    run.add_argument("--echo", action="store_true", help="Also echo the solver output.")
    run.add_argument("command", nargs=argparse.REMAINDER, help="-- gnss_fgo_imu_no_base ARGS...")

    score = subparsers.add_parser("score", help="Score route outputs against ground truth.")
    score.add_argument("--work-dir", type=Path, required=True,
                       help="Holds runs/<route>/{solution.csv,run_summary.json,native_summary.json}.")
    score.add_argument("--inputs-dir", type=Path, required=True,
                       help="Staged inputs holding <route>/ground_truth.csv.")
    score.add_argument("--routes", nargs="+", default=["H", "U", "A", "LAX-T"])
    score.add_argument("--output-json", type=Path, required=True)
    score.add_argument("--markdown-output", type=Path, default=None)

    figure = subparsers.add_parser("figure", help="Draw the OSM route figure (downloads OSM tiles).")
    figure.add_argument("--metrics-json", type=Path, required=True)
    figure.add_argument("--output", type=Path, required=True)
    return parser.parse_args(argv)


def main(argv: list[str] | None = None) -> int:
    args = parse_args(argv)
    if args.command_name == "run":
        return command_run(args)
    if args.command_name == "figure":
        return command_figure(args)
    return command_score(args)


if __name__ == "__main__":
    raise SystemExit(main())
