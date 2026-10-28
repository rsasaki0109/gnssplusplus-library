#!/usr/bin/env python3
"""Rebuild the Kaggle GSDC 2023-2024 official submission CSV from the raw test drives.

Backs the ``gnss reproduce gsdc-official`` lane.  The recipe
(``configs/reproduce/gsdc_official_recipe.json``) pins, for every one of the 40
test drives, the input files (SHA-256), the exact ``gnss_fgo_imu_no_base`` argv
and the SHA-256 of the solution the research run wrote.  Nothing here talks to
Kaggle: the gate is the SHA-256 of the assembled ``submission.csv`` (the file
submitted as Kaggle ref 57742915); the Kaggle score is a readback record only.
Every drive, stage-0 run and height map is pinned to one Linux GCC build;
MSVC outputs of the same drives differ by up to 0.08 m.

Subcommands, in lane order:

``verify-inputs``
    Check the SHA-256 of every raw input and every train ``ground_truth.csv``
    the height and road maps need.
``run --stage stage0``
    Run the 15 stage-0 (offset + extra-band) replays whose trajectories select
    the height-map points.  A drive is skipped when its height map is already
    present with the pinned SHA-256.
``height-maps``
    Build the per-drive height maps from the stage-0 trajectories and the
    Kaggle train ground truth, and check them against the pinned SHA-256.
``road-maps``
    Build the per-drive road maps (train ground truth within 30 m of the
    drive's raw WLS track, with each point's heading) and check them against
    the pinned SHA-256.
``run --stage final``
    Run the 40 final replays (17 Pixel5 heading, 9 modern clock, 14 retained
    height-map/relative-height drives), one process at a time unless
    ``--jobs N`` runs N at once (outputs do not depend on it).
``assemble``
    Assemble ``submission.csv`` on the official key list (Kaggle
    sample_submission order), copying each native coordinate verbatim; when
    the recipe has ``road_snap``, move epochs across track toward a road map
    built from the train ground truth.  Report the SHA-256 gate plus per-drive
    identity and row differences against ``--reference-submission`` when that
    file is available.

``run`` is resumable: a drive whose ``run.json`` records exit 0 and whose
``solution.csv`` still has the recorded SHA-256 is skipped.

A drive with an ``input_window`` (taroz ``settings_test.csv`` IdxStart) is
solved from that epoch: ``run`` first writes its Android GNSS rows from the
window epoch and its IMU rows from ``imu_lead_ms`` earlier (raw lines, SHA-256
pinned) under the work dir, and ``assemble`` fills the official keys before
the first native row with that row, as taroz ``submission.m`` does.

A drive with ``single_cpu: true`` runs pinned to one CPU, so GTSAM's TBB
starts no worker threads: in the refinement IMU pass of lax-p Pixel5 a TBB
worker overflows its stack in about two runs out of three, while the
single-threaded solve finishes with the same solution.
"""

from __future__ import annotations

import argparse
from concurrent.futures import ThreadPoolExecutor
import csv
import hashlib
import json
import math
import os
from pathlib import Path
import subprocess
import sys
import threading
import time
from typing import Any, Callable, Iterable, Mapping, Sequence

ROOT_DIR = Path(__file__).resolve().parents[3]
DEFAULT_RECIPE = ROOT_DIR / "configs" / "reproduce" / "gsdc_official_recipe.json"
EARTH_RADIUS_M = 6371008.8
COLUMNS = ["tripId", "UnixTimeMillis", "LatitudeDegrees", "LongitudeDegrees"]


# --------------------------------------------------------------------------
# Recipe helpers
# --------------------------------------------------------------------------


def sha256_file(path: Path) -> str:
    digest = hashlib.sha256()
    with Path(path).open("rb") as handle:
        for chunk in iter(lambda: handle.read(1 << 20), b""):
            digest.update(chunk)
    return digest.hexdigest()


def load_recipe(path: Path) -> dict[str, Any]:
    recipe = json.loads(Path(path).read_text(encoding="utf-8"))
    if recipe.get("schema") != "gsdc_official_recipe.v1":
        raise SystemExit(f"{path}: unsupported recipe schema {recipe.get('schema')!r}")
    ids = [drive["id"] for drive in recipe["drives"]]
    if len(ids) != len(set(ids)):
        raise SystemExit(f"{path}: duplicate drive ids")
    if set(ids) != set(recipe["keys"]["runs"]):
        raise SystemExit(f"{path}: drives and key list disagree")
    return recipe


def expand_keys(runs: Sequence[Sequence[int]]) -> list[int]:
    """Decode ``[first, step, count]`` runs into UnixTimeMillis values."""
    keys: list[int] = []
    for first, step, count in runs:
        keys.extend(int(first) + int(step) * index for index in range(int(count)))
    return keys


def encode_keys(keys: Iterable[int]) -> list[list[int]]:
    """Inverse of :func:`expand_keys` (used to build and test recipes)."""
    runs: list[list[int]] = []
    for key in keys:
        key = int(key)
        if runs and runs[-1][2] >= 2 and key - (runs[-1][0] + runs[-1][1] * (runs[-1][2] - 1)) == runs[-1][1]:
            runs[-1][2] += 1
        elif runs and runs[-1][2] == 1 and key > runs[-1][0]:
            runs[-1][1] = key - runs[-1][0]
            runs[-1][2] = 2
        else:
            runs.append([key, 1000, 1])
    return runs


def render(text: str, context: Mapping[str, str]) -> str:
    for key, value in context.items():
        text = text.replace("{" + key + "}", value)
    return text


def make_context(args: argparse.Namespace) -> dict[str, str]:
    def portable(path: Path | None) -> str:
        return str(Path(path).resolve()).replace("\\", "/") if path is not None else ""

    return {
        "gsdc_root": portable(getattr(args, "gsdc_root", None)),
        "gsdc_truth_root": portable(getattr(args, "gsdc_truth_root", None)),
        "work_dir": portable(args.work_dir),
        "bin": str(getattr(args, "bin", "") or ""),
    }


def select(entries: Sequence[Mapping[str, Any]], wanted: Sequence[str] | None, limit: int | None) -> list[Mapping[str, Any]]:
    chosen = list(entries)
    if wanted:
        known = {entry["id"] for entry in entries}
        unknown = [item for item in wanted if item not in known]
        if unknown:
            raise SystemExit(f"unknown drive id(s): {', '.join(unknown)}")
        chosen = [entry for entry in entries if entry["id"] in wanted]
    if limit is not None:
        chosen = chosen[:limit]
    return chosen


def truth_candidates(truth_root: Path, relative: str) -> list[Path]:
    """``relative`` is ``train/<course>/<phone>/ground_truth.csv``."""
    _, course, phone, name = relative.split("/")
    return [
        truth_root / relative,
        truth_root / "dataset_2023" / relative,
        truth_root / course / phone / name,
        truth_root / f"{course}__{phone}__{name}",
    ]


def locate_truth(truth_root: Path, relative: str) -> Path | None:
    return next((path for path in truth_candidates(truth_root, relative) if path.is_file()), None)


# --------------------------------------------------------------------------
# verify-inputs
# --------------------------------------------------------------------------


def command_verify_inputs(args: argparse.Namespace, recipe: Mapping[str, Any]) -> int:
    gsdc_root = Path(args.gsdc_root)
    wanted: dict[str, str] = {}
    for entry in [*recipe["stage0"], *recipe["drives"]]:
        for relative, digest in entry["inputs"].items():
            if wanted.setdefault(relative, digest) != digest:
                raise SystemExit(f"recipe pins two hashes for {relative}")
    problems: list[str] = []
    cache_path = Path(args.work_dir) / "input_sha256_cache.json"
    cache: dict[str, Any] = {}
    if cache_path.is_file():
        cache = json.loads(cache_path.read_text(encoding="utf-8"))
    for relative, digest in sorted(wanted.items()):
        path = gsdc_root / relative
        if not path.is_file():
            problems.append(f"missing {path}")
            continue
        stat = path.stat()
        stamp = [stat.st_size, stat.st_mtime_ns]
        hit = cache.get(relative)
        observed = hit["sha256"] if hit and hit.get("stamp") == stamp else sha256_file(path)
        cache[relative] = {"stamp": stamp, "sha256": observed}
        if observed != digest:
            problems.append(f"SHA-256 mismatch {path}: {observed} != {digest}")
    truth_files = recipe["height_maps"]["truth_files"]
    maps_present = all(
        (Path(render(spec["path"], make_context(args)))).is_file()
        and sha256_file(Path(render(spec["path"], make_context(args)))) == spec["sha256"]
        for spec in recipe["height_maps"]["maps"].values()
    )
    truth_needed = not maps_present or "road_snap" in recipe or "road_maps" in recipe
    if truth_needed:
        if not args.gsdc_truth_root:
            problems.append("height maps are not built yet and --gsdc-truth-root is not set" if not maps_present
                            else "the road maps need the train ground truth and --gsdc-truth-root is not set")
        else:
            for relative, digest in truth_files.items():
                located = locate_truth(Path(args.gsdc_truth_root), relative)
                if located is None:
                    problems.append(f"missing ground truth {relative} under {args.gsdc_truth_root}")
                elif sha256_file(located) != digest:
                    problems.append(f"SHA-256 mismatch {located}")
    Path(args.work_dir).mkdir(parents=True, exist_ok=True)
    cache_path.write_text(json.dumps(cache, indent=1) + "\n", encoding="utf-8")
    for problem in problems:
        print(f"error: {problem}", file=sys.stderr)
    print(f"verify-inputs: {len(wanted)} raw inputs, "
          f"{f'{len(truth_files)} ground-truth files' if truth_needed else 'height maps already present'}; "
          f"{len(problems)} problem(s)")
    return 1 if problems else 0


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
            stop.wait(1.0)
    except psutil.Error:
        return


def completed(record_path: Path, solution: Path) -> dict[str, Any] | None:
    """Return the run record when the drive already completed and is intact."""
    if not record_path.is_file() or not solution.is_file():
        return None
    record = json.loads(record_path.read_text(encoding="utf-8"))
    if record.get("returncode") != 0 or record.get("solution_sha256") != sha256_file(solution):
        return None
    return record


def _row_utc_ms(line: str, column: int) -> int | None:
    row = next(csv.reader([line]))
    value = row[column] if len(row) > column else ""
    return int(float(value)) if value else None


def prepare_input_window(entry: Mapping[str, Any], context: Mapping[str, str]) -> int:
    """Write the windowed Android GNSS/IMU inputs of ``entry``; return the window start (UnixTimeMillis)."""
    window = entry["input_window"]
    root = Path(context["gsdc_root"])
    with (root / window["gnss"]["from"]).open(newline="", encoding="utf-8") as stream:
        column = next(csv.reader([stream.readline()])).index("utcTimeMillis")
        epochs = sorted({utc for utc in (_row_utc_ms(line, column) for line in stream) if utc is not None})
    start = epochs[window["first_epoch_index"]]
    for part, cut in (("gnss", start), ("imu", start - window["imu_lead_ms"])):
        spec = window[part]
        out = Path(render(spec["to"], context))
        out.parent.mkdir(parents=True, exist_ok=True)
        with (root / spec["from"]).open(newline="", encoding="utf-8") as stream, \
                out.open("w", newline="", encoding="utf-8") as sink:
            header = stream.readline()
            sink.write(header)
            column = next(csv.reader([header])).index("utcTimeMillis")
            for line in stream:
                utc = _row_utc_ms(line, column)
                if utc is not None and utc >= cut:
                    sink.write(line)
        if sha256_file(out) != spec["sha256"]:
            raise SystemExit(f"{entry['id']}: windowed {part} input {out} does not match its pinned SHA-256")
    return start


def map_is_pinned(recipe: Mapping[str, Any], drive_id: str, context: Mapping[str, str]) -> bool:
    spec = recipe["height_maps"]["maps"].get(drive_id)
    if spec is None:
        return False
    path = Path(render(spec["path"], context))
    return path.is_file() and sha256_file(path) == spec["sha256"]


def single_cpu_preexec(entry: Mapping[str, Any]) -> Callable[[], None] | None:
    """Child setup for ``single_cpu``: pin the solver to one CPU so GTSAM's TBB starts no worker threads."""
    if not entry.get("single_cpu"):
        return None
    if not hasattr(os, "sched_setaffinity"):
        raise SystemExit(f"{entry['id']}: single_cpu needs os.sched_setaffinity (Linux); the recipe pins a Linux build")
    cpu = min(os.sched_getaffinity(0))
    return lambda: os.sched_setaffinity(0, {cpu})


def run_drive(entry: Mapping[str, Any], context: Mapping[str, str], env: Mapping[str, str]) -> dict[str, Any]:
    argv = [render(item, context) for item in entry["argv"]]
    preexec = single_cpu_preexec(entry)
    solution = Path(render(entry["output"], context))
    folder = solution.parent
    folder.mkdir(parents=True, exist_ok=True)
    started_unix = time.time()
    started = time.monotonic()
    peak = [0]
    stop = threading.Event()
    with (folder / "stdout.log").open("w", encoding="utf-8", errors="replace") as out, \
            (folder / "stderr.log").open("w", encoding="utf-8", errors="replace") as err:
        process = subprocess.Popen(argv, stdout=out, stderr=err, env=dict(env), preexec_fn=preexec)
        monitor = threading.Thread(target=_monitor_peak_rss, args=(process.pid, stop, peak), daemon=True)
        monitor.start()
        returncode = process.wait()
        stop.set()
        monitor.join(timeout=5.0)
    elapsed = time.monotonic() - started
    observed = sha256_file(solution) if solution.is_file() else None
    return {
        "id": entry["id"],
        "group": entry["group"],
        "argv": argv,
        "single_cpu": bool(entry.get("single_cpu")),
        "returncode": returncode,
        "started_unix": round(started_unix, 3),
        "wall_s": round(elapsed, 1),
        "peak_rss_mb": round(peak[0] / (1024.0 * 1024.0), 1) if peak[0] else None,
        "solution_sha256": observed,
        "expected_sha256": entry["output_sha256"],
        "identical": observed == entry["output_sha256"],
    }


def command_run(args: argparse.Namespace, recipe: Mapping[str, Any]) -> int:
    context = make_context(args)
    if not context["bin"]:
        raise SystemExit("run: pass --bin <gnss_fgo_imu_no_base>")
    entries = select(recipe["stage0" if args.stage == "stage0" else "drives"], args.drives, args.limit)
    env = os.environ.copy()
    dll_dir = args.dll_dir or os.environ.get("GTSAM_BIN_DIR")
    if dll_dir:
        env["PATH"] = str(Path(dll_dir)) + os.pathsep + env.get("PATH", "")
    failures = 0
    progress_path = Path(args.work_dir) / f"{args.stage}_progress.json"
    progress: dict[str, Any] = {}
    lock = threading.Lock()

    def note(entry: Mapping[str, Any], record: Mapping[str, Any]) -> None:
        nonlocal failures
        with lock:
            progress[entry["id"]] = {key: record[key] for key in ("returncode", "wall_s", "identical", "solution_sha256")}
            if record["returncode"] != 0:
                failures += 1
            progress_path.parent.mkdir(parents=True, exist_ok=True)
            progress_path.write_text(json.dumps(progress, indent=1) + "\n", encoding="utf-8")

    def execute(label: str, entry: Mapping[str, Any], record_path: Path) -> None:
        if entry.get("input_window"):
            start = prepare_input_window(entry, context)
            print(f"{label}: inputs windowed from epoch {entry['input_window']['first_epoch_index']} "
                  f"(UnixTimeMillis {start})", flush=True)
        print(f"{label}: running (research wall {entry['research_source']['record_wall_s']} s)", flush=True)
        record = run_drive(entry, context, env)
        record_path.write_text(json.dumps(record, indent=1) + "\n", encoding="utf-8")
        print(f"{label}: exit {record['returncode']} in {record['wall_s']} s, "
              f"{'identical' if record['identical'] else 'DIFFERS from the research output'}", flush=True)
        note(entry, record)

    pending = []
    for index, entry in enumerate(entries, start=1):
        solution = Path(render(entry["output"], context))
        record_path = solution.parent / "run.json"
        label = f"[{args.stage} {index}/{len(entries)}] {entry['id']}"
        if args.stage == "stage0" and map_is_pinned(recipe, entry["id"], context):
            print(f"{label}: height map already pinned; stage-0 replay not needed", flush=True)
            progress[entry["id"]] = {"skipped": "height map pinned"}
            continue
        record = None if args.force else completed(record_path, solution)
        if record is not None:
            print(f"{label}: done ({'identical' if record['identical'] else 'DIFFERS'}, {record['wall_s']} s)", flush=True)
            note(entry, record)
        elif entry.get("height_map") and not Path(render(entry["height_map"], context)).is_file():
            print(f"{label}: height map {render(entry['height_map'], context)} missing "
                  "(run --stage stage0 and height-maps first)", file=sys.stderr, flush=True)
            failures += 1
        elif entry.get("road_map") and not Path(render(entry["road_map"], context)).is_file():
            print(f"{label}: road map {render(entry['road_map'], context)} missing (run road-maps first)",
                  file=sys.stderr, flush=True)
            failures += 1
        elif args.dry_run:
            print(f"{label}: would run\n  " + subprocess.list2cmdline([render(i, context) for i in entry["argv"]]))
        else:
            pending.append((label, entry, record_path))
    # Drives of one stage are independent; --jobs only changes how many run at once.
    with ThreadPoolExecutor(max_workers=max(1, args.jobs)) as pool:
        for future in [pool.submit(execute, *item) for item in pending]:
            future.result()
    return 1 if failures else 0


# --------------------------------------------------------------------------
# height-maps
# --------------------------------------------------------------------------


def build_height_map(solution: Path, truth: Sequence[tuple[str, Any]], out: Path) -> dict[str, Any]:
    """The research map builder: train truth points within 30 m of the trajectory.

    ``truth`` is the list of ``(flat file name, Nx3 lat/lon/alt array)`` in flat
    file-name order.  Byte identity of the written map depends on this exact
    pandas/numpy/scipy sequence, so it is kept as it was.
    """
    import numpy as np  # noqa: PLC0415
    import pandas as pd  # noqa: PLC0415
    from scipy.spatial import cKDTree  # noqa: PLC0415

    lat0 = np.radians(37.4)

    def en(lat: Any, lon: Any) -> Any:
        return np.c_[np.radians(lon) * EARTH_RADIUS_M * np.cos(lat0), np.radians(lat) * EARTH_RADIUS_M]

    points = np.vstack([values for _, values in truth])
    trajectory = pd.read_csv(solution)
    query = en(trajectory.LatitudeDegrees.values, trajectory.LongitudeDegrees.values)
    distance, _ = cKDTree(query).query(en(points[:, 0], points[:, 1]), distance_upper_bound=30)
    keep = points[np.isfinite(distance)]
    coverage = (
        float(np.isfinite(cKDTree(en(keep[:, 0], keep[:, 1])).query(query, distance_upper_bound=15)[0]).mean())
        if len(keep) else 0.0
    )
    out.parent.mkdir(parents=True, exist_ok=True)
    pd.DataFrame(keep, columns=["lat_deg", "lon_deg", "height_m"]).to_csv(out, index=False, float_format="%.9f")
    return {"points": int(len(keep)), "coverage_15m": coverage}


def load_truth(recipe: Mapping[str, Any], truth_root: Path) -> list[tuple[str, Any]]:
    import pandas as pd  # noqa: PLC0415

    loaded: list[tuple[str, Any]] = []
    for relative, digest in recipe["height_maps"]["truth_files"].items():
        _, course, phone, name = relative.split("/")
        located = locate_truth(truth_root, relative)
        if located is None:
            raise SystemExit(f"height-maps: ground truth {relative} not found under {truth_root}")
        if sha256_file(located) != digest:
            raise SystemExit(f"height-maps: SHA-256 mismatch for {located}")
        frame = pd.read_csv(located)
        loaded.append((f"{course}__{phone}__{name}",
                       frame[["LatitudeDegrees", "LongitudeDegrees", "AltitudeMeters"]].values))
    # The research builder globbed flat <course>__<phone>__ground_truth.csv names.
    loaded.sort(key=lambda item: item[0])
    return loaded


def command_height_maps(args: argparse.Namespace, recipe: Mapping[str, Any]) -> int:
    context = make_context(args)
    stage0 = {entry["id"]: entry for entry in recipe["stage0"]}
    wanted = select([{"id": key} for key in recipe["height_maps"]["maps"]], args.drives, None)
    truth: list[tuple[str, Any]] | None = None
    report: dict[str, Any] = {}
    mismatches = 0
    for item in wanted:
        drive_id = item["id"]
        spec = recipe["height_maps"]["maps"][drive_id]
        out = Path(render(spec["path"], context))
        if out.is_file() and sha256_file(out) == spec["sha256"] and not args.force:
            report[drive_id] = {"sha256": spec["sha256"], "identical": True, "built": False}
            continue
        solution = Path(render(stage0[drive_id]["output"], context))
        if not solution.is_file():
            print(f"error: {drive_id}: stage-0 solution {solution} missing (run --stage stage0)", file=sys.stderr)
            mismatches += 1
            continue
        if truth is None:
            if not args.gsdc_truth_root:
                raise SystemExit("height-maps: pass --gsdc-truth-root")
            truth = load_truth(recipe, Path(args.gsdc_truth_root))
        info = build_height_map(solution, truth, out)
        observed = sha256_file(out)
        report[drive_id] = {**info, "sha256": observed, "identical": observed == spec["sha256"], "built": True}
        print(f"{drive_id}: {info['points']} points, "
              f"{'identical' if observed == spec['sha256'] else 'DIFFERS from the pinned map'}", flush=True)
        if observed != spec["sha256"]:
            mismatches += 1
    Path(args.work_dir).mkdir(parents=True, exist_ok=True)
    (Path(args.work_dir) / "height_maps_report.json").write_text(json.dumps(report, indent=1) + "\n", encoding="utf-8")
    print(f"height-maps: {sum(1 for r in report.values() if r['identical'])}/{len(wanted)} match the pinned SHA-256")
    return 1 if mismatches and not args.allow_mismatch else 0


# --------------------------------------------------------------------------
# road-maps
# --------------------------------------------------------------------------


def load_road_points(recipe: Mapping[str, Any], truth_root: Path) -> Any:
    """Moving train-truth points with their unit local heading, in ``truth_files`` key order.

    Each file is sorted by time; a point is kept when its central-difference
    step (local east/north metres) exceeds 1 m, and that step is its heading.
    """
    import numpy as np  # noqa: PLC0415
    import pandas as pd  # noqa: PLC0415

    frames = []
    for relative, digest in sorted(recipe["height_maps"]["truth_files"].items()):
        located = locate_truth(truth_root, relative)
        if located is None:
            raise SystemExit(f"road-maps: ground truth {relative} not found under {truth_root}")
        if sha256_file(located) != digest:
            raise SystemExit(f"road-maps: SHA-256 mismatch for {located}")
        frame = pd.read_csv(located).sort_values("UnixTimeMillis")
        lat, lon = frame.LatitudeDegrees.values, frame.LongitudeDegrees.values
        north = np.radians(lat) * EARTH_RADIUS_M
        d_east = np.gradient(np.radians(lon)) * EARTH_RADIUS_M * np.cos(np.radians(lat))
        d_north = np.gradient(north)
        length = np.hypot(d_east, d_north)
        keep = length > 1.0
        frames.append(pd.DataFrame({
            "lat_deg": lat[keep], "lon_deg": lon[keep], "height_m": frame.AltitudeMeters.values[keep],
            "heading_east": d_east[keep] / length[keep], "heading_north": d_north[keep] / length[keep]}))
    return pd.concat(frames, ignore_index=True)


def build_road_map(device_gnss: Path, points: Any, out: Path) -> dict[str, Any]:
    """Keep the road points within 30 m of the drive's raw WLS track (no solver run, no test truth)."""
    import numpy as np  # noqa: PLC0415
    import pandas as pd  # noqa: PLC0415
    from scipy.spatial import cKDTree  # noqa: PLC0415

    columns = ["WlsPositionXEcefMeters", "WlsPositionYEcefMeters", "WlsPositionZEcefMeters"]
    xyz = pd.read_csv(device_gnss, usecols=columns).dropna().values
    a, f = 6378137.0, 1 / 298.257223563
    e2 = f * (2 - f)
    lon = np.arctan2(xyz[:, 1], xyz[:, 0])
    p = np.hypot(xyz[:, 0], xyz[:, 1])
    lat = np.arctan2(xyz[:, 2], p * (1 - e2))
    for _ in range(5):
        n = a / np.sqrt(1 - e2 * np.sin(lat) ** 2)
        height = p / np.cos(lat) - n
        lat = np.arctan2(xyz[:, 2], p * (1 - e2 * n / (n + height)))
    lat0 = np.radians(37.4)

    def en(lat_deg: Any, lon_deg: Any) -> Any:
        return np.c_[np.radians(lon_deg) * EARTH_RADIUS_M * np.cos(lat0), np.radians(lat_deg) * EARTH_RADIUS_M]

    distance, _ = cKDTree(en(np.degrees(lat), np.degrees(lon))).query(
        en(points.lat_deg.values, points.lon_deg.values), distance_upper_bound=30)
    keep = points[np.isfinite(distance)]
    out.parent.mkdir(parents=True, exist_ok=True)
    keep.to_csv(out, index=False, float_format="%.9f", lineterminator="\n")
    return {"points": int(len(keep))}


def command_road_maps(args: argparse.Namespace, recipe: Mapping[str, Any]) -> int:
    context = make_context(args)
    drives = {entry["id"]: entry for entry in recipe["drives"]}
    wanted = select([{"id": key} for key in recipe.get("road_maps", {}).get("maps", {})], args.drives, None)
    points = None
    report: dict[str, Any] = {}
    mismatches = 0
    for item in wanted:
        drive_id = item["id"]
        spec = recipe["road_maps"]["maps"][drive_id]
        out = Path(render(spec["path"], context))
        if out.is_file() and sha256_file(out) == spec["sha256"] and not args.force:
            report[drive_id] = {"sha256": spec["sha256"], "identical": True, "built": False}
            continue
        if points is None:
            if not args.gsdc_truth_root:
                raise SystemExit("road-maps: pass --gsdc-truth-root")
            points = load_road_points(recipe, Path(args.gsdc_truth_root))
        device_gnss = Path(args.gsdc_root) / next(
            relative for relative in drives[drive_id]["inputs"] if relative.endswith("/device_gnss.csv"))
        info = build_road_map(device_gnss, points, out)
        observed = sha256_file(out)
        report[drive_id] = {**info, "sha256": observed, "identical": observed == spec["sha256"], "built": True}
        print(f"{drive_id}: {info['points']} points, "
              f"{'identical' if observed == spec['sha256'] else 'DIFFERS from the pinned map'}", flush=True)
        mismatches += observed != spec["sha256"]
    Path(args.work_dir).mkdir(parents=True, exist_ok=True)
    (Path(args.work_dir) / "road_maps_report.json").write_text(json.dumps(report, indent=1) + "\n", encoding="utf-8")
    print(f"road-maps: {sum(1 for r in report.values() if r['identical'])}/{len(wanted)} match the pinned SHA-256")
    return 1 if mismatches and not args.allow_mismatch else 0


# --------------------------------------------------------------------------
# road snap
# --------------------------------------------------------------------------


def load_road_map(recipe: Mapping[str, Any], truth_root: Path) -> tuple[Any, Any]:
    """Road-map points (east/north metres) and unit headings from the train ground truth.

    Files are read in ``truth_files`` key order, each sorted by time; a point
    keeps its central-difference step as heading when that step exceeds
    ``map_min_step_m``.  The numpy sequence is the one the rule was fixed
    with, so the snapped coordinates (and the SHA-256 gate) depend on it.
    """
    import numpy as np  # noqa: PLC0415
    import pandas as pd  # noqa: PLC0415

    spec = recipe["road_snap"]
    radius, lat0 = spec["earth_radius_m"], np.radians(spec["lat0_deg"])
    points, headings = [], []
    for relative, digest in sorted(recipe["height_maps"]["truth_files"].items()):
        located = locate_truth(truth_root, relative)
        if located is None:
            raise SystemExit(f"road snap: ground truth {relative} not found under {truth_root}")
        if sha256_file(located) != digest:
            raise SystemExit(f"road snap: SHA-256 mismatch for {located}")
        frame = pd.read_csv(located).sort_values("UnixTimeMillis")
        xy = np.c_[np.radians(frame.LongitudeDegrees.values) * radius * np.cos(lat0),
                   np.radians(frame.LatitudeDegrees.values) * radius]
        step = np.gradient(xy, axis=0)
        length = np.linalg.norm(step, axis=1)
        keep = length > spec["map_min_step_m"]
        points.append(xy[keep])
        headings.append(step[keep] / length[keep, None])
    return np.vstack(points), np.vstack(headings)


def road_snap(recipe: Mapping[str, Any], truth_root: Path, path: Path) -> dict[str, Any]:
    """Move the epochs of the submission at ``path`` across track toward the road map, in place.

    Per epoch whose own step exceeds ``min_epoch_step_m``: map points within
    ``radius_m`` heading within ``heading_deg`` of it give its offset along
    their left normals; offsets under ``max_offset_m`` are kept and the epoch
    moves by ``-step * median`` along their normalised mean normal.  Every
    coordinate is then written with ``float_format``.
    """
    import numpy as np  # noqa: PLC0415
    import pandas as pd  # noqa: PLC0415
    from scipy.spatial import cKDTree  # noqa: PLC0415

    spec = recipe["road_snap"]
    radius, lat0 = spec["earth_radius_m"], np.radians(spec["lat0_deg"])
    points, headings = load_road_map(recipe, truth_root)
    tree = cKDTree(points)
    min_cos = np.cos(np.radians(spec["heading_deg"]))
    submission = pd.read_csv(path)
    snapped = submission.copy()
    moved = 0
    touched: list[str] = []
    largest = 0.0
    for trip, frame in submission.groupby("tripId", sort=False):
        frame = frame.sort_values("UnixTimeMillis")
        xy = np.c_[np.radians(frame.LongitudeDegrees.values) * radius * np.cos(lat0),
                   np.radians(frame.LatitudeDegrees.values) * radius]
        step = np.gradient(xy, axis=0)
        length = np.linalg.norm(step, axis=1)
        unit = step / np.maximum(length, 1e-6)[:, None]
        shift = np.zeros_like(xy)
        for i, near in enumerate(tree.query_ball_point(xy, spec["radius_m"])):
            if length[i] < spec["min_epoch_step_m"] or not near:
                continue
            near = np.array(near)
            near = near[(headings[near] @ unit[i]) > min_cos]
            if not len(near):
                continue
            normals = np.c_[-headings[near, 1], headings[near, 0]]
            offsets = np.einsum("ij,ij->i", xy[i] - points[near], normals)
            kept = np.abs(offsets) < spec["max_offset_m"]
            if kept.sum() >= 1:
                normal = normals[kept].mean(0)
                normal /= np.linalg.norm(normal)
                shift[i] = -spec["step"] * np.median(offsets[kept]) * normal
        out = xy + shift
        snapped.loc[frame.index, "LatitudeDegrees"] = np.degrees(out[:, 1] / radius)
        snapped.loc[frame.index, "LongitudeDegrees"] = np.degrees(out[:, 0] / (radius * np.cos(lat0)))
        size = np.linalg.norm(shift, axis=1)
        if (size > 0).any():
            moved += int((size > 0).sum())
            touched.append(trip)
            largest = max(largest, float(size.max()))
    snapped.to_csv(path, index=False, float_format=spec["float_format"], lineterminator="\n")
    return {"epochs_moved": moved, "drives_touched": len(touched), "max_shift_m": round(largest, 4)}


# --------------------------------------------------------------------------
# assemble
# --------------------------------------------------------------------------


def read_native(path: Path) -> dict[int, tuple[str, str]]:
    with Path(path).open(newline="", encoding="utf-8-sig") as stream:
        return {int(row["UnixTimeMillis"]): (row["LatitudeDegrees"], row["LongitudeDegrees"])
                for row in csv.DictReader(stream)}


def read_submission(path: Path) -> dict[str, dict[int, tuple[str, str]]]:
    rows: dict[str, dict[int, tuple[str, str]]] = {}
    with Path(path).open(newline="", encoding="utf-8-sig") as stream:
        for row in csv.DictReader(stream):
            rows.setdefault(row["tripId"], {})[int(row["UnixTimeMillis"])] = (
                row["LatitudeDegrees"], row["LongitudeDegrees"])
    return rows


def haversine_m(lat1: float, lon1: float, lat2: float, lon2: float) -> float:
    p1, p2 = math.radians(lat1), math.radians(lat2)
    a = math.sin((p2 - p1) / 2.0) ** 2 + math.cos(p1) * math.cos(p2) * math.sin(math.radians(lon2 - lon1) / 2.0) ** 2
    return 2.0 * EARTH_RADIUS_M * math.asin(min(1.0, math.sqrt(a)))


def assemble_rows(
    recipe: Mapping[str, Any],
    solutions: Mapping[str, Path],
    fallback: Mapping[str, dict[int, tuple[str, str]]] | None = None,
) -> tuple[list[list[str]], dict[str, str]]:
    """Return submission rows in official key order and the source of each drive.

    Missing drives are an error unless ``fallback`` (the reference submission)
    supplies them, which only a partial ``--allow-partial`` check uses.  A drive
    with an ``input_window`` takes its first native row for the keys before it.
    """
    rows: list[list[str]] = []
    sources: dict[str, str] = {}
    windowed = {entry["id"] for entry in recipe["drives"] if entry.get("input_window")}
    for trip, runs in recipe["keys"]["runs"].items():
        path = solutions.get(trip)
        if path is not None and Path(path).is_file():
            native = read_native(Path(path))
            sources[trip] = "native"
        elif fallback is not None and trip in fallback:
            native = fallback[trip]
            sources[trip] = "reference"
        else:
            raise SystemExit(f"assemble: no solution for {trip}")
        first = min(native) if trip in windowed and sources[trip] == "native" and native else None
        for key in expand_keys(runs):
            if key not in native and (first is None or key > first):
                raise SystemExit(f"assemble: {trip} has no native row for UnixTimeMillis {key}")
            lat, lon = native.get(key) or native[first]
            rows.append([trip, str(key), lat, lon])
    return rows, sources


def write_submission(rows: Sequence[Sequence[str]], path: Path) -> str:
    path.parent.mkdir(parents=True, exist_ok=True)
    with path.open("w", newline="", encoding="utf-8") as stream:
        writer = csv.writer(stream, lineterminator="\n")
        writer.writerow(COLUMNS)
        writer.writerows(rows)
    return sha256_file(path)


def compare_drives(
    rows: Sequence[Sequence[str]],
    reference: Mapping[str, dict[int, tuple[str, str]]] | None,
) -> dict[str, dict[str, Any]]:
    by_trip: dict[str, list[Sequence[str]]] = {}
    for row in rows:
        by_trip.setdefault(row[0], []).append(row)
    report: dict[str, dict[str, Any]] = {}
    for trip, trip_rows in by_trip.items():
        entry: dict[str, Any] = {"rows": len(trip_rows)}
        if reference is not None:
            expected = reference.get(trip, {})
            differing = 0
            worst = 0.0
            for _, key, lat, lon in trip_rows:
                other = expected.get(int(key))
                if other is None:
                    differing += 1
                    worst = math.inf
                elif other != (lat, lon):
                    differing += 1
                    worst = max(worst, haversine_m(float(lat), float(lon), float(other[0]), float(other[1])))
            entry["rows_differing"] = differing
            entry["max_horizontal_diff_m"] = worst if math.isfinite(worst) else None
        report[trip] = entry
    return report


def command_assemble(args: argparse.Namespace, recipe: Mapping[str, Any]) -> int:
    context = make_context(args)
    solutions = {entry["id"]: Path(render(entry["output"], context)) for entry in recipe["drives"]}
    reference = None
    if args.reference_submission and Path(args.reference_submission).is_file():
        reference = read_submission(Path(args.reference_submission))
    rows, sources = assemble_rows(recipe, solutions, reference if args.allow_partial else None)
    out = Path(args.out)
    observed = write_submission(rows, out)
    snap = None
    if "road_snap" in recipe:
        if not args.gsdc_truth_root:
            raise SystemExit("assemble: the road snap needs --gsdc-truth-root")
        snap = {"assembled_sha256": observed, **road_snap(recipe, Path(args.gsdc_truth_root), out)}
        observed = sha256_file(out)
        with out.open(newline="", encoding="utf-8") as stream:
            rows = list(csv.reader(stream))[1:]
    expected = recipe["submission"]["sha256"]
    drive_report = compare_drives(rows, reference)
    records: dict[str, Any] = {}
    for entry in recipe["drives"]:
        record_path = solutions[entry["id"]].parent / "run.json"
        record = json.loads(record_path.read_text(encoding="utf-8")) if record_path.is_file() else None
        solution_sha = sha256_file(solutions[entry["id"]]) if solutions[entry["id"]].is_file() else None
        drive_report[entry["id"]].update(
            group=entry["group"],
            source=sources[entry["id"]],
            solution_identical=solution_sha == entry["output_sha256"] if solution_sha else None,
            wall_s=record.get("wall_s") if record else None,
        )
        records[entry["id"]] = record
    native = [d for d in drive_report.values() if d["source"] == "native"]
    groups: dict[str, dict[str, int]] = {}
    for drive in native:
        counts = groups.setdefault(drive["group"], {"drives": 0, "identical": 0})
        counts["drives"] += 1
        counts["identical"] += int(bool(drive["solution_identical"]))
    metrics = {
        "schema": "gsdc_official_reproduce.v1",
        "submitted_to_kaggle": False,
        "kaggle_ref": recipe["submission"]["kaggle_ref"],
        "submission": {
            "path": str(out),
            "rows": len(rows),
            "sha256": observed,
            "expected_sha256": expected,
            "sha256_match": observed == expected,
            "partial": any(d["source"] != "native" for d in drive_report.values()),
        },
        "road_snap": snap,
        "drives_total": len(drive_report),
        "drives_native": len(native),
        "drives_identical": sum(1 for d in native if d["solution_identical"]),
        "drives_rows_identical": (
            sum(1 for d in native if d.get("rows_differing") == 0) if reference is not None else None
        ),
        "reference_submission": str(args.reference_submission) if reference is not None else None,
        "groups": groups,
        "final_wall_s": round(sum(d["wall_s"] or 0.0 for d in native), 1),
        "drives": [{"id": trip, **info} for trip, info in drive_report.items()],
    }
    Path(args.metrics_json).parent.mkdir(parents=True, exist_ok=True)
    Path(args.metrics_json).write_text(json.dumps(metrics, indent=2) + "\n", encoding="utf-8")
    if snap is not None:
        print(f"road snap: {snap['epochs_moved']} epochs moved on {snap['drives_touched']} drives "
              f"(max {snap['max_shift_m']} m; assembled before the snap: {snap['assembled_sha256']})")
    print(f"submission: {out} rows={len(rows)} sha256={observed}")
    print(f"expected  : {expected} -> {'MATCH' if observed == expected else 'DIFFERENT'}")
    print(f"native drives identical to the research outputs: {metrics['drives_identical']}/{len(native)}"
          + (f" (partial: {len(drive_report) - len(native)} drives taken from the reference)"
             if metrics["submission"]["partial"] else ""))
    for trip, info in drive_report.items():
        if info["source"] == "native" and (not info["solution_identical"] or info.get("rows_differing")):
            print(f"  {trip}: solution_identical={info['solution_identical']} "
                  f"rows_differing={info.get('rows_differing')} max_diff_m={info.get('max_horizontal_diff_m')}")
    return 0


# --------------------------------------------------------------------------
# CLI
# --------------------------------------------------------------------------


def parse_args(argv: Sequence[str] | None = None) -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("--recipe", type=Path, default=DEFAULT_RECIPE)
    sub = parser.add_subparsers(dest="command", required=True)

    def common(p: argparse.ArgumentParser) -> None:
        p.add_argument("--work-dir", type=Path, required=True)
        p.add_argument("--gsdc-root", type=Path, default=None, help="taroz dataset_2023 root (holds test/<drive>/)")
        p.add_argument("--gsdc-truth-root", type=Path, default=None,
                       help="Kaggle train ground truth (nested train/<course>/<phone>/ or flat "
                            "<course>__<phone>__ground_truth.csv)")
        p.add_argument("--drives", nargs="+", default=None, help="Only these drive ids (tripId).")

    verify = sub.add_parser("verify-inputs", help="Check SHA-256 of all inputs.")
    common(verify)
    run = sub.add_parser("run", help="Run the stage-0 or final native replays (resumable).")
    common(run)
    run.add_argument("--stage", choices=["stage0", "final"], required=True)
    run.add_argument("--bin", default=None, help="gnss_fgo_imu_no_base executable")
    run.add_argument("--dll-dir", default=None, help="Prepended to PATH (default: $GTSAM_BIN_DIR).")
    run.add_argument("--limit", type=int, default=None)
    run.add_argument("--force", action="store_true", help="Rerun drives that already completed.")
    run.add_argument("--jobs", type=int, default=1, help="Run this many drives at once (outputs do not depend on it).")
    run.add_argument("--dry-run", action="store_true")
    maps = sub.add_parser("height-maps", help="Build and check the height maps from stage-0 trajectories.")
    common(maps)
    maps.add_argument("--force", action="store_true")
    maps.add_argument("--allow-mismatch", action="store_true")
    roads = sub.add_parser("road-maps", help="Build and check the road maps from the raw WLS tracks and train truth.")
    common(roads)
    roads.add_argument("--force", action="store_true")
    roads.add_argument("--allow-mismatch", action="store_true")
    assemble = sub.add_parser("assemble", help="Assemble submission.csv and check the SHA-256 gate.")
    common(assemble)
    assemble.add_argument("--out", type=Path, required=True)
    assemble.add_argument("--metrics-json", type=Path, required=True)
    assemble.add_argument("--reference-submission", type=Path,
                          default=os.environ.get("GSDC_OFFICIAL_REFERENCE_SUBMISSION") or None,
                          help="The submitted CSV, for per-drive row diffs (optional; default "
                               "$GSDC_OFFICIAL_REFERENCE_SUBMISSION).")
    assemble.add_argument("--allow-partial", action="store_true",
                          help="Fill drives without a native solution from --reference-submission.")
    return parser.parse_args(argv)


def main(argv: Sequence[str] | None = None) -> int:
    args = parse_args(argv)
    recipe = load_recipe(args.recipe)
    handlers = {
        "verify-inputs": command_verify_inputs,
        "run": command_run,
        "height-maps": command_height_maps,
        "road-maps": command_road_maps,
        "assemble": command_assemble,
    }
    return handlers[args.command](args, recipe)


if __name__ == "__main__":
    raise SystemExit(main())
