#!/usr/bin/env python3
"""Recorded-data real-time stress harness for ``gnss live``.

Replays a recorded RINEX run (for example a PPC run) as RTCM3 over local TCP or a minimal
local NTRIP caster with wall-clock pacing, injects link faults, runs ``gnss_live`` against the
sockets and measures survival, latency, recovery, accuracy against a reference, RSS/CPU and
parity with the batch ``gnss_solve``.  It is a recorded-data replay, not a field soak.

  gnss realtime-stress prepare --run-dir data/PPC-Dataset/tokyo/run1 --out-dir out/rt/prep
  gnss realtime-stress run --prepared out/rt/prep --out-dir out/rt/r1 --speed 5
  gnss realtime-stress run --prepared out/rt/prep --out-dir out/rt/f1 \\
      --fault base_outage@120+10 --fault corrupt_base@200+30=0.05
  gnss realtime-stress suite --prepared out/rt/prep --out-dir out/rt/suite --preset faults
"""

from __future__ import annotations

import argparse
import json
import os
import subprocess
import sys
import time
from concurrent.futures import ThreadPoolExecutor
from pathlib import Path
from typing import Any

from support.gnss_rtstress import (
    Fault, FAULT_KINDS, Prepared, Reference, Scenario, binary_supports, compare_to_batch, error_stats, find_binary,
    load_pos, run_scenario,
)


# gnss_live / gnss_solve option sets.  `moving` is the motion-aware fixed-jump gate that a vehicle
# rover needs (the bare default gate is a static 5 m one); `default` is bare gnss_live.
PROFILES = {
    "default": [],
    "moving": ["--max-pos-jump-min", "5", "--max-pos-jump-rate", "30"],
}


def live_args_for(args: argparse.Namespace) -> list[str]:
    return [*PROFILES[args.live_profile], *(args.live_arg or [])]


# ---------------------------------------------------------------------------
# prepare
# ---------------------------------------------------------------------------

def prepare(args: argparse.Namespace) -> int:
    run_dir = Path(args.run_dir)
    out_dir = Path(args.out_dir)
    out_dir.mkdir(parents=True, exist_ok=True)
    encode = find_binary("gnss_rtcm_encode", args.bin_dir)
    rover_obs = Path(args.rover_obs) if args.rover_obs else run_dir / "rover.obs"
    base_obs = Path(args.base_obs) if args.base_obs else run_dir / "base.obs"
    nav = Path(args.nav) if args.nav else run_dir / "base.nav"
    reference = Path(args.reference) if args.reference else run_dir / "reference.csv"
    for label, path in (("rover obs", rover_obs), ("base obs", base_obs), ("navigation", nav)):
        if not path.exists():
            raise SystemExit(f"missing {label}: {path}")

    window = []
    if args.start_offset:
        window += ["--start-offset", str(args.start_offset)]
    if args.duration:
        window += ["--duration", str(args.duration)]
    common = ["--systems", args.systems, "--format", args.format, *window]
    summaries = {}
    for role, obs, extra in (
        ("rover", rover_obs, ["--rinex-out", str(out_dir / "rover_src.obs")]),
        ("base", base_obs, ["--nav", str(nav), "--station", "--eph-interval", str(args.eph_interval),
                            *(["--eph-systems", args.eph_systems] if args.eph_systems else []),
                            "--rinex-out", str(out_dir / "base_src.obs")]),
    ):
        cmd = [encode, "--obs", str(obs), "--out", str(out_dir / f"{role}.rtcm"),
               "--index", str(out_dir / f"{role}.idx"),
               "--summary-json", str(out_dir / f"{role}_encode.json"), *common, *extra]
        print("+", " ".join(cmd))
        completed = subprocess.run(cmd, capture_output=True, text=True)
        if completed.returncode != 0:
            raise SystemExit(f"gnss_rtcm_encode failed for {role}:\n{completed.stdout}{completed.stderr}")
        summaries[role] = json.loads(completed.stdout.strip().splitlines()[-1])

    manifest = {
        "kind": "gnss_realtime_stress_prepared",
        "source_run": str(run_dir),
        "systems": args.systems,
        **({"eph_systems": args.eph_systems} if args.eph_systems else {}),
        "format": args.format,
        "window": {"start_offset": args.start_offset, "duration": args.duration},
        "files": {
            "rover_rtcm": "rover.rtcm", "rover_index": "rover.idx",
            "base_rtcm": "base.rtcm", "base_index": "base.idx",
            "rover_src_obs": "rover_src.obs", "base_src_obs": "base_src.obs",
        },
        "external": {"nav": str(nav.resolve()), "reference": str(reference.resolve()) if reference.exists() else None},
        "encode": summaries,
    }
    (out_dir / "manifest.json").write_text(json.dumps(manifest, indent=2) + "\n")
    print(f"prepared {summaries['rover']['epochs_written']} rover / {summaries['base']['epochs_written']} base epochs "
          f"-> {out_dir}")
    return 0


def _reference_for(prepared: Prepared) -> Reference | None:
    ref_path = prepared.manifest.get("files", {}).get("reference")
    if ref_path and (prepared.root / ref_path).exists():
        return Reference(prepared.root / ref_path)
    ext = prepared.manifest.get("external", {}).get("reference")
    if ext and Path(ext).exists():
        return Reference(Path(ext))
    return None


# ---------------------------------------------------------------------------
# batch parity
# ---------------------------------------------------------------------------

def run_batch(prepared: Prepared, bin_dir: str | None, out_pos: Path, extra: list[str] | None = None) -> dict[str, Any]:
    solve = find_binary("gnss_solve", bin_dir)
    nav = prepared.manifest["external"]["nav"]
    cmd = [solve, "--rover", str(prepared.path("rover_src_obs")), "--base", str(prepared.path("base_src_obs")),
           "--nav", nav, "--out", str(out_pos), "--no-kml", "--mode", "kinematic", *(extra or [])]
    t0 = time.monotonic()
    completed = subprocess.run(cmd, capture_output=True, text=True)
    wall = time.monotonic() - t0
    if completed.returncode != 0:
        raise SystemExit(f"gnss_solve failed:\n{completed.stdout[-2000:]}{completed.stderr[-2000:]}")
    return {"command": cmd, "wall_s": wall}


def run_offline_live(prepared: Prepared, bin_dir: str | None, out_pos: Path, live_args: list[str]) -> dict[str, Any]:
    """Run gnss_live on the same RTCM files (no sockets, no pacing): the reference for streaming effects."""
    live = find_binary("gnss_live", bin_dir)
    cmd = [live, "--rover-rtcm", str(prepared.path("rover_rtcm")), "--base-rtcm", str(prepared.path("base_rtcm")),
           "--out", str(out_pos), "--quiet", *live_args]
    t0 = time.monotonic()
    completed = subprocess.run(cmd, capture_output=True, text=True)
    return {"command": cmd, "wall_s": time.monotonic() - t0, "exit_code": completed.returncode,
            "stdout_tail": completed.stdout[-400:]}


def _rows(path: Path) -> list[str]:
    return [ln for ln in Path(path).read_text().splitlines() if ln and not ln.startswith("%")]


def parity(prepared: Prepared, result_dir: Path, bin_dir: str | None, batch_pos: Path | None = None,
           extra: list[str] | None = None, offline: bool = True) -> dict[str, Any]:
    """Compare a run with (a) batch gnss_solve and (b) gnss_live over the same files without sockets."""
    batch_pos = batch_pos or (prepared.root / "batch.pos")
    info: dict[str, Any] = {}
    if not batch_pos.exists():
        info = run_batch(prepared, bin_dir, batch_pos, extra)
    live: dict[int, tuple] = {}
    live_rows: list[str] = []
    for pos in sorted(Path(result_dir).glob("solution.*.pos")):
        live.update(load_pos(pos))
        live_rows += _rows(pos)
    batch = load_pos(batch_pos)
    out: dict[str, Any] = {"vs_gnss_solve": compare_to_batch(live, batch)}
    out["vs_gnss_solve"]["batch"] = info or {"cached": str(batch_pos)}
    if offline:
        offline_pos = Path(result_dir) / "offline_file.pos"
        off_info = run_offline_live(prepared, bin_dir, offline_pos, extra or [])
        off_rows = _rows(offline_pos) if offline_pos.exists() else []
        cmp_ = compare_to_batch(live, load_pos(offline_pos)) if offline_pos.exists() else {}
        cmp_.update({
            "rows_socket": len(live_rows), "rows_file": len(off_rows),
            "byte_identical_rows": live_rows == off_rows,
            "identical_row_count": sum(1 for a, b in zip(live_rows, off_rows) if a == b),
            "offline": {k: off_info[k] for k in ("wall_s", "exit_code")},
        })
        out["vs_gnss_live_files"] = cmp_
    return out


# ---------------------------------------------------------------------------
# scenarios
# ---------------------------------------------------------------------------

def _scn(name: str, **kw: Any) -> Scenario:
    return Scenario(name=name, **kw)


def preset_scenarios(preset: str, speed: float, window: float, transport: str) -> list[Scenario]:
    """Named scenario sets.  ``window`` is the run length in stream seconds.

    Fault times are stream seconds.  Windows of 300 s or more use fixed times (first fault at 100 s,
    second at 230 s, 10 s outages); shorter windows (the ctest smoke) scale them to the window.
    """
    if window >= 300.0:
        t1, t2, out_len, long_len = 100.0, 230.0, 10.0, 30.0
    else:
        t1, t2, out_len, long_len = 0.3 * window, 0.6 * window, 0.1 * window, 0.2 * window
    common: dict[str, Any] = {"speed": speed, "transport": transport}
    if preset in ("smoke", "faults", "faults_ntrip"):
        if preset == "faults_ntrip":
            common["transport"] = "ntrip"
        scenarios = [
            _scn("baseline", **common),
            _scn("base_outage", faults=[Fault("base_outage", t1, out_len)], **common),
            _scn("base_outage_long", faults=[Fault("base_outage", t1, long_len)], **common),
            _scn("base_stall", faults=[Fault("base_stall", t1, out_len)], **common),
            _scn("base_latency", faults=[Fault("base_latency", t1, 0.0, 2.0)], **common),
            _scn("base_jitter", faults=[Fault("base_jitter", t1, 0.0, 3.0)], **common),
            _scn("corrupt_base", faults=[Fault("corrupt_base", t1, 0.0, 0.05)], **common),
            _scn("corrupt_rover", faults=[Fault("corrupt_rover", t1, 0.0, 0.05)], **common),
            _scn("rover_disconnect", faults=[Fault("rover_disconnect", t1, out_len)], **common),
            _scn("rover_disconnect_supervised", faults=[Fault("rover_disconnect", t1, out_len)], supervise=True,
                 restart_delay=1.0 if speed <= 2.0 else 0.2, **common),
            _scn("rover_disconnect_reconnect", faults=[Fault("rover_disconnect", t1, out_len)], reconnect=True, **common),
            _scn("server_restart", faults=[Fault("server_restart", t1, out_len)], **common),
            _scn("server_restart_reconnect", faults=[Fault("server_restart", t1, out_len)], reconnect=True, **common),
            _scn("combined", faults=[Fault("base_outage", t1, out_len), Fault("corrupt_base", t2, 0.0, 0.03),
                                     Fault("base_latency", t2, 0.0, 1.0)], **common),
        ]
        return scenarios
    if preset == "transports":
        return [_scn("tcp_baseline", speed=speed, transport="tcp"), _scn("ntrip_baseline", speed=speed, transport="ntrip")]
    raise SystemExit(f"unknown preset {preset!r}")


# ---------------------------------------------------------------------------
# run / suite
# ---------------------------------------------------------------------------

def _log(msg: str) -> None:
    print(f"{time.strftime('%H:%M:%S')} {msg}", flush=True)


def execute(prepared: Prepared, scenario: Scenario, out_dir: Path, args: argparse.Namespace, reference: Reference | None) -> dict[str, Any]:
    result = run_scenario(prepared, scenario, out_dir, bin_dir=args.bin_dir, log=_log, reference=reference)
    if scenario.batch_parity or getattr(args, "batch_parity", False):
        try:
            result["batch_parity"] = parity(prepared, out_dir, args.bin_dir,
                                            batch_pos=prepared.root / f"batch.{args.live_profile}.pos",
                                            extra=PROFILES[args.live_profile])
        except SystemExit as exc:
            result["batch_parity"] = {"error": str(exc)}
        (out_dir / "result.json").write_text(json.dumps(result, indent=2, default=str) + "\n")
    return result


def one_line(result: dict[str, Any]) -> str:
    lat = result.get("latency_s", {})
    res = result.get("resources", {})
    acc = result.get("accuracy", {}).get("all", {})
    f = lambda v, p=2: "n/a" if v is None else f"{v:.{p}f}"
    return (f"{result['scenario']['name']}: survived={result['survived_to_end']} restarts={result['restarts']} "
            f"out={result['output_epochs']} lat_p50/p95/max={f(lat.get('p50'))}/{f(lat.get('p95'))}/{f(lat.get('max'))}s "
            f"fix={f(acc.get('fix_rate'), 3)} d3_p50={f(acc.get('d3_p50'))}m rss_peak={f(res.get('rss_peak_mb'), 1)}MB")


def cmd_run(args: argparse.Namespace) -> int:
    prepared = Prepared.load(Path(args.prepared))
    reference = _reference_for(prepared)
    faults = [Fault.parse(spec) for spec in args.fault]
    scenario = Scenario(
        name=args.name, speed=args.speed, transport=args.transport, faults=faults,
        start_offset=args.start_offset, duration=args.duration, loops=args.loops, loop_gap=args.loop_gap, max_wall=args.max_wall,
        supervise=args.supervise, restart_delay=args.restart_delay, reconnect=args.reconnect,
        reconnect_delay_ms=args.reconnect_delay_ms, live_args=live_args_for(args),
        sample_interval=args.sample_interval, compact=args.compact, batch_parity=args.batch_parity,
    )
    out_dir = Path(args.out_dir)
    result = execute(prepared, scenario, out_dir, args, reference)
    print(json.dumps(result, indent=2, default=str) if args.json else one_line(result))
    return 0 if (result["survived_to_end"] or faults or args.allow_failure) else 1


def cmd_suite(args: argparse.Namespace) -> int:
    prepared = Prepared.load(Path(args.prepared))
    reference = _reference_for(prepared)
    rover_idx = prepared.path("rover_index").read_text().splitlines()
    tows = [float(line.split()[0]) for line in rover_idx if line.strip()]
    window = (tows[-1] - tows[0]) if args.duration <= 0 else min(args.duration, tows[-1] - tows[0])
    scenarios = preset_scenarios(args.preset, args.speed, window, args.transport)
    live = find_binary("gnss_live", args.bin_dir)
    if not binary_supports(live, "--reconnect"):
        skipped = [s.name for s in scenarios if s.reconnect]
        if skipped:
            print("gnss_live has no --reconnect; skipping: " + ", ".join(skipped), file=sys.stderr)
        scenarios = [s for s in scenarios if not s.reconnect]
    if args.only:
        wanted = set(args.only)
        scenarios = [s for s in scenarios if s.name in wanted]
    for s in scenarios:
        s.duration = args.duration
        s.start_offset = args.start_offset
        s.live_args = live_args_for(args)
        if args.reconnect:
            s.reconnect = True
        s.sample_interval = args.sample_interval
        if args.batch_parity and s.name in ("baseline", "tcp_baseline"):
            s.batch_parity = True
    out_root = Path(args.out_dir)
    out_root.mkdir(parents=True, exist_ok=True)
    results: dict[str, Any] = {}

    def work(s: Scenario) -> tuple[str, dict[str, Any]]:
        return s.name, execute(prepared, s, out_root / s.name, args, reference)

    if args.jobs > 1:
        with ThreadPoolExecutor(max_workers=args.jobs) as pool:
            for name, res in pool.map(work, scenarios):
                results[name] = res
                _log(one_line(res))
    else:
        for s in scenarios:
            name, res = work(s)
            results[name] = res
            _log(one_line(res))
    summary = suite_summary(results, out_root)
    (out_root / "suite_summary.json").write_text(json.dumps(summary, indent=2, default=str) + "\n")
    print(format_table(summary))
    bad = [n for n, r in summary["scenarios"].items() if not r["faults_detected_all"]]
    if bad:
        print("faults not detected in: " + ", ".join(bad), file=sys.stderr)
    return 0 if not bad else 2


def _baseline_status(root: Path) -> list[tuple[float, int]]:
    path = root / "baseline" / "epochs.csv"
    seq: list[tuple[float, int]] = []
    if path.exists():
        import csv
        with open(path, newline="") as fh:
            for row in csv.DictReader(fh):
                try:
                    seq.append((float(row["stream_t"]), int(row["status"])))
                except (KeyError, ValueError):
                    continue
    return sorted(seq)


def _baseline_errors(root: Path) -> list[tuple[float, float, float, int, float]]:
    path = root / "baseline" / "epochs.csv"
    rows: list[tuple[float, float, float, int, float]] = []
    if path.exists():
        import csv
        with open(path, newline="") as fh:
            for row in csv.DictReader(fh):
                try:
                    rows.append((float(row["e"]), float(row["n"]), float(row["u"]), int(row["status"]), float(row["stream_t"])))
                except (KeyError, ValueError):
                    continue
    return rows


def suite_summary(results: dict[str, Any], root: Path | None = None) -> dict[str, Any]:
    base_seq = _baseline_status(root) if root is not None else []
    base_err = _baseline_errors(root) if root is not None else []
    rows: dict[str, Any] = {}
    for name, r in results.items():
        faults = r.get("faults", [])
        lat = r.get("latency_s", {})
        res = r.get("resources", {})
        acc = r.get("accuracy", {})
        allacc = acc.get("all", {})
        fault_rows = []
        for f in faults:
            end = f["at"] + f["dur"]
            base_fix = next((t - end for t, s in base_seq if t >= end and s == 4), None)
            rec_fix = f.get("recovery_fix_s")
            windows = {}
            for label in ("pre30", "pre60", "during", "post0_30", "post30_90", "post"):
                w = acc.get(f"fault{f['index']}_{f['kind']}_{label}")
                if w and w.get("epochs"):
                    windows[label] = {k: w.get(k) for k in ("epochs", "fix_rate", "d3_p50", "d3_p95", "d3_max", "fixed_wrong_gt_0p5m")}
            base_windows = {}
            if base_err:
                spans = ((("pre30", f["at"] - 30.0, f["at"]), ("during", f["at"], end),
                          ("post0_30", end, end + 30.0), ("post30_90", end + 30.0, end + 90.0))
                         if f["dur"] > 0 else (("pre60", f["at"] - 60.0, f["at"]), ("post", f["at"], 1.0e9)))
                for label, lo, hi in spans:
                    sel = [(e, n, u, st) for e, n, u, st, t in base_err if lo <= t < hi]
                    if sel:
                        base_windows[label] = {k: v for k, v in error_stats(sel).items()
                                               if k in ("epochs", "fix_rate", "d3_p50", "d3_p95", "d3_max", "fixed_wrong_gt_0p5m")}
            fault_rows.append({
                **{k: f.get(k) for k in ("kind", "at", "dur", "value", "detected", "evidence", "status_before_fault",
                                         "recovery_float_s", "recovery_fix_s", "max_output_gap_s")},
                "baseline_recovery_fix_s": base_fix,
                "excess_recovery_fix_s": (rec_fix - base_fix) if (rec_fix is not None and base_fix is not None) else None,
                "windows": windows,
                "baseline_windows": base_windows,
            })
        rows[name] = {
            "survived_to_end": r["survived_to_end"], "restarts": r["restarts"],
            "exit_codes": [p["exit_code"] for p in r["process"]],
            "terminations": [p["termination"] for p in r["process"]],
            "rover_epochs_sent": r["input"]["rover_epochs_sent"], "output_epochs": r["output_epochs"],
            "latency_p50_s": lat.get("p50"), "latency_p95_s": lat.get("p95"), "latency_max_s": lat.get("max"),
            "fix_rate": allacc.get("fix_rate"), "d3_p50_m": allacc.get("d3_p50"), "d3_p95_m": allacc.get("d3_p95"),
            "fixed_wrong_gt_0p5m": allacc.get("fixed_wrong_gt_0p5m"),
            "rss_peak_mb": res.get("rss_peak_mb"), "cpu_pct_mean": res.get("cpu_pct_mean"),
            "faults": fault_rows,
            "faults_detected_all": all(f["detected"] for f in faults) if faults else True,
            "decoder_errors": {"rover": r["live_totals"].get("rover_decoder_errors"),
                               "base": r["live_totals"].get("base_decoder_errors")},
            "skipped_rover_epochs": r["live_totals"].get("skipped_rover_epochs"),
            "held_base_epochs": r["live_totals"].get("held_base_epochs"),
            "wall_total_s": r.get("wall_total_s"),
        }
    return {"scenarios": rows}


def format_table(summary: dict[str, Any]) -> str:
    f = lambda v, p=2: "-" if v is None else f"{v:.{p}f}"
    lines = ["scenario | survived | restarts | out | lat p50/p95/max (s) | fix | d3 p50 (m) | recov float/fix (s) | detected"]
    for name, r in summary["scenarios"].items():
        rec = "; ".join(f"{f(x['recovery_float_s'], 1)}/{f(x['recovery_fix_s'], 1)}" for x in r["faults"]) or "-"
        det = "; ".join("Y" if x["detected"] else "N" for x in r["faults"]) or "-"
        lines.append(f"{name} | {r['survived_to_end']} | {r['restarts']} | {r['output_epochs']} | "
                     f"{f(r['latency_p50_s'])}/{f(r['latency_p95_s'])}/{f(r['latency_max_s'])} | {f(r['fix_rate'], 3)} | "
                     f"{f(r['d3_p50_m'])} | {rec} | {det}")
    return "\n".join(lines)


def cmd_report(args: argparse.Namespace) -> int:
    """Rebuild suite_summary.json and print the table from the result.json files of a finished suite."""
    root = Path(args.suite_dir)
    results = {}
    for result in sorted(root.glob("*/result.json")):
        results[result.parent.name] = json.loads(result.read_text())
    if not results:
        raise SystemExit(f"no */result.json under {root}")
    summary = suite_summary(results, root)
    (root / "suite_summary.json").write_text(json.dumps(summary, indent=2, default=str) + "\n")
    print(format_table(summary))
    return 0


def cmd_batch(args: argparse.Namespace) -> int:
    prepared = Prepared.load(Path(args.prepared))
    out = Path(args.out) if args.out else prepared.root / f"batch.{args.live_profile}.pos"
    info = run_batch(prepared, args.bin_dir, out, PROFILES[args.live_profile])
    print(f"batch solution -> {out} ({info['wall_s']:.1f}s)")
    return 0


def cmd_parity(args: argparse.Namespace) -> int:
    prepared = Prepared.load(Path(args.prepared))
    res = parity(prepared, Path(args.result_dir), args.bin_dir,
                 batch_pos=prepared.root / f"batch.{args.live_profile}.pos", extra=PROFILES[args.live_profile])
    print(json.dumps(res, indent=2, default=str))
    return 0


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(prog=os.environ.get("GNSS_CLI_NAME"), description=__doc__.split("\n\n")[0])
    parser.add_argument("--bin-dir", default=None, help="Directory (or build dir) holding gnss_live/gnss_rtcm_encode/gnss_solve.")
    sub = parser.add_subparsers(dest="command", required=True)

    p = sub.add_parser("prepare", help="Encode a recorded RINEX run into RTCM3 streams plus indexes.")
    p.add_argument("--run-dir", required=True, help="Directory with rover.obs, base.obs, base.nav, reference.csv (e.g. PPC run).")
    p.add_argument("--out-dir", required=True)
    p.add_argument("--rover-obs"), p.add_argument("--base-obs"), p.add_argument("--nav"), p.add_argument("--reference")
    p.add_argument("--systems", default="G", help="Constellations to encode, e.g. G,R,E,J,C (gnss_live assembles the "
                   "per-constellation MSM messages of an epoch). By default the base stream carries only 1019/1020 "
                   "ephemerides, so for E/J/C also pass --eph-systems (below) or give run "
                   "--live-arg=--nav-rinex --live-arg=<base.nav>. Default G, the setting of the fault-matrix and "
                   "soak results.")
    p.add_argument("--eph-systems", default=None, metavar="LIST",
                   help="Constellations that get ephemeris messages in the base stream, passed to gnss_rtcm_encode "
                   "(base only): G 1019, R 1020, E 1045/1046, J 1044, C 1042. Only systems that are also in "
                   "--systems are broadcast. Default: not passed, so the encoder's own default G,R applies.")
    p.add_argument("--format", default="msm7", choices=("msm7", "msm5", "msm4", "legacy"))
    p.add_argument("--start-offset", type=float, default=0.0)
    p.add_argument("--duration", type=float, default=0.0)
    p.add_argument("--eph-interval", type=float, default=30.0, help="Seconds between repeats of each ephemeris message (default 30).")
    p.set_defaults(func=prepare)

    def add_run_args(q: argparse.ArgumentParser) -> None:
        q.add_argument("--prepared", required=True)
        q.add_argument("--out-dir", required=True)
        q.add_argument("--speed", type=float, default=1.0, help="Pacing factor (1 = real time).")
        q.add_argument("--transport", choices=("tcp", "ntrip"), default="tcp")
        q.add_argument("--start-offset", type=float, default=0.0)
        q.add_argument("--duration", type=float, default=0.0)
        q.add_argument("--live-profile", choices=sorted(PROFILES), default="moving",
                       help="gnss_live option set: moving = motion-aware fixed-jump gate for a vehicle rover (default), "
                            "default = bare gnss_live.")
        q.add_argument("--live-arg", action="append", help="Extra argument for gnss_live (repeatable, e.g. --live-arg=--reconnect).")
        q.add_argument("--sample-interval", type=float, default=1.0)
        q.add_argument("--batch-parity", action="store_true", help="Also run gnss_solve on the same data and compare.")

    p = sub.add_parser("run", help="Run one scenario.")
    add_run_args(p)
    p.add_argument("--name", default="run")
    p.add_argument("--fault", action="append", default=[], metavar="KIND@AT+DUR=VALUE",
                   help="Fault, times in stream seconds. Kinds: " + ", ".join(FAULT_KINDS))
    p.add_argument("--loops", type=int, default=1, help="Repeat the window N times with time-shifted data (0 = until --max-wall).")
    p.add_argument("--loop-gap", type=float, default=60.0, help="Silent gap (stream s) between repeats of the window.")
    p.add_argument("--max-wall", type=float, default=0.0, help="Wall-clock cap in seconds (soak).")
    p.add_argument("--reconnect", action="store_true", help="Run gnss_live with --reconnect (needs a build that has it).")
    p.add_argument("--reconnect-delay-ms", type=int, default=200,
                   help="Delay between gnss_live reconnect attempts when --reconnect is set (default 200).")
    p.add_argument("--supervise", action="store_true", help="Restart gnss_live if it exits before the stream ends.")
    p.add_argument("--restart-delay", type=float, default=1.0)
    p.add_argument("--compact", action="store_true", help="Keep only time/status per output epoch (long soaks).")
    p.add_argument("--json", action="store_true")
    p.add_argument("--allow-failure", action="store_true")
    p.set_defaults(func=cmd_run)

    p = sub.add_parser("suite", help="Run a named scenario set.")
    add_run_args(p)
    p.add_argument("--preset", default="faults", choices=("smoke", "faults", "faults_ntrip", "transports"))
    p.add_argument("--only", action="append", help="Only run the named scenario (repeatable).")
    p.add_argument("--jobs", type=int, default=1, help="Scenarios to run in parallel.")
    p.add_argument("--reconnect", action="store_true", help="Run every scenario with gnss_live --reconnect (needs a build that has it).")
    p.set_defaults(func=cmd_suite)

    p = sub.add_parser("report", help="Rebuild suite_summary.json and print the table for a finished suite directory.")
    p.add_argument("--suite-dir", required=True)
    p.set_defaults(func=cmd_report)

    p = sub.add_parser("batch", help="Run gnss_solve on the prepared source RINEX (parity baseline).")
    p.add_argument("--prepared", required=True)
    p.add_argument("--out", default=None)
    p.add_argument("--live-profile", choices=sorted(PROFILES), default="moving")
    p.set_defaults(func=cmd_batch)

    p = sub.add_parser("parity", help="Compare a run's solution.*.pos with the batch solution.")
    p.add_argument("--prepared", required=True)
    p.add_argument("--result-dir", required=True)
    p.add_argument("--live-profile", choices=sorted(PROFILES), default="moving")
    p.set_defaults(func=cmd_parity)
    return parser


def main(argv: list[str] | None = None) -> int:
    args = build_parser().parse_args(argv)
    return int(args.func(args))


if __name__ == "__main__":
    sys.exit(main())
