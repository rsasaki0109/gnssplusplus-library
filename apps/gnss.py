#!/usr/bin/env python3
"""Cross-platform dispatcher for the libgnss++ command suite."""

from __future__ import annotations

import glob
import os
import subprocess
import sys


ROOT_DIR = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
APPS_DIR = os.path.dirname(os.path.abspath(__file__))
EXE_SUFFIX = ".exe" if os.name == "nt" else ""
BUILD_CONFIGS = ("Release", "RelWithDebInfo", "Debug", "MinSizeRel")

COMMANDS = {
    "solve": {
        "kind": "binary",
        "target": "gnss_solve",
        "summary": "Batch RTK post-processing from rover/base/nav RINEX files.",
    },
    "rinex-info": {
        "kind": "binary",
        "target": "gnss_rinex_info",
        "summary": "Inspect RINEX headers and optionally count epochs or ephemerides.",
    },
    "stream": {
        "kind": "binary",
        "target": "gnss_stream",
        "summary": "Read and relay RTCM from a file or NTRIP source with optional decode summaries.",
    },
    "ubx-info": {
        "kind": "binary",
        "target": "gnss_ubx_info",
        "summary": "Inspect UBX NAV/RAWX logs and optionally export RAWX epochs to RINEX.",
    },
    "convert": {
        "kind": "binary",
        "target": "gnss_convert",
        "summary": "Convert RTCM or UBX input into simple RINEX observation/navigation files.",
    },
    "replay": {
        "kind": "binary",
        "target": "gnss_replay",
        "summary": "Replay rover/base observations from RINEX, UBX, or RTCM through the RTK solver.",
    },
    "live": {
        "kind": "binary",
        "target": "gnss_live",
        "summary": "Continuously solve dual RTCM rover/base streams into a live RTK solution file.",
    },
    "rcv": {
        "kind": "python",
        "target": os.path.join(APPS_DIR, "gnss_rcv.py"),
        "summary": "Run the live solver from an rtkrcv-style config file and emit status snapshots.",
    },
    "stats": {
        "kind": "python",
        "target": os.path.join(ROOT_DIR, "tools", "rtk_stats.py"),
        "summary": "Show text statistics for a .pos solution file.",
    },
    "compare": {
        "kind": "python",
        "target": os.path.join(ROOT_DIR, "tools", "compare_rtklib.py"),
        "summary": "Compare libgnss++ output against RTKLIB output.",
    },
    "plot": {
        "kind": "python",
        "target": os.path.join(ROOT_DIR, "tools", "plot_rtk.py"),
        "summary": "Generate ENU/time-series plots for one or two .pos files.",
    },
    "trackplot": {
        "kind": "python",
        "target": os.path.join(ROOT_DIR, "tools", "plot_trajectory.py"),
        "summary": "Generate a 2D trajectory comparison plot with status colors.",
    },
    "rtklib2pos": {
        "kind": "python",
        "target": os.path.join(ROOT_DIR, "scripts", "convert_rtklib_pos.py"),
        "summary": "Convert raw RTKLIB .pos output into libgnss++ .pos format.",
    },
    "pos2kml": {
        "kind": "python",
        "target": os.path.join(APPS_DIR, "gnss_pos2kml"),
        "summary": "Convert libgnss++ or RTKLIB .pos output into KML.",
    },
    "driving-compare": {
        "kind": "python",
        "target": os.path.join(ROOT_DIR, "scripts", "generate_driving_comparison.py"),
        "summary": "Generate the UrbanNav/Odaiba comparison figure from solution files.",
    },
    "scorecard": {
        "kind": "python",
        "target": os.path.join(ROOT_DIR, "scripts", "generate_odaiba_scorecard.py"),
        "summary": "Generate the Odaiba benchmark scorecard image.",
    },
    "odaiba-benchmark": {
        "kind": "python",
        "target": os.path.join(APPS_DIR, "gnss_odaiba_benchmark.py"),
        "summary": "Run the full libgnss++ vs RTKLIB Odaiba benchmark pipeline.",
    },
    "odaiba-scan": {
        "kind": "python",
        "target": os.path.join(APPS_DIR, "gnss_odaiba_scan.py"),
        "summary": "Scan Odaiba in epoch windows and report reference-based metrics.",
    },
}


def usage() -> str:
    lines = [
        "Usage: gnss <command> [args...]",
        "",
        "Commands:",
    ]
    for name in sorted(COMMANDS):
        lines.append(f"  {name:<10} {COMMANDS[name]['summary']}")
    lines.extend(
        [
            "",
            "Examples:",
            "  python3 apps/gnss.py solve --data-dir data/driving --out output/rtk_solution.pos",
            "  python3 apps/gnss.py stream --input ntrip://user:pass@caster:2101/MOUNT --limit 10",
            "  python3 apps/gnss.py ubx-info --input logs/session.ubx --decode-observations",
            "  python3 apps/gnss.py convert --format ubx --input logs/session.ubx --obs-out output/session.obs",
            "  python3 apps/gnss.py replay --rover-rinex data/rover_kinematic.obs --base-rinex data/base_kinematic.obs --nav-rinex data/navigation_kinematic.nav --out output/replay.pos",
            "  python3 apps/gnss.py live --rover-rtcm rover.rtcm3 --base-rtcm base.rtcm3 --nav-rinex nav.rnx",
            "  python3 apps/gnss.py rcv start --config configs/live.conf --status-out output/receiver_status.json",
            "  python3 apps/gnss.py rcv status --status-out output/receiver_status.json --wait-seconds 5",
            "  python3 apps/gnss.py stats output/rtk_solution.pos",
            "  python3 apps/gnss.py compare output/rtk_solution.pos output/driving_rtklib_rtk.pos",
            "  python3 apps/gnss.py odaiba-benchmark --rtklib-bin /path/to/rnx2rtkp",
            "  python3 apps/gnss.py odaiba-scan --glonass-ar autocal --window-size 1000",
            "",
            "On Windows, use: py apps\\gnss.py <command> ...",
        ]
    )
    return "\n".join(lines)


def find_binary(target_name: str) -> str | None:
    filename = target_name + EXE_SUFFIX
    candidates = [
        os.path.join(APPS_DIR, filename),
        os.path.join(ROOT_DIR, "build", "apps", filename),
    ]
    for config in BUILD_CONFIGS:
        candidates.append(os.path.join(ROOT_DIR, "build", "apps", config, filename))
        candidates.append(os.path.join(ROOT_DIR, "build", config, "apps", filename))
        candidates.append(os.path.join(ROOT_DIR, "build", config, filename))

    for candidate in candidates:
        if os.path.isfile(candidate):
            return candidate

    recursive_hits = sorted(
        path for path in glob.glob(os.path.join(ROOT_DIR, "build", "**", filename), recursive=True)
        if os.path.isfile(path)
    )
    return recursive_hits[0] if recursive_hits else None


def run_python(target: str, command_name: str, args: list[str]) -> int:
    env = os.environ.copy()
    env["GNSS_CLI_NAME"] = f"gnss {command_name}"
    result = subprocess.run([sys.executable, target, *args], env=env)
    return result.returncode


def run_binary(target_name: str, args: list[str]) -> int:
    binary = find_binary(target_name)
    if binary is None:
        print(
            f"Error: built binary not found for {target_name}. "
            f"Run `cmake --build build --target {target_name}` first.",
            file=sys.stderr,
        )
        return 1
    result = subprocess.run([binary, *args])
    return result.returncode


def main() -> int:
    if len(sys.argv) < 2 or sys.argv[1] in ("-h", "--help", "help"):
        print(usage())
        return 0 if len(sys.argv) >= 2 else 1

    command_name = sys.argv[1]
    command = COMMANDS.get(command_name)
    if command is None:
        print(f"Error: unknown command `{command_name}`.\n", file=sys.stderr)
        print(usage(), file=sys.stderr)
        return 1

    args = sys.argv[2:]
    if command["kind"] == "python":
        return run_python(command["target"], command_name, args)
    return run_binary(command["target"], args)


if __name__ == "__main__":
    raise SystemExit(main())
