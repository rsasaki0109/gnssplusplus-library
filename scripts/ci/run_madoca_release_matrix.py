#!/usr/bin/env python3
"""Run the MADOCALIB/native MADOCA release-gate matrix."""

from __future__ import annotations

import argparse
import json
import math
import os
import subprocess
import sys
import time
from dataclasses import dataclass
from pathlib import Path
from typing import Any, Iterable, Sequence


ROOT_DIR = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(ROOT_DIR / "scripts" / "analysis"))

import madoca_solution_diff as solution_diff  # noqa: E402


SCHEMA_VERSION = "madoca_release_matrix.v1"
BASELINE_SCHEMA_VERSION = "madoca_release_baseline.v1"
COMMAND_SCHEMA_VERSION = "madoca_release_commands.v1"
PROFILE_NAMES = ("ppp", "pppar", "pppar-ion")
TRAJECTORY_RELATIVE_MARGIN = 0.05
TRAJECTORY_ABSOLUTE_MARGIN_M = 0.01
RUNTIME_MULTIPLIER = 2.0
NATIVE_ROW_FIELDS = (
    "madoca_l6d_constraint_epochs",
    "madoca_l6d_constraint_rows",
    "madoca_l6d_constraint_position_gate_epochs",
)


@dataclass(frozen=True)
class Station:
    key: str
    observation_name: str
    reference_ecef_m: tuple[float, float, float]


STATIONS = {
    "mizu": Station(
        "mizu",
        "MIZU00JPN_R_20250910000_01D_30S_MO.rnx",
        (-3857167.6484, 3108694.9138, 4004041.6876),
    ),
    "alic": Station(
        "alic",
        "ALIC00AUS_R_20250910000_01D_30S_MO.rnx",
        (-4052052.7249, 4212835.9727, -2545104.5766),
    ),
}


def case_key(station: str, profile: str, hours: int) -> str:
    return f"{station}.{profile}.{hours}h"


def hourly_letters(hours: int) -> str:
    if hours < 1 or hours > 24:
        raise ValueError("hours must be in the range 1..24")
    return "".join(chr(ord("A") + index) for index in range(hours))


def end_time(hours: int) -> str:
    hourly_letters(hours)
    return f"2025/04/01 {hours - 1:02d}:59:30"


def repeated_option(option: str, paths: Iterable[Path]) -> list[str]:
    result: list[str] = []
    for path in paths:
        result.extend((option, str(path)))
    return result


def l6_paths(
    data_root: Path,
    hours: int,
    prns: Sequence[int],
    *,
    ionosphere: bool,
) -> list[Path]:
    tree = "l6" if ionosphere else "l6_is-qzss-mdc-004"
    day_root = data_root / tree / "2025" / "091"
    return [
        day_root / f"2025091{letter}.{prn}.l6"
        for letter in hourly_letters(hours)
        for prn in prns
    ]


def common_paths(data_root: Path, station: Station) -> tuple[Path, Path, Path]:
    return (
        data_root / "rinex" / station.observation_name,
        data_root / "rinex" / "BRDM00DLR_S_20250910000_01D_MN.rnx",
        data_root / "igs20.atx",
    )


def bridge_command(
    binary: Path,
    data_root: Path,
    station: Station,
    profile: str,
    hours: int,
    output_pos: Path,
    summary_json: Path,
) -> list[str]:
    if profile not in PROFILE_NAMES:
        raise ValueError(f"unknown profile: {profile}")
    obs, nav, antex = common_paths(data_root, station)
    command = [
        str(binary),
        "--madocalib-bridge",
        "--madocalib-profile",
        profile,
        "--obs",
        str(obs),
        "--nav",
        str(nav),
        "--antex",
        str(antex),
        "--madocalib-start",
        "2025/04/01 00:00:00",
        "--madocalib-end",
        end_time(hours),
        "--madocalib-ti",
        "30",
        "--out",
        str(output_pos),
        "--summary-json",
        str(summary_json),
        "--quiet",
    ]
    command += repeated_option(
        "--madocalib-l6",
        l6_paths(data_root, hours, (204, 206), ionosphere=False),
    )
    if profile == "pppar-ion":
        command += repeated_option(
            "--madocalib-mdciono",
            l6_paths(data_root, hours, (200, 201), ionosphere=True),
        )
    return command


def native_command(
    binary: Path,
    data_root: Path,
    station: Station,
    profile: str,
    hours: int,
    output_pos: Path,
    summary_json: Path,
) -> list[str]:
    if profile not in PROFILE_NAMES:
        raise ValueError(f"unknown profile: {profile}")
    obs, nav, antex = common_paths(data_root, station)
    command = [
        str(binary),
        "--static" if profile == "ppp" else "--kinematic",
        "--obs",
        str(obs),
        "--nav",
        str(nav),
        "--antex",
        str(antex),
        "--max-epochs",
        str(hours * 120),
        "--emit-epoch-time",
        "--out",
        str(output_pos),
        "--summary-json",
        str(summary_json),
        "--quiet",
    ]
    command += repeated_option(
        "--madoca-l6",
        l6_paths(data_root, hours, (204, 206), ionosphere=False),
    )
    if profile != "ppp":
        command += [
            "--enable-ar",
            "--ar-method",
            "per-freq",
            "--ar-ratio-threshold",
            "1.5",
            "--convergence-policy",
            "local-enu",
            "--convergence-threshold-horizontal",
            "0.75",
            "--convergence-threshold-vertical",
            "1.25",
        ]
    if profile == "pppar-ion":
        command += repeated_option(
            "--madoca-l6d",
            l6_paths(data_root, hours, (200, 201), ionosphere=True),
        )
    return command


def require_inputs(command: Sequence[str]) -> None:
    path_options = {
        "--obs",
        "--nav",
        "--antex",
        "--madocalib-l6",
        "--madocalib-mdciono",
        "--madoca-l6",
        "--madoca-l6d",
    }
    for index, token in enumerate(command[:-1]):
        if token in path_options:
            path = Path(command[index + 1])
            if not path.is_file():
                raise FileNotFoundError(f"{token}: missing input: {path}")


def run_command(command: Sequence[str], stdout_path: Path, stderr_path: Path) -> float:
    require_inputs(command)
    started = time.perf_counter()
    with (
        stdout_path.open("w", encoding="utf-8") as stdout_handle,
        stderr_path.open("w", encoding="utf-8") as stderr_handle,
    ):
        completed = subprocess.run(
            command,
            cwd=ROOT_DIR,
            stdout=stdout_handle,
            stderr=stderr_handle,
            check=False,
        )
    elapsed = time.perf_counter() - started
    if completed.returncode != 0:
        raise RuntimeError(
            f"command failed with exit {completed.returncode}; see {stderr_path}"
        )
    return elapsed


def status_summary(matches: Sequence[solution_diff.MatchedPair]) -> dict[str, Any]:
    pair_counts: dict[str, int] = {}
    wrong_fixes = 0
    missed_fixes = 0
    for pair in matches:
        pair_key = f"{pair.base.status}->{pair.candidate.status}"
        pair_counts[pair_key] = pair_counts.get(pair_key, 0) + 1
        if pair.base.status != 1 and pair.candidate.status == 6:
            wrong_fixes += 1
        if pair.base.status == 1 and pair.candidate.status != 6:
            missed_fixes += 1
    return {
        "pair_counts": dict(sorted(pair_counts.items())),
        "wrong_fixes": wrong_fixes,
        "missed_fixes": missed_fixes,
    }


def hourly_boundary_summary(
    matches: Sequence[solution_diff.MatchedPair],
    window_seconds: float = 120.0,
) -> dict[str, Any]:
    selected: list[tuple[float, solution_diff.MatchedPair]] = []
    for pair in matches:
        seconds_from_day_start = pair.base.tow - 172800.0
        if seconds_from_day_start <= 0.0:
            continue
        remainder = seconds_from_day_start % 3600.0
        distance = min(remainder, 3600.0 - remainder)
        if distance <= window_seconds:
            delta_3d = math.sqrt(sum(value * value for value in pair.delta_enu_m))
            selected.append((delta_3d, pair))
    if not selected:
        return {
            "window_seconds": window_seconds,
            "matched_epochs": 0,
            "max_delta_3d_m": None,
            "max_week": None,
            "max_tow": None,
        }
    delta, pair = max(selected, key=lambda item: item[0])
    return {
        "window_seconds": window_seconds,
        "matched_epochs": len(selected),
        "max_delta_3d_m": delta,
        "max_week": pair.base.week,
        "max_tow": pair.base.tow,
    }


def make_reference(station: Station) -> solution_diff.ReferenceFrame:
    lat, lon, height = solution_diff.ecef_to_llh(*station.reference_ecef_m)
    return solution_diff.ReferenceFrame(*station.reference_ecef_m, lat, lon, height)


def evaluate_baseline(measured: dict[str, Any], baseline: dict[str, Any]) -> list[str]:
    failures: list[str] = []

    for field in (
        "base_status_counts",
        "candidate_status_counts",
        "status_pair_counts",
        "native_row_counts",
    ):
        if measured.get(field) != baseline.get(field):
            failures.append(
                f"{field}: measured={measured.get(field)!r} "
                f"expected={baseline.get(field)!r}"
            )
    if measured.get("matched_epochs", 0) < baseline.get("matched_epochs", 0):
        failures.append(
            f"matched_epochs: measured={measured.get('matched_epochs')!r} "
            f"minimum={baseline.get('matched_epochs')!r}"
        )
    for field in (
        "base_only_epochs",
        "candidate_only_epochs",
        "wrong_fixes",
        "missed_fixes",
        "delta_rms_3d_m",
        "delta_max_3d_m",
        "boundary_max_delta_3d_m",
        "native_runtime_s",
    ):
        expected = baseline.get(field)
        measured_value = measured.get(field)
        if (
            expected is not None
            and measured_value is not None
            and measured_value > expected
        ):
            failures.append(
                f"{field}: measured={measured_value!r} maximum={expected!r}"
            )
    return failures


def trajectory_ceiling(value: float | None) -> float | None:
    if value is None:
        return None
    return max(
        value * (1.0 + TRAJECTORY_RELATIVE_MARGIN),
        value + TRAJECTORY_ABSOLUTE_MARGIN_M,
    )


def measured_baseline(case: dict[str, Any]) -> dict[str, Any]:
    fields = (
        "matched_epochs",
        "base_only_epochs",
        "candidate_only_epochs",
        "base_status_counts",
        "candidate_status_counts",
        "status_pair_counts",
        "native_row_counts",
        "wrong_fixes",
        "missed_fixes",
    )
    baseline = {field: case[field] for field in fields}
    for field in (
        "delta_rms_3d_m",
        "delta_max_3d_m",
        "boundary_max_delta_3d_m",
    ):
        baseline[field] = trajectory_ceiling(case[field])
    baseline["native_runtime_s"] = case["native_runtime_s"] * RUNTIME_MULTIPLIER
    return baseline


def append_github_summary(payload: dict[str, Any]) -> None:
    summary_path = os.environ.get("GITHUB_STEP_SUMMARY")
    if not summary_path:
        return
    with Path(summary_path).open("a", encoding="utf-8") as handle:
        handle.write("### MADOCA release matrix\n\n")
        handle.write(
            f"- Result schema: `{payload['schema_version']}`\n"
            f"- Command schema: `{payload['command_schema_version']}`\n"
            f"- Baseline schema: `{payload['baseline_schema_version']}`\n"
            f"- Status: `{payload['status']}`\n\n"
        )
        handle.write(
            "| Case | Matched | Wrong Fix | Missed Fix | RMS 3D (m) | "
            "Max 3D (m) | Native runtime (s) | Commands |\n"
        )
        handle.write("| --- | ---: | ---: | ---: | ---: | ---: | ---: | --- |\n")
        for case in payload["cases"]:
            handle.write(
                f"| `{case['key']}` | {case['matched_epochs']} | "
                f"{case['wrong_fixes']} | {case['missed_fixes']} | "
                f"{case['delta_rms_3d_m']:.6f} | "
                f"{case['delta_max_3d_m']:.6f} | "
                f"{case['native_runtime_s']:.2f} | "
                f"`{case['artifact_dir']}/commands.json` |\n"
            )
        handle.write("\n")


def run_case(
    binary: Path,
    data_root: Path,
    output_dir: Path,
    station: Station,
    profile: str,
    hours: int,
) -> dict[str, Any]:
    key = case_key(station.key, profile, hours)
    case_dir = output_dir / key
    case_dir.mkdir(parents=True, exist_ok=True)
    bridge_pos = case_dir / "bridge.pos"
    native_pos = case_dir / "native.pos"
    bridge_summary = case_dir / "bridge_summary.json"
    native_summary = case_dir / "native_summary.json"
    bridge = bridge_command(
        binary, data_root, station, profile, hours, bridge_pos, bridge_summary
    )
    native = native_command(
        binary, data_root, station, profile, hours, native_pos, native_summary
    )
    (case_dir / "commands.json").write_text(
        json.dumps(
            {
                "schema_version": COMMAND_SCHEMA_VERSION,
                "bridge": bridge,
                "native": native,
            },
            indent=2,
        )
        + "\n",
        encoding="utf-8",
    )
    bridge_runtime = run_command(
        bridge, case_dir / "bridge.stdout.log", case_dir / "bridge.stderr.log"
    )
    native_runtime = run_command(
        native, case_dir / "native.stdout.log", case_dir / "native.stderr.log"
    )
    report, matches = solution_diff.build_report(
        bridge_pos,
        native_pos,
        make_reference(station),
        tolerance_s=0.001,
        tail_seconds=1800.0,
        base_label="bridge",
        candidate_label="native",
    )
    (case_dir / "solution_diff.json").write_text(
        json.dumps(report, indent=2, sort_keys=True) + "\n",
        encoding="utf-8",
    )
    solution_diff.write_matches_csv(case_dir / "solution_matches.csv", matches)
    statuses = status_summary(matches)
    boundary = hourly_boundary_summary(matches)
    comparison = report["comparison"]
    native_summary_payload = json.loads(native_summary.read_text(encoding="utf-8"))
    return {
        "key": key,
        "station": station.key,
        "profile": profile,
        "hours": hours,
        "bridge_runtime_s": bridge_runtime,
        "native_runtime_s": native_runtime,
        "matched_epochs": comparison["matched_epochs"],
        "base_only_epochs": report["base_only_epochs"],
        "candidate_only_epochs": report["candidate_only_epochs"],
        "base_status_counts": report["base"]["status_counts"],
        "candidate_status_counts": report["candidate"]["status_counts"],
        "status_pair_counts": statuses["pair_counts"],
        "native_row_counts": {
            field: native_summary_payload.get(field, 0) for field in NATIVE_ROW_FIELDS
        },
        "wrong_fixes": statuses["wrong_fixes"],
        "missed_fixes": statuses["missed_fixes"],
        "delta_rms_3d_m": comparison["delta_rms_3d_m"],
        "delta_max_3d_m": report["events"]["max_delta_3d"]["delta_3d_m"],
        "boundary_max_delta_3d_m": boundary["max_delta_3d_m"],
        "boundary": boundary,
        "artifact_dir": str(case_dir),
    }


def parse_csv_values(value: str) -> list[str]:
    return [item.strip().lower() for item in value.split(",") if item.strip()]


def parse_hours(value: str) -> list[int]:
    return [int(item) for item in parse_csv_values(value)]


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--binary", type=Path, required=True)
    parser.add_argument("--data-root", type=Path, required=True)
    parser.add_argument("--output-dir", type=Path, required=True)
    parser.add_argument("--stations", default="mizu,alic")
    parser.add_argument("--profiles", default="ppp,pppar,pppar-ion")
    parser.add_argument("--hours", default="1,6")
    parser.add_argument("--baseline", type=Path)
    parser.add_argument("--write-measured-baseline", type=Path)
    return parser.parse_args()


def main() -> int:
    args = parse_args()
    binary = args.binary.resolve()
    data_root = args.data_root.resolve()
    output_dir = args.output_dir.resolve()
    output_dir.mkdir(parents=True, exist_ok=True)
    if not binary.is_file():
        raise FileNotFoundError(f"missing gnss_ppp binary: {binary}")

    station_names = parse_csv_values(args.stations)
    profile_names = parse_csv_values(args.profiles)
    hours_values = parse_hours(args.hours)
    for station_name in station_names:
        if station_name not in STATIONS:
            raise ValueError(f"unknown station: {station_name}")
    for profile_name in profile_names:
        if profile_name not in PROFILE_NAMES:
            raise ValueError(f"unknown profile: {profile_name}")

    baseline_payload: dict[str, Any] | None = None
    if args.baseline is not None:
        baseline_payload = json.loads(args.baseline.read_text(encoding="utf-8"))
        if baseline_payload.get("schema_version") != BASELINE_SCHEMA_VERSION:
            raise ValueError("unsupported MADOCA release baseline schema")

    cases: list[dict[str, Any]] = []
    failures: list[str] = []
    for station_name in station_names:
        for profile_name in profile_names:
            for hours in hours_values:
                case = run_case(
                    binary,
                    data_root,
                    output_dir,
                    STATIONS[station_name],
                    profile_name,
                    hours,
                )
                if baseline_payload is not None:
                    expected = baseline_payload.get("cases", {}).get(case["key"])
                    case_failures = (
                        ["missing baseline"]
                        if expected is None
                        else evaluate_baseline(case, expected)
                    )
                    case["baseline_failures"] = case_failures
                    failures.extend(
                        f"{case['key']}: {failure}" for failure in case_failures
                    )
                cases.append(case)
                print(
                    f"{case['key']}: matched={case['matched_epochs']} "
                    f"rms={case['delta_rms_3d_m']:.6f} "
                    f"max={case['delta_max_3d_m']:.6f} "
                    f"wrong_fix={case['wrong_fixes']} "
                    f"missed_fix={case['missed_fixes']}"
                )

    payload = {
        "schema_version": SCHEMA_VERSION,
        "command_schema_version": COMMAND_SCHEMA_VERSION,
        "baseline_schema_version": BASELINE_SCHEMA_VERSION,
        "status": "failed"
        if failures
        else ("passed" if baseline_payload else "measured"),
        "binary": str(binary),
        "data_root": str(data_root),
        "cases": cases,
        "failures": failures,
    }
    (output_dir / "summary.json").write_text(
        json.dumps(payload, indent=2, sort_keys=True) + "\n",
        encoding="utf-8",
    )
    append_github_summary(payload)
    if args.write_measured_baseline is not None:
        baseline_cases: dict[str, Any] = {}
        if args.write_measured_baseline.is_file():
            existing_baseline = json.loads(
                args.write_measured_baseline.read_text(encoding="utf-8")
            )
            if existing_baseline.get("schema_version") != BASELINE_SCHEMA_VERSION:
                raise ValueError("unsupported existing MADOCA release baseline schema")
            baseline_cases.update(existing_baseline.get("cases", {}))
        for baseline_case in baseline_cases.values():
            baseline_case.pop("bridge_runtime_s", None)
        baseline_cases.update({case["key"]: measured_baseline(case) for case in cases})
        measured = {
            "schema_version": BASELINE_SCHEMA_VERSION,
            "cases": baseline_cases,
        }
        args.write_measured_baseline.parent.mkdir(parents=True, exist_ok=True)
        args.write_measured_baseline.write_text(
            json.dumps(measured, indent=2, sort_keys=True) + "\n",
            encoding="utf-8",
        )

    if failures:
        for failure in failures:
            print(f"FAIL: {failure}", file=sys.stderr)
        return 1
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
