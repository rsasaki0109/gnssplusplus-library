#!/usr/bin/env python3
"""Run SPP/FGO/RTK modes on the same RINEX inputs and summarize outputs."""

from __future__ import annotations

import argparse
from dataclasses import dataclass
import json
import math
import os
from pathlib import Path
import subprocess
import sys
import time
from typing import Any

from gnss_runtime import ensure_input_exists, resolve_gnss_command


ROOT_DIR = Path(__file__).resolve().parent.parent

STATUS_NAMES = {
    0: "NONE",
    1: "SPP",
    2: "DGPS",
    3: "FLOAT",
    4: "FIXED",
    5: "PPP_FLOAT",
    6: "PPP_FIXED",
}
FIXED_STATUSES = {4, 6}
FLOAT_STATUSES = {3, 5}


@dataclass(frozen=True)
class PosRecord:
    week: int
    tow: float
    x: float
    y: float
    z: float
    status: int
    satellites: int
    ratio: float | None


@dataclass(frozen=True)
class ModePlan:
    name: str
    pos_path: Path
    summary_path: Path | None
    command: list[str]
    external: bool = False


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(prog=os.environ.get("GNSS_CLI_NAME"))
    parser.add_argument("--obs", type=Path, help="Rover RINEX observation file.")
    parser.add_argument("--nav", type=Path, help="Navigation RINEX file.")
    parser.add_argument("--base", type=Path, help="Optional base RINEX observation file for RTK.")
    parser.add_argument(
        "--out-dir",
        type=Path,
        default=ROOT_DIR / "output/gnss_mode_compare",
        help="Directory for per-mode POS files and summaries.",
    )
    parser.add_argument(
        "--summary-json",
        type=Path,
        help="Output path for the aggregate JSON summary. Defaults to <out-dir>/summary.json.",
    )
    parser.add_argument(
        "--modes",
        default="auto",
        help=(
            "Comma-separated modes: auto, spp, fgo, rtk. Auto runs spp/fgo and "
            "rtk when --base or --rtk-pos is set."
        ),
    )
    parser.add_argument(
        "--max-epochs",
        type=int,
        default=0,
        help="Stop each solver after N epochs. Use 0 for the full input.",
    )
    parser.add_argument(
        "--fgo-skip-epochs",
        type=int,
        default=0,
        help="Skip the first N rover epochs when running FGO.",
    )
    parser.add_argument(
        "--match-tolerance-s",
        type=float,
        default=0.01,
        help="Epoch matching tolerance for pairwise mode deltas.",
    )
    parser.add_argument(
        "--reference-csv",
        type=Path,
        help="Optional PPC-style reference.csv used to compute per-mode reference errors.",
    )
    parser.add_argument(
        "--reference-match-tolerance-s",
        type=float,
        default=0.25,
        help="Epoch matching tolerance for --reference-csv metrics.",
    )
    parser.add_argument(
        "--rtk-mode",
        default="auto",
        choices=("auto", "kinematic", "static", "moving-base"),
        help="Mode passed to gnss solve when RTK is enabled.",
    )
    parser.add_argument(
        "--spp-pos",
        type=Path,
        help="Existing SPP POS file to summarize instead of running gnss spp.",
    )
    parser.add_argument(
        "--fgo-pos",
        type=Path,
        help="Existing FGO POS file to summarize instead of running gnss fgo.",
    )
    parser.add_argument(
        "--fgo-preset",
        choices=(
            "default",
            "real-data",
            "real-data-float",
            "real-data-fixed",
            "tdcp-only",
            "taroz-pc",
        ),
        help="Preset passed to gnss fgo when FGO is run by this tool.",
    )
    parser.add_argument(
        "--fgo-summary-json",
        type=Path,
        help="Existing FGO native summary JSON to include when --fgo-pos is used.",
    )
    parser.add_argument(
        "--rtk-pos",
        type=Path,
        help="Existing RTK POS file to summarize instead of running gnss solve.",
    )
    parser.add_argument(
        "--spp-extra-arg",
        action="append",
        default=[],
        help="Extra single argument passed to gnss spp. Repeat for multiple tokens.",
    )
    parser.add_argument(
        "--fgo-extra-arg",
        action="append",
        default=[],
        help="Extra single argument passed to gnss fgo. Repeat for multiple tokens.",
    )
    parser.add_argument(
        "--rtk-extra-arg",
        action="append",
        default=[],
        help="Extra single argument passed to gnss solve. Repeat for multiple tokens.",
    )
    parser.add_argument(
        "--require-mode-epochs-min",
        action="append",
        default=[],
        metavar="MODE=N",
        help="Fail if MODE has fewer than N solution epochs. Repeatable.",
    )
    parser.add_argument(
        "--require-mode-fix-rate-min",
        action="append",
        default=[],
        metavar="MODE=PCT",
        help="Fail if MODE fix rate is below PCT. Repeatable.",
    )
    parser.add_argument(
        "--require-mode-float-rate-min",
        action="append",
        default=[],
        metavar="MODE=PCT",
        help="Fail if MODE float rate is below PCT. Repeatable.",
    )
    parser.add_argument(
        "--require-pair-common-min",
        action="append",
        default=[],
        metavar="LEFT:RIGHT=N",
        help="Fail if a mode pair has fewer than N common epochs. Repeatable.",
    )
    parser.add_argument(
        "--require-pair-mean-3d-max",
        action="append",
        default=[],
        metavar="LEFT:RIGHT=M",
        help="Fail if pair mean 3D delta exceeds M meters. Repeatable.",
    )
    parser.add_argument(
        "--require-pair-p95-3d-max",
        action="append",
        default=[],
        metavar="LEFT:RIGHT=M",
        help="Fail if pair p95 3D delta exceeds M meters. Repeatable.",
    )
    parser.add_argument(
        "--keep-going",
        action="store_true",
        help="Continue running later modes if one mode exits with a non-zero status.",
    )
    parser.add_argument(
        "--dry-run",
        action="store_true",
        help="Write the planned commands and summary without executing solvers.",
    )
    return parser.parse_args()


def parse_modes(raw_modes: str, has_rtk_input: bool) -> list[str]:
    if raw_modes.strip().lower() == "auto":
        modes = ["spp", "fgo"]
        if has_rtk_input:
            modes.append("rtk")
        return modes

    modes: list[str] = []
    for token in raw_modes.split(","):
        mode = token.strip().lower()
        if not mode:
            continue
        if mode == "auto":
            for auto_mode in parse_modes("auto", has_rtk_input):
                if auto_mode not in modes:
                    modes.append(auto_mode)
            continue
        if mode not in {"spp", "fgo", "rtk"}:
            raise SystemExit(f"Unsupported mode `{mode}`; expected auto, spp, fgo, or rtk")
        if mode == "rtk" and not has_rtk_input:
            raise SystemExit("Mode `rtk` requires --base or --rtk-pos")
        if mode not in modes:
            modes.append(mode)
    if not modes:
        raise SystemExit("--modes did not select any modes")
    return modes


def build_mode_plans(args: argparse.Namespace, gnss_command: list[str]) -> list[ModePlan]:
    out_dir = args.out_dir
    modes = parse_modes(args.modes, args.base is not None or args.rtk_pos is not None)
    plans: list[ModePlan] = []
    for mode in modes:
        pos_path = out_dir / f"{mode}.pos"
        if mode == "spp":
            if args.spp_pos is not None:
                plans.append(ModePlan(mode, args.spp_pos, None, [], external=True))
                continue
            command = [
                *gnss_command,
                "spp",
                "--obs",
                str(args.obs),
                "--nav",
                str(args.nav),
                "--out",
                str(pos_path),
                "--quiet",
                *args.spp_extra_arg,
            ]
            if args.max_epochs > 0:
                command.extend(["--max-epochs", str(args.max_epochs)])
            plans.append(ModePlan(mode, pos_path, None, command))
        elif mode == "fgo":
            fgo_summary_path = out_dir / "fgo_summary.json"
            if args.fgo_pos is not None:
                plans.append(
                    ModePlan(
                        mode,
                        args.fgo_pos,
                        args.fgo_summary_json,
                        [],
                        external=True,
                    )
                )
                continue
            command = [
                *gnss_command,
                "fgo",
                "--obs",
                str(args.obs),
                "--nav",
                str(args.nav),
                "--out",
                str(pos_path),
                "--summary-json",
                str(fgo_summary_path),
                "--quiet",
            ]
            if args.fgo_preset is not None:
                command.extend(["--preset", args.fgo_preset])
                if args.fgo_preset == "taroz-pc":
                    command.extend(["--backend", "gtsam-pc"])
            if args.base is not None:
                command.extend(["--base", str(args.base)])
            command.extend(args.fgo_extra_arg)
            if args.fgo_skip_epochs > 0:
                command.extend(["--skip-epochs", str(args.fgo_skip_epochs)])
            if args.max_epochs > 0:
                command.extend(["--max-epochs", str(args.max_epochs)])
            plans.append(ModePlan(mode, pos_path, fgo_summary_path, command))
        elif mode == "rtk":
            if args.rtk_pos is not None:
                plans.append(ModePlan(mode, args.rtk_pos, None, [], external=True))
                continue
            command = [
                *gnss_command,
                "solve",
                "--rover",
                str(args.obs),
                "--base",
                str(args.base),
                "--nav",
                str(args.nav),
                "--out",
                str(pos_path),
                "--mode",
                args.rtk_mode,
                "--no-kml",
                *args.rtk_extra_arg,
            ]
            if args.max_epochs > 0:
                command.extend(["--max-epochs", str(args.max_epochs)])
            plans.append(ModePlan(mode, pos_path, None, command))
    return plans


def read_pos_records(path: Path) -> list[PosRecord]:
    records: list[PosRecord] = []
    with path.open(encoding="ascii", errors="ignore") as handle:
        for line_number, raw_line in enumerate(handle, start=1):
            line = raw_line.strip()
            if not line or line.startswith("%"):
                continue
            parts = line.split()
            if len(parts) < 10:
                raise ValueError(f"{path}:{line_number}: expected at least 10 POS columns")
            ratio = None
            if len(parts) > 11:
                try:
                    parsed_ratio = float(parts[11])
                except ValueError:
                    parsed_ratio = math.nan
                if math.isfinite(parsed_ratio):
                    ratio = parsed_ratio
            records.append(
                PosRecord(
                    week=int(float(parts[0])),
                    tow=float(parts[1]),
                    x=float(parts[2]),
                    y=float(parts[3]),
                    z=float(parts[4]),
                    status=int(float(parts[8])),
                    satellites=int(float(parts[9])),
                    ratio=ratio,
                )
            )
    return records


def percentile(values: list[float], percent: float) -> float | None:
    if not values:
        return None
    ordered = sorted(values)
    if len(ordered) == 1:
        return ordered[0]
    position = (len(ordered) - 1) * percent / 100.0
    lower = int(math.floor(position))
    upper = int(math.ceil(position))
    if lower == upper:
        return ordered[lower]
    fraction = position - lower
    return ordered[lower] * (1.0 - fraction) + ordered[upper] * fraction


def rate_percent(numerator: int, denominator: int) -> float | None:
    if denominator <= 0:
        return None
    return 100.0 * numerator / denominator


def summarize_records(records: list[PosRecord]) -> dict[str, Any]:
    status_counts_by_code: dict[str, int] = {}
    status_counts_by_name: dict[str, int] = {}
    for record in records:
        status_counts_by_code[str(record.status)] = status_counts_by_code.get(str(record.status), 0) + 1
        status_name = STATUS_NAMES.get(record.status, f"UNKNOWN_{record.status}")
        status_counts_by_name[status_name] = status_counts_by_name.get(status_name, 0) + 1

    fixed_epochs = sum(1 for record in records if record.status in FIXED_STATUSES)
    float_epochs = sum(1 for record in records if record.status in FLOAT_STATUSES)
    spp_epochs = sum(1 for record in records if record.status == 1)
    ratios = [record.ratio for record in records if record.ratio is not None]
    satellites = [record.satellites for record in records]
    payload: dict[str, Any] = {
        "epochs": len(records),
        "fixed_epochs": fixed_epochs,
        "float_epochs": float_epochs,
        "spp_epochs": spp_epochs,
        "fix_rate_percent": rate_percent(fixed_epochs, len(records)),
        "float_rate_percent": rate_percent(float_epochs, len(records)),
        "spp_rate_percent": rate_percent(spp_epochs, len(records)),
        "status_counts": status_counts_by_name,
        "status_counts_by_code": status_counts_by_code,
        "mean_satellites": sum(satellites) / len(satellites) if satellites else None,
        "mean_ratio": sum(ratios) / len(ratios) if ratios else None,
        "first_epoch": None,
        "last_epoch": None,
    }
    if records:
        first = min(records, key=lambda record: (record.week, record.tow))
        last = max(records, key=lambda record: (record.week, record.tow))
        payload["first_epoch"] = {"week": first.week, "tow": first.tow}
        payload["last_epoch"] = {"week": last.week, "tow": last.tow}
    return payload


def match_records(
    left_records: list[PosRecord],
    right_records: list[PosRecord],
    tolerance_s: float,
) -> list[tuple[PosRecord, PosRecord]]:
    left = sorted(left_records, key=lambda record: (record.week, record.tow))
    right = sorted(right_records, key=lambda record: (record.week, record.tow))
    matches: list[tuple[PosRecord, PosRecord]] = []
    i = 0
    j = 0
    while i < len(left) and j < len(right):
        left_record = left[i]
        right_record = right[j]
        if left_record.week == right_record.week:
            dt = left_record.tow - right_record.tow
            if abs(dt) <= tolerance_s:
                matches.append((left_record, right_record))
                i += 1
                j += 1
            elif dt < 0.0:
                i += 1
            else:
                j += 1
        elif left_record.week < right_record.week:
            i += 1
        else:
            j += 1
    return matches


def summarize_pairwise(
    left_name: str,
    left_records: list[PosRecord],
    right_name: str,
    right_records: list[PosRecord],
    tolerance_s: float,
) -> dict[str, Any]:
    matches = match_records(left_records, right_records, tolerance_s)
    deltas = [
        math.dist((left.x, left.y, left.z), (right.x, right.y, right.z))
        for left, right in matches
    ]
    return {
        "left": left_name,
        "right": right_name,
        "common_epochs": len(matches),
        "mean_3d_delta_m": sum(deltas) / len(deltas) if deltas else None,
        "median_3d_delta_m": percentile(deltas, 50.0),
        "p95_3d_delta_m": percentile(deltas, 95.0),
        "max_3d_delta_m": max(deltas) if deltas else None,
    }


def rounded(value: Any) -> Any:
    if isinstance(value, float):
        if not math.isfinite(value):
            return None
        return round(value, 6)
    if isinstance(value, list):
        return [rounded(item) for item in value]
    if isinstance(value, dict):
        return {key: rounded(item) for key, item in value.items()}
    return value


def run_plan(plan: ModePlan, keep_going: bool) -> tuple[subprocess.CompletedProcess[str], float]:
    print("+", " ".join(plan.command), flush=True)
    start_time = time.perf_counter()
    completed = subprocess.run(
        plan.command,
        cwd=ROOT_DIR,
        text=True,
        capture_output=True,
        check=False,
    )
    solver_wall_time_s = time.perf_counter() - start_time
    if completed.stdout:
        print(completed.stdout, end="")
    if completed.stderr:
        print(completed.stderr, end="", file=sys.stderr)
    if completed.returncode != 0 and not keep_going:
        raise SystemExit(f"{plan.name} failed with exit code {completed.returncode}")
    return completed, solver_wall_time_s


def load_json_if_present(path: Path | None) -> dict[str, Any] | None:
    if path is None or not path.exists():
        return None
    payload = json.loads(path.read_text(encoding="utf-8"))
    return payload if isinstance(payload, dict) else None


def load_reference_epochs(reference_csv: Path) -> list[Any]:
    scripts_dir = ROOT_DIR / "scripts"
    if str(scripts_dir) not in sys.path:
        sys.path.insert(0, str(scripts_dir))
    import generate_driving_comparison as comparison

    return comparison.read_reference_csv(reference_csv)


def summarize_reference_metrics(
    reference_epochs: list[Any],
    pos_path: Path,
    mode_name: str,
    tolerance_s: float,
    solver_wall_time_s: float | None,
) -> dict[str, Any]:
    scripts_dir = ROOT_DIR / "scripts"
    if str(scripts_dir) not in sys.path:
        sys.path.insert(0, str(scripts_dir))
    import generate_driving_comparison as comparison
    import gnss_ppc_metrics as ppc_metrics

    solution_epochs = comparison.read_libgnss_pos(pos_path)
    try:
        return ppc_metrics.summarize_solution_epochs(
            reference_epochs,
            solution_epochs,
            fixed_status=4,
            label=mode_name,
            match_tolerance_s=tolerance_s,
            solver_wall_time_s=solver_wall_time_s,
        )
    except SystemExit as exc:
        return {"error": str(exc)}


def parse_requirement_spec(raw_spec: str) -> tuple[str, float]:
    if "=" not in raw_spec:
        raise SystemExit(f"Requirement `{raw_spec}` must use NAME=VALUE syntax")
    key, raw_value = raw_spec.split("=", 1)
    key = key.strip().lower()
    if not key:
        raise SystemExit(f"Requirement `{raw_spec}` has an empty name")
    try:
        value = float(raw_value)
    except ValueError as exc:
        raise SystemExit(f"Requirement `{raw_spec}` has a non-numeric value") from exc
    if not math.isfinite(value):
        raise SystemExit(f"Requirement `{raw_spec}` must be finite")
    return key, value


def parse_pair_requirement_spec(raw_spec: str) -> tuple[str, str, float]:
    pair_name, value = parse_requirement_spec(raw_spec)
    if ":" not in pair_name:
        raise SystemExit(f"Requirement `{raw_spec}` must use LEFT:RIGHT=VALUE syntax")
    left, right = (token.strip().lower() for token in pair_name.split(":", 1))
    if not left or not right:
        raise SystemExit(f"Requirement `{raw_spec}` has an empty pair side")
    return left, right, value


def find_pair_summary(payload: dict[str, Any], left: str, right: str) -> dict[str, Any] | None:
    raw_pairwise = payload.get("pairwise")
    if not isinstance(raw_pairwise, list):
        return None
    for raw_entry in raw_pairwise:
        if not isinstance(raw_entry, dict):
            continue
        entry_left = str(raw_entry.get("left", "")).lower()
        entry_right = str(raw_entry.get("right", "")).lower()
        if (entry_left == left and entry_right == right) or (
            entry_left == right and entry_right == left
        ):
            return raw_entry
    return None


def evaluate_requirements(payload: dict[str, Any], args: argparse.Namespace) -> dict[str, Any]:
    failures: list[str] = []
    raw_modes = payload.get("modes")
    modes = raw_modes if isinstance(raw_modes, dict) else {}

    def mode_summary(mode: str, raw_spec: str) -> dict[str, Any] | None:
        raw_entry = modes.get(mode)
        if not isinstance(raw_entry, dict):
            failures.append(f"{raw_spec}: mode `{mode}` is missing")
            return None
        raw_summary = raw_entry.get("summary")
        if not isinstance(raw_summary, dict):
            failures.append(f"{raw_spec}: mode `{mode}` has no POS summary")
            return None
        return raw_summary

    for raw_spec in args.require_mode_epochs_min:
        mode, threshold = parse_requirement_spec(raw_spec)
        summary = mode_summary(mode, raw_spec)
        if summary is None:
            continue
        epochs = int(summary.get("epochs", 0))
        if epochs < threshold:
            failures.append(f"{raw_spec}: {mode} epochs {epochs} < {threshold:g}")

    for raw_spec in args.require_mode_fix_rate_min:
        mode, threshold = parse_requirement_spec(raw_spec)
        summary = mode_summary(mode, raw_spec)
        if summary is None:
            continue
        value = summary.get("fix_rate_percent")
        if value is None or float(value) < threshold:
            shown = "n/a" if value is None else f"{float(value):.6f}"
            failures.append(f"{raw_spec}: {mode} fix rate {shown}% < {threshold:g}%")

    for raw_spec in args.require_mode_float_rate_min:
        mode, threshold = parse_requirement_spec(raw_spec)
        summary = mode_summary(mode, raw_spec)
        if summary is None:
            continue
        value = summary.get("float_rate_percent")
        if value is None or float(value) < threshold:
            shown = "n/a" if value is None else f"{float(value):.6f}"
            failures.append(f"{raw_spec}: {mode} float rate {shown}% < {threshold:g}%")

    for raw_spec in args.require_pair_common_min:
        left, right, threshold = parse_pair_requirement_spec(raw_spec)
        pair = find_pair_summary(payload, left, right)
        if pair is None:
            failures.append(f"{raw_spec}: pair `{left}:{right}` is missing")
            continue
        common_epochs = int(pair.get("common_epochs", 0))
        if common_epochs < threshold:
            failures.append(
                f"{raw_spec}: {left}:{right} common epochs {common_epochs} < {threshold:g}"
            )

    for raw_spec in args.require_pair_mean_3d_max:
        left, right, threshold = parse_pair_requirement_spec(raw_spec)
        pair = find_pair_summary(payload, left, right)
        if pair is None:
            failures.append(f"{raw_spec}: pair `{left}:{right}` is missing")
            continue
        value = pair.get("mean_3d_delta_m")
        if value is None or float(value) > threshold:
            shown = "n/a" if value is None else f"{float(value):.6f}"
            failures.append(f"{raw_spec}: {left}:{right} mean 3D {shown} m > {threshold:g} m")

    for raw_spec in args.require_pair_p95_3d_max:
        left, right, threshold = parse_pair_requirement_spec(raw_spec)
        pair = find_pair_summary(payload, left, right)
        if pair is None:
            failures.append(f"{raw_spec}: pair `{left}:{right}` is missing")
            continue
        value = pair.get("p95_3d_delta_m")
        if value is None or float(value) > threshold:
            shown = "n/a" if value is None else f"{float(value):.6f}"
            failures.append(f"{raw_spec}: {left}:{right} p95 3D {shown} m > {threshold:g} m")

    return {
        "passed": not failures,
        "failures": failures,
    }


def build_payload(
    args: argparse.Namespace,
    plans: list[ModePlan],
    completed_by_mode: dict[str, subprocess.CompletedProcess[str] | None],
    wall_time_by_mode: dict[str, float | None],
) -> dict[str, Any]:
    mode_payloads: dict[str, Any] = {}
    records_by_mode: dict[str, list[PosRecord]] = {}
    reference_epochs = load_reference_epochs(args.reference_csv) if args.reference_csv else None
    for plan in plans:
        completed = completed_by_mode.get(plan.name)
        mode_payload: dict[str, Any] = {
            "command": plan.command,
            "output_pos": str(plan.pos_path),
            "native_summary_json": str(plan.summary_path) if plan.summary_path is not None else None,
            "returncode": completed.returncode if completed is not None else None,
            "solver_wall_time_s": wall_time_by_mode.get(plan.name),
            "status": (
                "external"
                if plan.external
                else ("planned" if completed is None else ("ok" if completed.returncode == 0 else "failed"))
            ),
            "summary": None,
            "native_summary": load_json_if_present(plan.summary_path),
        }
        if plan.pos_path.exists():
            records = read_pos_records(plan.pos_path)
            records_by_mode[plan.name] = records
            mode_payload["summary"] = summarize_records(records)
            if reference_epochs is not None:
                mode_payload["reference_summary"] = summarize_reference_metrics(
                    reference_epochs,
                    plan.pos_path,
                    plan.name,
                    args.reference_match_tolerance_s,
                    wall_time_by_mode.get(plan.name),
                )
        mode_payloads[plan.name] = rounded(mode_payload)

    pairwise: list[dict[str, Any]] = []
    mode_names = [plan.name for plan in plans if plan.name in records_by_mode]
    for left_index, left_name in enumerate(mode_names):
        for right_name in mode_names[left_index + 1:]:
            pairwise.append(
                rounded(
                    summarize_pairwise(
                        left_name,
                        records_by_mode[left_name],
                        right_name,
                        records_by_mode[right_name],
                        args.match_tolerance_s,
                    )
                )
            )

    return {
        "obs": str(args.obs) if args.obs is not None else None,
        "nav": str(args.nav) if args.nav is not None else None,
        "base": str(args.base) if args.base is not None else None,
        "reference_csv": str(args.reference_csv) if args.reference_csv is not None else None,
        "out_dir": str(args.out_dir),
        "max_epochs": args.max_epochs,
        "fgo_skip_epochs": args.fgo_skip_epochs,
        "match_tolerance_s": args.match_tolerance_s,
        "reference_match_tolerance_s": args.reference_match_tolerance_s,
        "modes": mode_payloads,
        "pairwise": pairwise,
    }


def print_text_summary(payload: dict[str, Any]) -> None:
    print("Mode comparison summary")
    modes = payload["modes"]
    assert isinstance(modes, dict)
    for name, raw_entry in modes.items():
        assert isinstance(raw_entry, dict)
        summary = raw_entry.get("summary")
        status = raw_entry.get("status")
        if not isinstance(summary, dict):
            print(f"  {name}: {status}")
            continue
        epochs = int(summary["epochs"])
        fixed = int(summary["fixed_epochs"])
        floats = int(summary["float_epochs"])
        spp = int(summary["spp_epochs"])
        mean_satellites = summary["mean_satellites"]
        mean_satellites_text = (
            f"{float(mean_satellites):.2f}" if mean_satellites is not None else "n/a"
        )
        line = (
            f"  {name}: epochs={epochs} fixed={fixed} float={floats} "
            f"spp={spp} mean_sats={mean_satellites_text}"
        )
        solver_wall_time_s = raw_entry.get("solver_wall_time_s")
        if solver_wall_time_s is not None:
            line += f" wall={float(solver_wall_time_s):.3f}s"
        reference_summary = raw_entry.get("reference_summary")
        if isinstance(reference_summary, dict):
            if "error" in reference_summary:
                line += f" ref_error={reference_summary['error']}"
            else:
                mean_h = reference_summary.get("mean_h_m")
                p95_h = reference_summary.get("p95_h_m")
                score_matched = reference_summary.get("ppc_score_3d_50cm_matched_pct")
                score_ref = reference_summary.get("ppc_score_3d_50cm_ref_pct")
                mean_h_text = f"{float(mean_h):.3f}" if mean_h is not None else "n/a"
                p95_h_text = f"{float(p95_h):.3f}" if p95_h is not None else "n/a"
                score_matched_text = (
                    f"{float(score_matched):.2f}" if score_matched is not None else "n/a"
                )
                score_ref_text = f"{float(score_ref):.2f}" if score_ref is not None else "n/a"
                line += (
                    f" ref_mean_h={mean_h_text}m"
                    f" ref_p95_h={p95_h_text}m"
                    f" ref_3d50_matched={score_matched_text}%"
                    f" ref_3d50_ref={score_ref_text}%"
                )
        print(line)
    pairwise = payload["pairwise"]
    assert isinstance(pairwise, list)
    for entry in pairwise:
        if not isinstance(entry, dict):
            continue
        mean_delta = entry["mean_3d_delta_m"]
        p95_delta = entry["p95_3d_delta_m"]
        mean_text = f"{float(mean_delta):.3f}" if mean_delta is not None else "n/a"
        p95_text = f"{float(p95_delta):.3f}" if p95_delta is not None else "n/a"
        print(
            f"  {entry['left']} vs {entry['right']}: common={entry['common_epochs']} "
            f"mean_3d={mean_text}m p95_3d={p95_text}m"
        )


def main() -> int:
    args = parse_args()
    if args.max_epochs < 0:
        raise SystemExit("--max-epochs must be >= 0")
    if args.fgo_skip_epochs < 0:
        raise SystemExit("--fgo-skip-epochs must be >= 0")
    if args.match_tolerance_s < 0.0:
        raise SystemExit("--match-tolerance-s must be >= 0")
    if args.reference_match_tolerance_s < 0.0:
        raise SystemExit("--reference-match-tolerance-s must be >= 0")

    args.out_dir.mkdir(parents=True, exist_ok=True)
    summary_json = args.summary_json or args.out_dir / "summary.json"
    summary_json.parent.mkdir(parents=True, exist_ok=True)

    gnss_command = resolve_gnss_command(ROOT_DIR)
    plans = build_mode_plans(args, gnss_command)
    needs_solver_inputs = any(not plan.external for plan in plans)
    if needs_solver_inputs:
        if args.obs is None:
            raise SystemExit("--obs is required when running a solver")
        if args.nav is None:
            raise SystemExit("--nav is required when running a solver")
        ensure_input_exists(args.obs, "rover observation file", ROOT_DIR)
        ensure_input_exists(args.nav, "navigation file", ROOT_DIR)
    if args.reference_csv is not None:
        ensure_input_exists(args.reference_csv, "reference CSV", ROOT_DIR)
    for plan in plans:
        if plan.external:
            ensure_input_exists(plan.pos_path, f"{plan.name} POS file", ROOT_DIR)
            if plan.summary_path is not None:
                ensure_input_exists(plan.summary_path, f"{plan.name} summary JSON", ROOT_DIR)
        elif plan.name == "rtk":
            if args.base is None:
                raise SystemExit("Mode `rtk` requires --base when --rtk-pos is not provided")
            ensure_input_exists(args.base, "base observation file", ROOT_DIR)
    completed_by_mode: dict[str, subprocess.CompletedProcess[str] | None] = {}
    wall_time_by_mode: dict[str, float | None] = {}
    if args.dry_run:
        for plan in plans:
            completed_by_mode[plan.name] = None
            wall_time_by_mode[plan.name] = None
            if plan.external:
                print(f"# {plan.name}: using existing POS {plan.pos_path}")
            else:
                print("+", " ".join(plan.command))
    else:
        for plan in plans:
            if plan.external:
                completed_by_mode[plan.name] = None
                wall_time_by_mode[plan.name] = None
                continue
            completed, solver_wall_time_s = run_plan(plan, args.keep_going)
            completed_by_mode[plan.name] = completed
            wall_time_by_mode[plan.name] = solver_wall_time_s

    payload = rounded(build_payload(args, plans, completed_by_mode, wall_time_by_mode))
    payload["requirements"] = evaluate_requirements(payload, args)
    summary_json.write_text(json.dumps(payload, indent=2, sort_keys=True) + "\n", encoding="utf-8")
    print_text_summary(payload)
    print(f"Summary JSON: {summary_json}")
    requirements = payload["requirements"]
    assert isinstance(requirements, dict)
    failures = requirements.get("failures")
    if isinstance(failures, list) and failures:
        print("Requirement checks failed:", file=sys.stderr)
        for failure in failures:
            print(f"  - {failure}", file=sys.stderr)
        return 2
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
