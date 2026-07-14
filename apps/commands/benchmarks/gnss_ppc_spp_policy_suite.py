#!/usr/bin/env python3
"""Run PPC SPP jump policy sweeps, reports, and comparisons as one suite."""

from __future__ import annotations

import argparse
from dataclasses import dataclass
import json
import os
from pathlib import Path
import re
import subprocess
import sys
from typing import Any


APPS_DIR = Path(__file__).resolve().parent
DEFAULT_PREFIX = "ppc_spp_policy_suite"
DEFAULT_RATES_MPS = "25,50,75,100,150,200"
DEFAULT_MIN_JUMPS_M = "10,20,30,40,50"
DEFAULT_MATCH_TOLERANCE_S = 0.25
DEFAULT_BRIDGE_MAX_GAP_S = 0.0
DEFAULT_BRIDGE_MAX_ANCHOR_SPEED_MPS = 0.0


@dataclass(frozen=True)
class CompareSolution:
    label: str
    path: Path


@dataclass(frozen=True)
class RunSpec:
    label: str
    reference_csv: Path
    pos: Path
    baseline_pos: Path | None = None
    compare_solutions: tuple[CompareSolution, ...] = ()


def sanitize_label(label: str) -> str:
    sanitized = re.sub(r"[^A-Za-z0-9_.-]+", "_", label.strip())
    return sanitized.strip("_.-") or "run"


def reference_from_path(path: Path) -> Path:
    if path.suffix.lower() == ".csv":
        return path
    return path / "reference.csv"


def parse_run_spec(text: str) -> RunSpec:
    if "=" not in text:
        raise argparse.ArgumentTypeError(
            "expected --run label=run_dir_or_reference_csv,pos[,baseline_pos]"
        )
    label, value = text.split("=", 1)
    label = label.strip()
    parts = [part.strip() for part in value.split(",")]
    if not label or len(parts) not in (2, 3) or not all(parts):
        raise argparse.ArgumentTypeError(
            "expected --run label=run_dir_or_reference_csv,pos[,baseline_pos]"
        )
    return RunSpec(
        label=label,
        reference_csv=reference_from_path(Path(parts[0])),
        pos=Path(parts[1]),
        baseline_pos=Path(parts[2]) if len(parts) == 3 else None,
    )


def resolve_manifest_path(value: Any, manifest_dir: Path) -> Path:
    if not isinstance(value, str) or not value.strip():
        raise SystemExit("Manifest paths must be non-empty strings")
    path = Path(value)
    if path.is_absolute():
        return path
    manifest_candidate = manifest_dir / path
    cwd_candidate = Path.cwd() / path
    if manifest_candidate.exists() or not cwd_candidate.exists():
        return manifest_candidate
    return cwd_candidate


def resolve_manifest_generated_path(value: Any) -> Path:
    if not isinstance(value, str) or not value.strip():
        raise SystemExit("Manifest generated paths must be non-empty strings")
    path = Path(value)
    if path.is_absolute():
        return path
    return Path.cwd() / path


def load_manifest_payload(path: Path) -> dict[str, Any]:
    payload = json.loads(path.read_text(encoding="utf-8"))
    if not isinstance(payload, dict):
        raise SystemExit(f"Manifest must be a JSON object: {path}")
    return payload


def load_manifest_runs(path: Path, payload: dict[str, Any]) -> list[RunSpec]:
    runs = payload.get("runs") if isinstance(payload, dict) else None
    if not isinstance(runs, list):
        raise SystemExit(f"Manifest has no runs list: {path}")

    specs: list[RunSpec] = []
    for index, item in enumerate(runs):
        if not isinstance(item, dict):
            raise SystemExit(f"Manifest run #{index + 1} is not an object")
        label = str(item.get("label") or "").strip()
        if not label:
            raise SystemExit(f"Manifest run #{index + 1} has no label")
        if "reference_csv" in item:
            reference_csv = resolve_manifest_path(item["reference_csv"], path.parent)
        elif "run_dir" in item:
            reference_csv = resolve_manifest_path(item["run_dir"], path.parent) / "reference.csv"
        else:
            raise SystemExit(f"Manifest run {label} needs reference_csv or run_dir")

        pos_value = item.get("pos", item.get("input_pos"))
        if pos_value is None:
            raise SystemExit(f"Manifest run {label} needs pos or input_pos")
        baseline_value = item.get("baseline_pos")
        compare_solutions: list[CompareSolution] = []
        for compare_item in item.get("compare_solutions", []):
            if not isinstance(compare_item, dict):
                raise SystemExit(f"Manifest run {label} has invalid compare_solutions item")
            compare_label = str(compare_item.get("label") or "").strip()
            compare_path = compare_item.get("pos", compare_item.get("path"))
            if not compare_label or compare_path is None:
                raise SystemExit(f"Manifest run {label} has incomplete compare solution")
            compare_solutions.append(
                CompareSolution(
                    compare_label,
                    resolve_manifest_path(compare_path, path.parent),
                )
            )

        specs.append(
            RunSpec(
                label=label,
                reference_csv=reference_csv,
                pos=resolve_manifest_path(pos_value, path.parent),
                baseline_pos=(
                    resolve_manifest_path(baseline_value, path.parent)
                    if baseline_value is not None
                    else None
                ),
                compare_solutions=tuple(compare_solutions),
            )
        )
    return specs


def manifest_value(
    payload: dict[str, Any],
    cli_value: Any,
    key: str,
    default_value: Any,
) -> Any:
    if cli_value is not None:
        return cli_value
    if key in payload:
        return payload[key]
    return default_value


def manifest_float(
    payload: dict[str, Any],
    cli_value: float | None,
    key: str,
    default_value: float | None,
) -> float | None:
    value = manifest_value(payload, cli_value, key, default_value)
    return float(value) if value is not None else None


def manifest_bool(payload: dict[str, Any], cli_value: bool, key: str) -> bool:
    if cli_value:
        return True
    return bool(payload.get(key, False))


def validate_run_specs(specs: list[RunSpec]) -> None:
    labels = [spec.label for spec in specs]
    if len(set(labels)) != len(labels):
        raise SystemExit("Run labels must be unique")
    for spec in specs:
        if not spec.reference_csv.exists():
            raise SystemExit(f"Reference CSV not found for {spec.label}: {spec.reference_csv}")
        if not spec.pos.exists():
            raise SystemExit(f"Input .pos not found for {spec.label}: {spec.pos}")
        if spec.baseline_pos is not None and not spec.baseline_pos.exists():
            raise SystemExit(f"Baseline .pos not found for {spec.label}: {spec.baseline_pos}")
        for solution in spec.compare_solutions:
            if not solution.path.exists():
                raise SystemExit(
                    f"Compare .pos not found for {spec.label}/{solution.label}: "
                    f"{solution.path}"
                )


def command_text(command: list[str]) -> str:
    return " ".join(str(part) for part in command)


def run_command(command: list[str], quiet: bool) -> None:
    result = subprocess.run(command, text=True, capture_output=True, check=False)
    if not quiet and result.stdout:
        print(result.stdout, end="")
    if result.returncode != 0:
        if result.stderr:
            print(result.stderr, file=sys.stderr, end="")
        raise SystemExit(result.returncode)


def add_optional_float(command: list[str], name: str, value: float | None) -> None:
    if value is not None:
        command.extend([name, f"{value:g}"])


def numeric_values(rows: list[dict[str, Any]], key: str) -> list[float]:
    values: list[float] = []
    for row in rows:
        value = row.get(key)
        if isinstance(value, (int, float)):
            values.append(float(value))
    return values


def policy_run_rows(policy_report: dict[str, Any] | None) -> list[dict[str, Any]]:
    runs = policy_report.get("runs") if isinstance(policy_report, dict) else None
    if not isinstance(runs, list):
        return []
    rows: list[dict[str, Any]] = []
    for run in runs:
        if not isinstance(run, dict):
            continue
        rows.append(
            {
                "label": run.get("label"),
                "selected_rate_mps": run.get("selected_rate_mps"),
                "selected_min_jump_m": run.get("selected_min_jump_m"),
                "policy_positioning_rate_pct": run.get("policy_positioning_rate_pct"),
                "positioning_drop_pct": run.get("positioning_drop_pct"),
                "policy_p95_h_m": run.get("policy_p95_h_m"),
                "p95_h_delta_m": run.get("p95_h_delta_m"),
                "jump_gate_rejected_epochs": run.get("jump_gate_rejected_epochs"),
                "bridge_inserted_epochs": run.get("bridge_inserted_epochs"),
            }
        )
    return rows


def build_suite_checks(
    policy_report: dict[str, Any] | None,
    *,
    dry_run: bool,
    max_p95_delta_m: float | None,
    max_positioning_drop_pct: float | None,
) -> dict[str, Any]:
    if dry_run:
        return {
            "dry_run": True,
            "passed": None,
            "failures": [],
            "max_p95_delta_m": max_p95_delta_m,
            "max_positioning_drop_pct": max_positioning_drop_pct,
        }
    report_checks = policy_report.get("checks") if isinstance(policy_report, dict) else None
    if isinstance(report_checks, dict):
        checks = dict(report_checks)
    else:
        checks = {
            "passed": False,
            "failures": ["policy report checks were not available"],
            "max_p95_delta_m": max_p95_delta_m,
            "max_positioning_drop_pct": max_positioning_drop_pct,
        }
    checks["dry_run"] = False
    return checks


def build_compare_solutions(spec: RunSpec, filtered_pos: Path) -> list[CompareSolution]:
    solutions: list[CompareSolution] = []
    if spec.baseline_pos is not None:
        solutions.append(CompareSolution("baseline", spec.baseline_pos))
    solutions.append(CompareSolution("input", spec.pos))
    solutions.extend(spec.compare_solutions)
    solutions.append(CompareSolution("policy", filtered_pos))

    seen: set[str] = set()
    unique: list[CompareSolution] = []
    for solution in solutions:
        if solution.label in seen:
            raise SystemExit(f"Duplicate compare label for {spec.label}: {solution.label}")
        seen.add(solution.label)
        unique.append(solution)
    return unique


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        prog=os.environ.get("GNSS_CLI_NAME"),
        description=__doc__,
    )
    parser.add_argument(
        "--manifest-json",
        type=Path,
        help=(
            "Suite manifest with runs containing label, reference_csv/run_dir, "
            "pos/input_pos, optional baseline_pos, and optional compare_solutions."
        ),
    )
    parser.add_argument(
        "--run",
        action="append",
        type=parse_run_spec,
        help="Run as label=run_dir_or_reference_csv,pos[,baseline_pos]. Repeat for each run.",
    )
    parser.add_argument("--output-dir", type=Path)
    parser.add_argument("--prefix")
    parser.add_argument("--rates-mps")
    parser.add_argument("--min-jumps-m")
    parser.add_argument("--match-tolerance-s", type=float)
    parser.add_argument("--bridge-max-gap-s", type=float)
    parser.add_argument("--bridge-max-anchor-speed-mps", type=float)
    parser.add_argument("--max-positioning-drop-pct", type=float)
    parser.add_argument("--min-positioning-rate-pct", type=float)
    parser.add_argument(
        "--max-p95-delta-m",
        type=float,
        help="Policy report check: selected policy p95 H must not exceed baseline by more than this value",
    )
    parser.add_argument("--skip-compare", action="store_true")
    parser.add_argument(
        "--dry-run",
        action="store_true",
        help="Validate inputs and write the suite planning summary without running sweep/report/compare commands.",
    )
    parser.add_argument("--quiet", action="store_true")
    parser.add_argument(
        "--summary-json",
        type=Path,
        help="Suite summary path (default: <output-dir>/<prefix>_suite_summary.json)",
    )
    return parser.parse_args()


def main() -> int:
    args = parse_args()
    specs: list[RunSpec] = []
    manifest_payload: dict[str, Any] = {}
    if args.manifest_json is not None:
        manifest_payload = load_manifest_payload(args.manifest_json)
        specs.extend(load_manifest_runs(args.manifest_json, manifest_payload))
    if args.run:
        specs.extend(args.run)
    if not specs:
        raise SystemExit("Provide --manifest-json, at least one --run, or both")
    validate_run_specs(specs)

    if args.output_dir is not None:
        output_dir = args.output_dir
    elif "output_dir" in manifest_payload:
        output_dir = resolve_manifest_generated_path(manifest_payload["output_dir"])
    else:
        raise SystemExit("Provide --output-dir or set output_dir in --manifest-json")

    prefix = str(manifest_value(manifest_payload, args.prefix, "prefix", DEFAULT_PREFIX))
    rates_mps = str(manifest_value(manifest_payload, args.rates_mps, "rates_mps", DEFAULT_RATES_MPS))
    min_jumps_m = str(
        manifest_value(manifest_payload, args.min_jumps_m, "min_jumps_m", DEFAULT_MIN_JUMPS_M)
    )
    match_tolerance_s = manifest_float(
        manifest_payload,
        args.match_tolerance_s,
        "match_tolerance_s",
        DEFAULT_MATCH_TOLERANCE_S,
    )
    bridge_max_gap_s = manifest_float(
        manifest_payload,
        args.bridge_max_gap_s,
        "bridge_max_gap_s",
        DEFAULT_BRIDGE_MAX_GAP_S,
    )
    bridge_max_anchor_speed_mps = manifest_float(
        manifest_payload,
        args.bridge_max_anchor_speed_mps,
        "bridge_max_anchor_speed_mps",
        DEFAULT_BRIDGE_MAX_ANCHOR_SPEED_MPS,
    )
    max_positioning_drop_pct = manifest_float(
        manifest_payload,
        args.max_positioning_drop_pct,
        "max_positioning_drop_pct",
        None,
    )
    min_positioning_rate_pct = manifest_float(
        manifest_payload,
        args.min_positioning_rate_pct,
        "min_positioning_rate_pct",
        None,
    )
    max_p95_delta_m = manifest_float(
        manifest_payload,
        args.max_p95_delta_m,
        "max_p95_delta_m",
        None,
    )
    skip_compare = manifest_bool(manifest_payload, args.skip_compare, "skip_compare")
    dry_run = manifest_bool(manifest_payload, args.dry_run, "dry_run")
    quiet = manifest_bool(manifest_payload, args.quiet, "quiet")

    output_dir.mkdir(parents=True, exist_ok=True)
    suite_summary_path = (
        args.summary_json
        if args.summary_json is not None
        else output_dir / f"{prefix}_suite_summary.json"
    )
    suite_summary_path.parent.mkdir(parents=True, exist_ok=True)

    jump_sweep_script = APPS_DIR / "gnss_ppc_spp_jump_sweep.py"
    policy_report_script = APPS_DIR / "gnss_ppc_spp_policy_report.py"
    compare_script = APPS_DIR / "gnss_ppc_spp_compare.py"

    run_records: list[dict[str, Any]] = []
    sweep_args: list[str] = []
    commands: list[str] = []
    for spec in specs:
        safe_label = sanitize_label(spec.label)
        stem = output_dir / f"{prefix}_{safe_label}"
        sweep_json = stem.with_name(f"{stem.name}_jump_sweep.json")
        sweep_csv = stem.with_name(f"{stem.name}_jump_sweep.csv")
        filtered_pos = stem.with_name(f"{stem.name}_policy_filtered.pos")
        compare_summary = stem.with_name(f"{stem.name}_compare.json")
        compare_csv = stem.with_name(f"{stem.name}_compare.csv")
        compare_matches = stem.with_name(f"{stem.name}_compare_matches.csv")
        compare_png = stem.with_name(f"{stem.name}_compare.png")

        sweep_command = [
            sys.executable,
            str(jump_sweep_script),
            "--reference-csv",
            str(spec.reference_csv),
            "--pos",
            str(spec.pos),
            "--rates-mps",
            rates_mps,
            "--min-jumps-m",
            min_jumps_m,
            "--match-tolerance-s",
            f"{match_tolerance_s:g}",
            "--bridge-max-gap-s",
            f"{bridge_max_gap_s:g}",
            "--bridge-max-anchor-speed-mps",
            f"{bridge_max_anchor_speed_mps:g}",
            "--summary-json",
            str(sweep_json),
            "--csv",
            str(sweep_csv),
            "--filtered-pos-out",
            str(filtered_pos),
        ]
        add_optional_float(
            sweep_command,
            "--max-positioning-drop-pct",
            max_positioning_drop_pct,
        )
        add_optional_float(
            sweep_command,
            "--min-positioning-rate-pct",
            min_positioning_rate_pct,
        )
        commands.append(command_text(sweep_command))
        if not dry_run:
            run_command(sweep_command, quiet)

        record: dict[str, Any] = {
            "label": spec.label,
            "reference_csv": str(spec.reference_csv),
            "input_pos": str(spec.pos),
            "baseline_pos": str(spec.baseline_pos) if spec.baseline_pos else None,
            "sweep_json": str(sweep_json),
            "sweep_csv": str(sweep_csv),
            "filtered_pos": str(filtered_pos),
        }
        sweep_args.extend(["--sweep", f"{spec.label}={sweep_json}"])

        if not skip_compare:
            compare_command = [
                sys.executable,
                str(compare_script),
                "--reference-csv",
                str(spec.reference_csv),
                "--match-tolerance-s",
                f"{match_tolerance_s:g}",
                "--summary-json",
                str(compare_summary),
                "--csv",
                str(compare_csv),
                "--matched-csv",
                str(compare_matches),
                "--png",
                str(compare_png),
                "--title",
                f"PPC SPP policy suite: {spec.label}",
            ]
            for solution in build_compare_solutions(spec, filtered_pos):
                compare_command.extend(["--solution", f"{solution.label}={solution.path}"])
            commands.append(command_text(compare_command))
            if not dry_run:
                run_command(compare_command, quiet)
            record.update(
                {
                    "compare_summary_json": str(compare_summary),
                    "compare_csv": str(compare_csv),
                    "compare_matched_csv": str(compare_matches),
                    "compare_png": str(compare_png),
                }
            )
        run_records.append(record)

    policy_report_json = output_dir / f"{prefix}_policy_report.json"
    policy_report_csv = output_dir / f"{prefix}_policy_report.csv"
    policy_command = [
        sys.executable,
        str(policy_report_script),
        *sweep_args,
        "--summary-json",
        str(policy_report_json),
        "--csv",
        str(policy_report_csv),
    ]
    add_optional_float(policy_command, "--max-p95-delta-m", max_p95_delta_m)
    add_optional_float(
        policy_command,
        "--max-positioning-drop-pct",
        max_positioning_drop_pct,
    )
    commands.append(command_text(policy_command))
    if not dry_run:
        run_command(policy_command, quiet)

    policy_report_payload: dict[str, Any] | None = None
    if not dry_run:
        policy_report_payload = json.loads(policy_report_json.read_text(encoding="utf-8"))
        if not isinstance(policy_report_payload, dict):
            raise SystemExit(f"Policy report was not a JSON object: {policy_report_json}")
    policy_runs = policy_run_rows(policy_report_payload)
    p95_deltas = numeric_values(policy_runs, "p95_h_delta_m")
    positioning_drops = numeric_values(policy_runs, "positioning_drop_pct")
    checks = build_suite_checks(
        policy_report_payload,
        dry_run=dry_run,
        max_p95_delta_m=max_p95_delta_m,
        max_positioning_drop_pct=max_positioning_drop_pct,
    )

    payload = {
        "manifest_json": str(args.manifest_json) if args.manifest_json else None,
        "dry_run": dry_run,
        "checks": checks,
        "run_count": len(run_records),
        "output_dir": str(output_dir),
        "prefix": prefix,
        "rates_mps": rates_mps,
        "min_jumps_m": min_jumps_m,
        "bridge_max_gap_s": bridge_max_gap_s,
        "bridge_max_anchor_speed_mps": bridge_max_anchor_speed_mps,
        "max_positioning_drop_pct": max_positioning_drop_pct,
        "min_positioning_rate_pct": min_positioning_rate_pct,
        "max_p95_delta_m": max_p95_delta_m,
        "policy_report_json": str(policy_report_json),
        "policy_report_csv": str(policy_report_csv),
        "worst_p95_h_delta_m": max(p95_deltas) if p95_deltas else None,
        "worst_positioning_drop_pct": max(positioning_drops) if positioning_drops else None,
        "policy_runs": policy_runs,
        "runs": run_records,
        "commands": commands,
    }
    suite_summary_path.write_text(json.dumps(payload, indent=2) + "\n", encoding="utf-8")

    print("PPC SPP policy suite")
    if dry_run:
        print("  dry_run: true")
    print(f"  runs: {len(run_records)}")
    print(f"  policy_report: {policy_report_json}")
    print(f"  policy_csv: {policy_report_csv}")
    print(f"  suite_summary: {suite_summary_path}")
    for record in run_records:
        print(
            f"    {record['label']}: "
            f"sweep={record['sweep_json']} filtered={record['filtered_pos']}"
        )
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
