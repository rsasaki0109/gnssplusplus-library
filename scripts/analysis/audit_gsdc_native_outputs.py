"""Audit frozen native outputs against original raw UTC keys (not official keys)."""
from __future__ import annotations

import argparse
from collections import Counter
import csv
import hashlib
import io
import json
import math
from pathlib import Path
from audit_raw_clock_cleanup import audit_raw_clock_cleanup


def digest(path: Path) -> str:
    with path.open("rb") as stream:
        return hashlib.file_digest(stream, "sha256").hexdigest()


def audit_bounded_imu_tail(imu: dict, argv: list[str]) -> dict:
    """Check the explicit measurement-hold policy without certifying real coverage."""
    if ("--native-leading-imu-states" not in argv or
            "--native-imu-supported-long-gaps" in argv or
            "--native-phase201-source-inclusive-forward-imu-schedule" in argv or
            imu.get("leading_imu_bounded_tail_requested") is not True or
            imu.get("bounded_tail_imu_coverage_verified") is not True or
            imu.get("leading_imu_states_requested") is not True or
            not isinstance(imu.get("leading_clock_only_epochs"), int) or
            imu["leading_clock_only_epochs"] <= 0):
        raise ValueError("invalid bounded-tail IMU coverage proof")
    names = ["imu_coverage_first_sample_relative_s", "imu_coverage_last_sample_relative_s",
             "imu_coverage_required_end_relative_s", "imu_required_real_end_relative_s",
             "imu_trailing_measurement_hold_s", "long_gap_imu_max_sample_gap_s"]
    values = [imu.get(name) for name in names]
    if any(type(v) not in (int, float) or not math.isfinite(v) for v in values):
        raise ValueError("nonfinite bounded-tail IMU coverage proof")
    first, last, end, real_end, hold, gap = values
    if (first > 0 or not 0 < real_end <= min(last, end) or
            not 0 <= hold <= .05 + 1e-9 or not 0 < gap <= .05 + 1e-9 or
            abs(hold - max(0., end - last)) > 1e-7 or
            imu.get("long_gap_imu_coverage_verified") is not (last >= end)):
        raise ValueError("inconsistent bounded-tail IMU coverage proof")
    return {"policy": "bounded-terminal-measurement-hold",
            "real_samples_bracket_full_interval": last >= end,
            "terminal_measurement_hold_s": hold, "maximum_sample_gap_s": gap}


def load_official_keys(path: Path, expected_sha256: str) -> dict[str, list[int]]:
    """Read only keys after matching the independently recorded official digest."""
    data = path.read_bytes().replace(b"\r\n", b"\n")
    if hashlib.sha256(data).hexdigest() != expected_sha256:
        raise ValueError("official sample normalized hash mismatch")
    keys: dict[str, list[int]] = {}
    for row in csv.DictReader(io.StringIO(data.decode("utf-8-sig"))):
        keys.setdefault(row["tripId"], []).append(int(row["UnixTimeMillis"]))
    if not keys or any(not values or any(a >= b for a, b in zip(values, values[1:]))
                       for values in keys.values()):
        raise ValueError("official keys must be nonempty and strictly ordered per drive")
    return keys


def audit(case: str, plan: dict, official_keys: dict[str, list[int]] | None = None) -> dict:
    folder = Path(plan["folder"])
    record = folder / "run.json"
    if not record.exists():
        return {"status": "no-completion-record"}
    run = json.loads(record.read_text())
    if run["argv"] != plan["argv"]:
        raise ValueError(f"{case}: replay arguments differ from audited plan")
    if run["state"] != "complete":
        return {"status": "no-completion-record", "recorded_pid": run.get("pid")}
    result = {"returncode": run["returncode"], "wall_s": run["wall_s"],
              "binary_sha256": run["binary_sha256"]}
    summary_path = folder / "summary.json"
    summary = json.loads(summary_path.read_text()) if summary_path.exists() else {}
    if run["returncode"]:
        result.update(status="execution-failed", failure_reason=summary.get("failure_reason"),
                      stderr_tail=(folder / "stderr.log").read_text(errors="replace").splitlines()[-4:])
        return result
    for name in ("solution.csv", "summary.json"):
        if digest(folder / name) != run["outputs"].get(name):
            raise ValueError(f"{case}: frozen output hash mismatch: {name}")
    gnss = Path(plan["argv"][plan["argv"].index("--android-gnss") + 1])
    # Never use the supplied device WLS/satellite coordinates for this audit.
    keys = []
    raw_drift = []
    cleanup_requested = "--native-source-raw-clock-drift-cleanup" in run["argv"]
    with gnss.open(newline="", encoding="utf-8-sig") as stream:
        for row in csv.DictReader(stream):
            if row["MessageType"] != "Raw":
                continue
            key = int(row["utcTimeMillis"])
            if not keys or key != keys[-1]:
                keys.append(key)
                if cleanup_requested:
                    token = row.get("DriftNanosPerSecond", "")
                    value = float(token) if token else float("nan")
                    raw_drift.append(value * 299792458.0 / 1e9)
    if any(a >= b for a, b in zip(keys, keys[1:])):
        raise ValueError(f"{case}: raw UTC groups are not strictly ordered")
    with (folder / "solution.csv").open(newline="", encoding="utf-8-sig") as stream:
        rows = list(csv.DictReader(stream))
    output_keys = [int(row["UnixTimeMillis"]) for row in rows]
    contract = summary["raw_utc_key_contract"]
    expected = keys[1:] if contract["warmup_epoch_excluded"] else keys
    missing = set(expected) - set(output_keys)
    extra = set(output_keys) - set(expected)
    valid_positions = all(
        row["phone"] == case and
        math.isfinite(float(row["LatitudeDegrees"])) and
        math.isfinite(float(row["LongitudeDegrees"])) and
        -90 <= float(row["LatitudeDegrees"]) <= 90 and
        -180 <= float(row["LongitudeDegrees"]) <= 180
        for row in rows
    )
    duplicates = len(output_keys) - len(set(output_keys))
    count_matches = (contract["target_epochs"] == len(rows) and
                     contract["raw_epoch_keys"] == len(keys))
    passed = bool(rows) and output_keys == expected and not duplicates and valid_positions and count_matches
    native_contract = (
        "--native-phase171-raw-p-no-doppler-imu-main" in run["argv"]
        and "--android-raw-clock-only" in run["argv"]
        and summary.get("status") == "imu-combined-factor"
        and summary.get("truth_used") is False
        and summary.get("android_gnss_diagnostics", {}).get("no_device_wls_seed") is True
        and contract.get("device_wls_coordinates_used") is False
        and summary.get("graph", {}).get("converged") is True
    )
    result.update(
        status="raw-key-coverage-verified" if passed else "coverage-failed",
        raw_utc_groups=len(keys), expected_output_keys=len(expected), output_rows=len(rows),
        missing_raw_keys=len(missing), extra_raw_keys=len(extra), duplicate_output_keys=duplicates,
        complete_ordered_raw_key_match=output_keys == expected, finite_phone_coordinates=valid_positions,
        summary_count_matches=count_matches, key_contract=contract,
        position_accuracy_evaluated=False,
        native_estimation_contract_verified=native_contract,
        native_exact_output_fraction=(contract["exact_solution_epochs"] / len(rows) if rows else 0.0),
        output_interpolation_or_hold_count=contract["interpolated_epochs"] + contract["edge_hold_epochs"],
        all_output_epochs_from_native_states=(passed and native_contract
                                    and contract["exact_solution_epochs"] == len(rows)
                                    and contract["interpolated_epochs"] == 0
                                    and contract["edge_hold_epochs"] == 0
                                    and contract["unresolved_epochs"] == 0),
    )
    if official_keys is not None:
        required = official_keys[case]
        required_set = set(required)
        missing_official = sorted(required_set - set(output_keys))
        selected = [key for key in output_keys if key in required_set]
        official_native = (
            selected == required and not duplicates and valid_positions and native_contract
            and contract["target_epochs"] == len(rows)
            and contract["exact_solution_epochs"] == len(rows)
            and contract["interpolated_epochs"] == 0
            and contract["edge_hold_epochs"] == 0
            and contract["unresolved_epochs"] == 0
        )
        result["official_key_coverage"] = {
            "required_keys": len(required), "missing_keys": missing_official,
            "extra_nonofficial_keys": sorted(set(output_keys) - required_set),
            "required_order_preserved": selected == required,
            "all_required_keys_from_native_states": official_native,
            "exact_submission_key_order": output_keys == required,
            "submission_assembled": False,
        }
    extended = "--native-imu-supported-long-gaps" in run["argv"]
    leading = "--native-leading-imu-states" in run["argv"]
    bounded_tail = "--native-leading-imu-bounded-tail" in run["argv"]
    if bounded_tail:
        result["bounded_tail_imu_coverage"] = audit_bounded_imu_tail(
            summary["imu_initialization"], run["argv"])
    elif extended or leading:
        imu = summary["imu_initialization"]
        maximum_gap = imu.get("long_gap_imu_max_sample_gap_s")
        if ((extended and imu.get("imu_supported_long_gaps_requested") is not True) or
                (leading and imu.get("leading_imu_states_requested") is not True) or
                imu.get("long_gap_imu_coverage_verified") is not True or
                not isinstance(maximum_gap, (int, float)) or
                not math.isfinite(maximum_gap) or not 0 < maximum_gap <= .05 + 1e-9):
            raise ValueError(f"{case}: missing or invalid long-gap IMU coverage proof")
        result["long_gap_mapped_imu_max_sample_gap_s"] = maximum_gap
    initialization = folder / "summary.json.initialization.json"
    if "--native-temporal-seed-initialization" in plan["argv"] and not initialization.exists():
        raise ValueError(f"{case}: missing temporal initialization provenance")
    if initialization.exists():
        if digest(initialization) != run["outputs"].get(initialization.name):
            raise ValueError(f"{case}: initialization provenance hash mismatch")
        seed = json.loads(initialization.read_text())
        result["temporal_initial_guess_count"] = seed["temporal_initial_guess_count"]
        result["independent_spp_accepted_epochs"] = seed["independent_spp_accepted_epochs"]
    else:
        result["temporal_initial_guess_count"] = 0
    if cleanup_requested:
        clock_path = folder / "summary.json.raw-clock-cleanup.json"
        if (not initialization.exists() or not clock_path.exists() or
                digest(clock_path) != run["outputs"].get(clock_path.name) or
                summary["imu_initialization"].get("native_source_raw_clock_drift_cleanup") is not True or
                contract["warmup_epoch_excluded"]):
            raise ValueError(f"{case}: missing raw clock cleanup provenance")
        result["raw_clock_drift_cleanup"] = audit_raw_clock_cleanup(
            keys, raw_drift, json.loads(clock_path.read_text()), seed)
    if leading:
        if not initialization.exists() or contract["warmup_epoch_excluded"]:
            raise ValueError(f"{case}: leading states require initialization proof and first output")
        leading_seeds = [s for s in seed["seeds"]
                         if s.get("reason") == "same-run-leading-linear-initial-guess-not-SPP"]
        if (len(leading_seeds) != summary["imu_initialization"].get("leading_clock_only_epochs") or
                any(not s.get("temporal_initial_guess") or s.get("raw_p_status") == "accepted" or
                    not s["raw_source_index"] < s["initial_guess_left_source"] < s["initial_guess_right_source"]
                    for s in leading_seeds)):
            raise ValueError(f"{case}: leading state provenance mismatch")
        result["leading_clock_only_epochs"] = len(leading_seeds)
    if "--native-samsung-clock-drift" in plan["argv"]:
        clock_path = folder / "summary.json.samsung-clock.json"
        if not clock_path.exists() or digest(clock_path) != run["outputs"].get(clock_path.name):
            raise ValueError(f"{case}: missing or changed Samsung clock provenance")
        clock = json.loads(clock_path.read_text())
        clock_rows = clock["epochs"]
        clock_keys = [row["utc"] for row in clock_rows]
        if (len(clock_keys) != contract["raw_epoch_keys"] or
                len(clock_keys) != len(set(clock_keys)) or
                clock_keys != [key for key in keys if key in set(clock_keys)] or
                not set(output_keys).issubset(clock_keys) or
                not all(math.isfinite(row["selected_drift_mps"]) for row in clock_rows) or
                not summary["imu_initialization"]["native_samsung_clock_drift"]):
            raise ValueError(f"{case}: Samsung clock provenance coverage mismatch")
        result["samsung_clock_drift"] = {
            "epochs": len(clock_rows), "jump_masks": clock["jump_masks"],
            "filled_values": clock["filled_values"], "trajectory": clock["trajectory"],
            # Loader-selected provenance is distinct from the full raw-key
            # coverage test above, which must still fail if any keys are lost.
            "missing_original_raw_keys": len(set(keys) - set(clock_keys)),
        }
    return result


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--plan", type=Path, required=True)
    parser.add_argument("--output-json", type=Path, required=True)
    parser.add_argument("--official-sample", type=Path)
    parser.add_argument("--official-sample-sha256")
    args = parser.parse_args()
    if bool(args.official_sample) != bool(args.official_sample_sha256):
        parser.error("official sample and its independently recorded hash are required together")
    official = load_official_keys(args.official_sample, args.official_sample_sha256) if args.official_sample else None
    plan = json.loads(args.plan.read_text())
    runs = {case: audit(case, entry, official) for case, entry in plan["runs"].items()}
    counts = dict(Counter(run["status"] for run in runs.values()))
    result = {"schema_version": "gsdc2023-native-output-audit.v1",
              "official_submission_keys_verified": official is not None,
        "official_sample_path": str(args.official_sample) if official is not None else None,
        "official_sample_normalized_sha256": args.official_sample_sha256,
              "pending_status_is_not_process_liveness": True,
              "status_counts": counts, "runs": runs}
    args.output_json.write_text(json.dumps(result, indent=2) + "\n", encoding="utf-8")
    print(json.dumps(counts))


if __name__ == "__main__":
    main()
