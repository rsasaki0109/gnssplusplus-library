"""Aggregate audited retries without claiming a homogeneous all-drive solution."""
from __future__ import annotations

import argparse
from collections import Counter
import json
from pathlib import Path

from audit_gsdc_native_outputs import audit, load_official_keys


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--plan", type=Path, required=True)
    parser.add_argument("--replay-root", type=Path, action="append", default=[])
    parser.add_argument("--output-json", type=Path, required=True)
    parser.add_argument("--official-sample", type=Path)
    parser.add_argument("--official-sample-sha256")
    args = parser.parse_args()
    if bool(args.official_sample) != bool(args.official_sample_sha256):
        parser.error("official sample and its independently recorded hash are required together")
    official = load_official_keys(args.official_sample, args.official_sample_sha256) if args.official_sample else None
    plan = json.loads(args.plan.read_text())["runs"]
    folders = {Path(entry["folder"]).resolve() for entry in plan.values()}
    for root in args.replay_root:
        if not root.is_dir():
            raise ValueError(f"Replay root not found: {root}")
        folders.update(p.parent.resolve() for p in root.rglob("run.json"))
    cases = {case: [] for case in plan}
    by_binary: dict[str, set[str]] = {}
    for folder in sorted(folders):
        record = folder / "run.json"
        if not record.exists():
            continue
        run = json.loads(record.read_text())
        argv = run["argv"]
        case = argv[argv.index("--dataset-id") + 1]
        if case not in cases:
            continue  # Development routes cannot contribute to test coverage.
        expected = plan[case]["argv"]
        for flag in ("--android-gnss", "--android-imu", "--nav", "--native-base-rinex"):
            if (Path(argv[argv.index(flag) + 1]).resolve() !=
                    Path(expected[expected.index(flag) + 1]).resolve()):
                raise ValueError(f"{case}: retry changed benchmark input {flag}")
        result = audit(case, {"folder": str(folder), "argv": argv}, official)
        attempt = {"folder": str(folder), "argv": argv, "audit": result}
        cases[case].append(attempt)
        if result.get("all_output_epochs_from_native_states"):
            by_binary.setdefault(result["binary_sha256"], set()).add(case)
    counts = Counter()
    for attempts in cases.values():
        if any(a["audit"].get("all_output_epochs_from_native_states") for a in attempts):
            counts["any_variant_native_raw_key_coverage"] += 1
        elif any(a["audit"]["status"] == "raw-key-coverage-verified" for a in attempts):
            counts["raw_key_coverage_without_verified_native_contract"] += 1
        else:
            counts["no_verified_native_raw_key_coverage"] += 1
    report = {
        "schema": "gsdc-native-replay-coverage.v1",
        "official_submission_keys_verified": official is not None,
        "official_sample_path": str(args.official_sample) if official is not None else None,
        "official_sample_normalized_sha256": args.official_sample_sha256,
        "official_complete_native_drives": (sum(any(a["audit"].get("official_key_coverage", {}).get("all_required_keys_from_native_states") for a in attempts) for attempts in cases.values()) if official is not None else None),
        "position_accuracy_evaluated": False,
        "homogeneous_all_drive_recipe_proven": False,
        "counts_are_unique_drives_across_different_variants": True,
        "pending_records_do_not_prove_process_liveness": True,
        "planned_drives": len(plan), "counts": dict(counts),
        "native_coverage_by_binary": {key: sorted(value) for key, value in by_binary.items()},
        "attempts_by_drive": cases,
    }
    args.output_json.write_text(json.dumps(report, indent=2) + "\n", encoding="utf-8")
    print(json.dumps(report["counts"]))


if __name__ == "__main__":
    main()
