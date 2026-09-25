"""Summarize a frozen train validation snapshot, preserving incomplete groups."""
from __future__ import annotations

import argparse
from collections import Counter, defaultdict
import hashlib
import json
from pathlib import Path
from statistics import mean


def summarize(plan: dict, validation: dict, grouping: dict[str, str] | None = None) -> dict:
    expected = plan["runs"]
    runs = validation["runs"]
    grouping = grouping or {}
    if set(grouping) - set(expected) or any(not isinstance(g, str) or not g for g in grouping.values()):
        raise ValueError("Grouping override contains unknown cases or invalid group names")
    original_groups = defaultdict(set)
    for case, entry in expected.items():
        original_groups[entry["route_group"]].add(grouping.get(case, entry["route_group"]))
    if any(len(destinations) != 1 for destinations in original_groups.values()):
        raise ValueError("Grouping override may merge but cannot split a frozen group")
    if set(expected) != set(runs):
        raise ValueError("Validation must account for every planned drive exactly once")
    phones, groups = defaultdict(list), defaultdict(list)
    for case, entry in expected.items():
        result = runs[case]
        if entry["route_group"] != result["route_group"]:
            raise ValueError(f"{case}: route group changed")
        phones[case.rsplit("/", 1)[1]].append(case)
        groups[grouping.get(case, entry["route_group"])].append(case)
        if result["status"] == "scored-native-development":
            audit = result["audit"]
            if not audit["all_output_epochs_from_native_states"]:
                raise ValueError(f"{case}: score without native provenance")
            if audit["binary_sha256"] != plan["binary_sha256"]:
                raise ValueError(f"{case}: executable differs from plan")
            score = result["score"]
            if (score["distance_variant"], score["percentile_variant"]) != (
                "haversine_sphere", "linear_n_minus_1"
            ):
                raise ValueError(f"{case}: metric convention changed")

    def aggregate(cases: list[str]) -> dict:
        scored = [runs[c] for c in cases if runs[c]["status"] == "scored-native-development"]
        complete = len(scored) == len(cases)
        audits = [r["audit"] for r in scored]
        epochs = sum(a["expected_output_keys"] for a in audits)
        return {
            "planned_drives": len(cases),
            "scored_drives": len(scored),
            "status_counts": dict(Counter(runs[c]["status"] for c in cases)),
            "complete": complete,
            "full_scope_mean_drive_score_m": mean(r["score"]["phone_score_m"] for r in scored) if complete else None,
            "completed_subset_mean_drive_score_m": mean(r["score"]["phone_score_m"] for r in scored) if scored else None,
            "completed_subset_mean_drive_p50_m": mean(r["score"]["p50_m"] for r in scored) if scored else None,
            "completed_subset_mean_drive_p95_m": mean(r["score"]["p95_m"] for r in scored) if scored else None,
            "completed_subset_raw_epochs": epochs,
            "completed_subset_truth_epochs": sum(r["truth_keys"] for r in scored),
            "completed_subset_unscored_native_epochs": sum(r["extra_native_keys_dropped"] for r in scored),
            "completed_subset_truth_coverage_fraction_of_raw_epochs": sum(r["truth_keys"] for r in scored) / epochs if epochs else None,
            "completed_subset_missing_raw_key_rate": sum(a["missing_raw_keys"] for a in audits) / epochs if epochs else None,
            "completed_subset_output_interpolation_or_hold_rate": sum(a["output_interpolation_or_hold_count"] for a in audits) / epochs if epochs else None,
            "completed_subset_temporal_initial_guess_count": sum(a["temporal_initial_guess_count"] for a in audits),
            "completed_subset_wall_s_sum": sum(a["wall_s"] for a in audits),
        }

    group_results = {g: {"cases": cases, **aggregate(cases)} for g, cases in sorted(groups.items())}
    full_groups = [g for g in group_results.values() if g["complete"]]
    return {
        "schema": "gsdc-train-validation-summary.v1",
        "official_score": False,
        "heldout": False,
        "evaluation_group_overrides": grouping,
        "overall": aggregate(list(expected)),
        "phones": {p: aggregate(cases) for p, cases in sorted(phones.items())},
        "route_groups": group_results,
        "complete_route_groups": len(full_groups),
        "planned_route_groups": len(groups),
        "full_scope_mean_route_group_score_m": mean(g["full_scope_mean_drive_score_m"] for g in full_groups) if len(full_groups) == len(groups) else None,
        "limitations": [
            "Snapshot of audited validation; pending records do not establish process liveness.",
            "Completed-subset means are not full-scope scores and can be selection biased.",
            "Mean per-drive P50/P95 are not pooled point-error percentiles.",
            "Output coverage rates cover scored drives only; failures and pending drives remain in status counts.",
            "Native epochs without truth are excluded only from accuracy scoring, not from native coverage audits.",
            "Temporal initial guesses precede native optimization; they are not output interpolation.",
            "Summed shared-load wall times are not elapsed batch time or a controlled performance benchmark.",
        ],
    }


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--plan", type=Path, required=True)
    parser.add_argument("--validation", type=Path, required=True)
    parser.add_argument("--output-json", type=Path, required=True)
    parser.add_argument("--grouping-audit", type=Path,
                        help="Pinned coincident-route evidence; merge evaluation groups only")
    args = parser.parse_args()
    # Hash exactly the bytes summarized, even if the live validator later updates them.
    raw = {name: path.read_bytes() for name, path in (("plan", args.plan), ("validation", args.validation))}
    grouping = {}
    if args.grouping_audit:
        raw["grouping_audit"] = args.grouping_audit.read_bytes()
        evidence = json.loads(raw["grouping_audit"])
        if evidence["plan_sha256"] != hashlib.sha256(raw["plan"]).hexdigest():
            raise ValueError("Grouping audit does not match frozen plan")
        grouping = {case: evidence["recommended_evaluation_group"] for case in evidence["cases"]}
    report = summarize(*(json.loads(raw[name]) for name in ("plan", "validation")), grouping)
    report["inputs"] = {name: {"path": str(path.resolve()), "sha256": hashlib.sha256(raw[name]).hexdigest()} for name, path in (("plan", args.plan), ("validation", args.validation))}
    if args.grouping_audit:
        report["inputs"]["grouping_audit"] = {
            "path": str(args.grouping_audit.resolve()),
            "sha256": hashlib.sha256(raw["grouping_audit"]).hexdigest()}
    args.output_json.parent.mkdir(parents=True, exist_ok=True)
    args.output_json.write_text(json.dumps(report, indent=2) + "\n", encoding="utf-8")
    print(json.dumps(report["overall"]))


if __name__ == "__main__":
    main()
