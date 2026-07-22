#!/usr/bin/env python3
"""Verify the evidence-backed PPC KF/FGO goal contract."""

from __future__ import annotations

import argparse
import json
from pathlib import Path
from typing import Any


ROOT = Path(__file__).resolve().parents[1]


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--summary-json", type=Path)
    parser.add_argument("--markdown-output", type=Path)
    parser.add_argument("--check-only", action="store_true")
    return parser.parse_args()


def read_json(relative: str) -> dict[str, Any]:
    payload = json.loads((ROOT / relative).read_text(encoding="utf-8"))
    if not isinstance(payload, dict):
        raise SystemExit(f"{relative}: expected a JSON object")
    return payload


def read_text(relative: str) -> str:
    return (ROOT / relative).read_text(encoding="utf-8")


def main() -> int:
    args = parse_args()
    metrics = read_json("docs/ppc_kf_fgo_goal_metrics.json")
    ledger = read_json("docs/ppc_wrong_fix_event_ledger.json")
    loo = read_json("docs/ppc_kinematic_integrity_loo.json")
    external = read_json("docs/ppc_residual_integrity_external_audit.json")
    consensus = metrics["evaluation"]["online_consensus_replays"][0]
    root_cause = read_text("docs/ppc_nagoya3_wrong_fix_root_cause.md")
    design = read_text("docs/ppc_online_consensus_design.md")
    reproduction = read_text("docs/ppc_reproduction.md")
    readme = read_text("README.md")
    license_text = read_text("LICENSE")
    cmake_tests = read_text("tests/CMakeLists.txt")
    consensus_header = read_text("include/libgnss++/algorithms/integrity_consensus.hpp")
    consensus_tests = read_text("tests/test_integrity_consensus.cpp")

    lib_macro = metrics["macro_mean"]["libgnss"]
    runs = {row["key"]: row for row in metrics["runs"]}
    ledger_summary = ledger["summary"]
    above_10m_epochs = sum(int(event["severity"]["above_10m"]) for event in ledger["events"])
    external_total = external["total"]

    checks: list[dict[str, Any]] = []

    def check(requirement: str, passed: bool, evidence: str, observed: Any) -> None:
        checks.append(
            {
                "requirement": requirement,
                "passed": bool(passed),
                "evidence": evidence,
                "observed": observed,
            }
        )

    check(
        "PPC weighted official score >= 78.7%",
        float(metrics["weighted_official_score_pct"]) >= 78.7,
        "docs/ppc_kf_fgo_goal_metrics.json",
        metrics["weighted_official_score_pct"],
    )
    check(
        "Tokyo run1 FIX >= 80.8%",
        float(runs["tokyo_run1"]["libgnss"]["fix_rate_pct"]) >= 80.8,
        "docs/ppc_kf_fgo_goal_metrics.json",
        runs["tokyo_run1"]["libgnss"]["fix_rate_pct"],
    )
    check(
        "Correct FIX/reference >= 66.230%",
        float(lib_macro["correct_fix_ref_pct"]) >= 66.230,
        "docs/ppc_kf_fgo_goal_metrics.json",
        lib_macro["correct_fix_ref_pct"],
    )
    check(
        "Macro Wrong FIX/FIX < 1.5%",
        float(lib_macro["wrong_fix_rate_pct"]) < 1.5,
        "docs/ppc_kf_fgo_goal_metrics.json",
        lib_macro["wrong_fix_rate_pct"],
    )
    check(
        "Total wrong FIX <= 700",
        int(ledger_summary["wrong_fix_epochs"]) <= 700,
        "docs/ppc_wrong_fix_event_ledger.json",
        ledger_summary["wrong_fix_epochs"],
    )
    check(
        "Wrong FIX above 10 m <= 10",
        above_10m_epochs <= 10,
        "docs/ppc_wrong_fix_event_ledger.json",
        above_10m_epochs,
    )
    check(
        "Wrong-FIX event ledger is runtime-truth-free",
        ledger.get("runtime_truth_used") is False
        and ledger.get("reference_truth_role") == "offline_event_labeling_only"
        and int(ledger_summary["events"]) > 0,
        "docs/ppc_wrong_fix_event_ledger.json",
        ledger_summary,
    )
    check(
        "Nagoya 3 106 m root cause and non-inherited hold diagnosis documented",
        "106.130 m" in root_cause
        and "not inherited from a previous hold state" in root_cause
        and "float-KF" in readme,
        "docs/ppc_nagoya3_wrong_fix_root_cause.md",
        "106.130 m; float-KF basin; no prior held integers",
    )
    check(
        "KF wrong-basin prevention and FGO recovery design documented",
        "wrong-basin" in root_cause
        and "Multi-shadow position recovery" in design
        and "recovery authority" in design,
        "docs/ppc_online_consensus_design.md",
        "detection/recovery authority separation",
    )
    check(
        "Online KF/FGO consensus implementation and tests exist",
        "MultiShadowPositionConsensus" in consensus_header
        and "MultiShadowPositionConsensusTest" in consensus_tests
        and consensus.get("runtime_truth_used") is False
        and consensus.get("positions_replaced") == 0
        and consensus.get("final_state") == "NORMAL",
        "include/libgnss++/algorithms/integrity_consensus.hpp; tests/test_integrity_consensus.cpp",
        {
            "runtime_truth_used": consensus.get("runtime_truth_used"),
            "positions_replaced": consensus.get("positions_replaced"),
            "final_state": consensus.get("final_state"),
        },
    )
    check(
        "Six-fold LOO and extension LOO are complete",
        loo.get("runtime_truth_used") is False
        and len(loo.get("folds", [])) == 6
        and len(loo.get("extension_folds", [])) == 6,
        "docs/ppc_kinematic_integrity_loo.json",
        {"folds": len(loo.get("folds", [])), "extension_folds": len(loo.get("extension_folds", []))},
    )
    check(
        "Active non-PPC receiver holdout catches wrong FIX without truth at runtime",
        external.get("reference_truth_used_by_runtime_policy") is False
        and external.get("external_validation_status") == "safe_no_false_demotions"
        and external.get("external_policy_active") is True
        and int(external_total["wrong_caught"]) > 0
        and int(external_total["correct_harmed"]) == 0,
        "docs/ppc_residual_integrity_external_audit.json",
        {
            "selected": external_total["selected_epochs"],
            "wrong_caught": external_total["wrong_caught"],
            "correct_harmed": external_total["correct_harmed"],
        },
    )
    check(
        "CI regression gate covers staged policy",
        "python_ppc_integrity_policy_tests" in cmake_tests
        and "python_ppc_goal_completion_audit_tests" in cmake_tests,
        "tests/CMakeLists.txt",
        "core CTest registrations",
    )
    required_docs = (
        "README.md",
        "docs/ppc_libgnss_gici_comparison.png",
        "docs/ppc_public_targets.png",
        "docs/ppc_kf_fgo_fix_status_xy.png",
        "docs/ppc_reproduction.md",
    )
    check(
        "README, tables, figures, and reproduction instructions are current",
        all((ROOT / path).exists() and (ROOT / path).stat().st_size > 0 for path in required_docs)
        and "78.8455%" in readme
        and "1.463%" in readme
        and "ppc_residual_integrity_external_audit.md" in readme
        and "--staged-integrity-audit" in reproduction,
        "; ".join(required_docs),
        "current staged metrics and external audit linked",
    )
    check(
        "MIT/GPL boundary is explicit and libgnss++ license remains MIT",
        license_text.startswith("MIT License")
        and metrics["provenance"].get("gici_license_boundary")
        == "GPL-3.0 external executable; NMEA output only"
        and "never copied, linked, or vendored" in design,
        "LICENSE; docs/ppc_kf_fgo_goal_metrics.json; docs/ppc_online_consensus_design.md",
        metrics["provenance"].get("gici_license_boundary"),
    )

    passed = all(row["passed"] for row in checks)
    payload = {
        "schema_version": 1,
        "passed": passed,
        "checks_passed": sum(row["passed"] for row in checks),
        "checks_total": len(checks),
        "checks": checks,
    }
    if not args.check_only:
        if args.summary_json is None or args.markdown_output is None:
            raise SystemExit("output mode requires --summary-json and --markdown-output")
        args.summary_json.parent.mkdir(parents=True, exist_ok=True)
        args.summary_json.write_text(json.dumps(payload, indent=2) + "\n", encoding="utf-8")
        lines = [
            "# PPC KF/FGO goal completion audit",
            "",
            f"Overall: **{'PASS' if passed else 'FAIL'}** ({payload['checks_passed']}/{payload['checks_total']}).",
            "",
            "| Requirement | Status | Observed | Evidence |",
            "|---|---|---|---|",
        ]
        for row in checks:
            observed = json.dumps(row["observed"], ensure_ascii=False, sort_keys=True)
            lines.append(
                f"| {row['requirement']} | {'PASS' if row['passed'] else 'FAIL'} | "
                f"`{observed}` | `{row['evidence']}` |"
            )
        args.markdown_output.parent.mkdir(parents=True, exist_ok=True)
        args.markdown_output.write_text("\n".join(lines) + "\n", encoding="utf-8")
    print(json.dumps({"passed": passed, "checks": f"{payload['checks_passed']}/{payload['checks_total']}"}))
    return 0 if passed else 1


if __name__ == "__main__":
    raise SystemExit(main())
