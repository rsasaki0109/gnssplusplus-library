#!/usr/bin/env python3
"""Phase105-pinned Phase104 evaluator with a runtime-linkage authority.

The Phase104 evaluator remains sealed.  This small launch-free shim redirects
its manifest, authorization, wrapper, and output-root pins to the Phase105
records while retaining the exact Phase104 source, metric, sidecar, and
truth-only evaluation routines.  It does not read raw or truth data during
pre-raw verification.
"""

from __future__ import annotations

import argparse
from pathlib import Path
import sys
from typing import Any


ROOT = Path(__file__).resolve().parents[3]
MODULE_DIR = Path(__file__).resolve().parent
if str(MODULE_DIR) not in sys.path:
    sys.path.insert(0, str(MODULE_DIR))

import gnss_smartphone_phase104_stage_main_attribution as _base  # noqa: E402


PHASE104_FREEZE = ROOT / "docs/use_cases/records/smartphone_r5_phase104_stage_main_accuracy_attribution_freeze_v1.json"
PHASE105_FREEZE = ROOT / "docs/use_cases/records/smartphone_r5_phase105_runtime_linkage_freeze_v1.json"
PHASE105_MANIFEST = ROOT / "docs/use_cases/records/smartphone_r5_phase105_runtime_linkage_manifest_v1.json"
PHASE105_AUTHORIZATION = ROOT / "docs/use_cases/records/smartphone_r5_phase105_runtime_linkage_raw_execution_authorization_v1.json"
PHASE105_TRUTH_AUTHORIZATION = ROOT / "docs/use_cases/records/smartphone_r5_phase105_runtime_linkage_truth_only_authorization_v1.json"
PHASE105_WRAPPER = ROOT / "apps/commands/benchmarks/gnss_smartphone_phase105_runtime_linkage_execute.py"
PHASE105_TESTS = ROOT / "tests/test_smartphone_phase105_runtime_linkage.py"
PHASE105_OUTPUT_RELATIVE_ROOT = "output/smartphone-r5/phase105-runtime-linkage-attribution-v1/"
PHASE105_CANDIDATE_ID = "phase105-child-only-trusted-gtsam-library-path-prepend-v1"
PHASE105_FREEZE_SHA = "3d44e1d9109be7e5d4cf5aba6f579976be2b71f5083c915b0d893567f002f36d"


def _configure_base() -> None:
    # Keep all Phase104 algorithms, pins, and metric implementation intact;
    # only the sealed record paths/output root are redirected for this fresh
    # one-shot run.
    _base.FREEZE = PHASE104_FREEZE
    _base.MANIFEST = PHASE105_MANIFEST
    _base.AUTHORIZATION = PHASE105_AUTHORIZATION
    _base.TRUTH_AUTHORIZATION = PHASE105_TRUTH_AUTHORIZATION
    _base.EVALUATOR = Path(__file__).resolve()
    _base.WRAPPER = PHASE105_WRAPPER
    _base.FOCUSED_TESTS = PHASE105_TESTS
    _base.OUTPUT_RELATIVE_ROOT = PHASE105_OUTPUT_RELATIVE_ROOT


_configure_base()


def verify_phase105_freeze() -> dict[str, Any]:
    if _base.sha256_file(PHASE105_FREEZE, "Phase105 runtime-linkage freeze") != PHASE105_FREEZE_SHA:
        raise _base.fail("Phase105 freeze SHA mismatch")
    freeze = _base.read_json(PHASE105_FREEZE, "Phase105 runtime-linkage freeze")
    for key, expected in {
        "schema_version": "smartphone-r5-phase105-runtime-linkage-freeze.v1",
        "phase": 105,
        "execution_label": "Luna Max",
        "status": "frozen-before-phase105-wrapper-fix-and-raw-execution",
    }.items():
        _base.assert_equal(freeze.get(key), expected, f"Phase105 freeze/{key}")
    authority = freeze.get("authority")
    if not isinstance(authority, dict):
        raise _base.fail("Phase105 freeze/authority missing")
    audit = authority.get("audit")
    if not isinstance(audit, dict):
        raise _base.fail("Phase105 freeze/audit missing")
    _base.assert_equal(audit.get("path"), "docs/use_cases/records/smartphone_r5_phase105_runtime_linkage_audit_v1.md", "Phase105 audit/path")
    _base.assert_equal(audit.get("sha256"), "af0985c7c52d478355cd66f4492e6000f12b7be239f27f7433812289d42d82a3", "Phase105 audit/sha256")
    candidate = freeze.get("candidate")
    if not isinstance(candidate, dict):
        raise _base.fail("Phase105 freeze/candidate missing")
    for key, expected in {
        "candidate_count": 1,
        "id": PHASE105_CANDIDATE_ID,
        "child_process_only": True,
        "exact_assignment": "LD_LIBRARY_PATH=/home/sasaki/.local/lib:${inherited_when_nonempty}",
        "parent_environment_changed": False,
        "native_binary_changed": False,
        "solver_or_graph_changed": False,
        "raw_input_contract_changed": False,
        "stage_or_main_contract_changed": False,
        "truth_or_accuracy_in_native": False,
        "no_fallback_or_rerun": True,
        "default_legacy_unchanged": True,
    }.items():
        _base.assert_equal(candidate.get(key), expected, f"Phase105 freeze/candidate/{key}")
    runtime = freeze.get("trusted_runtime_directory")
    if not isinstance(runtime, dict):
        raise _base.fail("Phase105 freeze/trusted runtime missing")
    _base.assert_equal(runtime.get("path"), "/home/sasaki/.local/lib", "Phase105 runtime/path")
    _base.assert_equal(runtime.get("prepend_exactly"), True, "Phase105 runtime/prepend")
    closure = freeze.get("ldd_closure_with_candidate")
    if not isinstance(closure, dict):
        raise _base.fail("Phase105 freeze/ldd closure missing")
    _base.assert_equal(closure.get("unresolved_entries"), [], "Phase105 ldd/unresolved")
    _base.assert_equal(closure.get("resolved_entry_count"), 19, "Phase105 ldd/count")
    accounting = freeze.get("read_accounting_at_freeze")
    if not isinstance(accounting, dict):
        raise _base.fail("Phase105 freeze/read accounting missing")
    for key in accounting:
        _base.assert_equal(accounting.get(key), 0, f"Phase105 freeze/accounting/{key}")
    return freeze


def verify_pre_raw() -> dict[str, Any]:
    verify_phase105_freeze()
    return _base.verify_pre_raw()


def verify_manifest() -> dict[str, Any]:
    verify_phase105_freeze()
    return _base.verify_manifest()


def verify_authorization(manifest: dict[str, Any] | None = None) -> dict[str, Any]:
    verify_phase105_freeze()
    return _base.verify_authorization(manifest)


def verify_truth_authorization() -> dict[str, Any]:
    verify_phase105_freeze()
    return _base.verify_truth_authorization()


def evaluate(output_root: Path, result_path: Path) -> dict[str, Any]:
    verify_phase105_freeze()
    return _base.evaluate(output_root, result_path)


def __getattr__(name: str) -> Any:
    # Expose the unchanged Phase104 parser/sealer helpers to the Phase105
    # wrapper and tests without copying algorithm or metric code.
    return getattr(_base, name)


def main(argv: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--verify-freeze", action="store_true")
    parser.add_argument("--verify-manifest", action="store_true")
    parser.add_argument("--verify-pre-raw", action="store_true")
    parser.add_argument("--evaluate", action="store_true")
    parser.add_argument("--output-root", type=Path)
    parser.add_argument("--result-json", type=Path)
    args = parser.parse_args(argv)
    modes = [args.verify_freeze, args.verify_manifest, args.verify_pre_raw, args.evaluate]
    if sum(bool(value) for value in modes) != 1:
        parser.error("choose exactly one verification/evaluation mode")
    try:
        if args.verify_freeze:
            verify_phase105_freeze()
        elif args.verify_manifest:
            verify_manifest()
        elif args.verify_pre_raw:
            import json
            print(json.dumps(verify_pre_raw(), indent=2, sort_keys=True))
        else:
            if args.output_root is None or args.result_json is None:
                parser.error("--evaluate requires --output-root and --result-json")
            import json
            print(json.dumps(evaluate(args.output_root.resolve(), args.result_json.resolve()), indent=2, sort_keys=True))
    except (_base.Phase104Error, OSError) as exc:
        print(f"phase105 contract/evaluator: fail-closed: {exc}")
        return 2
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
