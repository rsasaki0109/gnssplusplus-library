#!/usr/bin/env python3
"""Truth-only correction evaluator for the sealed Phase102 outputs.

Phase102 already performed the only two native runs.  This module never
launches native code and never reads raw inputs.  It verifies the immutable
Phase102 seals, applies one narrow correction to the optional ``mat_used``
summary-field check, and delegates the unchanged Phase102/Phase82 metric to
the pinned Phase102 evaluator in memory.  The only truth reads are the two
official evaluator-side reads performed after candidate preflight.
"""

from __future__ import annotations

import argparse
import importlib.util
import json
import math
import os
from pathlib import Path
from typing import Any


ROOT = Path(__file__).resolve().parents[3]
EVALUATOR = Path(__file__).resolve()
FREEZE = ROOT / "docs/use_cases/records/smartphone_r5_phase103_phase102_truth_only_evaluator_correction_freeze_v1.json"
AUTHORIZATION = ROOT / "docs/use_cases/records/smartphone_r5_phase103_phase102_truth_only_evaluator_correction_authorization_v1.json"
FOCUSED_TESTS = ROOT / "tests/test_smartphone_phase103_phase102_truth_only_evaluator_correction.py"
PHASE102_EVALUATOR = ROOT / "apps/commands/benchmarks/gnss_smartphone_phase102_epoch_clock_vector_accuracy.py"
PHASE102_RESULT = ROOT / "docs/use_cases/records/smartphone_r5_phase102_epoch_clock_vector_accuracy_result_v1.json"
PHASE102_FREEZE = ROOT / "docs/use_cases/records/smartphone_r5_phase102_epoch_clock_vector_accuracy_gate_freeze_v1.json"
PHASE102_MANIFEST = ROOT / "docs/use_cases/records/smartphone_r5_phase102_epoch_clock_vector_accuracy_manifest_v1.json"
PHASE102_AUTHORIZATION = ROOT / "docs/use_cases/records/smartphone_r5_phase102_epoch_clock_vector_raw_evaluation_authorization_v1.json"
OUTPUT_ROOT = ROOT / "output/smartphone-r5/phase102-epoch-clock-vector-accuracy-v1"
RESULT_JSON = ROOT / "docs/use_cases/records/smartphone_r5_phase103_phase102_truth_only_evaluator_correction_result_v1.json"

FREEZE_SHA = "984fdd26171e8327095cbe9b37ba519c55c71a952dfd150a24938f14f2d0c9d1"
FREEZE_COMMIT = "a0b9f72"
AUDIT_SHA = "c8ecf7582832eadd818bee18eb1e075b50361cf45e355500ad5896fc4884df06"
PHASE102_EVALUATOR_SHA = "576ebf9749d1f295737bd82716275a20eb30a5aeeff3fc3828de10960c2255d8"
PHASE102_RESULT_SHA = "d4fcd2246407ad22f75dbf528ef0bbc41c391af341df66f8958b2eff0c48ada2"
PHASE102_FREEZE_SHA = "e964c73160c5851a4e3f6db9497630dad7c8de5081301fa678ed52753a8b0bd1"
PHASE102_MANIFEST_SHA = "3b7b5dc48d6a172dda21c5d27bd4d8b92a64347615768ab87d4716ca17d81cd9"
PHASE102_AUTHORIZATION_SHA = "403b8feb245786180a12c0430de9a797e1c5c2c5355f1c75c79b11546ddce583"
CORRECTION_ID = "phase103-phase102-truth-only-evaluator-correction-v1"
CORRECTION_SCHEMA = "smartphone-r5-phase103-phase102-truth-only-evaluator-correction-result.v1"
CORRECTION_AUTH_SCHEMA = "smartphone-r5-phase103-phase102-truth-only-evaluator-correction-authorization.v1"
ROUTES = (
    "2021-03-16-18-59-us-ca-mtv-a/pixel5",
    "2022-04-01-18-22-us-ca-lax-t/pixel5",
)
DOMAIN_ROWS = {ROUTES[0]: 2158, ROUTES[1]: 1465}


class Phase103CorrectionError(ValueError):
    """Raised when the truth-only correction contract fails closed."""


def fail(message: str) -> Phase103CorrectionError:
    return Phase103CorrectionError(message)


def relative(path: Path) -> str:
    try:
        return str(path.relative_to(ROOT))
    except ValueError:
        return str(path)


def sha256_file(path: Path, label: str) -> str:
    if not path.is_file():
        raise fail(f"missing {label}: {path}")
    import hashlib

    digest = hashlib.sha256()
    with path.open("rb") as handle:
        for chunk in iter(lambda: handle.read(1024 * 1024), b""):
            digest.update(chunk)
    return digest.hexdigest()


def read_json(path: Path, label: str) -> dict[str, Any]:
    try:
        value = json.loads(path.read_text(encoding="utf-8"))
    except (OSError, UnicodeDecodeError, json.JSONDecodeError) as exc:
        raise fail(f"failed to read {label}: {path}: {exc}") from exc
    if not isinstance(value, dict):
        raise fail(f"{label} is not an object")
    return value


def atomic_json(path: Path, value: dict[str, Any]) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    import tempfile

    descriptor, temporary = tempfile.mkstemp(prefix=f".{path.name}.", suffix=".tmp", dir=path.parent)
    try:
        with os.fdopen(descriptor, "w", encoding="utf-8") as handle:
            json.dump(value, handle, indent=2, sort_keys=True, allow_nan=False)
            handle.write("\n")
            handle.flush()
            os.fsync(handle.fileno())
        os.replace(temporary, path)
    except BaseException:
        try:
            os.unlink(temporary)
        except OSError:
            pass
        raise


def atomic_text(path: Path, value: str) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    import tempfile

    descriptor, temporary = tempfile.mkstemp(prefix=f".{path.name}.", suffix=".tmp", dir=path.parent)
    try:
        with os.fdopen(descriptor, "w", encoding="utf-8") as handle:
            handle.write(value)
            handle.flush()
            os.fsync(handle.fileno())
        os.replace(temporary, path)
    except BaseException:
        try:
            os.unlink(temporary)
        except OSError:
            pass
        raise


def _load_phase102() -> Any:
    if sha256_file(PHASE102_EVALUATOR, "Phase102 evaluator") != PHASE102_EVALUATOR_SHA:
        raise fail("Phase102 evaluator source hash changed")
    spec = importlib.util.spec_from_file_location("phase102_pinned_for_phase103", PHASE102_EVALUATOR)
    if spec is None or spec.loader is None:
        raise fail("unable to load Phase102 evaluator")
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


def verify_freeze() -> dict[str, Any]:
    assert sha256_file(FREEZE, "Phase103 correction freeze") == FREEZE_SHA, "Phase103 correction freeze hash changed"
    freeze = read_json(FREEZE, "Phase103 correction freeze")
    for key, expected in {"schema_version": "smartphone-r5-phase103-phase102-truth-only-evaluator-correction-freeze.v1", "phase": 103, "execution_label": "Luna Max", "status": "frozen-before-phase103-truth-only-evaluation"}.items():
        if freeze.get(key) != expected:
            raise fail(f"freeze/{key}: expected {expected!r}, got {freeze.get(key)!r}")
    authority = freeze.get("authority", {})
    audit = authority.get("audit", {})
    if audit.get("commit") != "ce8f513" or audit.get("sha256") != AUDIT_SHA or sha256_file(ROOT / audit.get("path", ""), "Phase103 audit") != AUDIT_SHA:
        raise fail("Phase103 audit pin changed")
    result = authority.get("phase102_result", {})
    if result.get("commit") != "811af21" or result.get("sha256") != PHASE102_RESULT_SHA or sha256_file(PHASE102_RESULT, "Phase102 result") != PHASE102_RESULT_SHA:
        raise fail("Phase102 result pin changed")
    sealed_result = read_json(PHASE102_RESULT, "Phase102 sealed result")
    if sealed_result.get("status") != "no-go-phase102-epoch-clock-vector-accuracy-gates" or sealed_result.get("accuracy_scored") is not False or sealed_result.get("read_accounting", {}).get("truth_reads") != 0:
        raise fail("Phase102 sealed failure boundary changed")
    correction = freeze.get("correction", {})
    for key, expected in {"candidate_count": 1, "id": CORRECTION_ID, "native_solution_reuse": True, "native_solver_invocations": 0, "native_rerun": False, "raw_byte_reads_by_correction": 0, "phase102_solution_replacement": False, "no_algorithm_or_metric_change": True, "no_fallback_or_rerun": True, "truth_reads_per_route": 1}.items():
        if correction.get(key) != expected:
            raise fail(f"freeze/correction/{key}: expected {expected!r}, got {correction.get(key)!r}")
    if correction.get("routes") != list(ROUTES):
        raise fail("freeze/correction/routes changed")
    if correction.get("summary_optional_field_policy") != "mat_used is optional; if present it must be false; if absent it remains absent and is not synthesized":
        raise fail("freeze/optional mat_used policy changed")
    gates = freeze.get("metric_and_gates", {})
    for key, expected in {"source_freeze": "Phase102 freeze 2c86aa9", "metric_unchanged": True, "alignment_unchanged": True, "phase82_same_route_and_phase100_qr_scalar_clock_baselines": True, "strict_macro_gate_m": 0.782, "strict_macro_gate_is_and_gate": True}.items():
        if gates.get(key) != expected:
            raise fail(f"freeze/metric-and-gates/{key}: expected {expected!r}, got {gates.get(key)!r}")
    boundary = freeze.get("truth_evaluator_boundary", {})
    for key, expected in {"evaluator_process_count": 1, "truth_open_after_solution_seal": True, "truth_open_after_phase102_native_exit": True, "truth_reads": 2, "truth_reads_per_route": 1, "native_after_truth": False, "truth_path_or_bytes_in_native": False, "solution_rows_in_result": False}.items():
        if boundary.get(key) != expected:
            raise fail(f"freeze/truth-boundary/{key}: expected {expected!r}, got {boundary.get(key)!r}")
    return freeze


def verify_authorization() -> dict[str, Any]:
    freeze = verify_freeze()
    auth = read_json(AUTHORIZATION, "Phase103 truth-only authorization")
    for key, expected in {"schema_version": CORRECTION_AUTH_SCHEMA, "phase": 103, "execution_label": "Luna Max", "status": "authorized-for-phase103-truth-only-evaluator-correction"}.items():
        if auth.get(key) != expected:
            raise fail(f"authorization/{key}: expected {expected!r}, got {auth.get(key)!r}")
    authority = auth.get("authority")
    if not isinstance(authority, dict):
        raise fail("authorization/authority missing")
    pins = {
        "freeze": (FREEZE, FREEZE_SHA),
        "evaluator": (EVALUATOR, sha256_file(EVALUATOR, "Phase103 correction evaluator")),
        "focused_tests": (FOCUSED_TESTS, sha256_file(FOCUSED_TESTS, "Phase103 focused tests")),
        "phase102_result": (PHASE102_RESULT, PHASE102_RESULT_SHA),
        "phase102_evaluator": (PHASE102_EVALUATOR, PHASE102_EVALUATOR_SHA),
        "phase102_freeze": (PHASE102_FREEZE, PHASE102_FREEZE_SHA),
        "phase102_manifest": (PHASE102_MANIFEST, PHASE102_MANIFEST_SHA),
        "phase102_authorization": (PHASE102_AUTHORIZATION, PHASE102_AUTHORIZATION_SHA),
    }
    for key, (path, digest) in pins.items():
        pin = authority.get(key)
        if not isinstance(pin, dict) or pin.get("path") != relative(path) or pin.get("sha256") != digest:
            raise fail(f"authorization/{key} pin changed")
    if authority.get("freeze_commit") != FREEZE_COMMIT or authority.get("routes") != list(ROUTES) or authority.get("candidate_count") != 1:
        raise fail("authorization authority identity changed")
    candidate = auth.get("candidate", {})
    for key, expected in {"id": CORRECTION_ID, "candidate_count": 1, "native_solution_reuse": True, "native_solver_invocations": 0, "truth_reads": 2, "truth_reads_per_route": 1, "no_fallback_or_rerun": True, "solution_publication": False}.items():
        if candidate.get(key) != expected:
            raise fail(f"authorization/candidate/{key} changed")
    matrix = auth.get("matrix", {})
    for key, expected in {"native_invocations": 0, "truth_reads_in_evaluator": 2, "truth_reads_in_native": 0, "reruns": 0, "fallbacks": 0, "solution_publication": False}.items():
        if matrix.get(key) != expected:
            raise fail(f"authorization/matrix/{key} changed")
    policy = auth.get("execution_policy", {})
    for key, expected in {"truth_only_evaluation_authorized": True, "native_solver_invocations": 0, "truth_reads_per_route": 1, "native_after_truth": False, "solution_publication": False, "kaggle_submission": False}.items():
        if policy.get(key) != expected:
            raise fail(f"authorization/policy/{key} changed")
    return auth


def verify_pre_truth() -> dict[str, Any]:
    verify_freeze()
    return {"native_solver_invocations": 0, "raw_reads": 0, "truth_reads": 0, "accuracy_calculations": 0, "mat_base_pdc_precomputed_reads": 0, "kaggle_or_token_access": 0, "native_rerun": False}


def _validate_corrected_summary(payload: bytes, route: str) -> tuple[dict[str, Any], list[str]]:
    """Phase102 summary validation with one minimal optional-field correction."""
    try:
        summary = json.loads(payload.decode("utf-8"))
    except (UnicodeDecodeError, json.JSONDecodeError) as exc:
        raise fail(f"invalid native summary: {route}: {exc}") from exc
    if not isinstance(summary, dict):
        raise fail(f"native summary is not an object: {route}")
    expected = {
        "dataset_id": route,
        "truth_used": False,
        "production_default_changed": False,
        "base_factors": False,
        "native_pdc_state_bridge": False,
        "native_source_clock_c0d_factor_enabled": True,
        "native_source_clock_c0d_meter_state_parity_enabled": True,
        "native_source_clock_c0d_gnss_first_meter_state_handoff_enabled": True,
        "native_source_clock_c0d_epoch_vector_parity_enabled": True,
        "native_source_clock_c0d_phase99_main_multifrontal_qr_solver_enabled": True,
        "native_source_clock_c0d_phase99_main_multifrontal_qr_solver_selected": True,
        "selected_linear_solver_type": "MULTIFRONTAL_QR",
        "selected_solver_branch": "multifrontal",
        "selected_elimination_function": "EliminateQR",
    }
    for key, value in expected.items():
        if summary.get(key) != value:
            raise fail(f"summary/{route}/{key}: expected {value!r}, got {summary.get(key)!r}")
    # This is deliberately presence-sensitive.  Do not materialize a false
    # summary field when the native schema does not define it.
    if "mat_used" in summary and summary["mat_used"] is not False:
        raise fail(f"summary/{route}/mat_used: present value is not false")
    if not _finite_tree(summary):
        raise fail(f"native summary has nonfinite telemetry: {route}")
    base = _load_phase102()
    forbidden = base._forbidden_summary_keys(summary)
    return summary, forbidden


def _finite_tree(value: Any) -> bool:
    if isinstance(value, float):
        return math.isfinite(value)
    if isinstance(value, dict):
        return all(_finite_tree(child) for child in value.values())
    if isinstance(value, list):
        return all(_finite_tree(child) for child in value)
    return True


def _validate_sealed_lineage(run: dict[str, Any], solution_meta: dict[str, Any], route: str, base: Any) -> None:
    command = run.get("command")
    if not isinstance(command, list) or not command:
        raise fail(f"run metadata command missing: {route}")
    if any(base._forbidden_path(token) or token in base.FORBIDDEN_FLAGS for token in command):
        raise fail(f"sealed native command contains forbidden token: {route}")
    for flag in base.REQUIRED_FLAGS:
        if command.count(flag) != 1:
            raise fail(f"sealed native command selector count changed: {route}/{flag}")
    if command.count("--dataset-id") != 1 or command[command.index("--dataset-id") + 1] != route:
        raise fail(f"sealed native dataset identity changed: {route}")
    for flag, name, _placeholder in base.RAW_FLAGS:
        if command.count(flag) != 1:
            raise fail(f"sealed native raw flag count changed: {route}/{flag}")
        raw_path = command[command.index(flag) + 1]
        if Path(raw_path).name != name or base._forbidden_path(raw_path) or Path(raw_path).is_absolute() or ".." in Path(raw_path).parts:
            raise fail(f"sealed native raw lineage changed: {route}/{name}")
    if command.count("--out") != 1 or command[command.index("--out") + 1] != base._expected_output(route, "solution.csv"):
        raise fail(f"sealed native output path changed: {route}")
    if command.count("--summary-json") != 1 or command[command.index("--summary-json") + 1] != base._expected_output(route, "summary.json"):
        raise fail(f"sealed native summary path changed: {route}")
    raw_inputs = run.get("raw_inputs")
    if not isinstance(raw_inputs, dict) or set(raw_inputs) != set(base.RAW_NAMES):
        raise fail(f"sealed raw role set changed: {route}")
    for name in base.RAW_NAMES:
        pin = raw_inputs[name]
        if not isinstance(pin, dict) or not isinstance(pin.get("path"), str) or Path(pin["path"]).name != name or pin.get("read_by_runner") is not False:
            raise fail(f"sealed raw lineage metadata changed: {route}/{name}")
    for key, expected in {"raw_byte_reads_by_wrapper": 0, "raw_hash_reads_by_wrapper": 0, "truth_read_by_wrapper": False, "mat_base_pdc_precomputed_coordinate_reads": 0, "kaggle_or_token_access": 0, "raw_content_copied_or_transformed": False, "solution_output_published": False}.items():
        if run.get(key) != expected:
            raise fail(f"run metadata/{route}/{key}: expected {expected!r}, got {run.get(key)!r}")
    for key, expected in {"truth_reads": 0, "raw_byte_reads": 0, "mat_base_pdc_precomputed_coordinate_reads": 0, "coordinates_published_in_metadata": False}.items():
        if solution_meta.get(key) != expected:
            raise fail(f"solution metadata/{route}/{key}: expected {expected!r}, got {solution_meta.get(key)!r}")
    if solution_meta.get("rows") != DOMAIN_ROWS[route]:
        raise fail(f"solution seal row count changed: {route}")


def _patch_phase102(base: Any) -> None:
    """Install only the optional-summary and sealed-lineage corrections."""
    original_load = base._load_run_metadata

    def corrected_load(route: str, output_root: Path):
        run, solution_meta = original_load(route, output_root)
        _validate_sealed_lineage(run, solution_meta, route, base)
        return run, solution_meta

    base._load_run_metadata = corrected_load
    base._validate_native_summary = _validate_corrected_summary


def _rewrite_result(base_result: dict[str, Any]) -> dict[str, Any]:
    result = dict(base_result)
    result["schema_version"] = CORRECTION_SCHEMA
    result["phase"] = 103
    result["candidate"] = CORRECTION_ID
    old_status = str(base_result.get("status", ""))
    passed = old_status.startswith("go-")
    result["status"] = "go-phase103-phase102-truth-only-correction-accuracy-gates" if passed else "no-go-phase103-phase102-truth-only-correction-accuracy-gates"
    result["decision"] = "truth-only correction accuracy gate passed; release remains separately unauthorized" if passed else "truth-only correction accuracy gate failed closed; preserve artifacts and do not release"
    result["truth_only_correction"] = {
        "id": CORRECTION_ID,
        "optional_summary_field_policy": "mat_used absent is accepted without synthesis; present mat_used must be false",
        "native_solver_invocations": 0,
        "phase102_native_outputs_reused": 2,
        "raw_reads_by_correction": 0,
        "truth_reads": result.get("read_accounting", {}).get("truth_reads"),
        "solution_rows_in_result": False,
        "fallbacks": 0,
        "reruns": 0,
    }
    result["authority"] = {
        **result.get("authority", {}),
        "phase103_correction_freeze": {"path": relative(FREEZE), "sha256": FREEZE_SHA},
        "phase103_correction_evaluator": {"path": relative(EVALUATOR), "sha256": sha256_file(EVALUATOR, "Phase103 correction evaluator")},
        "phase103_truth_only_authorization": {"path": relative(AUTHORIZATION), "sha256": sha256_file(AUTHORIZATION, "Phase103 authorization"), "status": "authorized-for-phase103-truth-only-evaluator-correction"},
        "phase102_result": {"path": relative(PHASE102_RESULT), "sha256": PHASE102_RESULT_SHA, "native_outputs_reused": 2},
    }
    accounting = dict(result.get("read_accounting", {}))
    accounting["native_solver_invocations"] = 0
    accounting["phase102_native_solver_invocations_reused"] = 2
    accounting["raw_device_gnss_reads_by_correction"] = 0
    accounting["raw_device_imu_reads_by_correction"] = 0
    accounting["broadcast_navigation_reads_by_correction"] = 0
    accounting["truth_reads_by_process"] = "Phase103 truth-only evaluator subprocess"
    accounting["solution_seal_metadata_reads"] = 2
    accounting["phase102_native_output_root_reused"] = True
    accounting["reruns"] = 0
    accounting["fallbacks"] = 0
    result["read_accounting"] = accounting
    result["forbidden_lanes"] = {"truth_in_native_argv_or_environment": False, "native_solver_rerun": False, "mat": False, "base": False, "pdc": False, "precomputed_coordinates": False, "kaggle_or_token": False, "solution_rows_in_result": False}
    result["solution_output_published"] = False
    result["release_or_submission_authorized"] = False
    return result


def result_markdown(result: dict[str, Any]) -> str:
    aggregate = result.get("aggregate", {})
    lines = [
        "# Phase103 Phase102 truth-only evaluator correction result",
        "",
        f"- Status: `{result.get('status')}`",
        f"- Decision: {result.get('decision')}",
        "- Native solver: not rerun; immutable Phase102 MTV-A/LAX-T outputs reused",
        "- Truth: one read per route in this evaluator subprocess only",
        "- Solution rows: not included in this result",
        "",
        "## Macro",
        "",
        f"- Candidate macro: `{aggregate.get('candidate_macro_score_m')}` m",
        f"- Phase82 same-route macro: `{aggregate.get('phase82_same_route_two_route_macro_m')}` m",
        f"- Phase100 QR scalar-clock macro: `{aggregate.get('phase100_qr_scalar_clock_two_route_macro_m')}` m",
        f"- Strict `0.782 m` gate: `{result.get('strict_0_782_gate', {}).get('passed')}`",
        "",
        "## Routes",
        "",
        "| Route | Native reused | Candidate | Phase82 | Phase100 | Truth read | Gates |",
        "|---|---:|---:|---:|---:|---:|---|",
    ]
    for route in ROUTES:
        item = result.get("routes", {}).get(route, {})
        lines.append(f"| `{route}` | `true` | `{item.get('candidate', {}).get('score_m')}` | `{item.get('baseline_phase82_same_route', {}).get('score_m')}` | `{item.get('baseline_phase100_qr_scalar_clock', {}).get('score_m')}` | `{item.get('truth_read')}` | `{item.get('gates', {}).get('passed')}` |")
    lines.extend(["", "The only correction was presence-sensitive handling of optional native summary field `mat_used`; no false field was synthesized. GO does not authorize release, validation, or Kaggle submission.", ""])
    return "\n".join(lines)


def evaluate(output_root: Path = OUTPUT_ROOT, result_path: Path = RESULT_JSON) -> dict[str, Any]:
    verify_pre_truth()
    verify_authorization()
    base = _load_phase102()
    base.verify_freeze()
    base.verify_manifest()
    base.verify_authorization(base.verify_manifest())
    _patch_phase102(base)
    base_result = base.evaluate(output_root, result_path)
    # base.evaluate wrote a complete metric result to the requested path.  It
    # has consumed truth only in its single evaluator call; this rewrite only
    # changes schema/accounting labels and never touches coordinate rows.
    result = _rewrite_result(base_result)
    atomic_json(result_path, result)
    atomic_text(result_path.with_suffix(".md"), result_markdown(result))
    return result


def _write_fail_closed_result(error: str) -> dict[str, Any]:
    result = {"schema_version": CORRECTION_SCHEMA, "phase": 103, "execution_label": "Luna Max", "status": "no-go-phase103-phase102-truth-only-correction-accuracy-gates", "decision": "truth-only correction evaluator failed closed", "candidate": CORRECTION_ID, "error": error, "truth_only_correction": {"native_solver_invocations": 0, "native_outputs_reused": 2, "truth_reads": 0, "reruns": 0, "fallbacks": 0}, "strict_0_782_gate": {"threshold_m": 0.782, "passed": False}, "solution_output_published": False, "release_or_submission_authorized": False, "read_accounting": {"native_solver_invocations": 0, "phase102_native_solver_invocations_reused": 2, "truth_reads": 0, "raw_reads_by_correction": 0, "reruns": 0, "fallbacks": 0}, "forbidden_lanes": {"native_solver_rerun": False, "mat": False, "base": False, "pdc": False, "precomputed_coordinates": False, "kaggle_or_token": False, "solution_rows_in_result": False}}
    atomic_json(RESULT_JSON, result)
    atomic_text(RESULT_JSON.with_suffix(".md"), result_markdown(result))
    return result


def main(argv: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--verify-pre-truth", action="store_true")
    parser.add_argument("--evaluate", action="store_true")
    parser.add_argument("--output-root", type=Path, default=OUTPUT_ROOT)
    parser.add_argument("--result-json", type=Path, default=RESULT_JSON)
    args = parser.parse_args(argv)
    if args.verify_pre_truth == args.evaluate:
        parser.error("choose exactly one of --verify-pre-truth or --evaluate")
    try:
        if args.verify_pre_truth:
            print(json.dumps(verify_pre_truth(), indent=2, sort_keys=True))
            return 0
        result = evaluate(args.output_root, args.result_json)
        print(json.dumps(result, indent=2, sort_keys=True))
        return 0 if result.get("status", "").startswith("go-") else 1
    except Exception as exc:
        if args.evaluate:
            try:
                _write_fail_closed_result(str(exc))
            except Exception:
                pass
        print(f"phase103 truth-only correction evaluator: fail-closed: {exc}", file=os.sys.stderr)
        return 2


if __name__ == "__main__":
    raise SystemExit(main())
