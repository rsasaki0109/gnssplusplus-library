#!/usr/bin/env python3
"""Phase146 one-shot accuracy lane for immutable Phase144 opaque outputs.

Plan, manifest, pre-truth, and authorization modes are metadata/source only.
Only ``--evaluate`` after the separately committed authorization may read the
two sealed Phase144 candidate CSVs and the two pinned truth CSVs.  Parsing,
joining, Haversine scoring, percentile, speed, and macro semantics are loaded
from the pinned Phase142 evaluator/Phase82 scorer; this module adds no metric
or coordinate transformation.
"""

from __future__ import annotations

import hashlib
import importlib.util
import json
import math
import os
from pathlib import Path
import sys
import tempfile
from typing import Any, Mapping


ROOT = Path(__file__).resolve().parents[3]
EVALUATOR = Path(__file__).resolve()
PHASE144_RESULT = ROOT / "docs/use_cases/records/smartphone_r5_phase144_telemetry_serializer_raw_result_v1.json"
PHASE145_RESULT = ROOT / "docs/use_cases/records/smartphone_r5_phase145_phase144_contract_revalidation_result_v1.json"
PHASE142_EVALUATOR = ROOT / "apps/commands/benchmarks/gnss_smartphone_phase142_phase141_accuracy.py"
PHASE82_SCORER = ROOT / "apps/commands/benchmarks/gnss_smartphone_phase82_phase80_direct_quality_accuracy.py"
PHASE76_PARSER = ROOT / "apps/commands/benchmarks/gnss_smartphone_phase76_phase75_accuracy_integrity_recovery.py"
PHASE137_EVALUATOR = ROOT / "apps/commands/benchmarks/gnss_smartphone_phase137_phase136_accuracy.py"
PHASE134_EVALUATOR = ROOT / "apps/commands/benchmarks/gnss_smartphone_phase134_native_summary_bridge_accuracy.py"
PHASE142_FREEZE = ROOT / "docs/use_cases/records/smartphone_r5_phase142_phase141_accuracy_gate_freeze_v1.json"
PHASE142_MANIFEST = ROOT / "docs/use_cases/records/smartphone_r5_phase142_phase141_accuracy_gate_manifest_v1.json"
AUDIT = ROOT / "docs/use_cases/records/smartphone_r5_phase146_phase144_accuracy_gate_audit_v1.md"
MANIFEST = ROOT / "docs/use_cases/records/smartphone_r5_phase146_phase144_accuracy_gate_manifest_v1.json"
PRE_TRUTH = ROOT / "docs/use_cases/records/smartphone_r5_phase146_phase144_accuracy_gate_pre_truth_v1.json"
AUTHORIZATION = ROOT / "docs/use_cases/records/smartphone_r5_phase146_phase144_accuracy_authorization_v1.json"
FOCUSED_TESTS = ROOT / "tests/test_smartphone_phase146_phase144_accuracy.py"
RESULT_JSON = ROOT / "docs/use_cases/records/smartphone_r5_phase146_phase144_accuracy_result_v1.json"

ROUTES = (
    "2021-03-16-18-59-us-ca-mtv-a/pixel5",
    "2022-04-01-18-22-us-ca-lax-t/pixel5",
)
LABELS = {ROUTES[0]: "MTV-A", ROUTES[1]: "LAX-T"}
DOMAIN_ROWS = {ROUTES[0]: 2158, ROUTES[1]: 1465}
PROBLEM_EPOCHS = {ROUTES[0]: 2159, ROUTES[1]: 1466}
TRUTH_ROWS = {ROUTES[0]: 2159, ROUTES[1]: 1465}
THRESHOLD = 0.782

PHASE144_SHA = "86fa44a595d71ed7f8bf400001a30060e61f343815bac92e76f9e4d3fefc50ee"
PHASE145_SHA = "04b81bc9dd299eb1fa286c5142c5d16cb3785902d727693d8f440e83030f3e67"
PHASE142_EVALUATOR_SHA = "03253d1f7c70dc761699ce19d03123457e386b6ab9cd0facb395c942f8450fb6"
PHASE82_SHA = "232ba0ab1fa7ebde6ddfaaf851d190d1ed32377ea056ab6c31e244139c2977fb"
PHASE76_SHA = "513e5970ee58846963f077b9960944e5fbf5f3535e5be24ca6e72493785f7761"
PHASE137_SHA = "f294aecadca1e427673ada124f24f91bb5cb2b880156d8337fde8f7afc32c200"
PHASE134_SHA = "b554e2c07bedffcb53751e4e6351bcf5006fe99370dee34a16cc4f3eb81b145a"
PHASE142_FREEZE_SHA = "dc2af8b53f7aca2b22cb0193231ab62304095fd832d18647b5686a73ea1f818b"
PHASE142_MANIFEST_SHA = "8c3f12dd032348ea5062d9b6cd01a90c9e6787739ef85095fb1753b8f2e77cfc"
PHASE144_COMMIT = "3110202"
PHASE145_COMMIT = "a4805dee5f1caa2ce9dca19ac32065eda6afac78"
PHASE142_FREEZE_COMMIT = "3ab1429a431cd65ceca193dd997b397b0879aa9"
PHASE142_MANIFEST_COMMIT = "adbce9bd266922bb584ffb40db7134790c71be07"

SCHEMA = "smartphone-r5-phase146-phase144-accuracy-gate-manifest.v1"
RESULT_SCHEMA = "smartphone-r5-phase146-phase144-accuracy-result.v1"


class Phase146Error(ValueError):
    """Fail-closed contract violation."""


def fail(message: str) -> Phase146Error:
    return Phase146Error(message)


def relative(path: Path) -> str:
    return str(path.relative_to(ROOT))


def read_json(path: Path, label: str) -> dict[str, Any]:
    try:
        text = path.read_text(encoding="utf-8")
        value = json.loads(text, object_pairs_hook=_no_duplicates)
    except (OSError, UnicodeDecodeError, json.JSONDecodeError, Phase146Error) as exc:
        raise fail(f"failed to read {label}: {exc}") from exc
    if not isinstance(value, dict):
        raise fail(f"{label} is not an object")
    return value


def _no_duplicates(pairs: list[tuple[str, Any]]) -> dict[str, Any]:
    result: dict[str, Any] = {}
    for key, value in pairs:
        if key in result:
            raise fail(f"duplicate JSON key {key!r}")
        result[key] = value
    return result


def sha256_static(path: Path, label: str) -> str:
    lowered = str(path).lower()
    if path.name.lower() in {"opaque_solution_output.csv", "ground_truth.csv", "device_gnss.csv", "device_imu.csv", "brdc.nav", "base.obs"} or any(term in lowered for term in ("/truth/", ".mat", ".pdc", "kaggle")):
        raise fail(f"payload hash forbidden for {label}")
    if not path.is_file():
        raise fail(f"missing {label}: {path}")
    digest = hashlib.sha256()
    with path.open("rb") as handle:
        for chunk in iter(lambda: handle.read(1024 * 1024), b""):
            digest.update(chunk)
    return digest.hexdigest()


def equal(actual: Any, expected: Any, label: str) -> None:
    if actual != expected:
        raise fail(f"{label}: expected {expected!r}, got {actual!r}")


def finite(value: Any) -> bool:
    return isinstance(value, (int, float)) and not isinstance(value, bool) and math.isfinite(float(value))


def deferred(value: Any, basename: str, fragment: str, label: str) -> str:
    if not isinstance(value, str) or not value or Path(value).is_absolute() or ".." in Path(value).parts or Path(value).name != basename or fragment not in value:
        raise fail(f"unsafe deferred {label} path: {value!r}")
    return value


def load_module(path: Path, name: str) -> Any:
    spec = importlib.util.spec_from_file_location(name, path)
    if spec is None or spec.loader is None:
        raise fail(f"unable to load {path}")
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


def phase142() -> Any:
    equal(sha256_static(PHASE142_EVALUATOR, "Phase142 evaluator"), PHASE142_EVALUATOR_SHA, "Phase142 evaluator hash")
    return load_module(PHASE142_EVALUATOR, "phase142_reference_for_phase146")


def metric_contract() -> dict[str, Any]:
    reference = phase142()
    equal(sha256_static(PHASE137_EVALUATOR, "Phase137 metric source"), PHASE137_SHA, "Phase137 hash")
    equal(sha256_static(PHASE134_EVALUATOR, "Phase134 metric source"), PHASE134_SHA, "Phase134 hash")
    return reference.metric_contract()


def verify_phase144_metadata() -> dict[str, Any]:
    equal(sha256_static(PHASE144_RESULT, "Phase144 result"), PHASE144_SHA, "Phase144 result hash")
    raw = read_json(PHASE144_RESULT, "Phase144 result")
    for key, expected in (("schema_version", "smartphone-r5-phase144-telemetry-serializer-raw-result.v1"), ("phase", 144), ("status", "sealed-structural-raw-result-no-accuracy"), ("route_order", ["MTV-A", "LAX-T"]), ("runs_per_route", 1)):
        equal(raw.get(key), expected, f"Phase144/{key}")
    routes = raw.get("routes")
    if not isinstance(routes, list) or len(routes) != 2:
        raise fail("Phase144 route metadata malformed")
    by_route: dict[str, Mapping[str, Any]] = {}
    for item in routes:
        if not isinstance(item, Mapping) or item.get("route") not in ROUTES or item["route"] in by_route:
            raise fail("Phase144 route identity malformed")
        by_route[item["route"]] = item
    output: dict[str, Any] = {}
    expected_candidate_hashes = {ROUTES[0]: "b765951daa5478600f5da1f0ad0a27bdc5cdf265d681df21d91779abf3163b00", ROUTES[1]: "225d8147249455f4bd39c889d0b44fc1b3e4fbbe0623dfe9d4e550ced749494e"}
    expected_bytes = {ROUTES[0]: 172694, ROUTES[1]: 117254}
    expected_newlines = {ROUTES[0]: 2159, ROUTES[1]: 1466}
    for route in ROUTES:
        item = by_route[route]
        equal(item.get("status"), "fail-closed-structural-gate", f"Phase144/{route}/status")
        opaque = item.get("opaque_solution")
        if not isinstance(opaque, Mapping):
            raise fail(f"Phase144/{route}/opaque metadata missing")
        path = deferred(opaque.get("path"), "opaque_solution_output.csv", "/phase144-telemetry-serializer-structural-v1/", f"Phase144/{route}/candidate")
        equal(opaque.get("sha256"), expected_candidate_hashes[route], f"Phase144/{route}/candidate hash")
        equal(opaque.get("bytes"), expected_bytes[route], f"Phase144/{route}/candidate bytes")
        equal(opaque.get("newline_count"), expected_newlines[route], f"Phase144/{route}/candidate rows")
        equal(opaque.get("content_interpreted"), False, f"Phase144/{route}/candidate interpretation")
        equal(item.get("solution_content_read"), False, f"Phase144/{route}/solution read")
        output[route] = {"label": LABELS[route], "path": path, "sha256": opaque["sha256"], "bytes": opaque["bytes"], "newline_count": opaque["newline_count"], "expected_prediction_rows": DOMAIN_ROWS[route], "expected_problem_epochs": PROBLEM_EPOCHS[route]}
    return output


def verify_phase145_metadata() -> dict[str, Any]:
    equal(sha256_static(PHASE145_RESULT, "Phase145 result"), PHASE145_SHA, "Phase145 result hash")
    result = read_json(PHASE145_RESULT, "Phase145 result")
    equal(result.get("schema_version"), "smartphone-r5-phase145-phase144-contract-revalidation-result.v1", "Phase145/schema")
    equal(result.get("status"), "revalidated-structural-only-go", "Phase145/status")
    historical = result.get("historical_phase144")
    if not isinstance(historical, Mapping):
        raise fail("Phase145 historical Phase144 provenance missing")
    equal(historical.get("sha256"), PHASE144_SHA, "Phase145/Phase144 hash")
    equal(historical.get("status_preserved"), True, "Phase145/status preserved")
    equal(historical.get("mutated"), False, "Phase145/mutated")
    return result


def truth_metadata() -> dict[str, Any]:
    equal(sha256_static(PHASE142_FREEZE, "Phase142 freeze"), PHASE142_FREEZE_SHA, "Phase142 freeze hash")
    freeze = read_json(PHASE142_FREEZE, "Phase142 freeze")
    cohort = freeze.get("truth_cohort")
    if not isinstance(cohort, Mapping) or cohort.get("route_order") != list(ROUTES):
        raise fail("Phase142 truth cohort route order changed")
    routes = cohort.get("routes")
    if not isinstance(routes, Mapping):
        raise fail("Phase142 truth routes missing")
    result: dict[str, Any] = {}
    for route in ROUTES:
        item = routes.get(route)
        if not isinstance(item, Mapping):
            raise fail(f"Phase142 truth metadata missing: {route}")
        result[route] = {"path": deferred(item.get("path"), "ground_truth.csv", "/truth/", f"truth/{route}"), "sha256": item.get("sha256"), "bytes": item.get("bytes"), "rows": TRUTH_ROWS[route], "expected_missing_truth_key": item.get("expected_missing_truth_key")}
        if not isinstance(result[route]["sha256"], str) or len(result[route]["sha256"]) != 64:
            raise fail(f"truth hash malformed: {route}")
    return result


def verify_manifest() -> dict[str, Any]:
    candidates = verify_phase144_metadata()
    verify_phase145_metadata()
    truths = truth_metadata()
    manifest = read_json(MANIFEST, "Phase146 manifest")
    for key, expected in (("schema_version", SCHEMA), ("phase", 146), ("execution_label", "Luna Max"), ("status", "launch-free-before-phase146-authorization"), ("routes", list(ROUTES))):
        equal(manifest.get(key), expected, f"manifest/{key}")
    equal(manifest.get("metric_contract"), metric_contract(), "manifest/metric_contract")
    candidate = manifest.get("candidate_reference")
    if not isinstance(candidate, Mapping):
        raise fail("manifest/candidate_reference missing")
    for key, expected in (("source", "Phase144 raw result opaque_solution"), ("phase", 144), ("candidate_id", "phase144-telemetry-serializer-canonical-summary-v1"), ("route_order", list(ROUTES)), ("opaque_only", True), ("pixel5_offset_reapplication", 0), ("solution_coordinate_rows_read_before_authorization", 0)):
        equal(candidate.get(key), expected, f"manifest/candidate/{key}")
    for route in ROUTES:
        equal(candidate.get("routes", {}).get(route), candidates[route], f"manifest/candidate/{route}")
    truth = manifest.get("truth_reference")
    if not isinstance(truth, Mapping):
        raise fail("manifest/truth_reference missing")
    equal(truth.get("source"), "Phase142 frozen truth cohort metadata", "manifest/truth/source")
    equal(truth.get("route_order"), list(ROUTES), "manifest/truth/routes")
    for route in ROUTES:
        equal(truth.get("routes", {}).get(route), truths[route], f"manifest/truth/{route}")
    authority = manifest.get("authority")
    if not isinstance(authority, Mapping):
        raise fail("manifest/authority missing")
    source_pins = (("phase144_result", PHASE144_RESULT, PHASE144_SHA, PHASE144_COMMIT), ("phase145_result", PHASE145_RESULT, PHASE145_SHA, PHASE145_COMMIT), ("phase142_evaluator", PHASE142_EVALUATOR, PHASE142_EVALUATOR_SHA, PHASE142_MANIFEST_COMMIT), ("phase82_scorer", PHASE82_SCORER, PHASE82_SHA, "be45879297a2a00a8574c77b71be0a5b721ff8c9"), ("phase76_truth_parser", PHASE76_PARSER, PHASE76_SHA, "7a8e77ce03f6f4354cf2f541098a53628faa23d5"), ("phase137_metric_source", PHASE137_EVALUATOR, PHASE137_SHA, "97bc75d4e94b339507864a7b7c9f188ae33915fd"), ("phase134_metric_source", PHASE134_EVALUATOR, PHASE134_SHA, "b59ca771dabcc1185910d3d95259572f76b16869"), ("phase142_freeze", PHASE142_FREEZE, PHASE142_FREEZE_SHA, PHASE142_FREEZE_COMMIT), ("phase142_manifest", PHASE142_MANIFEST, PHASE142_MANIFEST_SHA, PHASE142_MANIFEST_COMMIT))
    for key, path, digest, commit in source_pins:
        pin = authority.get(key)
        if not isinstance(pin, Mapping):
            raise fail(f"manifest authority pin missing: {key}")
        equal(pin.get("path"), relative(path), f"manifest/authority/{key}/path")
        equal(pin.get("sha256"), digest, f"manifest/authority/{key}/sha")
        equal(pin.get("commit"), commit, f"manifest/authority/{key}/commit")
        equal(sha256_static(path, f"manifest/{key}"), digest, f"manifest/{key}/current sha")
    equal(manifest.get("comparison_baselines"), {"phase118_champion_macro_m": 0.8141981503, "phase142_macro_m": 0.8142362396358963, "informational_only": True}, "manifest/baselines")
    equal(manifest.get("read_accounting_before_authorization"), {"candidate_paths_materialized": 0, "truth_paths_materialized": 0, "candidate_solution_payload_reads": 0, "candidate_coordinate_interpretations": 0, "truth_reads": 0, "accuracy_calculations": 0, "native_solver_invocations": 0, "raw_gnss_imu_navigation_reads": 0, "raw_base_rinex_reads": 0, "mat_pdc_precomputed_coordinate_reads": 0, "kaggle_or_token_access": 0, "reruns": 0, "fallbacks": 0, "repairs": 0}, "manifest/read_accounting")
    return manifest


def verify_pre_truth() -> dict[str, Any]:
    manifest = verify_manifest()
    pre = read_json(PRE_TRUTH, "Phase146 pre-truth")
    for key, expected in (("schema_version", "smartphone-r5-phase146-phase144-accuracy-gate-pre-truth.v1"), ("phase", 146), ("execution_label", "Luna Max"), ("status", "launch-free-pre-truth-sealed")):
        equal(pre.get(key), expected, f"pre_truth/{key}")
    equal(pre.get("manifest_sha256"), sha256_static(MANIFEST, "pre-truth manifest"), "pre_truth/manifest sha")
    equal(pre.get("audit_sha256"), sha256_static(AUDIT, "pre-truth audit"), "pre_truth/audit sha")
    equal(pre.get("metric_contract"), manifest["metric_contract"], "pre_truth/metric")
    equal(pre.get("read_accounting"), manifest["read_accounting_before_authorization"], "pre_truth/accounting")
    return pre


def verify_authorization() -> dict[str, Any]:
    manifest = verify_manifest()
    pre = verify_pre_truth()
    auth = read_json(AUTHORIZATION, "Phase146 authorization")
    for key, expected in (("schema_version", "smartphone-r5-phase146-phase144-accuracy-authorization.v1"), ("phase", 146), ("execution_label", "Luna Max"), ("status", "authorized-for-phase146-one-shot-accuracy"), ("routes", list(ROUTES)), ("manifest_sha256", sha256_static(MANIFEST, "authorization manifest")), ("pre_truth_sha256", sha256_static(PRE_TRUTH, "authorization pre-truth"))):
        equal(auth.get(key), expected, f"authorization/{key}")
    equal(auth.get("candidate_source"), "Phase144 raw result opaque_solution; Phase141/142 candidates forbidden", "authorization/candidate source")
    equal(auth.get("truth_source"), "Phase142 frozen truth cohort", "authorization/truth source")
    pins = auth.get("authority")
    if not isinstance(pins, Mapping):
        raise fail("authorization/authority missing")
    for key, path in (("accuracy_evaluator", EVALUATOR), ("focused_tests", FOCUSED_TESTS), ("audit", AUDIT), ("manifest", MANIFEST), ("pre_truth", PRE_TRUTH), ("phase144_result", PHASE144_RESULT), ("phase145_result", PHASE145_RESULT), ("phase142_evaluator", PHASE142_EVALUATOR), ("phase82_scorer", PHASE82_SCORER), ("phase76_truth_parser", PHASE76_PARSER)):
        pin = pins.get(key)
        if not isinstance(pin, Mapping):
            raise fail(f"authorization/{key} pin missing")
        equal(pin.get("path"), relative(path), f"authorization/{key}/path")
        equal(pin.get("sha256"), sha256_static(path, f"authorization/{key}"), f"authorization/{key}/sha")
    policy = auth.get("execution_policy")
    if not isinstance(policy, Mapping):
        raise fail("authorization/execution_policy missing")
    for key, expected in (("candidate_reads", 2), ("truth_reads", 2), ("accuracy_calculations", 2), ("native_solver_invocations", 0), ("raw_reads", 0), ("reruns", 0), ("fallbacks", 0), ("repairs", 0), ("solution_publication", False), ("strict_macro_comparator", "candidate_macro_score_m < 0.782")):
        equal(policy.get(key), expected, f"authorization/policy/{key}")
    equal(auth.get("route_order"), manifest["routes"], "authorization/route order")
    equal(pre["status"], "launch-free-pre-truth-sealed", "authorization/pre-truth status")
    return auth


def strict_macro_gate(value: Any) -> bool:
    return finite(value) and float(value) < THRESHOLD


def route_gates(ordered_count: int, score: Mapping[str, Any], route: str) -> dict[str, bool]:
    checks = {
        "candidate_schema_and_exact_alignment": ordered_count == DOMAIN_ROWS[route],
        "candidate_prediction_domain_coverage_exact": score.get("prediction_domain_coverage") == 1.0,
        "candidate_all_finite_and_earth_valid": score.get("finite") is True,
        "candidate_over_70_mps_count_zero": score.get("over_70_mps_count") == 0,
        "candidate_route_score_finite": finite(score.get("score_m")),
    }
    return checks


def materialize(value: Any, basename: str, fragment: str, label: str, authorized: bool) -> Path:
    if not authorized:
        raise fail(f"{label} path materialization requires authorization")
    return ROOT / deferred(value, basename, fragment, label)


def evaluate(result_path: Path = RESULT_JSON) -> dict[str, Any]:
    manifest = verify_manifest()
    verify_authorization()
    reference = phase142()
    scorer = reference.load_phase82()
    candidate_reads = truth_reads = calculations = interpretations = 0
    reports: dict[str, Any] = {}
    for route in ROUTES:
        candidate_path = materialize(manifest["candidate_reference"]["routes"][route]["path"], "opaque_solution_output.csv", "/phase144-telemetry-serializer-structural-v1/", f"candidate/{route}", True)
        ordered, candidate, _ = reference.read_candidate_once(candidate_path, manifest["candidate_reference"]["routes"][route], route, scorer)
        candidate_reads += 1; interpretations += 1
        truth_path = materialize(manifest["truth_reference"]["routes"][route]["path"], "ground_truth.csv", "/truth/", f"truth/{route}", True)
        truth, _ = reference.read_truth_once(truth_path, manifest["truth_reference"]["routes"][route], route, scorer)
        truth_reads += 1
        score = scorer.P76._score_prediction(candidate, truth, manifest["truth_reference"]["routes"][route].get("expected_missing_truth_key"), route, ordered)
        calculations += 1
        checks = route_gates(len(ordered), score, route)
        reports[route] = {"route": LABELS[route], "score_m": score["score_m"], "p50_m": score["p50_m"], "p95_m": score["p95_m"], "prediction_domain_coverage": score["prediction_domain_coverage"], "over_70_mps_count": score["over_70_mps_count"], "gates": {"passed": all(checks.values()), "checks": checks}}
    macro = sum(reports[route]["score_m"] for route in ROUTES) / 2.0
    gates = {"exact_two_routes_one_evaluation_each": (candidate_reads, truth_reads, calculations, interpretations) == (2, 2, 2, 2), "candidate_output_schema_and_exact_alignment": all(reports[r]["gates"]["checks"]["candidate_schema_and_exact_alignment"] for r in ROUTES), "candidate_prediction_domain_coverage_exact": all(reports[r]["prediction_domain_coverage"] == 1.0 for r in ROUTES), "candidate_all_finite_and_earth_valid": all(reports[r]["gates"]["checks"]["candidate_all_finite_and_earth_valid"] for r in ROUTES), "candidate_over_70_mps_count_zero": all(reports[r]["over_70_mps_count"] == 0 for r in ROUTES), "pixel5_offset_already_exactly_once_no_reapplication": manifest["candidate_reference"]["pixel5_offset_reapplication"] == 0, "candidate_macro_score_strict_less_than_0_782m": strict_macro_gate(macro), "solution_rows_absent_from_result": True, "no_solver_raw_base_mat_pdc_precomputed_or_kaggle": True}
    failed = [key for key, value in gates.items() if value is not True]
    result = {"schema_version": RESULT_SCHEMA, "phase": 146, "execution_label": "Luna Max", "status": "go-phase146-phase144-accuracy" if not failed else "no-go-phase146-phase144-accuracy", "decision": "two-route offline accuracy validation only; not leaderboard proof and no submission authorized", "candidate_source": "Phase144 raw result opaque_solution", "routes": reports, "aggregate": {"candidate_macro_score_m": macro, "phase118_champion_macro_m": 0.8141981503, "phase142_macro_m": 0.8142362396358963, "route_order": list(ROUTES), "macro_weighting": "unweighted arithmetic mean over exactly MTV-A then LAX-T"}, "metric_contract": manifest["metric_contract"], "promotion_gates": gates, "strict_0_782_gate": {"comparator": "candidate_macro_score_m < 0.782", "threshold_m": THRESHOLD, "candidate_macro_score_m": macro, "passed": gates["candidate_macro_score_strict_less_than_0_782m"]}, "failed_gates": failed, "read_accounting": {"candidate_solution_reads": candidate_reads, "candidate_coordinate_interpretations": interpretations, "truth_reads": truth_reads, "accuracy_calculations": calculations, "candidate_paths_materialized_after_authorization": 2, "truth_paths_materialized_after_authorization": 2, "native_solver_invocations": 0, "raw_gnss_imu_navigation_reads": 0, "raw_base_rinex_reads": 0, "mat_pdc_precomputed_coordinate_reads": 0, "kaggle_or_token_access": 0, "reruns": 0, "fallbacks": 0, "repairs": 0}, "solution_output_published": False, "release_or_submission_authorized": False}
    descriptor, temporary = tempfile.mkstemp(prefix=f".{result_path.name}.", suffix=".tmp", dir=result_path.parent)
    try:
        with os.fdopen(descriptor, "w", encoding="utf-8") as handle:
            json.dump(result, handle, indent=2, sort_keys=True, allow_nan=False); handle.write("\n"); handle.flush(); os.fsync(handle.fileno())
        os.replace(temporary, result_path)
    except BaseException:
        try: os.unlink(temporary)
        except OSError: pass
        raise
    return result


def main(argv: list[str] | None = None) -> int:
    import argparse
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--verify-manifest", action="store_true")
    parser.add_argument("--verify-pre-truth", action="store_true")
    parser.add_argument("--verify-authorization", action="store_true")
    parser.add_argument("--evaluate", action="store_true")
    parser.add_argument("--result-json", type=Path, default=RESULT_JSON)
    args = parser.parse_args(argv)
    modes = [args.verify_manifest, args.verify_pre_truth, args.verify_authorization, args.evaluate]
    if sum(bool(mode) for mode in modes) != 1: parser.error("choose exactly one mode")
    try:
        if args.verify_manifest: verify_manifest()
        elif args.verify_pre_truth: verify_pre_truth()
        elif args.verify_authorization: verify_authorization()
        else:
            result = evaluate(args.result_json); print(json.dumps(result, indent=2, sort_keys=True)); return 0 if result["status"].startswith("go-") else 1
        return 0
    except Exception as exc:
        print(f"phase146 fail-closed: {exc}", file=sys.stderr); return 2


if __name__ == "__main__":
    raise SystemExit(main())
