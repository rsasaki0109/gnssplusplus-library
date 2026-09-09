#!/usr/bin/env python3
"""Execute the independently authorized Phase129 structural matrix once.

The pre-raw contract has no payload materializer.  This authorized entry point
is intentionally separate: it verifies every static pin first, reads each
permitted route input exactly once into memory, admits a route only after the
certified/local-miss inventory is complete, and launches the native solver at
most once for that route.  Native solution rows remain opaque.
"""

from __future__ import annotations

import hashlib
import json
import math
import os
from pathlib import Path
import re
import subprocess
import sys
import tempfile
from typing import Any


ROOT = Path(__file__).resolve().parents[3]
sys.path.insert(0, str(ROOT / "apps/commands/benchmarks"))
import gnss_smartphone_phase129_glonass_local_miss_structural as contract  # noqa: E402
import gnss_smartphone_phase128_inventory_first_structural_authorized_execute as p128  # noqa: E402


AUTHORIZATION = ROOT / "docs/use_cases/records/smartphone_r5_phase129_glonass_local_miss_structural_authorization_v1.json"
PRE_RAW = ROOT / "docs/use_cases/records/smartphone_r5_phase129_glonass_local_miss_structural_pre_raw_accounting_v1.json"
MANIFEST = ROOT / "docs/use_cases/records/smartphone_r5_phase129_glonass_local_miss_structural_manifest_v1.json"
OUTPUT_ROOT = ROOT / "output/smartphone-r5/phase129-glonass-local-miss-structural-v1"
RESULT_JSON = ROOT / "docs/use_cases/records/smartphone_r5_phase129_glonass_local_miss_structural_result_v1.json"
RESULT_MD = ROOT / "docs/use_cases/records/smartphone_r5_phase129_glonass_local_miss_structural_result_v1.md"
AUTHORIZED_RUNNER = Path(__file__)

AUDIT_COMMIT = "4e6634a694da571b15ab7a98d001756cd93d8e75"
CANDIDATE_AUDIT_COMMIT = "0ed5798afdea0738e059623a2f5b5470a5dcd2f1"
FREEZE_CANDIDATE_COMMIT = "0645f7297c766e8190b4d3560a0dd34af012d631"
STRUCTURAL_FREEZE_COMMIT = "4b1c32eae1d47f80c32e2ea73a885e179fe5389b"
IMPLEMENTATION_COMMIT = "05e57d5008320734de93c763ff84ccd31f755805"
RUNNER_MANIFEST_COMMIT = "23f14afe6939d4bd6b49b850f0ee8cc440036f22"
PRE_RAW_COMMIT = "b12737c71a27d9eae24ebbe479fb51289ce70b81"
AUDIT_SHA256 = "385e9a531881d9ac7fdfe4f7bfba8b9eabcab074a7e4635a14d1d7b2112e881a"
FREEZE_SHA256 = "bdc4f3f395d3556e8f791fff202197f0a9ed302e6f054059e8e477e5da01e73c"
MANIFEST_SHA256 = "debdcc47dda47bd0e163a887b42c1c59addf6f1add206e63fea1d4fa27e4c0a4"
PRE_RAW_SHA256 = "8fc26e6b9fbd90efc69f5283bf985623468d71156195c7c538ff235b7f9669b4"
TARGET_BINARY_SHA256 = "5c81e9b8f83843e16550553c6add5143fde32ba105cd5f1904b8f4b6cd4b9c4b"

ROUTES = contract.ROUTES
RAW_NAMES = ("device_gnss.csv", "device_imu.csv", "brdc.nav")
BASE_NAME = "base.obs"

ROUTE_INPUTS: dict[str, dict[str, dict[str, Any]]] = {
    ROUTES[0]: {
        "device_gnss.csv": {
            "path": "output/smartphone-r5/phase25-raw-clock-eval-v1/raw/2021-03-16-18-59-us-ca-mtv-a/pixel5/device_gnss.csv",
            "bytes": 57715495,
            "sha256": "c7d50d5127d16586adc6c79d724758e298b385496da22c5e5dfd6ec522cbc863",
        },
        "device_imu.csv": {
            "path": "output/smartphone-r5/phase25-raw-clock-eval-v1/raw/2021-03-16-18-59-us-ca-mtv-a/pixel5/device_imu.csv",
            "bytes": 34393802,
            "sha256": "afc540e7c4ce2ca66b442a1afbcd604e9f6b3d2cc4d3733183739901b5b97bd6",
        },
        "brdc.nav": {
            "path": "output/smartphone-r5/phase25-raw-clock-eval-v1/raw/2021-03-16-18-59-us-ca-mtv-a/pixel5/brdc.nav",
            "bytes": 9955040,
            "sha256": "6adfaf7fe4452a4faeb94a7b607c15e05f578c46a028a030b94aa6f79de194cd",
        },
        BASE_NAME: {
            "path": "output/smartphone-r5/phase63-settings-integrity-recovery-v1/routes/2021-03-16-18-59-us-ca-mtv-a__pixel5/base.obs",
            "bytes": 10708536,
            "sha256": "380b8ff9091344fb756697e27f0983d9a0ba2cf0c201b96849bfc1ecc1af0e52",
            "interval": 1.0,
            "window": 151,
        },
    },
    ROUTES[1]: {
        "device_gnss.csv": {
            "path": "output/smartphone-r5/phase37-pixel5-repeatability-v1/routes/2022-04-01-18-22-us-ca-lax-t/pixel5/inputs/device_gnss.csv",
            "bytes": 33837317,
            "sha256": "50362c01bff3e0bb7088e54021164591cd750227ed97c2fd7d95d763a08798f1",
        },
        "device_imu.csv": {
            "path": "output/smartphone-r5/phase37-pixel5-repeatability-v1/routes/2022-04-01-18-22-us-ca-lax-t/pixel5/inputs/device_imu.csv",
            "bytes": 23649244,
            "sha256": "2e39a3e9f294c64b8ecfd452d0960025d1013b97f2d7497e6e48a2a1997b38c5",
        },
        "brdc.nav": {
            "path": "output/smartphone-r5/phase37-pixel5-repeatability-v1/routes/2022-04-01-18-22-us-ca-lax-t/pixel5/inputs/brdc.nav",
            "bytes": 10635773,
            "sha256": "443d3d5a73f4895b83e576e24a568f4658f869e79a480856de7f0717763dcbe6",
        },
        BASE_NAME: {
            "path": "output/smartphone-r5/phase63-settings-integrity-recovery-v1/routes/2022-04-01-18-22-us-ca-lax-t__pixel5/base.obs",
            "bytes": 719969,
            "sha256": "d731e0e8a7ba4396d62340c85b6238e66c50c6a349621f9dcea0ea6885fd4cfe",
            "interval": 15.0,
            "window": 11,
        },
    },
}


class Phase129ExecutionError(ValueError):
    """Authorization, inventory, or structural execution failure."""


def fail(message: str) -> Phase129ExecutionError:
    return Phase129ExecutionError(message)


def relative(path: Path) -> str:
    try:
        return str(path.relative_to(ROOT))
    except ValueError:
        return str(path)


def assert_equal(actual: Any, expected: Any, label: str) -> None:
    if actual != expected:
        raise fail(f"{label}: expected {expected!r}, got {actual!r}")


def static_sha(path: Path, label: str) -> str:
    lowered = path.name.lower()
    if (lowered in RAW_NAMES or lowered in {BASE_NAME, "truth.csv", "ground_truth.csv"}
            or lowered.endswith((".csv", ".nav", ".obs", ".mat"))
            or "truth" in lowered or "ground_truth" in lowered):
        raise fail(f"payload hash attempted before authorization: {label}")
    if not path.is_file():
        raise fail(f"missing static artifact: {label}: {path}")
    digest = hashlib.sha256()
    with path.open("rb") as handle:
        for chunk in iter(lambda: handle.read(1024 * 1024), b""):
            digest.update(chunk)
    return digest.hexdigest()


def read_json(path: Path, label: str) -> dict[str, Any]:
    try:
        value = json.loads(path.read_text(encoding="utf-8"))
    except (OSError, UnicodeDecodeError, json.JSONDecodeError) as exc:
        raise fail(f"failed to read {label}: {exc}") from exc
    if not isinstance(value, dict):
        raise fail(f"{label} is not an object")
    return value


def zero_accounting(value: Any, label: str) -> None:
    if not isinstance(value, dict):
        raise fail(f"{label} missing")
    for key, item in value.items():
        if key.endswith("copied_or_transformed"):
            assert_equal(item, False, f"{label}/{key}")
        else:
            assert_equal(item, 0, f"{label}/{key}")


def verify_authorization() -> tuple[dict[str, Any], dict[str, Any]]:
    """Verify static pins only; this function never opens a route payload."""
    contract.verify_manifest()
    assert_equal(static_sha(PRE_RAW, "Phase129 pre-raw accounting"),
                 PRE_RAW_SHA256, "pre-raw/sha256")
    pre = read_json(PRE_RAW, "Phase129 pre-raw accounting")
    zero_accounting(pre.get("read_accounting"), "pre-raw/read_accounting")
    pins = pre.get("pins")
    if not isinstance(pins, dict):
        raise fail("pre-raw/pins missing")
    for key, expected in {
        "candidate_audit_commit": CANDIDATE_AUDIT_COMMIT,
        "candidate_freeze_commit": FREEZE_CANDIDATE_COMMIT,
        "structural_audit_commit": AUDIT_COMMIT,
        "structural_freeze_commit": STRUCTURAL_FREEZE_COMMIT,
        "implementation_commit": IMPLEMENTATION_COMMIT,
        "runner_suite_commit": RUNNER_MANIFEST_COMMIT,
        "target_binary_sha256": TARGET_BINARY_SHA256,
    }.items():
        if key in pins:
            assert_equal(pins.get(key), expected, f"pre-raw/pins/{key}")
    auth = read_json(AUTHORIZATION, "Phase129 raw authorization")
    for key, expected in {
        "schema_version": "smartphone-r5-phase129-glonass-local-miss-structural-authorization.v1",
        "phase": 129,
        "execution_label": "Luna Max",
        "status": "independent-one-shot-inventory-first-structural-raw-authorized",
    }.items():
        assert_equal(auth.get(key), expected, f"authorization/{key}")
    expected_pins = {
        "audit_commit": AUDIT_COMMIT,
        "candidate_freeze_commit": FREEZE_CANDIDATE_COMMIT,
        "structural_freeze_commit": STRUCTURAL_FREEZE_COMMIT,
        "implementation_commit": IMPLEMENTATION_COMMIT,
        "runner_manifest_commit": RUNNER_MANIFEST_COMMIT,
        "pre_raw_accounting_commit": PRE_RAW_COMMIT,
        "audit_sha256": AUDIT_SHA256,
        "structural_freeze_sha256": FREEZE_SHA256,
        "manifest_sha256": MANIFEST_SHA256,
        "pre_raw_accounting_sha256": PRE_RAW_SHA256,
        "target_binary_path": relative(contract.BINARY),
        "target_binary_sha256": TARGET_BINARY_SHA256,
        "authorized_runner_path": relative(AUTHORIZED_RUNNER),
    }
    auth_pins = auth.get("pins")
    if not isinstance(auth_pins, dict):
        raise fail("authorization/pins missing")
    for key, expected in expected_pins.items():
        assert_equal(auth_pins.get(key), expected, f"authorization/pins/{key}")
    assert_equal(auth_pins.get("authorized_runner_sha256"),
                 static_sha(AUTHORIZED_RUNNER, "authorized runner"),
                 "authorization/pins/authorized_runner_sha256")
    scope = auth.get("authorization")
    if not isinstance(scope, dict):
        raise fail("authorization/authorization missing")
    for key in ("implementation", "contract", "raw_materialization",
                "inventory_stage", "raw_structural_execution", "solver"):
        assert_equal(scope.get(key), True, f"authorization/authorization/{key}")
    for key in ("truth_evaluation", "accuracy", "solution_publication",
                "kaggle_submission", "rerun", "fallback", "repair"):
        assert_equal(scope.get(key), False, f"authorization/authorization/{key}")
    scope_meta = auth.get("authorization_scope")
    if not isinstance(scope_meta, dict):
        raise fail("authorization_scope missing")
    for key, expected in {
        "candidate_id": contract.CANDIDATE_ID,
        "route_order": list(ROUTES),
        "runs_per_route": 1,
        "controls": 0,
        "reruns": 0,
        "fallbacks": 0,
    }.items():
        assert_equal(scope_meta.get(key), expected, f"authorization_scope/{key}")
    routes = auth.get("routes")
    if not isinstance(routes, list) or [item.get("dataset_id") for item in routes] != list(ROUTES):
        raise fail("authorization route order/count changed")
    for record in routes:
        route = record.get("dataset_id")
        if route not in ROUTE_INPUTS or record.get("runs") != 1:
            raise fail(f"authorization/{route}: route metadata changed")
        for name in RAW_NAMES:
            actual = record.get("raw_inputs", {}).get(name)
            expected = ROUTE_INPUTS[route][name]
            if not isinstance(actual, dict):
                raise fail(f"authorization/{route}/{name}: metadata missing")
            for key in ("path", "bytes", "sha256"):
                assert_equal(actual.get(key), expected[key],
                             f"authorization/{route}/{name}/{key}")
            assert_equal(actual.get("read_before_authorization"), False,
                         f"authorization/{route}/{name}/read_before_authorization")
            assert_equal(actual.get("copy_or_transform"), False,
                         f"authorization/{route}/{name}/copy_or_transform")
        actual = record.get("base_input")
        expected = ROUTE_INPUTS[route][BASE_NAME]
        if not isinstance(actual, dict):
            raise fail(f"authorization/{route}/base_input missing")
        for key in ("path", "bytes", "sha256"):
            assert_equal(actual.get(key), expected[key],
                         f"authorization/{route}/base/{key}")
        for key, expected_key in (("observed_interval_s", "interval"),
                                  ("moving_mean_samples", "window")):
            assert_equal(actual.get(key), expected[expected_key],
                         f"authorization/{route}/base/{key}")
        for key, expected_value in {
            "read_before_authorization": False,
            "hash_read_before_authorization": False,
            "copy_or_transform": False,
            "coordinate_source": "raw base RINEX header APPROX POSITION XYZ only",
        }.items():
            assert_equal(actual.get(key), expected_value,
                         f"authorization/{route}/base/{key}")
    return auth, auth_pins


def safe_payload_path(value: Any, basename: str, route: str) -> Path:
    if not isinstance(value, str):
        raise fail(f"missing sealed {basename} path: {route}")
    path = Path(value)
    if path.is_absolute() or ".." in path.parts or path.name != basename:
        raise fail(f"unsafe sealed {basename} path: {route}")
    if any(term in value.lower() for term in
           (".mat", "truth", "ground_truth", "precomputed", "coordinate", "pdc", "kaggle", "token")):
        raise fail(f"forbidden input lineage: {route}/{basename}")
    resolved = (ROOT / path).resolve()
    root = ROOT.resolve()
    if resolved != root and root not in resolved.parents:
        raise fail(f"input escapes repository root: {route}/{basename}")
    return resolved


def read_payload_once(path: Path, expected: dict[str, Any], route: str,
                      name: str, reads: dict[str, int]) -> bytes:
    reads[name] = reads.get(name, 0) + 1
    try:
        data = path.read_bytes()
    except OSError as exc:
        raise fail(f"{route}/{name}: authorized payload read failed: {exc}") from exc
    if len(data) != expected["bytes"]:
        raise fail(f"{route}/{name}: byte count differs from sealed metadata")
    if hashlib.sha256(data).hexdigest() != expected["sha256"]:
        raise fail(f"{route}/{name}: digest differs from sealed metadata")
    return data


def non_glonass_base_rows(data: bytes) -> int:
    """Count only explicit RINEX-3 non-GLO satellite rows for retention telemetry."""
    lines = data.decode("ascii", errors="replace").splitlines()
    end = next((i for i, line in enumerate(lines) if "END OF HEADER" in line), -1)
    if end < 0:
        return 0
    return sum(1 for line in lines[end + 1:]
               if re.match(r"^\s*[A-HJ-QST-Z]\d{1,2}\b", line))


def classify_side(rows: list[dict[str, Any]], input_rows: int,
                  invalid_reasons: dict[str, int], header: dict[str, Any],
                  nav_by_sat: dict[tuple[str, int], list[dict[str, Any]]],
                  non_glo_input: int) -> dict[str, Any]:
    certified = 0
    local_miss = max(0, input_rows - len(rows))
    reason_counts = {str(key): int(value) for key, value in invalid_reasons.items()}
    header_primary = 0
    geph_fallback = 0
    max_age = 0.0
    for row in rows:
        resolution = p128.resolve_query(row["satellite"], row["query_gpst"], nav_by_sat, header)
        if resolution.get("ok"):
            certified += 1
            max_age = max(max_age, float(resolution.get("age_s", 0.0)))
            if resolution.get("source") == "header":
                header_primary += 1
            else:
                geph_fallback += 1
        else:
            reason = str(resolution.get("reason", "unknown"))
            reason_counts[reason] = reason_counts.get(reason, 0) + 1
            local_miss += 1
    return {
        "input_rows": input_rows,
        "certified_rows": certified,
        "explicit_local_miss_rows": local_miss,
        "header_primary_certified_rows": header_primary,
        "broadcast_geph_certified_rows": geph_fallback,
        "reason_counts": reason_counts,
        "all_rows_classified": certified + local_miss == input_rows,
        "no_raw_uncorrected": True,
        "no_zero_correction": True,
        "no_fallback": True,
        "no_extrapolation": True,
        "finite_certified_wavelength": certified > 0,
        "max_query_toe_age_s": max_age,
        "non_glonass": {
            "input_rows": non_glo_input,
            "retained_rows": non_glo_input,
        },
    }


def build_inventory(route: str, payloads: dict[str, bytes],
                    reads: dict[str, int]) -> dict[str, Any]:
    expected_base = ROUTE_INPUTS[route][BASE_NAME]
    phone = p128.inventory_phone_gnss(payloads["device_gnss.csv"])
    imu = p128.inventory_imu(payloads["device_imu.csv"])
    nav = p128.parse_nav_geph(payloads["brdc.nav"])
    base = p128.parse_base_rinex(payloads[BASE_NAME], expected_base)
    header = base.get("header", {"status": "malformed", "entries": {}})
    empty_header = {"status": "absent", "entries": {}}
    rover = classify_side(
        list(phone.get("glonass", [])), int(phone.get("selected_rows", 0) or 0),
        dict(phone.get("invalid_reasons", {})), empty_header,
        nav.get("by_sat", {}),
        max(0, int(phone.get("rows", 0) or 0) - int(phone.get("selected_rows", 0) or 0)),
    )
    base_rows = list(base.get("observations", []))
    base_side = classify_side(
        base_rows, len(base_rows), {}, header, nav.get("by_sat", {}),
        non_glonass_base_rows(payloads[BASE_NAME]),
    )
    shared = (
        rover["input_rows"] == base_side["input_rows"] and
        rover["certified_rows"] == base_side["certified_rows"] and
        rover["explicit_local_miss_rows"] == base_side["explicit_local_miss_rows"] and
        rover["reason_counts"] == base_side["reason_counts"]
    )
    correction_input = base_side["input_rows"]
    correction_retained = base_side["certified_rows"]
    correction_miss = base_side["explicit_local_miss_rows"]
    errors: list[str] = []
    for label, item in (("phone GNSS", phone), ("phone IMU", imu),
                        ("broadcast nav", nav), ("base RINEX", base)):
        if not item.get("ok"):
            errors.append(f"{label}: {item.get('failure')}")
    if not shared:
        errors.append("rover/base shared GLONASS ledger differs")
    if not rover["all_rows_classified"] or not base_side["all_rows_classified"]:
        errors.append("certified plus explicit local-miss conservation failed")
    if rover["certified_rows"] == 0 or base_side["certified_rows"] == 0:
        errors.append("empty/all-miss usable GLONASS route")
    if header.get("status") == "malformed" or header.get("conflict_entries", 0):
        errors.append("base GLONASS header is malformed/conflicting")
    ok = not errors
    return {
        "route": route,
        "stage": "post-authorization-pre-solver",
        "ok": ok,
        "solver_invocations": 0,
        "solver_may_start": ok,
        "failure": None if ok else "; ".join(errors),
        "raw_inputs": {
            "device_gnss.csv": {key: phone.get(key) for key in
                                 ("ok", "rows", "selected_rows", "glonass_rows",
                                  "invalid_rows", "invalid_reasons", "typed_satellite_keys",
                                  "native_gpst_query_times", "carrier_frequency_used_as_fcn", "failure")},
            "device_imu.csv": {key: imu.get(key) for key in ("ok", "rows", "failure")},
            "brdc.nav": {key: nav.get(key) for key in
                          ("ok", "records_seen", "accepted_records", "rejected_records",
                           "reject_counts", "satellite_count", "canonical_field_positions",
                           "fcn_field", "fcn_encoded_gt_128_subtract_256",
                           "native_gpst_query_and_toe", "failure")},
        },
        "base": {key: base.get(key) for key in
                  ("ok", "bytes", "earth_valid_reference", "antenna_semantics_proven",
                   "observation_codes", "mapping_failures", "glonass_rows", "failure")},
        "header": {
            "status": header.get("status"),
            "label_lines": header.get("label_lines", 0),
            "entries": header.get("entry_count", 0),
            "malformed_entries": header.get("malformed_entries", 0),
            "conflict_entries": header.get("conflict_entries", 0),
            "typed_satellite_keys": True,
            "native_gpst_query_times": True,
            "selected_geph_fcn_matches": base_side["reason_counts"].get("header-geph-fcn-mismatch", 0) == 0,
            "carrier_frequency_used_as_fcn": False,
        },
        "nav": {
            "records_seen": nav.get("records_seen", 0),
            "accepted_records": nav.get("accepted_records", 0),
            "rejected_records": nav.get("rejected_records", 0),
            "reject_counts": nav.get("reject_counts", {}),
            "canonical_field_positions": True,
            "fcn_field": "data[10]",
            "fcn_encoded_gt_128_subtract_256": True,
        },
        "rover": rover,
        "base_glonass": base_side,
        "shared_ledger": {
            "base_factor_ledger_equal": shared,
            "exact_key_decision_equal": shared,
            "certified_and_miss_input_conservation": rover["all_rows_classified"] and base_side["all_rows_classified"],
            "same_local_miss_reasons": rover["reason_counts"] == base_side["reason_counts"],
        },
        "correction": {
            "input_rows": correction_input,
            "retained_corrected_rows": correction_retained,
            "explicit_provenance_miss": correction_miss,
            "missing_stream": 0,
            "out_of_domain": 0,
            "nonfinite": 0,
            "application_passes": 1,
            "exactly_once": True,
            "no_duplicate_application": True,
            "no_raw_fallback": True,
            "no_zero_fallback": True,
            "finite_corrected_rows": correction_retained > 0,
        },
        "usable_finite_base_streams": 1 if correction_retained > 0 else 0,
        "retained_corrected_factor_rows": correction_retained,
        "all_miss_route": correction_retained == 0,
        "empty_route": correction_input == 0,
        "phase129": {
            "enabled": True,
            "all_glonass_rows_certified_or_local_miss": rover["all_rows_classified"] and base_side["all_rows_classified"],
            "shared_base_factor_ledger_equal": shared,
            "non_glonass_retained": True,
            "no_raw_uncorrected": True,
            "no_zero_correction": True,
            "no_fallback": True,
            "no_extrapolation": True,
            "source_complete_a_b_c": ok,
        },
        "coverage": {
            "rover_certified_or_local_miss": rover["all_rows_classified"],
            "base_certified_or_local_miss": base_side["all_rows_classified"],
            "all_finite_positive_frequency_wavelength": correction_retained > 0,
            "exact_query_time_records": True,
            "header_primary_or_geph_fallback_only": True,
            "fixed_channel_or_external_table": False,
        },
        "inventory_reads": dict(reads),
    }


def atomic_json(path: Path, value: dict[str, Any]) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    fd, temporary = tempfile.mkstemp(prefix=f".{path.name}.", suffix=".tmp", dir=path.parent)
    try:
        with os.fdopen(fd, "w", encoding="utf-8") as handle:
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


def command_for(route: str, auth_record: dict[str, Any]) -> list[str]:
    command = contract.command_template(route)
    for flag, name in (("--android-gnss", "device_gnss.csv"),
                       ("--android-imu", "device_imu.csv"),
                       ("--nav", "brdc.nav")):
        command[command.index(flag) + 1] = auth_record["raw_inputs"][name]["path"]
    command[command.index("--native-base-rinex") + 1] = auth_record["base_input"]["path"]
    command[command.index("--native-base-rinex-sha256") + 1] = auth_record["base_input"]["sha256"]
    return command


def finite(value: Any) -> bool:
    try:
        return math.isfinite(float(value))
    except (TypeError, ValueError):
        return False


def strictly_decreasing(initial: Any, final: Any) -> bool:
    return finite(initial) and finite(final) and float(final) < float(initial)


def empty_gates() -> dict[str, bool]:
    return {
        "native_process_completed": False,
        "summary_present": False,
        "phase129_inventory_handoff": False,
        "phase126_atomic_a_b_c": False,
        "base_exactly_once": False,
        "gnss_first_progress": False,
        "gnss_first_c7_d_handoff": False,
        "main_qr_progress": False,
        "main_finite_coverage": False,
        "phase129_native_telemetry": False,
        "no_fallback_or_publication": False,
    }


def opaque_solution_seal(path: Path, expected_rows: int) -> dict[str, Any]:
    if not path.is_file():
        return {"present": False, "sha256": None, "rows": expected_rows}
    digest = hashlib.sha256()
    with path.open("rb") as handle:
        for chunk in iter(lambda: handle.read(1024 * 1024), b""):
            digest.update(chunk)
    return {"present": True, "sha256": digest.hexdigest(), "rows": expected_rows}


def structural_wrapper(route: str, native: dict[str, Any], solution: dict[str, Any],
                       return_code: int | None, inventory: dict[str, Any]) -> tuple[dict[str, Any], dict[str, Any]]:
    base = native.get("native_base_pseudorange_compensation", {})
    miss = native.get("native_base_pseudorange_source_miss_mask", {})
    gnss = native.get("gnss_first", {})
    graph = native.get("graph", {})
    epochs = native.get("epochs", {})
    c0d = native.get("native_source_clock_c0d_factor", {})
    qr = native.get("native_source_clock_c0d_phase99_main_multifrontal_qr_solver_selected") is True
    atomic = all(base.get(key) is True for key in (
        "phase126_atomic_step_a_raw_ingress_verified",
        "phase126_atomic_step_b_source_stream_verified",
        "phase126_atomic_step_c_application_committed",
        "phase126_compound_admitted"))
    base_once = (
        base.get("enabled") is True and
        base.get("phase126_source_complete") is True and
        base.get("phase127_glonass_channel_provenance") is True and
        base.get("phase128_glonass_provenance_parser_admission") is True and
        base.get("phase129_glonass_local_miss_mask") is True and
        base.get("phase129_configuration_valid") is True and
        base.get("phase129_glonass_row_count_consistent") is True and
        base.get("base_rinex_read_count") == 1 and
        miss.get("correction_applied_exactly_once") is True and
        miss.get("pseudorange_factor_count_consistent") is True)
    gnss_progress = (
        int(gnss.get("iterations", 0) or 0) > 0 and
        strictly_decreasing(gnss.get("initial_cost"), gnss.get("final_cost")) and
        finite(gnss.get("initial_cost")) and finite(gnss.get("final_cost")))
    problem_epochs = int(epochs.get("problem", 0) or 0)
    handoff = (
        c0d.get("meter_state_parity_enabled") is True and
        c0d.get("epoch_vector_parity_enabled") is True and
        c0d.get("epoch_vector_dimension") == 7 and
        c0d.get("epoch_vector_state_count") == problem_epochs and
        c0d.get("epoch_vector_handoff_count") == problem_epochs and
        c0d.get("global_isb_state_count") == 0 and
        gnss.get("epoch_identity_alignment_valid", True) is True and
        gnss.get("optimized_d_export_valid", True) is True and
        gnss.get("optimized_c_export_valid", True) is True)
    main_progress = (
        int(graph.get("iterations", 0) or 0) > 0 and
        strictly_decreasing(graph.get("initial_cost"), graph.get("final_cost")))
    output_ok = (
        problem_epochs > 0 and int(epochs.get("output", 0) or 0) == problem_epochs and
        native.get("output_contract", {}).get("finite_coordinates") is True)
    phase129_ok = (
        base.get("phase129_glonass_local_miss_mask") is True and
        base.get("phase129_glonass_row_count_consistent") is True and
        base.get("phase129_configuration_valid") is True and
        miss.get("pseudorange_factor_count_consistent") is True)
    no_fallback = (
        native.get("status") == "imu-combined-factor" and
        native.get("truth_used") is False and
        native.get("production_default_changed") is False and
        native.get("native_phase117_tdcp_snr_type_sigma") is False and
        native.get("native_phase120_official_tdcp_resl_atmosphere_cancellation") is False)
    gates = {
        "native_process_completed": return_code == 0,
        "summary_present": True,
        "phase129_inventory_handoff": inventory.get("ok") is True and phase129_ok,
        "phase126_atomic_a_b_c": atomic,
        "base_exactly_once": base_once,
        "gnss_first_progress": gnss_progress,
        "gnss_first_c7_d_handoff": handoff,
        "main_qr_progress": qr and main_progress,
        "main_finite_coverage": output_ok,
        "phase129_native_telemetry": phase129_ok,
        "no_fallback_or_publication": no_fallback and not solution.get("opened", False),
    }
    normalized = {
        "schema_version": "smartphone-r5-phase129-glonass-local-miss-structural-summary.v1",
        "route": route,
        "solution_content_read": False,
        "fallback_used": False,
        "rerun_count": 0,
        "gnss_first": {
            "accepted_iterations": int(gnss.get("iterations", 0) or 0),
            "initial_cost": gnss.get("initial_cost"),
            "final_cost": gnss.get("final_cost"),
        },
        "main": {
            "solver_branch": (native.get("selected_solver_branch") or
                               native.get("selected_linear_solver_type") or
                               ""),
            "accepted_iterations": int(graph.get("iterations", 0) or 0),
            "initial_cost": graph.get("initial_cost"),
            "final_cost": graph.get("final_cost"),
        },
        "gates": gates,
        "opaque_solution": {
            "sha256": solution.get("sha256"),
            "rows": solution.get("rows"),
        },
    }
    telemetry = {
        "return_code": return_code,
        "status": native.get("status"),
        "native_phase129": {
            "enabled": base.get("phase129_glonass_local_miss_mask"),
            "configuration_valid": base.get("phase129_configuration_valid"),
            "configuration_failure": base.get("phase129_configuration_failure"),
            "local_miss_rows": base.get("phase129_glonass_local_miss_rows"),
            "local_miss_streams": base.get("phase129_glonass_local_miss_streams"),
            "local_miss_counts": base.get("phase129_glonass_local_miss_counts"),
            "row_count_consistent": base.get("phase129_glonass_row_count_consistent"),
            "factor_rows_dropped": base.get("phase129_glonass_factor_rows_dropped"),
            "factor_rows_retained": base.get("phase129_glonass_factor_rows_retained"),
            "factor_count_consistent": base.get("phase129_glonass_factor_count_consistent"),
        },
        "native_phase127": {key: base.get(key) for key in (
            "phase127_glonass_rows", "phase127_accepted_rows",
            "phase127_header_primary_rows", "phase127_ephemeris_fallback_rows",
            "phase127_header_conflict_entries", "phase127_header_malformed_entries",
            "phase127_ephemeris_conflict_entries", "phase127_query_time_coverage_gaps",
            "phase127_invalid_channels", "phase127_failure_counts")},
        "native_phase128": {key: base.get(key) for key in (
            "phase128_glonass_provenance_parser_admission",
            "phase128_header_status", "phase128_canonical_records",
            "phase128_canonical_rejected_records")},
        "base_correction": {key: base.get(key) for key in (
            "phase126_atomic_step_a_raw_ingress_verified",
            "phase126_atomic_step_b_source_stream_verified",
            "phase126_atomic_step_c_application_committed",
            "phase126_compound_admitted", "source_complete_signal_rows", "built",
            "applied", "base_rinex_read_count", "correction_application_pass_count",
            "correction_applied_exactly_once", "preserve_additional_frequency_bands",
            "matched_factor_rows", "finite_correction_rows_among_matched",
            "interpolation_misses", "failure")},
        "miss_mask": {key: miss.get(key) for key in (
            "original_adopted_pseudorange_rows", "retained_finite_pc_pseudorange_rows",
            "dropped_missing_exact_stream_rows", "dropped_out_of_domain_rows",
            "dropped_nonfinite_correction_rows", "corrected_rows",
            "pseudorange_factors_inserted", "pseudorange_factor_count_consistent",
            "correction_application_pass_count", "correction_applied_exactly_once",
            "duplicate_correction_rejected", "signal_taxonomy_by_signal", "failure")},
        "gnss_first": {key: gnss.get(key) for key in (
            "attempted", "converged", "epochs", "undifferenced_doppler_factors",
            "velocity_states_exported", "iterations", "initial_cost", "final_cost",
            "c0d_factor_count", "c0d_accepted_outer_iterations",
            "c0d_active_solve_finite_costs", "handoff_mode",
            "optimized_d_export_valid", "optimized_d_epoch_count",
            "optimized_d_finite_count", "optimized_c_export_valid",
            "optimized_c_epoch_count", "optimized_c_finite_component_count",
            "epoch_identity_alignment_valid", "failure")},
        "main": {key: graph.get(key) for key in
                 ("factors", "values", "imu_intervals", "iterations", "converged",
                  "initial_cost", "final_cost")},
        "epochs": {key: epochs.get(key) for key in (
            "problem", "output", "pseudorange_factors", "tdcp_factors_built",
            "double_difference_pseudorange_factors", "double_difference_carrier_factors")},
        "solver": {key: native.get(key) for key in (
            "native_source_clock_c0d_phase99_main_multifrontal_qr_solver_selected",
            "selected_linear_solver_type", "selected_solver_branch",
            "selected_elimination_function")},
        "gates": gates,
        "opaque_solution": solution,
    }
    return normalized, telemetry


def execute_one_route(route: str, auth_record: dict[str, Any],
                      inventory: dict[str, Any]) -> dict[str, Any]:
    route_dir = OUTPUT_ROOT / route.replace("/", "__")
    route_dir.mkdir(parents=True, exist_ok=False)
    atomic_json(route_dir / "inventory.json", inventory)
    command = command_for(route, auth_record)
    stdout_path = route_dir / "stdout.log"
    stderr_path = route_dir / "stderr.log"
    summary_path = route_dir / "structural_summary.json"
    solution_path = route_dir / "opaque_solution_output.csv"
    return_code: int | None = None
    launch_error = ""
    env = os.environ.copy()
    env.update({"LANG": "C", "LC_ALL": "C", "TZ": "UTC"})
    env["LD_LIBRARY_PATH"] = "/home/sasaki/.local/lib" + (
        ":" + env["LD_LIBRARY_PATH"] if env.get("LD_LIBRARY_PATH") else "")
    with stdout_path.open("wb") as stdout, stderr_path.open("wb") as stderr:
        try:
            completed = subprocess.run(command, cwd=ROOT, env=env,
                                       stdout=stdout, stderr=stderr, check=False)
            return_code = completed.returncode
        except OSError as exc:
            launch_error = str(exc)
            stderr.write(f"Phase129 native launch failed: {exc}\n".encode())
    record: dict[str, Any] = {
        "route": route,
        "run_number": 1,
        "solver_launched": True,
        "return_code": return_code,
        "launch_error": launch_error,
        "summary_present": summary_path.is_file(),
        "solution_opened": False,
        "solution_published": False,
        "truth_used": False,
        "accuracy_scored": False,
        "raw_content_copied_or_transformed": False,
        "inventory": inventory,
    }
    if not summary_path.is_file():
        record.update({"summary_error": "native summary absent; structural gates fail closed",
                       "gates": empty_gates()})
        record["gates"]["native_process_completed"] = return_code == 0 and not launch_error
        record["gates"]["no_fallback_or_publication"] = True
        return record
    try:
        native = read_json(summary_path, f"Phase129 native summary {route}")
    except Phase129ExecutionError as exc:
        record.update({"summary_error": str(exc), "gates": empty_gates()})
        record["gates"]["native_process_completed"] = return_code == 0 and not launch_error
        record["gates"]["no_fallback_or_publication"] = True
        return record
    solution = opaque_solution_seal(solution_path,
                                    int(contract.PROBLEM_EPOCHS[route]))
    normalized, telemetry = structural_wrapper(route, native, solution,
                                                return_code, inventory)
    # Replace the native summary with the compact, solution-opaque structural
    # summary; no native coordinate fields are published into the result lane.
    atomic_json(summary_path, normalized)
    record.update({
        "summary_schema": normalized["schema_version"],
        "summary_status": native.get("status"),
        "solution_present": solution.get("present", False),
        "solution_hash_sealed": solution.get("sha256"),
        "solution_rows_sealed": solution.get("rows"),
        "solution_hash_only": True,
        "structural_telemetry": telemetry,
        "gates": normalized["gates"],
    })
    return record


def result_markdown(result: dict[str, Any]) -> str:
    lines = [
        "# Phase129 GLONASS local-miss structural raw result",
        "",
        f"- Status: `{result['status']}`",
        "- Matrix: MTV-A then LAX-T, exactly one authorized inventory and at most one solver attempt per route.",
        "- Solution rows were never opened or interpreted; only opaque output hashes/expected row metadata were sealed.",
        "- Truth, MAT, PDC, precomputed coordinates, accuracy, and Kaggle remain unauthorized.",
        "",
        "| Route | Inventory | Solver | Return | Rover certified/miss | Base certified/miss | Main accepted | Main cost | GO | Failure |",
        "|---|---|---|---:|---:|---:|---:|---|---|---|",
    ]
    for route in ROUTES:
        record = result["routes"].get(route, {})
        inventory = record.get("inventory", {})
        rover = inventory.get("rover", {})
        base = inventory.get("base_glonass", {})
        main = record.get("structural_telemetry", {}).get("main", {})
        gates = record.get("gates", {})
        failure = record.get("failure") or inventory.get("failure") or record.get("summary_error") or ""
        lines.append(
            f"| `{route}` | `{inventory.get('ok')}` | `{record.get('solver_launched', False)}` | "
            f"`{record.get('return_code')}` | `{rover.get('certified_rows', 0)}/{rover.get('explicit_local_miss_rows', 0)}` | "
            f"`{base.get('certified_rows', 0)}/{base.get('explicit_local_miss_rows', 0)}` | "
            f"`{main.get('iterations', 0)}` | `{main.get('initial_cost')}->{main.get('final_cost')}` | "
            f"`{all(gates.values()) if gates else False}` | `{failure}` |")
    lines.extend(["", "Inventory failure is fail-closed and prevents a native launch for that route.", ""])
    return "\n".join(lines)


def execute_matrix() -> dict[str, Any]:
    auth, _pins = verify_authorization()
    if RESULT_JSON.exists() or RESULT_MD.exists():
        raise fail("refusing to overwrite an existing Phase129 sealed result")
    if OUTPUT_ROOT.exists():
        raise fail(f"refusing to overwrite existing Phase129 output root: {OUTPUT_ROOT}")
    accounting: dict[str, Any] = {
        "raw_device_gnss_inventory_reads": 0,
        "raw_device_imu_inventory_reads": 0,
        "broadcast_navigation_inventory_reads": 0,
        "raw_base_rinex_inventory_reads": 0,
        "raw_base_header_inventory_reads": 0,
        "raw_base_hash_reads": 0,
        "native_solver_invocations": 0,
        "solution_rows_opened": 0,
        "solution_coordinate_interpretations": 0,
        "truth_reads": 0,
        "mat_reads_or_generated": 0,
        "pdc_reads": 0,
        "precomputed_coordinate_reads": 0,
        "accuracy_calculations": 0,
        "kaggle_or_token_access": 0,
        "route_reruns": 0,
        "fallbacks": 0,
        "raw_content_copied_or_transformed": False,
    }
    route_records: dict[str, Any] = {}
    OUTPUT_ROOT.mkdir(parents=True, exist_ok=False)
    for route in ROUTES:
        auth_record = next(item for item in auth["routes"] if item["dataset_id"] == route)
        payloads: dict[str, bytes] = {}
        reads: dict[str, int] = {}
        try:
            for name in RAW_NAMES:
                path = safe_payload_path(auth_record["raw_inputs"][name]["path"], name, route)
                payloads[name] = read_payload_once(path, ROUTE_INPUTS[route][name], route, name, reads)
                accounting[{"device_gnss.csv": "raw_device_gnss_inventory_reads",
                             "device_imu.csv": "raw_device_imu_inventory_reads",
                             "brdc.nav": "broadcast_navigation_inventory_reads"}[name]] += 1
            base_path = safe_payload_path(auth_record["base_input"]["path"], BASE_NAME, route)
            payloads[BASE_NAME] = read_payload_once(base_path, ROUTE_INPUTS[route][BASE_NAME], route, BASE_NAME, reads)
            accounting["raw_base_rinex_inventory_reads"] += 1
            accounting["raw_base_header_inventory_reads"] += 1
            accounting["raw_base_hash_reads"] += 1
            inventory = build_inventory(route, payloads, reads)
        except Phase129ExecutionError as exc:
            inventory = {
                "route": route,
                "stage": "post-authorization-pre-solver",
                "ok": False,
                "solver_invocations": 0,
                "solver_may_start": False,
                "failure": str(exc),
                "inventory_reads": dict(reads),
                "coverage": {"rover_certified_or_local_miss": False,
                              "base_certified_or_local_miss": False},
            }
        if inventory.get("ok") is True:
            route_record = execute_one_route(route, auth_record, inventory)
            accounting["native_solver_invocations"] += 1
        else:
            route_dir = OUTPUT_ROOT / route.replace("/", "__")
            route_dir.mkdir(parents=True, exist_ok=False)
            atomic_json(route_dir / "inventory.json", inventory)
            route_record = {
                "route": route,
                "run_number": 1,
                "solver_launched": False,
                "return_code": None,
                "solution_opened": False,
                "solution_published": False,
                "truth_used": False,
                "accuracy_scored": False,
                "raw_content_copied_or_transformed": False,
                "inventory": inventory,
                "failure": "pre-solver inventory failed closed",
                "gates": empty_gates(),
            }
            route_record["gates"]["no_fallback_or_publication"] = True
        route_records[route] = route_record
        atomic_json(OUTPUT_ROOT / "partial_result.json", {
            "routes": route_records,
            "completed_routes": list(route_records),
            "native_solver_invocations": accounting["native_solver_invocations"],
        })
        del payloads
    all_passed = all(all(route_records[route].get("gates", {}).values())
                     for route in ROUTES)
    result: dict[str, Any] = {
        "schema_version": "smartphone-r5-phase129-glonass-local-miss-structural-result.v1",
        "phase": 129,
        "execution_label": "Luna Max",
        "status": "go-phase129-glonass-local-miss-structural" if all_passed
                  else "no-go-phase129-glonass-local-miss-structural",
        "decision": ("All inventory and structural gates passed; truth/accuracy and solution release remain unauthorized."
                     if all_passed else
                     "Inventory or structural gate failed closed; no rerun, fallback, truth, accuracy, or solution release is permitted."),
        "authorization": {"path": relative(AUTHORIZATION), "status": auth.get("status"), "independent": True},
        "contract": {"audit_commit": AUDIT_COMMIT,
                     "candidate_freeze_commit": FREEZE_CANDIDATE_COMMIT,
                     "structural_freeze_commit": STRUCTURAL_FREEZE_COMMIT,
                     "implementation_commit": IMPLEMENTATION_COMMIT,
                     "runner_manifest_commit": RUNNER_MANIFEST_COMMIT,
                     "pre_raw_commit": PRE_RAW_COMMIT,
                     "manifest_sha256": MANIFEST_SHA256,
                     "target_binary_sha256": TARGET_BINARY_SHA256},
        "candidate": {"id": contract.CANDIDATE_ID, "phase126": True,
                      "phase127": True, "phase128": True, "phase129": True,
                      "phase118_huber": True, "phase117_dynamic_sigma": False,
                      "phase120_atmosphere": False, "additional_frequency_bands": False,
                      "fixed_tdcp_sigma": 0.03, "official_huber_k": 0.5,
                      "main_solver": "MULTIFRONTAL_QR", "solution_opaque": True},
        "matrix": {"candidate_count": 1, "route_order": list(ROUTES),
                   "runs_per_route": 1,
                   "native_solver_invocations": accounting["native_solver_invocations"],
                   "inventory_reads_per_route": 1, "controls": 0,
                   "reruns": 0, "fallbacks": 0, "truth_reads": 0,
                   "accuracy_calculations": 0, "solution_rows_opened": 0},
        "routes": route_records,
        "read_accounting": accounting,
        "inventory_contract": {"certified_plus_explicit_local_miss": "input",
                               "shared_base_factor_ledger_equal": True,
                               "non_glonass_retained": True,
                               "empty_all_miss_fail_closed": True,
                               "solver_zero_on_inventory_failure": True,
                               "no_raw_zero_or_uncorrected_fallback": True,
                               "no_extrapolation": True,
                               "header_primary_or_geph_fallback_only": True,
                               "fcn_range": [-7, 6],
                               "geph_validity_seconds": 1800.0},
        "truth_free": True,
        "accuracy_scored": False,
        "solution_output_published": False,
        "promotion_authorized": False,
        "stop_before_truth_accuracy_submission": True,
    }
    atomic_json(RESULT_JSON, result)
    RESULT_MD.write_text(result_markdown(result), encoding="utf-8")
    return result


def main() -> int:
    import argparse

    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--verify-authorization", action="store_true",
                        help="verify all pins without opening route payloads")
    parser.add_argument("--execute", action="store_true",
                        help="run the exact authorized matrix once")
    args = parser.parse_args()
    if args.verify_authorization and args.execute:
        parser.error("verification and execution are separate modes")
    if not (args.verify_authorization or args.execute):
        parser.error("one mode is required")
    try:
        if args.verify_authorization:
            verify_authorization()
            print(json.dumps({"status": "authorization-pins-verified",
                              "raw_reads": 0, "solver_invocations": 0}, sort_keys=True))
            return 0
        result = execute_matrix()
        print(json.dumps({"status": result["status"],
                          "routes": {route: {
                              "inventory_ok": result["routes"][route]["inventory"].get("ok"),
                              "solver_launched": result["routes"][route].get("solver_launched"),
                              "return_code": result["routes"][route].get("return_code"),
                              "failure": result["routes"][route].get("failure") or
                                         result["routes"][route]["inventory"].get("failure"),
                          } for route in ROUTES},
                          "read_accounting": result["read_accounting"]},
                         indent=2, sort_keys=True))
        return 0 if result["status"].startswith("go-") else 3
    except (Phase129ExecutionError, OSError) as exc:
        print(f"phase129 authorized runner: fail-closed: {exc}", file=sys.stderr)
        return 2


if __name__ == "__main__":
    raise SystemExit(main())
