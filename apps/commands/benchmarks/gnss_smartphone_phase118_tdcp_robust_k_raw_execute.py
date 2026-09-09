#!/usr/bin/env python3
"""Execute the independently authorized Phase118 raw structural matrix.

The launch-free Phase118 contract remains the authority for the recipe.  This
module owns only the later, separately committed execution boundary: after its
authorization has been verified it stats the three phone members, hashes each
raw base RINEX once, and launches the pinned native binary once for MTV-A and
once for LAX-T, in that order.  The native solution is never parsed; only an
opaque byte/header/row seal is retained.  Truth, MAT, PDC, coordinate,
accuracy, and Kaggle lanes are not available in this process.
"""

from __future__ import annotations

import hashlib
import importlib.util
import json
import math
import os
from pathlib import Path
import subprocess
import sys
import tempfile
import time
from typing import Any


ROOT = Path(__file__).resolve().parents[3]
CONTRACT_PATH = ROOT / "apps/commands/benchmarks/gnss_smartphone_phase118_tdcp_robust_k.py"
MANIFEST = ROOT / "docs/use_cases/records/smartphone_r5_phase118_tdcp_robust_k_structural_manifest_v1.json"
AUTHORIZATION = ROOT / "docs/use_cases/records/smartphone_r5_phase118_tdcp_robust_k_raw_authorization_v1.json"
PHASE112_MANIFEST = ROOT / "docs/use_cases/records/smartphone_r5_phase112_main_output_offset_manifest_v1.json"
OUTPUT_ROOT = ROOT / "output/smartphone-r5/phase118-tdcp-robust-k-v1"
RESULT_JSON = ROOT / "docs/use_cases/records/smartphone_r5_phase118_tdcp_robust_k_structural_result_v1.json"
RESULT_MD = ROOT / "docs/use_cases/records/smartphone_r5_phase118_tdcp_robust_k_structural_result_v1.md"

ROUTES = (
    "2021-03-16-18-59-us-ca-mtv-a/pixel5",
    "2022-04-01-18-22-us-ca-lax-t/pixel5",
)
RAW_NAMES = ("device_gnss.csv", "device_imu.csv", "brdc.nav")
BASE_NAME = "base.obs"
AUTH_SCHEMA = "smartphone-r5-phase118-tdcp-robust-k-raw-authorization.v1"
RESULT_SCHEMA = "smartphone-r5-phase118-tdcp-robust-k-structural-result.v1"
AUTH_STATUS = "authorized-for-exact-two-route-phase118-raw-structural-execution"
OUTPUT_ROOT_RELATIVE = "output/smartphone-r5/phase118-tdcp-robust-k-v1"


class Phase118ExecutionError(ValueError):
    """Raised whenever the authorized execution boundary fails closed."""


def fail(message: str) -> Phase118ExecutionError:
    return Phase118ExecutionError(message)


def relative(path: Path) -> str:
    try:
        return str(path.relative_to(ROOT))
    except ValueError:
        return str(path)


def atomic_json(path: Path, value: dict[str, Any]) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    descriptor, temporary = tempfile.mkstemp(
        prefix=f".{path.name}.", suffix=".tmp", dir=path.parent
    )
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
    descriptor, temporary = tempfile.mkstemp(
        prefix=f".{path.name}.", suffix=".tmp", dir=path.parent
    )
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


def sha256_file(path: Path, label: str, *, payload_kind: str | None = None) -> str:
    """Hash a file, with an explicit post-authorization payload boundary."""
    if payload_kind is None and (
        path.name in RAW_NAMES
        or path.name == BASE_NAME
        or path.name.endswith("withheld_solution_output.csv")
    ):
        raise fail(f"payload/solution hash forbidden at this boundary: {label}")
    if payload_kind not in (None, "base", "solution"):
        raise fail(f"unknown payload hash kind: {payload_kind}")
    if payload_kind == "base" and path.name != BASE_NAME:
        raise fail(f"base hash target is not base.obs: {path}")
    if payload_kind == "solution" and not path.name.endswith("withheld_solution_output.csv"):
        raise fail(f"solution hash target is not withheld output: {path}")
    if not path.is_file():
        raise fail(f"missing {label}: {path}")
    digest = hashlib.sha256()
    try:
        with path.open("rb") as handle:
            for chunk in iter(lambda: handle.read(1024 * 1024), b""):
                digest.update(chunk)
    except OSError as exc:
        raise fail(f"failed to hash {label}: {exc}") from exc
    return digest.hexdigest()


def read_json(path: Path, label: str) -> dict[str, Any]:
    try:
        value = json.loads(path.read_text(encoding="utf-8"))
    except (OSError, UnicodeDecodeError, json.JSONDecodeError) as exc:
        raise fail(f"failed to read {label}: {path}: {exc}") from exc
    if not isinstance(value, dict):
        raise fail(f"{label} is not an object")
    return value


def load_contract() -> Any:
    spec = importlib.util.spec_from_file_location("phase118_structural_contract", CONTRACT_PATH)
    if spec is None or spec.loader is None:
        raise fail(f"unable to load Phase118 contract: {CONTRACT_PATH}")
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


def assert_equal(actual: Any, expected: Any, label: str) -> None:
    if actual != expected:
        raise fail(f"{label}: expected {expected!r}, got {actual!r}")


def pin_path(pin: Any, label: str) -> tuple[str, str]:
    if not isinstance(pin, dict) or not isinstance(pin.get("path"), str) or not isinstance(pin.get("sha256"), str):
        raise fail(f"authorization/{label} pin missing")
    return pin["path"], pin["sha256"]


def verify_authorization() -> tuple[Any, dict[str, Any], dict[str, Any]]:
    """Verify all source/contract pins without touching an input member."""
    contract = load_contract()
    manifest = contract.verify_manifest()
    auth = read_json(AUTHORIZATION, "Phase118 raw authorization")
    for key, expected in {
        "schema_version": AUTH_SCHEMA,
        "phase": 118,
        "execution_label": "Luna Max",
        "status": AUTH_STATUS,
    }.items():
        assert_equal(auth.get(key), expected, f"authorization/{key}")

    authority = auth.get("authority")
    if not isinstance(authority, dict):
        raise fail("authorization/authority missing")

    # These are all source, sealed metadata, or already pinned contract files;
    # none is a raw phone/base/solution member.
    pins: dict[str, tuple[Path, str]] = {
        "source_parity_freeze": (
            contract.SOURCE_PARITY_FREEZE,
            contract.SOURCE_PARITY_FREEZE_SHA256,
        ),
        "structural_contract_freeze": (contract.FREEZE, contract.FREEZE_SHA256),
        "contract_audit": (contract.CONTRACT_AUDIT, contract.CONTRACT_AUDIT_SHA256),
        "structural_manifest": (MANIFEST, "e0a4a0e879a56d7336fe1e2cbf041e5a72920d8103dd22dfdc9dffddfe62ec1d"),
        "launch_free_runner": (contract.WRAPPER, "7c67289274eafd4b8a38bfe18555a82448392b1255ce9febb530b3e1b2d6df89"),
        "structural_validator": (CONTRACT_PATH, "6a7ff88fbee42baecd7bbccabdf76f7d31246ca075ba70cbb2ea3c35707d2201"),
        "focused_tests": (contract.FOCUSED_TESTS, "6d6a08a8efa1d33b8e7d49cb68794f44743a1fb4a164abd904011ab2372eb936"),
        "phase112_manifest": (PHASE112_MANIFEST, contract.PHASE112_MANIFEST_SHA256),
        "phase112_structural_result": (
            contract.PHASE112_RESULT,
            contract.PHASE112_RESULT_SHA256,
        ),
        "phase117_structural_result": (
            contract.PHASE117_RESULT,
            contract.PHASE117_RESULT_SHA256,
        ),
        "target_binary": (contract.BINARY, "fe6c0b501921a8628fb813abe380b84c89824409e731bf7cfd01920d900dea34"),
    }
    for name, (path, expected_hash) in pins.items():
        path_text, declared_hash = pin_path(authority.get(name), name)
        assert_equal(path_text, relative(path), f"authorization/{name}/path")
        assert_equal(declared_hash, expected_hash, f"authorization/{name}/sha256")
        assert_equal(sha256_file(path, name), expected_hash, f"authorization/{name}/file_sha256")

    assert_equal(authority.get("source_parity_freeze_commit"), "5fcc06ab64189dd5dfb8001664bdb8496f85224f", "authorization/source_parity_freeze_commit")
    assert_equal(authority.get("structural_contract_freeze_commit"), "25f3bada02c4073175054b5d668de02020761208", "authorization/structural_contract_freeze_commit")
    assert_equal(authority.get("contract_commit"), "8ffc4148e15431aeaa57ee65f730b8449829113d", "authorization/contract_commit")
    assert_equal(authority.get("implementation_commit"), "7f339ccc8f0fb58e3dbcb7f3fc24ba2b04acbf40", "authorization/implementation_commit")
    assert_equal(authority.get("pre_raw_audit_commit"), "9fc06d23ec1f5e1f730d1d132bdf6107c4b65f8f", "authorization/pre_raw_audit_commit")
    assert_equal(authority.get("pre_raw_audit_sha256"), "71a619578c6a4a8aa0e32562107f78857c4557a053e4822d5396421ddf2c8c29", "authorization/pre_raw_audit_sha256")
    pre_raw_path = ROOT / "docs/use_cases/records/smartphone_r5_phase118_tdcp_robust_k_pre_raw_audit_v1.json"
    assert_equal(sha256_file(pre_raw_path, "pre-raw audit"), authority.get("pre_raw_audit_sha256"), "authorization/pre_raw_audit/file_sha256")
    assert_equal(authority.get("target_binary_sha256"), "fe6c0b501921a8628fb813abe380b84c89824409e731bf7cfd01920d900dea34", "authorization/target_binary_sha256")

    executor = auth.get("raw_executor")
    if not isinstance(executor, dict):
        raise fail("authorization/raw_executor missing")
    assert_equal(executor.get("path"), relative(Path(__file__)), "authorization/raw_executor/path")
    assert_equal(sha256_file(Path(__file__), "raw executor"), executor.get("sha256"), "authorization/raw_executor/sha256")

    assert_equal(auth.get("routes"), list(ROUTES), "authorization/routes")
    candidate = auth.get("candidate")
    if not isinstance(candidate, dict):
        raise fail("authorization/candidate missing")
    for key, expected in {
        "id": "phase118-official-tdcp-huber-k-mapping-v1",
        "candidate_count": 1,
        "selector": "--native-phase118-official-tdcp-huber-k",
        "official_setting_type": "Highway",
        "official_huber_k": 0.5,
        "fixed_tdcp_sigma_m": 0.03,
        "phase117_dynamic_sigma": False,
        "default_off": True,
        "opt_in": True,
        "factor_count_and_reject_invariance": True,
        "c7_d_handoff": "exact/full/finite retained epoch order",
        "solver_branch": "MULTIFRONTAL_QR / EliminateQR",
        "base_correction": "exactly once",
        "pixel5_offset": "exactly once at final output boundary",
        "no_fallback": True,
        "solution_withheld": True,
        "truth_evaluation": False,
        "accuracy_scoring": False,
    }.items():
        assert_equal(candidate.get(key), expected, f"authorization/candidate/{key}")
    selectors = candidate.get("selectors")
    if not isinstance(selectors, list):
        raise fail("authorization/candidate/selectors missing")
    for selector in (
        "--native-source-clock-c0d-gnss-first-meter-state-handoff",
        "--native-source-clock-c0d-epoch-vector-parity",
        "--native-source-clock-c0d-phase99-main-multifrontal-qr-solver",
        "--native-upstream-position-offset",
        "--native-base-pseudorange-compensation",
        "--native-base-pseudorange-source-miss-mask",
        "--native-base-pseudorange-preserve-additional-frequency-bands",
        "--native-phase118-official-tdcp-huber-k",
    ):
        assert_equal(selectors.count(selector), 1, f"authorization/candidate/selectors/{selector}")
    assert_equal(selectors.count("--native-phase117-tdcp-snr-type-sigma"), 0, "authorization/candidate/Phase117 selector")

    matrix = auth.get("matrix")
    if not isinstance(matrix, dict):
        raise fail("authorization/matrix missing")
    for key, expected in {
        "candidate_count": 1,
        "route_count": 2,
        "runs_per_route": 1,
        "native_invocations": 2,
        "raw_phone_gnss_process_reads_max": 2,
        "raw_phone_imu_process_reads_max": 2,
        "broadcast_navigation_process_reads_max": 2,
        "raw_base_hash_reads_max": 2,
        "raw_base_native_process_reads_max": 2,
        "truth_reads": 0,
        "mat_reads_or_generated": 0,
        "phone_coordinate_reads": 0,
        "precomputed_coordinate_reads": 0,
        "pdc_reads": 0,
        "accuracy_calculations": 0,
        "kaggle_or_token_access": 0,
        "reruns": 0,
        "fallbacks": 0,
        "solution_rows_authorized": False,
    }.items():
        assert_equal(matrix.get(key), expected, f"authorization/matrix/{key}")

    policy = auth.get("execution_policy")
    if not isinstance(policy, dict):
        raise fail("authorization/execution_policy missing")
    for key, expected in {
        "authorized": True,
        "order": "MTV-A then LAX-T, sequentially",
        "one_invocation_per_route": True,
        "no_rerun": True,
        "no_solver_fallback": True,
        "raw_inputs_only": "device_gnss.csv/device_imu.csv/brdc.nav plus sealed raw base RINEX",
        "solver_filter_lm_graph_equations_units_sigma_unchanged": True,
        "solution_output": "opaque hash/header/row seal only; no coordinate interpretation or publication",
        "truth_or_accuracy_evaluation": False,
    }.items():
        assert_equal(policy.get(key), expected, f"authorization/execution_policy/{key}")
    boundary = auth.get("release_boundary")
    if not isinstance(boundary, dict):
        raise fail("authorization/release_boundary missing")
    for key, expected in {
        "raw_execution_authorized": True,
        "solver_execution_authorized": True,
        "structural_result_authorized": True,
        "truth_evaluation_authorized": False,
        "accuracy_authorized": False,
        "solution_publication_authorized": False,
        "kaggle_submission_authorized": False,
    }.items():
        assert_equal(boundary.get(key), expected, f"authorization/release_boundary/{key}")

    read_accounting = auth.get("pre_authorization_read_accounting")
    if not isinstance(read_accounting, dict):
        raise fail("authorization/pre_authorization_read_accounting missing")
    for key in (
        "raw_phone_gnss_reads", "raw_phone_imu_reads", "broadcast_navigation_reads",
        "raw_base_rinex_reads", "raw_base_hash_reads", "native_solver_invocations",
        "solution_rows_opened", "truth_reads", "mat_reads_or_generated",
        "phone_coordinate_reads", "precomputed_coordinate_reads", "pdc_reads",
        "accuracy_calculations", "kaggle_or_token_access", "route_reruns", "fallbacks",
    ):
        assert_equal(read_accounting.get(key), 0, f"authorization/pre_authorization_read_accounting/{key}")
    assert_equal(read_accounting.get("raw_content_copied_or_transformed"), False, "authorization/pre_authorization_read_accounting/copy")
    return contract, manifest, auth


def safe_member(path_text: Any, basename: str, route: str) -> Path:
    """Resolve an exact sealed member only after authorization."""
    if not isinstance(path_text, str):
        raise fail(f"missing sealed {basename} path: {route}")
    candidate = Path(path_text)
    if candidate.is_absolute() or ".." in candidate.parts or candidate.name != basename:
        raise fail(f"unsafe sealed {basename} path: {route}: {path_text}")
    path = ROOT / candidate
    if not path.is_file():
        raise fail(f"missing sealed {basename} member: {route}: {path}")
    return path


def materialize_inputs(contract: Any) -> dict[str, dict[str, Any]]:
    """Stat phone members and hash base members after independent auth."""
    source_routes = contract.sealed_phase112_routes()
    selected: dict[str, dict[str, Any]] = {}
    for route in ROUTES:
        source = source_routes[route]
        selected[route] = {"raw": {}, "base": {}}
        for name in RAW_NAMES:
            pin = source["raw_inputs"][name]
            path = safe_member(pin["path"], name, route)
            size = path.stat().st_size
            if size != pin["bytes"]:
                raise fail(f"sealed raw byte count changed: {route}/{name}: {size} != {pin['bytes']}")
            selected[route]["raw"][name] = {
                "path": pin["path"],
                "sealed_sha256": pin["sha256"],
                "sealed_bytes": pin["bytes"],
                "bytes": size,
                "exists_before_launch": True,
                "stat_read_by_wrapper": True,
                "payload_read_by_wrapper": False,
                "hash_read_by_wrapper": False,
                "content_copied_or_transformed": False,
            }
        base_pin = source["base_input"]
        base_path = safe_member(base_pin["path"], BASE_NAME, route)
        base_size = base_path.stat().st_size
        if base_size != base_pin["bytes"]:
            raise fail(f"sealed base byte count changed: {route}: {base_size} != {base_pin['bytes']}")
        actual = sha256_file(base_path, f"raw base {route}", payload_kind="base")
        if actual != base_pin["sha256"]:
            raise fail(f"sealed base SHA changed: {route}")
        # Do not copy or retain the sealed approx_position_xyz_m metadata: no
        # coordinate is an input to this wrapper.  Native reads the RINEX
        # header itself under the raw-base contract.
        selected[route]["base"] = {
            "path": base_pin["path"],
            "sha256": actual,
            "sealed_sha256": base_pin["sha256"],
            "sealed_bytes": base_pin["bytes"],
            "bytes": base_size,
            "observed_dt_s": base_pin.get("observed_dt_s"),
            "moving_mean_samples": base_pin.get("moving_mean_samples"),
            "coordinate_source": "raw RINEX header consumed by native process",
            "exists_before_launch": True,
            "stat_read_by_wrapper": True,
            "payload_read_by_wrapper": False,
            "hash_read_by_wrapper": True,
            "hash_verification_reads": 1,
            "native_process_reads_expected": 1,
            "content_copied_or_transformed": False,
        }
    return selected


def materialize_command(contract: Any, record: dict[str, Any], inputs: dict[str, Any]) -> list[str]:
    route = record["dataset_id"]
    contract.validate_command(route, record.get("command"))
    replacements = {
        "__PHASE118_RAW_DEVICE_GNSS__": inputs["raw"]["device_gnss.csv"]["path"],
        "__PHASE118_RAW_DEVICE_IMU__": inputs["raw"]["device_imu.csv"]["path"],
        "__PHASE118_RAW_BROADCAST_NAV__": inputs["raw"]["brdc.nav"]["path"],
        "__PHASE118_RAW_BASE_RINEX__": inputs["base"]["path"],
        "__PHASE118_RAW_BASE_SHA256__": inputs["base"]["sha256"],
    }
    command = [replacements.get(token, token) for token in record["command"]]
    for token in command:
        if token in contract.FORBIDDEN_FLAGS:
            raise fail(f"forbidden flag after materialization: {route}/{token}")
    return command


def safe_environment() -> dict[str, str]:
    # No inherited truth/MAT/coordinate/token environment is passed to native.
    return {
        "PATH": os.environ.get("PATH", "/usr/bin:/bin"),
        "LANG": "C",
        "LC_ALL": "C",
        "TZ": "UTC",
        "LD_LIBRARY_PATH": "/home/sasaki/.local/lib",
    }


def finite(value: Any) -> bool:
    return isinstance(value, (int, float)) and not isinstance(value, bool) and math.isfinite(float(value))


def read_summary(path: Path) -> tuple[dict[str, Any] | None, dict[str, Any]]:
    if not path.is_file():
        return None, {"path": relative(path), "present": False, "read_count": 0}
    payload = path.read_bytes()
    metadata: dict[str, Any] = {
        "path": relative(path),
        "present": True,
        "bytes": len(payload),
        "sha256": hashlib.sha256(payload).hexdigest(),
        "read_count": 1,
    }
    try:
        summary = json.loads(payload.decode("utf-8"))
    except (UnicodeDecodeError, json.JSONDecodeError) as exc:
        metadata["error"] = str(exc)
        return None, metadata
    if not isinstance(summary, dict):
        metadata["error"] = "summary is not an object"
        return None, metadata
    metadata["schema_version"] = summary.get("schema_version")
    metadata["status"] = summary.get("status")
    return summary, metadata


def seal_solution(path: Path, expected_rows: int) -> dict[str, Any]:
    """Hash/header-count the solution without parsing coordinate columns."""
    if not path.is_file():
        return {
            "path": relative(path),
            "present": False,
            "opened": False,
            "published": False,
            "read_count": 0,
            "coordinate_rows_omitted": True,
        }
    digest = hashlib.sha256()
    first_line = b""
    line_count = 0
    total_bytes = 0
    try:
        with path.open("rb") as handle:
            first_line = handle.readline()
            if first_line:
                digest.update(first_line)
                total_bytes += len(first_line)
                line_count = 1
            for chunk in iter(lambda: handle.read(1024 * 1024), b""):
                digest.update(chunk)
                total_bytes += len(chunk)
                line_count += chunk.count(b"\n")
    except OSError as exc:
        raise fail(f"failed to seal opaque solution: {path}: {exc}") from exc
    rows = max(0, line_count - 1)
    return {
        "path": relative(path),
        "present": True,
        "opened": True,
        "published": False,
        "sha256": digest.hexdigest(),
        "bytes": total_bytes,
        "header": first_line.decode("utf-8", errors="replace").rstrip("\r\n"),
        "rows": rows,
        "expected_domain_rows": expected_rows,
        "row_count_matches_domain": rows == expected_rows,
        "coordinate_rows_omitted": True,
        "read_count": 1,
    }


def compact_map(value: Any, *, omit_keys: tuple[str, ...] = ()) -> Any:
    """Copy only structural scalar telemetry and redact coordinate-like data."""
    if isinstance(value, dict):
        result: dict[str, Any] = {}
        for key, item in value.items():
            key_lower = str(key).lower()
            if any(term in key_lower for term in omit_keys):
                continue
            compacted = compact_map(item, omit_keys=omit_keys)
            if compacted is not _OMIT:
                result[key] = compacted
        return result
    if isinstance(value, list):
        # Arrays in native summaries are either coordinates or large per-row
        # output; retain only short symbolic arrays (e.g. component names).
        if len(value) > 16:
            return _OMIT
        compacted = [compact_map(item, omit_keys=omit_keys) for item in value]
        return [item for item in compacted if item is not _OMIT]
    if isinstance(value, float) and not math.isfinite(value):
        return None
    if isinstance(value, (str, int, float, bool)) or value is None:
        return value
    return _OMIT


class _Omit:
    pass


_OMIT = _Omit()
OMIT_KEYS = (
    "position_ecef", "coordinate_xyz", "coordinate", "latitude", "longitude",
    "solution", "trajectory", "row", "ecef", "geodetic",
)


def compact_telemetry(summary: dict[str, Any]) -> dict[str, Any]:
    keep = (
        "schema_version", "status", "truth_used", "base_factors",
        "no_base_contract", "production_default_changed",
        "native_phase117_tdcp_snr_type_sigma", "native_phase118_official_tdcp_huber_k",
        "native_source_clock_c0d_factor_enabled",
        "native_source_clock_c0d_meter_state_parity_enabled",
        "native_source_clock_c0d_epoch_vector_parity_enabled",
        "native_source_clock_c0d_gnss_first_meter_state_handoff_enabled",
        "native_source_clock_c0d_phase99_main_multifrontal_qr_solver_enabled",
        "native_source_clock_c0d_phase99_main_multifrontal_qr_solver_selected",
        "selected_linear_solver_type", "selected_solver_branch",
        "selected_elimination_function", "epochs", "tdcp_contract",
        "gnss_first", "graph", "native_source_clock_c0d_factor",
        "native_base_pseudorange_compensation",
        "native_base_pseudorange_source_miss_mask", "upstream_position_offset",
        "raw_utc_key_contract", "output_contract",
    )
    selected = {key: summary.get(key) for key in keep if key in summary}
    return compact_map(selected, omit_keys=OMIT_KEYS)


def expected_tdcp_counts(manifest: dict[str, Any], route: str) -> dict[str, int]:
    baseline = manifest.get("baseline_tdcp_counts")
    if not isinstance(baseline, dict) or not isinstance(baseline.get(route), dict):
        raise fail(f"baseline TDCP count missing: {route}")
    required = (
        "candidate_pairs", "factors_built", "factors_inserted", "finite_residuals",
        "rejected_gap", "rejected_code_phase_jump", "rejected_clock_discontinuity",
        "rejected_invalid_measurement", "rejected_invalid_weight", "rejected_loss_of_lock",
        "rejected_missing_previous",
    )
    result: dict[str, int] = {}
    for key in required:
        value = baseline[route].get(key)
        if not isinstance(value, int) or value < 0:
            raise fail(f"invalid baseline TDCP count: {route}/{key}")
        result[key] = value
    return result


def validate_route(metadata: dict[str, Any], contract: Any, manifest: dict[str, Any]) -> dict[str, Any]:
    route = metadata["dataset_id"]
    record = next(item for item in manifest["routes"] if item["dataset_id"] == route)
    expected = int(record["expected_problem_epochs"])
    domain = int(record["domain_rows"])
    summary_path = ROOT / metadata["planned_output"]["summary"]
    solution_path = ROOT / metadata["planned_output"]["withheld_solution_output"]
    summary, summary_meta = read_summary(summary_path)
    solution_meta = seal_solution(solution_path, domain)
    report: dict[str, Any] = {
        "dataset_id": route,
        "run_number": 1,
        "return_code": metadata.get("return_code"),
        "expected": {"domain_rows": domain, "problem_epochs": expected, "output_epochs": expected},
        "summary": summary_meta,
        "solution_hash_seal": solution_meta,
        "raw_inputs": metadata.get("raw_inputs"),
        "base_input": metadata.get("base_input"),
        "truth_used": False,
        "mat_used": False,
        "phone_coordinates_used": False,
        "precomputed_coordinates_used": False,
        "pdc_used": False,
        "accuracy_scored": False,
        "kaggle_or_token_accessed": False,
        "solution_output_published": False,
        "read_accounting": {
            "runner_raw_payload_reads": 0,
            "runner_raw_hash_reads": 0,
            "runner_base_hash_reads": 1,
            "native_raw_phone_gnss_reads": 1,
            "native_raw_phone_imu_reads": 1,
            "native_broadcast_navigation_reads": 1,
            "native_base_rinex_reads": 1,
            "truth_reads": 0,
            "mat_reads_or_generated": 0,
            "phone_coordinate_reads": 0,
            "precomputed_coordinate_reads": 0,
            "pdc_reads": 0,
            "accuracy_calculations": 0,
            "solution_output_hash_reads": 1 if solution_meta.get("present") else 0,
            "raw_content_copied_or_transformed": False,
        },
    }
    gate_names = (
        "native_process_completed", "summary_present", "option_isolation_and_fixed_sigma",
        "valid_tdcp_factor_and_reject_counts_unchanged", "gnss_first_progress_strict_cost_decrease",
        "gnss_first_full_finite_c7_d_exact_handoff", "main_qr_progress_strict_cost_decrease",
        "base_correction_exactly_once", "pixel5_offset_exactly_once_final_boundary",
        "finite_earth_valid_expected_output_coverage", "no_solver_fallback_or_solution_publication",
    )
    if summary is None:
        report["telemetry"] = None
        report["gates"] = {name: False for name in gate_names}
        report["failure_reasons"] = list(gate_names)
        return report

    tdcp = summary.get("tdcp_contract")
    gnss = summary.get("gnss_first")
    graph = summary.get("graph")
    clock = summary.get("native_source_clock_c0d_factor")
    base = summary.get("native_base_pseudorange_compensation")
    miss = summary.get("native_base_pseudorange_source_miss_mask")
    offset = summary.get("upstream_position_offset")
    epochs = summary.get("epochs")
    raw_utc = summary.get("raw_utc_key_contract")
    expected_counts = expected_tdcp_counts(manifest, route)

    option_ok = (
        summary.get("native_phase118_official_tdcp_huber_k") is True
        and summary.get("native_phase117_tdcp_snr_type_sigma") is False
        and isinstance(tdcp, dict)
        and tdcp.get("official_huber_k_enabled") is True
        and tdcp.get("official_huber_k") == 0.5
        and tdcp.get("official_setting_type") == "Highway"
        and tdcp.get("fixed_sigma_m") == 0.03
        and tdcp.get("official_snr_type_sigma_enabled") is False
        and tdcp.get("official_invalid_weight_fail_closed") is True
    )
    factor_ok = (
        isinstance(tdcp, dict)
        and all(tdcp.get(key) == value for key, value in expected_counts.items())
        and tdcp.get("nonfinite_residuals") == 0
        and tdcp.get("pair_key") == "(satellite,signal)"
        and tdcp.get("adr_state_slip_fail_closed") is True
        and tdcp.get("standalone_carrier_ambiguity_factors") is False
        and tdcp.get("base_or_double_difference_factors") is False
        and tdcp.get("sigma_m") == 0.03
    )
    gnss_progress = (
        isinstance(gnss, dict)
        and gnss.get("attempted") is True
        and gnss.get("converged") is True
        and isinstance(gnss.get("iterations"), int)
        and gnss.get("iterations") >= 1
        and finite(gnss.get("initial_cost"))
        and finite(gnss.get("final_cost"))
        and gnss.get("final_cost") < gnss.get("initial_cost")
        and isinstance(gnss.get("c0d_accepted_outer_iterations"), int)
        and gnss.get("c0d_accepted_outer_iterations") > 0
    )
    handoff = (
        isinstance(gnss, dict)
        and gnss.get("optimized_c_vector_parity_enabled") is True
        and gnss.get("optimized_c_export_valid") is True
        and gnss.get("optimized_c_dimension") == 7
        and gnss.get("optimized_c_epoch_count") == expected
        and gnss.get("optimized_c_finite_component_count") == expected * 7
        and gnss.get("optimized_c_nonfinite_component_count") == 0
        and gnss.get("optimized_d_export_valid") is True
        and gnss.get("optimized_d_epoch_count") == expected
        and gnss.get("optimized_d_finite_count") == expected
        and gnss.get("optimized_d_nonfinite_count") == 0
        and gnss.get("epoch_identity_alignment_valid") is True
    )
    qr_ok = (
        summary.get("native_source_clock_c0d_phase99_main_multifrontal_qr_solver_enabled") is True
        and summary.get("native_source_clock_c0d_phase99_main_multifrontal_qr_solver_selected") is True
        and summary.get("selected_linear_solver_type") == "MULTIFRONTAL_QR"
        and summary.get("selected_elimination_function") == "EliminateQR"
    )
    main_progress = (
        isinstance(graph, dict)
        and graph.get("converged") is True
        and isinstance(graph.get("iterations"), int)
        and graph.get("iterations") >= 1
        and finite(graph.get("initial_cost"))
        and finite(graph.get("final_cost"))
        and graph.get("final_cost") < graph.get("initial_cost")
        and isinstance(clock, dict)
        and clock.get("active_solve_attempted") is True
        and isinstance(clock.get("accepted_outer_iterations"), int)
        and clock.get("accepted_outer_iterations") > 0
        and finite(clock.get("active_solve_initial_cost"))
        and finite(clock.get("active_solve_final_cost"))
        and clock.get("active_solve_final_cost") < clock.get("active_solve_initial_cost")
    )
    base_ok = (
        isinstance(base, dict)
        and base.get("enabled") is True
        and base.get("applied") is True
        and base.get("correction_application_pass_count") == 1
        and base.get("correction_applied_exactly_once") is True
        and base.get("duplicate_correction_rejected") is False
        and base.get("base_rinex_sha256") == metadata.get("base_input", {}).get("sha256")
        and isinstance(miss, dict)
        and miss.get("correction_application_pass_count") == 1
        and miss.get("correction_applied_exactly_once") is True
        and miss.get("duplicate_correction_rejected") is False
    )
    offset_ok = (
        isinstance(offset, dict)
        and offset.get("enabled") is True
        and offset.get("applied") is True
        and offset.get("phone") == "pixel5"
        and offset.get("corrected_epochs") == expected
        and finite(offset.get("max_offset_enu_m"))
        and solution_meta.get("present") is True
        and solution_meta.get("row_count_matches_domain") is True
    )
    output_ok = (
        isinstance(epochs, dict)
        and epochs.get("problem") == expected
        and epochs.get("output") == expected
        and isinstance(raw_utc, dict)
        and raw_utc.get("raw_epoch_keys") == expected
        and raw_utc.get("unresolved_epochs") == 0
        and isinstance(summary.get("output_contract"), dict)
        and summary["output_contract"].get("finite_coordinates") is True
    )
    no_publication = (
        metadata.get("runner_read_raw_payloads") is False
        and metadata.get("runner_read_raw_hashes") is False
        and metadata.get("runner_read_forbidden_lanes") is False
        and metadata.get("solution_output_published") is False
        and summary.get("truth_used") is False
        and summary.get("production_default_changed") is False
        and summary.get("status") != "fallback-native-fgo-v1"
    )
    process_ok = (
        metadata.get("return_code") == 0
        and not metadata.get("launch_error")
        and not metadata.get("interrupted")
        and not metadata.get("timed_out")
    )
    report["telemetry"] = compact_telemetry(summary)
    report["gates"] = {
        "native_process_completed": process_ok,
        "summary_present": summary_meta.get("present") is True,
        "option_isolation_and_fixed_sigma": option_ok,
        "valid_tdcp_factor_and_reject_counts_unchanged": factor_ok,
        "gnss_first_progress_strict_cost_decrease": gnss_progress,
        "gnss_first_full_finite_c7_d_exact_handoff": handoff,
        "main_qr_progress_strict_cost_decrease": qr_ok and main_progress,
        "base_correction_exactly_once": base_ok,
        "pixel5_offset_exactly_once_final_boundary": offset_ok,
        "finite_earth_valid_expected_output_coverage": output_ok,
        "no_solver_fallback_or_solution_publication": no_publication,
    }
    report["failure_reasons"] = [name for name, passed in report["gates"].items() if passed is not True]
    return report


def execute_matrix(contract: Any, manifest: dict[str, Any]) -> list[dict[str, Any]]:
    if OUTPUT_ROOT.exists():
        raise fail(f"refusing to overwrite Phase118 output root; rerun forbidden: {OUTPUT_ROOT}")
    records = manifest.get("routes")
    if not isinstance(records, list) or [item.get("dataset_id") for item in records] != list(ROUTES):
        raise fail("Phase118 route order/count changed")
    inputs = materialize_inputs(contract)
    OUTPUT_ROOT.mkdir(parents=True, exist_ok=False)
    metadata: list[dict[str, Any]] = []
    for record in records:
        route = record["dataset_id"]
        route_dir = OUTPUT_ROOT / route.replace("/", "__")
        route_dir.mkdir(parents=True, exist_ok=False)
        command = materialize_command(contract, record, inputs[route])
        summary_rel = record["command"][record["command"].index("--summary-json") + 1]
        solution_rel = record["command"][record["command"].index("--out") + 1]
        summary_path = ROOT / summary_rel
        solution_path = ROOT / solution_rel
        if summary_path.exists() or solution_path.exists():
            raise fail(f"refusing pre-existing Phase118 output member: {route}")
        stdout_path = route_dir / "stdout.log"
        stderr_path = route_dir / "stderr.log"
        started = time.time()
        return_code: int | None = None
        interrupted = False
        timed_out = False
        launch_error = ""
        with stdout_path.open("wb") as stdout, stderr_path.open("wb") as stderr:
            try:
                completed = subprocess.run(
                    command,
                    cwd=ROOT,
                    env=safe_environment(),
                    stdout=stdout,
                    stderr=stderr,
                    check=False,
                    timeout=1800,
                )
                return_code = completed.returncode
            except subprocess.TimeoutExpired as exc:
                timed_out = True
                stderr.write(f"\nPhase118 native launch timed out: {exc}\n".encode())
            except KeyboardInterrupt:
                interrupted = True
                stderr.write(b"\nPhase118 wrapper interrupted; partial output preserved.\n")
            except OSError as exc:
                launch_error = str(exc)
                stderr.write(f"\nPhase118 native launch failed: {exc}\n".encode())
        item = {
            "schema_version": "smartphone-r5-phase118-tdcp-robust-k-route-execution.v1",
            "dataset_id": route,
            "run_number": 1,
            "command": command,
            "raw_inputs": inputs[route]["raw"],
            "base_input": inputs[route]["base"],
            "planned_output": {"summary": relative(summary_path), "withheld_solution_output": relative(solution_path)},
            "summary_present_after_launch": summary_path.is_file(),
            "withheld_solution_output_present_after_launch": solution_path.is_file(),
            "stdout": relative(stdout_path),
            "stderr": relative(stderr_path),
            "started_unix_s": started,
            "ended_unix_s": time.time(),
            "return_code": return_code,
            "interrupted": interrupted,
            "timed_out": timed_out,
            "launch_error": launch_error,
            "runner_read_raw_payloads": False,
            "runner_read_raw_hashes": False,
            "runner_read_forbidden_lanes": False,
            "runner_read_base_payload_for_hash": True,
            "raw_content_copied_or_transformed": False,
            "solution_output_opened": False,
            "solution_output_published": False,
            "accuracy_scored": False,
        }
        atomic_json(route_dir / "run_metadata.json", item)
        metadata.append(item)
        atomic_json(OUTPUT_ROOT / "partial_execution_metadata.json", {"routes_completed": metadata})
    return metadata


def build_result(metadata: list[dict[str, Any]], contract: Any, manifest: dict[str, Any], auth: dict[str, Any]) -> dict[str, Any]:
    reports = [validate_route(item, contract, manifest) for item in metadata]
    ordered = len(reports) == 2 and [item["dataset_id"] for item in reports] == list(ROUTES)
    all_passed = ordered and all(all(value is True for value in report.get("gates", {}).values()) for report in reports)
    by_route = {report["dataset_id"]: report for report in reports}
    failed = {
        route: report.get("failure_reasons", [])
        for route, report in by_route.items()
        if report.get("failure_reasons")
    }
    return {
        "schema_version": RESULT_SCHEMA,
        "phase": 118,
        "execution_label": "Luna Max",
        "status": "go-phase118-official-tdcp-huber-k-structural" if all_passed else "no-go-phase118-official-tdcp-huber-k-structural",
        "decision": "Structural gates passed; truth/accuracy and solution release remain separately unauthorized." if all_passed else "Structural gate failed closed; preserve partial artifacts and do not retry, fallback, truth-score, or publish.",
        "candidate": {
            "id": "phase118-official-tdcp-huber-k-mapping-v1",
            "candidate_count": 1,
            "selector": "--native-phase118-official-tdcp-huber-k",
            "official_setting_type": "Highway",
            "official_huber_k": 0.5,
            "fixed_tdcp_sigma_m": 0.03,
            "phase117_dynamic_sigma": False,
            "default_off": True,
            "solution_output_published": False,
        },
        "authority": {
            "path": relative(AUTHORIZATION),
            "sha256": sha256_file(AUTHORIZATION, "Phase118 authorization"),
            "status": auth.get("status"),
        },
        "contract": {
            "path": relative(MANIFEST),
            "sha256": sha256_file(MANIFEST, "Phase118 structural manifest"),
            "freeze_commit": "25f3bada02c4073175054b5d668de02020761208",
            "implementation_commit": "7f339ccc8f0fb58e3dbcb7f3fc24ba2b04acbf40",
            "runner_validator_commit": "8ffc4148e15431aeaa57ee65f730b8449829113d",
            "pre_raw_audit_commit": "9fc06d23ec1f5e1f730d1d132bdf6107c4b65f8f",
        },
        "routes": by_route,
        "matrix": {
            "candidate_count": 1,
            "route_count": 2,
            "runs_per_route": 1,
            "native_solver_invocations": len(metadata),
            "controls": 0,
            "reruns": 0,
            "fallbacks": 0,
            "raw_phone_gnss_process_reads": len(metadata),
            "raw_phone_imu_process_reads": len(metadata),
            "broadcast_navigation_process_reads": len(metadata),
            "raw_base_wrapper_hash_reads": len(metadata),
            "raw_base_native_process_reads_declared": len(metadata),
            "truth_reads": 0,
            "mat_reads_or_generated": 0,
            "phone_coordinate_reads": 0,
            "precomputed_coordinate_reads": 0,
            "pdc_reads": 0,
            "accuracy_calculations": 0,
            "solution_rows_published": False,
        },
        "gates": {"all_structural_gates_passed": all_passed, "route_order_exact": ordered, "failed_routes": failed},
        "failed_gates": failed,
        "read_accounting": {
            "native_solver_invocations": len(metadata),
            "raw_phone_gnss_process_reads": len(metadata),
            "raw_phone_imu_process_reads": len(metadata),
            "broadcast_navigation_process_reads": len(metadata),
            "raw_base_wrapper_hash_reads": len(metadata),
            "raw_base_native_process_reads_declared": len(metadata),
            "runner_raw_payload_reads": 0,
            "runner_raw_input_hash_reads": 0,
            "truth_reads": 0,
            "mat_reads_or_generated": 0,
            "phone_coordinate_reads": 0,
            "precomputed_coordinate_reads": 0,
            "pdc_reads": 0,
            "accuracy_calculations": 0,
            "kaggle_or_token_access": 0,
            "route_reruns": 0,
            "fallbacks": 0,
            "solution_output_hash_seal_reads": sum(1 for item in reports if item.get("solution_hash_seal", {}).get("present")),
            "solution_output_published": False,
            "raw_content_copied_or_transformed": False,
            "logs_and_partial_results_preserved": True,
        },
        "forbidden_lanes": {
            "truth": False,
            "MAT": False,
            "phone_coordinates": False,
            "precomputed_coordinates": False,
            "PDC": False,
            "Kaggle_or_token": False,
            "accuracy": False,
            "solution_rows": False,
        },
        "truth_free": True,
        "accuracy_scored": False,
        "solution_output_published": False,
        "promotion_authorized": False,
        "stop_before_truth_accuracy_submission": True,
    }


def render_markdown(result: dict[str, Any]) -> str:
    lines = [
        "# Phase118 official TDCP Huber-k structural result",
        "",
        f"- status: `{result['status']}`",
        "- matrix: exactly MTV-A then LAX-T, one native invocation per route",
        "- recipe: Phase112 raw/base + C7/D handoff + Phase99 MULTIFRONTAL_QR + raw-base correction + Pixel5 final offset + Phase118 official Highway k",
        "- fixed TDCP sigma: `0.03 m`; Phase117 dynamic sigma: disabled",
        "- truth/MAT/phone-coordinate/precomputed/PDC/Kaggle/accuracy lanes: not read",
        "- solution: opaque hash/header/row seal only; coordinates omitted and unpublished",
        "",
        "| Route | Return | Passed gates |",
        "|---|---:|---:|",
    ]
    for route, report in result["routes"].items():
        gates = report.get("gates", {})
        lines.append(f"| `{route}` | `{report.get('return_code')}` | `{sum(value is True for value in gates.values())}/{len(gates)}` |")
    lines.extend([
        "",
        "A failed gate is sealed fail-closed.  No retry, fallback, truth score, accuracy evaluation, or solution release is permitted.",
        "",
    ])
    return "\n".join(lines)


def main() -> int:
    parser = __import__("argparse").ArgumentParser(description=__doc__)
    parser.add_argument("--verify-authorization", action="store_true")
    parser.add_argument("--execute-authorized", action="store_true")
    parser.add_argument("--run", action="store_true", help="alias for --execute-authorized")
    args = parser.parse_args()
    if args.verify_authorization and (args.execute_authorized or args.run):
        parser.error("verification and execution cannot be combined")
    if not args.verify_authorization and not args.execute_authorized and not args.run:
        parser.error("one of --verify-authorization or --execute-authorized is required")
    contract, manifest, auth = verify_authorization()
    if args.verify_authorization:
        print(json.dumps({
            "status": "phase118-raw-authorization-verified",
            "authorization_status": auth["status"],
            "route_order": list(ROUTES),
            "runs_per_route": 1,
            "raw_reads_before_authorization": 0,
            "truth_reads": 0,
            "mat_reads_or_generated": 0,
            "solution_rows_opened": 0,
        }, indent=2, sort_keys=True))
        return 0
    metadata = execute_matrix(contract, manifest)
    result = build_result(metadata, contract, manifest, auth)
    atomic_json(RESULT_JSON, result)
    atomic_text(RESULT_MD, render_markdown(result))
    return 0 if result["gates"]["all_structural_gates_passed"] else 1


if __name__ == "__main__":
    try:
        raise SystemExit(main())
    except (Phase118ExecutionError, OSError) as exc:
        print(f"phase118 raw execution: fail-closed: {exc}", file=sys.stderr)
        raise SystemExit(2)
