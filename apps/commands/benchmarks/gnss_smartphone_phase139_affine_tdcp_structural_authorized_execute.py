#!/usr/bin/env python3
"""Phase139-authorized one-shot entry point for the Phase138 raw lane.

This boundary binds the corrected Phase139 flat-route parser and its
launch-free artifacts to a fresh authorization.  The underlying Phase138
runner performs the existing raw hash, native invocation, opaque-output seal,
and structural validation.  This adapter only verifies the new pins; it does
not alter the native command, recipe, factors, or solver.
"""

from __future__ import annotations

import argparse
import hashlib
import importlib.util
import json
from pathlib import Path
from typing import Any, Mapping


ROOT = Path(__file__).resolve().parents[3]
AUTH = ROOT / (
    "docs/use_cases/records/"
    "smartphone_r5_phase139_affine_tdcp_structural_raw_authorization_v1.json"
)
RESULT = ROOT / (
    "docs/use_cases/records/"
    "smartphone_r5_phase139_affine_tdcp_structural_raw_result_v1.json"
)
BASE_PATH = ROOT / (
    "apps/commands/benchmarks/"
    "gnss_smartphone_phase138_affine_tdcp_structural_authorized_execute.py"
)


def _load_base():
    spec = importlib.util.spec_from_file_location(
        "phase139_fixed_phase138_authorized_wrapper", BASE_PATH
    )
    if spec is None or spec.loader is None:
        raise RuntimeError("unable to load corrected Phase139 wrapper")
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


BASE = _load_base()
# The base runner resolves these globals at call time.  Keep its execution
# and structural projection unchanged while binding the fresh Phase139 lane.
BASE.AUTH = AUTH
BASE.RESULT = RESULT

SCHEMA_VERSION = "smartphone-r5-phase138-affine-tdcp-structural-raw-authorization.v1"
PHASE139_AUDIT_COMMIT = "5b1d03ea255f24a57208e1bd9f17f50e345483de"
PHASE139_FREEZE_COMMIT = "e20e3891d68dcadc2ef3f2b2ac5e9a315de73eb8"
PHASE139_IMPLEMENTATION_COMMIT = "7c2634821b0584ef3f6a9f2847c69b9201938275"
PHASE139_MANIFEST_COMMIT = "13eb9cb2db98518836e2e52cc6699c0e3f57d95e"
PHASE139_PRE_RAW_COMMIT = "ff7762799e6f74aeb2e4e0c008b87547cc29f7ac"
PHASE139_AUDIT = ROOT / (
    "docs/use_cases/records/"
    "smartphone_r5_phase139_wrapper_schema_audit_v1.md"
)
PHASE139_FREEZE = ROOT / (
    "docs/use_cases/records/"
    "smartphone_r5_phase139_wrapper_schema_freeze_v1.json"
)
PHASE139_MANIFEST = ROOT / (
    "docs/use_cases/records/"
    "smartphone_r5_phase139_wrapper_schema_manifest_v1.json"
)
PHASE139_PRE_RAW = ROOT / (
    "docs/use_cases/records/"
    "smartphone_r5_phase139_wrapper_schema_pre_raw_accounting_v1.json"
)
PHASE139_WRAPPER_SHA256 = (
    "5ac690c15261d6e7c11dde0075963eee2e0d6098249f924c027b1d95a8bd590b"
)
PHASE139_TEST_SHA256 = (
    "ce4a912f5eb225ff349cf550d31d97a0504e4ea889c52848cfa18a5216a2a988"
)
PHASE139_MANIFEST_SHA256 = (
    "cfe647d3d49ba103ef9334946547de5d1cbba5270eb57be045ee64cfa31b6e42"
)
PHASE139_FREEZE_SHA256 = (
    "e3d516a6bb620ef8b0b0a70555059f887a76855d96db2fbf6795cbe7e29bd20c"
)
PHASE139_AUDIT_SHA256 = (
    "a1f6db77375194f54efd2b1564544601868e687943e776c9ca900494b05a501f"
)
PHASE139_PRE_RAW_SHA256 = (
    "1dd4fdd9be9e1e88d52eea8060ebf38539ac8fd6a4bd3890e60935f743d46347"
)


def _fail(message: str):
    return BASE.AuthorizationError(message)


def _mapping(value: Any, label: str) -> Mapping[str, Any]:
    if not isinstance(value, Mapping):
        raise _fail(f"{label}: expected object")
    return value


def _digest(path: Path, expected: str, label: str) -> None:
    digest = hashlib.sha256()
    try:
        with path.open("rb") as handle:
            for chunk in iter(lambda: handle.read(1024 * 1024), b""):
                digest.update(chunk)
    except OSError as exc:
        raise _fail(f"{label}: {exc}") from exc
    actual = digest.hexdigest()
    if actual != expected:
        raise _fail(f"{label}: expected {expected}, got {actual}")


def _zero_accounting(value: Any, label: str) -> None:
    account = _mapping(value, label)
    for key, item in account.items():
        if isinstance(item, bool):
            if item is not False:
                raise _fail(f"{label}/{key}: expected false")
        elif isinstance(item, int):
            if item != 0:
                raise _fail(f"{label}/{key}: expected zero")
        elif item not in {"read-only", "sealed-metadata-only"}:
            raise _fail(f"{label}/{key}: invalid pre-raw marker")


def _verify_phase139_artifacts(pins: Mapping[str, Any]) -> None:
    for key, expected in {
        "phase139_audit_sha256": PHASE139_AUDIT_SHA256,
        "phase139_freeze_sha256": PHASE139_FREEZE_SHA256,
        "phase139_manifest_sha256": PHASE139_MANIFEST_SHA256,
        "phase139_pre_raw_sha256": PHASE139_PRE_RAW_SHA256,
        "phase139_wrapper_sha256": PHASE139_WRAPPER_SHA256,
        "phase139_focused_test_sha256": PHASE139_TEST_SHA256,
    }.items():
        if pins.get(key) != expected:
            raise _fail(f"authorization/pins/{key}: digest mismatch")
    _digest(PHASE139_AUDIT, PHASE139_AUDIT_SHA256, "Phase139 audit")
    _digest(PHASE139_FREEZE, PHASE139_FREEZE_SHA256, "Phase139 freeze")
    _digest(PHASE139_MANIFEST, PHASE139_MANIFEST_SHA256, "Phase139 manifest")
    _digest(PHASE139_PRE_RAW, PHASE139_PRE_RAW_SHA256, "Phase139 pre-raw")
    _digest(BASE_PATH, PHASE139_WRAPPER_SHA256, "Phase139 fixed wrapper")
    _digest(
        ROOT / "tests/test_smartphone_phase139_wrapper_schema.py",
        PHASE139_TEST_SHA256,
        "Phase139 focused tests",
    )
    manifest = BASE.read_object(PHASE139_MANIFEST, "Phase139 manifest")
    if manifest.get("schema_version") != "smartphone-r5-phase139-wrapper-schema-manifest.v1":
        raise _fail("Phase139 manifest schema mismatch")
    if manifest.get("implementation_commit") != PHASE139_IMPLEMENTATION_COMMIT:
        raise _fail("Phase139 manifest implementation pin mismatch")
    if manifest.get("freeze_sha256") != PHASE139_FREEZE_SHA256:
        raise _fail("Phase139 manifest freeze digest mismatch")
    wrapper = _mapping(manifest.get("wrapper"), "Phase139 manifest/wrapper")
    if wrapper.get("sha256") != PHASE139_WRAPPER_SHA256:
        raise _fail("Phase139 manifest wrapper digest mismatch")
    focused = _mapping(manifest.get("focused_tests"), "Phase139 manifest/focused_tests")
    if focused.get("sha256") != PHASE139_TEST_SHA256:
        raise _fail("Phase139 manifest test digest mismatch")
    freeze = BASE.read_object(PHASE139_FREEZE, "Phase139 freeze")
    if freeze.get("schema_version") != "smartphone-r5-phase139-wrapper-schema-freeze.v1":
        raise _fail("Phase139 freeze schema mismatch")
    freeze_audit_commit = freeze.get("audit", {}).get("commit")
    if (not isinstance(freeze_audit_commit, str) or
            not PHASE139_AUDIT_COMMIT.startswith(freeze_audit_commit)):
        raise _fail("Phase139 freeze audit pin mismatch")
    pre_raw = BASE.read_object(PHASE139_PRE_RAW, "Phase139 pre-raw")
    if pre_raw.get("implementation", {}).get("commit") != PHASE139_IMPLEMENTATION_COMMIT:
        raise _fail("Phase139 pre-raw implementation pin mismatch")
    if pre_raw.get("manifest", {}).get("commit") != PHASE139_MANIFEST_COMMIT:
        raise _fail("Phase139 pre-raw manifest pin mismatch")
    _zero_accounting(pre_raw.get("pre_raw_read_accounting"), "Phase139 pre-raw")


def verify_phase139(auth: Mapping[str, Any]) -> None:
    """Verify a new authorization without opening any raw payload."""
    if auth.get("schema_version") != SCHEMA_VERSION:
        raise _fail("authorization/schema_version mismatch")
    if auth.get("phase") != 138:
        raise _fail("authorization/phase must remain 138 for the Phase138 lane")
    if auth.get("status") != "independent-one-shot-structural-raw-authorized":
        raise _fail("authorization/status mismatch")
    authorization = _mapping(auth.get("authorization"), "authorization")
    for key in ("implementation", "contract", "raw_materialization",
                "raw_structural_execution", "solver"):
        if authorization.get(key) is not True:
            raise _fail(f"authorization/{key}: required true")
    for key in ("truth_evaluation", "accuracy", "solution_publication",
                "kaggle_submission", "rerun", "fallback", "repair", "sweep"):
        if authorization.get(key) is not False:
            raise _fail(f"authorization/{key}: required false")

    scope = _mapping(auth.get("authorization_scope"), "authorization_scope")
    if scope.get("candidate_id") != "phase138-affine-tdcp-anchor-range-constant-v1":
        raise _fail("authorization candidate mismatch")
    if scope.get("route_order") != ["MTV-A", "LAX-T"]:
        raise _fail("authorization route order mismatch")
    for key, expected in {
        "runs_per_route": 1,
        "controls": 0,
        "reruns": 0,
        "fallbacks": 0,
        "repairs": 0,
        "sweeps": 0,
    }.items():
        if scope.get(key) != expected:
            raise _fail(f"authorization_scope/{key}: expected {expected}")

    pins = _mapping(auth.get("pins"), "authorization/pins")
    expected_commits = {
        "phase139_audit_commit": PHASE139_AUDIT_COMMIT,
        "phase139_freeze_commit": PHASE139_FREEZE_COMMIT,
        "phase139_implementation_commit": PHASE139_IMPLEMENTATION_COMMIT,
        "phase139_manifest_commit": PHASE139_MANIFEST_COMMIT,
        "phase139_pre_raw_commit": PHASE139_PRE_RAW_COMMIT,
        "phase138_implementation_commit": BASE.STATIC.IMPLEMENTATION_COMMIT,
    }
    for key, expected in expected_commits.items():
        if pins.get(key) != expected:
            raise _fail(f"authorization/pins/{key}: expected {expected}")
    if pins.get("phase139_wrapper_path") != str(BASE_PATH.relative_to(ROOT)):
        raise _fail("authorization/pins/phase139_wrapper_path mismatch")
    if pins.get("authorized_runner_path") != str(Path(__file__).relative_to(ROOT)):
        raise _fail("authorization/pins/authorized_runner_path mismatch")
    authorized_sha = pins.get("authorized_runner_sha256")
    if not isinstance(authorized_sha, str) or len(authorized_sha) != 64:
        raise _fail("authorization/pins/authorized_runner_sha256 missing")
    _digest(Path(__file__), authorized_sha, "authorized Phase139 runner")
    if pins.get("phase138_target_binary_sha256") != BASE.STATIC.TARGET_BINARY_SHA256:
        raise _fail("authorization target binary pin mismatch")
    _verify_phase139_artifacts(pins)

    # This static validation reads only tracked source/contract artifacts.
    static = BASE.STATIC.launch_free_validation()
    if static.get("raw_execution_authorized") is not False:
        raise _fail("historical Phase138 contract is unexpectedly raw-authorized")

    recipe = _mapping(auth.get("recipe"), "authorization/recipe")
    for key in (
        "phase135_official_affine_measurement_family",
        "phase138_affine_tdcp_anchor_range_constant",
        "phase118_official_tdcp_huber_k",
        "phase107_raw_base_compensation",
        "phase107_raw_base_source_miss_mask",
    ):
        if recipe.get(key) is not True:
            raise _fail(f"recipe/{key}: required true")
    for key in (
        "phase117_dynamic_tdcp_sigma",
        "phase120_official_tdcp_resl_atmosphere_cancellation",
        "phase126_raw_base_source_complete",
        "phase127_glonass_channel_provenance",
        "phase128_glonass_provenance_parser_admission",
        "phase129_glonass_local_miss_mask",
        "phase130_shared_ledger_key_local_support",
        "phase131_canonical_correction_band_key",
        "phase132_typed_canonical_preflight",
        "phase133_runner_native_selector_boundary",
        "phase134_native_summary_bridge",
        "phase107_preserve_additional_frequency_bands",
    ):
        if recipe.get(key) is not False:
            raise _fail(f"recipe/{key}: forbidden selector active")
    for key, expected in {
        "phase118_fixed_tdcp_sigma_m": 0.03,
        "phase118_highway_huber_k": 0.5,
        "main_linear_solver": "MULTIFRONTAL_QR",
        "main_elimination": "EliminateQR",
        "c7_dimension": 7,
        "c_units": "metres",
        "d_units": "metres/second",
        "pixel5_offset_applications": 1,
    }.items():
        if recipe.get(key) != expected:
            raise _fail(f"recipe/{key}: expected {expected!r}")
    forbidden = _mapping(auth.get("forbidden"), "authorization/forbidden")
    for key in ("truth", "MAT", "PDC", "precomputed_coordinates",
                "accuracy", "Kaggle", "solution_publication"):
        if forbidden.get(key) is not True:
            raise _fail(f"authorization/forbidden/{key}: required true")
    _zero_accounting(auth.get("pre_authorization_read_accounting"),
                     "authorization/pre_authorization_read_accounting")
    routes = auth.get("routes")
    if not isinstance(routes, list) or len(routes) != 2:
        raise _fail("authorization/routes must contain exactly two records")
    for index, expected in enumerate(("MTV-A", "LAX-T")):
        record = routes[index]
        if not isinstance(record, Mapping) or record.get("target") != expected:
            raise _fail(f"authorization/routes/{index}: order mismatch")
        # Phase139's corrected parser validates the flat metadata schema and
        # path safety without testing existence or reading payload bytes.
        BASE.actual_payload_paths(record)


# BASE.run() resolves this symbol in its own module namespace.  Replacing it
# is the only adapter hook; all route execution/normalization stays pinned to
# the corrected Phase139 implementation.
BASE.verify_authorization = verify_phase139


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--authorization", type=Path, default=AUTH)
    parser.add_argument(
        "--verify-authorization",
        action="store_true",
        help="verify Phase139 pins without materializing raw input",
    )
    args = parser.parse_args()
    try:
        auth = BASE.read_object(args.authorization, "Phase139 authorization")
        if args.verify_authorization:
            verify_phase139(auth)
            print("PHASE139_AUTHORIZATION_VERIFIED: raw reads=0 solver=0")
            return 0
        return BASE.run(args.authorization)
    except BASE.AuthorizationError as exc:
        print(f"PHASE139_RAW_FAIL_CLOSED: {exc}")
        return 2


if __name__ == "__main__":
    raise SystemExit(main())
