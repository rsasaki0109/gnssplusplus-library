#!/usr/bin/env python3
"""One-shot Phase135 structural raw executor.

This wrapper is intentionally the only post-authorization launch boundary for
Phase135.  It validates the sealed static contract, hashes each already-sealed
raw input exactly once after authorization, runs MTV-A then LAX-T at most once,
and records only opaque solution metadata plus native structural telemetry.
It never opens truth, MAT, PDC, precomputed-coordinate, or accuracy files and
does not print or parse solution coordinates.
"""

from __future__ import annotations

import argparse
import hashlib
import importlib.util
import json
import os
from pathlib import Path
import subprocess
import sys
from typing import Any, Mapping


ROOT = Path(__file__).resolve().parents[3]
AUTH = ROOT / (
    "docs/use_cases/records/"
    "smartphone_r5_phase135_official_affine_structural_raw_authorization_v1.json"
)
RUNNER_PATH = ROOT / (
    "apps/commands/benchmarks/"
    "gnss_smartphone_phase135_official_affine_structural.py"
)
RESULT = ROOT / (
    "output/smartphone-r5/phase135-official-affine-structural-v1/"
    "structural_raw_result.json"
)
FORBIDDEN_INPUT_TERMS = (
    ".mat",
    "truth",
    "ground_truth",
    "pdc",
    "precomputed",
    "kaggle",
    "token",
)
OPAQUE_OUTPUT_NAME = "opaque_solution_output.csv"
LEGITIMATE_PDC_ALGORITHM_OPTION = "--native-pdc-imu-tdcp-no-bridge"
ALLOWED_INPUT_NAMES = {"device_gnss.csv", "device_imu.csv", "brdc.nav", "base.obs"}


class AuthorizationError(ValueError):
    """A fail-closed authorization or structural violation."""


def fail(message: str) -> AuthorizationError:
    return AuthorizationError(message)


def load_runner():
    spec = importlib.util.spec_from_file_location("phase135_static_runner", RUNNER_PATH)
    if spec is None or spec.loader is None:
        raise fail("unable to load static Phase135 validator")
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


STATIC = load_runner()


def read_object(path: Path, label: str) -> dict[str, Any]:
    try:
        value = json.loads(path.read_text(encoding="utf-8"))
    except (OSError, UnicodeDecodeError, json.JSONDecodeError) as exc:
        raise fail(f"{label}: {exc}") from exc
    if not isinstance(value, dict):
        raise fail(f"{label}: expected object")
    return value


def sha256_file(path: Path, label: str) -> tuple[str, int]:
    """Hash one authorized payload without retaining or interpreting it."""
    if not path.is_file():
        raise fail(f"{label}: missing authorized input {path}")
    digest = hashlib.sha256()
    size = 0
    with path.open("rb") as handle:
        for chunk in iter(lambda: handle.read(1024 * 1024), b""):
            size += len(chunk)
            digest.update(chunk)
    return digest.hexdigest(), size


def opaque_solution_metadata(path: Path) -> dict[str, Any]:
    """Return hash/byte/newline metadata only; never parse CSV fields."""
    digest = hashlib.sha256()
    size = 0
    rows = 0
    with path.open("rb") as handle:
        for chunk in iter(lambda: handle.read(1024 * 1024), b""):
            size += len(chunk)
            rows += chunk.count(b"\n")
            digest.update(chunk)
    return {
        "path": str(path.relative_to(ROOT)),
        "sha256": digest.hexdigest(),
        "bytes": size,
        "newline_count": rows,
        "content_interpreted": False,
    }


def digest_bytes(data: bytes) -> str:
    return hashlib.sha256(data).hexdigest()


def _require(mapping: Mapping[str, Any], key: str, label: str) -> Any:
    if key not in mapping:
        raise fail(f"{label}/{key}: missing")
    return mapping[key]


def verify_authorization(auth: Mapping[str, Any]) -> None:
    if auth.get("phase") != 135:
        raise fail("authorization/phase is not 135")
    if auth.get("status") != "independent-one-shot-structural-raw-authorized":
        raise fail("authorization/status is not independent raw authorization")
    authorization = _require(auth, "authorization", "authorization")
    if not isinstance(authorization, Mapping):
        raise fail("authorization/authorization missing")
    for key in ("implementation", "contract", "raw_materialization", "raw_structural_execution", "solver"):
        if authorization.get(key) is not True:
            raise fail(f"authorization/{key}: required true")
    for key in ("truth_evaluation", "accuracy", "solution_publication", "kaggle_submission", "rerun", "fallback", "repair", "sweep"):
        if authorization.get(key) is not False:
            raise fail(f"authorization/{key}: required false")
    scope = _require(auth, "authorization_scope", "authorization")
    if not isinstance(scope, Mapping):
        raise fail("authorization/authorization_scope missing")
    if scope.get("route_order") != ["MTV-A", "LAX-T"]:
        raise fail("authorization route order is not MTV-A then LAX-T")
    if scope.get("runs_per_route") != 1 or scope.get("controls") != 0:
        raise fail("authorization one-shot policy violation")
    pins = _require(auth, "pins", "authorization")
    if not isinstance(pins, Mapping):
        raise fail("authorization/pins missing")
    expected_pins = {
        "source_audit_commit": "01b7b68a37c668422c7ca396cfe56edab706f819",
        "doppler_correction_commit": "f9a1fc9403e7072435a30d9e06aa8ea59493cdc5",
        "structural_freeze_commit": "bd1ef39ba9a7a29c210d953a7edc1e60011fb7bd",
        "runner_manifest_commit": "be2f83721df24912f29cb59ae1c26a342d8701b5",
        "pre_raw_accounting_commit": "142761dd530768ee6f4b20b181fe099ccc4db000",
    }
    for key, expected in expected_pins.items():
        if pins.get(key) != expected:
            raise fail(f"authorization/pins/{key}: expected {expected}")
    # The parent pin is intentionally checked against the actual full Git
    # object below.  The static historical documents retain their own pinned
    # metadata and are verified separately by launch_free_validation().
    if pins.get("doppler_correction_commit_canonical") != "f9a1fc9403e7072435a30d9e06aa8ea59493cdc5":
        raise fail("authorization canonical Doppler correction pin mismatch")
    if pins.get("freeze_sha256") != STATIC.FREEZE_SHA256:
        raise fail("authorization freeze SHA mismatch")
    if pins.get("manifest_sha256") != "867262de16b5a9684113331153af5212fba68966ab4835a1aa81c504a98a7ce1":
        raise fail("authorization manifest SHA mismatch")
    if pins.get("pre_raw_sha256") != "68c57fe1d1ddac818ae45cd3c4d10d0975df70169798071220283ec91c1c40df":
        raise fail("authorization pre-raw SHA mismatch")
    if pins.get("target_binary_sha256") != STATIC.TARGET_BINARY_SHA256:
        raise fail("authorization binary SHA mismatch")
    static_result = STATIC.launch_free_validation()
    if static_result["raw_execution_authorized"] is not False:
        raise fail("static contract is already marked raw-authorized")
    recipe = _require(auth, "recipe", "authorization")
    if not isinstance(recipe, Mapping):
        raise fail("authorization/recipe missing")
    if recipe.get("phase135_official_affine_measurement_family") is not True:
        raise fail("Phase135 selector is not active")
    if recipe.get("phase118_official_tdcp_huber_k") is not True:
        raise fail("Phase118 selector is not active")
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
            raise fail(f"recipe/{key}: forbidden selector active")
    if recipe.get("phase107_raw_base_compensation") is not True or recipe.get("phase107_raw_base_source_miss_mask") is not True:
        raise fail("Phase107 raw-base recipe is incomplete")
    if recipe.get("phase118_fixed_tdcp_sigma_m") != 0.03 or recipe.get("phase118_highway_huber_threshold_sigma") != 0.8:
        raise fail("Phase118 sigma/Huber recipe changed")
    if recipe.get("main_linear_solver") != "MULTIFRONTAL_QR" or recipe.get("main_elimination") != "EliminateQR":
        raise fail("QR solver recipe changed")
    accounting = _require(auth, "pre_authorization_read_accounting", "authorization")
    if not isinstance(accounting, Mapping):
        raise fail("pre-authorization accounting missing")
    for key, value in accounting.items():
        if isinstance(value, bool):
            if value is not False:
                raise fail(f"pre-authorization/{key}: nonzero")
        elif isinstance(value, int) and value != 0:
            raise fail(f"pre-authorization/{key}: nonzero")
    forbidden = _require(auth, "forbidden", "authorization")
    if not isinstance(forbidden, Mapping):
        raise fail("authorization/forbidden missing")
    for key in ("truth", "MAT", "PDC", "precomputed_coordinates", "accuracy", "Kaggle", "solution_publication"):
        if forbidden.get(key) is not True:
            raise fail(f"authorization/forbidden/{key}: missing true")


def verify_static_pins(auth: Mapping[str, Any]) -> None:
    pins = auth["pins"]
    static_paths = {
        "freeze_sha256": STATIC.FREEZE,
        "manifest_sha256": STATIC.MANIFEST,
        "runner_sha256": STATIC.RUNNER,
        "target_binary_sha256": STATIC.NATIVE_BINARY,
        "pre_raw_sha256": ROOT / "docs/use_cases/records/smartphone_r5_phase135_official_affine_structural_pre_raw_accounting_v1.json",
        "authorized_runner_sha256": ROOT / "apps/commands/benchmarks/gnss_smartphone_phase135_official_affine_structural_authorized_execute.py",
    }
    for key, path in static_paths.items():
        digest, _ = sha256_file(path, key)
        if digest != pins.get(key):
            raise fail(f"static pin {key}: expected {pins.get(key)}, got {digest}")


def actual_payload_paths(route: Mapping[str, Any]) -> dict[str, Path]:
    raw = _require(route, "raw_inputs", "route")
    base = _require(route, "base_input", "route")
    if not isinstance(raw, Mapping) or not isinstance(base, Mapping):
        raise fail("route raw/base input metadata missing")
    result: dict[str, Path] = {}
    for name, metadata in raw.items():
        if name not in ALLOWED_INPUT_NAMES or name == "base.obs":
            raise fail(f"unsupported raw input name {name!r}")
        if not isinstance(metadata, Mapping):
            raise fail(f"raw input metadata {name} missing")
        result[name] = ROOT / str(_require(metadata, "path", f"raw/{name}"))
    if "base.obs" in raw:
        raise fail("base.obs must be declared in base_input only")
    result["base.obs"] = ROOT / str(_require(base, "path", "base"))
    for name, path in result.items():
        lowered = str(path).lower()
        if any(term in lowered for term in FORBIDDEN_INPUT_TERMS):
            raise fail(f"forbidden input path {path}")
        if path.name != name:
            raise fail(f"input basename mismatch for {name}: {path.name}")
    return result


def command_for(route: str, paths: Mapping[str, Path], output_dir: Path) -> list[str]:
    command = list(STATIC.command_template(route))
    substitutions = {
        STATIC.RAW_PLACEHOLDERS["--android-gnss"]: str(paths["device_gnss.csv"]),
        STATIC.RAW_PLACEHOLDERS["--android-imu"]: str(paths["device_imu.csv"]),
        STATIC.RAW_PLACEHOLDERS["--nav"]: str(paths["brdc.nav"]),
        STATIC.RAW_PLACEHOLDERS["--native-base-rinex"]: str(paths["base.obs"]),
        STATIC.RAW_PLACEHOLDERS["--native-base-rinex-sha256"]: "",
    }
    for index, token in enumerate(command):
        if token in substitutions:
            command[index] = substitutions[token]
    # The base digest is supplied from the authorization metadata after the
    # base file is hashed, so the caller fills this exact token below.
    command[command.index("--out") + 1] = str(output_dir / "opaque_solution_output.csv")
    command[command.index("--summary-json") + 1] = str(output_dir / "native_summary.json")
    return command


def _forbidden_input_value(value: str) -> bool:
    """Return whether a non-option argv value names forbidden input/access.

    The raw-only policy applies to paths and values, not to option spellings.
    In particular, the native algorithm selector containing ``pdc`` is not a
    PDC input path and is validated in the option namespace below.
    """
    lowered = value.lower()
    return any(term in lowered for term in FORBIDDEN_INPUT_TERMS)


def _validate_typed_argv(command: list[str], route: str) -> None:
    """Validate argv by its option/value positions, fail-closed.

    The static template is the ownership table for options.  Comparing each
    option position rejects unknown, reordered, or injected selectors while
    allowing the authorized raw paths and output destinations to be replaced.
    Forbidden substring checks are intentionally limited to non-option
    values, where they protect the raw/artifact lineage boundary.
    """
    expected = STATIC.command_template(route)
    if len(command) != len(expected):
        raise fail(
            f"{route}: argv length differs from static template "
            f"({len(command)} != {len(expected)})"
        )
    for index, (actual, template_token) in enumerate(zip(command, expected)):
        if template_token.startswith("--"):
            if actual != template_token:
                if actual.startswith("--") and "pdc" in actual.lower():
                    raise fail(f"{route}: forbidden or unknown PDC option {actual!r}")
                raise fail(
                    f"{route}: option at argv index {index} is {actual!r}; "
                    f"expected {template_token!r}"
                )
            if (
                "pdc" in actual.lower()
                and actual != LEGITIMATE_PDC_ALGORITHM_OPTION
            ):
                raise fail(f"{route}: forbidden PDC algorithm option {actual!r}")
            continue
        if actual.startswith("--"):
            raise fail(
                f"{route}: unknown selector/value injection at argv index "
                f"{index}: {actual!r}"
            )
        opaque_output = (
            index > 0
            and expected[index - 1] == "--out"
            and Path(actual).name == OPAQUE_OUTPUT_NAME
            and not _forbidden_input_value(str(Path(actual).parent))
        )
        if _forbidden_input_value(actual) and not opaque_output:
            raise fail(f"{route}: forbidden input/artifact value {actual!r}")


def verify_command_raw_only(command: list[str], route: str) -> None:
    """Validate the raw-only command without conflating option and path terms."""
    _validate_typed_argv(command, route)
    for selector in STATIC.REQUIRED_RECIPE_FLAGS + STATIC.ON_SELECTORS:
        if command.count(selector) != 1:
            raise fail(f"{route}: selector {selector} count is not one")
    for selector in STATIC.OFF_SELECTORS:
        if selector in command:
            raise fail(f"{route}: forbidden selector {selector}")
    if command.count("--native-base-rinex") != 1 or command.count("--native-base-rinex-sha256") != 1:
        raise fail(f"{route}: base arguments are not exact")


def _digest_and_size(path: Path) -> tuple[str, int]:
    return sha256_file(path, str(path))


def run_one_route(route_record: Mapping[str, Any], auth: Mapping[str, Any]) -> dict[str, Any]:
    route = str(_require(route_record, "dataset_id", "route"))
    paths = actual_payload_paths(route_record)
    expected = {}
    for name, metadata in route_record["raw_inputs"].items():
        expected[name] = (str(_require(metadata, "sha256", f"raw/{name}")), int(_require(metadata, "bytes", f"raw/{name}")))
    base_meta = route_record["base_input"]
    expected["base.obs"] = (str(_require(base_meta, "sha256", "base")), int(_require(base_meta, "bytes", "base")))
    reads: dict[str, int] = {name: 0 for name in paths}
    hashes: dict[str, Any] = {}
    for name in ("device_gnss.csv", "device_imu.csv", "brdc.nav", "base.obs"):
        digest, size = _digest_and_size(paths[name])
        reads[name] = 1
        hashes[name] = {"sha256": digest, "bytes": size, "expected_sha256": expected[name][0], "expected_bytes": expected[name][1], "match": digest == expected[name][0] and size == expected[name][1]}
        if not hashes[name]["match"]:
            return {"route": route, "status": "fail-closed-preflight", "solver_invocations": 0, "read_accounting": reads, "input_hashes": hashes, "failure": f"{name}: sealed hash/size mismatch"}
    out_dir = ROOT / "output/smartphone-r5/phase135-official-affine-structural-v1" / route.replace("/", "__")
    output_path = out_dir / "opaque_solution_output.csv"
    summary_path = out_dir / "native_summary.json"
    stdout_path = out_dir / "native.stdout.log"
    stderr_path = out_dir / "native.stderr.log"
    if any(path.exists() for path in (output_path, summary_path, stdout_path, stderr_path)):
        return {"route": route, "status": "fail-closed-existing-output", "solver_invocations": 0, "read_accounting": reads, "input_hashes": hashes, "failure": "one-shot output path already exists"}
    out_dir.mkdir(parents=True, exist_ok=False)
    command = command_for(route, paths, out_dir)
    command[command.index("--native-base-rinex-sha256") + 1] = hashes["base.obs"]["sha256"]
    verify_command_raw_only(command, route)
    env = os.environ.copy()
    local_lib = "/home/sasaki/.local/lib"
    env["LD_LIBRARY_PATH"] = local_lib + ((":" + env["LD_LIBRARY_PATH"]) if env.get("LD_LIBRARY_PATH") else "")
    completed = subprocess.run(command, cwd=ROOT, env=env, capture_output=True, check=False)
    stdout_path.write_bytes(completed.stdout)
    stderr_path.write_bytes(completed.stderr)
    execution: dict[str, Any] = {
        "command": command,
        "invocation_count": 1,
        "return_code": completed.returncode,
        "stdout_sha256": digest_bytes(completed.stdout),
        "stdout_bytes": len(completed.stdout),
        "stderr_sha256": digest_bytes(completed.stderr),
        "stderr_bytes": len(completed.stderr),
    }
    route_result: dict[str, Any] = {
        "route": route,
        "status": "native-returned",
        "solver_invocations": 1,
        "read_accounting": reads,
        "input_hashes": hashes,
        "execution": execution,
    }
    if output_path.is_file():
        route_result["opaque_solution"] = opaque_solution_metadata(output_path)
    else:
        route_result["opaque_solution"] = None
    if summary_path.is_file():
        summary_bytes = summary_path.read_bytes()
        route_result["native_summary"] = {
            "path": str(summary_path.relative_to(ROOT)),
            "sha256": digest_bytes(summary_bytes),
            "bytes": len(summary_bytes),
            "coordinate_fields_interpreted": False,
        }
        try:
            native = json.loads(summary_bytes)
            route_result["native_summary_schema"] = native.get("schema_version") if isinstance(native, Mapping) else None
            if isinstance(native, Mapping):
                # Copy only structural scalar/metadata fields.  No solution
                # coordinate arrays are traversed or emitted.
                route_result["native_structural_metadata"] = {
                    key: native[key]
                    for key in (
                        "status",
                        "dataset_id",
                        "truth_used",
                        "native_phase117_tdcp_snr_type_sigma",
                        "native_phase118_official_tdcp_huber_k",
                        "native_phase120_official_tdcp_resl_atmosphere_cancellation",
                        "native_phase126_raw_base_source_complete",
                        "native_phase127_glonass_channel_provenance",
                        "native_phase128_glonass_provenance_parser_admission",
                        "native_phase129_glonass_local_miss_mask",
                        "native_phase131_canonical_correction_band_key",
                        "native_source_clock_c0d_factor",
                        "selected_linear_solver_type",
                        "selected_solver_branch",
                        "selected_elimination_function",
                        "phase135_official_affine_measurement_family",
                        "epochs",
                        "upstream_observable_quality",
                        "native_source_clock_c0d_factor",
                        "native_base_pseudorange_compensation",
                        "gnss_first",
                        "main",
                        "output_contract",
                        "upstream_position_offset",
                        "tdcp_contract",
                    )
                    if key in native
                }
        except (UnicodeDecodeError, json.JSONDecodeError):
            route_result["native_summary_parse_error"] = True
    else:
        route_result["native_summary"] = None
    route_result["solution_content_read"] = False
    return route_result


def run(auth_path: Path = AUTH) -> int:
    auth = read_object(auth_path, "authorization")
    verify_authorization(auth)
    verify_static_pins(auth)
    route_records = auth.get("routes")
    if not isinstance(route_records, list) or len(route_records) != 2:
        raise fail("authorization routes must contain exactly two records")
    results = []
    for index, target in enumerate(("MTV-A", "LAX-T")):
        record = route_records[index]
        if not isinstance(record, Mapping) or record.get("target") != target:
            raise fail(f"route order mismatch at index {index}")
        results.append(run_one_route(record, auth))
    result = {
        "schema_version": "smartphone-r5-phase135-official-affine-structural-raw-result.v1",
        "phase": 135,
        "status": "sealed-structural-raw-result",
        "candidate_id": auth["authorization_scope"]["candidate_id"],
        "route_order": ["MTV-A", "LAX-T"],
        "routes": results,
        "policy": {
            "truth_used": False,
            "mat_used": False,
            "pdc_used": False,
            "precomputed_coordinates_used": False,
            "accuracy_evaluation": False,
            "kaggle_access": False,
            "solution_publication": False,
            "rerun": False,
            "fallback": False,
            "repair": False,
            "solution_coordinate_interpretation": False,
        },
        "read_accounting": {
            "raw_phone_gnss_reads": sum(r["read_accounting"].get("device_gnss.csv", 0) for r in results),
            "raw_phone_imu_reads": sum(r["read_accounting"].get("device_imu.csv", 0) for r in results),
            "broadcast_navigation_reads": sum(r["read_accounting"].get("brdc.nav", 0) for r in results),
            "raw_base_rinex_reads": sum(r["read_accounting"].get("base.obs", 0) for r in results),
            "native_solver_invocations": sum(r.get("solver_invocations", 0) for r in results),
            "truth_reads": 0,
            "solution_coordinate_reads": 0,
            "accuracy_calculations": 0,
            "mat_pdc_precomputed_coordinate_reads": 0,
            "kaggle_or_token_access": 0,
            "reruns_fallbacks_repairs_sweeps": 0,
        },
        "solution_policy": "opaque hash/bytes/newline seal only; no coordinate fields read or published",
        "structural_validator": "launch-free validator pin is static; native structural metadata preserved without coordinate interpretation",
        "authorization_commit_is_separate": True,
    }
    RESULT.parent.mkdir(parents=True, exist_ok=True)
    RESULT.write_text(json.dumps(result, indent=2, sort_keys=True) + "\n", encoding="utf-8")
    return 0


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--authorization", type=Path, default=AUTH)
    args = parser.parse_args()
    try:
        return run(args.authorization)
    except AuthorizationError as exc:
        print(f"PHASE135_RAW_FAIL_CLOSED: {exc}")
        return 2


if __name__ == "__main__":
    raise SystemExit(main())
