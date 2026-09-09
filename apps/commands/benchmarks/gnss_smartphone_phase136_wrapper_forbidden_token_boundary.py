#!/usr/bin/env python3
"""Launch-free Phase136 wrapper-boundary qualification.

This validator reads only pinned source and sealed metadata.  It exercises the
authorized wrapper's typed argv check with synthetic values; it never opens a
raw payload, launches the native solver, reads a solution/truth row, or
accesses MAT/PDC/precomputed/Kaggle resources.
"""

from __future__ import annotations

import importlib.util
import json
from pathlib import Path
from typing import Any, Mapping


ROOT = Path(__file__).resolve().parents[3]
AUDIT = ROOT / (
    "docs/use_cases/records/"
    "smartphone_r5_phase136_wrapper_forbidden_token_boundary_audit_v1.md"
)
FREEZE = ROOT / (
    "docs/use_cases/records/"
    "smartphone_r5_phase136_wrapper_forbidden_token_boundary_freeze_v1.json"
)
MANIFEST = ROOT / (
    "docs/use_cases/records/"
    "smartphone_r5_phase136_wrapper_forbidden_token_boundary_manifest_v1.json"
)
AUTHORIZED = ROOT / (
    "apps/commands/benchmarks/"
    "gnss_smartphone_phase135_official_affine_structural_authorized_execute.py"
)
NATIVE_APP = ROOT / "apps/native/gnss_fgo_imu_no_base.cpp"


class Phase136QualificationError(ValueError):
    """A launch-free Phase136 boundary violation."""


def _load_authorized_wrapper():
    spec = importlib.util.spec_from_file_location(
        "phase136_authorized_wrapper_for_qualification", AUTHORIZED
    )
    if spec is None or spec.loader is None:
        raise Phase136QualificationError("unable to load authorized wrapper")
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


def _read_json(path: Path, label: str) -> dict[str, Any]:
    try:
        value = json.loads(path.read_text(encoding="utf-8"))
    except (OSError, UnicodeDecodeError, json.JSONDecodeError) as exc:
        raise Phase136QualificationError(f"{label}: {exc}") from exc
    if not isinstance(value, dict):
        raise Phase136QualificationError(f"{label}: expected object")
    return value


def _assert(condition: bool, message: str) -> None:
    if not condition:
        raise Phase136QualificationError(message)


def _safe_command(wrapper: Any, route: str) -> list[str]:
    paths = {
        "device_gnss.csv": Path("/synthetic/raw/device_gnss.csv"),
        "device_imu.csv": Path("/synthetic/raw/device_imu.csv"),
        "brdc.nav": Path("/synthetic/raw/brdc.nav"),
        "base.obs": Path("/synthetic/base/base.obs"),
    }
    command = wrapper.command_for(route, paths, Path("/synthetic/opaque"))
    command[command.index("--native-base-rinex-sha256") + 1] = "b" * 64
    return command


def _assert_rejected(wrapper: Any, command: list[str], route: str) -> None:
    try:
        wrapper.verify_command_raw_only(command, route)
    except (wrapper.AuthorizationError, ValueError):
        return
    raise Phase136QualificationError("synthetic forbidden argv was accepted")


def launch_free_validation() -> dict[str, Any]:
    freeze = _read_json(FREEZE, "freeze")
    manifest = _read_json(MANIFEST, "manifest")
    _assert(freeze.get("phase") == 136, "freeze phase is not 136")
    _assert(manifest.get("phase") == 136, "manifest phase is not 136")
    _assert(freeze.get("decision", {}).get("candidate_count") == 1,
            "freeze does not select exactly one candidate")
    _assert(freeze["decision"]["raw_materialization_authorized"] is False,
            "freeze unexpectedly authorizes raw materialization")
    _assert(freeze["decision"]["solver_execution_authorized"] is False,
            "freeze unexpectedly authorizes solver execution")
    _assert(manifest.get("raw_materialization_authorized") is False,
            "manifest unexpectedly authorizes raw materialization")
    _assert(manifest.get("solver_execution_authorized") is False,
            "manifest unexpectedly authorizes solver execution")

    source = NATIVE_APP.read_text(encoding="utf-8")
    required_option = "--native-pdc-imu-tdcp-no-bridge"
    _assert(required_option in source, "native required option missing from source")
    _assert("options.native_pdc_imu_tdcp_no_bridge = true" in source,
            "native parser ownership is missing")

    wrapper = _load_authorized_wrapper()
    routes = tuple(wrapper.STATIC.ROUTES)
    for route in routes:
        command = _safe_command(wrapper, route)
        wrapper.verify_command_raw_only(command, route)
        _assert(command.count(required_option) == 1,
                f"{route}: required native option count is not one")
        expected_options = [
            token for token in wrapper.STATIC.command_template(route)
            if token.startswith("--")
        ]
        actual_options = [token for token in command if token.startswith("--")]
        _assert(actual_options == expected_options,
                f"{route}: option ownership/order changed")

        forbidden_path = list(command)
        forbidden_path[forbidden_path.index("--android-gnss") + 1] = (
            "/synthetic/pdc/device_gnss.csv"
        )
        _assert_rejected(wrapper, forbidden_path, route)

        forbidden_value = list(command)
        forbidden_value[forbidden_value.index("--dataset-id") + 1] = (
            "pdc-artifact-value"
        )
        _assert_rejected(wrapper, forbidden_value, route)

        unknown = list(command)
        unknown[unknown.index("--native-source-direct-observable-quality")] = (
            "--unknown-selector"
        )
        _assert_rejected(wrapper, unknown, route)

        duplicate = list(command)
        duplicate[duplicate.index("--native-source-direct-observable-quality")] = (
            required_option
        )
        _assert_rejected(wrapper, duplicate, route)

    accounting = manifest.get("pre_raw_read_accounting")
    _assert(isinstance(accounting, Mapping), "manifest pre-raw accounting missing")
    for key, value in accounting.items():
        if isinstance(value, bool):
            _assert(value is False, f"manifest/{key}: nonzero boolean read")
        elif isinstance(value, int):
            _assert(value == 0, f"manifest/{key}: nonzero read")

    return {
        "status": "launch-free-qualified",
        "phase": 136,
        "routes": list(routes),
        "required_native_pdc_algorithm_option": {
            "token": required_option,
            "accepted_exactly_once": True,
        },
        "forbidden_input_path_value": "rejected",
        "unknown_selector": "rejected",
        "duplicate_required_option": "rejected",
        "raw_phone_gnss_reads": 0,
        "raw_phone_imu_reads": 0,
        "broadcast_navigation_reads": 0,
        "raw_base_rinex_reads": 0,
        "native_solver_invocations": 0,
        "solution_coordinate_reads": 0,
        "truth_reads": 0,
        "mat_pdc_precomputed_reads": 0,
        "kaggle_or_token_access": 0,
    }


def main() -> int:
    result = launch_free_validation()
    print(json.dumps(result, indent=2, sort_keys=True))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
