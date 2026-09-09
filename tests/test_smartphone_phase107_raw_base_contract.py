"""Launch-free contract tests for the Phase107 raw-base admission.

These tests import only the pre-raw evaluator.  They do not stat or open a
device raw file or a base RINEX member and do not launch the native binary.
"""

from __future__ import annotations

import importlib.util
import json
from pathlib import Path


ROOT = Path(__file__).resolve().parents[1]
EVALUATOR_PATH = ROOT / "apps/commands/benchmarks/gnss_smartphone_phase107_raw_base_source_parity.py"


def evaluator():
    spec = importlib.util.spec_from_file_location("phase107_contract_tests", EVALUATOR_PATH)
    assert spec is not None and spec.loader is not None
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


def test_freeze_and_pre_raw_are_pinned_without_input_activity():
    module = evaluator()
    module.verify_freeze()
    result = module.verify_pre_raw()
    assert result["status"] == "pre-raw-verified"
    assert result["raw_reads"] == 0
    assert result["base_payload_reads"] == 0
    assert result["base_hash_verification_reads"] == 0
    assert result["native_solver_invocations"] == 0
    assert result["truth_reads"] == 0
    assert result["mat_reads_or_generated"] == 0


def test_manifest_pins_exact_two_routes_and_phase107_candidate():
    module = evaluator()
    manifest = module.verify_manifest()
    assert [item["dataset_id"] for item in manifest["routes"]] == list(module.ROUTES)
    assert manifest["candidate"]["id"] == module.CANDIDATE_ID
    assert manifest["candidate"]["candidate_count"] == 1
    assert manifest["candidate"]["default_off_outside_this_command"] is True
    assert manifest["candidate"]["base_correction_applied_exactly_once"] is True


def test_manifest_commands_use_raw_and_base_placeholders_exactly_once():
    module = evaluator()
    manifest = module.verify_manifest()
    for route in manifest["routes"]:
        command = route["command"]
        assert command.count("--native-base-rinex") == 1
        assert command.count("--native-base-rinex-sha256") == 1
        assert command[command.index("--native-base-rinex") + 1] == "__PHASE107_RAW_BASE_RINEX__"
        assert command[command.index("--native-base-rinex-sha256") + 1] == "__PHASE107_RAW_BASE_SHA256__"
        for flag, _, placeholder in module.RAW_FLAGS:
            assert command.count(flag) == 1
            assert command[command.index(flag) + 1] == placeholder


def test_manifest_forbids_alternate_solver_or_coordinate_lanes():
    module = evaluator()
    manifest = module.verify_manifest()
    for route in manifest["routes"]:
        command = route["command"]
        assert "--native-base-pseudorange-preserve-additional-frequency-bands" not in command
        assert "--obs" not in command
        assert "--native-pdc-state-bridge" not in command
        assert "--native-direct-doppler-wls-handoff" not in command
        assert "--native-source-clock-c0d-gnss-first-raw-drift-d-initializer" not in command
        assert "--native-source-clock-c0d-phase98-solver-rank-diagnostic" not in command
        assert "--native-phase104-stage-main-attribution" not in command


def test_sealed_base_metadata_is_exact_and_pre_raw_marked_unread():
    module = evaluator()
    base = module.phase65_base_metadata()
    assert base[module.ROUTES[0]]["sha256"] == "380b8ff9091344fb756697e27f0983d9a0ba2cf0c201b96849bfc1ecc1af0e52"
    assert base[module.ROUTES[0]]["bytes"] == 10708536
    assert base[module.ROUTES[0]]["observed_dt_s"] == 1.0
    assert base[module.ROUTES[0]]["moving_mean_samples"] == 151
    assert base[module.ROUTES[1]]["sha256"] == "d731e0e8a7ba4396d62340c85b6238e66c50c6a349621f9dcea0ea6885fd4cfe"
    assert base[module.ROUTES[1]]["bytes"] == 719969
    assert base[module.ROUTES[1]]["observed_dt_s"] == 15.0
    assert all(pin["read_at_manifest_creation"] is False for pin in base.values())
    assert all(pin["hash_read_at_manifest_creation"] is False for pin in base.values())


def test_raw_metadata_is_phase95_sealed_without_payload_access():
    module = evaluator()
    raw = module.phase95_raw_metadata()
    for route in module.ROUTES:
        for name in module.RAW_NAMES:
            assert raw[route][name]["read_at_manifest_creation"] is False
            assert raw[route][name]["read_by_pre_raw_verifier"] is False
            assert len(raw[route][name]["sha256"]) == 64


def test_phase107_source_admits_only_existing_correction_path():
    source = (ROOT / "apps/native/gnss_fgo_imu_no_base.cpp").read_text(encoding="utf-8")
    assert "phase107_raw_base_source_parity_admission" in source
    assert "base_pseudorange_compensation::subtractCorrection" in source
    assert "gnss_first_problem = problem" in source
    assert "native_base_pseudorange_source_miss_mask" in source


def test_legacy_selector_remains_default_off():
    config = (ROOT / "include/libgnss++/algorithms/fgo_config.hpp").read_text(encoding="utf-8")
    assert "use_native_source_clock_c0d_gnss_first_meter_state_handoff = false" in config
    assert "use_native_source_clock_c0d_epoch_vector_parity = false" in config
    assert "use_native_source_clock_c0d_phase99_main_multifrontal_qr_solver =\n            false" in config


def test_wrapper_hashes_only_base_after_authorization_and_withholds_solution():
    wrapper = (ROOT / "apps/commands/benchmarks/gnss_smartphone_phase107_raw_base_source_parity_execute.py").read_text(encoding="utf-8")
    assert "load_pinned_contract()" in wrapper
    assert "hash_base_member" in wrapper
    assert "runner_read_raw_payloads\": False" in wrapper
    assert "solution_output_opened\": False" in wrapper
    assert "solution_output_published\": False" in wrapper
    assert "subprocess.run" in wrapper


def test_pre_raw_evaluator_has_no_solver_launch_api():
    source = EVALUATOR_PATH.read_text(encoding="utf-8")
    assert "import subprocess" not in source
    assert "subprocess.run" not in source
    assert "os.system" not in source
