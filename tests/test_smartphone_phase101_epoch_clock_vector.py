"""Focused, launch-free checks for the Phase101 raw-only contract."""

from __future__ import annotations

import importlib.util
from pathlib import Path


ROOT = Path(__file__).resolve().parents[1]
EVALUATOR_PATH = ROOT / "apps/commands/benchmarks/gnss_smartphone_phase101_epoch_clock_vector.py"


def evaluator():
    spec = importlib.util.spec_from_file_location("phase101_contract_test_module", EVALUATOR_PATH)
    assert spec is not None and spec.loader is not None
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


def test_phase101_freeze_is_pinned_and_single_candidate():
    module = evaluator()
    freeze = module.verify_freeze()
    assert freeze["phase"] == 101
    assert freeze["exactly_one_candidate"]["candidate_id"] == module.CANDIDATE_ID
    assert freeze["scope"]["routes_for_later_structural_qualification"] == list(module.ROUTES)


def test_phase95_path_metadata_is_exact_and_unread():
    module = evaluator()
    paths = module.phase95_paths()
    assert list(paths) == list(module.ROUTES)
    for route in module.ROUTES:
        assert set(paths[route]) == set(module.RAW_NAMES)
        for pin in paths[route].values():
            assert pin["read_by_runner"] is False
            assert len(pin["sha256"]) == 64


def test_phase101_manifest_is_exact_two_route_raw_only_contract():
    module = evaluator()
    manifest = module.verify_manifest()
    assert [record["dataset_id"] for record in manifest["routes"]] == list(module.ROUTES)
    assert manifest["matrix"]["native_invocations_planned"] == 2
    assert manifest["matrix"]["truth_reads_planned"] == 0
    assert manifest["matrix"]["solution_rows_authorized"] is False


def test_phase101_commands_enable_only_frozen_selectors():
    module = evaluator()
    paths = module.phase95_paths()
    manifest = module.verify_manifest()
    for record in manifest["routes"]:
        route = record["dataset_id"]
        command = record["command"]
        for flag in module.REQUIRED_FLAGS:
            assert command.count(flag) == 1
        module.validate_command(route, command, record, paths[route])
        assert "--native-source-clock-c0d-phase94-stage-diagnostics" not in command
        assert "--native-source-clock-c0d-phase96-main-diagnostics" not in command
        assert "--native-source-clock-c0d-phase98-solver-rank-diagnostic" not in command


def test_phase101_state_contract_is_c7_plus_d_without_global_isb():
    module = evaluator()
    state = module.verify_freeze()["state_contract"]["candidate_representation"]
    manifest_state = module.verify_manifest()["candidate"]["state_and_handoff"]
    assert state["clock_units"] == "metres"
    assert state["drift_units"] == "metres/second"
    assert manifest_state["clock_dimension"] == 7
    assert manifest_state["global_isb_state_count"] == 0
    assert manifest_state["drift_initializer_source"] == "retained EpochSeed.receiver_clock_drift_mps"


def test_phase101_implementation_pins_default_off_qr_and_result_surface():
    module = evaluator()
    module.verify_implementation()
    source = module.APP.read_text(encoding="utf-8")
    config = module.CONFIG.read_text(encoding="utf-8")
    fgo = module.FGO.read_text(encoding="utf-8")
    assert module.SELECTOR in source
    assert "use_native_source_clock_c0d_epoch_vector_parity = false" in config
    assert "epoch_clock_bias_components_m" in fgo
    assert "MULTIFRONTAL_QR" in source or "MULTIFRONTAL_QR" in module.BACKEND.read_text(encoding="utf-8")


def test_phase101_pre_raw_has_zero_activity_and_no_launch():
    module = evaluator()
    pre_raw = module.verify_pre_raw()
    assert pre_raw["raw_reads"] == 0
    assert pre_raw["native_solver_invocations"] == 0
    assert pre_raw["truth_reads"] == 0
    assert pre_raw["accuracy_scored"] is False
    assert pre_raw["solution_output_published"] is False
