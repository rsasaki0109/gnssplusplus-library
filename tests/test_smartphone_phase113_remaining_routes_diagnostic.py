"""Focused launch-free checks for the Phase113 diagnostic boundary."""

from __future__ import annotations

import importlib.util
from pathlib import Path


ROOT = Path(__file__).resolve().parents[1]
MODULE_PATH = ROOT / "apps/commands/benchmarks/gnss_smartphone_phase113_remaining_routes_diagnostic.py"
SPEC = importlib.util.spec_from_file_location("phase113_remaining_routes_contract", MODULE_PATH)
assert SPEC is not None and SPEC.loader is not None
MODULE = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(MODULE)


def test_freeze_and_sealed_metadata_are_read_only() -> None:
    freeze = MODULE.verify_freeze()
    raw = MODULE.phase95_raw_metadata()
    base = MODULE.phase65_base_metadata()
    assert freeze["phase"] == 113
    assert freeze["candidate"]["id"] == MODULE.CANDIDATE_ID
    assert freeze["candidate"]["algorithmic_candidate_count"] == 0
    assert list(raw) == list(MODULE.ROUTES)
    assert list(base) == list(MODULE.ROUTES)
    for route in MODULE.ROUTES:
        assert set(raw[route]) == set(MODULE.RAW_NAMES)
        assert base[route]["coordinate_source"] == "raw RINEX header APPROX POSITION XYZ"
        for name in MODULE.RAW_NAMES:
            assert raw[route][name]["read_by_pre_raw_verifier"] is False
            assert raw[route][name]["read_at_manifest_creation"] is False
        assert base[route]["read_by_pre_raw_verifier"] is False


def test_exact_phase113_command_has_no_phase97_or_additional_band_selector() -> None:
    for route in MODULE.ROUTES:
        command = MODULE.command_template(route)
        assert command[0] == "build/apps/gnss_fgo_imu_no_base"
        assert command.count(MODULE.PHASE101_SELECTOR) == 1
        assert command.count(MODULE.VECTOR_SELECTOR) == 1
        assert command.count(MODULE.QR_SELECTOR) == 1
        assert command.count(MODULE.BASE_SELECTOR) == 1
        assert command.count(MODULE.BASE_MISS_SELECTOR) == 1
        assert "--native-source-clock-c0d-phase97-singular-system-diagnostics" not in command
        assert "--native-base-pseudorange-preserve-additional-frequency-bands" not in command
        assert "--native-source-clock-c0d-phase94-stage-diagnostics" not in command
        assert MODULE.output_path(route, "withheld_solution_output.csv") in command


def test_freeze_contract_keeps_algorithm_and_solution_lanes_closed() -> None:
    freeze = MODULE.verify_freeze()
    contract = freeze["candidate"]["unchanged_contract"]
    assert all(contract[key] is True for key in (
        "graph_and_factors", "values_and_initialization", "equations_and_units",
        "sigma_and_filter", "lm_schedule_ordering_and_damping", "no_guard_removal",
        "no_velocity_fallback_or_synthesis", "no_fallback", "no_solution_coordinate_publication",
    ))
    assert freeze["execution_boundary"]["raw_execution_authorized"] is False
    assert freeze["execution_boundary"]["solution_publication_authorized"] is False
