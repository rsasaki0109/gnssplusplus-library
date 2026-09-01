"""Structural, truth-free contracts for the Phase42 row-attrition audit."""

import json
from pathlib import Path


ROOT = Path(__file__).resolve().parents[1]


def _read(relative: str) -> str:
    return (ROOT / relative).read_text(encoding="utf-8")


def test_phase42_freeze_is_raw_only_and_has_fixed_routes():
    freeze_path = (
        ROOT
        / "docs/use_cases/records/"
        "smartphone_r5_phase42_doppler_row_attrition_freeze_v1.json"
    )
    freeze = json.loads(freeze_path.read_text(encoding="utf-8"))
    assert freeze["phase"] == 42
    assert freeze["status"] == "frozen-before-implementation-and-audit"
    assert freeze["authority"]["base_commit"] == "1f187c3"
    assert freeze["authority"]["phase41_freeze_is_immutable"] is True
    assert len(freeze["objective"]["structural_routes"]) == 6
    assert freeze["objective"]["route_order_is_fixed"] is True
    assert freeze["input_contract"]["raw_only"] is True
    assert freeze["truth_accounting"]["allowed_truth_reads"] == 0
    assert freeze["truth_accounting"]["mat_inputs_or_outputs"] == 0
    assert freeze["truth_accounting"]["precomputed_coordinates_used"] == 0
    assert "ground_truth.csv" in freeze["input_contract"]["forbidden_inputs"]
    assert "device_imu.csv" in freeze["input_contract"]["forbidden_inputs"]


def test_phase42_audit_keeps_exclusive_stages_and_fgo_identity():
    audit = _read("apps/native/gnss_doppler_row_attrition_audit.cpp")
    cmake = _read("apps/CMakeLists.txt")

    assert "gnss_doppler_row_attrition_audit" in cmake
    assert "raw_row_diagnostics" in audit
    assert "first_reason" in audit
    assert "nav_state_call1" in audit
    assert "nav_state_call2" in audit
    assert "ephemeris_present" in audit
    assert "ephemeris_healthy" in audit
    assert "clock_jump" in audit
    assert "includes_receiver_clock_drift" in audit
    assert "problem.undifferenced_doppler_factors.size()" in audit
    assert "first_reason_is_exclusive" in audit
    assert "verify_enriched_pseudorange = false" in audit
    assert "device_wls_coordinates_used" in audit
    assert "truth_used" in audit
    assert "imu_open_count" in audit
    assert "ground_truth" in audit
    assert "submission.csv" in audit


def test_phase42_loader_retains_raw_quality_and_key_provenance():
    observation = _read("include/libgnss++/core/observation.hpp")
    loader_header = _read("include/libgnss++/io/android_raw_gnss.hpp")
    loader = _read("src/io/android_raw_gnss.cpp")

    assert "raw_row_index" in observation
    assert "raw_snr_masked" in observation
    assert "raw_multipath_masked" in observation
    assert "AndroidRawGnssRowDiagnostic" in loader_header
    assert "raw_row_diagnostics" in loader_header
    assert "raw_diagnostic.raw_row_index" in loader
    assert "raw_diagnostic.snr_masked" in loader
    assert "raw_diagnostic.multipath_masked" in loader
    assert "raw_diagnostic.selected_epoch_index" in loader
    assert "publishRawDiagnostic" in loader


def test_phase42_no_candidate_or_threshold_mutation():
    audit = _read("apps/native/gnss_doppler_row_attrition_audit.cpp")
    assert "concrete_generalizable_bug_proven" in audit
    assert "correction_applied" in audit
    assert "threshold_or_sign_change" in audit
    assert "use_corrected_undifferenced_doppler_factors = true" in audit
    assert "min_elevation_deg = 0.0" in audit
    assert "min_snr_dbhz = 0.0" in audit
