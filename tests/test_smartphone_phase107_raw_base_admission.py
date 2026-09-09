"""Launch-free checks for the Phase107 raw-base admission boundary.

These tests inspect only the pinned freeze, native source text, and headers.
They never open, hash, or execute raw GNSS/IMU/navigation/base files, truth,
MAT, solver, or Kaggle artifacts.
"""

from __future__ import annotations

import json
from pathlib import Path


ROOT = Path(__file__).resolve().parents[1]
APP = ROOT / "apps/native/gnss_fgo_imu_no_base.cpp"
CONFIG = ROOT / "include/libgnss++/algorithms/fgo_config.hpp"
FGO = ROOT / "include/libgnss++/algorithms/fgo.hpp"
FREEZE = ROOT / (
    "docs/use_cases/records/"
    "smartphone_r5_phase107_raw_base_source_parity_freeze_v1.json"
)


def _source() -> str:
    return APP.read_text(encoding="utf-8")


def _phase101_handoff_guard() -> str:
    source = _source()
    start = source.index(
        "if (options.native_source_clock_c0d_gnss_first_meter_state_handoff) {"
    )
    end = source.index(
        "if (options.fgo_imu_sparse_recovery", start
    )
    return source[start:end]


def test_freeze_keeps_phase107_as_one_unexecuted_opt_in_candidate() -> None:
    freeze = json.loads(FREEZE.read_text(encoding="utf-8"))
    assert freeze["phase"] == 107
    assert freeze["decision"]["candidate_count"] == 1
    assert freeze["decision"]["candidate_id"] == (
        "phase107-raw-base-rinex-source-exact-pseudorange-phase101-c7d-qr-v1"
    )
    assert freeze["decision"]["current_phase101_plus_base_combination_executable"] is False
    assert freeze["authorization_boundary"]["raw_base_execution_authorized"] is False
    assert freeze["read_accounting_phase107"]["raw_base_rinex_bytes"] == 0


def test_exact_phase107_raw_base_admission_is_narrow() -> None:
    guard = _phase101_handoff_guard()
    assert "const bool has_any_base_input =" in guard
    assert "const bool phase107_raw_base_source_parity_admission =" in guard
    assert "const bool phase109_raw_base_frequency_parity_admission =" in guard
    assert "const bool phase101_raw_base_source_parity_admission =" in guard
    assert "options.native_base_pseudorange_compensation &&" in guard
    assert "options.native_base_pseudorange_source_miss_mask &&" in guard
    assert "!options.native_base_pseudorange_preserve_additional_frequency_bands &&" in guard
    assert "options.native_base_pseudorange_preserve_additional_frequency_bands &&" in guard
    assert "!options.native_base_rinex_path.empty() &&" in guard
    assert "!options.native_base_rinex_sha256.empty();" in guard
    assert "phase107_raw_base_source_parity_admission ||" in guard
    assert "if (has_any_base_input && !phase101_raw_base_source_parity_admission)" in guard
    assert "forbids base/external coordinate inputs" not in guard


def test_incompatible_mat_and_coordinate_guards_remain_fail_closed() -> None:
    source = _source()
    assert "hasMatExtension(*path)" in source
    assert "MATLAB .mat paths are forbidden by the native/raw contract" in source
    assert "const auto hasForbiddenPhase104PathTerm" in source
    for term in (".mat", "truth", "base", "pdc", "precomputed", "coordinate", "kaggle"):
        assert f'"{term}"' in source
    # The Phase101 admission still rejects all non-raw handoff alternatives.
    guard = _phase101_handoff_guard()
    for term in (
        "options.native_source_clock_c0d_gnss_first_raw_drift_d_initializer",
        "options.native_gnss_first_velocity_only_handoff",
        "options.native_direct_doppler_wls_handoff",
        "options.native_pdc_state_bridge",
        "options.native_upstream_quality",
    ):
        assert term in guard


def test_base_correction_is_applied_once_to_shared_problem() -> None:
    source = _source()
    assert source.count(
        "libgnss::base_pseudorange_compensation::subtractCorrection("
    ) == 1
    assert "gnss_first_problem = problem" in source
    assert "correction is applied once to the shared problem" in source
    assert "base_pseudorange_source_miss_mask" in source


def test_selector_off_and_legacy_config_remain_default_off() -> None:
    config = CONFIG.read_text(encoding="utf-8")
    marker = "use_native_source_clock_c0d_gnss_first_meter_state_handoff"
    window = config[config.index(marker) : config.index(marker) + 120]
    assert "= false;" in window
    assert "use_native_source_clock_c0d_epoch_vector_parity = false;" in config
    assert (
        "use_native_source_clock_c0d_phase99_main_multifrontal_qr_solver =\n"
        "            false;"
    ) in config
    # No result field or configuration unit was added by the guard-only patch.
    assert "epoch_clock_drift_mps" in FGO.read_text(encoding="utf-8")


def test_admission_reuses_existing_same_run_base_rinex_path() -> None:
    source = _source()
    assert "base_reader.open(options.native_base_rinex_path)" in source
    assert "base_reader.readAllObservations(base_series)" in source
    assert "base_config.base_position_ecef = base_header.approximate_position;" in source
    assert "base_pseudorange_model.build(base_series, nav, base_config)" in source
    assert "problem = processor.buildPseudorangeProblem(epochs, nav);" in source
