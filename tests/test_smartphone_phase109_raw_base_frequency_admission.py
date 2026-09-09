"""Launch-free Phase109 additional-frequency-band admission checks.

These checks inspect source and the sealed freeze only.  They never open or
hash raw GNSS/IMU/navigation/base/truth payloads and never launch a solver.
"""

from __future__ import annotations

import json
from pathlib import Path


ROOT = Path(__file__).resolve().parents[1]
APP = ROOT / "apps/native/gnss_fgo_imu_no_base.cpp"
MISS_MASK = ROOT / "src/algorithms/source_pseudorange_miss_mask.cpp"
MISS_MASK_HEADER = ROOT / "include/libgnss++/algorithms/source_pseudorange_miss_mask.hpp"
FGO_HEADER = ROOT / "include/libgnss++/algorithms/fgo.hpp"
FREEZE = ROOT / (
    "docs/use_cases/records/"
    "smartphone_r5_phase109_raw_base_miss_frequency_parity_freeze_v1.json"
)


def test_phase109_freeze_is_one_unexecuted_candidate() -> None:
    freeze = json.loads(FREEZE.read_text(encoding="utf-8"))
    assert freeze["phase"] == 109
    assert freeze["decision"]["candidate_count"] == 1
    assert freeze["decision"]["candidate_id"] == (
        "phase109-phase101-raw-base-existing-additional-frequency-band-guard-v1"
    )
    assert freeze["decision"]["raw_execution_authorized_at_freeze"] is False
    assert freeze["authorization_boundary"]["phase109_raw_base_execution"] is False
    assert freeze["read_accounting_phase109"]["raw_base_rinex_bytes"] == 0


def test_phase109_admission_requires_exact_phase101_recipe() -> None:
    source = APP.read_text(encoding="utf-8")
    assert "phase109_raw_base_frequency_parity_admission" in source
    assert "phase101_raw_base_source_parity_admission" in source
    assert "phase101_exact_selector_recipe" in source
    assert "options.native_source_clock_c0d_epoch_vector_parity &&" in source
    assert "options.native_source_clock_c0d_phase99_main_multifrontal_qr_solver;" in source
    assert "options.native_base_pseudorange_preserve_additional_frequency_bands &&" in source
    assert "options.native_base_pseudorange_source_miss_mask &&" in source
    assert "!options.native_base_rinex_path.empty() &&" in source
    assert "!options.native_base_rinex_sha256.empty();" in source
    assert "requires the Phase101 meter-state handoff" in source
    assert "cannot be combined with --native-base-pseudorange-preserve" not in source


def test_phase109_keeps_reader_default_off_and_shared_single_pass_contract() -> None:
    source = APP.read_text(encoding="utf-8")
    rinex_header = (ROOT / "include/libgnss++/io/rinex.hpp").read_text(encoding="utf-8")
    assert "bool preserve_additional_frequency_bands_ = false;" in rinex_header
    assert "base_reader.setPreserveAdditionalFrequencyBands(" in source
    assert "base_pseudorange_model.build(base_series, nav, base_config)" in source
    assert "source_model_build_count" in source
    assert "correction_application_pass_count" in source
    assert "correction_applied_exactly_once" in source
    assert "duplicate_correction_rejected" in source
    assert source.count(
        "libgnss::base_pseudorange_compensation::subtractCorrection("
    ) == 1


def test_phase109_signal_taxonomy_and_double_apply_guard_are_source_exact() -> None:
    mask_header = MISS_MASK_HEADER.read_text(encoding="utf-8")
    mask_source = MISS_MASK.read_text(encoding="utf-8")
    fgo_header = FGO_HEADER.read_text(encoding="utf-8")
    for field in (
        "retained_finite_pc_rows",
        "corrected_rows",
        "dropped_missing_exact_stream_rows",
        "dropped_out_of_domain_rows",
        "factor_count_consistent",
    ):
        assert field in mask_header
        assert field in mask_source
    assert "std::map<SignalType, SignalCounts> signal_counts;" in mask_header
    assert "correction_already_applied" in mask_header
    assert "native_base_pseudorange_correction_applied" in fgo_header
    assert "source miss-mask correction was already applied" in mask_source
    assert "signal_count_consistent" in mask_source


def test_phase109_does_not_admit_pdc_or_alternate_handoff_lanes() -> None:
    source = APP.read_text(encoding="utf-8")
    start = source.index(
        "if (options.native_source_clock_c0d_gnss_first_meter_state_handoff) {"
    )
    end = source.index("if (options.fgo_imu_sparse_recovery", start)
    guard = source[start:end]
    for term in (
        "options.native_pdc_state_bridge",
        "options.native_upstream_quality",
        "options.native_gnss_first_velocity_only_handoff",
        "options.native_direct_doppler_wls_handoff",
    ):
        assert term in guard
