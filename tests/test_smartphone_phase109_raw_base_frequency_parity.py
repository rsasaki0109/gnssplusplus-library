"""Launch-free checks for the Phase109 raw-base structural contract."""

from __future__ import annotations

import importlib.util
import json
from pathlib import Path


ROOT = Path(__file__).resolve().parents[1]
MODULE_PATH = ROOT / "apps/commands/benchmarks/gnss_smartphone_phase109_raw_base_frequency_parity.py"
SPEC = importlib.util.spec_from_file_location("phase109_frequency_contract", MODULE_PATH)
assert SPEC is not None and SPEC.loader is not None
MODULE = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(MODULE)


def test_freeze_and_pre_raw_contract_are_unexecuted() -> None:
    freeze = MODULE.verify_freeze()
    pre_raw = MODULE.verify_pre_raw()
    assert freeze["phase"] == 109
    assert freeze["decision"]["candidate_count"] == 1
    assert freeze["decision"]["candidate_id"] == MODULE.CANDIDATE_ID
    assert pre_raw["raw_execution_authorized"] is False
    for key in (
        "raw_phone_gnss_reads", "raw_phone_imu_reads", "broadcast_navigation_reads",
        "raw_base_rinex_reads", "raw_base_hash_reads", "native_solver_invocations",
        "truth_reads", "mat_reads_or_generated", "phone_coordinate_reads",
        "precomputed_coordinate_reads", "pdc_reads", "accuracy_calculations",
    ):
        assert pre_raw[key] == 0


def test_manifest_has_exact_two_route_preserve_recipe() -> None:
    manifest = MODULE.verify_manifest()
    assert manifest["routes"] and len(manifest["routes"]) == 2
    assert [record["dataset_id"] for record in manifest["routes"]] == list(MODULE.ROUTES)
    assert manifest["candidate"]["selectors"] == [
        MODULE.PHASE93_SELECTOR, MODULE.VECTOR_SELECTOR, MODULE.QR_SELECTOR
    ]
    assert manifest["candidate"]["base_selectors"] == [
        "--native-base-pseudorange-compensation",
        "--native-base-pseudorange-source-miss-mask",
        MODULE.PRESERVE_SELECTOR,
    ]
    for record in manifest["routes"]:
        command = record["command"]
        assert command.count(MODULE.PRESERVE_SELECTOR) == 1
        assert command.count("--native-base-rinex") == 1
        assert command.count("--native-base-rinex-sha256") == 1
        assert record["runs"] == 1


def test_manifest_forbids_solution_truth_and_coordinate_lanes() -> None:
    manifest = MODULE.verify_manifest()
    assert manifest["candidate"]["truth_evaluation"] is False
    assert manifest["candidate"]["accuracy_scoring"] is False
    assert manifest["candidate"]["solution_output_publication"] is False
    assert manifest["raw_input_contract"]["truth_mat_precomputed_coordinate_pdc_kaggle_accuracy"] is False
    assert manifest["execution_authorization"]["raw_execution_authorized"] is False
    assert manifest["execution_authorization"]["solution_output_authorized"] is False


def test_source_has_signal_taxonomy_and_exactly_once_telemetry() -> None:
    app = MODULE.APP.read_text(encoding="utf-8")
    mask = MODULE.MISS_MASK.read_text(encoding="utf-8")
    for marker in (
        "phase101_exact_selector_recipe",
        "phase109_raw_base_frequency_parity_admission",
        "native_base_pseudorange_preserve_additional_frequency_bands",
        "source_miss_taxonomy_by_signal",
        "correction_application_pass_count",
        "correction_applied_exactly_once",
        "duplicate_correction_rejected",
    ):
        assert marker in app
    for marker in (
        "std::map<SignalType, SignalCounts> signal_counts;",
        "dropped_missing_exact_stream_rows",
        "dropped_out_of_domain_rows",
        "correction_already_applied",
        "native_base_pseudorange_correction_applied",
    ):
        assert marker in mask or marker in (MODULE.MISS_MASK_HEADER.read_text(encoding="utf-8"))


def test_phase109_reader_and_base_path_are_not_admitted_to_other_handoffs() -> None:
    source = MODULE.APP.read_text(encoding="utf-8")
    start = source.index(
        "if (options.native_source_clock_c0d_gnss_first_meter_state_handoff) {"
    )
    end = source.index("if (options.native_source_clock_c0d_epoch_vector_parity)", start)
    guard = source[start:end]
    assert "phase101_exact_selector_recipe" in guard
    assert "phase107_raw_base_source_parity_admission" in guard
    assert "phase109_raw_base_frequency_parity_admission" in guard
    assert "options.native_pdc_state_bridge" in guard
    assert "options.native_direct_doppler_wls_handoff" in guard
    assert "options.native_upstream_quality" in guard


def test_manifest_is_valid_json_and_raw_payloads_are_not_read_by_verifier() -> None:
    manifest = json.loads(MODULE.MANIFEST.read_text(encoding="utf-8"))
    assert manifest["read_accounting_before_authorization"]["raw_base_rinex_reads"] == 0
    assert manifest["read_accounting_before_authorization"]["raw_phone_gnss_reads"] == 0
    assert manifest["read_accounting_before_authorization"]["raw_phone_imu_reads"] == 0
    assert "input-member hash forbidden before authorization" in MODULE_PATH.read_text(encoding="utf-8")
    assert "hash_base" not in MODULE_PATH.read_text(encoding="utf-8")
