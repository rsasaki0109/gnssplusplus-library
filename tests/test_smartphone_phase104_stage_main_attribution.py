"""Focused, truth-free tests for the Phase104 attribution contract."""

from __future__ import annotations

import math
import sys
from pathlib import Path


ROOT = Path(__file__).resolve().parents[1]
MODULE_DIR = ROOT / "apps/commands/benchmarks"
if str(MODULE_DIR) not in sys.path:
    sys.path.insert(0, str(MODULE_DIR))

import gnss_smartphone_phase104_stage_main_attribution as contract


def test_freeze_and_single_candidate_contract() -> None:
    freeze = contract.verify_freeze()
    candidate = freeze["candidate"]
    assert candidate["candidate_count"] == 1
    assert candidate["selector_default_off"] is True
    assert candidate["copy_only"] is True
    assert candidate["stage_copy_may_feed_solver"] is False
    assert candidate["no_fallback_or_rerun"] is True
    assert candidate["routes"] == list(contract.ROUTES)


def test_manifest_is_raw_placeholder_only_and_exact_two_route_matrix() -> None:
    manifest = contract.verify_manifest()
    assert manifest["candidate"]["selectors"][-1] == contract.PHASE104_SELECTOR
    assert manifest["candidate"]["stage_sidecar_reinput"] is False
    assert manifest["matrix"]["native_invocations_planned"] == 2
    assert manifest["matrix"]["truth_reads_planned"] == 0
    assert manifest["matrix"]["reruns"] == 0
    for record in manifest["routes"]:
        for name, placeholder in (
            ("device_gnss.csv", "__PHASE95_RAW_DEVICE_GNSS__"),
            ("device_imu.csv", "__PHASE95_RAW_DEVICE_IMU__"),
            ("brdc.nav", "__PHASE95_RAW_BROADCAST_NAV__"),
        ):
            assert record["raw_inputs"][name]["path"] == placeholder
            assert record["raw_inputs"][name]["sha256"] is None
            assert record["raw_inputs"][name]["read_at_manifest_creation"] is False


def test_pre_raw_verification_has_no_execution_or_truth_reads() -> None:
    result = contract.verify_pre_raw()
    assert result["status"] == "pre-raw-verified"
    assert result["raw_reads"] == 0
    assert result["native_solver_invocations"] == 0
    assert result["truth_reads"] == 0
    assert result["accuracy_calculations"] == 0
    assert result["solution_output_published"] is False


def test_implementation_pins_and_copy_boundary() -> None:
    details = contract.verify_implementation()
    assert details["implementation_commit"] == contract.IMPLEMENTATION_COMMIT
    source = contract.APP.read_text(encoding="utf-8")
    handoff = source.index("problem.native_source_clock_c0d_gnss_first_d_handoff_mps =")
    observer = source.index(
        "if (options.native_phase104_stage_main_accuracy_attribution)", handoff
    )
    main_input = source.index("bool use_imu = buildImuInput")
    assert handoff < observer < main_input
    assert "native_phase104_stage_main_accuracy_attribution = false" in source


def test_ecef_conversion_is_finite_and_earth_valid() -> None:
    lat, lon = contract._ecef_to_lat_lon(contract.WGS84_A_M, 0.0, 0.0)
    assert math.isfinite(lat) and math.isfinite(lon)
    assert abs(lat) < 1.0e-10
    assert abs(lon) < 1.0e-10


def test_truth_free_metric_synthetic_exact_keys() -> None:
    route = "synthetic/pixel5"
    keys = [(route, 1000), (route, 2000), (route, 3000)]
    prediction = {
        "keys": keys,
        "coordinates": [(37.0, -122.0), (37.0, -122.0), (37.0, -122.0)],
    }
    truth = {key: coordinate for key, coordinate in zip(keys, prediction["coordinates"])}
    scored = contract._score_prediction(prediction, truth, route, [])
    assert scored["prediction_rows"] == 3
    assert scored["matched_rows"] == 3
    assert scored["missing_truth_rows"] == 0
    assert scored["finite"] is True
    assert scored["score_m"] == 0.0
    assert scored["over_70_mps_count"] == 0
