"""Focused truth-free tests for the Phase58 C/N0 audit contract."""

from __future__ import annotations

from decimal import Decimal
from pathlib import Path
import sys
import unittest


ROOT = Path(__file__).resolve().parents[1]
BENCHMARKS = ROOT / "apps" / "commands" / "benchmarks"
sys.path.insert(0, str(BENCHMARKS))

import gnss_smartphone_phase48_pixel5_raw_code_rate_uncertainty_audit as phase48  # noqa: E402
import gnss_smartphone_phase58_pixel5_cn0_doppler_calibration as audit  # noqa: E402


def _row(index: int, *, utc_ms: int, time_ns: int, pseudorange_m: float, cn0: str = "40") -> phase48.Row:
    return phase48.Row(
        row_number=index + 2,
        utc_ms=utc_ms,
        time_ns=time_ns,
        full_bias_ns=0,
        bias_ns=Decimal(0),
        offset_ns=Decimal(0),
        received_sv_time_ns=Decimal("10000000000"),
        rate_mps=Decimal("2"),
        rate_uncertainty_mps=None,
        sv_uncertainty_ns=None,
        state=0,
        multipath=None,
        cn0=Decimal(cn0),
        hcdc=0,
        svid=1,
        system="GALILEO",
        signal="GAL_E1",
        frequency_hz=Decimal("1575420000"),
        segment=0,
        pseudorange_m=pseudorange_m,
        code_masked=False,
    )


class Phase58Cn0AuditTest(unittest.TestCase):
    def test_freeze_is_sealed_before_raw_read(self) -> None:
        freeze = audit._verify_freeze()
        self.assertEqual(freeze["status"], "frozen-before-phase58-raw-read")
        self.assertEqual(freeze["input_policy"]["raw_device_gnss_read_count_per_route"], 1)
        self.assertFalse(freeze["pre_read_assertions"]["raw_payload_opened"])
        self.assertEqual(freeze["numeric_gates"]["loo_calibration"]["folds"], 4)

    def test_source_shape_and_candidate_floor(self) -> None:
        self.assertAlmostEqual(audit._shape(40.0), 1.0, places=12)
        self.assertAlmostEqual(audit._shape(20.0), 10.0, places=12)
        self.assertLess(audit._shape(50.0), audit._shape(40.0))
        self.assertAlmostEqual(audit._candidate_sigma(40.0, 0.1), 0.2, places=12)
        self.assertAlmostEqual(audit._candidate_sigma(30.0, 0.1), 0.316227766016838, places=12)
        self.assertIsNone(audit._candidate_sigma(None, 1.0))

    def test_closure_sign_and_pair_cn0_are_endpoint_mean(self) -> None:
        epochs = [
            phase48.Epoch(key_ms=1000, rows=[_row(0, utc_ms=1000, time_ns=0, pseudorange_m=20.0, cn0="30")]),
            phase48.Epoch(key_ms=2000, rows=[_row(1, utc_ms=2000, time_ns=1_000_000_000, pseudorange_m=22.5, cn0="50")]),
        ]
        transitions, _ = audit._transition_items(epochs, phase48)
        self.assertEqual(len(transitions), 1)
        self.assertAlmostEqual(transitions[0]["raw_residual_m"], 0.5, places=12)
        self.assertAlmostEqual(transitions[0]["pair_cn0_dbhz"], 40.0, places=12)
        self.assertAlmostEqual(transitions[0]["abs_centered_rate_residual_mps"], 0.0, places=12)

    def test_linear_percentile_matches_source_contract(self) -> None:
        self.assertAlmostEqual(audit._linear_percentile([10.0, 20.0], 85.0), 20.0, places=12)
        self.assertAlmostEqual(audit._linear_percentile([0.0, 10.0, 20.0, 30.0], 50.0), 15.0, places=12)

    def test_scale_fit_is_median_and_loo_calibration_is_explicit(self) -> None:
        items = [
            {"pair_cn0_dbhz": 40.0, "cn0_dbhz": 40.0, "abs_centered_rate_residual_mps": 2.0, "dt_s": 1.0},
            {"pair_cn0_dbhz": 30.0, "cn0_dbhz": 30.0, "abs_centered_rate_residual_mps": 20.0, "dt_s": 1.0},
            {"pair_cn0_dbhz": 50.0, "cn0_dbhz": 50.0, "abs_centered_rate_residual_mps": 0.2, "dt_s": 1.0},
        ]
        self.assertAlmostEqual(audit._fit_alpha(items), 2.0, places=12)
        calibration = audit._calibration(items, 2.0)
        self.assertEqual(calibration["candidate_above_fixed_sigma_count"], 3)
        self.assertGreater(calibration["candidate_sigma_excess_p95_mps"], 0.0)

    def test_bins_and_presentation_reject_collapsed_group_accounting(self) -> None:
        self.assertEqual(audit._cn0_bin(20.0), "20_to_25")
        self.assertEqual(audit._cn0_bin(25.0), "25_to_30")
        self.assertEqual(audit._cn0_bin(40.0), ">=40")
        report = {
            "rows": {"finite_pair_cn0_count": 2},
            "cn0_bins": {"ordered": {label: {"count": 0} for label in audit.CN0_BIN_LABELS}},
        }
        aggregate = {
            "route_median_abs_rate_residual_mps": {route: 1.0 for route in audit.ROUTES},
            "aggregate_median_mps": 1.0,
            "aggregate_mad_mps": 0.0,
        }
        loo = {"folds": [{"omitted_route": route} for route in audit.ROUTES]}
        integrity = audit._presentation_integrity({route: report for route in audit.ROUTES}, aggregate, loo)
        self.assertFalse(integrity["all_group_counts_sum"])

    def test_static_source_contract_and_no_solver_reader(self) -> None:
        freeze = audit._verify_freeze()
        source = audit._source_contract(freeze)
        self.assertTrue(source["adapter"]["parses_cn0_dbhz"])
        self.assertTrue(source["fgo"]["uses_cn0_snr"])
        self.assertTrue(source["upstream_contract"]["doppler_sigma_divide_12"])
        self.assertTrue(source["config"]["upstream_quality_default_false"])
        self.assertTrue(source["config"]["fixed_sigma_default_0_2"])
        self.assertTrue(source["fgo"]["candidate_cn0_calibration_absent"])
        text = Path(audit.__file__).read_text(encoding="utf-8")
        self.assertNotIn("subprocess", text)
        self.assertNotIn("truth_maps", text)
        self.assertNotIn("nav.calculate", text)


if __name__ == "__main__":
    unittest.main()
