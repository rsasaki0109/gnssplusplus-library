"""Focused, truth-free contract tests for the Phase57 raw audit."""

from __future__ import annotations

from decimal import Decimal
from pathlib import Path
import sys
import unittest


ROOT = Path(__file__).resolve().parents[1]
BENCHMARKS = ROOT / "apps" / "commands" / "benchmarks"
sys.path.insert(0, str(BENCHMARKS))

import gnss_smartphone_phase48_pixel5_raw_code_rate_uncertainty_audit as phase48  # noqa: E402
import gnss_smartphone_phase57_pixel5_rate_uncertainty_audit as audit  # noqa: E402


def _row(
    index: int,
    *,
    utc_ms: int,
    time_ns: int,
    pseudorange_m: float,
    rate_mps: str = "2",
    rate_uncertainty_mps: str | None = "0.4",
    signal: str = "GAL_E1",
    svid: int = 1,
) -> phase48.Row:
    row = phase48.Row(
        row_number=index + 2,
        utc_ms=utc_ms,
        time_ns=time_ns,
        full_bias_ns=0,
        bias_ns=Decimal(0),
        offset_ns=Decimal(0),
        received_sv_time_ns=Decimal("10000000000"),
        rate_mps=Decimal(rate_mps),
        rate_uncertainty_mps=(Decimal(rate_uncertainty_mps) if rate_uncertainty_mps is not None else None),
        sv_uncertainty_ns=None,
        state=0,
        multipath=None,
        cn0=Decimal("40"),
        hcdc=0,
        svid=svid,
        system="GALILEO",
        signal=signal,
        frequency_hz=Decimal("1575420000"),
        segment=0,
        pseudorange_m=pseudorange_m,
        code_masked=False,
    )
    return row


class Phase57RateUncertaintyAuditTest(unittest.TestCase):
    def test_freeze_is_sealed_before_raw_read(self) -> None:
        freeze = audit._verify_freeze()
        self.assertEqual(freeze["status"], "frozen-before-phase57-raw-read")
        self.assertEqual(freeze["input_policy"]["raw_device_gnss_read_count_per_route"], 1)
        self.assertFalse(freeze["pre_read_assertions"]["raw_payload_opened"])
        self.assertEqual(
            freeze["numeric_gates"]["current_fgo_adoption"]["min_fixed_lower_bound_affected_fraction_each_route"],
            0.1,
        )

    def test_phase25_rate_closure_sign_and_pair_sigma(self) -> None:
        epochs = [
            phase48.Epoch(key_ms=1000, rows=[_row(0, utc_ms=1000, time_ns=0, pseudorange_m=20.0)]),
            phase48.Epoch(key_ms=2000, rows=[_row(1, utc_ms=2000, time_ns=1_000_000_000, pseudorange_m=22.5)]),
        ]
        transitions, _ = audit._transition_items(epochs)
        self.assertEqual(len(transitions), 1)
        self.assertAlmostEqual(transitions[0]["raw_residual_m"], 0.5, places=12)
        self.assertAlmostEqual(transitions[0]["pair_uncertainty_mps"], Decimal("0.4").__float__() * 2**0.5, places=12)
        self.assertAlmostEqual(transitions[0]["pair_integrated_sigma_m"], 0.2 * 2**0.5, places=12)
        self.assertEqual(transitions[0]["abs_centered_residual_m"], 0.0)

    def test_segment_boundary_is_excluded_and_bins_are_fixed(self) -> None:
        first = _row(0, utc_ms=1000, time_ns=0, pseudorange_m=20.0)
        second = _row(1, utc_ms=2000, time_ns=1_000_000_000, pseudorange_m=22.0)
        second.segment = 1
        transitions, _ = audit._transition_items([
            phase48.Epoch(key_ms=1000, rows=[first]),
            phase48.Epoch(key_ms=2000, rows=[second]),
        ])
        self.assertEqual(transitions, [])
        self.assertEqual(audit._fixed_bin(0.1), "<=0.1mps")
        self.assertEqual(audit._fixed_bin(0.5), "0.1_to_0.5mps")
        self.assertEqual(audit._fixed_bin(2.0), "0.5_to_2mps")
        self.assertEqual(audit._fixed_bin(2.1), ">2mps")

    def test_source_contract_covers_fixed_and_snr_sigma_paths(self) -> None:
        freeze = audit._verify_freeze()
        source = audit._source_contract(freeze)
        self.assertTrue(source["adapter"]["parses_pseudorange_rate_mps"])
        self.assertFalse(source["adapter"]["parses_pseudorange_rate_uncertainty_mps"])
        self.assertFalse(source["observation"]["retains_candidate_uncertainty_mps"])
        self.assertFalse(source["fgo"]["consumes_candidate_uncertainty"])
        self.assertTrue(source["fgo"]["fixed_undifferenced_sigma_path"])
        self.assertTrue(source["fgo"]["fixed_single_difference_sigma_path"])
        self.assertTrue(source["fgo"]["snr_derived_sigma_path"])
        self.assertTrue(source["fgo"]["upstream_quality_default_false"])
        self.assertTrue(source["upstream_contract"]["doppler_sigma_divide_12"])

    def test_snr_sigma_is_source_exact_and_impact_is_not_field_presence(self) -> None:
        sigma = audit._snr_doppler_sigma("GAL_E1", 40.0, {"L1": 40.0, "L5": 40.0})
        self.assertAlmostEqual(sigma, 1.0 / 12.0, places=12)
        self.assertIsNone(audit._snr_doppler_sigma("UNKNOWN", 40.0, {"L1": 40.0, "L5": 40.0}))
        items = [
            {"signal": "GAL_E1", "cn0": 40.0, "rate_uncertainty_mps": 0.05},
            {"signal": "GAL_E1", "cn0": 40.0, "rate_uncertainty_mps": 0.3},
        ]
        impact = audit._factor_impact(items, {"L1": 40.0, "L5": 40.0})
        self.assertEqual(impact["raw_eligible_factor_proxy_count"], 2)
        self.assertAlmostEqual(impact["fixed_lower_bound_affected_fraction"], 0.5)
        self.assertAlmostEqual(impact["snr_enabled_affected_fraction"], 0.5)
        self.assertGreater(impact["fixed_lower_bound_sigma_inflation_proxy_1s_m"]["p95"], 0.0)

    def test_evaluator_has_no_solver_or_truth_payload_reader(self) -> None:
        source = Path(audit.__file__).read_text(encoding="utf-8")
        self.assertNotIn("subprocess", source)
        self.assertNotIn("truth_maps", source)
        self.assertNotIn("nav.calculate", source)
        self.assertNotIn("gnss_fgo --", source)


if __name__ == "__main__":
    unittest.main()
