"""Focused truth-free tests for the Phase61 C/N0 CMC audit."""

from __future__ import annotations

from decimal import Decimal
from pathlib import Path
import sys
import unittest


ROOT = Path(__file__).resolve().parents[1]
BENCHMARKS = ROOT / "apps" / "commands" / "benchmarks"
sys.path.insert(0, str(BENCHMARKS))

import gnss_smartphone_phase55_pixel5_adr_uncertainty_audit as phase55  # noqa: E402
import gnss_smartphone_phase61_pixel5_cn0_pseudorange_calibration as audit  # noqa: E402


def _row(index: int, *, pseudorange: float, adr: float, cn0: str = "40", svid: int = 1, signal: str = "GAL_E1") -> phase55.Row:
    return phase55.Row(
        row_number=index + 2,
        utc_ms=1000 + index * 1000,
        time_ns=index * 1_000_000_000,
        full_bias_ns=0,
        bias_ns=Decimal(0),
        offset_ns=Decimal(0),
        received_sv_time_ns=Decimal("10000000000"),
        rate_mps=Decimal(0),
        rate_uncertainty_mps=None,
        adr_state=phase55.ADR_VALID,
        adr_m=Decimal(str(adr)),
        adr_uncertainty_m=None,
        state=None,
        multipath=None,
        cn0=Decimal(cn0),
        hcdc=0,
        svid=svid,
        system="GALILEO",
        signal=signal,
        frequency_hz=Decimal("1575420000"),
        segment=0,
        pseudorange_m=pseudorange,
        code_masked=False,
        adr_masked=False,
    )


class Phase61Cn0PseudorangeAuditTest(unittest.TestCase):
    def test_freeze_is_sealed_before_raw_read(self) -> None:
        freeze = audit._verify_freeze()
        self.assertEqual(freeze["status"], "frozen-before-phase61-raw-read")
        self.assertEqual(freeze["fixed_observable"]["minimum_arc_length"], 2)
        self.assertFalse(freeze["pre_read_assertions"]["phase61_raw_device_gnss_opened"])
        self.assertEqual(freeze["input_policy"]["raw_device_gnss_read_count_total"], 4)

    def test_cmc_sign_and_singleton_arc_policy(self) -> None:
        rows = [_row(0, pseudorange=20_000_000.0, adr=19_999_990.0), _row(1, pseudorange=20_000_004.0, adr=19_999_993.0)]
        items, _events, info = audit._cmc_rows(rows, phase55)
        self.assertEqual(info["eligible_arc_min_rows"], 2)
        self.assertEqual(len(items), 2)
        self.assertAlmostEqual(items[0]["cmc_m"], 10.0, places=12)
        self.assertAlmostEqual(items[1]["cmc_m"], 11.0, places=12)
        self.assertAlmostEqual(items[0]["abs_centered_cmc_m"], 0.5, places=12)
        singleton, _events, singleton_info = audit._cmc_rows([_row(0, pseudorange=20_000_000.0, adr=19_999_990.0)], phase55)
        self.assertEqual(singleton, [])
        self.assertEqual(singleton_info["singleton_arc_rows_excluded"], 1)

    def test_source_shape_alpha_and_fixed_bins(self) -> None:
        self.assertAlmostEqual(audit._shape(40.0), 1.0, places=12)
        self.assertAlmostEqual(audit._shape(20.0), 10.0, places=12)
        items = [
            {"cn0_dbhz": 40.0, "abs_centered_cmc_m": 2.0},
            {"cn0_dbhz": 20.0, "abs_centered_cmc_m": 20.0},
        ]
        self.assertAlmostEqual(audit._fit_alpha(items), 2.0, places=12)
        self.assertEqual(audit._cn0_bin(20.0), "20_to_25")
        self.assertEqual(audit._cn0_bin(40.0), ">=40")

    def test_group_summaries_and_route_aggregate_do_not_collapse(self) -> None:
        items = [
            {"group": "A", "cn0_dbhz": 30.0, "abs_centered_cmc_m": 1.0},
            {"group": "A", "cn0_dbhz": 35.0, "abs_centered_cmc_m": 2.0},
            {"group": "B", "cn0_dbhz": 30.0, "abs_centered_cmc_m": 3.0},
        ]
        groups = audit._group_relation(items, lambda item: item["group"], 2)
        self.assertEqual(groups["A"]["count"], 2)
        self.assertEqual(groups["B"]["count"], 1)
        self.assertEqual(groups["A"]["abs_centered_cmc_m"]["count"], 2)
        self.assertEqual(groups["B"]["abs_centered_cmc_m"]["count"], 1)
        reports = {}
        for route, value in zip(audit.ROUTES, (1.0, 2.0, 3.0, 4.0)):
            report_items = [{"cn0_dbhz": 40.0, "abs_centered_cmc_m": value, "centered_cmc_m": value, "utcTimeMillis": 1, "hcdc": 0, "system": "GPS", "svid": 1, "signal": "GPS_L1CA"}]
            reports[route] = {
                "_items": report_items,
                "cmc_residual": {"finite_cn0_count": 1},
                "cn0_bins": {"ordered": {label: {"count": 1 if label == ">=40" else 0} for label in audit.CN0_BIN_LABELS}},
                "groups": {
                    "satellite": {"GPS:1": {"count": 1, "finite_cn0_count": 1, "abs_centered_cmc_m": {"count": 1}}},
                    "signal": {"GPS:GPS_L1CA": {"count": 1, "finite_cn0_count": 1, "abs_centered_cmc_m": {"count": 1}}},
                    "signal_group_count": 1,
                },
            }
        medians = {route: float(index + 1) for index, route in enumerate(audit.ROUTES)}
        aggregate = {"route_median_abs_centered_cmc_m": medians, "aggregate_median_m": audit._median(medians.values()), "aggregate_mad_m": audit._mad(medians.values())}
        integrity = audit._presentation_integrity(reports, aggregate, {"folds": [{"omitted_route": route} for route in audit.ROUTES]})
        self.assertTrue(integrity["four_route_medians_retained_in_order"])
        self.assertTrue(integrity["aggregate_recomputed_exact"])
        self.assertTrue(integrity["no_collapsed_route_aggregate"])

    def test_evaluator_has_no_solver_or_truth_payload_reader(self) -> None:
        source = Path(audit.__file__).read_text(encoding="utf-8")
        self.assertNotIn("subprocess", source)
        self.assertNotIn("truth_maps", source)
        self.assertNotIn("gnss_fgo", source)


if __name__ == "__main__":
    unittest.main()
