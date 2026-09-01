"""Truth/raw-input-free focused tests for the Phase55 ADR uncertainty audit."""

from __future__ import annotations

import csv
from decimal import Decimal
import io
from pathlib import Path
import sys
import unittest


ROOT = Path(__file__).resolve().parents[1]
BENCHMARKS = ROOT / "apps" / "commands" / "benchmarks"
sys.path.insert(0, str(BENCHMARKS))

import gnss_smartphone_phase55_pixel5_adr_uncertainty_audit as audit  # noqa: E402


def _row(
    index: int,
    *,
    utc_ms: int | None = None,
    time_ns: int | None = None,
    adr_m: str = "10",
    rate_mps: str = "2",
    uncertainty_m: str | None = "0.03",
    adr_state: int = audit.ADR_VALID,
    signal: str = "GAL_E1",
    svid: int = 1,
) -> audit.Row:
    return audit.Row(
        row_number=index + 2,
        utc_ms=1000 + index * 1000 if utc_ms is None else utc_ms,
        time_ns=index * 1_000_000_000 if time_ns is None else time_ns,
        full_bias_ns=0,
        bias_ns=Decimal(0),
        offset_ns=Decimal(0),
        received_sv_time_ns=Decimal(1_000_000),
        rate_mps=Decimal(rate_mps),
        rate_uncertainty_mps=None,
        adr_state=adr_state,
        adr_m=Decimal(adr_m),
        adr_uncertainty_m=Decimal(uncertainty_m) if uncertainty_m is not None else None,
        state=None,
        multipath=None,
        cn0=None,
        hcdc=0,
        svid=svid,
        system="GALILEO",
        signal=signal,
        frequency_hz=Decimal("1575420000"),
        segment=0,
        code_masked=False,
        adr_masked=False,
    )


class Phase55AdrUncertaintyAuditTest(unittest.TestCase):
    def test_freeze_is_sealed_before_raw_read_and_definitions_match(self) -> None:
        freeze = audit._verify_freeze()
        self.assertEqual(freeze["status"], "frozen-before-phase55-raw-read")
        self.assertFalse(freeze["pre_read_assertions"]["phase55_raw_reads_before_freeze"])
        self.assertEqual(
            freeze["numeric_gates"]["fixed_bins_m"]["definitions"],
            ["<=0.01", "0.01_to_0.1", "0.1_to_1", ">1"],
        )

    def test_source_field_is_parsed_when_present_without_imputation(self) -> None:
        columns = list(audit.REQUIRED_COLUMNS) + [
            "BiasNanos", "AccumulatedDeltaRangeUncertaintyMeters",
        ]
        values = {name: "" for name in columns}
        values.update({
            "MessageType": "Raw", "utcTimeMillis": "1000", "TimeNanos": "1000000000",
            "FullBiasNanos": "0", "BiasNanos": "0", "ReceivedSvTimeNanos": "1000000",
            "Svid": "1", "ConstellationType": "6", "SignalType": "GAL_E1",
            "CarrierFrequencyHz": "1575420000", "PseudorangeRateMetersPerSecond": "2",
            "AccumulatedDeltaRangeState": "1", "AccumulatedDeltaRangeMeters": "10",
            "AccumulatedDeltaRangeUncertaintyMeters": "0.03",
        })
        output = io.StringIO(newline="")
        writer = csv.DictWriter(output, fieldnames=columns)
        writer.writeheader()
        writer.writerow(values)
        rows, metadata = audit._parse_payload(output.getvalue().encode())
        self.assertTrue(metadata["adr_uncertainty_field_present"])
        self.assertEqual(rows[0].adr_uncertainty_m, Decimal("0.03"))

    def test_pair_sigma_and_closure_sign_are_source_exact(self) -> None:
        rows = [_row(0, adr_m="10"), _row(1, adr_m="12.5"), _row(2, adr_m="14.5")]
        pairs = audit._pair_items(rows)
        self.assertEqual(len(pairs), 2)
        self.assertTrue(all(item["uncertainty_eligible"] for item in pairs))
        self.assertAlmostEqual(pairs[0]["u_pair_m"], 0.03 * 2**0.5, places=12)
        # +2.5 m ADR increment - +2.0 m trapezoidal rate integral = +0.5 m.
        self.assertAlmostEqual(pairs[0]["residual_m"], 0.5, places=12)
        self.assertEqual(pairs[0]["centered_residual_m"], 0.0)
        self.assertEqual(audit._fixed_bin(0.01), "<=0.01m")
        self.assertEqual(audit._fixed_bin(0.1), "0.01_to_0.1m")
        self.assertEqual(audit._fixed_bin(1.0), "0.1_to_1m")

    def test_missing_uncertainty_stays_ordinary_but_is_excluded(self) -> None:
        pairs = audit._pair_items([_row(0), _row(1, uncertainty_m=None)])
        self.assertEqual(pairs[0]["reason"], "uncertainty_missing")
        self.assertTrue(pairs[0]["ordinary"])
        self.assertFalse(pairs[0]["uncertainty_eligible"])
        self.assertIsNone(pairs[0]["u_pair_m"])

    def test_reset_invalid_and_gap_boundaries_are_not_ordinary(self) -> None:
        reset = audit._pair_items([_row(0), _row(1, adr_state=audit.ADR_VALID | audit.ADR_RESET)])
        invalid = audit._pair_items([_row(0), _row(1, adr_state=0)])
        gap = audit._pair_items([_row(0), _row(1, time_ns=2_000_000_000)])
        self.assertEqual(reset[0]["reason"], "adr_reset")
        self.assertEqual(invalid[0]["reason"], "invalid_adr_state")
        self.assertEqual(gap[0]["reason"], "gap")
        self.assertFalse(reset[0]["ordinary"])
        self.assertFalse(invalid[0]["ordinary"])
        self.assertFalse(gap[0]["ordinary"])

    def test_presentation_integrity_rejects_stale_groups_and_collapsed_medians(self) -> None:
        reports: dict[str, dict[str, object]] = {}
        for index, route in enumerate(audit.ROUTES):
            reports[route] = {
                "rows": {"pair_count": 2, "ordinary_pair_count": 2, "uncertainty_pair_count": 2},
                "pair_reasons": {"ordinary_uncertainty": 2},
                "groups": {
                    "state": {"valid_unresolved": {"count": 2}},
                    "signal_frequency": {"GALILEO:GAL_E1:1575420000Hz": {"count": 2}},
                    "satellite": {"GALILEO:1": {"count": 2}},
                },
                "uncertainty": {
                    "fixed_bins": {
                        "<=0.01m": {"count": 2}, "0.01_to_0.1m": {"count": 0},
                        "0.1_to_1m": {"count": 0}, ">1m": {"count": 0},
                    },
                    "quantiles": {"q1": {"count": 1}, "q2": {"count": 1}, "q3": {"count": 0}, "q4": {"count": 0}},
                },
                "headers": {"columns": ["MessageType"], "optional_field_presence": {}},
            }
        medians = {route: float(index + 1) for index, route in enumerate(audit.ROUTES)}
        aggregate = {
            "route_median_abs_uncertainty_residual_m": medians,
            "route_median_aggregate_m": audit._median(medians.values()),
            "route_median_mad_m": audit._mad(medians.values()),
        }
        loo = {"folds": [{"omitted_route": route} for route in audit.ROUTES]}
        good = audit._presentation_integrity(reports, aggregate, loo, 0)
        self.assertTrue(good["aggregate_recomputed_exact"])
        self.assertTrue(good["four_route_medians_retained"])
        reports[audit.ROUTES[0]]["groups"]["state"]["valid_unresolved"]["count"] = 1  # type: ignore[index]
        bad = audit._presentation_integrity(reports, aggregate, loo, 0)
        self.assertFalse(bad["state_group_counts_sum_all"])
        medians.pop(audit.ROUTES[-1])
        aggregate["route_median_abs_uncertainty_residual_m"] = medians
        bad = audit._presentation_integrity(reports, aggregate, loo, 0)
        self.assertFalse(bad["four_route_medians_retained"])

    def test_evaluator_has_no_solver_or_forbidden_truth_input(self) -> None:
        source = Path(audit.__file__).read_text(encoding="utf-8")
        self.assertNotIn("subprocess", source)
        self.assertNotIn("truth_maps", source)
        self.assertNotIn("gnss_fgo", source)
        self.assertNotIn("open_truth", source)


if __name__ == "__main__":
    unittest.main()
