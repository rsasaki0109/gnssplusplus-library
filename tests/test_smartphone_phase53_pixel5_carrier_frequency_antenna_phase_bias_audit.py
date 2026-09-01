"""Focused truth-free contract tests for the Phase53 raw audit."""

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

import gnss_smartphone_phase53_pixel5_carrier_frequency_antenna_phase_bias_audit as audit  # noqa: E402


def _row(
    row_number: int,
    utc_ms: int,
    time_ns: int,
    svid: int,
    adr_m: float,
    *,
    rate_mps: float = 10.0,
    frequency_hz: float = 1_575_420_000.0,
    state: int = 1,
    hcdc: int = 0,
    direct_phase_cycles: Decimal | None = None,
    signal: str = "GPS_L1CA",
) -> audit.Row:
    return audit.Row(
        row_number=row_number,
        utc_ms=utc_ms,
        time_ns=time_ns,
        full_bias_ns=0,
        bias_ns=Decimal(0),
        offset_ns=Decimal(0),
        received_sv_time_ns=Decimal(0),
        rate_mps=Decimal(str(rate_mps)),
        adr_state=state,
        adr_m=Decimal(str(adr_m)),
        state=0x4001,
        multipath=0,
        cn0=Decimal("35"),
        hcdc=hcdc,
        svid=svid,
        system="GPS",
        signal=signal,
        frequency_hz=Decimal(str(frequency_hz)),
        direct_phase_cycles=direct_phase_cycles,
        pseudorange_m=20_000_000.0,
        code_masked=False,
        carrier_masked=False,
    )


class Phase53CarrierFrequencyAuditTest(unittest.TestCase):
    def test_freeze_is_pre_read_and_contract_is_unchanged(self) -> None:
        freeze = audit._verify_freeze()
        self.assertEqual(freeze["status"], "frozen-before-phase53-raw-read")
        self.assertEqual(tuple(freeze["cohort"]["route_order"]), audit.ROUTES)
        self.assertEqual(freeze["input_policy"]["truth_read_count"], 0)
        self.assertEqual(freeze["input_policy"]["raw_device_gnss_read_count_per_route"], 1)
        self.assertEqual(freeze["source_decision_policy"]["next_single_raw_physical_factor_if_no_go"], audit.NEXT_FACTOR)
        self.assertFalse(freeze["pre_read_assertions"]["raw_header_inspected_before_freeze"])
        self.assertTrue(freeze["numeric_gates"]["direct_source_availability"]["antenna_phase_center_or_bias_field_required_for_native_antenna_correction"])

    def test_frequency_relation_and_android_phase_sign(self) -> None:
        previous = _row(2, 0, 0, 1, 100.0)
        current = _row(3, 1000, 1_000_000_000, 1, 110.0)
        components = audit._pair_components(previous, current)
        self.assertIsNotNone(components)
        assert components is not None
        self.assertAlmostEqual(components["delta_phase_m"], 10.0, places=9)
        self.assertAlmostEqual(components["frequency_residual_m"], 0.0, places=9)
        self.assertAlmostEqual(components["control_residual_m"], 0.0, places=9)
        self.assertAlmostEqual(components["frequency_leakage_m"], 0.0, places=9)
        self.assertEqual(audit._frequency_group(current), "GPS:GPS_L1CA:1575420000Hz")
        phi = 100.0 * 1_575_420_000.0 / audit.SPEED_OF_LIGHT_MPS
        phase_row = _row(4, 0, 0, 1, 100.0, direct_phase_cycles=Decimal(str(-phi)))
        self.assertAlmostEqual(audit._direct_phase_residual(phase_row), 0.0, places=12)

    def test_pair_boundaries_are_not_promoted_to_ordinary(self) -> None:
        previous = _row(2, 0, 0, 1, 100.0)
        reset = _row(3, 1000, 1_000_000_000, 1, 110.0, state=1 | audit.ADR_RESET)
        hcdc = _row(3, 1000, 1_000_000_000, 1, 110.0, hcdc=1)
        gap = _row(3, 3000, 3_000_000_000, 1, 110.0)
        self.assertEqual(audit._pair_reason(previous, reset, audit._pair_components(previous, reset)), "adr_reset")
        self.assertEqual(audit._pair_reason(previous, hcdc, audit._pair_components(previous, hcdc)), "hcdc_or_segment_boundary")
        self.assertEqual(audit._pair_reason(previous, gap, audit._pair_components(previous, gap)), "gap")

    def test_optional_phase_and_antenna_fields_are_separated(self) -> None:
        headers = [
            "MessageType", "utcTimeMillis", "TimeNanos", "FullBiasNanos", "BiasNanos",
            "ReceivedSvTimeNanos", "Svid", "ConstellationType", "SignalType",
            "CarrierFrequencyHz", "PseudorangeRateMetersPerSecond",
            "AccumulatedDeltaRangeState", "AccumulatedDeltaRangeMeters", "State",
            "HardwareClockDiscontinuityCount", "MultipathIndicator", "Cn0DbHz",
            "CarrierPhase", "CarrierPhaseUncertainty", "AccumulatedDeltaRangeUncertaintyMeters",
            "AntennaPhaseBiasMeters",
        ]
        records = []
        for index in range(2):
            records.append({
                "MessageType": "Raw", "utcTimeMillis": str(index * 1000),
                "TimeNanos": str(index * 1_000_000_000), "FullBiasNanos": "0", "BiasNanos": "0",
                "ReceivedSvTimeNanos": "0", "Svid": "1", "ConstellationType": "1",
                "SignalType": "GPS_L1CA", "CarrierFrequencyHz": "1575420000",
                "PseudorangeRateMetersPerSecond": "10", "AccumulatedDeltaRangeState": "1",
                "AccumulatedDeltaRangeMeters": str(index * 10), "State": "16385",
                "HardwareClockDiscontinuityCount": "0", "MultipathIndicator": "0", "Cn0DbHz": "35",
                "CarrierPhase": "-1", "CarrierPhaseUncertainty": "0.1",
                "AccumulatedDeltaRangeUncertaintyMeters": "0.2", "AntennaPhaseBiasMeters": "0.3",
            })
        stream = io.StringIO()
        writer = csv.DictWriter(stream, fieldnames=headers)
        writer.writeheader()
        writer.writerows(records)
        rows, metadata = audit._parse_payload(stream.getvalue().encode())
        self.assertEqual(len(rows), 2)
        self.assertEqual(metadata["direct_phase_fields"], ["CarrierPhase"])
        self.assertEqual(metadata["phase_uncertainty_fields"], ["CarrierPhaseUncertainty", "AccumulatedDeltaRangeUncertaintyMeters"])
        self.assertEqual(metadata["antenna_phase_bias_fields"], ["AntennaPhaseBiasMeters"])
        self.assertEqual(rows[0].antenna_fields["AntennaPhaseBiasMeters"], Decimal("0.3"))

    def _integrity_fixture(self) -> tuple[dict[str, dict[str, object]], dict[str, object], dict[str, object]]:
        reports: dict[str, dict[str, object]] = {}
        for route in audit.ROUTES:
            reports[route] = {
                "rows": {"pair_count": 3, "ordinary_pair_count": 3},
                "pair_reasons": {"ordinary": 3},
                "headers": {"columns": [], "optional_field_presence": {}},
                "groups": {
                    "signal_frequency": {"GPS:GPS_L1CA:1575420000Hz": {"count": 3}},
                    "satellite": {"GPS:1": {"count": 3}},
                    "state": {"ordinary": {"count": 3}},
                },
            }
        medians = {route: float(index) for index, route in enumerate(audit.ROUTES, start=1)}
        aggregate: dict[str, object] = {
            "route_median_abs_frequency_residual_m": medians,
            "route_median_abs_frequency_residual_aggregate_m": audit._median(medians.values()),
            "route_median_abs_frequency_residual_mad_m": audit._mad(medians.values()),
        }
        loo = {"folds": [{"omitted_route": route} for route in audit.ROUTES]}
        return reports, aggregate, loo

    def test_presentation_integrity_catches_stale_groups_and_collapsed_routes(self) -> None:
        reports, aggregate, loo = self._integrity_fixture()
        good = audit._presentation_integrity(reports, aggregate, loo)
        self.assertTrue(good["pair_reason_counts_sum_all"])
        self.assertTrue(good["state_group_counts_sum_all"])
        self.assertTrue(good["signal_frequency_group_counts_sum_all"])
        self.assertTrue(good["satellite_group_counts_sum_all"])
        self.assertTrue(good["four_route_medians_retained"])
        self.assertTrue(good["aggregate_recomputed_exact"])
        reports[audit.ROUTES[0]]["groups"]["signal_frequency"]["GPS:GPS_L1CA:1575420000Hz"]["count"] = 2  # type: ignore[index]
        stale = audit._presentation_integrity(reports, aggregate, loo)
        self.assertFalse(stale["signal_frequency_group_counts_sum_all"])
        collapsed = dict(aggregate)
        collapsed["route_median_abs_frequency_residual_m"] = {audit.ROUTES[0]: 1.0}
        self.assertFalse(audit._presentation_integrity(reports, collapsed, loo)["four_route_medians_retained"])

    def test_static_source_contract_is_solver_free_and_antenna_parser_absent(self) -> None:
        freeze = audit._verify_freeze()
        contract = audit._static_contract(freeze)
        self.assertTrue(contract["adapter_parses_carrier_frequency"])
        self.assertTrue(contract["adapter_parses_accumulated_delta_range"])
        self.assertTrue(contract["adapter_has_no_android_antenna_phase_field_parser"])
        self.assertTrue(contract["adapter_header_has_no_android_antenna_phase_field"])
        source = Path(audit.__file__).read_text(encoding="utf-8")
        self.assertNotIn("subprocess", source)
        self.assertNotIn("zipfile", source)
        self.assertNotIn("gnss_fgo_imu_no_base", source)


if __name__ == "__main__":
    unittest.main()
