"""Focused raw-only contract tests for the Phase50 half-cycle audit."""

from __future__ import annotations

from decimal import Decimal
from pathlib import Path
import sys
import tempfile
import unittest


ROOT = Path(__file__).resolve().parents[1]
BENCHMARKS = ROOT / "apps" / "commands" / "benchmarks"
sys.path.insert(0, str(BENCHMARKS))

import gnss_smartphone_phase50_pixel5_adr_half_cycle_audit as audit  # noqa: E402


def _row(
    row_number: int,
    utc_ms: int,
    time_ns: int,
    svid: int,
    adr_m: float,
    *,
    state: int = 1,
    rate_mps: float = 0.0,
    hcdc: int = 0,
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
        rate_uncertainty_mps=Decimal("0.2"),
        adr_state=state,
        adr_m=Decimal(str(adr_m)),
        state=0x4001,
        multipath=0,
        cn0=Decimal("35"),
        hcdc=hcdc,
        svid=svid,
        system="GPS",
        signal=signal,
        frequency_hz=Decimal("1575420000"),
        pseudorange_m=20_000_000.0,
        code_masked=False,
        carrier_masked=False,
    )


class Phase50AdrHalfCycleAuditTest(unittest.TestCase):
    def test_freeze_is_pre_read_and_bits_and_gates_are_sealed(self) -> None:
        freeze = audit._verify_freeze()
        self.assertEqual(freeze["status"], "frozen-before-phase50-raw-read")
        self.assertEqual(tuple(freeze["cohort"]["route_order"]), audit.ROUTES)
        self.assertEqual(freeze["input_policy"]["truth_read_count"], 0)
        bits = freeze["adr_state_contract"]
        self.assertEqual(bits["half_cycle_resolved_bit"], 8)
        self.assertEqual(bits["half_cycle_reported_bit"], 16)
        self.assertEqual(freeze["numeric_gates"]["state_class_population"]["resolved_stable_min_pairs_per_route"], 100)
        self.assertEqual(freeze["numeric_gates"]["half_cycle_cluster"]["nearest_integer_half_cycle_tolerance"], 0.15)
        self.assertFalse(freeze["pre_read_assertions"]["raw_payload_opened"])

    def test_state_pair_classes_partition_stable_and_toggle_states(self) -> None:
        valid = audit.ADR_VALID
        resolved = audit.ADR_HALF_CYCLE_RESOLVED
        reported = audit.ADR_HALF_CYCLE_REPORTED
        self.assertEqual(audit._state_pair_class(_row(2, 0, 0, 1, 0, state=valid | resolved), _row(3, 1000, 1_000_000_000, 1, 0, state=valid | resolved)), "resolved_to_resolved")
        self.assertEqual(audit._state_pair_class(_row(2, 0, 0, 1, 0, state=valid), _row(3, 1000, 1_000_000_000, 1, 0, state=valid)), "unresolved_reported_stable")
        self.assertEqual(audit._state_pair_class(_row(2, 0, 0, 1, 0, state=valid), _row(3, 1000, 1_000_000_000, 1, 0, state=valid | resolved)), "resolved_toggle")
        self.assertEqual(audit._state_pair_class(_row(2, 0, 0, 1, 0, state=valid), _row(3, 1000, 1_000_000_000, 1, 0, state=valid | reported)), "reported_toggle")
        self.assertEqual(audit._state_pair_class(_row(2, 0, 0, 1, 0, state=valid | resolved | reported), _row(3, 1000, 1_000_000_000, 1, 0, state=valid)), "resolved_and_reported_toggle")
        self.assertEqual(audit._adr_bits(valid | resolved | reported), ["VALID", "HALF_CYCLE_RESOLVED", "HALF_CYCLE_REPORTED"])

    def test_sign_trapezoid_and_half_cycle_units_are_fixed(self) -> None:
        previous = _row(2, 0, 0, 1, 100.0, rate_mps=10.0)
        current = _row(3, 1000, 1_000_000_000, 1, 110.0, rate_mps=10.0)
        self.assertAlmostEqual(audit._signed_adr(previous, "pixel5"), 100.0)
        self.assertAlmostEqual(audit._signed_adr(previous, "sm-a205u"), -100.0)
        transitions, _ = audit._transitions([audit.Epoch(0, [previous]), audit.Epoch(1000, [current])])
        self.assertAlmostEqual(float(transitions[0]["raw_delta_adr_m"]), 10.0)
        self.assertAlmostEqual(float(transitions[0]["raw_rate_integral_m"]), 10.0)
        self.assertAlmostEqual(float(transitions[0]["raw_residual_m"]), 0.0)

        half_wavelength = 0.5 * audit.SPEED_OF_LIGHT_MPS / 1_575_420_000.0
        epochs: list[audit.Epoch] = []
        values = {1: [0.0, half_wavelength, half_wavelength], 2: [0.0, 0.0, 0.0], 3: [0.0, 0.0, 0.0]}
        states = {1: [1, 1 | audit.ADR_HALF_CYCLE_RESOLVED, 1 | audit.ADR_HALF_CYCLE_RESOLVED], 2: [1, 1, 1], 3: [1, 1, 1]}
        row_number = 2
        for index in range(3):
            rows = []
            for svid in (1, 2, 3):
                rows.append(_row(row_number, index * 1000, index * 1_000_000_000, svid, values[svid][index], state=states[svid][index]))
                row_number += 1
            epochs.append(audit.Epoch(index * 1000, rows))
        transitions, _ = audit._transitions(epochs)
        for item in transitions:
            if item["ordinary"]:
                frequency = float(item["frequency_hz"])
                wavelength = audit.SPEED_OF_LIGHT_MPS / frequency
                item["wavelength_m"] = wavelength
                item["half_wavelength_m"] = 0.5 * wavelength
                units = float(item["centered_residual_m"]) / item["half_wavelength_m"]
                item["half_cycle_units"] = units
                item["abs_half_cycle_units"] = abs(units)
                nearest = round(units)
                item["nearest_integer_half_cycle"] = nearest
                item["half_cycle_distance"] = abs(units - nearest)
        toggles = [item for item in transitions if item["implicated"]]
        self.assertEqual(len(toggles), 1)
        self.assertEqual(toggles[0]["state_pair_class"], "resolved_toggle")
        self.assertAlmostEqual(float(toggles[0]["half_cycle_units"]), 1.0, places=6)
        self.assertLessEqual(float(toggles[0]["half_cycle_distance"]), 0.15)

    def test_reset_hcdc_and_gap_are_excluded_separately(self) -> None:
        self.assertEqual(audit._pair_reason(_row(2, 0, 0, 1, 0, state=1 | audit.ADR_RESET), _row(3, 1000, 1_000_000_000, 1, 0), 1_000_000_000, 0.0), "adr_reset")
        self.assertEqual(audit._pair_reason(_row(2, 0, 0, 1, 0), _row(3, 1000, 1_000_000_000, 1, 0, hcdc=1), 1_000_000_000, 0.0), "hcdc_boundary")
        self.assertEqual(audit._pair_reason(_row(2, 0, 0, 1, 0), _row(3, 3000, 3_000_000_000, 1, 0), 3_000_000_000, 0.0), "gap")

    def _integrity_fixture(self) -> tuple[dict[str, dict[str, object]], dict[str, object]]:
        reports: dict[str, dict[str, object]] = {}
        for route in audit.ROUTES:
            reports[route] = {
                "rows": {"pair_count": 3, "ordinary_tdcp_eligible_count": 3},
                "adr_state": {
                    "pair_reasons": {"ordinary": 3},
                    "pair_state_classes": {"resolved_to_resolved": 2, "unresolved_reported_stable": 1},
                },
                "adr_rate_residual": {
                    "signal_groups": {"GPS:GPS_L1CA": {"count": 3, "ordinary_count": 3}},
                    "satellite_groups": {"GPS:1:GPS_L1CA": {"count": 3, "ordinary_count": 3}},
                },
                "events": {"count": 0},
                "_events": [],
            }
        medians = {route: float(index) for index, route in enumerate(audit.ROUTES, start=1)}
        aggregate: dict[str, object] = {
            "route_median_abs_residual_m": medians,
            "route_median_abs_residual_aggregate_m": audit._median(medians.values()),
            "route_median_abs_residual_mad_m": audit._mad(medians.values()),
        }
        return reports, aggregate

    def test_presentation_integrity_catches_stale_groups_and_collapsed_routes(self) -> None:
        reports, aggregate = self._integrity_fixture()
        good = audit._presentation_integrity(reports, aggregate, 0)
        self.assertTrue(good["pair_reason_counts_sum_all"])
        self.assertTrue(good["state_class_counts_sum_all"])
        self.assertTrue(good["signal_group_counts_sum_all"])
        self.assertTrue(good["satellite_group_counts_sum_all"])
        self.assertTrue(good["four_route_medians_retained"])
        self.assertTrue(good["aggregate_recomputed_exact"])
        self.assertTrue(good["event_count_exact"])
        reports[audit.ROUTES[0]]["adr_rate_residual"]["signal_groups"]["GPS:GPS_L1CA"]["count"] = 39  # type: ignore[index]
        stale = audit._presentation_integrity(reports, aggregate, 0)
        self.assertFalse(stale["signal_group_counts_sum_all"])
        collapsed = dict(aggregate)
        collapsed["route_median_abs_residual_m"] = {audit.ROUTES[0]: 1.0}
        self.assertFalse(audit._presentation_integrity(reports, collapsed, 0)["four_route_medians_retained"])

    def test_static_contract_is_solver_free_and_half_cycle_bits_are_not_loader_gates(self) -> None:
        freeze = audit._verify_freeze()
        contract = audit._static_contract(freeze)
        self.assertTrue(contract["adapter_parses_adr_state"])
        self.assertTrue(contract["adapter_carrier_mask_ignores_half_cycle_bits"])
        self.assertTrue(contract["adapter_lli_ignores_half_cycle_bits"])
        self.assertTrue(contract["fgo_tdcp_arc_reset_uses_loss_of_lock_lli"])
        source = Path(audit.__file__).read_text(encoding="utf-8")
        self.assertNotIn("subprocess", source)
        self.assertNotIn("zipfile", source)
        self.assertNotIn("run_native", source)

    def test_one_read_hash_accounting_uses_same_buffer(self) -> None:
        with tempfile.TemporaryDirectory(prefix="phase50-") as directory:
            path = Path(directory) / "synthetic.csv"
            path.write_bytes(b"Raw\n")
            payload, digest = audit._read_bytes_once(path, "synthetic raw")
        self.assertEqual(payload, b"Raw\n")
        self.assertEqual(digest, audit._sha256_bytes(payload))


if __name__ == "__main__":
    unittest.main()
