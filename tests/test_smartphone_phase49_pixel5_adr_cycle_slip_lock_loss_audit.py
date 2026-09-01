"""Focused raw-only contract tests for the Phase49 ADR audit."""

from __future__ import annotations

from decimal import Decimal
from pathlib import Path
import sys
import tempfile
import unittest


ROOT = Path(__file__).resolve().parents[1]
BENCHMARKS = ROOT / "apps" / "commands" / "benchmarks"
sys.path.insert(0, str(BENCHMARKS))

import gnss_smartphone_phase49_pixel5_adr_cycle_slip_lock_loss_audit as audit  # noqa: E402


def _row(
    row_number: int,
    utc_ms: int,
    time_ns: int,
    svid: int,
    adr_m: float,
    *,
    rate_mps: float = 10.0,
    state: int | None = 1,
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


def _epochs_with_one_jump() -> list[audit.Epoch]:
    """Three satellites make the receiver center zero at the jump epoch."""
    epochs: list[audit.Epoch] = []
    values = {
        1: [0.0, 10.0, 22.0, 32.0],
        2: [0.0, 10.0, 20.0, 30.0],
        3: [0.0, 10.0, 20.0, 30.0],
    }
    row_number = 2
    for index in range(4):
        utc_ms = index * 1000
        rows = []
        for svid in (1, 2, 3):
            rows.append(_row(row_number, utc_ms, utc_ms * 1_000_000, svid, values[svid][index]))
            row_number += 1
        epochs.append(audit.Epoch(key_ms=utc_ms, rows=rows))
    return epochs


class Phase49AdrCycleSlipLockLossAuditTest(unittest.TestCase):
    def test_freeze_is_pre_read_and_gates_are_sealed(self) -> None:
        freeze = audit._verify_freeze()
        self.assertEqual(freeze["status"], "frozen-before-phase49-raw-read")
        self.assertEqual(tuple(freeze["cohort"]["route_order"]), audit.ROUTES)
        self.assertEqual(freeze["input_policy"]["truth_read_count"], 0)
        self.assertEqual(freeze["numeric_gates"]["candidate_score_population"]["score_threshold_m"], 1.5)
        self.assertTrue(freeze["numeric_gates"]["candidate_score_population"]["two_sided_adjacency_required"])
        self.assertFalse(freeze["pre_read_assertions"]["raw_payload_opened"])

    def test_adr_state_bits_and_classes_are_explicit(self) -> None:
        self.assertEqual(audit._adr_bits(1), ["VALID"])
        self.assertEqual(audit._adr_class(1), "VALID_UNFLAGGED")
        self.assertEqual(audit._adr_class(1 | audit.ADR_RESET), "RESET")
        self.assertEqual(audit._adr_class(1 | audit.ADR_CYCLE_SLIP), "CYCLE_SLIP")
        self.assertEqual(audit._adr_class(1 | audit.ADR_HALF_CYCLE_REPORTED), "VALID_HALF_CYCLE_REPORTED")
        self.assertEqual(
            audit._adr_class(1 | audit.ADR_HALF_CYCLE_REPORTED | audit.ADR_HALF_CYCLE_RESOLVED),
            "VALID_HALF_CYCLE_REPORTED_RESOLVED",
        )
        self.assertFalse(audit._adr_unflagged(_row(2, 0, 0, 1, 0.0, state=1 | audit.ADR_RESET)))
        self.assertEqual(audit._pair_reason(_row(2, 0, 0, 1, 0.0, state=1 | audit.ADR_RESET), _row(3, 1000, 1_000_000_000, 1, 10.0), 1_000_000_000, 0.0), "adr_reset")
        self.assertEqual(
            audit._pair_reason(
                _row(2, 0, 0, 1, 0.0),
                _row(3, 1000, 1_000_000_000, 1, 10.0, hcdc=1),
                1_000_000_000,
                0.0,
            ),
            "hcdc_boundary",
        )
        self.assertEqual(
            audit._pair_reason(
                _row(2, 0, 0, 1, 0.0),
                _row(3, 3000, 3_000_000_000, 1, 30.0),
                3_000_000_000,
                0.0,
            ),
            "gap",
        )

    def test_pixel5_sign_and_trapezoid_residual_are_fixed(self) -> None:
        previous = _row(2, 0, 0, 1, 100.0)
        current = _row(3, 1000, 1_000_000_000, 1, 112.0)
        self.assertAlmostEqual(audit._signed_adr(previous, "pixel5"), 100.0)
        self.assertAlmostEqual(audit._signed_adr(previous, "sm-a205u"), -100.0)
        transitions, _ = audit._transitions([audit.Epoch(0, [previous]), audit.Epoch(1000, [current])])
        self.assertEqual(len(transitions), 1)
        self.assertAlmostEqual(float(transitions[0]["raw_delta_adr_m"]), 12.0)
        self.assertAlmostEqual(float(transitions[0]["raw_rate_integral_m"]), 10.0)
        self.assertAlmostEqual(float(transitions[0]["raw_residual_m"]), 2.0)

    def test_two_sided_jump_is_scored_only_for_ordinary_adjacent_pairs(self) -> None:
        transitions, events = audit._transitions(_epochs_with_one_jump())
        ordinary = [item for item in transitions if item["ordinary"]]
        candidates = [item for item in ordinary if item["candidate"]]
        self.assertEqual(len(transitions), 9)
        self.assertEqual(len(ordinary), 9)
        self.assertEqual(len(candidates), 1)
        self.assertEqual(candidates[0]["svid"], 1)
        self.assertTrue(candidates[0]["two_sided_context"])
        self.assertAlmostEqual(float(candidates[0]["abs_centered_residual_m"]), 2.0)
        self.assertTrue(any(item["kind"] == "unflagged_adr_jump_candidate" for item in events))

        reset_epochs = _epochs_with_one_jump()
        reset_epochs[2].rows[0].adr_state = 1 | audit.ADR_CYCLE_SLIP
        reset_transitions, _ = audit._transitions(reset_epochs)
        reasons = [item["reason"] for item in reset_transitions if item["svid"] == 1]
        self.assertIn("cycle_slip", reasons)
        self.assertEqual(sum(item["candidate"] for item in reset_transitions if item["svid"] == 1), 0)

    def test_pair_summary_reports_normalized_residual_and_stable_arc(self) -> None:
        transitions, _ = audit._transitions(_epochs_with_one_jump())
        report = audit._pair_summary(transitions)
        self.assertEqual(report["ordinary_tdcp_eligible_count"], 9)
        self.assertEqual(report["candidate_count"], 1)
        self.assertGreaterEqual(report["candidate_p95_excess_m"], 0.5)
        self.assertEqual(report["two_sided_context_fraction"], 1.0)
        self.assertIn("normalized_abs_centered_residual", report)

    def _integrity_fixture(self) -> tuple[dict[str, dict[str, object]], dict[str, object]]:
        reports: dict[str, dict[str, object]] = {}
        for route in audit.ROUTES:
            reports[route] = {
                "rows": {"pair_count": 3, "ordinary_tdcp_eligible_count": 3},
                "adr_state": {"pair_reasons": {"ordinary": 3}},
                "adr_rate_residual": {
                    "signal_groups": {
                        "GPS:GPS_L1CA": {"count": 3, "ordinary_count": 3},
                    },
                    "satellite_groups": {
                        "GPS:1:GPS_L1CA": {"count": 3, "ordinary_count": 3},
                    },
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
        self.assertTrue(good["state_group_counts_sum_all"])
        self.assertTrue(good["signal_group_counts_sum_all"])
        self.assertTrue(good["satellite_group_counts_sum_all"])
        self.assertTrue(good["four_route_medians_retained"])
        self.assertTrue(good["aggregate_recomputed_exact"])
        self.assertTrue(good["event_count_exact"])

        # A stale inner-loop count like Phase47's failure mode must fail closed.
        reports[audit.ROUTES[0]]["adr_rate_residual"]["signal_groups"]["GPS:GPS_L1CA"]["count"] = 39  # type: ignore[index]
        stale = audit._presentation_integrity(reports, aggregate, 0)
        self.assertFalse(stale["state_group_counts_sum_all"])

        collapsed = dict(aggregate)
        collapsed["route_median_abs_residual_m"] = {audit.ROUTES[0]: 1.0}
        collapsed_check = audit._presentation_integrity(reports, collapsed, 0)
        self.assertFalse(collapsed_check["four_route_medians_retained"])

    def test_static_contract_is_solver_free_and_phase48_metrics_are_not_read(self) -> None:
        freeze = audit._verify_freeze()
        contract = audit._static_contract(freeze)
        self.assertIn("source_hashes", contract)
        self.assertTrue(contract["adapter_parses_adr_state"])
        self.assertTrue(contract["adapter_uses_published_adr_sign_policy"])
        self.assertTrue(contract["fgo_tdcp_key_gap_lli_hcdc_contract_present"])
        source = Path(audit.__file__).read_text(encoding="utf-8")
        self.assertNotIn("subprocess", source)
        self.assertNotIn("zipfile", source)
        self.assertNotIn("run_native", source)

    def test_one_read_hash_accounting_uses_same_buffer(self) -> None:
        with tempfile.TemporaryDirectory(prefix="phase49-") as directory:
            path = Path(directory) / "synthetic.csv"
            path.write_bytes(b"Raw\n")
            payload, digest = audit._read_bytes_once(path, "synthetic raw")
        self.assertEqual(payload, b"Raw\n")
        self.assertEqual(digest, audit._sha256_bytes(payload))


if __name__ == "__main__":
    unittest.main()
