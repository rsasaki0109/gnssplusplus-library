"""Focused raw-only contract tests for the Phase48 uncertainty audit."""

from __future__ import annotations

from decimal import Decimal
from pathlib import Path
import sys
import tempfile
import unittest


ROOT = Path(__file__).resolve().parents[1]
BENCHMARKS = ROOT / "apps" / "commands" / "benchmarks"
sys.path.insert(0, str(BENCHMARKS))

import gnss_smartphone_phase48_pixel5_raw_code_rate_uncertainty_audit as audit  # noqa: E402


def _row(
    row_number: int,
    utc_ms: int,
    time_ns: int,
    svid: int,
    signal: str,
    pseudorange_m: float,
    rate_mps: float,
    *,
    uncertainty_ns: float | None = 5.0,
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
        sv_uncertainty_ns=None if uncertainty_ns is None else Decimal(str(uncertainty_ns)),
        state=0x4001,
        multipath=0,
        cn0=Decimal("35"),
        hcdc=0,
        svid=svid,
        system="GPS",
        signal=signal,
        frequency_hz=Decimal("1575420000"),
        pseudorange_m=pseudorange_m,
        code_masked=False,
    )


def _transition_items() -> list[dict[str, object]]:
    """Make two endpoint transitions with known raw residual signs."""
    epochs = [
        audit.Epoch(
            key_ms=0,
            rows=[
                _row(2, 0, 0, 1, "GPS_L1CA", 100.0, 10.0),
                _row(4, 0, 0, 2, "GPS_L1CA", 200.0, 10.0),
            ],
        ),
        audit.Epoch(
            key_ms=1000,
            rows=[
                _row(3, 1000, 1_000_000_000, 1, "GPS_L1CA", 112.0, 10.0),
                _row(5, 1000, 1_000_000_000, 2, "GPS_L1CA", 210.0, 10.0),
            ],
        ),
    ]
    transitions, _ = audit._transitions(epochs)
    return transitions


class Phase48RawCodeRateUncertaintyAuditTest(unittest.TestCase):
    def test_freeze_is_pre_read_and_gates_are_sealed(self) -> None:
        freeze = audit._verify_freeze()
        self.assertEqual(freeze["status"], "frozen-before-phase48-raw-read")
        self.assertEqual(tuple(freeze["cohort"]["route_order"]), audit.ROUTES)
        self.assertEqual(freeze["input_policy"]["truth_read_count"], 0)
        self.assertEqual(freeze["numeric_gates"]["high_vs_low"]["high_bucket"], ">10ns")
        self.assertEqual(freeze["numeric_gates"]["spearman"]["min_each_route"], 0.35)
        self.assertFalse(freeze["pre_read_assertions"]["raw_payload_opened"])

    def test_trapezoid_code_rate_sign_and_clock_center_are_explicit(self) -> None:
        transitions = _transition_items()
        self.assertEqual(len(transitions), 2)
        # (112-100) - (10+10)/2*1 = +2 m.  The second transition is +0 m.
        self.assertAlmostEqual(float(transitions[0]["raw_residual_m"]), 2.0)
        self.assertAlmostEqual(float(transitions[1]["raw_residual_m"]), 0.0)
        self.assertAlmostEqual(float(transitions[0]["clock_group_median_m"]), 1.0)
        self.assertAlmostEqual(float(transitions[0]["abs_centered_residual_m"]), 1.0)

    def test_high_materiality_pool_is_strictly_greater_than_ten_ns(self) -> None:
        items = [
            {"sv_uncertainty_ns": 5.0, "abs_centered_residual_m": 1.0},
            {"sv_uncertainty_ns": 20.0, "abs_centered_residual_m": 3.0},
            {"sv_uncertainty_ns": 200.0, "abs_centered_residual_m": 5.0},
        ]
        report = audit._bucket_report(items)
        self.assertEqual(report["high_low"]["high_definition"], ">10ns")
        self.assertEqual(report["high_low"]["low_count"], 1)
        self.assertEqual(report["high_low"]["high_count"], 2)
        self.assertGreater(report["high_low"]["p95_excess_m"], 2.0)
        self.assertGreaterEqual(float(report["high_low"]["p95_ratio"]), 1.5)

    def test_rank_correlation_uses_tie_averaged_ranks(self) -> None:
        self.assertAlmostEqual(audit._spearman([1.0, 1.0, 2.0], [2.0, 3.0, 4.0]), 0.8660254038)
        self.assertEqual(audit._spearman([1.0, 1.0], [2.0, 3.0]), 0.0)

    def test_presentation_integrity_catches_stale_groups_and_collapsed_routes(self) -> None:
        reports: dict[str, dict[str, object]] = {}
        for index, route in enumerate(audit.ROUTES, start=1):
            ordered = {
                "missing": {"count": 0},
                "<=10ns": {"count": 2},
                "10_to_100ns": {"count": 1},
                ">100ns": {"count": 0},
            }
            reports[route] = {
                "rows": {"transition_count": 3},
                "code_rate_residual": {"spearman_and_buckets": {"ordered": ordered}},
            }
        medians = {route: float(index) for index, route in enumerate(audit.ROUTES, start=1)}
        aggregate = {
            "route_median_abs_residual_m": medians,
            "route_median_abs_residual_aggregate_m": audit._median(medians.values()),
            "route_median_abs_residual_mad_m": audit._mad(medians.values()),
        }
        good = audit._presentation_integrity(reports, aggregate)
        self.assertTrue(good["group_counts_sum_all"])
        self.assertTrue(good["four_route_medians_retained"])
        self.assertTrue(good["aggregate_recomputed_exact"])

        # A stale inner-loop count (Phase47's failure mode) must fail closed.
        reports[audit.ROUTES[0]]["code_rate_residual"]["spearman_and_buckets"]["ordered"]["<=10ns"]["count"] = 39  # type: ignore[index]
        stale = audit._presentation_integrity(reports, aggregate)
        self.assertFalse(stale["group_counts_sum_all"])

        # A single collapsed aggregate value must not masquerade as four routes.
        collapsed = dict(aggregate)
        collapsed["route_median_abs_residual_m"] = {audit.ROUTES[0]: medians[audit.ROUTES[0]]}
        collapsed_check = audit._presentation_integrity(reports, collapsed)
        self.assertFalse(collapsed_check["four_route_medians_retained"])

    def test_static_contract_audit_is_solver_free_and_field_not_assumed(self) -> None:
        freeze = audit._verify_freeze()
        contract = audit._static_contract(freeze)
        self.assertIn("source_hashes", contract)
        self.assertFalse(contract["adapter_parses_received_sv_time_uncertainty"])
        self.assertFalse(contract["fgo_consumes_received_sv_time_uncertainty_as_sigma"])
        source = Path(audit.__file__).read_text(encoding="utf-8")
        self.assertNotIn("subprocess", source)
        self.assertNotIn("zipfile", source)
        self.assertNotIn("run_native", source)

    def test_one_read_hash_accounting_uses_same_buffer(self) -> None:
        with tempfile.TemporaryDirectory(prefix="phase48-") as directory:
            path = Path(directory) / "synthetic.csv"
            path.write_bytes(b"Raw\n")
            payload, digest = audit._read_bytes_once(path, "synthetic raw")
        self.assertEqual(payload, b"Raw\n")
        self.assertEqual(digest, audit._sha256_bytes(payload))


if __name__ == "__main__":
    unittest.main()
