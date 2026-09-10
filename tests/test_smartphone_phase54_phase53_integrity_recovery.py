"""Focused tests for the Phase54 immutable-artifact integrity recovery."""

from __future__ import annotations

from pathlib import Path
import sys
import tempfile
import unittest


ROOT = Path(__file__).resolve().parents[1]
BENCHMARKS = ROOT / "apps" / "commands" / "benchmarks"
sys.path.insert(0, str(BENCHMARKS))

import gnss_smartphone_phase54_phase53_integrity_recovery as recovery  # noqa: E402


def _route_report(index: int) -> dict[str, object]:
    return {
        "rows": {
            "pair_count": 3,
            "ordinary_pair_count": 2,
            "unsupported_signal_rows": 100 + index,
            "nonmonotonic_epoch_key_count": 0,
            "repeated_epoch_key_count": 0,
            "signal_frequency_group_count": 1,
        },
        "headers": {"columns": ["MessageType"], "optional_field_presence": {}},
        "pair_reasons": {"ordinary": 2, "gap": 1},
        "groups": {
            "signal_frequency": {"GPS:GPS_L1CA:1575420000Hz": {"count": 2}},
            "satellite": {"GPS:1": {"count": 2}},
            "state": {"ordinary": {"count": 2}, "gap": {"count": 1}},
        },
        "residuals": {
            "frequency_aware_centered_m": {"median": float(index + 1)},
            "frequency_aware_abs_m": {"median": float(index + 1), "p95_abs": 0.1},
            "constant_frequency_control_abs_m": {"p95_abs": 0.1},
            "frequency_leakage_abs_m": {"p95_abs": 1.0e-11},
        },
        "antenna_phase_bias": {"available": False},
        "gate_observations": {
            "all_core_finite": True,
            "frequency_leakage_p95_abs_m": 1.0e-11,
            "frequency_vs_control_p95_excess_m": 0.0,
            "spearman": 0.0,
        },
    }


class Phase54IntegrityRecoveryTest(unittest.TestCase):
    def test_freeze_is_sealed_and_excludes_unsupported_rows_from_gate(self) -> None:
        freeze = recovery._verify_freeze()
        self.assertEqual(freeze["status"], "frozen-before-phase54-scorer")
        self.assertEqual(freeze["phase54_input_policy"]["raw_device_gnss_reads"], 0)
        self.assertEqual(
            freeze["integrity_contract"]["unsupported_signal_rows"]["gate_role"],
            "informational_only_excluded_from_raw_input_integrity",
        )
        self.assertFalse(freeze["pre_read_assertions"]["phase53_raw_reopen"])

    def _bundle(self) -> tuple[dict[str, object], dict[str, object], dict[str, object], dict[str, object]]:
        routes = {route: _route_report(index) for index, route in enumerate(recovery.ROUTES)}
        medians = {route: float(index + 1) for index, route in enumerate(recovery.ROUTES)}
        result: dict[str, object] = {
            "status": "no-go-carrier-frequency-antenna-phase-bias-not-identifiable",
            "phase": 53,
            "events": {"count": 0},
            "aggregate": {
                "route_median_abs_frequency_residual_m": medians,
                "route_median_abs_frequency_residual_aggregate_m": recovery._median(medians.values()),
                "route_median_abs_frequency_residual_mad_m": recovery._mad(medians.values()),
            },
            "loo": {"folds": [{"omitted_route": route} for route in recovery.ROUTES]},
            "gates": {"all_passed": False, "observed": {"raw_input_integrity": False, "presentation_integrity": True}},
            "decision": {
                "correction_authorized": False,
                "next_single_raw_physical_factor": "raw Android per-satellite accumulated-delta-range uncertainty (AccumulatedDeltaRangeUncertaintyMeters)",
            },
        }
        routes_artifact = {"phase": 53, "route_order": list(recovery.ROUTES), "routes": routes}
        events = {"phase": 53, "count": 0}
        output_manifest = {"artifacts": {"result": {"bytes": 1, "sha256": "a" * 64}, "routes": {"bytes": 1, "sha256": "b" * 64}, "events": {"bytes": 1, "sha256": "c" * 64}}}
        return result, routes_artifact, events, output_manifest

    def test_recomputed_integrity_passes_with_unsupported_rows(self) -> None:
        result, routes, events, output_manifest = self._bundle()
        recomputed = recovery._recompute_integrity(result, routes, events, output_manifest)
        self.assertTrue(recomputed["raw_input_integrity"]["all_passed"])
        self.assertTrue(recomputed["presentation_integrity"]["all_passed"])
        self.assertEqual(
            recomputed["raw_input_integrity"]["route_checks"][recovery.ROUTES[0]]["unsupported_signal_rows_informational"],
            100,
        )

    def test_stale_group_or_collapsed_route_fails_closed(self) -> None:
        result, routes, events, output_manifest = self._bundle()
        routes["routes"][recovery.ROUTES[0]]["groups"]["state"]["gap"]["count"] = 2  # type: ignore[index]
        recomputed = recovery._recompute_integrity(result, routes, events, output_manifest)
        self.assertFalse(recomputed["presentation_integrity"]["all_passed"])
        routes, events, output_manifest = self._bundle()[1:]
        result["aggregate"]["route_median_abs_frequency_residual_m"] = {recovery.ROUTES[0]: 1.0}  # type: ignore[index]
        recomputed = recovery._recompute_integrity(result, routes, events, output_manifest)
        self.assertFalse(recomputed["presentation_integrity"]["four_route_medians_retained"])

    def test_physical_phase53_no_go_is_preserved(self) -> None:
        result, routes, events, output_manifest = self._bundle()
        physical = recovery._physical_no_go(result, routes, {"unused": True})
        self.assertTrue(physical["immutable_no_go"])
        self.assertTrue(physical["checks"]["correction_authorized_false"])
        self.assertTrue(physical["checks"]["routewise_spearman_zero"])

    def test_single_read_helper_hashes_without_raw_input(self) -> None:
        with tempfile.TemporaryDirectory(prefix="phase54-") as directory:
            path = Path(directory) / "sealed.json"
            path.write_bytes(b"{}\n")
            payload, digest = recovery._read_bytes_once(path, "sealed artifact")
        self.assertEqual(payload, b"{}\n")
        self.assertEqual(digest, recovery._sha256_bytes(payload))

    def test_source_has_no_process_or_solver_invocation(self) -> None:
        source = Path(recovery.__file__).read_text(encoding="utf-8")
        self.assertNotIn("subprocess", source)
        self.assertNotIn("Popen", source)
        self.assertNotIn("gnss_fgo", source)
        self.assertNotIn("truth_maps", source)


if __name__ == "__main__":
    unittest.main()
