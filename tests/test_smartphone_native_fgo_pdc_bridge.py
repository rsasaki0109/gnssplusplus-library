#!/usr/bin/env python3
"""Contract tests for the truth-free native-FGO P/D/C bridge."""

from __future__ import annotations

import math
from pathlib import Path
import sys
import unittest
from types import SimpleNamespace


ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "apps" / "commands" / "benchmarks"))
import gnss_smartphone_native_fgo_pdc_bridge as bridge  # noqa: E402


class NativeFgoPdcBridgeTests(unittest.TestCase):
    def test_command_is_no_base_and_enables_raw_doppler(self) -> None:
        route = {"obs": Path("rover.obs"), "nav": Path("brdc.nav")}
        command = bridge._command(Path("gnss_fgo"), route, Path("tmp"), 30)
        self.assertIn("--undifferenced-doppler-factors", command)
        self.assertIn("--carrier-phase-factors", command)
        self.assertIn("--velocity-states", command)
        self.assertIn("--velocity-motion-factors", command)
        self.assertIn("--ambiguity-between-factors", command)
        self.assertIn("--no-dd-factors", command)
        self.assertNotIn("--base", command)

    def test_summary_requires_finite_nonzero_factor_coverage(self) -> None:
        summary = {
            "backend": "eigen",
            "preset": "default",
            "input_epochs": 2,
            "optimized_epochs": 2,
            "valid_solutions": 2,
            "pseudorange_factors": 8,
            "undifferenced_doppler_factors": 8,
            "tdcp_factors": 4,
            "carrier_phase_factors": 4,
            "ambiguity_states": 2,
            "motion_factors": 1,
            "graph_factors": 30,
            "graph_values": 12,
            "initial_cost": 10.0,
            "final_cost": 5.0,
            "single_difference_doppler_factors": 0,
            "single_difference_tdcp_factors": 0,
            "double_difference_pseudorange_factors": 0,
            "double_difference_carrier_factors": 0,
            "use_undifferenced_doppler_factors": True,
            "use_corrected_undifferenced_doppler_factors": False,
            "use_single_difference_doppler_factors": False,
            "use_single_difference_tdcp_factors": False,
            "use_velocity_states": True,
            "use_velocity_motion_factors": True,
        }
        self.assertEqual(bridge._validate_summary(summary), summary)
        summary["undifferenced_doppler_factors"] = 0
        with self.assertRaises(bridge.BridgeError):
            bridge._validate_summary(summary)

    def test_keyed_csv_rejects_missing_duplicate_and_nonfinite(self) -> None:
        good = [SimpleNamespace(timestamp=1000, latitude=35.0, longitude=139.0)]
        content, stats = bridge._keyed_csv("route/phone", good, [1000])
        self.assertEqual(content.decode("ascii").splitlines()[0], ",".join(bridge.SUBMISSION_FIELDS))
        self.assertEqual(stats["missing_device_epochs"], 0)

        _, missing_stats = bridge._keyed_csv("route/phone", good, [1000, 2000])
        self.assertEqual(missing_stats["missing_device_epochs"], 1)
        duplicate = good + [SimpleNamespace(timestamp=1000, latitude=35.0, longitude=139.0)]
        with self.assertRaises(bridge.BridgeError):
            bridge._keyed_csv("route/phone", duplicate, [1000])
        nonfinite = [SimpleNamespace(timestamp=1000, latitude=math.nan, longitude=139.0)]
        with self.assertRaises(bridge.BridgeError):
            bridge._keyed_csv("route/phone", nonfinite, [1000])

    def test_fallback_requires_explicit_hash(self) -> None:
        with self.assertRaises(bridge.BridgeError):
            bridge._fallback_rows(ROOT / "does-not-exist.csv", None)


if __name__ == "__main__":
    unittest.main()
