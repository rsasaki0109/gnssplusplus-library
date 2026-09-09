"""Structural and no-input CLI tests for the Phase80 direct quality lane."""

from __future__ import annotations

import hashlib
import json
import os
from pathlib import Path
import subprocess
import unittest


ROOT = Path(__file__).resolve().parents[1]
APP = ROOT / "apps/native/gnss_fgo_imu_no_base.cpp"
CONFIG = ROOT / "include/libgnss++/algorithms/fgo_config.hpp"
GTSAM_BACKEND = ROOT / "src/algorithms/fgo_gtsam_backend.cpp"
EIGEN_BACKEND = ROOT / "src/algorithms/fgo.cpp"
FREEZE = ROOT / "docs/use_cases/records/smartphone_r5_phase80_source_exact_direct_p_quality_freeze_v1.json"
EXPECTED_FREEZE_SHA256 = "9ebdde21fe7fd955029243d3bdbba3a9f9d3832388d67480760394177c8f685e"

ROUTES = {
    "2021-03-16-18-59-us-ca-mtv-a/pixel5": ("Highway", 0.2, 0.8),
    "2021-08-24-20-32-us-ca-mtv-h/pixel5": ("Street", 0.1, 0.4),
    "2022-04-01-18-22-us-ca-lax-t/pixel5": ("Highway", 0.2, 0.8),
    "2023-03-08-21-34-us-ca-mtv-u/pixel5": ("Street", 0.1, 0.4),
}


class Phase80DirectObservableQualityTests(unittest.TestCase):
    def test_freeze_is_sealed_with_exact_route_huber_mapping(self) -> None:
        self.assertEqual(hashlib.sha256(FREEZE.read_bytes()).hexdigest(), EXPECTED_FREEZE_SHA256)
        freeze = json.loads(FREEZE.read_text(encoding="utf-8"))
        candidate = freeze["candidate"]
        self.assertTrue(candidate["opt_in"])
        self.assertTrue(candidate["default_off"])
        self.assertTrue(candidate["direct_no_pdc"])
        self.assertFalse(candidate["pdc_state_bridge"])
        self.assertEqual(candidate["source_quality_flag"], "--native-source-direct-observable-quality")
        self.assertEqual(candidate["direct_flag_requires"], ["--native-pdc-imu-tdcp-no-bridge"])
        contract = freeze["source_quality_contract"]
        self.assertEqual(contract["doppler_model"], "SNR")
        self.assertEqual(contract["D_sn_ratio"], "1/12")
        self.assertEqual(contract["Dmask_res_mps"], 3.0)
        self.assertFalse(contract["doppler_residual_screen_applies_to_initialization"])
        self.assertTrue(contract["whole_contract_includes_p_and_d"])
        by_route = freeze["public_settings_huber"]["by_route"]
        self.assertEqual(set(by_route), set(ROUTES))
        for route, expected in ROUTES.items():
            self.assertEqual(
                (by_route[route]["environment"],
                 by_route[route]["pseudorange_huber_threshold_sigma"],
                 by_route[route]["doppler_huber_threshold_sigma"]),
                expected,
            )

    def test_direct_flag_is_parsed_and_bypasses_legacy_pdc_branch(self) -> None:
        source = APP.read_text(encoding="utf-8")
        self.assertIn("--native-source-direct-observable-quality", source)
        self.assertIn("native_source_direct_observable_quality = true", source)
        self.assertIn("requires ", source)
        self.assertIn("--native-pdc-imu-tdcp-no-bridge", source)
        self.assertIn("conflicts with ", source)
        self.assertIn("--native-upstream-quality", source)
        self.assertIn("if (dataset_id != entry.route_id)", source)

        bridge_anchor = source.index("NativePdcBridgeReport pdc_bridge_report;")
        bridge_start = source.index("if (options.native_upstream_quality) {", bridge_anchor)
        bridge_end = source.index("} else {", bridge_start)
        self.assertNotIn("native_source_direct_observable_quality", source[bridge_start:bridge_end])

        config_anchor = source.index("libgnss::FGOProcessor::FGOConfig config;")
        direct_start = source.index(
            "if (options.native_source_direct_observable_quality) {", config_anchor
        )
        direct_end = source.index(
            "if (options.native_upstream_absolute_doppler_screen)", direct_start
        )
        direct_block = source[direct_start:direct_end]
        self.assertIn("config.use_upstream_observable_quality = true", direct_block)
        self.assertIn("config.pseudorange_huber_threshold_sigma", direct_block)
        self.assertIn("config.undifferenced_doppler_huber_threshold_sigma", direct_block)
        self.assertNotIn("use_native_pdc_state_bridge = true", direct_block)

    def test_doppler_robust_threshold_is_separate_from_tdcp(self) -> None:
        config = CONFIG.read_text(encoding="utf-8")
        gtsam = GTSAM_BACKEND.read_text(encoding="utf-8")
        eigen = EIGEN_BACKEND.read_text(encoding="utf-8")
        self.assertIn("undifferenced_doppler_huber_threshold_sigma = 4.0", config)
        self.assertIn("config.undifferenced_doppler_huber_threshold_sigma", gtsam)
        self.assertIn("config_.undifferenced_doppler_huber_threshold_sigma", eigen)
        self.assertIn("config.tdcp_huber_threshold_sigma", gtsam)

    def test_cli_help_advertises_flag_without_opening_route_inputs(self) -> None:
        binary_dir = Path(os.environ.get("GNSSPP_BINARY_DIR", ROOT / "build"))
        binary = binary_dir / "apps/gnss_fgo_imu_no_base"
        if not binary.exists():
            self.skipTest(f"Release binary not present: {binary}")
        environment = os.environ.copy()
        gtsam_library_dir = Path("/home/sasaki/.local/lib")
        if gtsam_library_dir.is_dir():
            existing_library_path = environment.get("LD_LIBRARY_PATH", "")
            environment["LD_LIBRARY_PATH"] = ":".join(
                part for part in (str(gtsam_library_dir), existing_library_path) if part
            )
        completed = subprocess.run(
            [str(binary), "--help"],
            cwd=ROOT,
            check=False,
            capture_output=True,
            text=True,
            env=environment,
        )
        if completed.returncode == 127:
            self.skipTest(f"CLI dependencies unavailable for {binary}: {completed.stderr}")
        self.assertIn("--native-source-direct-observable-quality", completed.stdout)
        self.assertNotIn("fewer than two observation epochs", completed.stderr)

    def test_summary_telemetry_has_one_distinct_direct_object(self) -> None:
        source = APP.read_text(encoding="utf-8")
        escaped_object = r'\"native_source_direct_observable_quality\": {'
        self.assertEqual(source.count(escaped_object), 1)
        self.assertIn(r'\"native_source_direct_observable_quality_enabled\":', source)
        for key in (
            r'\"enabled\": true',
            r'\"direct_no_pdc\": true',
            r'\"pdc_bridge\": false',
            r'\"environment\":',
            r'\"p_and_d_huber\":',
            r'\"config\":',
            r'\"counts\":',
        ):
            self.assertIn(key, source)


if __name__ == "__main__":
    unittest.main()
