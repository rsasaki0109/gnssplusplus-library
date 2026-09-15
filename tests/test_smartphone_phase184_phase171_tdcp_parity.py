"""Launch-free Phase184 source-TDCP parity contract tests.

The cached source is inspected as algorithm provenance only.  This test does
not open raw phone data, truth, MAT, trajectory, or native solver outputs.
"""

from __future__ import annotations

from pathlib import Path
import unittest

from frozen_contract import require_files


ROOT = Path(__file__).resolve().parents[1]
APP = ROOT / "apps/native/gnss_fgo_imu_no_base.cpp"
CONFIG = ROOT / "include/libgnss++/algorithms/fgo_config.hpp"
FGO = ROOT / "src/algorithms/fgo.cpp"
GTSAM = ROOT / "src/algorithms/fgo_gtsam_backend.cpp"
SOURCE_PARAMS = ROOT / "output/reproducibility-cache/gsdc2023/parameters.m"
SOURCE_GRAPH = ROOT / "output/reproducibility-cache/gsdc2023/fgo_gnss_imu.m"


class Phase184SourceTdcpParityTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls) -> None:
        cls.app = APP.read_text(encoding="utf-8")
        cls.config = CONFIG.read_text(encoding="utf-8")
        cls.fgo = FGO.read_text(encoding="utf-8")
        cls.gtsam = GTSAM.read_text(encoding="utf-8")

    def test_cached_street_contract_is_explicit(self) -> None:
        require_files("Phase184 reproducibility-cache sources", [SOURCE_PARAMS, SOURCE_GRAPH])
        params = SOURCE_PARAMS.read_text(encoding="utf-8")
        source_graph = SOURCE_GRAPH.read_text(encoding="utf-8")
        self.assertIn(
            'if setting.Type == "Street" || setting.Type == "Mix"',
            params,
        )
        street = params.index("prm.L_robust_prm = 0.2")
        street_block = params[street - 120 : street + 180]
        self.assertIn("prm.L_robust_prm = 0.2", street_block)
        self.assertIn("prm.L_kernel = huber(prm.L_robust_prm)", params)
        self.assertIn("noise_robust(prm.L_kernel, noise)", source_graph)

    def test_phase184_is_opt_in_and_separate_from_phase118(self) -> None:
        selector = "--native-phase184-source-tdcp-huber-k"
        self.assertIn(selector, self.app)
        self.assertIn("native_phase184_source_tdcp_huber_k = false", self.app)
        self.assertIn("options.native_phase184_source_tdcp_huber_k = true", self.app)
        self.assertIn(
            "use_native_phase184_source_tdcp_huber_k = false", self.config
        )
        self.assertIn("Phase184 source TDCP Huber-k cannot be combined with Phase118", self.fgo)
        self.assertIn("Phase184 source TDCP Huber-k cannot be combined with Phase117", self.fgo)
        self.assertIn("resolveOrdinaryTdcpHuberThresholdSigma", self.fgo)
        self.assertIn("resolveOrdinaryTdcpHuberThresholdSigma", self.gtsam)

    def test_phase171_staging_and_main_share_only_the_dedicated_mapping(self) -> None:
        branch = self.app.index("if (options.native_phase171_raw_p_no_doppler_imu_main) {")
        processor = self.app.index(
            "const libgnss::FGOProcessor processor(config);", branch
        )
        stage_processor = self.app.index(
            "const libgnss::FGOProcessor gnss_first_processor(gnss_first_config);",
            branch,
        )
        phase171 = self.app[branch:processor]
        staging = self.app[branch:stage_processor]
        self.assertIn(
            "if (options.native_phase184_source_tdcp_huber_k)", phase171
        )
        self.assertIn(
            "config.use_native_phase184_source_tdcp_huber_k = true;", phase171
        )
        self.assertIn(
            "gnss_first_config.use_native_raw_p_no_doppler_graph =\n"
            "                !phase171_ecef_doppler;",
            staging,
        )
        self.assertIn(
            "gnss_first_config.use_native_raw_p_ecef_doppler_gnss_first =\n"
            "                phase171_ecef_doppler;",
            staging,
        )
        self.assertIn(
            "gnss_first_config.use_native_phase167_raw_p_no_doppler_lm_termination_budget =\n                true;",
            staging,
        )
        self.assertNotIn(
            "gnss_first_config.use_native_phase184_source_tdcp_huber_k = false;",
            phase171,
        )
        self.assertIn("config.tdcp_sigma_m = kNativeTdcpSigmaM;", phase171)

    def test_legacy_threshold_remains_default_when_selector_is_off(self) -> None:
        self.assertIn("double tdcp_huber_threshold_sigma = 4.0;", self.config)
        self.assertIn(
            "threshold_sigma = config.tdcp_huber_threshold_sigma;", self.config
        )
        self.assertIn(
            "if (config.use_official_tdcp_huber_k &&\n        config.use_native_phase184_source_tdcp_huber_k)",
            self.config,
        )
        # The dedicated mapping is only reached behind its own bool; source
        # code must not silently turn it on from the Phase171 selector.
        self.assertIn(
            "if (config.use_native_phase184_source_tdcp_huber_k)", self.config
        )


if __name__ == "__main__":
    unittest.main()
