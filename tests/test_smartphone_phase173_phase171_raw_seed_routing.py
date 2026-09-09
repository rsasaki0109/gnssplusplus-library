"""Launch-free regression for Phase171 raw-P selector routing.

This test reads only the application source and the sealed Phase172 manifest.
It does not open route payloads, solution output, truth, MAT/PDC artifacts, or
start a native process.
"""

from __future__ import annotations

import json
from pathlib import Path
import unittest


ROOT = Path(__file__).resolve().parents[1]
APP = ROOT / "apps/native/gnss_fgo_imu_no_base.cpp"
PHASE172_MANIFEST = ROOT / (
    "docs/use_cases/records/"
    "smartphone_r5_phase172_h_native_raw_p_no_doppler_imu_main_manifest_v1.json"
)


class Phase173RawSeedRoutingTests(unittest.TestCase):
    def test_phase171_still_executes_shared_raw_seed_stage(self) -> None:
        source = APP.read_text(encoding="utf-8")
        adapter_start = source.index(
            "RawPNoDopplerSeedAdapterResult phase165_adapter"
        )
        adapter_end = source.index(
            "if (options.native_phase149_raw_p_seed_stage)", adapter_start
        )
        adapter = source[adapter_start:adapter_end]
        self.assertIn(
            "if (options.native_phase165_raw_p_no_doppler_graph) {", adapter
        )
        self.assertNotIn(
            "native_phase165_raw_p_no_doppler_graph &&\n"
            "        !options.native_phase171_raw_p_no_doppler_imu_main",
            adapter,
        )
        self.assertIn("raw_p_seed::solve(epochs, nav, seed_config)", adapter)
        self.assertIn("adaptSameRunNoDopplerSeeds", adapter)

    def test_phase171_suppresses_only_terminal_phase165_graph(self) -> None:
        source = APP.read_text(encoding="utf-8")
        rekey_start = source.index(
            "std::vector<libgnss::raw_p_seed::RawPNoDopplerSeed> retained_seeds"
        )
        rekey_end = source.index(
            "if (options.native_base_pseudorange_compensation)", rekey_start
        )
        rekey = source[rekey_start:rekey_end]
        self.assertIn(
            "if (options.native_phase171_raw_p_no_doppler_imu_main) {", rekey
        )
        self.assertIn(
            "Phase171 consumes this exact retained-key seed vector", rekey
        )
        self.assertIn("} else {", rekey)
        self.assertIn("processor.optimizeProblem(problem)", rekey)
        self.assertLess(
            rekey.index("if (options.native_phase171_raw_p_no_doppler_imu_main)"),
            rekey.index("processor.optimizeProblem(problem)"),
        )

    def test_phase172_pin_requires_shared_stage_and_phase171_main(self) -> None:
        manifest = json.loads(PHASE172_MANIFEST.read_text(encoding="utf-8"))
        argv = manifest["argv"]
        self.assertEqual(manifest["phase"], 172)
        for flag in (
            "--native-phase157-raw-p-bootstrap",
            "--native-phase165-raw-p-no-doppler-graph",
            "--native-phase167-raw-p-no-doppler-lm-termination-budget",
            "--native-phase171-raw-p-no-doppler-imu-main",
        ):
            self.assertEqual(argv.count(flag), 1, flag)
        self.assertFalse(manifest["selector_contract"]["phase159_collect_all_epochs"])
        self.assertTrue(manifest["selector_contract"]["phase171_same_run_imu_main"])
        self.assertFalse(manifest["output_policy"]["candidate_output_reused_as_solver_input"])


if __name__ == "__main__":
    unittest.main()
