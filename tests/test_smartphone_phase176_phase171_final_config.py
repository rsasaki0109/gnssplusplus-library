"""Launch-free regression for the Phase171 final GNSS-first config.

The Phase164 backend requires velocity-motion factors for its dedicated
Point3/V/C7/D recipe.  This test checks the application staging code after the
Phase171 branch and before processor construction, where a common default used
to overwrite that required setting.  It reads source only and launches no
native process or dataset.
"""

from __future__ import annotations

from pathlib import Path
import unittest


ROOT = Path(__file__).resolve().parents[1]
APP = ROOT / "apps/native/gnss_fgo_imu_no_base.cpp"
BACKEND = ROOT / "src/algorithms/fgo_gtsam_backend.cpp"


class Phase176FinalConfigTests(unittest.TestCase):
    def test_phase171_velocity_motion_recipe_survives_common_defaults(self) -> None:
        source = APP.read_text(encoding="utf-8")
        branch = source.index("} else if (phase171_imu_main) {")
        processor = source.index(
            "const libgnss::FGOProcessor gnss_first_processor(gnss_first_config);",
            branch,
        )
        staged = source[branch:processor]
        self.assertIn(
            "gnss_first_config.use_velocity_motion_factors = true;", staged
        )
        self.assertIn(
            "gnss_first_config.use_velocity_motion_factors = phase171_imu_main;",
            staged,
        )
        self.assertNotIn(
            "gnss_first_config.use_velocity_motion_factors = false;", staged
        )

    def test_phase164_guard_still_requires_velocity_motion_factors(self) -> None:
        source = BACKEND.read_text(encoding="utf-8")
        guard = source[source.index("if (use_native_raw_p_seed_graph)"):]
        guard = guard[: guard.index(
            "std::vector<std::size_t> pseudorange_rows_per_epoch"
        )]
        self.assertIn("!config.use_velocity_motion_factors", guard)
        self.assertIn(
            "Phase164 requires the complete Point3/V/C7/D raw-P ", source
        )


if __name__ == "__main__":
    unittest.main()
