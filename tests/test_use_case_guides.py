#!/usr/bin/env python3
"""Contract checks for the short, copy/paste-oriented use-case routes."""

import re
import unittest
from pathlib import Path


ROOT_DIR = Path(__file__).resolve().parents[1]
GUIDES = {
    "urban": ROOT_DIR / "docs" / "use_cases" / "urban_rtk_fgo.md",
    "rtklib": ROOT_DIR / "docs" / "use_cases" / "rtklib_migration.md",
    "ros2": ROOT_DIR / "docs" / "use_cases" / "ros2.md",
    "qzss": ROOT_DIR / "docs" / "use_cases" / "qzss_l6.md",
}


class UseCaseGuideTest(unittest.TestCase):
    def test_routes_and_required_sections(self) -> None:
        overview = ROOT_DIR / "docs" / "use_cases.md"
        self.assertTrue(overview.is_file())
        overview_text = overview.read_text(encoding="utf-8")
        for guide_key, guide_path in GUIDES.items():
            self.assertTrue(guide_path.is_file(), guide_key)
            self.assertIn(f"use_cases/{guide_path.name}", overview_text)

            guide = guide_path.read_text(encoding="utf-8")
            self.assertRegex(guide, r"```bash")
            self.assertRegex(guide.lower(), r"first (output|artifacts?)")
            self.assertRegex(guide.lower(), r"exit criteria")
            self.assertRegex(guide.lower(), r"boundary|does not")
            self.assertIn("Next step", guide)

    def test_commands_are_the_existing_route_surfaces(self) -> None:
        urban = GUIDES["urban"].read_text(encoding="utf-8")
        for token in (
            "python3 apps/gnss.py fuse",
            "--navi776-tc",
            "--rtk-pos-out",
            "--gnss-pos",
            "--zupt --no-nhc",
            "--zupt --nhc",
            "python3 apps/gnss.py trackplot",
            "python3 apps/gnss.py pos2kml",
            "python3 apps/gnss.py urban-bridge-score",
            "python3 apps/gnss.py urban-continuity-bundle",
            "--segments-csv",
            "--status all",
            "reference.csv",
            "manifest.json",
            "fused_trajectory.png",
            "../fgo_nhc_zupt_fix_rate_audit.md",
        ):
            self.assertIn(token, urban)

        rtklib = GUIDES["rtklib"].read_text(encoding="utf-8")
        for token in (
            "python3 apps/gnss.py solve",
            "--config configs/examples/solve.example.toml",
            "--rover",
            "--base",
            "--nav",
            "--out",
            "scripts/convert_rtklib_pos.py",
            "build/apps/gnss_solve",
            "../robotics_quickstart.md",
        ):
            self.assertIn(token, rtklib)

        ros2 = GUIDES["ros2"].read_text(encoding="utf-8")
        for token in (
            "ros2-doctor",
            "ros2-bag-doctor",
            "colcon build --symlink-install --packages-select gnss_raw_driver",
            "ros2 bag play",
            "ros2 run gnss_raw_driver gnss_bag_processor_node",
        ):
            self.assertIn(token, ros2)

        qzss = GUIDES["qzss"].read_text(encoding="utf-8")
        for token in (
            "qzss-l6-info",
            "--extract-data-parts",
            "--extract-subframes",
            "clas-ppp",
            "--qzss-l6",
            "--qzss-gps-week",
            "--madoca-l6",
            "--madoca-l6d",
            "--ar-method per-freq",
            "build/apps/gnss_ppp",
            "../robotics_quickstart.md",
        ):
            self.assertIn(token, qzss)

    def test_tokyo_ppc_lever_arm_preserves_the_published_signed_z(self) -> None:
        """The urban recipe must not silently flip Tokyo's vertical lever arm."""
        contract_doc = (ROOT_DIR / "docs" / "ppc_reproduction.md").read_text(encoding="utf-8")
        self.assertIn("`[0.31, 0.0, -0.55]`", contract_doc)

        urban = GUIDES["urban"].read_text(encoding="utf-8")
        imu_fusion = (ROOT_DIR / "docs" / "imu_fusion.md").read_text(encoding="utf-8")
        example_config = (ROOT_DIR / "configs" / "examples" / "fuse.example.toml").read_text(
            encoding="utf-8"
        )
        self.assertIn("--lever-arm 0.31,0,-0.55", urban)
        self.assertIn("`0.31,0,-0.55`", urban)
        self.assertIn("--lever-arm 0.31,0,-0.55", imu_fusion)
        self.assertIn("lever_arm = [0.31, 0.0, -0.55]", example_config)
        self.assertNotIn("0.31,0,0.55", urban)
        self.assertNotIn("0.31,0,0.55", imu_fusion)

    def test_referenced_small_fixtures_exist(self) -> None:
        for relative_path in (
            "configs/examples/fuse.example.toml",
            "configs/examples/solve.example.toml",
            "scripts/convert_rtklib_pos.py",
            "ros2/gnss_raw_driver/README.md",
            "tests/cli_cases/_support.py",
        ):
            self.assertTrue((ROOT_DIR / relative_path).is_file(), relative_path)

    def test_native_build_and_ros2_shell_contracts_are_documented(self) -> None:
        rtklib = GUIDES["rtklib"].read_text(encoding="utf-8")
        qzss = GUIDES["qzss"].read_text(encoding="utf-8")
        ros2 = GUIDES["ros2"].read_text(encoding="utf-8")
        self.assertIn("cmake --build build --target gnss_solve", rtklib)
        self.assertIn("test -x build/apps/gnss_solve", rtklib)
        self.assertIn("cmake --build build --target gnss_ppp", qzss)
        self.assertIn("test -x build/apps/gnss_ppp", qzss)
        self.assertIn("source install/setup.bash", ros2)
        self.assertIn("source ros2/install/setup.bash", ros2)
        self.assertIn("git rev-parse --show-toplevel", ros2)

    def test_entrypoint_and_mkdocs_links(self) -> None:
        readme = (ROOT_DIR / "README.md").read_text(encoding="utf-8")
        index = (ROOT_DIR / "docs" / "index.md").read_text(encoding="utf-8")
        overview = (ROOT_DIR / "docs" / "use_cases.md").read_text(encoding="utf-8")
        nav = (ROOT_DIR / "mkdocs.yml").read_text(encoding="utf-8")

        self.assertIn("docs/use_cases.md", readme)
        self.assertIn("docs/use_cases/urban_rtk_fgo.md", readme)
        self.assertIn("docs/use_cases/rtklib_migration.md", readme)
        self.assertIn("docs/use_cases/ros2.md", readme)
        self.assertIn("docs/use_cases/qzss_l6.md", readme)
        self.assertIn("[Use cases](use_cases.md)", index)
        self.assertIn("use_cases/urban_rtk_imu_field_checklist.md", index)
        self.assertIn("use_cases/urban_rtk_imu_field_checklist.md", overview)
        self.assertIn("  - Use cases:", nav)
        for relative_path in (
            "use_cases.md",
            "use_cases/urban_rtk_fgo.md",
            "use_cases/urban_rtk_imu_field_checklist.md",
            "use_cases/rtklib_migration.md",
            "use_cases/ros2.md",
            "use_cases/qzss_l6.md",
        ):
            self.assertIn(relative_path, nav)

    def test_r1_field_checklist_and_holdout_decision_are_linked(self) -> None:
        checklist_path = ROOT_DIR / "docs" / "use_cases" / "urban_rtk_imu_field_checklist.md"
        self.assertTrue(checklist_path.is_file())
        checklist = checklist_path.read_text(encoding="utf-8")
        urban = GUIDES["urban"].read_text(encoding="utf-8")
        for token in (
            "R1-09",
            "urban-continuity-bundle",
            "fused.kml",
            "usable",
            "degraded",
            "unusable",
            "60 s",
            "r1_holdout_run2",
            "r1_holdout_run3",
            "Do **not** rerun",
        ):
            self.assertIn(token, checklist)
        for token in (
            "urban_rtk_imu_field_checklist.md",
            "R1 frozen release decision and holdout evidence",
            "r1_holdout_run2",
            "r1_holdout_run3",
            "do not tune or rerun either holdout",
        ):
            self.assertIn(token, urban)

    def test_guides_link_to_interfaces_validation_and_benchmarks(self) -> None:
        for guide_path in GUIDES.values():
            guide = guide_path.read_text(encoding="utf-8")
            for relative_path in ("../interfaces.md", "../validation.md", "../benchmarks.md"):
                self.assertIn(relative_path, guide, guide_path.name)

    def test_positive_overclaim_tokens_are_absent(self) -> None:
        forbidden = (
            r"\bfully compatible\b",
            r"\bcomplete compatibility\b",
            r"\buniversally superior\b",
            r"\bfull coverage\b",
            r"\bguarantees? production (?:real[- ]time|field)",
        )
        for guide_path in GUIDES.values():
            guide = guide_path.read_text(encoding="utf-8").lower()
            for token in forbidden:
                self.assertIsNone(re.search(token, guide), f"{token}: {guide_path.name}")


if __name__ == "__main__":
    unittest.main()
