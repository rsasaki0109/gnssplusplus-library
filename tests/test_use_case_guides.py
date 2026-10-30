#!/usr/bin/env python3
"""Contract checks for the short, copy/paste-oriented use-case routes."""

import os
import re
import shlex
import subprocess
import tempfile
import unittest
from pathlib import Path


ROOT_DIR = Path(__file__).resolve().parents[1]
GUIDES = {
    "urban": ROOT_DIR / "docs" / "use_cases" / "urban_rtk_fgo.md",
    "rtklib": ROOT_DIR / "docs" / "use_cases" / "rtklib_migration.md",
    "ros2": ROOT_DIR / "docs" / "use_cases" / "ros2.md",
    "qzss": ROOT_DIR / "docs" / "use_cases" / "qzss_l6.md",
    "canyon": ROOT_DIR / "docs" / "use_cases" / "urban_canyon_rtk_options.md",
    "live": ROOT_DIR / "docs" / "use_cases" / "realtime_rtk_gnss_live.md",
}


def documented_live_command(guide: str) -> list[str]:
    """Return argv of the First output `gnss_live` command, exactly as documented.

    The fenced block is joined across backslash continuations and cut at the first
    shell pipe (the documented `| tee` only keeps a log copy).
    """
    for block in re.findall(r"```bash\n(.*?)```", guide, flags=re.S):
        if block.lstrip().startswith("./build/apps/gnss_live"):
            joined = re.sub(r"\\\n", " ", block).strip()
            argv = shlex.split(joined.split("|")[0])
            return argv
    raise AssertionError("no ./build/apps/gnss_live block in the guide")


def built_gnss_live() -> Path | None:
    build_dirs = [os.environ.get("GNSSPP_BUILD_DIR"), str(ROOT_DIR / "build")]
    for build_dir in build_dirs:
        if not build_dir:
            continue
        for candidate in (Path(build_dir) / "apps" / "gnss_live", Path(build_dir) / "gnss_live"):
            if candidate.is_file() and os.access(candidate, os.X_OK):
                return candidate
    return None


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

        canyon = GUIDES["canyon"].read_text(encoding="utf-8")
        for token in (
            "--rtk-canyon-carrier-prior",
            "--rtk-snr-weighting --rtk-snr-elevation-aware",
            "--rtk-snr-code-only",
            "python3 apps/gnss.py ppc-demo",
            "--max-epochs -1",
            "engaged_epochs=",
            "Do not combine the carrier prior with the SNR options",
            "urban_rtk_fgo.md",
            "../rtk_tuning_gates.md",
        ):
            self.assertIn(token, canyon)

        live = GUIDES["live"].read_text(encoding="utf-8")
        for token in (
            "--rover-rtcm",
            "--rover-ubx",
            "--base-rtcm",
            "ntrip://",
            "tcp://",
            "serial://",
            "--nav-rinex",
            "--base-ecef",
            "--no-rtcm-epoch-assembly",
            "--reconnect",
            "--stall-timeout-ms 5000",
            "--base-outage-fallback spp",
            "--out output/use_cases/live/first.pos --format pos",
            "--pos-uncertainty",
            "--pos-fix-diagnostics",
            "--preset low-cost",
            "--max-pos-jump-min 5 --max-pos-jump-rate 30",
            "--kinematic-post-filter",
            "tests/fixtures/realtime_stress",
            "written_solutions=201",
            "fixed_solutions=197",
            "fallback_spp_epochs=0",
            "python3 apps/gnss.py realtime-stress prepare",
            "python3 apps/gnss.py realtime-stress run",
            "base_outage@",
            "base_blackhole@",
            "Wrong FIX on the real-time path",
            "wrong-fix-on-the-real-time-path",
            "FloatFixH",
            "not an integrity guarantee",
            "not calibrated",
            "../realtime_stress.md",
            "urban_canyon_rtk_options.md",
            "../rtk_wrong_fix_holdout.md",
            "../interfaces.md#stall-watchdog-for-gnss-live-stall-timeout-ms",
        ):
            self.assertIn(token, live)

    def test_tokyo_ppc_lever_arm_is_passed_in_body_flu(self) -> None:
        """`--lever-arm` is body FLU; the PPC README's Tokyo value is FRD."""
        contract_doc = (ROOT_DIR / "docs" / "ppc_reproduction.md").read_text(encoding="utf-8")
        self.assertIn("`[0.31, 0.0, -0.55]`", contract_doc)

        urban = GUIDES["urban"].read_text(encoding="utf-8")
        imu_fusion = (ROOT_DIR / "docs" / "imu_fusion.md").read_text(encoding="utf-8")
        example_config = (ROOT_DIR / "configs" / "examples" / "fuse.example.toml").read_text(
            encoding="utf-8"
        )
        self.assertIn("--lever-arm 0.31,0,0.55", urban)
        self.assertIn("`0.31,0,0.55`", urban)
        self.assertIn("ppc_rtk_vs_gnss_imu_fusion.md#ppc-lever-arm-convention", urban)
        self.assertIn("--lever-arm 0.31,0,0.55", imu_fusion)
        self.assertIn("lever_arm = [0.31, 0.0, 0.55]", example_config)
        self.assertNotIn("--lever-arm 0.31,0,-0.55", urban)
        self.assertNotIn("--lever-arm 0.31,0,-0.55", imu_fusion)

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
        self.assertIn("docs/use_cases/realtime_rtk_gnss_live.md", readme)
        self.assertIn("[Use cases](use_cases.md)", index)
        self.assertIn("use_cases/urban_rtk_imu_field_checklist.md", index)
        self.assertIn("use_cases/urban_rtk_imu_field_checklist.md", overview)
        self.assertIn("  - Guides:", nav)
        for relative_path in (
            "use_cases.md",
            "use_cases/urban_rtk_fgo.md",
            "use_cases/urban_rtk_imu_field_checklist.md",
            "use_cases/rtklib_migration.md",
            "use_cases/ros2.md",
            "use_cases/qzss_l6.md",
            "use_cases/urban_canyon_rtk_options.md",
            "use_cases/realtime_rtk_gnss_live.md",
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

    def test_live_guide_relative_links_and_anchors_resolve(self) -> None:
        """No mkdocs here: check the guide's relative links and #anchors by hand."""

        def slug(heading: str) -> str:
            # Python-Markdown `toc` slugify: drop punctuation, collapse runs of -/space.
            heading = re.sub(r"[^\w\s-]", "", heading.strip().lower())
            return re.sub(r"[-\s]+", "-", heading)

        def anchors(path: Path) -> set[str]:
            found = set()
            in_fence = False
            for line in path.read_text(encoding="utf-8").splitlines():
                if line.startswith("```"):
                    in_fence = not in_fence
                elif not in_fence:
                    match = re.match(r"#{1,6}\s+(.*)", line)
                    if match:
                        found.add(slug(match.group(1)))
            return found

        guide_path = GUIDES["live"]
        text = guide_path.read_text(encoding="utf-8")
        text = re.sub(r"```.*?```", "", text, flags=re.S)
        links = re.findall(r"\]\(([^)\s]+)\)", text)
        self.assertGreater(len(links), 20)
        for link in links:
            if link.startswith(("http://", "https://", "mailto:")):
                continue
            target, _, fragment = link.partition("#")
            path = (guide_path.parent / target).resolve() if target else guide_path
            self.assertTrue(path.is_file(), link)
            if fragment:
                self.assertIn(fragment, anchors(path), link)

    def test_live_guide_command_runs_on_the_tracked_fixture(self) -> None:
        """Run the documented First output command and check what the guide says it writes."""
        live_binary = built_gnss_live()
        if live_binary is None:
            self.skipTest("gnss_live is not built (cmake --build build --target gnss_live)")
        guide = GUIDES["live"].read_text(encoding="utf-8")
        argv = documented_live_command(guide)
        self.assertEqual(argv[0], "./build/apps/gnss_live")
        argv[0] = str(live_binary)
        for flag in ("--rover-rtcm", "--base-rtcm"):
            self.assertTrue((ROOT_DIR / argv[argv.index(flag) + 1]).is_file(), flag)

        with tempfile.TemporaryDirectory(prefix="gnsspp_live_guide_") as temp_dir:
            out_path = Path(temp_dir) / "first.pos"
            argv[argv.index("--out") + 1] = str(out_path)
            result = subprocess.run(
                argv, cwd=ROOT_DIR, capture_output=True, text=True, timeout=120, check=False
            )
            self.assertEqual(result.returncode, 0, result.stdout + result.stderr)
            summary = next(
                (line for line in result.stdout.splitlines() if line.startswith("summary:")), ""
            )
            self.assertTrue(summary, result.stdout + result.stderr)
            fields = dict(token.split("=", 1) for token in summary.split()[1:] if "=" in token)
            self.assertEqual(fields["termination"], "rover_source_exhausted")
            # The guide quotes these; keep it honest.
            for name in ("written_solutions", "fixed_solutions"):
                self.assertIn(f"{name}={fields[name]}", guide, name)
            for name in (
                "skipped_rover_epochs",
                "fallback_spp_epochs",
                "rover_stalls",
                "base_stalls",
                "rover_decoder_errors",
                "base_decoder_errors",
            ):
                self.assertEqual(fields[name], "0", name)

            lines = out_path.read_text(encoding="utf-8").splitlines()
            header = next(line for line in lines if line.startswith("% GPS_Week")).split()[1:]
            for column in (
                "Status",
                "sdn(m)",
                "sde(m)",
                "sdu(m)",
                "DiffAge(s)",
                "FloatFixH(m)",
                "FloatFixU(m)",
            ):
                self.assertIn(column, header)
            self.assertEqual(header[-2:], ["FloatFixH(m)", "FloatFixU(m)"])
            status_column = header.index("Status")
            rows = [line.split() for line in lines if not line.startswith("%") and line.strip()]
            self.assertEqual(len(rows), int(fields["written_solutions"]))
            self.assertEqual(len(rows), 201)
            for row in rows:
                self.assertEqual(len(row), len(header))
            fixed = [row for row in rows if row[status_column] == "4"]
            self.assertEqual(len(fixed), int(fields["fixed_solutions"]))
            for row in rows:
                if row[status_column] == "4":
                    self.assertGreaterEqual(float(row[-2]), 0.0)
                    self.assertGreaterEqual(float(row[-1]), 0.0)
                else:
                    self.assertEqual(row[-2:], ["nan", "nan"])
            self.assertFalse([row for row in rows if row[status_column] == "1"])

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
