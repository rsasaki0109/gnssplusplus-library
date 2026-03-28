#!/usr/bin/env python3
"""Smoke tests for install/export packaging."""

from __future__ import annotations

import os
import shutil
import subprocess
import tempfile
import unittest
from pathlib import Path


ROOT_DIR = Path(__file__).resolve().parents[1]
BUILD_DIR = ROOT_DIR / "build"


def repo_data_exists(*relative_paths: str) -> bool:
    return all((ROOT_DIR / relative_path).exists() for relative_path in relative_paths)


class PackagingSmokeTest(unittest.TestCase):
    def test_cmake_install_exports_expected_layout(self) -> None:
        self.assertTrue(BUILD_DIR.exists(), "build directory must exist before packaging test")

        with tempfile.TemporaryDirectory(prefix="gnss_install_") as temp_dir:
            prefix = Path(temp_dir) / "prefix"
            subprocess.run(
                ["cmake", "--install", str(BUILD_DIR), "--prefix", str(prefix)],
                check=True,
                cwd=ROOT_DIR,
            )

            expected_paths = [
                prefix / "bin" / "gnss",
                prefix / "bin" / "gnss_spp",
                prefix / "bin" / "gnss_solve",
                prefix / "bin" / "gnss_ppp",
                prefix / "bin" / "gnss_nav_products",
                prefix / "bin" / "gnss_rcv.py",
                prefix / "bin" / "gnss_web.py",
                prefix / "bin" / "gnss_rtk_kinematic_signoff.py",
                prefix / "bin" / "gnss_ppp_static_signoff.py",
                prefix / "bin" / "gnss_ppp_kinematic_signoff.py",
                prefix / "bin" / "gnss_ppc_demo.py",
                prefix / "bin" / "gnss_clas_ppp.py",
                prefix / "bin" / "gnss_sbp_info.py",
                prefix / "bin" / "gnss_sbf_info.py",
                prefix / "bin" / "gnss_trimble_info.py",
                prefix / "bin" / "gnss_skytraq_info.py",
                prefix / "bin" / "gnss_binex_info.py",
                prefix / "bin" / "gnss_qzss_l6_info.py",
                prefix / "include" / "libgnss++" / "gnss.hpp",
                prefix / "lib" / "libgnss_lib.a",
                prefix / "lib" / "libgnss_lib_noopt.a",
                prefix / "lib" / "cmake" / "libgnsspp" / "libgnssppConfig.cmake",
                prefix / "lib" / "pkgconfig" / "libgnsspp.pc",
                prefix / "tools" / "rtk_stats.py",
                prefix / "scripts" / "generate_driving_comparison.py",
                prefix / "scripts" / "generate_feature_overview_card.py",
                prefix / "scripts" / "generate_odaiba_social_card.py",
                prefix / "configs" / "live.example.conf",
            ]
            ros2_binary = next((path for path in BUILD_DIR.rglob("gnss_solution_node") if path.is_file()), None)
            if ros2_binary is not None:
                expected_paths.append(prefix / "bin" / "gnss_solution_node")
            for path in expected_paths:
                self.assertTrue(path.exists(), f"missing installed artifact: {path}")

            pc_contents = (prefix / "lib" / "pkgconfig" / "libgnsspp.pc").read_text(encoding="utf-8")
            self.assertIn("Libs: -L${libdir} -lgnss_lib -lgnss_lib_noopt", pc_contents)
            self.assertIn("Cflags: -I${includedir}", pc_contents)

            config_contents = (
                prefix / "lib" / "cmake" / "libgnsspp" / "libgnssppConfig.cmake"
            ).read_text(encoding="utf-8")
            self.assertIn("find_dependency(Eigen3 REQUIRED)", config_contents)
            self.assertIn("find_dependency(Threads REQUIRED)", config_contents)

            python_packages = list(prefix.rglob("libgnsspp/__init__.py"))
            self.assertTrue(python_packages, "missing installed Python package libgnsspp")
            python_package_dir = python_packages[0].parent
            extension_modules = list(python_package_dir.glob("_libgnsspp*.so"))
            self.assertTrue(extension_modules, "missing installed Python extension module")

            env = dict(os.environ)
            env["PATH"] = str(prefix / "bin") + os.pathsep + env.get("PATH", "")

            if repo_data_exists(
                "data/short_baseline/TSK200JPN_R_20240010000_01D_30S_MO.rnx",
                "data/short_baseline/TSKB00JPN_R_20240010000_01D_30S_MO.rnx",
                "data/short_baseline/BRDC00IGS_R_20240010000_01D_MN.rnx",
            ):
                short_baseline_out = prefix / "tmp_short_baseline.pos"
                short_baseline_summary = prefix / "tmp_short_baseline.json"
                subprocess.run(
                    [
                        str(prefix / "bin" / "gnss"),
                        "short-baseline-signoff",
                        "--rover",
                        str(ROOT_DIR / "data/short_baseline/TSK200JPN_R_20240010000_01D_30S_MO.rnx"),
                        "--base",
                        str(ROOT_DIR / "data/short_baseline/TSKB00JPN_R_20240010000_01D_30S_MO.rnx"),
                        "--nav",
                        str(ROOT_DIR / "data/short_baseline/BRDC00IGS_R_20240010000_01D_MN.rnx"),
                        "--out",
                        str(short_baseline_out),
                        "--summary-json",
                        str(short_baseline_summary),
                        "--max-epochs",
                        "5",
                    ],
                    check=True,
                    cwd=ROOT_DIR,
                    env=env,
                )
                self.assertTrue(short_baseline_out.exists(), "installed short-baseline signoff did not write .pos")
                self.assertTrue(short_baseline_summary.exists(), "installed short-baseline signoff did not write summary")

            if repo_data_exists("data/rover_static.obs", "data/navigation_static.nav"):
                ppp_out = prefix / "tmp_ppp_static.pos"
                ppp_summary = prefix / "tmp_ppp_static.json"
                subprocess.run(
                    [
                        str(prefix / "bin" / "gnss"),
                        "ppp-static-signoff",
                        "--obs",
                        str(ROOT_DIR / "data/rover_static.obs"),
                        "--nav",
                        str(ROOT_DIR / "data/navigation_static.nav"),
                        "--out",
                        str(ppp_out),
                        "--summary-json",
                        str(ppp_summary),
                        "--max-epochs",
                        "5",
                    ],
                    check=True,
                    cwd=ROOT_DIR,
                    env=env,
                )
                self.assertTrue(ppp_out.exists(), "installed PPP static signoff did not write .pos")
                self.assertTrue(ppp_summary.exists(), "installed PPP static signoff did not write summary")

            if repo_data_exists(
                "output/rtk_solution.pos",
                "output/driving_rtklib_rtk.pos",
                "data/driving/Tokyo_Data/Odaiba/reference.csv",
            ):
                social_card_png = prefix / "tmp_social_card.png"
                subprocess.run(
                    [
                        str(prefix / "bin" / "gnss"),
                        "social-card",
                        "--lib-pos",
                        str(ROOT_DIR / "output" / "rtk_solution.pos"),
                        "--rtklib-pos",
                        str(ROOT_DIR / "output" / "driving_rtklib_rtk.pos"),
                        "--reference-csv",
                        str(ROOT_DIR / "data" / "driving" / "Tokyo_Data" / "Odaiba" / "reference.csv"),
                        "--output",
                        str(social_card_png),
                    ],
                    check=True,
                    cwd=ROOT_DIR,
                    env=env,
                )
                self.assertTrue(social_card_png.exists(), "installed social-card did not write PNG")

            feature_overview_png = prefix / "tmp_feature_overview.png"
            subprocess.run(
                [
                    "python3",
                    str(prefix / "scripts" / "generate_feature_overview_card.py"),
                    "--output",
                    str(feature_overview_png),
                ],
                check=True,
                cwd=ROOT_DIR,
                env=env,
            )
            self.assertTrue(feature_overview_png.exists(), "installed feature overview generator did not write PNG")

            web_help = subprocess.run(
                [str(prefix / "bin" / "gnss"), "web", "--help"],
                check=True,
                cwd=ROOT_DIR,
                env=env,
                capture_output=True,
                text=True,
            )
            self.assertIn("local web UI", web_help.stdout)


if __name__ == "__main__":
    unittest.main()
