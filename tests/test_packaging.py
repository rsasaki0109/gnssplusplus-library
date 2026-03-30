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
    def test_docker_files_exist_and_look_like_runtime_packaging(self) -> None:
        dockerfile = ROOT_DIR / "Dockerfile"
        dockerignore = ROOT_DIR / ".dockerignore"
        compose_file = ROOT_DIR / "compose.yaml"
        docker_workflow = ROOT_DIR / ".github" / "workflows" / "docker.yml"

        self.assertTrue(dockerfile.exists(), "missing Dockerfile")
        self.assertTrue(dockerignore.exists(), "missing .dockerignore")
        self.assertTrue(compose_file.exists(), "missing compose.yaml")
        self.assertTrue(docker_workflow.exists(), "missing docker workflow")

        dockerfile_text = dockerfile.read_text(encoding="utf-8")
        self.assertIn("FROM ubuntu:24.04 AS builder", dockerfile_text)
        self.assertIn("-DBUILD_TESTING=OFF", dockerfile_text)
        self.assertIn("cmake --install build --prefix /opt/libgnsspp", dockerfile_text)
        self.assertIn("ENTRYPOINT [\"gnss\"]", dockerfile_text)
        self.assertIn("PYTHONPATH=/opt/libgnsspp/lib/python3/site-packages", dockerfile_text)

        dockerignore_text = dockerignore.read_text(encoding="utf-8")
        self.assertIn("build", dockerignore_text)
        self.assertIn("output", dockerignore_text)
        self.assertIn("data", dockerignore_text)

        compose_text = compose_file.read_text(encoding="utf-8")
        self.assertIn("gnss-web", compose_text)
        self.assertIn("ghcr.io/rsasaki0109/gnssplusplus-library:develop", compose_text)
        self.assertIn("8085:8085", compose_text)

        docker_workflow_text = docker_workflow.read_text(encoding="utf-8")
        self.assertIn("ghcr.io/rsasaki0109/gnssplusplus-library", docker_workflow_text)
        self.assertIn("docker/build-push-action", docker_workflow_text)

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
                prefix / "bin" / "gnss_visibility",
                prefix / "bin" / "gnss_visibility_plot.py",
                prefix / "bin" / "gnss_nav_products",
                prefix / "bin" / "gnss_fetch_products.py",
                prefix / "bin" / "gnss_artifact_manifest.py",
                prefix / "bin" / "gnss_ionex_info.py",
                prefix / "bin" / "gnss_dcb_info.py",
                prefix / "bin" / "gnss_rcv.py",
                prefix / "bin" / "gnss_web.py",
                prefix / "bin" / "gnss_toml_config.py",
                prefix / "bin" / "gnss_live_signoff.py",
                prefix / "bin" / "gnss_moving_base_signoff.py",
                prefix / "bin" / "gnss_scorpion_moving_base_signoff.py",
                prefix / "bin" / "gnss_moving_base_prepare.py",
                prefix / "bin" / "gnss_moving_base_plot.py",
                prefix / "bin" / "gnss_rtk_kinematic_signoff.py",
                prefix / "bin" / "gnss_ppp_static_signoff.py",
                prefix / "bin" / "gnss_ppp_kinematic_signoff.py",
                prefix / "bin" / "gnss_ppp_products_signoff.py",
                prefix / "bin" / "gnss_ppc_demo.py",
                prefix / "bin" / "gnss_ppc_rtk_signoff.py",
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
                prefix / "scripts" / "generate_architecture_diagram.py",
                prefix / "scripts" / "generate_feature_overview_card.py",
                prefix / "scripts" / "generate_odaiba_social_card.py",
                prefix / "configs" / "live.example.conf",
                prefix / "configs" / "ppp_products_ppc.example.toml",
                prefix / "configs" / "moving_base_signoff.example.toml",
                prefix / "configs" / "live_signoff.example.toml",
                prefix / "configs" / "ppc_rtk_signoff.example.toml",
                prefix / "configs" / "web.example.toml",
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
                visibility_csv = prefix / "tmp_visibility.csv"
                visibility_summary = prefix / "tmp_visibility.json"
                visibility_png = prefix / "tmp_visibility.png"
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

                subprocess.run(
                    [
                        str(prefix / "bin" / "gnss"),
                        "ppp-products-signoff",
                        "--help",
                    ],
                    check=True,
                    cwd=ROOT_DIR,
                    env=env,
                    stdout=subprocess.DEVNULL,
                )

                subprocess.run(
                    [
                        str(prefix / "bin" / "gnss"),
                        "visibility",
                        "--obs",
                        str(ROOT_DIR / "data/rover_static.obs"),
                        "--nav",
                        str(ROOT_DIR / "data/navigation_static.nav"),
                        "--csv",
                        str(visibility_csv),
                        "--summary-json",
                        str(visibility_summary),
                        "--max-epochs",
                        "3",
                        "--quiet",
                    ],
                    check=True,
                    cwd=ROOT_DIR,
                    env=env,
                )
                self.assertTrue(visibility_csv.exists(), "installed visibility did not write CSV")
                self.assertTrue(visibility_summary.exists(), "installed visibility did not write summary")
                subprocess.run(
                    [
                        str(prefix / "bin" / "gnss"),
                        "visibility-plot",
                        str(visibility_csv),
                        str(visibility_png),
                    ],
                    check=True,
                    cwd=ROOT_DIR,
                    env=env,
                )
                self.assertTrue(visibility_png.exists(), "installed visibility-plot did not write PNG")

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

            architecture_png = prefix / "tmp_architecture.png"
            subprocess.run(
                [
                    "python3",
                    str(prefix / "scripts" / "generate_architecture_diagram.py"),
                    "--output",
                    str(architecture_png),
                ],
                check=True,
                cwd=ROOT_DIR,
                env=env,
            )
            self.assertTrue(architecture_png.exists(), "installed architecture generator did not write PNG")

            web_help = subprocess.run(
                [str(prefix / "bin" / "gnss"), "web", "--help"],
                check=True,
                cwd=ROOT_DIR,
                env=env,
                capture_output=True,
                text=True,
            )
            self.assertIn("local web UI", web_help.stdout)

            live_signoff_help = subprocess.run(
                [str(prefix / "bin" / "gnss"), "live-signoff", "--help"],
                check=True,
                cwd=ROOT_DIR,
                env=env,
                capture_output=True,
                text=True,
            )
            self.assertIn("realtime", live_signoff_help.stdout.lower())

            moving_base_signoff_help = subprocess.run(
                [str(prefix / "bin" / "gnss"), "moving-base-signoff", "--help"],
                check=True,
                cwd=ROOT_DIR,
                env=env,
                capture_output=True,
                text=True,
            )
            self.assertIn("moving-base", moving_base_signoff_help.stdout.lower())

            scorpion_moving_base_help = subprocess.run(
                [str(prefix / "bin" / "gnss"), "scorpion-moving-base-signoff", "--help"],
                check=True,
                cwd=ROOT_DIR,
                env=env,
                capture_output=True,
                text=True,
            )
            self.assertIn("scorpion", scorpion_moving_base_help.stdout.lower())

            moving_base_plot_help = subprocess.run(
                [str(prefix / "bin" / "gnss"), "moving-base-plot", "--help"],
                check=True,
                cwd=ROOT_DIR,
                env=env,
                capture_output=True,
                text=True,
            )
            self.assertIn("baseline", moving_base_plot_help.stdout.lower())

            fetch_products_help = subprocess.run(
                [str(prefix / "bin" / "gnss"), "fetch-products", "--help"],
                check=True,
                cwd=ROOT_DIR,
                env=env,
                capture_output=True,
                text=True,
            )
            self.assertIn("sp3=", fetch_products_help.stdout.lower())
            self.assertIn("--preset", fetch_products_help.stdout)

            artifact_manifest_help = subprocess.run(
                [str(prefix / "bin" / "gnss"), "artifact-manifest", "--help"],
                check=True,
                cwd=ROOT_DIR,
                env=env,
                capture_output=True,
                text=True,
            )
            self.assertIn("artifact manifest", artifact_manifest_help.stdout.lower())

            ionex_help = subprocess.run(
                [str(prefix / "bin" / "gnss"), "ionex-info", "--help"],
                check=True,
                cwd=ROOT_DIR,
                env=env,
                capture_output=True,
                text=True,
            )
            self.assertIn("IONEX", ionex_help.stdout)

            dcb_help = subprocess.run(
                [str(prefix / "bin" / "gnss"), "dcb-info", "--help"],
                check=True,
                cwd=ROOT_DIR,
                env=env,
                capture_output=True,
                text=True,
            )
            self.assertIn("DCB", dcb_help.stdout)

            visibility_help = subprocess.run(
                [str(prefix / "bin" / "gnss"), "visibility", "--help"],
                check=True,
                cwd=ROOT_DIR,
                env=env,
                capture_output=True,
                text=True,
            )
            self.assertIn("visibility rows", visibility_help.stdout.lower())

            visibility_plot_help = subprocess.run(
                [str(prefix / "bin" / "gnss"), "visibility-plot", "--help"],
                check=True,
                cwd=ROOT_DIR,
                env=env,
                capture_output=True,
                text=True,
            )
            self.assertIn("visibility csv", visibility_plot_help.stdout.lower())


if __name__ == "__main__":
    unittest.main()
