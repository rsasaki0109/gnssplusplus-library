#!/usr/bin/env python3
"""Unit tests for `gnss reproduce` (manifests, rendering, metric checks).

These tests need no datasets or built binaries.
"""

from __future__ import annotations

import json
import os
from pathlib import Path
import subprocess
import sys
import tempfile
import textwrap
import unittest


ROOT_DIR = Path(__file__).resolve().parents[1]
GNSS_CLI = ROOT_DIR / "apps" / "gnss.py"
sys.path.insert(0, str(ROOT_DIR / "apps" / "commands"))
sys.path.insert(0, str(ROOT_DIR / "apps" / "commands" / "benchmarks"))

import gnss_reproduce as reproduce  # noqa: E402


READY_LANES = {
    "clas-ppc", "spp-policy", "rtk-demo5", "odaiba", "fgo-tokyo", "gsdc-dev-routes", "ppc-goal",
    "gsdc-official",
}
PLANNED_LANES: set[str] = set()


def write_manifest(directory: Path, text: str, name: str = "lane.toml") -> Path:
    path = directory / name
    path.write_text(textwrap.dedent(text), encoding="utf-8")
    return path


class ManifestTest(unittest.TestCase):
    def test_tracked_manifests_load(self) -> None:
        manifests = reproduce.discover_manifests()
        self.assertTrue(READY_LANES | PLANNED_LANES <= set(manifests))
        for name in READY_LANES:
            manifest = manifests[name]
            self.assertEqual(manifest["lane"]["status"], "ready")
            self.assertTrue(manifest["steps"], name)
            self.assertTrue(manifest["metrics"], name)
            self.assertTrue(
                any(metric.get("gate", True) for metric in manifest["metrics"]),
                f"{name} has no gated metric",
            )
        for name in PLANNED_LANES:
            self.assertEqual(manifests[name]["lane"]["status"], "planned")

    def test_tracked_manifests_reference_existing_repo_files(self) -> None:
        manifests = reproduce.discover_manifests()
        for name in READY_LANES:
            for step in [*manifests[name]["steps"], *manifests[name]["docs_steps"]]:
                for item in step["argv"]:
                    if item.startswith(("scripts/", "configs/")) and "{" not in item:
                        self.assertTrue((ROOT_DIR / item).is_file(), f"{name}: missing {item}")

    def test_metric_foreach_expands_placeholders_and_overrides(self) -> None:
        with tempfile.TemporaryDirectory() as tmp:
            path = write_manifest(
                Path(tmp),
                """
                [lane]
                name = "demo"
                title = "Demo lane"

                [[steps]]
                name = "noop"
                argv = ["{python}", "-c", "pass"]

                [[metrics]]
                name = "{run} rate"
                source = "{work_dir}/summary.json"
                path = "runs[key={run}].rate"
                abs_tol = 0.1
                foreach = [{ run = "a", expected = 1.0 }, { run = "b", expected = 2.0, abs_tol = 0.5 }]
                """,
            )
            manifest = reproduce.load_manifest(path)
        metrics = manifest["metrics"]
        self.assertEqual([metric["name"] for metric in metrics], ["a rate", "b rate"])
        self.assertEqual(metrics[0]["path"], "runs[key=a].rate")
        self.assertEqual(metrics[0]["source"], "{work_dir}/summary.json")
        self.assertEqual((metrics[0]["expected"], metrics[0]["abs_tol"]), (1.0, 0.1))
        self.assertEqual((metrics[1]["expected"], metrics[1]["abs_tol"]), (2.0, 0.5))

    def test_invalid_manifests_are_rejected(self) -> None:
        cases = {
            "missing argv": """
                [lane]
                name = "x"
                title = "x"
                [[steps]]
                name = "s"
            """,
            "expected without tolerance": """
                [lane]
                name = "x"
                title = "x"
                [[steps]]
                name = "s"
                argv = ["a"]
                [[metrics]]
                name = "m"
                source = "s.json"
                path = "a"
                expected = 1.0
            """,
            "unknown dataset": """
                [lane]
                name = "x"
                title = "x"
                [datasets.nope]
                required = []
                [[steps]]
                name = "s"
                argv = ["a"]
            """,
            "ready lane without steps": """
                [lane]
                name = "x"
                title = "x"
            """,
        }
        for label, text in cases.items():
            with self.subTest(label), tempfile.TemporaryDirectory() as tmp:
                path = write_manifest(Path(tmp), text)
                with self.assertRaises(reproduce.ReproduceError):
                    reproduce.load_manifest(path)


class RenderTest(unittest.TestCase):
    def test_render_steps_expands_foreach_gnss_and_bin(self) -> None:
        context = {
            "python": "py",
            "work_dir": "out",
            "ppc_root": "/data/PPC",
        }
        steps = [
            {
                "name": "spp-{city}_{run}",
                "argv": ["{gnss}", "spp", "--obs", "{ppc_root}/{city}/{run}/rover.obs", "--bin", "{bin:gnss_nonexistent_tool}"],
                "env": {"LANE_OUT": "{work_dir}"},
                "foreach": [{"city": "tokyo", "run": "run1"}, {"city": "nagoya", "run": "run2"}],
            }
        ]
        rendered = reproduce.render_steps(steps, context, build_dir=None, strict=False)
        self.assertEqual([step["name"] for step in rendered], ["spp-tokyo_run1", "spp-nagoya_run2"])
        argv = rendered[1]["argv"]
        self.assertEqual(argv[:3], ["py", "apps/gnss.py", "spp"])
        self.assertIn("/data/PPC/nagoya/run2/rover.obs", argv)
        self.assertTrue(argv[-1].endswith("gnss_nonexistent_tool" + reproduce.EXE_SUFFIX))
        self.assertEqual(rendered[0]["env"], {"LANE_OUT": "out"})

    def test_strict_rendering_fails_for_missing_binary_and_unknown_placeholder(self) -> None:
        with self.assertRaises(reproduce.ReproduceError):
            reproduce.render_text("{bin:gnss_nonexistent_tool}", {}, build_dir=None, strict=True)
        with self.assertRaises(reproduce.ReproduceError):
            reproduce.render_text("{nope}", {}, build_dir=None, strict=False)

    def test_dry_run_prints_lane_commands(self) -> None:
        result = subprocess.run(
            [
                sys.executable, str(GNSS_CLI), "reproduce", "spp-policy", "--dry-run",
                "--ppc-root", "/datasets/PPC-Dataset", "--work-dir", "output/reproduce/test-dry-run",
            ],
            cwd=ROOT_DIR, check=False, capture_output=True, text=True,
        )
        self.assertEqual(result.returncode, 0, result.stderr)
        self.assertIn("apps/gnss.py spp --obs /datasets/PPC-Dataset/tokyo/run1/rover.obs", result.stdout)
        self.assertIn("--adaptive-robust-weighting", result.stdout)
        self.assertIn("ppc-spp-policy-suite", result.stdout)
        self.assertIn("# warning: dataset `ppc` incomplete", result.stdout)
        self.assertFalse((ROOT_DIR / "output" / "reproduce" / "test-dry-run").exists())

    def test_update_docs_dry_run_includes_docs_steps(self) -> None:
        result = subprocess.run(
            [
                sys.executable, str(GNSS_CLI), "reproduce", "rtk-demo5", "--dry-run", "--update-docs",
                "--ppc-root", "/datasets/PPC-Dataset", "--rtklib-bin", "/opt/rtklib/rnx2rtkp",
            ],
            cwd=ROOT_DIR, check=False, capture_output=True, text=True,
        )
        self.assertEqual(result.returncode, 0, result.stderr)
        self.assertIn("--rtklib-config configs/reproduce/rtklib_demo5_ppc.conf", result.stdout)
        self.assertIn("update_ppc_coverage_readme.py", result.stdout)
        self.assertIn("docs/ppc_rtk_demo5_scorecard.png", result.stdout)

    def test_list_json_and_planned_lane(self) -> None:
        result = subprocess.run(
            [sys.executable, str(GNSS_CLI), "reproduce", "list", "--json"],
            cwd=ROOT_DIR, check=False, capture_output=True, text=True,
        )
        self.assertEqual(result.returncode, 0, result.stderr)
        lanes = {row["lane"]: row for row in json.loads(result.stdout)["lanes"]}
        self.assertTrue(READY_LANES | PLANNED_LANES <= set(lanes))
        self.assertEqual(lanes["gsdc-official"]["status"], "ready")
        # Every tracked lane is ready now; a planned lane still refuses to run.
        with tempfile.TemporaryDirectory() as tmp:
            manifest = write_manifest(Path(tmp), """
                [lane]
                name = "synthetic-planned"
                status = "planned"
                title = "planned lane"
            """)
            planned = subprocess.run(
                [sys.executable, str(GNSS_CLI), "reproduce", "synthetic-planned", "--manifest", str(manifest)],
                cwd=ROOT_DIR, check=False, capture_output=True, text=True,
            )
        self.assertEqual(planned.returncode, 2)
        self.assertIn("planned", planned.stderr)


class FgoTokyoLaneTest(unittest.TestCase):
    def test_fgo_tokyo_dry_run_renders_both_presets(self) -> None:
        result = subprocess.run(
            [
                sys.executable, str(GNSS_CLI), "reproduce", "fgo-tokyo", "--dry-run", "--update-docs",
                "--ppc-root", "/datasets/PPC-Dataset", "--work-dir", "output/reproduce/test-fgo-dry-run",
            ],
            cwd=ROOT_DIR, check=False, capture_output=True, text=True,
        )
        self.assertEqual(result.returncode, 0, result.stderr)
        commands = [line for line in result.stdout.splitlines() if "fgo_tokyo_reproduce.py run" in line]
        self.assertEqual(len(commands), 6)
        gf = [line for line in commands if "/gf_reset/" in line]
        baseline = [line for line in commands if "/baseline/" in line]
        self.assertEqual(len(gf), 3)
        self.assertTrue(all(line.rstrip().endswith("--gf-slip-reset") for line in gf))
        self.assertFalse(any("--gf-slip-reset" in line for line in baseline))
        for line in commands:
            self.assertIn("gnss_fgo_parity", line)
            self.assertIn("--imu /datasets/PPC-Dataset/tokyo/run", line)
            self.assertIn("--fixed-lag 5", line)
        self.assertIn("plot_fgo_parity_runs.py", result.stdout)
        self.assertFalse((ROOT_DIR / "output" / "reproduce" / "test-fgo-dry-run").exists())

    def test_fgo_tokyo_scorer_parses_parity_stdout(self) -> None:
        sys.path.insert(0, str(ROOT_DIR / "scripts" / "experiments" / "ppc"))
        import fgo_tokyo_reproduce as fgo  # noqa: E402

        text = textwrap.dedent(
            """
            === (a4) MILESTONE 2c: IncrementalFixedLagSmoother (full-scale) ===
              lag=5 s, epochs=11000, smoother_updates=11000, peak_window_vars=40
              wall_clock=463.5 s (42.1 ms/epoch), nonfinite=0, NONE_epochs=0
              per-epoch LAMBDA: attempts=9000, fixed_epochs=5918/11000 (53.8% fix-rate), best_ratio=99
              Geometry-free slip reset: on (confirmed_resets=120, gross_spp_demotions=14)
              horizontal error vs reference.csv:
                FLOAT: n=5000 rms=3.5 m max=40 m
                FIXED: n=5900 rms=1.18 m max=30 m
                ALL (float+fixed) <50cm rate=54.9%
            """
        )
        parsed = fgo.parse_parity_stdout(text)
        self.assertEqual(parsed["epochs"], 11000)
        self.assertEqual(parsed["gf_guard_demotions"], 14)
        self.assertEqual(parsed["gf_confirmed_resets"], 120)
        self.assertAlmostEqual(parsed["lambda_fix_rate_pct"], 53.8)
        self.assertAlmostEqual(parsed["ref_fixed_rms_h_m"], 1.18)
        self.assertAlmostEqual(parsed["ref_under50_pct"], 54.9)
        comparison = fgo.compare_with_reference(
            [
                {"label": "tokyo_run1", "under50_pct": 54.9, "fix_rate_pct": 53.8, "fixed_rms_h_m": 1.180},
                {"label": "tokyo_run2", "under50_pct": 85.7, "fix_rate_pct": 78.6, "fixed_rms_h_m": 0.109},
                {"label": "tokyo_run3", "under50_pct": 77.5, "fix_rate_pct": 69.3, "fixed_rms_h_m": 0.125},
            ]
        )
        self.assertEqual(comparison["runs_better"], {"under50_pct": 2, "fix_rate_pct": 3, "fixed_rms_h_m": 2})
        self.assertAlmostEqual(comparison["mean_delta"]["fix_rate_pct"], 10.666667, places=5)
        self.assertAlmostEqual(comparison["mean_delta"]["under50_pct"], 7.866667, places=5)


class GsdcDevRoutesLaneTest(unittest.TestCase):
    @staticmethod
    def _import(name: str):
        path = str(ROOT_DIR / "scripts" / "experiments" / "gsdc")
        if path not in sys.path:
            sys.path.insert(0, path)
        return __import__(name)

    def test_dry_run_renders_four_routes_with_surveyed_bases(self) -> None:
        result = subprocess.run(
            [
                sys.executable, str(GNSS_CLI), "reproduce", "gsdc-dev-routes", "--dry-run", "--update-docs",
                "--gsdc-root", "/datasets/gsdc2023/dataset_2023",
                "--work-dir", "output/reproduce/test-gsdc-dry-run",
            ],
            cwd=ROOT_DIR, check=False, capture_output=True, text=True,
        )
        self.assertEqual(result.returncode, 0, result.stderr)
        commands = [line for line in result.stdout.splitlines() if "gsdc_dev_routes_reproduce.py run" in line]
        self.assertEqual(len(commands), 4)
        expected = {
            "H": ("2021-08-24-20-32-us-ca-mtv-h/pixel5", "-2698117.9416 -4301326.2649 3847286.2750"),
            "U": ("2023-03-08-21-34-us-ca-mtv-u/pixel5", "-2698117.9861 -4301326.2071 3847286.2977"),
            "A": ("2021-03-16-18-59-us-ca-mtv-a/pixel5", "-2703116.3177 -4291766.7551 3854248.0736"),
            "LAX-T": ("2022-04-01-18-22-us-ca-lax-t/pixel5", "-2507799.2243 -4676369.3031 3526891.0358"),
        }
        for route, (dataset, ecef) in expected.items():
            line = next(line for line in commands if f"--label {route} " in line)
            self.assertIn(f"--dataset-id {dataset}", line)
            self.assertIn(f"--native-base-position-ecef {ecef}", line)
            self.assertIn("gnss_fgo_imu_no_base", line)
            self.assertEqual("--native-sparse-p-staging" in line, route == "LAX-T")
            self.assertEqual("--native-joint-ionosphere 3 0.02 1.5" in line, route == "LAX-T")
        # The truth root falls back to the GSDC root.
        self.assertIn("--truth-root /datasets/gsdc2023/dataset_2023", result.stdout)
        self.assertIn("gsdc_dev_routes_reproduce.py figure", result.stdout)
        self.assertFalse((ROOT_DIR / "output" / "reproduce" / "test-gsdc-dry-run").exists())

    def test_scorer_percentiles_and_exact_join(self) -> None:
        scorer = self._import("gsdc_dev_routes_reproduce")
        self.assertAlmostEqual(scorer.percentile([1.0, 2.0, 3.0, 4.0], 50.0), 2.5)
        self.assertAlmostEqual(scorer.percentile([0.0, 10.0], 95.0), 9.5)
        # 1e-5 deg of latitude = R * 1e-5 * pi / 180.
        self.assertAlmostEqual(scorer.haversine_m(0.0, 0.0, 1e-5, 0.0), 6371008.8 * 1e-5 * 3.141592653589793 / 180.0)
        with tempfile.TemporaryDirectory() as tmp:
            tmp_path = Path(tmp)
            truth = tmp_path / "ground_truth.csv"
            pred = tmp_path / "solution.csv"
            step = 1.0 / (6371008.8 * 3.141592653589793 / 180.0)  # 1 m of latitude
            truth_rows = ["MessageType,LatitudeDegrees,LongitudeDegrees,UnixTimeMillis"]
            pred_rows = ["phone,UnixTimeMillis,LatitudeDegrees,LongitudeDegrees"]
            for i in range(10):
                truth_rows.append(f"Fix,37.0,-122.0,{1000 + i * 1000}")
                pred_rows.append(f"pixel5,{1000 + i * 1000},{37.0 + i * step:.12f},-122.0")
            pred_rows.append("pixel5,99000,37.0,-122.0")  # not in truth
            truth.write_text("\n".join(truth_rows) + "\n", encoding="utf-8")
            pred.write_text("\n".join(pred_rows) + "\n", encoding="utf-8")
            row = scorer.score_route(pred, truth)
        self.assertEqual(row["matched_rows"], 10)
        self.assertEqual(row["unmatched_prediction_rows"], 1)
        self.assertEqual(row["unmatched_truth_rows"], 0)
        self.assertAlmostEqual(row["p50_m"], 4.5, places=4)
        self.assertAlmostEqual(row["p95_m"], 8.55, places=4)
        self.assertAlmostEqual(row["score_m"], (4.5 + 8.55) / 2.0, places=4)
        self.assertAlmostEqual(row["mean_m"], 4.5, places=4)

    def test_stager_verifies_sha256_from_tree_and_zip(self) -> None:
        import hashlib
        import zipfile

        stager = self._import("stage_gsdc_dev_route_inputs")
        route = stager.ROUTES["H"]
        members = stager.member_names(route)
        contents = {staged: f"{staged} payload\n".encode() for staged in members}
        pins = {staged: hashlib.sha256(data).hexdigest() for staged, data in contents.items()}
        original = dict(route["sha256"])
        route["sha256"].update(pins)
        try:
            with tempfile.TemporaryDirectory() as tmp:
                tmp_path = Path(tmp)
                tree = tmp_path / "dataset_2023"
                archive = tmp_path / "dataset_2023.zip"
                with zipfile.ZipFile(archive, "w") as handle:
                    for staged, relative in members.items():
                        path = tree / relative
                        path.parent.mkdir(parents=True, exist_ok=True)
                        path.write_bytes(contents[staged])
                        handle.writestr(f"dataset_2023/{relative}", contents[staged])
                    base_csv = (
                        "Base,Year,X,Y,Z,Lat,Lon,H\n"
                        "P221,2021,-2698117.9416,-4301326.2649,3847286.2750,0,0,0\n"
                    )
                    handle.writestr("dataset_2023/base/base_position.csv", base_csv)
                for source in (tmp_path, archive):
                    out = tmp_path / f"out_{source.suffix or 'dir'}"
                    code = stager.main(["--gsdc-root", str(source), "--out", str(out), "--routes", "H", "--copy"])
                    self.assertEqual(code, 0)
                    staging = json.loads((out / "staging.json").read_text(encoding="utf-8"))
                    self.assertEqual(staging["routes"][0]["files"]["base.obs"]["sha256"], pins["base.obs"])
                    self.assertEqual((out / "H" / "device_gnss.csv").read_bytes(), contents["device_gnss.csv"])
                self.assertTrue(staging["base_position_check"]["checked"])
                # Flat truth layout via --truth-root, and a corrupted input fails.
                (tree / members["ground_truth.csv"]).unlink()
                flat = tmp_path / "truth"
                flat.mkdir()
                (flat / f"{route['drive']}__pixel5__ground_truth.csv").write_bytes(contents["ground_truth.csv"])
                out = tmp_path / "out_flat"
                self.assertEqual(
                    stager.main(["--gsdc-root", str(tree), "--truth-root", str(flat), "--out", str(out), "--routes", "H"]),
                    0,
                )
                (tree / members["brdc.nav"]).write_bytes(b"corrupted\n")
                self.assertEqual(
                    stager.main(["--gsdc-root", str(tree), "--truth-root", str(flat),
                                 "--out", str(tmp_path / "out_bad"), "--routes", "H"]),
                    1,
                )
        finally:
            route["sha256"].clear()
            route["sha256"].update(original)


class PpcGoalLaneTest(unittest.TestCase):
    @staticmethod
    def _import(name: str):
        path = str(ROOT_DIR / "scripts" / "experiments" / "ppc")
        if path not in sys.path:
            sys.path.insert(0, path)
        return __import__(name)

    def test_manifest_inputs_match_pinned_stager_table(self) -> None:
        stager = self._import("stage_ppc_goal_inputs")
        manifest = reproduce.discover_manifests()["ppc-goal"]
        required = manifest["datasets"]["ppc_goal_inputs"]["required"]
        self.assertEqual(set(required), set(stager.FROZEN_INPUTS))
        self.assertEqual(len(required), 26)
        for relative, (sha, _role) in stager.FROZEN_INPUTS.items():
            self.assertRegex(sha, r"^[0-9a-f]{64}$", relative)
        # Every frozen input is consumed by some step.
        rendered = " ".join(item for step in manifest["steps"] for item in step["argv"])
        for relative in required:
            self.assertIn("{ppc_goal_inputs_root}/" + relative, rendered)

    def test_dry_run_uses_inputs_option_and_env(self) -> None:
        base = [
            sys.executable, str(GNSS_CLI), "reproduce", "ppc-goal", "--dry-run", "--update-docs",
            "--ppc-root", "/datasets/PPC-Dataset",
            "--work-dir", "output/reproduce/test-ppc-goal-dry-run",
        ]
        result = subprocess.run(
            [*base, "--ppc-goal-inputs", "/archive/ppc_goal_inputs"],
            cwd=ROOT_DIR, check=False, capture_output=True, text=True,
        )
        self.assertEqual(result.returncode, 0, result.stderr)
        out = result.stdout
        self.assertIn("stage_ppc_goal_inputs.py --inputs-root /archive/ppc_goal_inputs", out)
        self.assertIn("--baseline-pos /archive/ppc_goal_inputs/tokyo1_selected_quality_rtkbaseline_tier2_truthfree.pos", out)
        self.assertIn("apply_ppc_fgo_position_consensus.py", out)
        self.assertIn("--residual-streak-buffer-prefix", out)
        self.assertIn("ppc-demo --dataset-root /datasets/PPC-Dataset --city nagoya --run run1", out)
        self.assertIn("--use-existing-solution", out)
        self.assertIn("--summary-json docs/ppc_kf_fgo_goal_metrics.json", out)
        self.assertIn("--output docs/ppc_kf_fgo_fix_status_xy.png", out)
        self.assertIn("warning: dataset `ppc_goal_inputs` incomplete", out)
        self.assertIn("--ppc-goal-inputs or GNSSPP_PPC_GOAL_INPUTS", out)
        self.assertFalse((ROOT_DIR / "output" / "reproduce" / "test-ppc-goal-dry-run").exists())

        env = {**os.environ, "GNSSPP_PPC_GOAL_INPUTS": "/env/ppc_goal_inputs"}
        from_env = subprocess.run(base, cwd=ROOT_DIR, check=False, capture_output=True, text=True, env=env)
        self.assertEqual(from_env.returncode, 0, from_env.stderr)
        self.assertIn("--inputs-root /env/ppc_goal_inputs", from_env.stdout)

    def test_stager_verifies_and_exports(self) -> None:
        import hashlib

        stager = self._import("stage_ppc_goal_inputs")
        original = dict(stager.FROZEN_INPUTS)
        with tempfile.TemporaryDirectory() as tmp:
            root = Path(tmp) / "output"
            pins = {}
            for relative, (_sha, role) in original.items():
                path = root / relative
                path.parent.mkdir(parents=True, exist_ok=True)
                data = f"{relative}\n".encode()
                path.write_bytes(data)
                pins[relative] = (hashlib.sha256(data).hexdigest(), role)
            stager.FROZEN_INPUTS.clear()
            stager.FROZEN_INPUTS.update(pins)
            try:
                summary = Path(tmp) / "verify.json"
                out = Path(tmp) / "export"
                self.assertEqual(
                    stager.main(["--inputs-root", str(root), "--summary-json", str(summary), "--out", str(out)]), 0
                )
                self.assertTrue(json.loads(summary.read_text(encoding="utf-8"))["passed"])
                self.assertTrue((out / "gici_common" / "tokyo1.pos").is_file())
                (root / "tc_m3_full_t1_on" / "rtk.pos").write_bytes(b"corrupted\n")
                self.assertEqual(stager.main(["--inputs-root", str(root), "--summary-json", str(summary)]), 1)
                self.assertFalse(json.loads(summary.read_text(encoding="utf-8"))["passed"])
            finally:
                stager.FROZEN_INPUTS.clear()
                stager.FROZEN_INPUTS.update(original)


class CheckTest(unittest.TestCase):
    def test_lookup_path_supports_keys_indexes_and_selectors(self) -> None:
        payload = {"runs": [{"key": "a", "v": {"x": 1.5}}, {"key": "b", "v": {"x": [3, 4]}}]}
        self.assertEqual(reproduce.lookup_path(payload, "runs[key=a].v.x"), 1.5)
        self.assertEqual(reproduce.lookup_path(payload, "runs[1].v.x[1]"), 4)
        with self.assertRaises(KeyError):
            reproduce.lookup_path(payload, "runs[key=zzz].v")

    def test_evaluate_metric_tolerance_bounds_and_bool(self) -> None:
        ok = reproduce.evaluate_metric({"name": "m", "expected": 17.0, "abs_tol": 0.05}, 17.04)
        self.assertTrue(ok["passed"])
        drift = reproduce.evaluate_metric({"name": "m", "expected": 17.0, "abs_tol": 0.05}, 16.9)
        self.assertFalse(drift["passed"])
        relative = reproduce.evaluate_metric({"name": "m", "expected": 100.0, "rel_tol": 0.01}, 100.9)
        self.assertTrue(relative["passed"])
        self.assertFalse(reproduce.evaluate_metric({"name": "m", "max": 0.0}, 0.01)["passed"])
        self.assertTrue(reproduce.evaluate_metric({"name": "m", "max": 0.0}, -3.0)["passed"])
        self.assertFalse(reproduce.evaluate_metric({"name": "m", "min": 1.0}, 0.0)["passed"])
        self.assertTrue(reproduce.evaluate_metric({"name": "m", "expected": True}, True)["passed"])
        self.assertFalse(reproduce.evaluate_metric({"name": "m", "expected": True}, False)["passed"])
        self.assertFalse(reproduce.evaluate_metric({"name": "m", "max": 1.0}, None)["passed"])
        self.assertFalse(reproduce.evaluate_metric({"name": "m", "max": 1.0}, float("nan"))["passed"])

    def test_check_metrics_with_synthetic_payloads(self) -> None:
        payloads = {
            "out/summary.json": {
                "aggregate": {"avg_positioning_delta_pct": 16.9},
                "lib": {"p95": 5.0, "median": 0.70},
                "rtk": {"p95": 26.0, "median": 0.68},
            }
        }
        metrics = [
            {"name": "pos", "source": "{work_dir}/summary.json", "path": "aggregate.avg_positioning_delta_pct",
             "expected": 17.0, "abs_tol": 0.05},
            {"name": "p95 wins", "source": "{work_dir}/summary.json", "path": "lib.p95",
             "minus_path": "rtk.p95", "max": 0.0},
            {"name": "hmed", "source": "{work_dir}/summary.json", "path": "lib.median",
             "minus_path": "rtk.median", "max": 0.0, "gate": False},
            {"name": "missing", "source": "{work_dir}/other.json", "path": "x", "max": 0.0},
        ]

        def loader(path: str):
            if path not in payloads:
                raise FileNotFoundError(path)
            return payloads[path]

        results = reproduce.check_metrics(metrics, {"work_dir": "out"}, loader=loader)
        by_name = {result["name"]: result for result in results}
        self.assertFalse(by_name["pos"]["passed"])
        self.assertAlmostEqual(by_name["pos"]["delta"], -0.1)
        self.assertTrue(by_name["p95 wins"]["passed"])
        self.assertAlmostEqual(by_name["p95 wins"]["observed"], -21.0)
        self.assertFalse(by_name["hmed"]["passed"])
        self.assertFalse(by_name["hmed"]["gate"])
        self.assertIn("metrics source missing", by_name["missing"]["reasons"][0])
        table = reproduce.format_check_table(results)
        self.assertIn("| p95 wins |", table)
        self.assertIn("INFO (not gated)", table)
        self.assertIn("FAIL:", table)

    def test_check_only_exit_codes(self) -> None:
        with tempfile.TemporaryDirectory() as tmp:
            tmp_path = Path(tmp)
            work = tmp_path / "work"
            work.mkdir()
            (work / "summary.json").write_text(json.dumps({"value": 1.0}), encoding="utf-8")
            template = """
                [lane]
                name = "synthetic-check"
                title = "Synthetic"

                [[steps]]
                name = "noop"
                argv = ["{{python}}", "-c", "pass"]

                [[metrics]]
                name = "value"
                source = "{{work_dir}}/summary.json"
                path = "value"
                expected = {expected}
                abs_tol = 0.01
            """
            for expected, code in ((1.0, 0), (2.0, 3)):
                manifest = write_manifest(tmp_path, template.format(expected=expected), f"m{code}.toml")
                result = subprocess.run(
                    [
                        sys.executable, str(GNSS_CLI), "reproduce", "synthetic-check",
                        "--manifest", str(manifest), "--check-only", "--check", "--work-dir", str(work),
                    ],
                    cwd=ROOT_DIR, check=False, capture_output=True, text=True,
                    env={**os.environ, "PYTHONIOENCODING": "utf-8"},
                )
                self.assertEqual(result.returncode, code, result.stdout + result.stderr)
                report = json.loads((work / "reproduce_result.json").read_text(encoding="utf-8"))
                self.assertEqual(report["passed"], code == 0)


class GsdcOfficialLaneTest(unittest.TestCase):
    @staticmethod
    def _import():
        path = str(ROOT_DIR / "scripts" / "experiments" / "gsdc")
        if path not in sys.path:
            sys.path.insert(0, path)
        return __import__("gsdc_official_reproduce")

    def test_tracked_recipe_is_complete_and_portable(self) -> None:
        driver = self._import()
        recipe = driver.load_recipe(driver.DEFAULT_RECIPE)
        text = driver.DEFAULT_RECIPE.read_text(encoding="utf-8")
        self.assertNotIn("E:/", text)
        self.assertNotIn("E:\\\\", text)
        submission = recipe["submission"]
        self.assertEqual(submission["kaggle_ref"], 56625084)
        self.assertTrue(submission["sha256"].startswith("cbd1fde1"))
        self.assertEqual(len(recipe["drives"]), 40)
        groups: dict[str, int] = {}
        for drive in recipe["drives"]:
            groups[drive["group"]] = groups.get(drive["group"], 0) + 1
        self.assertEqual(groups, {"pixel5-heading": 17, "modern-clock": 9, "retained-height": 14})
        total = sum(len(driver.expand_keys(runs)) for runs in recipe["keys"]["runs"].values())
        self.assertEqual(total, submission["rows"])
        self.assertEqual(total, 71936)
        maps = recipe["height_maps"]["maps"]
        self.assertEqual({entry["id"] for entry in recipe["stage0"]}, set(maps))
        for entry in [*recipe["stage0"], *recipe["drives"]]:
            self.assertEqual(entry["argv"][0], "{bin}")
            self.assertEqual(len(entry["output_sha256"]), 64)
            argv = entry["argv"]
            self.assertEqual(argv[argv.index("--dataset-id") + 1], entry["id"])
            self.assertEqual(argv[argv.index("--out") + 1], entry["output"])
            for item in argv:
                if item.startswith("{gsdc_root}/"):
                    self.assertIn(item[len("{gsdc_root}/"):], entry["inputs"])
                self.assertFalse(":/" in item or ":\\" in item, item)
            if "--native-height-map" in argv:
                self.assertEqual(argv[argv.index("--native-height-map") + 1], maps[entry["id"]]["path"])
        self.assertEqual(len(recipe["height_maps"]["truth_files"]), 156)

    def test_dry_run_renders_the_five_steps(self) -> None:
        result = subprocess.run(
            [
                sys.executable, str(GNSS_CLI), "reproduce", "gsdc-official", "--dry-run",
                "--gsdc-root", "/datasets/gsdc2023/dataset_2023",
                "--gsdc-truth-root", "/datasets/gsdc2023/kaggle_train_gt",
                "--work-dir", "output/reproduce/test-gsdc-official-dry-run",
            ],
            cwd=ROOT_DIR, check=False, capture_output=True, text=True,
        )
        self.assertEqual(result.returncode, 0, result.stderr)
        for fragment in (
            "gsdc_official_reproduce.py verify-inputs",
            "gsdc_official_reproduce.py run --stage stage0",
            "gsdc_official_reproduce.py height-maps",
            "gsdc_official_reproduce.py run --stage final",
            "gsdc_official_reproduce.py assemble",
            "--gsdc-truth-root /datasets/gsdc2023/kaggle_train_gt",
            "gnss_fgo_imu_no_base",
        ):
            self.assertIn(fragment, result.stdout)
        self.assertNotIn("kaggle competitions submit", result.stdout)
        self.assertFalse((ROOT_DIR / "output" / "reproduce" / "test-gsdc-official-dry-run").exists())

    def test_key_run_length_round_trip(self) -> None:
        driver = self._import()
        keys = [1000, 2000, 3000, 3999, 4999, 50000, 95000, 96000]
        runs = driver.encode_keys(keys)
        self.assertEqual(driver.expand_keys(runs), keys)
        self.assertEqual(driver.encode_keys([7]), [[7, 1000, 1]])

    def _synthetic_recipe(self, tmp: Path) -> tuple[dict, Path]:
        driver = self._import()
        drives = []
        runs = {}
        solutions = {}
        for index, trip in enumerate(["a-course/phone1", "b-course/phone2"]):
            slug = trip.replace("/", "__")
            path = tmp / "work" / "final" / slug / "solution.csv"
            path.parent.mkdir(parents=True)
            lines = ["phone,UnixTimeMillis,LatitudeDegrees,LongitudeDegrees"]
            for step in range(4):  # native writes one extra epoch (step 3)
                lines.append(f"{trip},{1000 * (step + 1)},37.{index}{step}0000000,-122.0000000000")
            path.write_text("\n".join(lines) + "\n", encoding="utf-8")
            solutions[trip] = path
            runs[trip] = driver.encode_keys([1000, 2000, 3000])
            drives.append({
                "id": trip, "group": "pixel5-heading" if index == 0 else "retained-height",
                "output": "{work_dir}/final/" + slug + "/solution.csv",
                "output_sha256": driver.sha256_file(path),
            })
        recipe = {
            "schema": "gsdc_official_recipe.v1",
            "submission": {"sha256": "", "kaggle_ref": 56625084},
            "keys": {"runs": runs},
            "drives": drives, "stage0": [], "height_maps": {"maps": {}, "truth_files": {}},
        }
        return recipe, tmp / "work"

    def test_assembly_sha_gate_and_per_drive_diffs(self) -> None:
        driver = self._import()
        with tempfile.TemporaryDirectory() as tmp:
            tmp_path = Path(tmp)
            recipe, work = self._synthetic_recipe(tmp_path)
            rows, sources = driver.assemble_rows(recipe, {
                d["id"]: work / "final" / d["id"].replace("/", "__") / "solution.csv" for d in recipe["drives"]})
            self.assertEqual(len(rows), 6)  # the extra native epoch is dropped
            self.assertEqual(rows[0], ["a-course/phone1", "1000", "37.000000000", "-122.0000000000"])
            self.assertEqual(set(sources.values()), {"native"})
            reference = tmp_path / "reference.csv"
            reference_sha = driver.write_submission(rows, reference)
            self.assertTrue(reference.read_bytes().startswith(
                b"tripId,UnixTimeMillis,LatitudeDegrees,LongitudeDegrees\na-course/phone1,1000,"))
            recipe["submission"]["sha256"] = reference_sha
            recipe_path = tmp_path / "recipe.json"
            recipe_path.write_text(json.dumps(recipe), encoding="utf-8")
            metrics = tmp_path / "metrics.json"
            common = ["--recipe", str(recipe_path), "assemble", "--work-dir", str(work),
                      "--out", str(tmp_path / "submission.csv"), "--metrics-json", str(metrics),
                      "--reference-submission", str(reference)]
            self.assertEqual(driver.main(common), 0)
            payload = json.loads(metrics.read_text(encoding="utf-8"))
            self.assertTrue(payload["submission"]["sha256_match"])
            self.assertFalse(payload["submitted_to_kaggle"])
            self.assertEqual(payload["drives_identical"], 2)
            self.assertEqual(payload["drives_rows_identical"], 2)
            # Move one coordinate by ~1.1 m of latitude in the second drive.
            second = work / "final" / "b-course__phone2" / "solution.csv"
            second.write_text(second.read_text(encoding="utf-8").replace(
                "37.100000000,", "37.100010000,"), encoding="utf-8")
            self.assertEqual(driver.main(common), 0)
            payload = json.loads(metrics.read_text(encoding="utf-8"))
            self.assertFalse(payload["submission"]["sha256_match"])
            self.assertEqual(payload["drives_identical"], 1)
            drive = next(d for d in payload["drives"] if d["id"] == "b-course/phone2")
            self.assertFalse(drive["solution_identical"])
            self.assertEqual(drive["rows_differing"], 1)
            self.assertAlmostEqual(drive["max_horizontal_diff_m"], 1.112, places=2)
            # A missing drive fails unless --allow-partial takes it from the reference.
            second.unlink()
            with self.assertRaises(SystemExit):
                driver.main(common)
            self.assertEqual(driver.main([*common, "--allow-partial"]), 0)
            payload = json.loads(metrics.read_text(encoding="utf-8"))
            self.assertTrue(payload["submission"]["partial"])
            self.assertTrue(payload["submission"]["sha256_match"])
            self.assertEqual(payload["drives_native"], 1)

    def test_run_resume_skips_completed_drives(self) -> None:
        driver = self._import()
        with tempfile.TemporaryDirectory() as tmp:
            tmp_path = Path(tmp)
            recipe, work = self._synthetic_recipe(tmp_path)
            fake = tmp_path / "fake_solver.py"
            fake.write_text(textwrap.dedent("""
                import sys
                out = sys.argv[sys.argv.index('--out') + 1]
                open(out, 'w').write('phone,UnixTimeMillis,LatitudeDegrees,LongitudeDegrees\\n')
            """), encoding="utf-8")
            for drive in recipe["drives"]:
                drive["argv"] = ["{bin}", str(fake), "--dataset-id", drive["id"], "--out", drive["output"]]
                drive["research_source"] = {"record_wall_s": 1.0}
            recipe_path = tmp_path / "recipe.json"
            recipe_path.write_text(json.dumps(recipe), encoding="utf-8")
            argv = ["--recipe", str(recipe_path), "run", "--stage", "final", "--work-dir", str(work),
                    "--bin", sys.executable, "--drives", "a-course/phone1"]
            self.assertEqual(driver.main(argv), 0)
            record_path = work / "final" / "a-course__phone1" / "run.json"
            record = json.loads(record_path.read_text(encoding="utf-8"))
            self.assertEqual(record["returncode"], 0)
            self.assertFalse(record["identical"])  # the fake solver writes no rows
            record_path.write_text(json.dumps({**record, "wall_s": 123.0}), encoding="utf-8")
            self.assertEqual(driver.main(argv), 0)  # resumed: not rerun
            self.assertEqual(json.loads(record_path.read_text(encoding="utf-8"))["wall_s"], 123.0)


if __name__ == "__main__":
    unittest.main()
