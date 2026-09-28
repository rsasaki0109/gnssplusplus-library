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


READY_LANES = {"clas-ppc", "spp-policy", "rtk-demo5", "odaiba"}
PLANNED_LANES = {"fgo-tokyo", "gsdc-dev-routes", "ppc-goal", "gsdc-official"}


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
        self.assertEqual(lanes["gsdc-official"]["status"], "planned")
        planned = subprocess.run(
            [sys.executable, str(GNSS_CLI), "reproduce", "gsdc-official"],
            cwd=ROOT_DIR, check=False, capture_output=True, text=True,
        )
        self.assertEqual(planned.returncode, 2)
        self.assertIn("planned", planned.stderr)


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


if __name__ == "__main__":
    unittest.main()
