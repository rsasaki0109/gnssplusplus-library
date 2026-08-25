#!/usr/bin/env python3
"""Browser-free gnss web HTTP user-experience smoke tests."""

from __future__ import annotations

import argparse
import importlib
import json
import os
import shutil
import subprocess
import sys
import tempfile
import threading
import unittest
from http.server import ThreadingHTTPServer
from pathlib import Path
from types import ModuleType
from urllib import error, request


ROOT_DIR = Path(__file__).resolve().parents[1]
COMMANDS_DIR = ROOT_DIR / "apps" / "commands"
APPS_DIR = COMMANDS_DIR / "visualization"
BENCHMARKS_DIR = COMMANDS_DIR / "benchmarks"

if str(COMMANDS_DIR) not in sys.path:
    sys.path.insert(0, str(COMMANDS_DIR))
if str(APPS_DIR) not in sys.path:
    sys.path.insert(0, str(APPS_DIR))
if str(BENCHMARKS_DIR) not in sys.path:
    sys.path.insert(0, str(BENCHMARKS_DIR))

from support import gnss_runtime

import gnss_web  # noqa: E402
import gnss_odaiba_benchmark  # noqa: E402
import gnss_odaiba_scan  # noqa: E402

load_python_module = gnss_runtime.load_python_module


def make_args(root: Path, manifest_path: Path) -> argparse.Namespace:
    return argparse.Namespace(
        root=root,
        lib_pos=None,
        rtklib_pos=None,
        odaiba_summary=None,
        rcv_status=None,
        ppc_summary_glob="output/ppc_*_summary.json",
        live_summary_glob="output/live*_summary.json",
        robotics_summary_glob="output/robotics_smoke*/**/*.json",
        ros2_bag_summary_glob="output/ros2_bag*_summary.json",
        field_report_glob="output/field_report*.json",
        visibility_summary_glob="output/visibility*_summary.json",
        moving_base_summary_glob="output/*moving_base_summary.json",
        ppp_products_summary_glob="output/*ppp*_products*_summary.json",
        wrong_fix_ledger_glob="output/*wrong_fix*ledger*.json",
        artifact_manifest=manifest_path,
        docs_url=gnss_web.DOCS_SITE_URL,
    )


def fetch_text(url: str) -> tuple[int, str, str]:
    try:
        with request.urlopen(url, timeout=5.0) as response:
            return (
                response.status,
                response.headers.get("Content-Type", ""),
                response.read().decode("utf-8"),
            )
    except error.HTTPError as exc:
        return (
            exc.code,
            exc.headers.get("Content-Type", ""),
            exc.read().decode("utf-8"),
        )


def fetch_json(url: str) -> tuple[int, str, dict[str, object]]:
    status, content_type, body = fetch_text(url)
    return status, content_type, json.loads(body)


class _ObservedRLock:
    def __init__(self) -> None:
        self._lock = threading.RLock()
        self._attempt_lock = threading.Lock()
        self._attempts = 0
        self.second_attempt = threading.Event()

    def __enter__(self) -> _ObservedRLock:
        with self._attempt_lock:
            self._attempts += 1
            if self._attempts == 2:
                self.second_attempt.set()
        self._lock.acquire()
        return self

    def __exit__(self, exc_type: object, exc_value: object, traceback: object) -> None:
        self._lock.release()


class WebHttpUxTest(unittest.TestCase):
    def test_load_python_module_registers_dataclass_and_caches(self) -> None:
        module_name = "_web_http_ux_dataclass_module"
        with tempfile.TemporaryDirectory(prefix="gnss_runtime_dataclass_") as temp_dir:
            module_path = Path(temp_dir) / "dataclass_module.py"
            module_path.write_text(
                "from dataclasses import dataclass\n"
                "@dataclass\n"
                "class Sample:\n"
                "    value: int\n",
                encoding="utf-8",
            )
            original_sys_path = list(sys.path)
            try:
                module = load_python_module(module_name, module_path)
                cached = load_python_module(module_name, module_path)
                self.assertIs(module, cached)
                self.assertEqual(module.Sample(7).value, 7)
                self.assertIs(sys.modules[module_name], module)
                module_path.unlink()
                self.assertIs(load_python_module(module_name, module_path), module)
                self.assertEqual(sys.path, original_sys_path)
            finally:
                sys.modules.pop(module_name, None)

    def test_load_python_module_is_atomic_for_concurrent_callers(self) -> None:
        module_name = "_web_http_ux_concurrent_module"
        control_name = "_web_http_ux_concurrent_control"
        started = threading.Event()
        allow_finish = threading.Event()
        control = ModuleType(control_name)
        control.started = started
        control.allow_finish = allow_finish
        observed_lock = _ObservedRLock()
        original_lock = gnss_runtime._MODULE_LOAD_LOCK
        results: list[ModuleType | None] = [None, None]
        ready_values: list[object | None] = [None, None]
        errors: list[BaseException | None] = [None, None]
        threads: list[threading.Thread] = []

        with tempfile.TemporaryDirectory(prefix="gnss_runtime_concurrent_") as temp_dir:
            module_path = Path(temp_dir) / "concurrent_module.py"
            module_path.write_text(
                f"from {control_name} import allow_finish, started\n"
                "started.set()\n"
                "if not allow_finish.wait(5.0):\n"
                "    raise RuntimeError('concurrent loader test timed out')\n"
                "READY = object()\n",
                encoding="utf-8",
            )

            def load_in_thread(index: int) -> None:
                try:
                    module = load_python_module(module_name, module_path)
                    ready_values[index] = module.READY
                    results[index] = module
                except BaseException as exc:
                    errors[index] = exc

            sys.modules[control_name] = control
            gnss_runtime._MODULE_LOAD_LOCK = observed_lock
            try:
                threads.append(threading.Thread(target=load_in_thread, args=(0,)))
                threads[0].start()
                self.assertTrue(started.wait(5.0), "first module load did not start")

                threads.append(threading.Thread(target=load_in_thread, args=(1,)))
                threads[1].start()
                self.assertTrue(
                    observed_lock.second_attempt.wait(5.0),
                    "second module load did not contend on the loader lock",
                )
                allow_finish.set()
                for thread in threads:
                    thread.join(timeout=5.0)

                self.assertTrue(all(not thread.is_alive() for thread in threads))
                self.assertEqual(errors, [None, None])
                self.assertIs(results[0], results[1])
                self.assertIsNotNone(results[0])
                self.assertIs(ready_values[0], ready_values[1])
            finally:
                allow_finish.set()
                for thread in threads:
                    thread.join(timeout=5.0)
                gnss_runtime._MODULE_LOAD_LOCK = original_lock
                gnss_runtime._MODULES_LOADING.discard(module_name)
                sys.modules.pop(module_name, None)
                sys.modules.pop(control_name, None)

    def test_load_python_module_rejects_collisions_and_rolls_back_failures(self) -> None:
        module_name = "_web_http_ux_collision_module"
        failure_name = "_web_http_ux_failure_module"
        replacement_name = "_web_http_ux_replacement_module"
        with tempfile.TemporaryDirectory(prefix="gnss_runtime_boundary_") as temp_dir:
            temp_root = Path(temp_dir)
            first_path = temp_root / "first.py"
            second_path = temp_root / "second.py"
            failure_path = temp_root / "failure.py"
            replacement_path = temp_root / "replacement.py"
            first_path.write_text("VALUE = 'first'\n", encoding="utf-8")
            second_path.write_text("VALUE = 'second'\n", encoding="utf-8")
            failure_path.write_text("raise SystemExit('expected loader exit')\n", encoding="utf-8")
            replacement_path.write_text(
                "import sys\n"
                "from types import ModuleType\n"
                "replacement = ModuleType(__name__)\n"
                "replacement.preserved = True\n"
                "sys.modules[__name__] = replacement\n"
                "raise SystemExit('expected replacement exit')\n",
                encoding="utf-8",
            )
            try:
                module = load_python_module(module_name, first_path)
                with self.assertRaisesRegex(ImportError, "Module name collision"):
                    load_python_module(module_name, second_path)
                self.assertIs(sys.modules[module_name], module)

                with self.assertRaisesRegex(SystemExit, "expected loader exit"):
                    load_python_module(failure_name, failure_path)
                self.assertNotIn(failure_name, sys.modules)

                with self.assertRaisesRegex(SystemExit, "expected replacement exit"):
                    load_python_module(replacement_name, replacement_path)
                replacement = sys.modules.get(replacement_name)
                self.assertIsInstance(replacement, ModuleType)
                self.assertIs(getattr(replacement, "preserved", None), True)
            finally:
                sys.modules.pop(module_name, None)
                sys.modules.pop(failure_name, None)
                sys.modules.pop(replacement_name, None)

    def test_source_tree_odaiba_commands_preserve_comparison_aliases(self) -> None:
        comparison = gnss_web.driving_comparison
        self.assertIs(gnss_odaiba_benchmark.driving_comparison, comparison)
        self.assertIs(gnss_odaiba_scan.driving_comparison, comparison)
        for function_name in (
            "match_to_reference",
            "read_libgnss_pos",
            "read_reference_csv",
            "read_rtklib_pos",
            "summarize",
        ):
            self.assertIs(getattr(gnss_odaiba_scan, function_name), getattr(comparison, function_name))

    def test_importing_gnss_web_does_not_add_scripts_directory_to_sys_path(self) -> None:
        scripts_entry = str((ROOT_DIR / "scripts").resolve())
        original_sys_path = list(sys.path)
        try:
            sys.path[:] = [entry for entry in sys.path if entry != scripts_entry]
            before_import = list(sys.path)
            importlib.reload(gnss_web)
            self.assertEqual(sys.path, before_import)
            self.assertNotIn(scripts_entry, sys.path)
        finally:
            sys.path[:] = original_sys_path

    def test_odaiba_command_sources_do_not_access_sys_path(self) -> None:
        command_sources = (
            ROOT_DIR / "apps/commands/visualization/gnss_web.py",
            ROOT_DIR / "apps/commands/benchmarks/gnss_odaiba_benchmark.py",
            ROOT_DIR / "apps/commands/benchmarks/gnss_odaiba_scan.py",
        )
        for source_path in command_sources:
            self.assertNotIn("sys.path", source_path.read_text(encoding="utf-8"), source_path)

    def test_flat_installed_commands_load_shared_script_without_pythonpath(self) -> None:
        with tempfile.TemporaryDirectory(prefix="gnss_flat_install_") as temp_dir:
            prefix = Path(temp_dir)
            bin_dir = prefix / "bin"
            support_dir = bin_dir / "support"
            scripts_dir = prefix / "scripts"
            support_dir.mkdir(parents=True)
            scripts_dir.mkdir()

            support_source = COMMANDS_DIR / "support"
            for filename in ("__init__.py", "gnss_runtime.py", "gnss_toml_config.py"):
                shutil.copy2(support_source / filename, support_dir / filename)

            command_cases = (
                ("web", APPS_DIR / "gnss_web.py", "--port-file"),
                (
                    "odaiba-benchmark",
                    BENCHMARKS_DIR / "gnss_odaiba_benchmark.py",
                    "--summary-json",
                ),
                ("odaiba-scan", BENCHMARKS_DIR / "gnss_odaiba_scan.py", "--window-size"),
            )
            for _, source_path, _ in command_cases:
                shutil.copy2(source_path, bin_dir / source_path.name)
            shutil.copy2(
                ROOT_DIR / "scripts" / "generate_driving_comparison.py",
                scripts_dir / "generate_driving_comparison.py",
            )

            base_env = dict(os.environ)
            base_env.pop("PYTHONPATH", None)
            base_env["PYTHONDONTWRITEBYTECODE"] = "1"
            self.assertNotIn("PYTHONPATH", base_env)

            for command_name, source_path, expected_option in command_cases:
                env = dict(base_env)
                env["GNSS_CLI_NAME"] = f"gnss {command_name}"
                completed = subprocess.run(
                    [sys.executable, str(bin_dir / source_path.name), "--help"],
                    cwd=prefix,
                    env=env,
                    capture_output=True,
                    text=True,
                    check=False,
                    timeout=10.0,
                )
                self.assertEqual(completed.returncode, 0, msg=completed.stderr)
                self.assertIn(expected_option, completed.stdout)
                self.assertNotIn("Traceback (most recent call last)", completed.stderr)

    def test_web_http_entrypoints_are_stable_without_browser(self) -> None:
        with tempfile.TemporaryDirectory(prefix="gnss_web_http_") as temp_dir:
            root = Path(temp_dir)
            output = root / "output"
            output.mkdir()
            artifact_path = output / "synthetic.txt"
            artifact_path.write_text("synthetic artifact\n", encoding="utf-8", newline="\n")
            manifest_path = output / "artifact_manifest.json"
            manifest_path.write_text(
                json.dumps(
                    {
                        "root": str(root),
                        "bundle_count": 1,
                        "bundles": [
                            {
                                "category": "ppc",
                                "label": "ppc_tokyo_run1_summary.json",
                                "summary_json": "output/ppc_tokyo_run1_summary.json",
                                "status": "excellent",
                                "headline": "200 matched / 98.0 fix / 0.114 p95 H",
                                "artifacts": {
                                    "summary": "output/ppc_tokyo_run1_summary.json",
                                    "text": "output/synthetic.txt",
                                },
                                "metrics": {"matched_epochs": 200},
                            }
                        ],
                    }
                ),
                encoding="utf-8",
            )

            server = ThreadingHTTPServer(
                ("127.0.0.1", 0),
                gnss_web.make_handler(make_args(root, manifest_path)),
            )
            thread = threading.Thread(target=server.serve_forever, daemon=True)
            thread.start()
            try:
                bound_port = int(server.server_address[1])
                base_url = f"http://127.0.0.1:{bound_port}"

                status, content_type, html = fetch_text(f"{base_url}/")
                self.assertEqual(status, 200)
                self.assertIn("text/html", content_type)
                self.assertIn("libgnss++ local web UI", html)
                self.assertIn("/api/overview", html)
                self.assertIn("Live PPP view", html)
                self.assertIn("live-pause-btn", html)
                self.assertIn("live-export-btn", html)

                status, content_type, health = fetch_json(f"{base_url}/api/health")
                self.assertEqual(status, 200)
                self.assertIn("application/json", content_type)
                self.assertIs(health["ok"], True)

                status, _, overview = fetch_json(f"{base_url}/api/overview")
                self.assertEqual(status, 200)
                self.assertEqual(overview["artifacts"]["artifact_manifest"], "output/artifact_manifest.json")
                self.assertEqual(len(overview["artifact_manifest"]), 1)
                bundle = overview["artifact_manifest"][0]
                self.assertEqual(bundle["category"], "ppc")
                self.assertEqual(bundle["artifacts"]["text"], "output/synthetic.txt")

                status, content_type, artifact = fetch_text(
                    f"{base_url}/artifact?path=output/synthetic.txt"
                )
                self.assertEqual(status, 200)
                self.assertIn("text/plain", content_type)
                self.assertEqual(artifact, "synthetic artifact\n")

                status, _, missing_path = fetch_json(f"{base_url}/artifact")
                self.assertEqual(status, 400)
                self.assertEqual(missing_path["error"], "missing artifact path")

                status, _, escaped_path = fetch_json(f"{base_url}/artifact?path=../outside.txt")
                self.assertEqual(status, 400)
                self.assertEqual(escaped_path["error"], "artifact path escapes artifact root")

                live_pos = output / "live.pos"
                live_pos.write_text(
                    "% LibGNSS++ Position Solution\n"
                    "% Format: pos\n"
                    "2324 100.0 0 0 0 35.0 139.0 10.0 6 9 1.0 0 1\n"
                    "2324 101.0 0 0 0 35.1 139.1 10.1 5 8 1.2 0 1\n",
                    encoding="utf-8",
                )
                status, _, live_all = fetch_json(f"{base_url}/api/live-pos?path=output/live.pos&after=0")
                self.assertEqual(status, 200)
                self.assertTrue(live_all["available"])
                self.assertEqual(len(live_all["points"]), 2)
                self.assertEqual(live_all["last_tow"], 101.0)

                status, _, live_incremental = fetch_json(
                    f"{base_url}/api/live-pos?path=output/live.pos&after=101.0"
                )
                self.assertEqual(status, 200)
                self.assertEqual(live_incremental["points"], [])
                self.assertEqual(live_incremental["last_tow"], 101.0)

                stream_stats = output / "stream_stats.json"
                stream_stats.write_text(
                    json.dumps(
                        {
                            "source": "ntrip://caster/MOUNT1",
                            "connected": True,
                            "reconnect_count": 2,
                            "messages_total": 5,
                            "last_message_type": 1005,
                            "last_message_name": "Stationary RTK reference station ARP",
                            "message_counts": {"1005": 5},
                        }
                    ),
                    encoding="utf-8",
                )
                status, _, stats_payload = fetch_json(
                    f"{base_url}/api/stream-stats?path=output/stream_stats.json"
                )
                self.assertEqual(status, 200)
                self.assertTrue(stats_payload["available"])
                self.assertEqual(stats_payload["reconnect_count"], 2)
                self.assertEqual(stats_payload["message_counts"]["1005"], 5)
            finally:
                server.shutdown()
                server.server_close()
                thread.join(timeout=5)


class StatusTrajectoryPayloadTest(unittest.TestCase):
    def test_status_trajectory_classifies_wrong_fix(self) -> None:
        with tempfile.TemporaryDirectory(prefix="gnss_status_traj_") as temp_dir:
            root = Path(temp_dir)
            pos = root / "run.pos"
            truth_ecef = (-3957184.0, 3310231.0, 3737703.0)
            rows = [
                "% week tow x y z sdn sdnE sdnN sdu sdu sdne sdnu sdue sdu sdu age ratio",
            ]
            epochs = [
                (2300, 1.0, truth_ecef, 4),
                (2300, 30.0, tuple(v + 0.05 for v in truth_ecef), 4),
                (2300, 60.0, tuple(v + 2.0 for v in truth_ecef), 4),
                (2300, 90.0, tuple(v - 0.3 for v in truth_ecef), 3),
            ]
            for week, tow, ecef, status in epochs:
                rows.append(f"{week} {tow} " + " ".join(f"{v:.4f}" for v in ecef) +
                            f" 0.01 0.01 0.01 {status} 8 0.5")
            pos.write_text("\n".join(rows) + "\n", encoding="utf-8", newline="\n")
            reference = root / "reference.csv"
            ref_lines = [
                "GPS Week,GPS TOW (s),ECEF X (m),ECEF Y (m),ECEF Z (m)",
            ]
            for week, tow, _, _ in epochs:
                ref_lines.append(f"{week},{tow}," + ",".join(f"{v:.4f}" for v in truth_ecef))
            reference.write_text("\n".join(ref_lines) + "\n", encoding="utf-8", newline="\n")

            payload = gnss_web.build_status_trajectory_payload(pos, root, reference, 0.5)
            self.assertTrue(payload["available"])
            self.assertEqual(payload["epoch_count"], 4)
            self.assertEqual(payload["wrong_fix_count"], 1)
            self.assertEqual(payload["status_counts"].get("FIXED"), 2)
            self.assertEqual(payload["status_counts"].get("FLOAT"), 1)
            wrong = [p for p in payload["points"] if p["status"] == "WRONG_FIX"]
            self.assertEqual(len(wrong), 1)
            self.assertGreater(wrong[0]["error_m"], 0.5)

    def test_status_trajectory_missing_file(self) -> None:
        payload = gnss_web.build_status_trajectory_payload(
            Path("does_not_exist.pos"), Path("."), None, 0.5
        )
        self.assertFalse(payload["available"])
        self.assertIn("not found", payload["error"])


if __name__ == "__main__":
    unittest.main()
