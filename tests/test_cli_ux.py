#!/usr/bin/env python3
"""Lightweight CLI user-experience smoke tests."""

from __future__ import annotations

import importlib.util
import json
import re
import subprocess
import sys
import tempfile
import unittest
from pathlib import Path


ROOT_DIR = Path(__file__).resolve().parents[1]
GNSS_CLI = ROOT_DIR / "apps" / "gnss.py"
GNSS_EXAMPLE_COMMAND_RE = re.compile(
    r"python3\s+apps/gnss\.py(?:\s+([^\s\\`),'\"),]+))?"
)


class CliUxTest(unittest.TestCase):
    @classmethod
    def dispatcher_module(cls):
        spec = importlib.util.spec_from_file_location("gnss_dispatcher", GNSS_CLI)
        if spec is None or spec.loader is None:
            raise RuntimeError(f"failed to load dispatcher module from {GNSS_CLI}")
        module = importlib.util.module_from_spec(spec)
        spec.loader.exec_module(module)
        return module

    @classmethod
    def dispatcher_commands(cls) -> set[str]:
        return set(cls.dispatcher_module().COMMANDS)

    def run_gnss(self, *args: str) -> subprocess.CompletedProcess[str]:
        return subprocess.run(
            [sys.executable, str(GNSS_CLI), *args],
            cwd=ROOT_DIR,
            check=False,
            capture_output=True,
            text=True,
        )

    def assert_no_traceback(self, result: subprocess.CompletedProcess[str]) -> None:
        combined = result.stdout + result.stderr
        self.assertNotIn("Traceback (most recent call last)", combined)
        self.assertNotIn("ModuleNotFoundError", combined)

    def commands_from_gnss_example(self, text: str) -> list[str]:
        commands: list[str] = []
        for match in GNSS_EXAMPLE_COMMAND_RE.finditer(text):
            command = match.group(1)
            if command is None or command.startswith("-"):
                continue
            commands.append(command)
        return commands

    def stale_gnss_example_commands_in_lines(
        self,
        lines: list[tuple[str, str]],
    ) -> tuple[int, list[str]]:
        known_commands = self.dispatcher_commands()
        seen_examples = 0
        stale_examples: list[str] = []

        for label, line in lines:
            for command in self.commands_from_gnss_example(line):
                seen_examples += 1
                if command not in known_commands:
                    stale_examples.append(f"{label}: {command}")

        return seen_examples, stale_examples

    def stale_gnss_example_commands_in_files(
        self,
        paths: list[Path],
    ) -> tuple[int, list[str]]:
        lines: list[tuple[str, str]] = []
        for path in paths:
            relative_path = path.relative_to(ROOT_DIR)
            for line_number, line in enumerate(
                path.read_text(encoding="utf-8").splitlines(),
                1,
            ):
                lines.append((f"{relative_path}:{line_number}", line))
        return self.stale_gnss_example_commands_in_lines(lines)

    def test_top_level_help_keeps_primary_workflows_discoverable(self) -> None:
        result = self.run_gnss("--help")

        self.assertEqual(result.returncode, 0, msg=result.stderr)
        self.assert_no_traceback(result)
        self.assertIn("Usage: gnss <command> [args...]", result.stdout)
        self.assertIn("Commands:", result.stdout)
        self.assertIn("Examples:", result.stdout)
        for command in (
            "doctor",
            "solve",
            "ppp",
            "clas-ppp",
            "qzss-l6-info",
            "ppc-rtk-signoff",
            "web",
        ):
            self.assertIn(f"  {command}", result.stdout)

        examples = [
            line.strip()
            for line in result.stdout.splitlines()
            if line.strip().startswith("python3 apps/gnss.py ")
        ]
        duplicates = sorted({line for line in examples if examples.count(line) > 1})
        self.assertEqual(duplicates, [])

        seen_examples, stale_examples = self.stale_gnss_example_commands_in_lines(
            [(line, line) for line in examples]
        )
        self.assertGreaterEqual(seen_examples, 1)
        self.assertEqual(stale_examples, [])

    def test_representative_subcommand_help_uses_dispatcher_names(self) -> None:
        expectations = {
            "doctor": ("usage: gnss doctor", "--strict"),
            "qzss-l6-info": (
                "usage: gnss qzss-l6-info",
                "--compact-bias-row-materialization",
            ),
            "clas-ppp": (
                "usage: gnss clas-ppp",
                "--receiver-antenna-type",
                "--compact-code-bias-composition-policy",
            ),
            "web": ("usage: gnss web", "--artifact-manifest"),
        }

        for command, snippets in expectations.items():
            with self.subTest(command=command):
                result = self.run_gnss(command, "--help")
                self.assertEqual(result.returncode, 0, msg=result.stderr)
                self.assert_no_traceback(result)
                for snippet in snippets:
                    self.assertIn(snippet, result.stdout)

    def test_user_input_errors_are_actionable_not_tracebacks(self) -> None:
        cases = [
            (
                ("does-not-exist",),
                1,
                "Error: unknown command `does-not-exist`.",
                "Commands:",
            ),
            (
                ("qzss-l6-info",),
                2,
                "usage: gnss qzss-l6-info",
                "the following arguments are required: --input",
            ),
            (
                ("clas-ppp",),
                2,
                "usage: gnss clas-ppp",
                "the following arguments are required: --obs, --nav, --out",
            ),
        ]

        for args, expected_code, *snippets in cases:
            with self.subTest(args=args):
                result = self.run_gnss(*args)
                self.assertEqual(result.returncode, expected_code)
                self.assert_no_traceback(result)
                combined = result.stdout + result.stderr
                for snippet in snippets:
                    self.assertIn(snippet, combined)

    def test_unknown_command_suggests_registered_dispatcher_names(self) -> None:
        cases = [
            ("clas_ppp", "clas-ppp"),
            ("ppc-rtk-signof", "ppc-rtk-signoff"),
            ("qzss-l6-inf", "qzss-l6-info"),
        ]

        for typo, expected_command in cases:
            with self.subTest(typo=typo):
                result = self.run_gnss(typo)
                self.assertEqual(result.returncode, 1)
                self.assert_no_traceback(result)
                self.assertEqual(result.stdout, "")
                self.assertIn(f"Error: unknown command `{typo}`.", result.stderr)
                self.assertIn("Did you mean", result.stderr)
                self.assertIn(expected_command, result.stderr)
                self.assertIn("Commands:", result.stderr)

    def test_doctor_json_stays_machine_readable_for_setup_tools(self) -> None:
        with tempfile.TemporaryDirectory(prefix="gnss_cli_ux_doctor_") as temp_dir:
            missing_root = Path(temp_dir) / "missing-root"
            expected_root = str(missing_root.resolve())
            result = self.run_gnss("doctor", "--root", str(missing_root), "--json")

        self.assertEqual(result.returncode, 0, msg=result.stderr)
        self.assert_no_traceback(result)
        self.assertEqual(result.stderr, "")
        payload = json.loads(result.stdout)
        self.assertIs(payload["ok"], False)
        self.assertEqual(payload["root"], expected_root)
        self.assertIsInstance(payload["checks"], list)
        self.assertGreaterEqual(len(payload["checks"]), 3)
        self.assertIn("next_commands", payload)

    def test_doctor_strict_json_failure_stays_machine_readable(self) -> None:
        with tempfile.TemporaryDirectory(prefix="gnss_cli_ux_doctor_strict_") as temp_dir:
            missing_root = Path(temp_dir) / "missing-root"
            expected_root = str(missing_root.resolve())
            result = self.run_gnss(
                "doctor",
                "--root",
                str(missing_root),
                "--json",
                "--strict",
            )

        self.assertEqual(result.returncode, 1)
        self.assert_no_traceback(result)
        self.assertEqual(result.stderr, "")
        payload = json.loads(result.stdout)
        self.assertIs(payload["ok"], False)
        self.assertEqual(payload["root"], expected_root)
        missing_checks = [
            item for item in payload["checks"] if item["status"] == "missing"
        ]
        self.assertGreaterEqual(len(missing_checks), 1)

    def test_doctor_text_output_stays_actionable_for_first_run(self) -> None:
        with tempfile.TemporaryDirectory(prefix="gnss_cli_ux_doctor_text_") as temp_dir:
            missing_root = Path(temp_dir) / "missing-root"
            result = self.run_gnss("doctor", "--root", str(missing_root))

        self.assertEqual(result.returncode, 0, msg=result.stderr)
        self.assert_no_traceback(result)
        self.assertEqual(result.stderr, "")
        self.assertIn("libgnss++ doctor", result.stdout)
        self.assertIn("[missing] repository root:", result.stdout)
        self.assertIn("Recommended next commands:", result.stdout)

        next_command_lines = [
            line.strip()
            for line in result.stdout.splitlines()
            if line.strip().startswith("python3 apps/gnss.py ")
        ]
        seen_examples, stale_examples = self.stale_gnss_example_commands_in_lines(
            [(line, line) for line in next_command_lines]
        )
        self.assertGreaterEqual(seen_examples, 4)
        self.assertEqual(stale_examples, [])

    def test_dispatcher_registry_keeps_python_targets_runnable(self) -> None:
        dispatcher = self.dispatcher_module()
        failures: list[str] = []
        repo_root = ROOT_DIR.resolve()

        for command, metadata in sorted(dispatcher.COMMANDS.items()):
            kind = metadata.get("kind")
            target = metadata.get("target")
            summary = metadata.get("summary")

            if kind not in {"python", "binary"}:
                failures.append(f"{command}: unsupported kind {kind!r}")
            if not isinstance(target, str) or not target:
                failures.append(f"{command}: missing target")
                continue
            if not isinstance(summary, str) or not summary.strip():
                failures.append(f"{command}: missing summary")

            if kind != "python":
                continue
            target_path = Path(target)
            if not target_path.exists():
                failures.append(f"{command}: python target missing: {target}")
                continue
            try:
                target_path.resolve().relative_to(repo_root)
            except ValueError:
                failures.append(f"{command}: python target escapes repo root: {target}")

        self.assertEqual(failures, [])

    def test_markdown_gnss_examples_use_registered_dispatcher_commands(self) -> None:
        markdown_files = [ROOT_DIR / "README.md", *sorted((ROOT_DIR / "docs").rglob("*.md"))]
        seen_examples, stale_examples = self.stale_gnss_example_commands_in_files(markdown_files)

        self.assertGreaterEqual(seen_examples, 20)
        self.assertEqual(stale_examples, [])

    def test_app_generated_gnss_examples_use_registered_dispatcher_commands(self) -> None:
        app_files = sorted((ROOT_DIR / "apps").glob("*.py"))
        seen_examples, stale_examples = self.stale_gnss_example_commands_in_files(app_files)

        self.assertGreaterEqual(seen_examples, 20)
        self.assertEqual(stale_examples, [])


if __name__ == "__main__":
    unittest.main()
