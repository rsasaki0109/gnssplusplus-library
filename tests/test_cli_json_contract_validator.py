#!/usr/bin/env python3
"""Tests for the CLI JSON contract validator used by CI and Docker smoke."""

from __future__ import annotations

import importlib.util
import subprocess
import sys
import unittest
from pathlib import Path
from unittest import mock


ROOT_DIR = Path(__file__).resolve().parents[1]
VALIDATOR_PATH = ROOT_DIR / "scripts" / "ci" / "validate_cli_json_contract.py"


def load_validator_module():
    spec = importlib.util.spec_from_file_location("validate_cli_json_contract", VALIDATOR_PATH)
    if spec is None or spec.loader is None:
        raise RuntimeError(f"failed to load validator module from {VALIDATOR_PATH}")
    module = importlib.util.module_from_spec(spec)
    sys.modules[spec.name] = module
    spec.loader.exec_module(module)
    return module


class CliJsonContractValidatorTest(unittest.TestCase):
    @classmethod
    def setUpClass(cls) -> None:
        cls.validator = load_validator_module()

    def test_local_cli_contract_passes(self) -> None:
        self.validator.validate_contract((sys.executable, str(ROOT_DIR / "apps" / "gnss.py")))

    def test_commands_validator_rejects_stale_shape(self) -> None:
        with self.assertRaisesRegex(AssertionError, "unexpected keys"):
            self.validator.validate_commands_payload(
                {
                    "schema_version": 1,
                    "filters": {"kind": None},
                    "count": 1,
                    "commands": [
                        {
                            "kind": "builtin",
                            "name": "commands",
                            "summary": "List commands.",
                            "extra": "field",
                        }
                    ],
                },
                "commands --json",
                expected_filters={"kind": None},
            )

    def test_doctor_validator_rejects_stale_next_command(self) -> None:
        with self.assertRaisesRegex(AssertionError, "stale next_commands"):
            self.validator.validate_doctor_payload(
                {
                    "root": str(ROOT_DIR),
                    "ok": False,
                    "checks": [
                        {
                            "name": "repository root",
                            "status": "missing",
                            "detail": "missing",
                        }
                    ],
                    "next_commands": ["python3 apps/gnss.py does-not-exist"],
                },
                "doctor --json --strict",
                known_commands={"commands", "doctor"},
            )

    def test_json_loader_reports_invalid_stdout(self) -> None:
        result = subprocess.CompletedProcess(
            args=["gnss", "commands", "--json"],
            returncode=0,
            stdout="not json\n",
            stderr="",
        )
        with self.assertRaisesRegex(AssertionError, "stdout is not valid JSON"):
            self.validator.load_stdout_json(result, "commands --json")

    def test_contract_runs_expected_cli_invocations(self) -> None:
        calls: list[tuple[str, ...]] = []

        def fake_validate(invocation, label, *, expected_filters=None, known_commands=None):
            calls.append(tuple(invocation.command))
            if label == "commands --json":
                self.assertEqual(invocation.expected_returncode, 0)
                return {"commands", "doctor", "ppp", "clas-ppp"}
            if label == "commands --query ppp --limit 3 --json":
                self.assertEqual(invocation.expected_returncode, 0)
                return {"ppp", "clas-ppp"}
            if label == "doctor missing-root --json --strict":
                self.assertEqual(invocation.expected_returncode, 1)
                self.assertEqual(known_commands, {"commands", "doctor", "ppp", "clas-ppp"})
                return {}
            raise AssertionError(f"unexpected label: {label}")

        with mock.patch.object(self.validator, "validate_invocation_json", side_effect=fake_validate):
            self.validator.validate_contract(("docker", "run", "--rm", "libgnsspp-ci"))

        self.assertEqual(
            calls,
            [
                ("docker", "run", "--rm", "libgnsspp-ci", "commands", "--json"),
                (
                    "docker",
                    "run",
                    "--rm",
                    "libgnsspp-ci",
                    "commands",
                    "--query",
                    "ppp",
                    "--limit",
                    "3",
                    "--json",
                ),
                (
                    "docker",
                    "run",
                    "--rm",
                    "libgnsspp-ci",
                    "doctor",
                    "--root",
                    "/tmp/gnsspp-missing-root",
                    "--json",
                    "--strict",
                ),
            ],
        )


if __name__ == "__main__":
    unittest.main()
