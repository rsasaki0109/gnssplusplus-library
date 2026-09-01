"""Focused tests for the Phase75 scorer-only truth-schema recovery."""

from __future__ import annotations

import hashlib
import importlib.util
import json
from pathlib import Path
import tempfile
import unittest


ROOT = Path(__file__).resolve().parents[1]
RUNNER_PATH = ROOT / "apps/commands/benchmarks/gnss_smartphone_phase75_phase74_accuracy_integrity_recovery.py"
FREEZE = ROOT / "docs/use_cases/records/smartphone_r5_phase75_phase74_accuracy_integrity_recovery_freeze_v1.json"
EXPECTED_FREEZE_SHA256 = "47dcbaa1f2e7ec8d43e87905d29a2d7fd9c520327516db8c3c871d62fd67b663"
ROUTE = "2021-03-16-18-59-us-ca-mtv-a/pixel5"

_SPEC = importlib.util.spec_from_file_location("phase75_accuracy_recovery", RUNNER_PATH)
if _SPEC is None or _SPEC.loader is None:  # pragma: no cover
    raise RuntimeError(f"failed to load runner: {RUNNER_PATH}")
RUNNER = importlib.util.module_from_spec(_SPEC)
_SPEC.loader.exec_module(RUNNER)


class Phase75AccuracyRecoveryTests(unittest.TestCase):
    def test_freeze_pin_and_read_contract(self) -> None:
        self.assertEqual(hashlib.sha256(FREEZE.read_bytes()).hexdigest(), EXPECTED_FREEZE_SHA256)
        freeze = json.loads(FREEZE.read_text(encoding="utf-8"))
        self.assertEqual(freeze["status"], "frozen-before-phase75-truth-read")
        self.assertEqual(freeze["truth_parser_contract"]["reader"], "CSV DictReader over header names")
        self.assertEqual(freeze["truth_parser_contract"]["required_fields"], ["UnixTimeMillis", "LatitudeDegrees", "LongitudeDegrees"])
        self.assertEqual(freeze["read_policy"]["truth_reads_before_manifest"], 0)
        self.assertEqual(freeze["read_policy"]["truth_reads_per_route"], 1)

    def test_dictreader_accepts_optional_phone_and_additional_columns(self) -> None:
        payload = (
            b"extra,phone,UnixTimeMillis,LatitudeDegrees,LongitudeDegrees\n"
            + f"x,{ROUTE},100,37.0,-122.0\n".encode()
        )
        self.assertEqual(RUNNER._parse_truth_dictreader(payload, ROUTE), {100: (37.0, -122.0)})

    def test_dictreader_assigns_route_when_phone_is_absent(self) -> None:
        payload = b"UnixTimeMillis,LatitudeDegrees,LongitudeDegrees,quality\n100,37.0,-122.0,ok\n"
        self.assertEqual(RUNNER._parse_truth_dictreader(payload, ROUTE), {100: (37.0, -122.0)})

    def test_dictreader_rejects_wrong_phone_and_unnamed_extra_cells(self) -> None:
        wrong_phone = b"phone,UnixTimeMillis,LatitudeDegrees,LongitudeDegrees\nother,100,37,-122\n"
        with self.assertRaises(RUNNER.Phase75RecoveryError):
            RUNNER._parse_truth_dictreader(wrong_phone, ROUTE)
        extra_cells = b"UnixTimeMillis,LatitudeDegrees,LongitudeDegrees\n100,37,-122,unexpected\n"
        with self.assertRaises(RUNNER.Phase75RecoveryError):
            RUNNER._parse_truth_dictreader(extra_cells, ROUTE)

    def test_recovery_has_no_native_or_solver_subprocess_and_new_root(self) -> None:
        source = RUNNER_PATH.read_text(encoding="utf-8").lower()
        self.assertNotIn("import subprocess", source)
        self.assertNotIn("subprocess.run", source)
        freeze = json.loads(FREEZE.read_text(encoding="utf-8"))
        self.assertEqual(freeze["recovery_scope"]["new_output_root"], "output/smartphone-r5/phase75-phase74-accuracy-integrity-recovery-v1")
        self.assertFalse(freeze["read_policy"]["native_or_solver_subprocess"])


if __name__ == "__main__":
    unittest.main()
