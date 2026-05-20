"""Unit tests for ``apps/gnss_dd_residuals_filtering``."""

from __future__ import annotations

import sys
import unittest
from pathlib import Path


ROOT_DIR = Path(__file__).resolve().parents[1]
APPS_DIR = ROOT_DIR / "apps"
sys.path.insert(0, str(APPS_DIR))

import gnss_dd_residuals_filtering as filtering  # noqa: E402


def _row(*, kind: str, freq: int) -> dict:
    return {"kind": kind, "frequency_index": freq}


class TestFilterRecords(unittest.TestCase):
    def setUp(self) -> None:
        self.records = [
            _row(kind="phase", freq=0),
            _row(kind="phase", freq=1),
            _row(kind="code", freq=0),
            _row(kind="code", freq=2),
        ]

    def test_all_kind_keeps_every_row(self):
        self.assertEqual(len(filtering.filter_records(self.records)), 4)

    def test_phase_kind_filters_code_out(self):
        out = filtering.filter_records(self.records, kind="phase")
        self.assertTrue(all(record["kind"] == "phase" for record in out))
        self.assertEqual(len(out), 2)

    def test_code_kind_filters_phase_out(self):
        out = filtering.filter_records(self.records, kind="code")
        self.assertTrue(all(record["kind"] == "code" for record in out))
        self.assertEqual(len(out), 2)

    def test_frequency_index_alone_filters(self):
        out = filtering.filter_records(self.records, frequency_index=0)
        self.assertEqual(len(out), 2)
        self.assertTrue(all(record["frequency_index"] == 0 for record in out))

    def test_kind_and_frequency_index_compose(self):
        out = filtering.filter_records(self.records, kind="phase", frequency_index=1)
        self.assertEqual(out, [_row(kind="phase", freq=1)])

    def test_empty_input_returns_empty(self):
        self.assertEqual(filtering.filter_records([], kind="phase", frequency_index=0), [])

    def test_no_filter_returns_same_list_instance(self):
        # Default arguments must short-circuit, not allocate a new list copy;
        # this guards against accidental O(n) churn in CI runs with millions
        # of rows.
        out = filtering.filter_records(self.records)
        self.assertIs(out, self.records)


if __name__ == "__main__":
    unittest.main()
