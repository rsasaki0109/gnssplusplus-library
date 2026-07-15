#!/usr/bin/env python3
"""Focused unit tests for Compact SSR bias-bank lifecycle mechanics."""

from __future__ import annotations

import importlib.util
import sys
import unittest
from pathlib import Path


ROOT_DIR = Path(__file__).resolve().parents[1]
SCRIPT_PATH = (
    ROOT_DIR / "apps" / "commands" / "receivers" / "gnss_qzss_l6_bias.py"
)

spec = importlib.util.spec_from_file_location("gnss_qzss_l6_bias", SCRIPT_PATH)
assert spec is not None and spec.loader is not None
bias = importlib.util.module_from_spec(spec)
sys.modules[spec.name] = bias
spec.loader.exec_module(bias)


PENDING = "pending"
SAME = "same"
CLOSE = "close"
LATEST = "latest"
DELAYED = "delayed"


def candidate_anchors(
    banks: bias.BiasBanks,
    tow: int,
    policy: str,
) -> list[int]:
    return bias.candidate_bank_anchors(
        banks,
        tow,
        policy,
        pending_policy=PENDING,
        same_bucket_policy=SAME,
        close_bucket_policy=CLOSE,
        latest_preceding_policy=LATEST,
        delayed_policy=DELAYED,
        bucket_seconds=30,
        effective_delay_seconds=15,
    )


class BiasBankLifecycleTest(unittest.TestCase):
    def test_anchor_uses_inclusive_thirty_second_buckets(self) -> None:
        cases = ((0, 0), (29, 0), (30, 30), (59, 30), (60, 60))
        for tow, expected in cases:
            with self.subTest(tow=tow):
                self.assertEqual(bias.bank_anchor_tow(tow, 30), expected)

    def test_store_prunes_only_banks_older_than_retention_window(self) -> None:
        banks: bias.BiasBanks | None = None
        for tow in (0, 30, 180, 210):
            banks = bias.store_bias_bank_entry(
                banks,
                tow,
                "G14",
                9,
                float(tow),
                bucket_seconds=30,
                retention_seconds=180,
            )
        self.assertEqual(sorted(banks), [30, 180, 210])

    def test_bank_policies_are_explicit_at_thirty_second_boundary(self) -> None:
        banks = {
            90: {"G14": {9: 0.9}},
            120: {"G14": {9: 1.2}},
        }
        cases = (
            (149, SAME, [120]),
            (150, SAME, [150]),
            (150, CLOSE, [120]),
            (150, LATEST, [120, 90]),
            (150, PENDING, []),
        )
        for tow, policy, expected in cases:
            with self.subTest(tow=tow, policy=policy):
                self.assertEqual(candidate_anchors(banks, tow, policy), expected)

    def test_delayed_policy_switches_on_at_fifteen_second_boundary(self) -> None:
        banks = {120: {"G14": {9: 1.2}}}
        cases = (
            (130, []),
            (134, []),
            (135, [120]),
            (140, [120]),
        )
        for tow, expected in cases:
            with self.subTest(tow=tow):
                self.assertEqual(candidate_anchors(banks, tow, DELAYED), expected)

    def test_entry_and_row_lookup_use_the_same_anchor_precedence(self) -> None:
        banks = {
            90: {"G14": {2: 0.2, 9: 0.9}},
            120: {"G14": {9: 1.2}},
        }
        anchors = candidate_anchors(banks, 150, LATEST)
        self.assertEqual(
            bias.lookup_bias_bank_entry(banks, anchors, "G14", 9),
            1.2,
        )
        self.assertEqual(
            bias.lookup_bias_bank_entry(banks, anchors, "G14", 2),
            0.2,
        )
        self.assertEqual(
            bias.lookup_bias_bank_rows(banks, anchors, "G14"),
            {9: 1.2},
        )

    def test_materialization_adds_missing_rows_without_overwriting_network(self) -> None:
        pending = {"G14": {9: 4.0}}
        sources = {"G14": {9: 5}}
        base = {
            "G14": {2: 0.2, 9: 0.9},
            "G25": {2: 0.3},
        }
        bias.materialize_missing_bias_rows(
            pending,
            target_satellites=("G14", "G25"),
            resolve_rows=lambda satellite: dict(base.get(satellite, {})),
            pending_sources=sources,
            source_subtype=6,
        )
        self.assertEqual(pending["G14"], {2: 0.2, 9: 4.0})
        self.assertEqual(sources["G14"], {2: 6, 9: 5})
        self.assertEqual(pending["G25"], {2: 0.3})
        self.assertEqual(sources["G25"], {2: 6})

    def test_code_and_phase_composition_share_one_contract(self) -> None:
        cases = (
            ("direct", 1.0),
            ("sum", 1.25),
            ("base", 0.25),
        )
        for policy, expected in cases:
            with self.subTest(policy=policy):
                self.assertEqual(
                    bias.compose_bias_value(
                        1.0,
                        0.25,
                        policy,
                        direct_policy="direct",
                        base_plus_network_policy="sum",
                        base_only_if_present_policy="base",
                    ),
                    expected,
                )


if __name__ == "__main__":
    unittest.main()
