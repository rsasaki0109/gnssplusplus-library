"""Run-to-run spread driver for the PPC --navi776-tc replay (scripts/analysis/ppc_fuse_spread.py)."""

import importlib.util
import math
from pathlib import Path
import sys
import unittest

ROOT = Path(__file__).resolve().parents[1]
spec = importlib.util.spec_from_file_location(
    "ppc_fuse_spread", ROOT / "scripts/analysis/ppc_fuse_spread.py"
)
spread = importlib.util.module_from_spec(spec)
sys.modules["ppc_fuse_spread"] = spread
spec.loader.exec_module(spread)


def epoch(status, east=0.0, north=0.0, up=0.0):
    return spread.comparison.MatchedEpoch(
        tow=0.0, traj_east_m=0.0, traj_north_m=0.0, traj_up_m=0.0,
        east_m=east, north_m=north, up_m=up,
        horiz_error_m=math.hypot(east, north), status=status,
    )


class PpcFuseSpreadTest(unittest.TestCase):
    def test_nine_members_move_one_axis_by_two_or_four_centimetres(self) -> None:
        base = spread.LEVER_ARMS["tokyo"]
        members = spread.member_lever_arms(base)
        self.assertEqual(len(members), 9)
        self.assertEqual(members[0], "0.310,0.000,0.550")
        self.assertEqual(len(set(members)), 9)
        for member in members[1:]:
            offsets = [round(float(v) - b, 3) for v, b in zip(member.split(","), base)]
            moved = [o for o in offsets if o != 0.0]
            self.assertEqual(len(moved), 1)
            self.assertIn(abs(moved[0]), (0.02, 0.04))

    def test_score_counts_correct_fix_by_3d_error_and_wrong_fix_per_fix(self) -> None:
        matched = [
            epoch(4, 0.1, 0.1, 0.1),   # correct FIX
            epoch(4, 0.1, 0.0, 0.6),   # vertical-only wrong FIX
            epoch(4, 4.0, 0.0, 0.0),   # wrong FIX above 3 m
            epoch(5),                  # FLOAT
        ]
        result = spread.score(matched)
        self.assertAlmostEqual(result["correct_pct"], 25.0)
        self.assertAlmostEqual(result["wrong_pct"], 200.0 / 3.0)
        self.assertAlmostEqual(result["fix_pct"], 75.0)
        self.assertEqual(result["fix_above_3m"], 1.0)
        with self.assertRaises(ValueError):
            spread.score([])

    def test_summary_gives_sample_sd_per_run_and_standard_error_of_the_mean(self) -> None:
        def m(correct):
            return {"correct_pct": correct, "wrong_pct": 1.0, "fix_pct": 50.0, "fix_above_3m": 0.0}

        summary = spread.summarize({"a": [m(10.0), m(12.0)], "b": [m(20.0), m(20.0)]})
        self.assertAlmostEqual(summary["runs"]["a"]["correct_pct"]["mean"], 11.0)
        self.assertAlmostEqual(summary["runs"]["a"]["correct_pct"]["sd"], math.sqrt(2.0))
        self.assertAlmostEqual(summary["mean"]["correct_pct"]["mean"], 15.5)
        # sqrt(sd_a^2 / n_a + sd_b^2 / n_b) / runs = sqrt(2 / 2) / 2
        self.assertAlmostEqual(summary["mean"]["correct_pct"]["se"], 0.5)
        with self.assertRaises(ValueError):
            spread.summarize({"a": [m(10.0)]})

    def test_arguments_after_the_separator_reach_gnss_fuse(self) -> None:
        args = spread.parse_args(["--out-dir", "out", "--runs", "nagoya/run3", "--",
                                  "--hold-kalman-update", "--min-fix-satellites", "4"])
        self.assertEqual(args.extra, ["--hold-kalman-update", "--min-fix-satellites", "4"])
        self.assertEqual(args.runs, ["nagoya/run3"])
        self.assertEqual(spread.parse_args(["--out-dir", "out"]).extra, [])


if __name__ == "__main__":
    unittest.main()
