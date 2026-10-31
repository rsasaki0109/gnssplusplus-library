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
        strict = spread.score(matched, correct_3d_m=0.1)
        self.assertAlmostEqual(strict["correct_pct"], 0.0)
        self.assertAlmostEqual(strict["wrong_pct"], 100.0)
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
        self.assertEqual(args.extra,
                         ["--navi776-tc", "--hold-kalman-update", "--min-fix-satellites", "4"])
        self.assertEqual(args.runs, ["nagoya/run3"])
        self.assertEqual(spread.parse_args(["--out-dir", "out"]).extra, ["--navi776-tc"])
        args = spread.parse_args(["--out-dir", "out", "--without-navi776-tc", "--",
                                  "--tc-closed-loop", "--tc-velocity-states"])
        self.assertEqual(args.extra, ["--tc-closed-loop", "--tc-velocity-states"])
        self.assertEqual(spread.parse_args(["--out-dir", "out"]).correct_3d_m, 0.5)
        with self.assertRaises(SystemExit):
            spread.parse_args(["--out-dir", "out", "--correct-3d-m", "0"])

    def test_truth_offset_is_rotated_by_the_reference_heading(self) -> None:
        # Heading 90 deg (east): forward is east, right is south.
        headings = [(0.0, math.radians(90.0)), (1.0, math.radians(90.0))]
        antenna_error = epoch(4, east=-0.5, north=-0.1, up=-0.2)  # 0.5 m behind, 0.1 m right, 0.2 m down
        moved = spread.move_truth_to_antenna([antenna_error], headings, (0.1, -0.5, -0.2))
        self.assertAlmostEqual(moved[0].east_m, 0.0)
        self.assertAlmostEqual(moved[0].north_m, 0.0)
        self.assertAlmostEqual(moved[0].up_m, 0.0)
        self.assertAlmostEqual(moved[0].horiz_error_m, 0.0)
        self.assertAlmostEqual(spread.score(moved)["correct_pct"], 100.0)
        self.assertAlmostEqual(spread.score([antenna_error])["correct_pct"], 0.0)

    def test_truth_offset_argument_needs_three_numbers(self) -> None:
        args = spread.parse_args(["--out-dir", "out", "--truth-offset-body=-0.014,-0.563,-0.227"])
        self.assertEqual(args.truth_offset_body, (-0.014, -0.563, -0.227))
        self.assertIsNone(spread.parse_args(["--out-dir", "out"]).truth_offset_body)
        for bad in ("1,2", "a,b,c", "1,2,nan"):
            with self.assertRaises(SystemExit):
                spread.parse_args(["--out-dir", "out", f"--truth-offset-body={bad}"])

    def test_runs_outside_ppc_need_an_explicit_lever_arm(self) -> None:
        with self.assertRaises(SystemExit):
            spread.parse_args(["--out-dir", "out", "--runs", "urbannav/Odaiba_trimble"])
        args = spread.parse_args(["--out-dir", "out", "--runs", "urbannav/Odaiba_trimble",
                                  "--lever-arm", "0,0,0"])
        self.assertEqual(args.lever_arm, (0.0, 0.0, 0.0))
        self.assertEqual(spread.member_lever_arms(args.lever_arm)[0], "0.000,0.000,0.000")
        with self.assertRaises(SystemExit):
            spread.parse_args(["--out-dir", "out", "--lever-arm", "1,2"])


if __name__ == "__main__":
    unittest.main()
