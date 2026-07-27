"""Unit tests for scripts/search_imu_time_offset.py (navi.776 C)."""

import sys
import unittest
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent.parent / "scripts"))

import search_imu_time_offset as sito  # noqa: E402


class ScoreLineParsing(unittest.TestCase):
    def test_parses_canned_stdout(self):
        stdout = (
            "RTK solution stream written: out.pos\n"
            "imu_time_offset_score: offset_s=-0.04 "
            "J_mean_sq_correction_m2=0.012345 correction_epochs=2870 "
            "applied_updates=2900\n"
            "IMU samples loaded: 306001\n"
        )
        match = sito.SCORE_RE.search(stdout)
        self.assertIsNotNone(match)
        self.assertAlmostEqual(float(match.group("offset")), -0.04)
        self.assertAlmostEqual(float(match.group("j")), 0.012345)
        self.assertEqual(int(match.group("epochs")), 2870)
        self.assertEqual(int(match.group("applied")), 2900)

    def test_no_match_on_missing_line(self):
        self.assertIsNone(sito.SCORE_RE.search("no score here"))


class ArgminSelection(unittest.TestCase):
    @staticmethod
    def row(offset, j, epochs=1000, applied=1000, exit_code=0):
        return {"offset_s": offset, "j_m2": j, "correction_epochs": epochs,
                "applied_updates": applied, "exit_code": exit_code}

    def test_picks_minimum_j(self):
        rows = [self.row(-0.1, 0.5), self.row(0.0, 0.2), self.row(0.1, 0.4)]
        best = sito.pick_argmin(rows, 0.8)
        self.assertEqual(best["offset_s"], 0.0)

    def test_rejects_low_coverage_candidates(self):
        rows = [self.row(-0.1, 0.001, epochs=100),  # tiny J but 10% coverage
                self.row(0.0, 0.2, epochs=1000)]
        best = sito.pick_argmin(rows, 0.8)
        self.assertEqual(best["offset_s"], 0.0)

    def test_rejects_failed_runs_and_nan(self):
        rows = [self.row(-0.1, 0.001, exit_code=1),
                self.row(0.05, float("nan")),
                self.row(0.0, 0.3)]
        best = sito.pick_argmin(rows, 0.8)
        self.assertEqual(best["offset_s"], 0.0)

    def test_returns_none_when_nothing_usable(self):
        rows = [self.row(0.0, float("nan")), self.row(0.1, 0.1, epochs=0)]
        self.assertIsNone(sito.pick_argmin(rows, 0.8))


class CandidateGrid(unittest.TestCase):
    def test_exhaustive_grid_is_paper_faithful(self):
        args = type("A", (), {"coarse_fine": "off", "offset_min": -1.0,
                              "offset_max": 1.0, "offset_step": 0.02,
                              "coarse_step": 0.10})()
        grid = sito.build_candidates(args)
        self.assertEqual(len(grid), 101)
        self.assertAlmostEqual(grid[0], -1.0)
        self.assertAlmostEqual(grid[-1], 1.0)

    def test_coarse_grid(self):
        args = type("A", (), {"coarse_fine": "on", "offset_min": -1.0,
                              "offset_max": 1.0, "offset_step": 0.02,
                              "coarse_step": 0.10})()
        grid = sito.build_candidates(args)
        self.assertEqual(len(grid), 21)


if __name__ == "__main__":
    unittest.main()
