"""Regressions for geometric sky visibility surviving receiver observation gaps."""

import json
import math
from pathlib import Path
import unittest

ROOT = Path(__file__).resolve().parents[1]


class SkySatelliteContinuityTest(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.sky = json.loads((ROOT / "docs/odaiba_hero_sky.json").read_text())
        cls.frames = cls.sky["frames"]

    def frame_at(self, tow):
        return next(f for f in self.frames if abs(f["time"] % 604800 - tow) < 0.001)

    def test_short_clip_keeps_every_above_horizon_satellite(self):
        cohort = set(self.sky["cohort_ids"])
        self.assertEqual(len(cohort), 32)
        self.assertEqual(len(self.frames), 201)
        for frame in self.frames:
            with self.subTest(time=frame["time"]):
                rows = frame["satellites"]
                self.assertEqual(len(rows), len(cohort))
                self.assertEqual({r["id"] for r in rows}, cohort)
                self.assertTrue(all(r["el_deg"] > 0 for r in rows))
                self.assertTrue(all(type(r["observed"]) is bool for r in rows))

    def test_c04_los_is_not_hidden_when_its_observation_drops(self):
        # Public Odaiba regression: C04 disappears from the RINEX epoch at
        # 273700.4, even though its geometry remains LOS and nearly stationary.
        before = next(
            r for r in self.frame_at(273700.3)["satellites"] if r["id"] == "C04"
        )
        after = next(
            r for r in self.frame_at(273700.4)["satellites"] if r["id"] == "C04"
        )
        self.assertTrue(before["observed"])
        self.assertFalse(after["observed"])
        self.assertEqual(before["model_status"], "LOS")
        self.assertEqual(after["model_status"], "LOS")
        self.assertLess(
            math.hypot(after["x"] - before["x"], after["y"] - before["y"]), 0.01
        )

    def test_observation_availability_stays_independent_of_model_visibility(self):
        rows = [r for f in self.frames for r in f["satellites"]]
        self.assertEqual(len(rows), 6432)
        self.assertEqual(sum(r["observed"] for r in rows), 4711)
        missing = [r for r in rows if not r["observed"]]
        self.assertEqual(len(missing), 1721)
        self.assertTrue(any(r["model_status"] == "LOS" for r in missing))
        self.assertTrue(any(r["model_status"] == "NLOS" for r in missing))


if __name__ == "__main__":
    unittest.main()
