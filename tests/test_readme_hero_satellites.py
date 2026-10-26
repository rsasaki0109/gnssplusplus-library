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
        self.assertEqual(len(cohort), 25)
        self.assertEqual(len(self.frames), 201)
        for frame in self.frames:
            with self.subTest(time=frame["time"]):
                rows = frame["satellites"]
                self.assertEqual(len(rows), len(cohort))
                self.assertEqual({r["id"] for r in rows}, cohort)
                self.assertTrue(all(r["el_deg"] > 0 for r in rows))
                self.assertTrue(all(type(r["observed"]) is bool for r in rows))

    def test_c04_los_is_not_hidden_when_its_observation_drops(self):
        # Public Odaiba regression: C04 (GEO) disappears from the RINEX epoch at
        # 273885.0, even though its geometry remains LOS and nearly stationary.
        before = next(
            r for r in self.frame_at(273884.9)["satellites"] if r["id"] == "C04"
        )
        after = next(
            r for r in self.frame_at(273885.0)["satellites"] if r["id"] == "C04"
        )
        self.assertTrue(before["observed"])
        self.assertFalse(after["observed"])
        self.assertEqual(before["model_status"], "LOS")
        self.assertEqual(after["model_status"], "LOS")
        self.assertLess(
            math.hypot(after["x"] - before["x"], after["y"] - before["y"]), 0.01
        )

    def test_bridges_block_satellites_in_this_clip(self):
        # The elevated structure along this road is in PLATEAU's bridge class,
        # not the building class. With buildings alone it would stay model LOS.
        self.assertTrue(self.sky["model"]["classes"]["brid"])
        nlos = [
            r for f in self.frames for r in f["satellites"] if r["model_status"] == "NLOS"
        ]
        self.assertGreaterEqual(sum(r["hit_class"] == "brid" for r in nlos), 1000)
        start = self.frame_at(273874.0)["satellites"]
        self.assertGreaterEqual(
            sum(r["model_status"] == "NLOS" and r["hit_class"] == "brid" for r in start), 8
        )

    def test_model_visibility_agrees_with_reception(self):
        # Model LOS samples are mostly reported and model NLOS samples mostly
        # not; the annotations remain predictions, not measured labels.
        rows = [r for f in self.frames for r in f["satellites"]]
        los = [r for r in rows if r["model_status"] == "LOS"]
        nlos = [r for r in rows if r["model_status"] == "NLOS"]
        self.assertGreater(sum(r["observed"] for r in los) / len(los), 0.8)
        self.assertLess(sum(r["observed"] for r in nlos) / len(nlos), 0.2)

    def test_observation_availability_stays_independent_of_model_visibility(self):
        rows = [r for f in self.frames for r in f["satellites"]]
        self.assertEqual(len(rows), 5025)
        self.assertEqual(sum(r["observed"] for r in rows), 2803)
        missing = [r for r in rows if not r["observed"]]
        self.assertEqual(len(missing), 2222)
        self.assertTrue(any(r["model_status"] == "LOS" for r in missing))
        self.assertTrue(any(r["model_status"] == "NLOS" for r in missing))


if __name__ == "__main__":
    unittest.main()
