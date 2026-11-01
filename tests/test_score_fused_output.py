"""Fused-output scoring and the per-run rule (scripts/analysis/score_fused_output.py)."""

import contextlib
import importlib.util
import io
import json
import math
from pathlib import Path
import sys
import tempfile
import unittest

ROOT = Path(__file__).resolve().parents[1]
spec = importlib.util.spec_from_file_location(
    "score_fused_output", ROOT / "scripts/analysis/score_fused_output.py"
)
scoring = importlib.util.module_from_spec(spec)
sys.modules["score_fused_output"] = scoring
spec.loader.exec_module(scoring)

WEEK = 2243
T0 = 274800.0
ORIGIN = (30.528701, 114.357105, 40.0)     # Wuhan


def ecef(lat_deg, lon_deg, h):
    a, f = 6378137.0, 1 / 298.257223563
    e2 = f * (2 - f)
    lat, lon = math.radians(lat_deg), math.radians(lon_deg)
    n = a / math.sqrt(1 - e2 * math.sin(lat) ** 2)
    return ((n + h) * math.cos(lat) * math.cos(lon), (n + h) * math.cos(lat) * math.sin(lon),
            (n * (1 - e2) + h) * math.sin(lat))


def write_reference(path, count):
    lat, lon, h = ORIGIN
    x, y, z = ecef(lat, lon, h)
    lines = ["GPS TOW (s),GPS Week,Latitude (deg),Longitude (deg),Ellipsoid Height (m),"
             "ECEF X (m),ECEF Y (m),ECEF Z (m)"]
    lines += [f"{T0 + i:.3f},{WEEK},{lat:.9f},{lon:.9f},{h:.4f},{x:.4f},{y:.4f},{z:.4f}"
              for i in range(count)]
    path.write_text("\n".join(lines) + "\n", encoding="utf-8")


def write_pos(path, up_errors, start=0, offset_s=0.0):
    """One epoch per reference second from `start`, displaced along the local up by `up_errors`."""
    lat, lon, h = ORIGIN
    lines = ["% synthetic"]
    for i, error in enumerate(up_errors):
        x, y, z = ecef(lat, lon, h + error)
        lines.append(f"{WEEK} {T0 + start + i + offset_s:.6f} {x:.6f} {y:.6f} {z:.6f} "
                     f"{lat:.9f} {lon:.9f} {h + error:.6f} 3 10 999.9 0.0")
    path.write_text("\n".join(lines) + "\n", encoding="utf-8")


class Scoring(unittest.TestCase):
    def test_skip_matching_and_linear_percentiles(self):
        with tempfile.TemporaryDirectory() as tmp:
            reference = Path(tmp) / "reference.csv"
            write_reference(reference, 200)
            pos = Path(tmp) / "fused.pos"
            # 60 skipped epochs with a 1 km error, then 0..100 m in 1 m steps.
            write_pos(pos, [1000.0] * scoring.SKIP_EPOCHS + [float(i) for i in range(101)])
            result = scoring.score_file(pos, reference)
            self.assertEqual(result["epochs"], 101)
            self.assertAlmostEqual(result["p50_m"], 50.0, places=3)
            self.assertAlmostEqual(result["p90_m"], 90.0, places=3)

            late = Path(tmp) / "late.pos"
            write_pos(late, [0.0] * 100, offset_s=0.3)     # beyond the 0.25 s tolerance
            with self.assertRaises(ValueError):
                scoring.score_file(late, reference)

    def test_quantile_band_is_one_percent_of_the_control(self):
        self.assertEqual(scoring.compare_quantile(1.0, 0.989), "lower")
        self.assertEqual(scoring.compare_quantile(1.0, 0.991), "equal")
        self.assertEqual(scoring.compare_quantile(1.0, 1.009), "equal")
        self.assertEqual(scoring.compare_quantile(1.0, 1.011), "higher")

    def test_run_outcomes(self):
        def arm(p50, p90, epochs=1000):
            return {"p50_m": p50, "p90_m": p90, "epochs": epochs}

        outcome = lambda a, b: scoring.run_outcome(a, b)["outcome"]
        self.assertEqual(outcome(arm(2.43, 60.3), arm(0.12, 9.4)), "better")
        self.assertEqual(outcome(arm(0.117, 1.641), arm(0.105, 1.641)), "better")   # p90 equal
        self.assertEqual(outcome(arm(0.10, 1.6), arm(0.1001, 1.6)), "equal")
        self.assertEqual(outcome(arm(0.10, 1.6), arm(0.05, 1.7)), "worse")         # one higher
        self.assertEqual(outcome(arm(0.10, 1.6), arm(0.05, 1.0, 989)), "worse")    # fewer epochs
        self.assertEqual(outcome(arm(0.10, 1.6), arm(0.05, 1.0, 990)), "better")
        self.assertEqual(outcome(arm(66.2, 479), arm(140.7, 265)), "unusable")     # both p50 > 10 m
        self.assertEqual(outcome(arm(73048, 210610), arm(9.9, 126)), "better")

    def test_candidate_outcome(self):
        self.assertEqual(scoring.candidate_outcome(["better", "equal"], [True, True]), "pass")
        self.assertEqual(scoring.candidate_outcome(["better", "unusable"], [True, True]), "pass")
        self.assertEqual(scoring.candidate_outcome(["better", "worse"], [True, True]), "fail")
        self.assertEqual(scoring.candidate_outcome(["better", "better"], [True, False]), "fail")
        self.assertEqual(scoring.candidate_outcome(["equal", "unusable"], [True, True]), "inconclusive")
        self.assertEqual(scoring.candidate_outcome(["unusable", "unusable"], [True, True]), "inconclusive")

    def test_cli(self):
        with tempfile.TemporaryDirectory() as tmp:
            reference = Path(tmp) / "reference.csv"
            write_reference(reference, 200)
            control, candidate = Path(tmp) / "control.pos", Path(tmp) / "candidate.pos"
            write_pos(control, [float(i % 10) for i in range(160)])
            write_pos(candidate, [float(i % 10) / 2 for i in range(160)])
            stdout = io.StringIO()
            with contextlib.redirect_stdout(stdout):
                code = scoring.main(["--reference", str(reference), "--control", str(control),
                                     "--candidate", str(candidate)])
            self.assertEqual(code, 0)
            report = json.loads(stdout.getvalue())
            self.assertEqual(report["rule"]["outcome"], "better")
            self.assertEqual(report["control"]["epochs"], 100)

            with contextlib.redirect_stderr(io.StringIO()):
                self.assertEqual(scoring.main(["--reference", str(reference),
                                               "--pos", str(Path(tmp) / "missing.pos")]), 2)
                with self.assertRaises(SystemExit):
                    scoring.main(["--reference", str(reference), "--control", str(control)])


if __name__ == "__main__":
    unittest.main()
