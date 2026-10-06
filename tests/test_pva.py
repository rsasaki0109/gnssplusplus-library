"""Independent rotation/time/availability witnesses; no field fixture needed."""
import csv
import math
from pathlib import Path
import subprocess
import sys
import tempfile
import unittest

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "apps/commands/benchmarks"))
import gnss_pva_metrics as m


def fixture(directory):
    """Project-authored snapshots exercise the scorer, not a sensor solver."""
    truth, estimates = [], []
    for i in range(12):
        heading = [359, 1, 1, 1, 1, 90, 90, 0, 0, 0, 0, 0][i]
        ve, vn = (0, 0) if i < 3 else ((0, -1) if i == 7 else (0, 1))
        truth.append(dict(zip(("GPS Week", "GPS TOW (s)", "Latitude (deg)", "Longitude (deg)",
            "ECEF X (m)", "ECEF Y (m)", "ECEF Z (m)", "Roll (deg)", "Pitch (deg)", "Heading (deg)",
            "East Velocity (m/s)", "North Velocity (m/s)", "Up Velocity (m/s)"),
            (2200, 1000+i, 0, 0, 6378137, 0, 0, 0, 0, heading, ve, vn, 0))))
        h = heading if i < 11 else 180
        a = math.radians(90-h)/2
        row = dict(rover_week=2200, rover_tow=1000+i, attitude_week=2200, attitude_tow=1000+i,
                   attitude_available=int(i > 0), heading_aligned=int(i > 1), heading_converged=int(1 < i < 11),
                   fusion_initialized=int(i > 0), gnss_position_updated=int(i > 0), reset_generation=0,
                   processing_ms=.5, qw=math.cos(a), qx=0, qy=0, qz=math.sin(a))
        for prefix in ("rtk", "fused"):
            row.update({prefix+"_status": 1 if prefix == "rtk" or i > 0 else 0, prefix+"_week": 2200,
                prefix+"_tow": 1000+i, prefix+"_x_m": 6378137, prefix+"_y_m": 0, prefix+"_z_m": 0,
                prefix+"_has_velocity": int(prefix == "rtk" or i > 0), prefix+"_vx_mps": 0,
                prefix+"_vy_mps": ve, prefix+"_vz_mps": vn})
        fixed = [[0, 1, 0], [0, 0, 1], [1, 0, 0]]
        for r in range(3):
            for c in range(3): row[f"ecef_to_enu_{r}{c}"] = fixed[r][c]
        if i == 0:
            for k in ("qw", "qx", "qy", "qz"): row[k] = "nan"
        estimates.append(row)
    paths = directory/"synthetic_pva.csv", directory/"synthetic_reference.csv"
    for path, rows in zip(paths, (estimates, truth)):
        write(path, rows)
    return paths


def write(path, rows):
    with path.open("w", encoding="utf-8", newline="") as f:
        writer = csv.DictWriter(f, fieldnames=list(rows[0]))
        writer.writeheader()
        writer.writerows(rows)


class PvaTest(unittest.TestCase):
    def setUp(self):
        self.temp = tempfile.TemporaryDirectory()
        self.addCleanup(self.temp.cleanup)
        self.directory = Path(self.temp.name)
        self.estimate, self.reference = fixture(self.directory)

    def test_missing_heading_health_does_not_hide_half_turn(self):
        result, rows = m.score(self.estimate, self.reference)
        self.assertEqual(result["epochs"], 12)
        self.assertEqual(result["scenes"]["all"]["metrics"]["heading_deg"]["count"], 10)
        self.assertAlmostEqual(result["scenes"]["all"]["metrics"]["heading_deg"]["max"], 180)
        self.assertAlmostEqual(result["healthy_secondary"]["heading_deg"]["max"], 0, places=6)
        self.assertEqual(result["heading_error_over_150_epochs"], 1)
        self.assertAlmostEqual(result["coverage"]["attitude_available"], 11/12)
        self.assertEqual(result["generations"][0]["first_heading_s"], 2)
        self.assertEqual(result["scenes"]["reverse"]["epochs"], 1)

    def test_angle_wrap_and_quaternion_sign(self):
        self.assertEqual(m.circular(359-1), -2)
        q = [math.sqrt(.5), 0, 0, math.sqrt(.5)]
        self.assertEqual(m.quaternion_matrix(q), m.quaternion_matrix([-v for v in q]))
        self.assertAlmostEqual(m.rotation_error(m.euler_matrix(0,0,359), m.euler_matrix(0,0,1)), 2)

    def test_fixed_to_current_frame_transport(self):
        fixed, current = m.enu(35, 139), m.enu(36, 140)
        expected = m.euler_matrix(17, -23, 312)
        body_fixed = m.multiply(m.multiply(fixed, m.transpose(current)), m.multiply(m.multiply(m.P, expected), m.D))
        restored = m.multiply(m.multiply(m.P, m.multiply(m.multiply(current, m.transpose(fixed)), body_fixed)), m.D)
        self.assertLess(m.rotation_error(restored, expected), 1e-5)
        for a, b in zip(m.matrix_euler(restored), (17, -23, 312)): self.assertAlmostEqual(a, b)

    def test_invalid_available_quaternion_time_and_rotation_are_rejected(self):
        original = m.read_rows(self.estimate)
        for key, value in (("qw", "2"), ("attitude_tow", "1010"), ("ecef_to_enu_00", "1")):
            rows = [dict(r) for r in original]
            rows[3][key] = value
            write(self.estimate, rows)
            with self.assertRaises(ValueError): m.score(self.estimate, self.reference)

    def test_duplicate_time_is_rejected_and_no_match_fails(self):
        rows = m.read_rows(self.reference)
        rows[1]["GPS TOW (s)"] = rows[0]["GPS TOW (s)"]
        write(self.reference, rows)
        with self.assertRaises(ValueError): m.score(self.estimate, self.reference)
        _, self.reference = fixture(self.directory)
        rows = m.read_rows(self.reference)
        for r in rows: r["GPS Week"] = "2201"
        write(self.reference, rows)
        with self.assertRaisesRegex(ValueError, "no estimate timestamps"): m.score(self.estimate, self.reference)

    def test_empty_stats_and_partial_truth_are_explicit(self):
        self.assertIsNone(m.stats([])["rmse"])
        rows = m.read_rows(self.reference)[2:]
        write(self.reference, rows)
        report, _ = m.score(self.estimate, self.reference)
        self.assertEqual(report["missing_truth_epochs"], 2)
        self.assertAlmostEqual(report["match_fraction"], 10/12)

    def test_dispatcher_score_only_and_no_overwrite(self):
        out = self.directory/"report"
        command = [sys.executable, str(ROOT/"apps/gnss.py"), "pva-evaluate", "--estimate", str(self.estimate),
                   "--reference", str(self.reference), "--output-dir", str(out)]
        first = subprocess.run(command, capture_output=True, text=True)
        self.assertEqual(first.returncode, 0, first.stderr)
        self.assertTrue((out/"manifest.json").exists())
        self.assertNotEqual(subprocess.run(command, capture_output=True).returncode, 0)


if __name__ == "__main__":
    unittest.main()
