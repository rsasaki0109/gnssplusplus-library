"""Runtime CLI rejection tests; no raw or truth paths are supplied."""

import os
from pathlib import Path
import subprocess
import unittest


ROOT = Path(__file__).resolve().parents[1]
APP = ROOT / "build/apps/gnss_fgo_imu_no_base"
FLAG = "--native-phase205-source-count-bias-density"
LANE = [
    "--native-phase165-raw-p-no-doppler-graph",
    "--native-phase167-raw-p-no-doppler-lm-termination-budget",
    "--native-phase171-raw-p-no-doppler-imu-main",
    "--native-phase171-raw-p-ecef-doppler-gnss-first",
    "--android-utc-wall-clock-fallback",
]


class BiasDensityCliTest(unittest.TestCase):
    def reject(self, args):
        env = os.environ.copy()
        env["LD_LIBRARY_PATH"] = "/home/sasaki/.local/lib:" + env.get("LD_LIBRARY_PATH", "")
        result = subprocess.run([str(APP), *args], cwd=ROOT, env=env,
                                capture_output=True, text=True, timeout=10)
        self.assertEqual(result.returncode, 2, result.stdout + result.stderr)
        self.assertIn("Phase205 requires", result.stderr)
        self.assertNotIn("Unknown argument", result.stderr)

    def test_missing_lane(self):
        self.reject([FLAG])

    def test_wrong_phone(self):
        self.reject(["--dataset-id", "synthetic/pixel4", *LANE, FLAG])

    def test_incompatible_schedule(self):
        self.reject(["--dataset-id", "synthetic/pixel5", *LANE, FLAG,
                     "--native-phase201-source-inclusive-forward-imu-schedule"])

    def test_missing_utc_fallback(self):
        self.reject(["--dataset-id", "synthetic/pixel5", *LANE[:-1], FLAG])


if __name__ == "__main__":
    unittest.main()
