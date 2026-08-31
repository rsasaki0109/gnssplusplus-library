from __future__ import annotations

import csv
import json
from pathlib import Path
import tempfile
import unittest

ROOT = Path(__file__).resolve().parents[1]
import sys

sys.path.insert(0, str(ROOT / "apps" / "commands" / "benchmarks"))
import gnss_smartphone_phone_data_mat as converter  # noqa: E402


class PhoneDataMatConverterTests(unittest.TestCase):
    def test_numeric_accel_gyro_fields_convert_to_loadable_csv(self) -> None:
        try:
            import numpy as np
            from scipy.io import savemat
        except ImportError as exc:  # pragma: no cover - CI environment choice
            self.skipTest(f"SciPy unavailable: {exc}")
        with tempfile.TemporaryDirectory(prefix="phone_data_mat_test_") as directory:
            root = Path(directory)
            source = root / "phone_data.mat"
            output = root / "imu_processed.csv"
            savemat(
                source,
                {
                    "acc": {
                        "n": 3,
                        "utcms": np.array([1_700_000_000_000, 1_700_000_000_010, 1_700_000_000_020]),
                        "xyz": np.array([[0.0, 0.0, 9.8], [0.1, 0.0, 9.8], [0.2, 0.0, 9.8]]),
                    },
                    "gyro": {
                        "n": 2,
                        "utcms": np.array([1_700_000_000_005, 1_700_000_000_015]),
                        "xyz": np.array([[1.0, 0.0, 0.0], [0.0, 2.0, 0.0]]),
                    },
                },
            )
            report = converter.convert_mat(source, output)
            self.assertEqual(report["paired_rows"], 2)
            self.assertEqual(report["omitted_outside_25ms"], 0)
            with output.open(newline="", encoding="ascii") as handle:
                rows = list(csv.DictReader(handle))
            self.assertEqual(len(rows), 2)
            self.assertAlmostEqual(float(rows[0]["Acc Z (m/s^2)"]), 9.8)
            self.assertAlmostEqual(float(rows[0]["Ang Rate X (deg/s)"]), 57.2957795, places=5)
            # UTC -> GPST adds the frozen 18 seconds, so the resulting week/TOW
            # is stable and independently checkable without MATLAB objects.
            self.assertEqual(rows[0]["GPS Week"], "2288")

    def test_inconsistent_duplicate_timestamp_fails_closed(self) -> None:
        try:
            import numpy as np
            from scipy.io import savemat
        except ImportError as exc:  # pragma: no cover
            self.skipTest(f"SciPy unavailable: {exc}")
        with tempfile.TemporaryDirectory(prefix="phone_data_mat_bad_") as directory:
            source = Path(directory) / "phone_data.mat"
            output = Path(directory) / "imu.csv"
            savemat(
                source,
                {
                    "acc": {
                        "utcms": np.array([1_700_000_000_000, 1_700_000_000_000]),
                        "xyz": np.array([[0.0, 0.0, 9.8], [1.0, 0.0, 9.8]]),
                    },
                    "gyro": {
                        "utcms": np.array([1_700_000_000_000]),
                        "xyz": np.array([[0.0, 0.0, 0.0]]),
                    },
                },
            )
            with self.assertRaises(converter.MatImuError):
                converter.convert_mat(source, output)

    def test_freeze_and_native_entrypoint_are_no_base_and_explicitly_mounted(self) -> None:
        freeze = json.loads(
            (ROOT / "docs/use_cases/records/smartphone_r5_gsdc2023_native_fgo_v2_mat_no_base_freeze.json").read_text()
        )
        self.assertEqual(freeze["status"], "frozen-before-entrypoint-implementation")
        self.assertEqual(freeze["upstream_contracts"]["license"], "MIT")
        self.assertEqual(freeze["upstream_contracts"]["gtsam_gnss_license"], "MIT")
        self.assertFalse(freeze["truth_policy"]["validation_or_holdout_materialized"])
        source = (ROOT / "apps/native/gnss_fgo_imu_no_base.cpp").read_text()
        self.assertIn("config.use_double_difference_factors = false", source)
        self.assertIn("CombinedImuFactor", ROOT.joinpath("src/algorithms/fgo_gtsam_backend.cpp").read_text())
        self.assertIn("Eigen::AngleAxisd", source)
        self.assertIn("-85.0, 178.0, -94.0", source)


if __name__ == "__main__":
    unittest.main()
