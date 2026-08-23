from __future__ import annotations

from pathlib import Path
import sys
import unittest


ROOT_DIR = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT_DIR / "apps" / "commands"))

from support.gnss_static_metrics import parse_ecef, static_truth_metrics  # noqa: E402


class StaticMetricsTest(unittest.TestCase):
    def test_parse_ecef_is_fail_closed(self) -> None:
        self.assertEqual(parse_ecef("1,2,3"), (1.0, 2.0, 3.0))
        for invalid in ("1,2", "1,2,nan", "1,2,inf", "x,2,3"):
            with self.subTest(invalid=invalid), self.assertRaises(ValueError):
                parse_ecef(invalid)

    def test_filters_population_and_reports_empirical_covariance(self) -> None:
        truth = (-3957184.9682679, 3310231.00877522, 3737703.81925572)
        rows = [
            {"x": truth[0] + 0.1, "y": truth[1], "z": truth[2], "status": 4},
            {"x": truth[0] - 0.1, "y": truth[1], "z": truth[2], "status": 4},
            {"x": truth[0] + 100.0, "y": truth[1], "z": truth[2], "status": 3},
        ]
        result = static_truth_metrics(rows, truth, accepted_statuses={4})
        self.assertEqual(result["population"]["accepted_epochs"], 2)
        self.assertAlmostEqual(result["error_3d_m"], 0.0, places=8)
        self.assertAlmostEqual(result["empirical_covariance_ecef_m2"][0][0], 0.02, places=8)
        self.assertIn("not solver posterior", result["covariance_role"])


if __name__ == "__main__":
    unittest.main()
