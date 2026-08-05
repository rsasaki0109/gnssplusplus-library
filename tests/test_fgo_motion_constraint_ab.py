import csv
import importlib.util
import tempfile
import unittest
from pathlib import Path


SCRIPT = Path(__file__).parents[1] / "scripts" / "analyze_fgo_motion_constraint_ab.py"
SPEC = importlib.util.spec_from_file_location("motion_ab", SCRIPT)
assert SPEC and SPEC.loader
MODULE = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(MODULE)


class MotionConstraintAbTest(unittest.TestCase):
    def write_rows(self, path: Path, rows: list[dict[str, object]]) -> None:
        with path.open("w", newline="", encoding="utf-8") as stream:
            writer = csv.DictWriter(stream, fieldnames=rows[0].keys())
            writer.writeheader()
            writer.writerows(rows)

    def test_compare_rejects_added_wrong_fix(self) -> None:
        fields = {
            "n_err_m": 0.0, "u_err_m": 0.0, "zupt_candidate": 0,
            "zupt_applied": 0, "nhc_candidate": 0, "nhc_applied": 0,
        }
        baseline_rows = [
            {"tow": 1, "status": "FIXED", "e_err_m": 0.1, "horiz_err_m": 0.1, **fields},
            {"tow": 2, "status": "FLOAT", "e_err_m": 2.0, "horiz_err_m": 2.0, **fields},
        ]
        variant_rows = [
            baseline_rows[0],
            {"tow": 2, "status": "FIXED", "e_err_m": 2.0, "horiz_err_m": 2.0, **fields},
        ]
        with tempfile.TemporaryDirectory() as directory:
            base_path = Path(directory) / "base.csv"
            variant_path = Path(directory) / "variant.csv"
            self.write_rows(base_path, baseline_rows)
            self.write_rows(variant_path, variant_rows)
            result = MODULE.compare(MODULE.load(base_path), MODULE.load(variant_path), 10.0, 10.0)
        self.assertEqual(result["added_wrong_fix"], 1)
        self.assertFalse(result["passes"])

    def test_compare_accepts_three_safe_added_fixes(self) -> None:
        fields = {
            "n_err_m": 0.0, "u_err_m": 0.0, "zupt_candidate": 0,
            "zupt_applied": 0, "nhc_candidate": 0, "nhc_applied": 0,
        }
        baseline_rows = [
            {"tow": 1, "status": "FIXED", "e_err_m": 0.1, "horiz_err_m": 0.1, **fields},
            *[
                {"tow": tow, "status": "FLOAT", "e_err_m": 0.1, "horiz_err_m": 0.1, **fields}
                for tow in (2, 3, 4)
            ],
        ]
        variant_rows = [dict(row, status="FIXED") for row in baseline_rows]
        with tempfile.TemporaryDirectory() as directory:
            base_path = Path(directory) / "base.csv"
            variant_path = Path(directory) / "variant.csv"
            self.write_rows(base_path, baseline_rows)
            self.write_rows(variant_path, variant_rows)
            result = MODULE.compare(MODULE.load(base_path), MODULE.load(variant_path), 10.0, 10.5)
        self.assertEqual(result["added_correct_fix"], 3)
        self.assertTrue(result["passes"])


if __name__ == "__main__":
    unittest.main()
