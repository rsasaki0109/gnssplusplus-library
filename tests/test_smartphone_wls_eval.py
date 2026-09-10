from __future__ import annotations

from pathlib import Path
import sys
import unittest


ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "apps" / "commands"))
sys.path.insert(0, str(ROOT / "apps" / "commands" / "benchmarks"))
import gnss_smartphone_wls_eval as EVAL  # noqa: E402


class SmartphoneWlsEvalTests(unittest.TestCase):
    def test_selection_record_roles_and_candidate_set_are_frozen(self) -> None:
        record = EVAL._load_selection_record(EVAL.DEFAULT_SELECTION_RECORD)
        roles = record["fixed_roles"]
        self.assertEqual(tuple(roles["candidate_train"]), EVAL.TRAIN_IDS)
        self.assertEqual(tuple(roles["validation"]), EVAL.VALIDATION_IDS)
        self.assertEqual(tuple(roles["development_main_regression"]), (EVAL.MAIN_ID,))
        self.assertEqual(tuple(roles["additional_truth_opened_audit"]), EVAL.AUDIT_IDS)
        self.assertEqual(record["sealed_data_policy"]["designated_holdout_id"], EVAL.HOLDOUT_ID)
        self.assertFalse(record["sealed_data_policy"]["holdout_content_opened"])
        self.assertEqual(
            [candidate["id"] for candidate in record["candidate_set"]],
            [candidate["id"] for candidate in EVAL.CANDIDATES],
        )

    def test_route_specs_are_all_non_holdout_and_fixed(self) -> None:
        specs = EVAL._route_specs()
        self.assertEqual(tuple(specs), EVAL.ALL_IDS)
        self.assertNotIn(EVAL.HOLDOUT_ID, specs)
        self.assertEqual(specs[EVAL.MAIN_ID].role, "development-main-regression")
        self.assertEqual(
            {spec.role for dataset_id, spec in specs.items() if dataset_id in EVAL.AUDIT_IDS},
            {"additional-truth-opened-audit"},
        )


if __name__ == "__main__":
    unittest.main()
