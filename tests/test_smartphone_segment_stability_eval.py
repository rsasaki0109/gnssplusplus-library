from __future__ import annotations

from pathlib import Path
import sys
import unittest


ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "apps" / "commands"))
sys.path.insert(0, str(ROOT / "apps" / "commands" / "benchmarks"))
import gnss_smartphone_segment_stability_eval as EVAL  # noqa: E402


class SmartphoneSegmentStabilityEvalTests(unittest.TestCase):
    def test_selection_record_and_roles_are_frozen(self) -> None:
        record = EVAL._load_selection_record(EVAL.DEFAULT_SELECTION_RECORD)
        self.assertEqual(
            tuple(record["fixed_roles"]["candidate_train"]),
            EVAL.TRAIN_EVAL_IDS,
        )
        self.assertEqual(
            record["new_validation_selection"]["dataset_id"],
            EVAL.NEW_VALIDATION_ID,
        )
        self.assertEqual(
            [candidate["id"] for candidate in record["candidate_set"]],
            ["segment_r15_d15", "segment_r20_d20", "segment_r30_d30"],
        )
        self.assertFalse(record["sealed_data_policy"]["holdout_content_opened"])

    def test_segment_gate_does_not_use_reject_fraction_by_default(self) -> None:
        self.assertTrue(all(candidate["reject_fraction_max"] is None for candidate in EVAL.CANDIDATES))
        self.assertEqual(
            EVAL.PREVIOUSLY_USED_IDS[-1],
            "2021-03-16-20-40-us-ca-mtv-b/pixel4xl",
        )


if __name__ == "__main__":
    unittest.main()
