import copy
from pathlib import Path
import sys
import unittest

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "scripts" / "analysis"))
from summarize_gsdc_train_validation import summarize


class EvaluationGroupingTest(unittest.TestCase):
    def setUp(self):
        self.plan = {"runs": {"a/pixel5": {"route_group": "a"},
                              "b/pixel7pro": {"route_group": "b"}}}
        self.validation = {"runs": {
            case: {"route_group": entry["route_group"], "status": "not-started"}
            for case, entry in self.plan["runs"].items()}}

    def test_merge_retains_both_phones_and_incomplete_status(self):
        before = copy.deepcopy(self.plan)
        result = summarize(self.plan, self.validation,
                           {case: "same-drive" for case in self.plan["runs"]})
        self.assertEqual(result["planned_route_groups"], 1)
        self.assertEqual(result["route_groups"]["same-drive"]["planned_drives"], 2)
        self.assertEqual(set(result["phones"]), {"pixel5", "pixel7pro"})
        self.assertIsNone(result["full_scope_mean_route_group_score_m"])
        self.assertEqual(self.plan, before)
        self.assertEqual(summarize(self.plan, self.validation)["planned_route_groups"], 2)

    def test_rejects_splitting_existing_group_or_unknown_case(self):
        self.plan["runs"]["b/pixel7pro"]["route_group"] = "a"
        self.validation["runs"]["b/pixel7pro"]["route_group"] = "a"
        with self.assertRaisesRegex(ValueError, "cannot split"):
            summarize(self.plan, self.validation, {"a/pixel5": "changed"})
        with self.assertRaisesRegex(ValueError, "unknown cases"):
            summarize(self.plan, self.validation, {"missing/phone": "a"})

    def test_override_cannot_hide_changed_validation_group(self):
        self.validation["runs"]["b/pixel7pro"]["route_group"] = "wrong"
        with self.assertRaisesRegex(ValueError, "route group changed"):
            summarize(self.plan, self.validation,
                      {case: "same-drive" for case in self.plan["runs"]})
