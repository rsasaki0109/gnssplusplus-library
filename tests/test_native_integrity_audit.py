"""Offline labeling must preserve missing and unrecovered event costs."""
from pathlib import Path
import sys
from types import SimpleNamespace
import unittest

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "scripts/analysis"))
import audit_ppc_native_integrity as audit


def row(tow, error, *, fixed=True):
    return {"key": (2200, tow), "stamp": 2200 * 604800.0 + tow,
            "fixed": fixed, "error_m": error, "runtime_classes": ["no_named_runtime_warning"]}


class NativeIntegrityAuditTest(unittest.TestCase):
    def test_missing_interval_is_included_in_recovery_delay_and_final_event_is_censored(self):
        rows = [row(10.0, 3), row(10.2, 3), row(12.0, 1, fixed=False),
                row(13.0, 0.1), row(14.0, 4)]
        events = audit.recovery_events(rows, 2.0)
        self.assertEqual(len(events), 2)
        self.assertEqual(events[0]["epochs"], 2)
        self.assertAlmostEqual(events[0]["recovery_delay_s"], 2.8)
        self.assertTrue(events[-1]["right_censored"])
        self.assertIsNone(events[-1]["recovery_delay_s"])

    def test_status_demotion_does_not_count_as_accuracy_recovery(self):
        baseline = [row(10, 3), row(11, 0.1), row(12, 4)]
        candidate = [row(10, 3, fixed=False), row(11, 0.1, fixed=False)]
        result = audit.compare(baseline, candidate)["2.0"]
        self.assertEqual(result["baseline_wrong_fixed_removed"], 2)
        self.assertEqual(result["baseline_wrong_epochs_now_accurate"], 0)
        self.assertEqual(result["baseline_wrong_epochs_now_missing"], 1)
        self.assertEqual(result["baseline_correct_fixed_lost"], 1)

    def test_both_thresholds_use_full_admitted_input_denominator(self):
        summary = audit.summarize([row(10, 1), row(11, 3)], 4)
        self.assertEqual(summary["missing_or_unmatched_epochs"], 2)
        self.assertEqual(summary["thresholds"]["0.5"]["wrong_fixed_epochs"], 2)
        self.assertEqual(summary["thresholds"]["2.0"]["wrong_fixed_epochs"], 1)
        with self.assertRaises(ValueError):
            audit.summarize([row(10, 1), row(11, 3)], 1)

    def test_runtime_classifier_has_no_reference_argument_and_preserves_unknowns(self):
        epoch = SimpleNamespace(nsat=14, ratio=10, prefit_rms_m=1, post_rms_m=1,
                                nis_per_obs=1, outliers=0, observations=20)
        self.assertEqual(audit.runtime_classes(epoch), ["no_named_runtime_warning"])
        epoch.prefit_rms_m = None
        self.assertEqual(audit.runtime_classes(epoch), ["incomplete_runtime_diagnostics"])

    def test_paired_adoption_rejects_partial_or_different_binary_evidence(self):
        import copy
        baseline = {"state": "passed", "evaluation": "full", "paths": ["rtk"], "runs": ["tokyo/run1"],
                    "max_epochs": -1, "fix_recovery": False, "inputs": [], "binaries": {"gnss_solve": "a"},
                    "runtime_libraries": [], "source": {"contents_sha256": "same"},
                    "build": {"settings": {}}, "steps": [], "artifacts": []}
        candidate = copy.deepcopy(baseline)
        candidate["fix_recovery"] = True
        candidate["binaries"]["gnss_solve"] = "b"
        with self.assertRaisesRegex(ValueError, "binaries"):
            audit.paired_provenance(baseline, candidate, Path("off"), Path("on"), baseline["runs"])
        candidate["binaries"] = baseline["binaries"]
        candidate["state"] = "running"
        with self.assertRaisesRegex(ValueError, "successful full"):
            audit.paired_provenance(baseline, candidate, Path("off"), Path("on"), baseline["runs"])

    def test_runtime_clean_recovery_is_separate_from_accuracy_and_preserves_censoring(self):
        import tempfile
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory) / "decisions.csv"
            path.write_text("gps_week,tow,state,demote_fixed,request_primary_reset,recovered\n"
                            "2200,10,2,1,1,0\n2200,12,0,0,0,1\n2200,15,2,1,1,0\n", encoding="utf-8")
            result = audit.recovery_decisions(path)
            self.assertEqual(result["primary_reset_requests"], 2)
            self.assertEqual(result["clean_candidate_recoveries"], 1)
            self.assertEqual(result["right_censored_quarantines"], 1)
            self.assertEqual(result["events"][0]["clean_recovery_delay_s"], 2.0)


if __name__ == "__main__":
    unittest.main()
