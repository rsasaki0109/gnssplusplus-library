import copy
from pathlib import Path
import sys
import unittest

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "scripts" / "analysis"))
from audit_raw_clock_cleanup import audit_raw_clock_cleanup


class RawClockCleanupAuditTest(unittest.TestCase):
    def setUp(self):
        self.keys = list(range(1000, 9000, 1000))
        self.originals = [float("nan")] * 5 + [4.49688687, 4.49688687, 3.9]
        self.proof = {"truth_used": False, "jump_masks": 0, "filled_values": 5, "epochs": []}
        self.seeds = {"clock_rate_source": "source-raw-clock-drift-jump-mask-fill", "seeds": []}
        for i, key in enumerate(self.keys):
            donor = max(i, 5)
            value = self.originals[donor]
            self.proof["epochs"].append({"utc": key, "raw_source_index": i,
                "original_drift_mps": None if i < 5 else value,
                "selected_drift_mps": value, "left_epoch_index": donor,
                "right_epoch_index": donor, "right_weight": 0.})
            self.seeds["seeds"].append({"raw_source_index": i, "selected_clock_rate_mps": value})

    def test_actual_leading_missing_pattern(self):
        result = audit_raw_clock_cleanup(self.keys, self.originals, self.proof, self.seeds)
        self.assertEqual(result["filled_values"], 5)
        self.assertFalse(result["output_coordinate_interpolation"])

    def test_changed_original_donor_value_or_epoch_is_rejected(self):
        for key, value in [("original_drift_mps", 1.), ("selected_drift_mps", 0.),
                           ("left_epoch_index", 6), ("utc", 999), ("right_weight", 1.)]:
            proof = copy.deepcopy(self.proof)
            proof["epochs"][0][key] = value
            with self.subTest(field=key), self.assertRaises(ValueError):
                audit_raw_clock_cleanup(self.keys, self.originals, proof, self.seeds)

    def test_seed_must_use_verified_clock_rate_and_disclose_preprocessing(self):
        for change in ["value", "source"]:
            seeds = copy.deepcopy(self.seeds)
            if change == "value": seeds["seeds"][0]["selected_clock_rate_mps"] = 0.
            else: seeds["clock_rate_source"] = "exact-original-epoch-receiver_clock_drift_mps"
            with self.subTest(change=change), self.assertRaises(ValueError):
                audit_raw_clock_cleanup(self.keys, self.originals, self.proof, seeds)

    def test_no_finite_support_fails(self):
        with self.assertRaisesRegex(ValueError, "no finite support"):
            audit_raw_clock_cleanup(self.keys, [float("nan")]*8, self.proof, self.seeds)

    def test_old_binary_without_numeric_seed_export_cannot_pass(self):
        seeds = copy.deepcopy(self.seeds)
        del seeds["seeds"][0]["selected_clock_rate_mps"]
        with self.assertRaisesRegex(ValueError, "numeric seed export missing"):
            audit_raw_clock_cleanup(self.keys, self.originals, self.proof, seeds)

    def test_jump_mask_requires_linear_donors_not_nearest(self):
        originals = [0., 1., 200., 3., 4.]
        proof = {"truth_used": False, "jump_masks": 3, "filled_values": 3, "epochs": []}
        seeds = {"clock_rate_source": self.seeds["clock_rate_source"], "seeds": []}
        for i in range(5):
            proof["epochs"].append({"utc": i, "raw_source_index": i,
                "original_drift_mps": originals[i], "selected_drift_mps": float(i),
                "left_epoch_index": i if i in (0, 4) else 0,
                "right_epoch_index": i if i in (0, 4) else 4,
                "right_weight": 0. if i in (0, 4) else i/4})
            seeds["seeds"].append({"raw_source_index": i, "selected_clock_rate_mps": float(i)})
        self.assertEqual(audit_raw_clock_cleanup(list(range(5)), originals, proof, seeds)["jump_masks"], 3)
        proof["epochs"][2]["right_epoch_index"] = 0
        with self.assertRaises(ValueError):
            audit_raw_clock_cleanup(list(range(5)), originals, proof, seeds)
