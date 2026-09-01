from __future__ import annotations

import csv
import io
import json
from pathlib import Path
import tempfile
import unittest
import zipfile
import sys


ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "apps" / "commands"))
sys.path.insert(0, str(ROOT / "apps" / "commands" / "benchmarks"))
import gnss_smartphone_generalization as GENERALIZATION  # noqa: E402
import gnss_smartphone_reacquisition_conservative_eval as EVAL  # noqa: E402


class SmartphoneConservativeReacquisitionTests(unittest.TestCase):
    def test_selection_record_freezes_new_route_and_candidates(self) -> None:
        record = EVAL._load_selection_record(EVAL.DEFAULT_SELECTION_RECORD)
        self.assertEqual(
            record["new_validation_selection"]["dataset_id"],
            EVAL.NEW_VALIDATION_ID,
        )
        self.assertEqual(
            tuple(item["id"] for item in record["candidate_set"]),
            ("reacq_r15_d15", "reacq_r20_d20", "reacq_r30_d30"),
        )
        self.assertFalse(record["sealed_data_policy"]["holdout_content_opened"])

    def test_signal_only_screen_reads_selected_device_without_truth_member(self) -> None:
        route, phone = EVAL.NEW_VALIDATION_ID.split("/", 1)
        member = GENERALIZATION._member_names(route, phone)["device_gnss"]
        fields = (
            "SignalType",
            "ConstellationType",
            "CarrierFrequencyHz",
            "CodeType",
            "Svid",
            "RawPseudorangeMeters",
            "utcTimeMillis",
        )
        buffer = io.StringIO(newline="")
        writer = csv.DictWriter(buffer, fieldnames=fields, lineterminator="\n")
        writer.writeheader()
        writer.writerow(
            {
                "SignalType": "GAL_E1_C_P",
                "ConstellationType": "6",
                "CarrierFrequencyHz": "1575420000",
                "CodeType": "",
                "Svid": "2",
                "RawPseudorangeMeters": "24000000",
                "utcTimeMillis": "1615927201435",
            }
        )
        row = {
            "dataset_id": EVAL.NEW_VALIDATION_ID,
            "required_files_complete": True,
            "broadcast_nav_present": True,
            "route": route,
            "phone": phone,
        }
        inventory = {"train": {"records": [row]}}
        with tempfile.TemporaryDirectory(prefix="reacq_conservative_screen_") as temp:
            archive_path = Path(temp) / "screen.zip"
            with zipfile.ZipFile(archive_path, "w") as archive:
                archive.writestr(member, buffer.getvalue())
            result = EVAL._signal_only_screen(archive_path, inventory, EVAL.NEW_VALIDATION_ID)
        self.assertEqual(result["valid_galileo_e1_rows"], 1)
        self.assertEqual(result["galileo_e1_epochs"], 1)
        self.assertFalse(result["truth_opened"])

    def test_main_gate_rejects_non_byte_identical_candidate(self) -> None:
        def score(h50: float, h95: float, v95: float, diagnostic: float) -> dict:
            return {
                "availability_ratio": 1.0,
                "truth_coverage_ratio": 1.0,
                "horizontal_wgs84_m": {"p50_m": h50, "p95_m": h95},
                "horizontal_haversine_m": {"p50_m": h50, "p95_m": h95},
                "vertical_p95_abs_m": v95,
                "kaggle_diagnostic_score_variants_m": {
                    key: diagnostic for key in EVAL.DIAGNOSTIC_KEYS
                },
            }

        existing_train = {EVAL.TRAIN_IDS[0]: score(10, 30, 20, 15), EVAL.TRAIN_IDS[1]: score(10, 30, 20, 15)}
        candidate_train = {EVAL.TRAIN_IDS[0]: score(9, 10, 10, 5), EVAL.TRAIN_IDS[1]: score(9, 10, 10, 5)}
        train_scores = {"existing-smoother": existing_train}
        for candidate in EVAL.CANDIDATES:
            train_scores[candidate["id"]] = candidate_train
        validation_raw = {EVAL.NEW_VALIDATION_ID: score(5, 20, 10, 10)}
        validation_existing = {EVAL.NEW_VALIDATION_ID: score(6, 50, 20, 30)}
        validation_scores = {
            candidate["id"]: {EVAL.NEW_VALIDATION_ID: score(5, 15, 10, 10)}
            for candidate in EVAL.CANDIDATES
        }
        main_existing = {EVAL.MAIN_ID: score(5, 20, 10, 10)}
        main_scores = {
            candidate["id"]: {EVAL.MAIN_ID: score(5, 20, 10, 10)}
            for candidate in EVAL.CANDIDATES
        }
        report, selected = EVAL._candidate_report(
            train_scores,
            validation_scores,
            validation_raw,
            validation_existing,
            main_scores,
            main_existing,
            {candidate["id"]: False for candidate in EVAL.CANDIDATES},
            {candidate["id"]: 1 for candidate in EVAL.CANDIDATES},
        )
        self.assertEqual(selected, "reacq_r15_d15")
        self.assertEqual(report["promotion_decision"], "no-go-development-main-contract-or-regression")
        self.assertFalse(report["development_main_gate"]["pos_byte_identical"])
        self.assertFalse(report["development_main_gate"]["reacquisition_count_zero"])


if __name__ == "__main__":
    unittest.main()
