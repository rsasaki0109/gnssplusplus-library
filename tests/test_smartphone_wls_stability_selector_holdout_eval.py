from __future__ import annotations

import json
from pathlib import Path
import subprocess
import sys
import tempfile
import unittest


ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "apps" / "commands"))
sys.path.insert(0, str(ROOT / "apps" / "commands" / "benchmarks"))
import gnss_smartphone_wls_stability_selector_holdout_eval as HOLDOUT  # noqa: E402
import gnss_smartphone_gnss_adapter as ADAPTER  # noqa: E402
import gnss_smartphone_wls as WLS  # noqa: E402


class SmartphoneWlsStabilitySelectorHoldoutTests(unittest.TestCase):
    def test_v4_reserved_holdout_central_contract_is_truth_free(self) -> None:
        """The v4 schema guard fixes the next holdout before payload access."""

        members = {
            "device_gnss": {
                "name": "dataset_2023/train/2023-05-25-19-10-us-ca-sjc-be2/sm-s908b/device_gnss.csv",
                "file_size": 41270203,
                "compressed_size": 9386392,
                "crc32_hex": "2c3681dc",
            },
            "broadcast_nav": {
                "name": "dataset_2023/train/2023-05-25-19-10-us-ca-sjc-be2/brdc.nav",
                "file_size": 12040709,
                "compressed_size": 1135227,
                "crc32_hex": "49c3c738",
            },
            "ground_truth": {
                "name": "dataset_2023/train/2023-05-25-19-10-us-ca-sjc-be2/sm-s908b/ground_truth.csv",
                "file_size": 136035,
                "compressed_size": 36454,
                "crc32_hex": "df378389",
            },
        }
        freeze = {
            "schema_version": "smartphone-r5-wls-stability-selector-holdout-freeze.v4",
            "holdout": {
                "id": HOLDOUT.V4_HOLDOUT_ID,
                "device_model": "sm-s908b",
                "skip_epochs": 1,
                "central_directory_members": members,
            },
        }
        self.assertEqual(HOLDOUT._run_holdout_id(freeze), HOLDOUT.V4_HOLDOUT_ID)
        self.assertEqual(HOLDOUT._central_contract(freeze), members)
        altered = dict(freeze)
        altered["holdout"] = dict(freeze["holdout"])
        altered["holdout"]["central_directory_members"] = dict(members)
        altered["holdout"]["central_directory_members"]["device_gnss"] = dict(
            members["device_gnss"], crc32_hex="not-crc"
        )
        with self.assertRaises(HOLDOUT.HoldoutEvaluationError):
            HOLDOUT._central_contract(altered)

    def test_v1_freeze_hash_key_mapping_regression_fixture(self) -> None:
        freeze = json.loads(
            HOLDOUT.DEFAULT_FREEZE.read_text(encoding="utf-8")
        )
        expected = {
            "device_gnss": "f6c05015842cc0a389ed2c426b15aed980819a80a2baa828ca77bfa53aba245b",
            "broadcast_nav": "21236469a4048817d53bac9faaab04bd87558d100c56ebf8fec25740edc7f696",
            "ground_truth": "abb4076259100afa716cba5f845f285a719b19e17d16c6f2903355bbe58879a7",
        }
        for key, value in expected.items():
            self.assertEqual(HOLDOUT._frozen_holdout_hash(freeze, key), value)

    def test_v1_failure_fixture_is_sealed_without_truth_access(self) -> None:
        failure_path = (
            ROOT
            / "output"
            / "smartphone-r5"
            / "wls-stability-selector-holdout-run2-v1"
            / "wls_stability_selector_holdout_run2_failure.json"
        )
        failure = json.loads(failure_path.read_text(encoding="utf-8"))
        self.assertEqual(failure["status"], "sealed-failed-one-shot")
        self.assertEqual(failure["truth_open_count"], 0)
        self.assertFalse(failure["post_holdout_tuning"])

    def test_v3_full_relative_source_key_authorizes_adapter_and_wls_fixture(self) -> None:
        """Materialization and truth-free adapter startup honor the source key."""

        freeze = {
            "schema_version": "smartphone-r5-wls-stability-selector-holdout-freeze.v3",
            "holdout_execution_contract": {"authorized": True},
            "source_files": {
                "apps/commands/benchmarks/gnss_smartphone_gnss_adapter.py": {
                    "sha256": ADAPTER.sha256(Path(ADAPTER.__file__)),
                },
                "apps/commands/benchmarks/gnss_smartphone_wls.py": {
                    "sha256": WLS._sha256(Path(WLS.__file__)),
                },
            },
        }
        with tempfile.TemporaryDirectory(prefix="smartphone_holdout_v3_fixture_") as name:
            path = Path(name) / "freeze.json"
            path.write_text(json.dumps(freeze), encoding="utf-8")
            ADAPTER._verify_sealed_holdout_eval_freeze(path)
            WLS._verify_sealed_holdout_eval_freeze(path)

            # Use a tiny materialized input rather than touching the archive;
            # this exercises the same authorization point immediately before
            # the truth-free adapter smoke path used by the holdout harness.
            import csv

            materialized = Path(name) / "materialized"
            materialized.mkdir()
            device = materialized / "device_gnss.csv"
            row = {
                "MessageType": "Raw",
                "utcTimeMillis": "1694113198000",
                "TimeNanos": "1000000000",
                "FullBiasNanos": "-1300000000000000000",
                "HardwareClockDiscontinuityCount": "10",
                "Svid": "3",
                "State": "9",
                "ReceivedSvTimeNanos": "123456789",
                "ReceivedSvTimeUncertaintyNanos": "10",
                "Cn0DbHz": "35.5",
                "PseudorangeRateMetersPerSecond": "-12.25",
                "AccumulatedDeltaRangeState": "1",
                "AccumulatedDeltaRangeMeters": "123.5",
                "AccumulatedDeltaRangeUncertaintyMeters": "0.02",
                "CarrierFrequencyHz": "1575420000",
                "ConstellationType": "1",
                "CodeType": "",
                "SignalType": "GPS_L1_CA",
                "ArrivalTimeNanosSinceGpsEpoch": "1378148416000000000",
                "RawPseudorangeMeters": "22000000.25",
                "RawPseudorangeUncertaintyMeters": "4.5",
                "WlsPositionXEcefMeters": "-2700000.0",
                "WlsPositionYEcefMeters": "-4300000.0",
                "WlsPositionZEcefMeters": "3850000.0",
            }
            temporary_device = materialized / ".device_gnss.csv.tmp"
            with temporary_device.open("w", encoding="utf-8", newline="") as handle:
                writer = csv.DictWriter(handle, fieldnames=ADAPTER.DEVICE_REQUIRED)
                writer.writeheader()
                writer.writerow(row)
                handle.flush()
            temporary_device.replace(device)
            output = Path(name) / "adapter-smoke"
            result = subprocess.run(
                [
                    sys.executable,
                    str(ROOT / "apps" / "gnss.py"),
                    "smartphone-gnss-adapter",
                    "--device-gnss",
                    str(device),
                    "--truth-free",
                    "--output-dir",
                    str(output),
                    "--dataset-id",
                    "fixture/phone",
                    "--device-model",
                    "test-phone",
                    "--source-url",
                    "https://example.test/dataset",
                    "--source-terms",
                    "test fixture only",
                    "--role",
                    "holdout",
                    "--skip-epochs",
                    "0",
                    "--sealed-holdout-eval-freeze",
                    str(path),
                ],
                cwd=ROOT,
                text=True,
                capture_output=True,
                check=False,
            )
            self.assertEqual(result.returncode, 0, result.stderr)
            summary = json.loads((output / "summary.json").read_text(encoding="utf-8"))
            self.assertTrue(summary["truth_free"])
            self.assertEqual(summary["decision"], "truth-free-pipeline")

            basename_only = dict(freeze)
            basename_only["source_files"] = {
                "gnss_smartphone_gnss_adapter.py": freeze["source_files"][
                    "apps/commands/benchmarks/gnss_smartphone_gnss_adapter.py"
                ],
                "gnss_smartphone_wls.py": freeze["source_files"][
                    "apps/commands/benchmarks/gnss_smartphone_wls.py"
                ],
            }
            path.write_text(json.dumps(basename_only), encoding="utf-8")
            with self.assertRaises(SystemExit):
                ADAPTER._verify_sealed_holdout_eval_freeze(path)
            with self.assertRaises(WLS.WlsPositionError):
                WLS._verify_sealed_holdout_eval_freeze(path)


if __name__ == "__main__":
    unittest.main()
