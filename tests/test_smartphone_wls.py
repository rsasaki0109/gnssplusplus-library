from __future__ import annotations

import csv
import hashlib
import json
from pathlib import Path
import sys
import tempfile
import unittest


ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "apps" / "commands"))
sys.path.insert(0, str(ROOT / "apps" / "commands" / "benchmarks"))
import gnss_smartphone_wls as WLS  # noqa: E402


FIELDS = (
    "MessageType",
    "utcTimeMillis",
    "HardwareClockDiscontinuityCount",
    "Svid",
    "WlsPositionXEcefMeters",
    "WlsPositionYEcefMeters",
    "WlsPositionZEcefMeters",
)


class SmartphoneWlsTests(unittest.TestCase):
    def _write_csv(self, path: Path, rows: list[dict[str, str]]) -> None:
        with path.open("w", newline="", encoding="utf-8") as handle:
            writer = csv.DictWriter(handle, fieldnames=FIELDS, lineterminator="\n")
            writer.writeheader()
            writer.writerows(rows)

    def _row(
        self,
        timestamp: int,
        svid: int,
        x: str = "-2704229.0",
        y: str = "-4288800.0",
        z: str = "3856689.0",
        clock: str = "0",
    ) -> dict[str, str]:
        return {
            "MessageType": "Raw",
            "utcTimeMillis": str(timestamp),
            "HardwareClockDiscontinuityCount": clock,
            "Svid": str(svid),
            "WlsPositionXEcefMeters": x,
            "WlsPositionYEcefMeters": y,
            "WlsPositionZEcefMeters": z,
        }

    def _write_multi_phone_holdout_authorization(
        self,
        root: Path,
        *,
        route: str = WLS.MULTI_PHONE_HOLDOUT_ROUTE,
        algorithm_hash: str = WLS.MULTI_PHONE_HOLDOUT_ALGORITHM_PARAMETER_HASH,
        sparse: bool = False,
        gap_diagnostic: bool = False,
        out_of_range_omission: bool = False,
    ) -> tuple[Path, Path]:
        """Build the smallest multi-phone freeze accepted by the guard."""

        root.mkdir(parents=True, exist_ok=True)
        record_path = root / "freeze.json"
        manifest_path = root / "freeze-manifest.json"
        record = {
            "schema_version": (
                WLS.MULTI_PHONE_HOLDOUT_FREEZE_SCHEMA_V1_3
                if gap_diagnostic
                else WLS.MULTI_PHONE_HOLDOUT_FREEZE_SCHEMA_V1_2
                if sparse
                else WLS.MULTI_PHONE_HOLDOUT_FREEZE_SCHEMA
            ),
            "status": "frozen-before-holdout-payload-access",
            "holdout_route": route,
            "algorithm_parameter_hash": algorithm_hash,
            "holdout_execution_contract": {
                "authorized": True,
                "truth_free_phase": True,
                "no_post_holdout_tuning": True,
                "route": route,
                "phone_allowlist": list(WLS.MULTI_PHONE_HOLDOUT_PHONE_ALLOWLIST),
                "algorithm_parameter_hash": algorithm_hash,
            },
            "source_hashes": {
                "apps/commands/benchmarks/gnss_smartphone_wls.py": WLS._sha256(
                    Path(WLS.__file__)
                )
            },
        }
        if sparse:
            record["algorithm_core_hash"] = WLS.MULTI_PHONE_HOLDOUT_ALGORITHM_CORE_HASH
            record["holdout_execution_contract"]["algorithm_core_hash"] = (
                WLS.MULTI_PHONE_HOLDOUT_ALGORITHM_CORE_HASH
            )
            record["holdout_execution_contract"]["allow_missing_wls_epochs"] = True
            record["sparse_wls_contract"] = {
                "allow_missing_wls_epochs": True,
                "single_phone_default_fail_closed": True,
                "partial_coordinate_triplet_policy": "fail-closed",
                "nonfinite_coordinate_policy": "fail-closed",
                "inconsistent_coordinate_policy": "fail-closed",
            }
            if out_of_range_omission:
                record["sparse_wls_contract"][
                    "allow_out_of_range_wls_epochs_as_omission"
                ] = True
            if gap_diagnostic:
                record["holdout_execution_contract"]["allow_timestamp_gaps_as_diagnostic"] = True
                record["holdout_execution_contract"]["extrapolation_policy"] = "forbidden"
                record["sparse_wls_contract"]["allow_timestamp_gaps_as_diagnostic"] = True
                record["sparse_wls_contract"]["extrapolation_policy"] = "forbidden"
        record_path.write_text(json.dumps(record), encoding="utf-8")
        manifest = {
            "schema_version": (
                WLS.MULTI_PHONE_HOLDOUT_FREEZE_MANIFEST_SCHEMA_V1_3
                if gap_diagnostic
                else WLS.MULTI_PHONE_HOLDOUT_FREEZE_MANIFEST_SCHEMA_V1_2
                if sparse
                else WLS.MULTI_PHONE_HOLDOUT_FREEZE_MANIFEST_SCHEMA
            ),
            "freeze_record": {
                "path": str(record_path),
                "sha256": WLS._sha256(record_path),
            },
            "algorithm_parameter_hash": algorithm_hash,
        }
        if sparse:
            manifest["algorithm_core_hash"] = WLS.MULTI_PHONE_HOLDOUT_ALGORITHM_CORE_HASH
            manifest["sparse_wls_contract"] = {"allow_missing_wls_epochs": True}
            if gap_diagnostic:
                manifest["sparse_wls_contract"]["allow_timestamp_gaps_as_diagnostic"] = True
                manifest["sparse_wls_contract"]["extrapolation_policy"] = "forbidden"
        manifest_path.write_text(json.dumps(manifest), encoding="utf-8")
        return record_path, manifest_path

    def test_extracts_consistent_epochs_and_publishes_final_paths(self) -> None:
        with tempfile.TemporaryDirectory() as name:
            root = Path(name)
            device = root / "device_gnss.csv"
            self._write_csv(
                device,
                [
                    self._row(1_609_797_015_000, 1),
                    self._row(1_609_797_015_000, 2),
                    self._row(1_609_797_016_000, 1, x="-2704228.0"),
                ],
            )
            extraction = WLS.extract_epochs(device)
            self.assertEqual(extraction.input_epochs, 2)
            self.assertEqual(extraction.epochs[0].svid_count, 2)
            self.assertEqual(extraction.epochs[1].ecef[0], -2704228.0)
            output = root / "output"
            submission = root / "submission.csv"
            manifest = WLS.extract_to_directory(
                device,
                output,
                skip_epochs=0,
                submission_output=submission,
                phone="fixture-phone",
                dataset_id="fixture/route",
            )
            self.assertEqual(manifest["artifacts"]["position"]["path"], str(output / "wls.pos"))
            self.assertTrue((output / "wls.pos").is_file())
            self.assertTrue((output / "receiver_wls.csv").is_file())
            self.assertTrue((output / "wls_manifest.json").is_file())
            self.assertTrue(submission.is_file())
            self.assertTrue(submission.with_name("submission.csv.manifest.json").is_file())
            self.assertIn(
                "phone,UnixTimeMillis,LatitudeDegrees,LongitudeDegrees",
                submission.read_text(encoding="utf-8"),
            )
            self.assertFalse("ground_truth" in manifest["inputs"] and manifest["inputs"]["ground_truth"])
            self.assertEqual(
                manifest["artifacts"]["position"]["sha256"],
                hashlib.sha256((output / "wls.pos").read_bytes()).hexdigest(),
            )

    def test_missing_or_partial_wls_fails_closed_with_classification(self) -> None:
        cases = {
            "missing": ("", "", ""),
            "partial": ("", "-4288800.0", "3856689.0"),
        }
        for label, values in cases.items():
            with self.subTest(label=label), tempfile.TemporaryDirectory() as name:
                device = Path(name) / "device_gnss.csv"
                row = self._row(1_609_797_015_000, 1)
                row["WlsPositionXEcefMeters"] = values[0]
                row["WlsPositionYEcefMeters"] = values[1]
                row["WlsPositionZEcefMeters"] = values[2]
                self._write_csv(device, [row])
                with self.assertRaises(WLS.WlsPositionError) as context:
                    WLS.extract_epochs(device)
                expected = "missing_wls_epoch" if label == "missing" else "partial_wls_epoch"
                self.assertEqual(context.exception.classification, expected)

    def test_out_of_range_epoch_is_omitted_only_in_explicit_sparse_mode(self) -> None:
        base = 1_609_797_015_000
        bad = self._row(
            base + 1_000,
            1,
            x="-2326881.9506835593",
            y="-4188329.210636505",
            z="3489470.616096731",
        )
        with tempfile.TemporaryDirectory() as name:
            root = Path(name)
            device = root / "out-of-range.csv"
            self._write_csv(device, [self._row(base, 1), bad, self._row(base + 2_000, 1)])
            with self.assertRaises(WLS.WlsPositionError) as context:
                WLS.extract_epochs(device, allow_missing_wls_epochs=True)
            self.assertEqual(context.exception.classification, "ecef_out_of_range")
            extraction = WLS.extract_epochs(
                device,
                allow_missing_wls_epochs=True,
                allow_invalid_wls_epochs=True,
            )
            self.assertEqual([epoch.timestamp_ms for epoch in extraction.epochs], [base, base + 2_000])
            self.assertEqual(extraction.classification_counts["sparse_invalid_wls_epoch"], 1)
            self.assertEqual(extraction.sparse_omission_records[0]["timestamp_ms"], base + 1_000)
            self.assertEqual(extraction.sparse_omission_records[0]["invalid_row_count"], 1)

    def test_sparse_out_of_range_does_not_weaken_partial_or_nonfinite_contract(self) -> None:
        base = 1_609_797_015_000
        with tempfile.TemporaryDirectory() as name:
            root = Path(name)
            partial = self._row(base, 1, x="")
            partial["WlsPositionYEcefMeters"] = "-4288800.0"
            partial["WlsPositionZEcefMeters"] = "3856689.0"
            partial_path = root / "partial.csv"
            self._write_csv(partial_path, [partial, self._row(base + 1_000, 1)])
            with self.assertRaises(WLS.WlsPositionError) as context:
                WLS.extract_epochs(
                    partial_path,
                    allow_missing_wls_epochs=True,
                    allow_invalid_wls_epochs=True,
                )
            self.assertEqual(context.exception.classification, "partial_wls_epoch")

            nonfinite = self._row(base, 1, x="nan")
            nonfinite_path = root / "nonfinite.csv"
            self._write_csv(nonfinite_path, [nonfinite, self._row(base + 1_000, 1)])
            with self.assertRaises(WLS.WlsPositionError) as context:
                WLS.extract_epochs(
                    nonfinite_path,
                    allow_missing_wls_epochs=True,
                    allow_invalid_wls_epochs=True,
                )
            self.assertEqual(context.exception.classification, "invalid_WlsPositionXEcefMeters")

    def test_out_of_range_omission_requires_explicit_authorized_sparse_contract(self) -> None:
        base = 1_609_797_015_000
        bad = self._row(
            base,
            1,
            x="-2326881.9506835593",
            y="-4188329.210636505",
            z="3489470.616096731",
        )
        with tempfile.TemporaryDirectory() as name:
            root = Path(name)
            device = root / "device.csv"
            self._write_csv(device, [bad, self._row(base + 1_000, 1)])
            freeze, manifest = self._write_multi_phone_holdout_authorization(
                root, sparse=True, gap_diagnostic=True
            )
            with self.assertRaises(WLS.WlsPositionError):
                WLS.extract_to_directory(
                    device,
                    root / "unauthorized-range",
                    skip_epochs=0,
                    role="holdout",
                    dataset_id=f"{WLS.MULTI_PHONE_HOLDOUT_ROUTE}/sm-a205u",
                    sealed_holdout_eval_freeze=freeze,
                    sealed_holdout_eval_freeze_manifest=manifest,
                    truth_free=True,
                    allow_missing_wls_epochs=True,
                    allow_invalid_wls_epochs=True,
                )
            freeze, manifest = self._write_multi_phone_holdout_authorization(
                root / "authorized", sparse=True, gap_diagnostic=True,
                out_of_range_omission=True
            )
            with self.assertRaises(WLS.WlsPositionError):
                # Holdout freezes intentionally cannot authorize this test-only
                # extension, even when their sparse contract is valid.
                WLS.extract_to_directory(
                    device,
                    root / "holdout-range",
                    skip_epochs=0,
                    role="holdout",
                    dataset_id=f"{WLS.MULTI_PHONE_HOLDOUT_ROUTE}/sm-a205u",
                    sealed_holdout_eval_freeze=freeze,
                    sealed_holdout_eval_freeze_manifest=manifest,
                    truth_free=True,
                    allow_missing_wls_epochs=True,
                    allow_invalid_wls_epochs=True,
                )

    def test_inconsistent_ecef_timestamp_and_clock_fail_closed(self) -> None:
        with tempfile.TemporaryDirectory() as name:
            root = Path(name)
            inconsistent = root / "inconsistent.csv"
            self._write_csv(
                inconsistent,
                [
                    self._row(1_609_797_015_000, 1),
                    self._row(1_609_797_015_000, 2, x="-2704228.0"),
                ],
            )
            with self.assertRaises(WLS.WlsPositionError) as context:
                WLS.extract_epochs(inconsistent)
            self.assertEqual(context.exception.classification, "inconsistent_epoch_wls")

            clock = root / "clock.csv"
            self._write_csv(
                clock,
                [
                    self._row(1_609_797_015_000, 1, clock="0"),
                    self._row(1_609_797_016_000, 1, clock="1"),
                    self._row(1_609_797_017_000, 1, clock="0"),
                ],
            )
            with self.assertRaises(WLS.WlsPositionError) as context:
                WLS.extract_epochs(clock)
            self.assertEqual(context.exception.classification, "clock_discontinuity_backwards")

    def test_timestamp_gap_boundary_matches_hatch_contract(self) -> None:
        """Only gaps over 1500 ms become fail-closed reset boundaries."""

        base_timestamp = 1_609_797_015_000
        for delta_ms, expected_gap_count in ((1_001, 0), (1_500, 0), (1_501, 1)):
            with self.subTest(delta_ms=delta_ms), tempfile.TemporaryDirectory() as name:
                device = Path(name) / "timestamp-boundary.csv"
                self._write_csv(
                    device,
                    [
                        self._row(base_timestamp, 1),
                        self._row(base_timestamp + delta_ms, 1),
                    ],
                )
                extraction = WLS.extract_epochs(device)
                self.assertEqual(
                    extraction.timestamp_gap_count,
                    expected_gap_count,
                )
                self.assertEqual(
                    extraction.classification_counts["timestamp_gap"],
                    expected_gap_count,
                )
                self.assertEqual(
                    extraction.max_timestamp_gap_s,
                    delta_ms / 1000.0,
                )

    def test_timestamp_gap_is_fail_closed_for_single_phone_publish(self) -> None:
        base_timestamp = 1_609_797_015_000
        with tempfile.TemporaryDirectory() as name:
            root = Path(name)
            device = root / "gap.csv"
            self._write_csv(
                device,
                [
                    self._row(base_timestamp, 1),
                    self._row(base_timestamp + 2_001, 1),
                ],
            )
            with self.assertRaises(WLS.WlsPositionError) as context:
                WLS.extract_to_directory(device, root / "default-output", skip_epochs=0)
            self.assertEqual(context.exception.classification, "timestamp_gap")
            self.assertFalse((root / "default-output" / "wls.pos").exists())

    def test_v1_3_sparse_gap_is_diagnostic_only_under_sealed_authorization(self) -> None:
        base_timestamp = 1_609_797_015_000
        with tempfile.TemporaryDirectory() as name:
            root = Path(name)
            freeze, manifest = self._write_multi_phone_holdout_authorization(
                root, sparse=True, gap_diagnostic=True
            )
            device = root / "gap.csv"
            self._write_csv(
                device,
                [
                    self._row(base_timestamp, 1),
                    self._row(base_timestamp + 2_001, 1),
                ],
            )
            output = root / "v1-3-output"
            result = WLS.extract_to_directory(
                device,
                output,
                skip_epochs=0,
                role="holdout",
                dataset_id=f"{WLS.MULTI_PHONE_HOLDOUT_ROUTE}/sm-a205u",
                sealed_holdout_eval_freeze=freeze,
                sealed_holdout_eval_freeze_manifest=manifest,
                truth_free=True,
                allow_missing_wls_epochs=True,
            )
            self.assertTrue(result["truth_free"])
            summary = json.loads((output / "wls_summary.json").read_text(encoding="utf-8"))
            manifest_payload = json.loads(
                (output / "wls_manifest.json").read_text(encoding="utf-8")
            )
            self.assertTrue(summary["validation"]["timestamp_gap_allowed_as_diagnostic"])
            self.assertEqual(summary["populations"]["timestamp_gap_count_gt_1500ms"], 1)
            self.assertTrue(manifest_payload["contract"]["timestamp_gap_allowed_as_diagnostic"])

    def test_multi_phone_holdout_authorization_is_fail_closed_at_all_boundaries(self) -> None:
        with tempfile.TemporaryDirectory() as name:
            root = Path(name)
            freeze, manifest = self._write_multi_phone_holdout_authorization(root)
            authorized = f"{WLS.MULTI_PHONE_HOLDOUT_ROUTE}/sm-a205u"
            WLS._verify_sealed_holdout_eval_freeze(
                freeze, manifest, dataset_id=authorized, truth_free=True
            )

            with self.assertRaises(WLS.WlsPositionError):
                WLS._verify_sealed_holdout_eval_freeze(
                    freeze, dataset_id=authorized, truth_free=True
                )

            invalid_manifest = root / "invalid-manifest.json"
            invalid_manifest.write_text(
                json.dumps(
                    {
                        "schema_version": WLS.MULTI_PHONE_HOLDOUT_FREEZE_MANIFEST_SCHEMA,
                        "freeze_record": {"sha256": "wrong"},
                        "algorithm_parameter_hash": WLS.MULTI_PHONE_HOLDOUT_ALGORITHM_PARAMETER_HASH,
                    }
                ),
                encoding="utf-8",
            )
            with self.assertRaises(WLS.WlsPositionError):
                WLS._verify_sealed_holdout_eval_freeze(
                    freeze, invalid_manifest, dataset_id=authorized, truth_free=True
                )

            with self.assertRaises(WLS.WlsPositionError):
                WLS._verify_sealed_holdout_eval_freeze(
                    freeze,
                    manifest,
                    dataset_id="different-route/sm-a205u",
                    truth_free=True,
                )
            with self.assertRaises(WLS.WlsPositionError):
                WLS._verify_sealed_holdout_eval_freeze(
                    freeze,
                    manifest,
                    dataset_id=f"{WLS.MULTI_PHONE_HOLDOUT_ROUTE}/unlisted-phone",
                    truth_free=True,
                )
            wrong_algorithm_freeze, wrong_algorithm_manifest = (
                self._write_multi_phone_holdout_authorization(
                    root / "wrong-algorithm", algorithm_hash="wrong"
                )
            )
            with self.assertRaises(WLS.WlsPositionError):
                WLS._verify_sealed_holdout_eval_freeze(
                    wrong_algorithm_freeze,
                    wrong_algorithm_manifest,
                    dataset_id=authorized,
                    truth_free=True,
                )
            with self.assertRaises(WLS.WlsPositionError):
                WLS._verify_sealed_holdout_eval_freeze(
                    freeze, manifest, dataset_id=authorized, truth_free=False
                )

    def test_multi_phone_holdout_authorized_truth_free_wls_smoke_is_archive_free(self) -> None:
        with tempfile.TemporaryDirectory() as name:
            root = Path(name)
            freeze, manifest = self._write_multi_phone_holdout_authorization(root)
            device = root / "device_gnss.csv"
            self._write_csv(
                device,
                [
                    self._row(1_609_797_015_000, 1),
                    self._row(1_609_797_016_000, 1),
                ],
            )
            output = root / "holdout-wls"
            result = WLS.extract_to_directory(
                device,
                output,
                skip_epochs=0,
                role="holdout",
                dataset_id=f"{WLS.MULTI_PHONE_HOLDOUT_ROUTE}/sm-a205u",
                sealed_holdout_eval_freeze=freeze,
                sealed_holdout_eval_freeze_manifest=manifest,
                truth_free=True,
            )
            self.assertTrue((output / "wls.pos").is_file())
            self.assertTrue(result["truth_free"])

            with self.assertRaises(WLS.WlsPositionError):
                WLS.extract_to_directory(
                    device,
                    root / "unauthorized-holdout-wls",
                    skip_epochs=0,
                    role="holdout",
                    dataset_id=f"{WLS.MULTI_PHONE_HOLDOUT_ROUTE}/sm-a205u",
                    truth_free=True,
                )

    def test_sparse_wls_leading_and_internal_blank_epochs_are_recorded_only_when_explicit(self) -> None:
        base_timestamp = 1_609_797_015_000
        cases = {
            "leading": [
                self._row(base_timestamp, 1, x="", y="", z=""),
                self._row(base_timestamp + 1_000, 1),
            ],
            "internal": [
                self._row(base_timestamp, 1),
                self._row(base_timestamp + 1_000, 1, x="", y="", z=""),
                self._row(base_timestamp + 2_000, 1),
            ],
        }
        for label, rows in cases.items():
            with self.subTest(label=label), tempfile.TemporaryDirectory() as name:
                device = Path(name) / "sparse.csv"
                self._write_csv(device, rows)
                with self.assertRaises(WLS.WlsPositionError) as default_error:
                    WLS.extract_epochs(device)
                self.assertEqual(default_error.exception.classification, "missing_wls_epoch")

                extraction = WLS.extract_epochs(
                    device, allow_missing_wls_epochs=True
                )
                self.assertEqual(extraction.blank_wls_row_count, 1)
                self.assertEqual(len(extraction.sparse_omission_records), 1)
                self.assertEqual(
                    extraction.sparse_omission_records[0]["reason"],
                    "all WLS coordinate fields blank",
                )
                self.assertEqual(
                    WLS.selected_timestamp_keys(extraction, 0),
                    tuple(int(row["utcTimeMillis"]) for row in rows),
                )
                selected = WLS.select_epochs(extraction, 0)
                self.assertEqual(
                    len(selected),
                    1 if label == "leading" else 2,
                )

    def test_sparse_wls_partial_nonfinite_and_inconsistent_rows_remain_fail_closed(self) -> None:
        base_timestamp = 1_609_797_015_000
        cases = {
            "partial": [
                self._row(base_timestamp, 1, x="", y="-4288800.0", z="3856689.0"),
                self._row(base_timestamp + 1_000, 1),
            ],
            "nonfinite": [
                self._row(base_timestamp, 1, x="nan"),
                self._row(base_timestamp + 1_000, 1),
            ],
            "inconsistent": [
                self._row(base_timestamp, 1),
                self._row(base_timestamp, 2, x="-2704228.0"),
            ],
        }
        expected = {
            "partial": "partial_wls_epoch",
            "nonfinite": "invalid_WlsPositionXEcefMeters",
            "inconsistent": "inconsistent_epoch_wls",
        }
        for label, rows in cases.items():
            with self.subTest(label=label), tempfile.TemporaryDirectory() as name:
                device = Path(name) / f"{label}.csv"
                self._write_csv(device, rows)
                with self.assertRaises(WLS.WlsPositionError) as context:
                    WLS.extract_epochs(device, allow_missing_wls_epochs=True)
                self.assertEqual(context.exception.classification, expected[label])

    def test_sparse_wls_requires_v1_2_authorization_and_publishes_omission_metadata(self) -> None:
        with tempfile.TemporaryDirectory() as name:
            root = Path(name)
            freeze, manifest = self._write_multi_phone_holdout_authorization(
                root, sparse=True
            )
            device = root / "device_gnss.csv"
            self._write_csv(
                device,
                [
                    self._row(1_609_797_015_000, 1, x="", y="", z=""),
                    self._row(1_609_797_016_000, 1),
                ],
            )
            output = root / "sparse-holdout-wls"
            result = WLS.extract_to_directory(
                device,
                output,
                skip_epochs=0,
                role="holdout",
                dataset_id=f"{WLS.MULTI_PHONE_HOLDOUT_ROUTE}/sm-a205u",
                sealed_holdout_eval_freeze=freeze,
                sealed_holdout_eval_freeze_manifest=manifest,
                truth_free=True,
                allow_missing_wls_epochs=True,
            )
            self.assertTrue(result["truth_free"])
            summary = json.loads(
                (output / "wls_summary.json").read_text(encoding="utf-8")
            )
            self.assertTrue(summary["validation"]["allow_missing_wls_epochs"])
            self.assertEqual(summary["populations"]["blank_wls_row_count"], 1)
            self.assertEqual(summary["populations"]["sparse_omitted_epoch_count"], 1)
            self.assertEqual(
                summary["populations"]["sparse_omitted_epoch_records"][0]["reason"],
                "all WLS coordinate fields blank",
            )

            v1_freeze, v1_manifest = self._write_multi_phone_holdout_authorization(
                root / "v1", sparse=False
            )
            with self.assertRaises(WLS.WlsPositionError):
                WLS.extract_to_directory(
                    device,
                    root / "sparse-v1-rejected",
                    skip_epochs=0,
                    role="holdout",
                    dataset_id=f"{WLS.MULTI_PHONE_HOLDOUT_ROUTE}/sm-a205u",
                    sealed_holdout_eval_freeze=v1_freeze,
                    sealed_holdout_eval_freeze_manifest=v1_manifest,
                    truth_free=True,
                    allow_missing_wls_epochs=True,
                )


if __name__ == "__main__":
    unittest.main()
