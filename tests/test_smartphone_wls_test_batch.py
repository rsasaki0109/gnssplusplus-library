from __future__ import annotations

import csv
import io
import json
from pathlib import Path
import sys
import tempfile
import unittest
import zipfile

import numpy as np


ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "apps" / "commands"))
sys.path.insert(0, str(ROOT / "apps" / "commands" / "benchmarks"))
import gnss_smartphone_wls_test_batch as BATCH  # noqa: E402
import gnss_smartphone_trajectory_smoother as SMOOTHER  # noqa: E402
import gnss_smartphone_wls as WLS  # noqa: E402


class SmartphoneWlsTestBatchTests(unittest.TestCase):
    @staticmethod
    def _row(timestamp: int, x: float) -> SMOOTHER.PositionRow:
        return SMOOTHER.PositionRow(
            week=2200,
            tow=float(timestamp) / 1000.0,
            timestamp_ms=timestamp,
            ecef=np.array([x, 0.0, 6370000.0]),
            latitude=0.0,
            longitude=0.0,
            height=0.0,
            status=1,
            satellites=4,
            pdop=0.0,
            ratio=0.0,
            fixed_ambiguities=0,
            iterations=0,
            source_line=1,
        )

    def test_derived_key_order_is_explicit_opt_in(self) -> None:
        self.assertFalse(BATCH.parse_args([]).allow_derived_unverified_key_order)
        self.assertTrue(
            BATCH.parse_args(["--allow-derived-unverified-key-order"])
            .allow_derived_unverified_key_order
        )

    def test_inventory_is_test_only_central_directory_metadata(self) -> None:
        with tempfile.TemporaryDirectory() as name:
            archive_path = Path(name) / "dataset.zip"
            with zipfile.ZipFile(archive_path, "w") as archive:
                archive.writestr("dataset_2023/test/r1/brdc.nav", b"nav")
                archive.writestr(
                    "dataset_2023/test/r1/pixel5/device_gnss.csv", b"device"
                )
                archive.writestr(
                    "dataset_2023/test/r1/pixel5/ground_truth.csv", b"must-not-be-read"
                )
            inventory = BATCH.inventory_archive(archive_path)
            self.assertEqual(inventory["test"]["route_count"], 1)
            record = inventory["test"]["records"][0]
            self.assertTrue(record["required_files_complete"])
            self.assertTrue(record["truth_present"])
            self.assertFalse(inventory["archive"]["member_content_read"])
            self.assertFalse(inventory["truth_policy"]["test_truth_payload_opened"])

    def test_sample_schema_rejects_key_mismatch_and_accepts_blank_coordinates(self) -> None:
        with tempfile.TemporaryDirectory() as name:
            root = Path(name)
            sample = root / "sample_submission.csv"
            sample.write_text(
                "phone,UnixTimeMillis,LatitudeDegrees,LongitudeDegrees\n"
                "r1_pixel5,1000,,\n",
                encoding="utf-8",
            )
            parsed = BATCH._read_sample_submission(sample)
            self.assertEqual(parsed["key_count"], 1)
            bad = root / "bad.csv"
            bad.write_text("phone,UnixTimeMillis,LatitudeDegrees\nfoo,1,0\n", encoding="utf-8")
            with self.assertRaises(BATCH.TestBatchError):
                BATCH._read_sample_submission(bad)

    def test_single_phone_is_raw_and_multi_phone_uses_frozen_median(self) -> None:
        epoch = 1_609_459_200_000
        single_state = {
            "a": {
                "positions": [self._row(epoch, 10.0)],
                "position_timestamps": [epoch],
                "position_by_timestamp": {epoch: self._row(epoch, 10.0)},
                "selected_keys": [epoch],
                "native_fallback": None,
            }
        }
        output, stats = BATCH._test_fuse_positions(single_state)
        self.assertEqual(stats["phone_count_histogram"], {"1": 1})
        self.assertEqual(output["a"][0].ecef[0], 10.0)

        states = {
            "a": {
                "positions": [self._row(epoch, 10.0)],
                "position_timestamps": [epoch],
                "position_by_timestamp": {epoch: self._row(epoch, 10.0)},
                "selected_keys": [epoch],
                "native_fallback": None,
            },
            "b": {
                "positions": [self._row(epoch + 6, 20.0)],
                "position_timestamps": [epoch + 6],
                "position_by_timestamp": {epoch + 6: self._row(epoch + 6, 20.0)},
                "selected_keys": [epoch],
                "native_fallback": None,
            },
        }
        output, stats = BATCH._test_fuse_positions(states)
        self.assertEqual(output["a"][0].ecef[0], 15.0)
        self.assertEqual(output["b"][0].ecef[0], 15.0)
        self.assertEqual(stats["max_abs_alignment_offset_ms"], 6.0)

    def test_missing_epoch_uses_exact_native_fallback_or_fails_closed(self) -> None:
        epoch = 1_609_459_200_000
        fallback = self._row(epoch, 42.0)
        state = {
            "a": {
                "positions": [],
                "position_timestamps": [],
                "position_by_timestamp": {},
                "selected_keys": [epoch],
                "native_fallback": {"position_by_timestamp": {epoch: fallback}},
            }
        }
        output, stats = BATCH._test_fuse_positions(state)
        self.assertEqual(output["a"][0].ecef[0], 42.0)
        self.assertEqual(stats["native_fallback_epoch_count"], 1)
        state["a"]["native_fallback"] = None
        with self.assertRaises(BATCH.TestBatchError):
            BATCH._test_fuse_positions(state)

    def test_completeness_fallback_native_nearest_uses_earlier_tie(self) -> None:
        epoch = 1_609_459_200_000
        earlier = self._row(epoch - 10, 10.0)
        later = self._row(epoch + 10, 30.0)
        state = {
            "a": {
                "positions": [],
                "position_timestamps": [],
                "position_by_timestamp": {},
                "selected_keys": [epoch],
                "native_fallback": {
                    "positions": [earlier, later],
                    "position_by_timestamp": {earlier.timestamp_ms: earlier, later.timestamp_ms: later},
                },
            }
        }
        output, stats = BATCH._test_fuse_positions(
            state, allow_completeness_fallback=True
        )
        self.assertEqual(output["a"][0].timestamp_ms, epoch)
        self.assertEqual(output["a"][0].ecef[0], 10.0)
        self.assertEqual(stats["native_nearest_fallback_epoch_count"], 1)
        self.assertEqual(stats["wls_interpolation_epoch_count"], 0)

    def test_completeness_fallback_interpolates_only_bracketed_short_gap(self) -> None:
        epoch = 1_609_459_200_000
        before = self._row(epoch - 5_000, 10.0)
        after = self._row(epoch + 5_000, 30.0)
        state = {
            "a": {
                "positions": [before, after],
                "position_timestamps": [before.timestamp_ms, after.timestamp_ms],
                "position_by_timestamp": {before.timestamp_ms: before, after.timestamp_ms: after},
                "selected_keys": [epoch],
                "native_fallback": None,
            }
        }
        output, stats = BATCH._test_fuse_positions(
            state, allow_completeness_fallback=True
        )
        self.assertEqual(output["a"][0].timestamp_ms, epoch)
        self.assertAlmostEqual(output["a"][0].ecef[0], 20.0)
        self.assertEqual(stats["wls_interpolation_epoch_count"], 1)
        self.assertEqual(stats["epoch_manifest"][0]["interpolation_source_gap_ms"], 10_000)

    def test_completeness_fallback_rejects_edge_and_long_gap(self) -> None:
        epoch = 1_609_459_200_000
        first = self._row(epoch + 1_000, 10.0)
        edge_state = {
            "a": {
                "positions": [first],
                "position_timestamps": [first.timestamp_ms],
                "position_by_timestamp": {first.timestamp_ms: first},
                "selected_keys": [epoch],
                "native_fallback": None,
            }
        }
        with self.assertRaises(BATCH.TestBatchError):
            BATCH._test_fuse_positions(edge_state, allow_completeness_fallback=True)
        before = self._row(epoch - 6_000, 10.0)
        after = self._row(epoch + 6_000, 30.0)
        long_state = {
            "a": {
                "positions": [before, after],
                "position_timestamps": [before.timestamp_ms, after.timestamp_ms],
                "position_by_timestamp": {before.timestamp_ms: before, after.timestamp_ms: after},
                "selected_keys": [epoch],
                "native_fallback": None,
            }
        }
        with self.assertRaises(BATCH.TestBatchError):
            BATCH._test_fuse_positions(long_state, allow_completeness_fallback=True)

    def test_edge_completeness_fallback_holds_leading_and_trailing_source(self) -> None:
        epoch = 1_609_459_200_000
        leading_source = self._row(epoch + 2_000, 42.0)
        leading = {
            "a": {
                "positions": [leading_source],
                "position_timestamps": [leading_source.timestamp_ms],
                "position_by_timestamp": {leading_source.timestamp_ms: leading_source},
                "selected_keys": [epoch, epoch + 1_000, epoch + 2_000],
                "epoch_clock_segments": {
                    "clock_by_timestamp": {
                        epoch: 0,
                        epoch + 1_000: 0,
                        epoch + 2_000: 0,
                    },
                    "segment_by_timestamp": {
                        epoch: 0,
                        epoch + 1_000: 0,
                        epoch + 2_000: 0,
                    },
                },
                "native_fallback": None,
            }
        }
        output, stats = BATCH._test_fuse_positions(
            leading,
            allow_completeness_fallback=True,
            allow_edge_completeness_fallback=True,
        )
        self.assertEqual([row.timestamp_ms for row in output["a"]], leading["a"]["selected_keys"])
        self.assertEqual([row.ecef[0] for row in output["a"]], [42.0, 42.0, 42.0])
        self.assertEqual(stats["edge_constant_hold_epoch_count"], 2)
        self.assertEqual(stats["edge_constant_hold_direction_counts"]["leading"], 2)
        self.assertEqual(stats["edge_constant_hold_gap_ms"]["max_ms"], 2_000)

        trailing_source = self._row(epoch, -7.0)
        trailing = {
            "a": {
                "positions": [trailing_source],
                "position_timestamps": [trailing_source.timestamp_ms],
                "position_by_timestamp": {trailing_source.timestamp_ms: trailing_source},
                "selected_keys": [epoch, epoch + 1_000, epoch + 2_000],
                "epoch_clock_segments": {
                    "clock_by_timestamp": {
                        epoch: 0,
                        epoch + 1_000: 0,
                        epoch + 2_000: 0,
                    },
                    "segment_by_timestamp": {
                        epoch: 0,
                        epoch + 1_000: 0,
                        epoch + 2_000: 0,
                    },
                },
                "native_fallback": None,
            }
        }
        output, stats = BATCH._test_fuse_positions(
            trailing,
            allow_completeness_fallback=True,
            allow_edge_completeness_fallback=True,
        )
        self.assertEqual([row.ecef[0] for row in output["a"]], [-7.0, -7.0, -7.0])
        self.assertEqual(stats["edge_constant_hold_direction_counts"]["trailing"], 2)

    def test_edge_completeness_fallback_is_fail_closed_for_long_gap_or_clock_break(self) -> None:
        epoch = 1_609_459_200_000
        long_source = self._row(epoch + 10_001, 1.0)
        long_state = {
            "a": {
                "positions": [long_source],
                "position_timestamps": [long_source.timestamp_ms],
                "position_by_timestamp": {long_source.timestamp_ms: long_source},
                "selected_keys": [epoch],
                "epoch_clock_segments": {
                    "clock_by_timestamp": {epoch: 0, epoch + 10_001: 0},
                    "segment_by_timestamp": {epoch: 0, epoch + 10_001: 0},
                },
                "native_fallback": None,
            }
        }
        with self.assertRaisesRegex(BATCH.TestBatchError, "edge hold exceeds"):
            BATCH._test_fuse_positions(
                long_state,
                allow_completeness_fallback=True,
                allow_edge_completeness_fallback=True,
            )
        source = self._row(epoch + 1_000, 1.0)
        broken_state = {
            "a": {
                "positions": [source],
                "position_timestamps": [source.timestamp_ms],
                "position_by_timestamp": {source.timestamp_ms: source},
                "selected_keys": [epoch, epoch + 1_000],
                "epoch_clock_segments": {
                    "clock_by_timestamp": {epoch: 1, epoch + 1_000: 0},
                    "segment_by_timestamp": {epoch: 1, epoch + 1_000: 0},
                },
                "native_fallback": None,
            }
        }
        with self.assertRaisesRegex(BATCH.TestBatchError, "clock/segment boundary"):
            BATCH._test_fuse_positions(
                broken_state,
                allow_completeness_fallback=True,
                allow_edge_completeness_fallback=True,
            )

    def test_edge_completeness_v2_1_1000ms_passes_and_1001ms_fails(self) -> None:
        epoch = 1_609_459_200_000

        def state(source_timestamp: int) -> dict[str, dict[str, object]]:
            source = self._row(source_timestamp, 9.0)
            return {
                "a": {
                    "positions": [source],
                    "position_timestamps": [source_timestamp],
                    "position_by_timestamp": {source_timestamp: source},
                    "selected_keys": [epoch, source_timestamp],
                    "epoch_clock_segments": {
                        "clock_by_timestamp": {epoch: 0, source_timestamp: 0},
                        "segment_by_timestamp": {epoch: 0, source_timestamp: 0},
                    },
                    "native_fallback": None,
                }
            }

        output, stats = BATCH._test_fuse_positions(
            state(epoch + 1_000),
            allow_completeness_fallback=True,
            allow_edge_completeness_fallback=True,
            edge_hold_max_gap_ms=BATCH.EDGE_COMPLETENESS_FALLBACK_V2_1_MAX_GAP_MS,
        )
        self.assertEqual([row.ecef[0] for row in output["a"]], [9.0, 9.0])
        self.assertEqual(stats["edge_constant_hold_max_gap_ms"], 1_000)
        with self.assertRaisesRegex(BATCH.TestBatchError, "edge hold exceeds"):
            BATCH._test_fuse_positions(
                state(epoch + 1_001),
                allow_completeness_fallback=True,
                allow_edge_completeness_fallback=True,
                edge_hold_max_gap_ms=BATCH.EDGE_COMPLETENESS_FALLBACK_V2_1_MAX_GAP_MS,
            )

    def test_sample_order_is_authoritative_and_output_key_set_is_exact(self) -> None:
        records = [
            {
                "dataset_id": "r1/pixel5",
                "sample_phone_key": "r1_pixel5",
                "phone": "pixel5",
            }
        ]
        sample = {
            "rows": [
                {"phone": "r1_pixel5", "timestamp": 2},
                {"phone": "r1_pixel5", "timestamp": 1},
            ],
            "key_count": 2,
        }
        output, contract = BATCH._ordered_submission_bytes(
            sample,
            records,
            {("r1_pixel5", 1): (1.0, 2.0), ("r1_pixel5", 2): (3.0, 4.0)},
        )
        self.assertTrue(contract["exact_sample_order"])
        rows = list(csv.DictReader(io.StringIO(output.decode("utf-8"))))
        self.assertEqual([row["UnixTimeMillis"] for row in rows], ["2", "1"])
        with self.assertRaises(BATCH.TestBatchError):
            BATCH._ordered_submission_bytes(
                sample,
                records,
                {("r1_pixel5", 1): (1.0, 2.0)},
            )

    def test_derived_key_manifest_is_canonical_and_unverified(self) -> None:
        epoch = 1_609_459_200_000
        with tempfile.TemporaryDirectory() as name:
            root = Path(name)
            device = root / "device_gnss.csv"
            device.write_text(
                "MessageType,utcTimeMillis\n"
                f"Raw,{epoch}\n"
                f"Raw,{epoch}\n"
                f"Raw,{epoch + 1000}\n"
                f"Raw,{epoch + 2000}\n",
                encoding="utf-8",
            )
            positions = [self._row(epoch + 1000, 10.0), self._row(epoch + 2000, 20.0)]
            state = {
                "dataset_id": "r1/pixel5",
                "route": "r1",
                "phone": "pixel5",
                "device_path": device,
                "selected_keys": [epoch + 1000, epoch + 2000],
            }
            key_path, row_map, manifest = BATCH._derived_key_manifest(
                [
                    {
                        "route": "r1",
                        "states": {"pixel5": state},
                        "selected_positions": {"pixel5": positions},
                    }
                ],
                [{"dataset_id": "r1/pixel5"}],
                root,
            )
            self.assertTrue(key_path.is_file())
            self.assertFalse(manifest["official_sample_verified"])
            self.assertTrue(manifest["cannot_submit_to_kaggle"])
            self.assertEqual(
                [(entry["phone"], entry["UnixTimeMillis"]) for entry in manifest["keys"]],
                [("r1/pixel5", epoch + 1000), ("r1/pixel5", epoch + 2000)],
            )
            output, contract = BATCH._derived_submission_bytes(manifest, row_map)
            self.assertTrue(contract["derived_key_order"])
            rows = list(csv.DictReader(io.StringIO(output.decode("utf-8"))))
            self.assertEqual(rows[0]["phone"], "r1/pixel5")
            self.assertEqual(rows[1]["UnixTimeMillis"], str(epoch + 2000))

    def test_derived_key_source_rejects_nonmonotonic_or_nonfinite_timestamp(self) -> None:
        with tempfile.TemporaryDirectory() as name:
            root = Path(name)
            device = root / "device_gnss.csv"
            device.write_text(
                "MessageType,utcTimeMillis\nRaw,2000\nRaw,1000\n", encoding="utf-8"
            )
            with self.assertRaises(BATCH.TestBatchError):
                BATCH._read_device_epoch_keys(device, skip_epochs=0)
            device.write_text(
                "MessageType,utcTimeMillis\nRaw," + "9" * 400 + "\n",
                encoding="utf-8",
            )
            with self.assertRaises(BATCH.TestBatchError):
                BATCH._read_device_epoch_keys(device, skip_epochs=0)

    def test_promote_derived_with_sample_reorders_only_and_never_reruns(self) -> None:
        epoch = 1_609_459_200_000
        with tempfile.TemporaryDirectory() as name:
            root = Path(name)
            output_dir = root / "out"
            output_dir.mkdir()
            archive = root / "dataset.zip"
            archive.write_bytes(b"archive")
            inventory_path = root / "inventory.json"
            inventory = {
                "test": {
                    "records": [
                        {
                            "dataset_id": "r1/pixel5",
                            "sample_phone_key": "r1_pixel5",
                            "route": "r1",
                            "phone": "pixel5",
                        }
                    ]
                }
            }
            inventory_path.write_text(json.dumps(inventory), encoding="utf-8")
            key_manifest_path = output_dir / "provisional_key_manifest.json"
            key_manifest_path.write_text(
                json.dumps(
                    {
                        "schema_version": BATCH.DERIVED_KEY_MANIFEST_SCHEMA,
                        "official_sample_verified": False,
                        "cannot_submit_to_kaggle": True,
                        "keys": [
                            {"phone": "r1/pixel5", "UnixTimeMillis": epoch},
                            {"phone": "r1/pixel5", "UnixTimeMillis": epoch + 1000},
                        ],
                    }
                ),
                encoding="utf-8",
            )
            provisional_path = output_dir / "submission_derived_unverified.csv"
            provisional_path.write_text(
                "phone,UnixTimeMillis,LatitudeDegrees,LongitudeDegrees\n"
                f"r1/pixel5,{epoch},1.0,2.0\n"
                f"r1/pixel5,{epoch + 1000},3.0,4.0\n",
                encoding="utf-8",
            )
            run_manifest_path = output_dir / "test_batch_derived_run_manifest.json"
            run_manifest_path.write_text(
                json.dumps(
                    {
                        "schema_version": BATCH.DERIVED_RUN_MANIFEST_SCHEMA,
                        "status": "completed-truth-free-derived-submission",
                        "official_sample_verified": False,
                        "archive": {"sha256": BATCH._sha256(archive)},
                        "inventory": {"sha256": BATCH._sha256(inventory_path)},
                        "truth_policy": {"truth_open_count": 0},
                        "provisional_key_manifest": BATCH._artifact(key_manifest_path),
                        "provisional_submission": BATCH._artifact(provisional_path),
                    }
                ),
                encoding="utf-8",
            )
            sample = root / "sample_submission.csv"
            sample.write_text(
                "phone,UnixTimeMillis,LatitudeDegrees,LongitudeDegrees\n"
                f"r1_pixel5,{epoch + 1000},,\n"
                f"r1_pixel5,{epoch},,\n",
                encoding="utf-8",
            )
            result = BATCH._promote_derived_with_sample(
                output_dir,
                sample,
                inventory_path,
                archive,
                inventory,
            )
            self.assertEqual(result, 0)
            with (output_dir / "submission.csv").open(encoding="utf-8") as handle:
                rows = list(csv.DictReader(handle))
            self.assertEqual([row["UnixTimeMillis"] for row in rows], [str(epoch + 1000), str(epoch)])
            promotion = json.loads(
                (output_dir / "test_batch_derived_promotion_manifest.json").read_text(encoding="utf-8")
            )
            self.assertTrue(promotion["official_sample"]["sha256"])
            self.assertFalse(promotion["algorithm_recomputed"])

    def test_official_sample_reconciliation_v2_renames_header_and_uses_exact_or_sample_fallback(self) -> None:
        self.assertTrue(
            BATCH.parse_args(["--reconcile-derived-with-sample-v2"])
            .reconcile_derived_with_sample_v2
        )
        sample_rows = [
            {
                "trip_id": "r1/pixel5",
                "timestamp": 2,
                "timestamp_text": "2",
                "latitude": 11.0,
                "longitude": 21.0,
            },
            {
                "trip_id": "r1/pixel5",
                "timestamp": 3,
                "timestamp_text": "3",
                "latitude": 12.0,
                "longitude": 22.0,
            },
        ]
        output, reconciliation = BATCH._reconcile_official_sample_rows(
            sample_rows,
            {
                ("r1/pixel5", 2): (10.0, 20.0),
                ("r1/pixel5", 4): (13.0, 23.0),
            },
        )
        rows = list(csv.DictReader(io.StringIO(output.decode("utf-8"))))
        self.assertEqual(
            rows[0].keys(),
            {"tripId", "UnixTimeMillis", "LatitudeDegrees", "LongitudeDegrees"},
        )
        self.assertEqual([row["tripId"] for row in rows], ["r1/pixel5", "r1/pixel5"])
        self.assertEqual([row["UnixTimeMillis"] for row in rows], ["2", "3"])
        self.assertEqual(rows[0]["LatitudeDegrees"], "10.000000000000")
        self.assertEqual(rows[1]["LatitudeDegrees"], "12.000000000000")
        self.assertEqual(
            reconciliation["source_counts"],
            {"exact": 1, "sample_fallback": 1, "drop_extra": 1},
        )
        self.assertTrue(reconciliation["contract"]["sample_order_preserved"])
        self.assertFalse(reconciliation["contract"]["nearest_or_remap"])

    def test_official_sample_reconciliation_v2_rejects_duplicate_and_nonfinite_fixture_rows(self) -> None:
        with tempfile.TemporaryDirectory() as name:
            root = Path(name)
            duplicate = root / "duplicate.csv"
            duplicate.write_text(
                "tripId,UnixTimeMillis,LatitudeDegrees,LongitudeDegrees\n"
                "r1/pixel5,1,1,2\n"
                "r1/pixel5,1,1,2\n",
                encoding="utf-8",
            )
            with self.assertRaisesRegex(BATCH.TestBatchError, "duplicates key"):
                BATCH._read_official_sample_submission(duplicate)
            nonfinite = root / "nonfinite.csv"
            nonfinite.write_text(
                "tripId,UnixTimeMillis,LatitudeDegrees,LongitudeDegrees\n"
                "r1/pixel5,1,nan,2\n",
                encoding="utf-8",
            )
            with self.assertRaisesRegex(BATCH.TestBatchError, "invalid LatitudeDegrees"):
                BATCH._read_official_sample_submission(nonfinite)
            generated_duplicate = root / "generated-duplicate.csv"
            generated_duplicate.write_text(
                "phone,UnixTimeMillis,LatitudeDegrees,LongitudeDegrees\n"
                "r1/pixel5,1,1,2\n"
                "r1/pixel5,1,1,2\n",
                encoding="utf-8",
            )
            with self.assertRaisesRegex(BATCH.TestBatchError, "duplicates key"):
                BATCH._read_generated_submission(generated_duplicate)
            generated_nonfinite = root / "generated-nonfinite.csv"
            generated_nonfinite.write_text(
                "phone,UnixTimeMillis,LatitudeDegrees,LongitudeDegrees\n"
                "r1/pixel5,1,nan,2\n",
                encoding="utf-8",
            )
            with self.assertRaisesRegex(BATCH.TestBatchError, "non-finite coordinate"):
                BATCH._read_generated_submission(generated_nonfinite)

    def test_test_role_requires_matching_sealed_authorization(self) -> None:
        fields = (
            "MessageType",
            "utcTimeMillis",
            "HardwareClockDiscontinuityCount",
            "Svid",
            "WlsPositionXEcefMeters",
            "WlsPositionYEcefMeters",
            "WlsPositionZEcefMeters",
        )
        with tempfile.TemporaryDirectory() as name:
            root = Path(name)
            device = root / "device_gnss.csv"
            with device.open("w", newline="", encoding="utf-8") as handle:
                writer = csv.DictWriter(handle, fieldnames=fields, lineterminator="\n")
                writer.writeheader()
                for timestamp in (1_609_797_015_000, 1_609_797_016_000):
                    writer.writerow(
                        {
                            "MessageType": "Raw",
                            "utcTimeMillis": str(timestamp),
                            "HardwareClockDiscontinuityCount": "0",
                            "Svid": "1",
                            "WlsPositionXEcefMeters": "-2704229.0",
                            "WlsPositionYEcefMeters": "-4288800.0",
                            "WlsPositionZEcefMeters": "3856689.0",
                        }
                    )
            record_path = root / "authorization.json"
            manifest_path = root / "authorization-manifest.json"
            dataset_id = "fixture/phone"
            record = {
                "schema_version": WLS.TEST_WLS_AUTHORIZATION_SCHEMA,
                "status": "sealed-before-test-payload-access",
                "algorithm_parameter_hash": WLS.MULTI_PHONE_HOLDOUT_ALGORITHM_PARAMETER_HASH,
                "algorithm_core_hash": WLS.MULTI_PHONE_HOLDOUT_ALGORITHM_CORE_HASH,
                "test_execution_contract": {
                    "authorized": True,
                    "role": "test",
                    "truth_free_phase": True,
                    "truth_access_forbidden": True,
                    "no_post_test_tuning": True,
                    "dataset_allowlist": [dataset_id],
                    "alignment_tolerance_ms": 10,
                    "skip_epochs": 1,
                },
                "sparse_wls_contract": {
                    "allow_missing_wls_epochs": True,
                    "single_phone_default_fail_closed": True,
                    "allow_timestamp_gaps_as_diagnostic": True,
                    "extrapolation_policy": "forbidden",
                    "partial_coordinate_triplet_policy": "fail-closed",
                    "nonfinite_coordinate_policy": "fail-closed",
                    "inconsistent_coordinate_policy": "fail-closed",
                },
                "source_hashes": {
                    "apps/commands/benchmarks/gnss_smartphone_wls.py": {
                        "sha256": WLS._sha256(Path(WLS.__file__))
                    }
                },
            }
            record_path.write_text(json.dumps(record), encoding="utf-8")
            manifest_path.write_text(
                json.dumps(
                    {
                        "schema_version": WLS.TEST_WLS_AUTHORIZATION_MANIFEST_SCHEMA,
                        "authorization_record": {
                            "sha256": WLS._sha256(record_path)
                        },
                        "algorithm_parameter_hash": WLS.MULTI_PHONE_HOLDOUT_ALGORITHM_PARAMETER_HASH,
                        "algorithm_core_hash": WLS.MULTI_PHONE_HOLDOUT_ALGORITHM_CORE_HASH,
                        "dataset_allowlist": [dataset_id],
                        "wls_source_sha256": WLS._sha256(Path(WLS.__file__)),
                    }
                ),
                encoding="utf-8",
            )
            result = WLS.extract_to_directory(
                device,
                root / "wls",
                skip_epochs=0,
                role="test",
                dataset_id=dataset_id,
                sealed_test_authorization=record_path,
                sealed_test_authorization_manifest=manifest_path,
                allow_missing_wls_epochs=True,
            )
            self.assertEqual(result["role"], "test")
            self.assertTrue(result["truth_free"])
            with self.assertRaises(WLS.WlsPositionError):
                WLS.extract_to_directory(
                    device,
                    root / "unauthorized",
                    skip_epochs=0,
                    role="test",
                    dataset_id=dataset_id,
                    allow_missing_wls_epochs=True,
                )


if __name__ == "__main__":
    unittest.main()
