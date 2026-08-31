#!/usr/bin/env python3
"""Truth-free contracts for the published GSDC MAT-result promotion lane."""

from __future__ import annotations

import csv
import io
from pathlib import Path
import struct
import sys
import tempfile
import unittest

import numpy as np


ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "apps" / "commands" / "benchmarks"))
import gnss_smartphone_gsdc2023_upstream_mat as upstream  # noqa: E402


def _synthetic_cells() -> list[object]:
    """Build a small R2024a FileWrapper heap with one gt.Gpos object."""
    strings = b"n\0llh\0xyz\0enu\0orgllh\0orgxyz\0Gpos\0gt\0v2\0v3\0Gvel\0"
    strings += b"\0" * ((-len(strings)) % 8)
    h2 = 40 + len(strings)
    h3 = h2 + 3 * 16
    h4 = h3
    h5 = h4 + 2 * 24
    props = bytearray()
    props.extend(struct.pack("<I", 0))
    props.extend(b"\0" * 4)
    props.extend(struct.pack("<I", 6))
    for name_index, heap_index in ((1, 0), (2, 1), (3, 2), (4, 3), (5, 4), (6, 5)):
        props.extend(struct.pack("<3I", name_index, 1, heap_index))
    while len(props) % 8:
        props.append(0)
    property_start = h5
    property_end = property_start + len(props)
    segment = bytearray(struct.pack("<10I", 4, 11, h2, h3, h4, h5, property_end, property_end, property_end, property_end))
    segment.extend(strings)
    segment.extend(struct.pack("<4I", 0, 0, 0, 0))
    segment.extend(struct.pack("<4I", 8, 7, 0, 0))
    segment.extend(struct.pack("<4I", 8, 11, 0, 0))
    segment.extend(struct.pack("<6I", 0, 0, 0, 0, 0, 0))
    segment.extend(struct.pack("<6I", 1, 0, 0, 0, 1, 1))
    segment.extend(props)
    cells: list[object] = [np.frombuffer(bytes(segment), dtype=np.uint8).reshape(-1, 1)]
    cells.extend(
        [
            np.empty((1, 0), dtype=float),
            np.array([[2.0]]),
            np.array([[35.0, 139.0, 10.0], [35.0001, 139.0001, 10.1]]),
            np.zeros((2, 3)),
            np.zeros((2, 3)),
            np.array([[35.0, 139.0, 10.0]]),
            np.zeros((1, 3)),
        ]
    )
    return cells


class SmartphoneGsdC2023UpstreamMatTests(unittest.TestCase):
    def test_synthetic_mcos_property_table_decodes_gpos(self) -> None:
        objects = upstream._parse_mcos_properties(_synthetic_cells())
        self.assertEqual(objects[1]["__class__"], "gt.Gpos")
        np.testing.assert_allclose(objects[1]["llh"], [[35.0, 139.0, 10.0], [35.0001, 139.0001, 10.1]])

    def test_malformed_workspace_is_fail_closed(self) -> None:
        with self.assertRaises(upstream.UpstreamMatError):
            upstream._workspace_from_bytes(b"\x00\x01IM\x01\x00\x00\x00")

    def test_reconcile_exact_interpolation_edge_and_extra_without_sample_values(self) -> None:
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            sample = root / "sample.csv"
            sample.write_text(
                "tripId,UnixTimeMillis,LatitudeDegrees,LongitudeDegrees\n"
                "r/p,1000,34.640195,-120.589642\n"
                "r/p,2000,34.640195,-120.589642\n"
                "r/p,3000,34.640195,-120.589642\n",
                encoding="ascii",
            )
            positions = [
                upstream.Position("r/p", 1000, 35.0, 139.0, 10.0, "fixture"),
                upstream.Position("r/p", 3000, 35.0002, 139.0002, 10.2, "fixture"),
            ]
            output = root / "submission.csv"
            manifest = root / "manifest.json"
            result = upstream.reconcile_sample(sample, positions, output, manifest)
            self.assertEqual(result["source_counts"], {"exact": 2, "edge_hold": 0, "interpolated": 1, "unresolved": 0})
            self.assertEqual(result["extra_upstream_keys_dropped"], 0)
            self.assertEqual(output.read_text(encoding="ascii").splitlines()[0], "tripId,UnixTimeMillis,LatitudeDegrees,LongitudeDegrees")
            self.assertNotIn("34.640195", output.read_text(encoding="ascii"))

    def test_sample_dummy_is_never_emitted(self) -> None:
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            sample = root / "sample.csv"
            sample.write_text(
                "tripId,UnixTimeMillis,LatitudeDegrees,LongitudeDegrees\n"
                "r/p,1000,0,0\n",
                encoding="ascii",
            )
            row = upstream.Position("r/p", 1000, *upstream.KNOWN_SAMPLE_DUMMY, 0.0, "fixture")
            with self.assertRaises(upstream.UpstreamMatError):
                upstream.reconcile_sample(sample, [row], root / "out.csv", root / "manifest.json")

    def test_materialized_published_result_decodes_when_available(self) -> None:
        path = ROOT / "data/gsdc2023/materialized/dataset_2023/train/2023-05-24-20-26-us-ca-sjc-ge2/pixel7pro/result_gnss_imu.mat"
        if not path.is_file():
            self.skipTest("materialized MAT smoke input is not present")
        decoded = upstream.decode_result_bytes(path.read_bytes())
        self.assertEqual(decoded["llh"].shape[1], 3)
        self.assertTrue(np.isfinite(decoded["llh"]).all())

    def test_phone_data_workspace_is_the_upstream_epoch_authority(self) -> None:
        path = ROOT / "data/gsdc2023/materialized/dataset_2023/train/2023-05-24-20-26-us-ca-sjc-ge2/pixel7pro/phone_data.mat"
        if not path.is_file():
            self.skipTest("materialized phone_data.mat smoke input is not present")
        timestamps = upstream.timestamps_from_phone_data_bytes(path.read_bytes())
        self.assertEqual(len(timestamps), 1384)
        self.assertEqual(timestamps[:2], [1684960017999, 1684960018999])
        self.assertEqual(timestamps, sorted(set(timestamps)))


if __name__ == "__main__":
    unittest.main()
