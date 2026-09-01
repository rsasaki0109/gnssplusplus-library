#!/usr/bin/env python3
"""Truth-free contracts for the native PDC overlap stitch."""

from __future__ import annotations

import math
from pathlib import Path
import sys
import unittest


ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "apps" / "commands" / "benchmarks"))
import gnss_smartphone_native_fgo_pdc_windowed as windowed  # noqa: E402
import gnss_smartphone_native_fgo_pdc_windowed_stitch as stitch  # noqa: E402


def _row(index: int, *, x_offset: float = 0.0, key_index: int | None = None) -> windowed.PosRow:
    key_epoch = index if key_index is None else key_index
    x = 6370000.0 + float(index) * 10.0 + x_offset
    return windowed.PosRow(
        key=(2200, key_epoch * 1000),
        week=2200,
        tow=float(key_epoch),
        ecef=(x, 100.0, 200.0),
        text=(
            f"2200 {key_epoch:.6f} {x:.6f} 100 200 0 0 0 "
            "3 10 1 0 0 8"
        ),
    )


def _candidate(index: int, segment: int, start: int, rows: list[windowed.PosRow]) -> stitch.CandidateWindow:
    return stitch.CandidateWindow(
        index=index,
        segment_index=segment,
        start_epoch=start,
        count=len(rows),
        rows=tuple(rows),
        summary={},
        source_path=Path(f"window-{index:04d}/fgo.pos"),
    )


class SmartphoneNativeFgoPdcWindowedStitchTests(unittest.TestCase):
    def test_triangular_weight_is_low_at_edges_and_maximal_at_center(self) -> None:
        weights = [stitch.triangular_weight(index, 5) for index in range(5)]
        self.assertEqual(weights, [1.0, 2.0, 3.0, 2.0, 1.0])

    def test_equal_overlapping_windows_are_deterministic(self) -> None:
        canonical = [_row(index) for index in range(6)]
        windows = [
            _candidate(0, 0, 0, canonical[:4]),
            _candidate(1, 0, 2, canonical[2:]),
        ]
        retained, disagreements = stitch._mark_overlap_rejections(windows, canonical)
        selected, sources, stats = stitch._candidate_values(canonical, retained)
        self.assertEqual(disagreements[0]["decision"], "retain_both_window_contributions")
        self.assertEqual([row.ecef for row in selected], [row.ecef for row in canonical])
        self.assertEqual(sources[:2], ["single_window_unchanged", "single_window_unchanged"])
        self.assertEqual([item["contributors"] for item in stats], [1, 1, 2, 2, 1, 1])

    def test_constant_offset_is_blended_in_ecef_not_translated(self) -> None:
        canonical = [_row(index) for index in range(6)]
        offset_rows = [_row(index, x_offset=10.0) for index in range(2, 6)]
        windows = [
            _candidate(0, 0, 0, canonical[:4]),
            _candidate(1, 0, 2, offset_rows),
        ]
        retained, _ = stitch._mark_overlap_rejections(windows, canonical)
        selected, sources, _ = stitch._candidate_values(canonical, retained)
        self.assertEqual(sources[0], "single_window_unchanged")
        self.assertAlmostEqual(selected[2].ecef[0], canonical[2].ecef[0] + 10.0 / 3.0, places=6)
        self.assertAlmostEqual(selected[3].ecef[0], canonical[3].ecef[0] + 20.0 / 3.0, places=6)
        self.assertEqual(sources[4:], ["single_window_unchanged", "single_window_unchanged"])
        self.assertTrue(all(math.isfinite(value) for value in selected[2].ecef))

    def test_overlap_edge_outlier_rejects_both_windows_and_falls_back(self) -> None:
        canonical = [_row(index) for index in range(6)]
        outlier = [_row(index, x_offset=(200.0 if index == 2 else 0.0)) for index in range(2, 6)]
        windows = [
            _candidate(0, 0, 0, canonical[:4]),
            _candidate(1, 0, 2, outlier),
        ]
        retained, disagreements = stitch._mark_overlap_rejections(windows, canonical)
        selected, sources, _ = stitch._candidate_values(canonical, retained)
        self.assertEqual(disagreements[0]["decision"], "reject_both_window_contributions")
        self.assertEqual({item.contribution for item in retained}, {"rejected_overlap"})
        self.assertEqual([row.text for row in selected], [row.text for row in canonical])
        self.assertEqual(set(sources), {"sealed_v5_fallback_missing_or_rejected"})

    def test_missing_window_uses_sealed_fallback_only_for_uncovered_epochs(self) -> None:
        canonical = [_row(index) for index in range(6)]
        selected, sources, _ = stitch._candidate_values(
            canonical, [_candidate(0, 0, 0, canonical[:4])]
        )
        self.assertEqual([row.text for row in selected[:4]], [row.text for row in canonical[:4]])
        self.assertEqual(sources[4:], ["sealed_v5_fallback_missing_or_rejected"] * 2)

    def test_clock_boundary_never_blends_incompatible_segments(self) -> None:
        # The 3-second key gap creates two canonical clock segments.  The
        # segment-0 artifact deliberately collides with segment-1 keys; only
        # the segment-1 estimate may contribute there.
        canonical = [_row(0), _row(1), _row(5), _row(6)]
        segment_zero = [_row(0), _row(1), _row(5, x_offset=500.0, key_index=5), _row(6, x_offset=500.0, key_index=6)]
        segment_one = [_row(5, x_offset=10.0), _row(6, x_offset=10.0)]
        selected, sources, stats = stitch._candidate_values(
            canonical,
            [_candidate(0, 0, 0, segment_zero), _candidate(1, 1, 2, segment_one)],
        )
        self.assertAlmostEqual(selected[2].ecef[0], segment_one[0].ecef[0], places=6)
        self.assertEqual(sources[2], "single_window_unchanged")
        self.assertEqual(stats[2]["contributors"], 1)

    def test_ecef_blend_converts_to_finite_wgs84_columns(self) -> None:
        canonical = _row(0)
        left = windowed.PosRow(canonical.key, canonical.week, canonical.tow, (6378137.0, 0.0, 0.0), canonical.text)
        right = windowed.PosRow(canonical.key, canonical.week, canonical.tow, (6378137.0, 100.0, 0.0), canonical.text)
        blended = stitch._blended_row(canonical, [(left, 1.0), (right, 1.0)])
        self.assertAlmostEqual(blended.ecef[1], 50.0, places=6)
        fields = blended.text.split()
        self.assertTrue(all(math.isfinite(float(fields[index])) for index in range(2, 8)))
        self.assertGreater(float(fields[6]), 0.0)
        self.assertLess(float(fields[6]), 0.01)


if __name__ == "__main__":
    unittest.main()
