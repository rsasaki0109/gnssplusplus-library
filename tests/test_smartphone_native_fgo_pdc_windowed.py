#!/usr/bin/env python3
"""Truth-free contracts for the bounded smartphone PDC window lane."""

from __future__ import annotations

from pathlib import Path
import sys
import tempfile
import unittest


ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "apps" / "commands" / "benchmarks"))
import gnss_smartphone_native_fgo_pdc_windowed as windowed  # noqa: E402


def _row(index: int, x: float | None = None) -> windowed.PosRow:
    value = float(index if x is None else x)
    return windowed.PosRow(
        key=(2200, index * 1000),
        week=2200,
        tow=index,
        ecef=(6370000.0 + value, 100.0, 200.0),
        text=f"2200 {index:.6f} {6370000.0 + value:.6f} 100 200 0 0 0 3 10 1 0 0 8",
    )


def _result(index: int, start: int, rows: list[windowed.PosRow]) -> windowed.WindowResult:
    spec = windowed.WindowSpec(index, 0, start, len(rows))
    return windowed.WindowResult(
        spec=spec,
        source="windowed_native_fgo",
        rows=tuple(rows),
        summary={},
        command=(),
        return_code=0,
        wall_seconds=0.01,
        peak_rss_kib=1,
        reason=None,
        artifact_hashes={},
    )


class SmartphoneNativeFgoPdcWindowedTests(unittest.TestCase):
    def test_window_schedule_is_covering_and_final_window_is_full(self) -> None:
        rows = [_row(index) for index in range(250)]
        specs = windowed.window_specs(rows)
        self.assertEqual([(item.start_epoch, item.count) for item in specs], [(0, 120), (60, 120), (120, 120), (130, 120)])
        covered = {
            index
            for item in specs
            for index in range(item.start_epoch, item.start_epoch + item.count)
        }
        self.assertEqual(covered, set(range(250)))

    def test_overlap_stitch_prefers_window_interior_and_is_deterministic(self) -> None:
        canonical = [_row(index) for index in range(180)]
        first = _result(0, 0, canonical[:120])
        second = _result(1, 60, canonical[60:180])
        stitched, report = windowed.stitch_rows(canonical, [first, second])
        self.assertEqual([item.key for item in stitched], [item.key for item in canonical])
        self.assertEqual(report["source_counts"], {"windowed_native_fgo": 180, "sealed_native_fgo_fallback": 0})
        # At global 90 the second window is one sample deeper from its edge;
        # at global 60 the first window wins the deterministic tie.
        self.assertEqual(stitched[60].text, canonical[60].text)
        self.assertEqual(stitched[90].text, canonical[90].text)

    def test_failed_window_is_not_a_coordinate_source(self) -> None:
        canonical = [_row(index) for index in range(120)]
        failed = windowed.WindowResult(
            spec=windowed.WindowSpec(0, 0, 0, 120),
            source="sealed_native_fgo_fallback",
            rows=(), summary=None, command=(), return_code=1,
            wall_seconds=0.1, peak_rss_kib=1, reason="solver failure",
            artifact_hashes={},
        )
        stitched, report = windowed.stitch_rows(canonical, [failed])
        self.assertEqual([row.text for row in stitched], [row.text for row in canonical])
        self.assertEqual(report["fallback_key_count"], 120)

    def test_physical_jump_marks_stitch_fail_closed(self) -> None:
        canonical = [_row(index) for index in range(3)]
        jumped = [_row(0), _row(1, 1000.0), _row(2)]
        result = _result(0, 0, jumped)
        _, report = windowed.stitch_rows(canonical, [result])
        self.assertFalse(report["continuity_ok"])
        self.assertGreater(report["above_transition_speed_count"], 0)

    def test_position_reader_rejects_nonfinite_and_out_of_earth_rows(self) -> None:
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory) / "bad.pos"
            path.write_text("2200 0.0 nan 0 0 0 0 0 3 4 1 0 0 8\n", encoding="utf-8")
            with self.assertRaises(windowed.WindowedPdcError):
                windowed.read_pos(path)
            path.write_text("2200 0.0 100 100 100 0 0 0 3 4 1 0 0 8\n", encoding="utf-8")
            with self.assertRaises(windowed.WindowedPdcError):
                windowed.read_pos(path)

    def test_command_is_fixed_no_base_and_truth_free(self) -> None:
        command = windowed.build_command(
            Path("gnss_fgo"), Path("rover.obs"), Path("brdc.nav"),
            windowed.WindowSpec(2, 0, 60, 120), Path("out"),
        )
        self.assertEqual(command, windowed.build_command(
            Path("gnss_fgo"), Path("rover.obs"), Path("brdc.nav"),
            windowed.WindowSpec(2, 0, 60, 120), Path("out"),
        ))
        self.assertNotIn("--base", command)
        for flag in (
            "--corrected-undifferenced-doppler-factors",
            "--velocity-states", "--velocity-motion-factors",
            "--carrier-phase-factors", "--ambiguity-between-factors",
            "--no-dd-factors",
        ):
            self.assertIn(flag, command)
        self.assertEqual(command[command.index("--max-iterations") + 1], "1000")


if __name__ == "__main__":
    unittest.main()
