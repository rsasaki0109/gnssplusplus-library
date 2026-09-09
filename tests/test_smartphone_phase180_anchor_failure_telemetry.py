"""Launch-free Phase180 regression for late Android clock-load failures.

The GNSS-first result has already returned before the Android IMU clock
boundary.  When the existing Phase94 diagnostic selector is enabled, that
later failure must publish the stage aggregate without relabelling the
successful GNSS-first stage as failed.
"""

from __future__ import annotations

from pathlib import Path
import unittest


ROOT = Path(__file__).resolve().parents[1]
APP = ROOT / "apps/native/gnss_fgo_imu_no_base.cpp"


class Phase180AnchorFailureTelemetryTests(unittest.TestCase):
    def test_elapsed_anchor_failure_preserves_stage_telemetry(self) -> None:
        source = APP.read_text(encoding="utf-8")
        start = source.index('std::cerr << "failed to load GNSS elapsed-time anchors: "')
        end = source.index("return 1;", start)
        branch = source[start:end]
        self.assertIn(
            'phase94RecordFailure(\n'
            '                        phase94_diagnostics, "gnss-elapsed-anchor-load",',
            branch,
        )
        self.assertIn("writePhase94StageDiagnostics", branch)
        self.assertNotIn("phase94_diagnostics.gnss_first.failure", branch)

    def test_utc_mapping_failure_is_a_late_boundary_failure(self) -> None:
        source = APP.read_text(encoding="utf-8")
        start = source.index('std::cerr << "failed to load raw GNSS UTC/GPS mapping: "')
        end = source.index("return 1;", start)
        branch = source[start:end]
        self.assertIn(
            'phase94RecordFailure(\n'
            '                            phase94_diagnostics, "gnss-utc-mapping-load",',
            branch,
        )
        self.assertIn("writePhase94StageDiagnostics", branch)
        self.assertNotIn("phase94_diagnostics.gnss_first.failure", branch)


if __name__ == "__main__":
    unittest.main()
