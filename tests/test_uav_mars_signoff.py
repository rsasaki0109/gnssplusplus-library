from __future__ import annotations

import importlib.util
import math
from pathlib import Path
import tempfile
import unittest


ROOT = Path(__file__).resolve().parents[1]
MODULE_PATH = ROOT / "apps" / "commands" / "benchmarks" / "gnss_uav_mars_signoff.py"
SPEC = importlib.util.spec_from_file_location("gnss_uav_mars_signoff", MODULE_PATH)
assert SPEC and SPEC.loader
SIGNOFF = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(SIGNOFF)


class UavMarsSignoffTests(unittest.TestCase):
    def test_body_flu_quaternion_uses_clockwise_enu_yaw_without_fitted_offset(self) -> None:
        yaw_ccw_deg = -96.0
        half = math.radians(yaw_ccw_deg) / 2.0
        row = {"qx": "0", "qy": "0", "qz": str(math.sin(half)), "qw": str(math.cos(half))}
        self.assertAlmostEqual(SIGNOFF.quaternion_clockwise_enu_yaw_deg(row), 96.0, places=10)

    def test_wrapped_yaw_error_crosses_north_without_discontinuity(self) -> None:
        self.assertAlmostEqual(SIGNOFF.wrapped_error_deg(359.0, 1.0), 2.0)

    def test_rejects_non_unit_attitude_quaternion(self) -> None:
        with self.assertRaises(SystemExit):
            SIGNOFF.quaternion_clockwise_enu_yaw_deg(
                {"qx": "0", "qy": "0", "qz": "0", "qw": "0.5"}
            )

    def test_rejects_non_monotonic_position_time(self) -> None:
        content = "\n".join(
            (
                "2285 100.0 0 0 0 1 2 3 1 10 1",
                "2285 100.0 0 0 0 1 2 3 1 10 1",
            )
        )
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory) / "solution.pos"
            path.write_text(content + "\n", encoding="ascii")
            with self.assertRaises(SystemExit) as raised:
                SIGNOFF.read_position(path, 18)
            self.assertIn("not strictly increasing", str(raised.exception))


if __name__ == "__main__":
    unittest.main()
