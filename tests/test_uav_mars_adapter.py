from __future__ import annotations

import importlib.util
from pathlib import Path
from types import SimpleNamespace
import tempfile
import unittest


ROOT = Path(__file__).resolve().parents[1]
MODULE_PATH = ROOT / "apps" / "commands" / "benchmarks" / "gnss_uav_mars_adapter.py"
SPEC = importlib.util.spec_from_file_location("gnss_uav_mars_adapter", MODULE_PATH)
assert SPEC and SPEC.loader
ADAPTER = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(ADAPTER)


def observation(sat: int, frequencies: list[float], week: int = 2300, tow: float = 100.0) -> SimpleNamespace:
    count = len(frequencies)
    return SimpleNamespace(
        time=SimpleNamespace(week=week, tow=tow),
        sat=sat,
        freqs=frequencies,
        CN0=[40.0] * count,
        LLI=[0] * count,
        code=[1] * count,
        psr=[22_000_000.0 + index for index in range(count)],
        psr_std=[1.0] * count,
        cp=[110_000_000.0 + index for index in range(count)],
        cp_std=[0.01] * count,
        dopp=[-1000.0 - index for index in range(count)],
        dopp_std=[0.1] * count,
        status=[0x03] * count,
    )


class UavMarsAdapterTests(unittest.TestCase):
    def test_writes_mixed_rinex_and_inventories_unmapped_signal(self) -> None:
        epoch = SimpleNamespace(
            meas=[
                observation(1, [1_575_420_000.0, 1_227_600_000.0]),
                observation(60, [1_575_420_000.0, 1_207_140_000.0]),
                observation(98, [1_561_098_000.0, 1_207_140_000.0, 9.0]),
            ]
        )
        with tempfile.TemporaryDirectory() as directory:
            output = Path(directory) / "observations.rnx"
            summary = ADAPTER.write_rinex_observations(output, [epoch], (1.0, 2.0, 3.0))
            text = output.read_text(encoding="ascii")
            self.assertIn("G01", text)
            self.assertIn("E01", text)
            self.assertIn("C01", text)
            self.assertEqual(summary["epochs"], 1)
            self.assertEqual(summary["mapped_signal_rows"]["G:1C"], 1)
            self.assertEqual(summary["unmapped_signal_rows"]["C:9.000"], 1)

    def test_rejects_inconsistent_signal_arrays(self) -> None:
        obs = observation(1, [1_575_420_000.0])
        obs.status = []
        with tempfile.TemporaryDirectory() as directory:
            with self.assertRaises(SystemExit) as raised:
                ADAPTER.write_rinex_observations(
                    Path(directory) / "observations.rnx",
                    [SimpleNamespace(meas=[obs])],
                    (1.0, 2.0, 3.0),
                )
            self.assertIn("inconsistent signal arrays", str(raised.exception))

    def test_rejects_non_monotonic_gnss_time(self) -> None:
        first = SimpleNamespace(meas=[observation(1, [1_575_420_000.0], tow=100.0)])
        second = SimpleNamespace(meas=[observation(1, [1_575_420_000.0], tow=100.0)])
        with tempfile.TemporaryDirectory() as directory:
            with self.assertRaises(SystemExit) as raised:
                ADAPTER.write_rinex_observations(
                    Path(directory) / "observations.rnx", [first, second], (1.0, 2.0, 3.0)
                )
            self.assertIn("not strictly increasing", str(raised.exception))


if __name__ == "__main__":
    unittest.main()
