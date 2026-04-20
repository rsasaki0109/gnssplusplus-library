#!/usr/bin/env python3
"""Smoke tests for the pybind11-based Python bindings."""

from __future__ import annotations

import tempfile
import unittest
from pathlib import Path

import libgnsspp


ROOT_DIR = Path(__file__).resolve().parents[1]


def repo_data_exists(*relative_paths: str) -> bool:
    return all((ROOT_DIR / relative_path).exists() for relative_path in relative_paths)


class PythonBindingsSmokeTest(unittest.TestCase):
    def setUp(self) -> None:
        method = self._testMethodName
        if method in {
            "test_read_rinex_header_returns_expected_fields",
            "test_read_rinex_observation_epochs_returns_epoch_summaries",
            "test_solve_spp_file_returns_valid_solution_records",
            "test_solve_ppp_file_returns_valid_solution_records",
        } and not repo_data_exists("data/rover_static.obs", "data/navigation_static.nav"):
            self.skipTest("repo static test data is not available")

        if method == "test_solve_rtk_file_returns_short_baseline_solution_records" and not repo_data_exists(
            "data/short_baseline/TSK200JPN_R_20240010000_01D_30S_MO.rnx",
            "data/short_baseline/TSKB00JPN_R_20240010000_01D_30S_MO.rnx",
            "data/short_baseline/BRDC00IGS_R_20240010000_01D_MN.rnx",
        ):
            self.skipTest("repo short-baseline test data is not available")

    def test_read_rinex_header_returns_expected_fields(self) -> None:
        header = libgnsspp.read_rinex_header(str(ROOT_DIR / "data" / "rover_static.obs"))
        self.assertGreaterEqual(header["version"], 2.0)
        self.assertEqual(header["file_type"], "observation")
        self.assertEqual(len(header["approximate_position_ecef_m"]), 3)
        self.assertIn("observation_types", header)

    def test_read_rinex_observation_epochs_returns_epoch_summaries(self) -> None:
        epochs = libgnsspp.read_rinex_observation_epochs(
            str(ROOT_DIR / "data" / "rover_static.obs"),
            max_epochs=2,
        )
        self.assertEqual(len(epochs), 2)
        self.assertGreater(epochs[0]["num_observations"], 0)
        self.assertGreater(epochs[0]["num_satellites"], 0)
        self.assertGreaterEqual(epochs[0]["num_systems"], 1)
        self.assertIn("satellites", epochs[0])
        self.assertTrue(all(isinstance(token, str) for token in epochs[0]["satellites"]))

    def test_load_solution_and_statistics(self) -> None:
        with tempfile.TemporaryDirectory(prefix="gnss_python_bindings_") as temp_dir:
            solution_path = Path(temp_dir) / "sample.pos"
            solution_path.write_text(
                "% synthetic solution\n"
                "1316 518400.0 -3978242.0 3382841.0 3649903.0 35.0 139.0 10.0 5 9 1.0\n"
                "1316 518430.0 -3978243.0 3382840.0 3649902.0 35.0 139.0 10.0 6 10 2.5\n",
                encoding="ascii",
            )

            solution = libgnsspp.load_solution(str(solution_path))
            self.assertFalse(solution.is_empty())
            self.assertEqual(solution.size(), 2)

            last_solution = solution.last_solution()
            self.assertTrue(last_solution.is_valid())
            self.assertTrue(last_solution.is_fixed())
            self.assertEqual(last_solution.status, libgnsspp.SolutionStatus.PPP_FIXED)
            self.assertEqual(last_solution.num_satellites, 10)

            records = solution.records()
            self.assertEqual(len(records), 2)
            self.assertEqual(records[0].status, libgnsspp.SolutionStatus.PPP_FLOAT)
            self.assertEqual(records[1].status_name, "PPP_FIXED")
            self.assertIn("$GPGGA", records[1].to_nmea())

            stats = solution.statistics()
            self.assertEqual(stats.total_epochs, 2)
            self.assertEqual(stats.valid_solutions, 2)

            fixed_only = solution.filter_by_status(libgnsspp.SolutionStatus.PPP_FIXED)
            self.assertEqual(len(fixed_only), 1)
            self.assertEqual(fixed_only[0].status_name, "PPP_FIXED")

            start, end = solution.time_span()
            self.assertEqual(start.week, 1316)
            self.assertAlmostEqual(start.tow, 518400.0)
            self.assertAlmostEqual(end.tow, 518430.0)

    def test_coordinate_helpers_round_trip(self) -> None:
        ecef = (-3978242.0, 3382841.0, 3649903.0)
        llh = libgnsspp.ecef_to_geodetic_deg(ecef)
        self.assertEqual(len(llh), 3)
        self.assertGreater(llh[0], 30.0)
        self.assertGreater(llh[1], 130.0)

        ecef_round_trip = libgnsspp.geodetic_deg_to_ecef(llh)
        for original, converted in zip(ecef, ecef_round_trip):
            self.assertAlmostEqual(original, converted, delta=5.0)

        self.assertEqual(
            libgnsspp.solution_status_name(libgnsspp.SolutionStatus.PPP_FLOAT),
            "PPP_FLOAT",
        )

    def test_solve_spp_file_returns_valid_solution_records(self) -> None:
        solution = libgnsspp.solve_spp_file(
            str(ROOT_DIR / "data" / "rover_static.obs"),
            str(ROOT_DIR / "data" / "navigation_static.nav"),
            max_epochs=3,
        )
        self.assertGreaterEqual(solution.size(), 1)
        self.assertTrue(any(record.is_valid() for record in solution.records()))

    def test_solve_ppp_file_returns_valid_solution_records(self) -> None:
        solution = libgnsspp.solve_ppp_file(
            str(ROOT_DIR / "data" / "rover_static.obs"),
            str(ROOT_DIR / "data" / "navigation_static.nav"),
            max_epochs=5,
            kinematic_mode=False,
        )
        self.assertGreaterEqual(solution.size(), 1)
        self.assertTrue(
            any(
                record.status
                in (
                    libgnsspp.SolutionStatus.SPP,
                    libgnsspp.SolutionStatus.PPP_FLOAT,
                    libgnsspp.SolutionStatus.PPP_FIXED,
                )
                for record in solution.records()
            )
        )

    def test_solve_rtk_file_returns_short_baseline_solution_records(self) -> None:
        solution = libgnsspp.solve_rtk_file(
            str(ROOT_DIR / "data" / "short_baseline" / "TSK200JPN_R_20240010000_01D_30S_MO.rnx"),
            str(ROOT_DIR / "data" / "short_baseline" / "TSKB00JPN_R_20240010000_01D_30S_MO.rnx"),
            str(ROOT_DIR / "data" / "short_baseline" / "BRDC00IGS_R_20240010000_01D_MN.rnx"),
            max_epochs=10,
            iono_mode="off",
        )
        self.assertGreaterEqual(solution.size(), 1)
        self.assertTrue(
            any(
                record.status in (libgnsspp.SolutionStatus.FLOAT, libgnsspp.SolutionStatus.FIXED)
                for record in solution.records()
            )
        )


if __name__ == "__main__":
    unittest.main()
