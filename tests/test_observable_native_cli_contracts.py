"""Characterize observable-native CLI parsing and output contracts."""

from __future__ import annotations

import csv
import json
import math
import os
import subprocess
import tempfile
import unittest
from dataclasses import dataclass
from pathlib import Path
from typing import Sequence


ROOT_DIR = Path(__file__).resolve().parents[1]
BUILD_CONFIGS = ("Release", "RelWithDebInfo", "Debug", "MinSizeRel")


@dataclass(frozen=True)
class CliVariant:
    binary_name: str
    output_columns: str
    has_tdcp: bool = False
    has_velocity: bool = False


VARIANTS = (
    CliVariant("gnss_pos_pd", "position/clock"),
    CliVariant("gnss_pos_pdc", "position/clock", has_tdcp=True),
    CliVariant("gnss_pos_vel_pd", "position/velocity", has_velocity=True),
    CliVariant(
        "gnss_pos_vel_pdc",
        "position/velocity",
        has_tdcp=True,
        has_velocity=True,
    ),
)

PRESETS = {
    "gnss_pos_pd": "taroz-pos-pd",
    "gnss_pos_pdc": "taroz-pos-pdc",
    "gnss_pos_vel_pd": "taroz-pd",
    "gnss_pos_vel_pdc": "taroz-pdc",
}
RECEIVER_Y_M = 6_378_137.0

POSITION_OUTPUT_COLUMNS = (
    "epoch,gps_week,gps_tow,spp_x_m,spp_y_m,spp_z_m,"
    "fgo_x_m,fgo_y_m,fgo_z_m,fgo_c_gps_m,fgo_c_glo_m,"
    "fgo_c_gal_m,fgo_c_qzs_m,fgo_c_bds_m,clock_jump"
).split(",")
VELOCITY_OUTPUT_COLUMNS = (
    "epoch,gps_week,gps_tow,spp_x_m,spp_y_m,spp_z_m,"
    "fgo_x_m,fgo_y_m,fgo_z_m,fgo_vx_mps,fgo_vy_mps,fgo_vz_mps,"
    "fgo_c_gps_m,fgo_c_glo_m,fgo_c_gal_m,fgo_c_qzs_m,fgo_c_bds_m,"
    "fgo_clock_drift_mps,clock_jump"
).split(",")

POSITION_PD_FACTOR_COLUMNS = (
    "factor_type,epoch_index,gps_week,gps_tow,satellite,signal,"
    "previous_epoch_index,midpoint_gps_week,midpoint_gps_tow,clock_group,"
    "snr_dbhz,elevation_deg,sigma_p_m,sigma_d_mps,res_pc_m,res_d_mps,"
    "measured_range_rate_mps,modeled_range_rate_mps,"
    "satellite_clock_drift_mps,dt_s,final_residual_m,los_x,los_y,los_z"
).split(",")
POSITION_PDC_FACTOR_COLUMNS = (
    "factor_type,epoch_index,gps_week,gps_tow,satellite,signal,"
    "previous_epoch_index,midpoint_gps_week,midpoint_gps_tow,clock_group,"
    "snr_dbhz,elevation_deg,sigma_p_m,sigma_d_mps,sigma_c_m,res_pc_m,"
    "res_d_mps,res_tdcp_m,measured_range_rate_mps,modeled_range_rate_mps,"
    "satellite_clock_drift_mps,dt_s,previous_carrier_residual_m,"
    "current_carrier_residual_m,previous_raw_carrier_cycles,"
    "current_raw_carrier_cycles,wavelength_m,final_residual_m,los_x,los_y,"
    "los_z"
).split(",")
VELOCITY_PD_FACTOR_COLUMNS = (
    "factor_type,epoch_index,gps_week,gps_tow,satellite,signal,clock_group,"
    "snr_dbhz,elevation_deg,sigma_p_m,sigma_d_mps,res_pc_m,res_d_mps,"
    "measured_range_rate_mps,modeled_range_rate_mps,"
    "satellite_clock_drift_mps,final_residual_m,los_x,los_y,los_z"
).split(",")
VELOCITY_PDC_FACTOR_COLUMNS = (
    "factor_type,epoch_index,gps_week,gps_tow,satellite,signal,"
    "previous_epoch_index,clock_group,snr_dbhz,elevation_deg,"
    "previous_elevation_deg,sigma_p_m,sigma_d_mps,sigma_c_m,res_pc_m,"
    "res_d_mps,res_tdcp_m,measured_range_rate_mps,modeled_range_rate_mps,"
    "satellite_clock_drift_mps,previous_carrier_residual_m,"
    "current_carrier_residual_m,previous_raw_carrier_cycles,"
    "current_raw_carrier_cycles,wavelength_m,final_residual_m,los_x,los_y,"
    "los_z"
).split(",")

EXPECTED_NUMERICS = {
    "gnss_pos_pd": {
        "initial_cost": 757717.4850571656,
        "residual_rms_mps": 848200.4549021618,
        "first_doppler_residual_mps": -199.81773244051908,
        "first_measured_range_rate_mps": 190.38881963476405,
    },
    "gnss_pos_pdc": {
        "initial_cost": 764080.5166325375,
        "residual_rms_mps": 734563.1558817065,
        "first_doppler_residual_mps": -199.81773244051908,
        "first_measured_range_rate_mps": 190.38881963476405,
    },
    "gnss_pos_vel_pd": {
        "initial_cost": 758808.2748244688,
        "residual_rms_mps": 734563.1482411561,
        "first_doppler_residual_mps": -199.88024226507372,
        "first_measured_range_rate_mps": 190.29367279836487,
    },
    "gnss_pos_vel_pdc": {
        "initial_cost": 765171.3063998406,
        "residual_rms_mps": 657013.2662043745,
        "first_doppler_residual_mps": -199.88024226507372,
        "first_measured_range_rate_mps": 190.29367279836487,
    },
}

COMMON_TUNING_OPTIONS = (
    ("--seed-match-tolerance", "0"),
    ("--seed-interpolation-max-gap", "0"),
    ("--min-snr", "0"),
    ("--min-elevation", "0"),
    ("--pseudorange-sigma-zenith", "1"),
    ("--doppler-sigma-zenith", "1"),
    ("--position-prior-sigma", "1"),
    ("--clock-prior-sigma", "1"),
    ("--clock-motion-sigma", "1"),
    ("--clock-jump-sigma", "1"),
    ("--isb-motion-sigma", "1"),
    ("--huber-threshold", "1"),
)
VELOCITY_TUNING_OPTIONS = (
    ("--velocity-prior-sigma", "1"),
    ("--clock-drift-prior-sigma", "1"),
    ("--motion-sigma", "1"),
    ("--clock-drift-between-sigma", "1"),
)
TDCP_TUNING_OPTION = ("--tdcp-sigma-zenith", "1")
INTEGER_LIMIT_OPTIONS = ("--skip-epochs", "--max-epochs", "--max-iterations")


class ObservableNativeCliContractsTest(unittest.TestCase):
    """Lock behavior that the upcoming shared frontend must preserve."""

    maxDiff = None

    @classmethod
    def setUpClass(cls) -> None:
        binary_dir = Path(
            os.environ.get("GNSSPP_BINARY_DIR", ROOT_DIR / "build")
        ).resolve()
        cls.executables: dict[str, Path] = {}
        for variant in VARIANTS:
            executable_name = (
                f"{variant.binary_name}.exe" if os.name == "nt"
                else variant.binary_name
            )
            candidates = [
                binary_dir / "apps" / executable_name,
                *(
                    binary_dir / "apps" / config / executable_name
                    for config in BUILD_CONFIGS
                ),
                *(
                    binary_dir / config / "apps" / executable_name
                    for config in BUILD_CONFIGS
                ),
                *(
                    binary_dir / config / executable_name
                    for config in BUILD_CONFIGS
                ),
            ]
            executable = next(
                (candidate for candidate in candidates if candidate.is_file()),
                None,
            )
            if executable is None:
                rendered = ", ".join(str(candidate) for candidate in candidates)
                raise RuntimeError(
                    f"{variant.binary_name} executable not found; checked: "
                    f"{rendered}"
                )
            cls.executables[variant.binary_name] = executable

    @classmethod
    def run_cli(
        cls,
        variant: CliVariant,
        *options: str,
    ) -> subprocess.CompletedProcess[str]:
        return subprocess.run(
            [str(cls.executables[variant.binary_name]), *options],
            check=False,
            capture_output=True,
            text=True,
            encoding="utf-8",
            errors="replace",
            timeout=10,
        )

    @staticmethod
    def rinex_header_line(content: str, label: str) -> str:
        return f"{content:<60}{label}\n"

    @staticmethod
    def rinex_float(value: float) -> str:
        return f"{value:19.12E}".replace("E", "D")

    @staticmethod
    def rinex_observation_field(value: float) -> str:
        return f"{value:14.3f}  "

    @classmethod
    def write_success_fixture(cls, directory: Path) -> tuple[Path, Path, Path]:
        """Write two epochs that exercise P/D/TDCP without external datasets."""
        observation_path = directory / "observable-contract.obs"
        navigation_path = directory / "observable-contract.nav"
        seed_path = directory / "observable-contract.pos"

        observation_text = cls.rinex_header_line(
            "     3.04           OBSERVATION DATA    G",
            "RINEX VERSION / TYPE",
        )
        observation_text += cls.rinex_header_line(
            "G    4 C1C L1C D1C S1C",
            "SYS / # / OBS TYPES",
        )
        observation_text += cls.rinex_header_line("", "END OF HEADER")
        observations = (
            (0.0, 22_000_000.0, 115_610_000.0, -1000.0),
            (1.0, 22_000_100.0, 115_610_520.0, -1001.0),
        )
        for second, pseudorange, carrier_phase, doppler in observations:
            observation_text += (
                f"> 2022 03 10 00 00 {second:10.7f}  0  1\n"
                "G03"
                f"{cls.rinex_observation_field(pseudorange)}"
                f"{cls.rinex_observation_field(carrier_phase)}"
                f"{cls.rinex_observation_field(doppler)}"
                f"{cls.rinex_observation_field(45.0)}\n"
            )
        observation_path.write_text(observation_text, encoding="ascii")

        navigation_text = cls.rinex_header_line(
            "     3.04           NAVIGATION DATA     G",
            "RINEX VERSION / TYPE",
        )
        navigation_text += cls.rinex_header_line("", "END OF HEADER")
        navigation_text += (
            "G03 2022  3 10  0  0  0"
            f"{cls.rinex_float(1.25e-4)}"
            f"{cls.rinex_float(-2.0e-12)}"
            f"{cls.rinex_float(0.0)}\n"
        )
        # This healthy GPS ephemeris is shared with the RINEX writer round-trip
        # fixture.  The WGS84-equator +Y receiver seed keeps G03 well above the
        # default 15-degree elevation mask at both epochs.
        navigation_rows = (
            (12.0, 88.0, 4.3e-9, 0.12),
            (1.2e-6, 0.01, -2.2e-6, 5153.7954775),
            (345_600.0, 3.1e-8, 1.1, -4.1e-8),
            (0.94, 250.0, 0.52, -8.1e-9),
            (1.1e-10, 0.0, 2200.0, 0.0),
            (2.4, 0.0, -2.3e-8, 25.0),
            (345_570.0, 0.0, 0.0, 0.0),
        )
        for row in navigation_rows:
            navigation_text += "    "
            navigation_text += "".join(cls.rinex_float(value) for value in row)
            navigation_text += "\n"
        navigation_path.write_text(navigation_text, encoding="ascii")

        seed_path.write_text(
            f"2200 345600 0 {RECEIVER_Y_M:.0f} 0\n"
            f"2200 345601 0 {RECEIVER_Y_M:.0f} 0\n",
            encoding="ascii",
        )
        return observation_path, navigation_path, seed_path

    @staticmethod
    def expected_summary(variant: CliVariant) -> dict[str, object]:
        numerics = EXPECTED_NUMERICS[variant.binary_name]
        expected: dict[str, object] = {
            "preset": PRESETS[variant.binary_name],
            "backend": "eigen",
            "debug_problem_only": True,
            "optimized_epochs": 2,
            "valid_position_epochs": 2,
            "seed_matched_epochs": 2,
            "seed_interpolated_epochs": 0,
            "nsat": 1,
            "pseudorange_factors": 2,
            "position_prior_factors": 2,
            "clock_prior_factors": 2,
            "clock_motion_factors": 1,
            "min_snr_dbhz": 35.0,
            "min_elevation_deg": 15.0,
            "pseudorange_sigma_zenith_m": 3.0,
            "doppler_sigma_zenith_mps": 0.2,
            "position_prior_sigma_m": 1000.0,
            "clock_prior_sigma_m": 1_000_000.0,
            "clock_jump_sigma_m": 1_000_000.0,
            "huber_threshold_sigma": 1.234,
            "iterations": 0,
            "converged": False,
            "initial_cost": numerics["initial_cost"],
            "final_cost": numerics["initial_cost"],
            "residual_rms_mps": numerics["residual_rms_mps"],
        }
        if variant.has_velocity:
            expected.update(
                {
                    "valid_velocity_epochs": 2,
                    "doppler_factors": 2,
                    "velocity_prior_factors": 2,
                    "clock_drift_prior_factors": 2,
                    "motion_factors": 1,
                    "clock_drift_between_factors": 1,
                    "graph_factors": 16 if variant.has_tdcp else 15,
                    "graph_values": 8,
                    "velocity_prior_sigma_mps": 1000.0,
                    "clock_drift_prior_sigma_mps": 1000.0,
                    "motion_sigma_m": 0.1,
                    "clock_motion_sigma_m": 0.1,
                    "clock_drift_between_sigma_mps": 0.1,
                }
            )
        else:
            expected.update(
                {
                    "valid_clock_epochs": 2,
                    "doppler_factors": 1,
                    "graph_factors": 9 if variant.has_tdcp else 8,
                    "graph_values": 4,
                    "clock_motion_sigma_m": 100.0,
                }
            )
        if variant.has_tdcp:
            expected.update(
                {
                    "tdcp_factors": 1,
                    "tdcp_sigma_zenith_m": 0.05,
                }
            )
        return expected

    @staticmethod
    def expected_factor_columns(variant: CliVariant) -> list[str]:
        if variant.has_velocity:
            return (
                VELOCITY_PDC_FACTOR_COLUMNS
                if variant.has_tdcp
                else VELOCITY_PD_FACTOR_COLUMNS
            )
        return (
            POSITION_PDC_FACTOR_COLUMNS
            if variant.has_tdcp
            else POSITION_PD_FACTOR_COLUMNS
        )

    def assert_contract_float(self, actual: str | float, expected: float) -> None:
        actual_float = float(actual)
        # Keep the contract sensitive to millimetre-scale range changes while
        # tolerating the final few libm digits across supported toolchains.
        self.assertTrue(
            math.isclose(actual_float, expected, rel_tol=1e-9, abs_tol=1e-9),
            msg=f"{actual_float!r} != {expected!r}",
        )

    @classmethod
    def expected_usage(cls, variant: CliVariant, message: str) -> str:
        executable = cls.executables[variant.binary_name]
        option_lines = [
            "Options:",
            "  --out-csv PATH                 "
            f"Write per-epoch {variant.output_columns} CSV.",
            "  --factor-debug-csv PATH        Write per-factor debug CSV.",
            "  --graph-csv PATH               Write taroz-like graph detail CSV.",
            "  --summary-json PATH            Write JSON summary.",
            "  --max-epochs N                 Limit processed epochs; 0 means all.",
            "  --skip-epochs N                Skip leading observation epochs.",
            "  --max-iterations N             IRLS iteration limit.",
        ]
        if variant.has_tdcp:
            option_lines.append(
                "  --tdcp-sigma-zenith M          TDCP zenith sigma in meters."
            )
        option_lines.extend(
            [
                "  --debug-problem-only           Build factors and skip optimization.",
                "  --quiet                        Suppress human-readable summary.",
            ]
        )
        return (
            f"Error: {message}\n\n"
            f"Usage: {executable} --obs rover.obs --nav base.nav "
            "--seed-pos rover_1Hz_spp.pos [options]\n\n"
            + "\n".join(option_lines)
            + "\n"
        )

    def assert_usage_error(
        self,
        variant: CliVariant,
        options: Sequence[str],
        message: str,
    ) -> None:
        result = self.run_cli(variant, *options)

        self.assertEqual(result.returncode, 2, msg=result.stderr)
        self.assertEqual(result.stdout, "")
        self.assertEqual(result.stderr, self.expected_usage(variant, message))

    def assert_missing_observation_error(
        self,
        variant: CliVariant,
        missing_observation: Path,
        *extra_options: str,
    ) -> subprocess.CompletedProcess[str]:
        result = self.run_cli(
            variant,
            "--obs",
            str(missing_observation),
            "--nav",
            str(missing_observation.with_suffix(".nav")),
            "--seed-pos",
            str(missing_observation.with_suffix(".pos")),
            *extra_options,
        )

        self.assertEqual(result.returncode, 1, msg=result.stderr)
        self.assertEqual(result.stdout, "")
        self.assertEqual(
            result.stderr,
            f"Error: failed to open observation file: {missing_observation}\n",
        )
        return result

    def test_help_and_short_help_are_usage_errors(self) -> None:
        for variant in VARIANTS:
            for option in ("--help", "-h"):
                with self.subTest(binary=variant.binary_name, option=option):
                    self.assert_usage_error(
                        variant,
                        (option,),
                        "help requested",
                    )

    def test_required_argument_order_is_stable(self) -> None:
        cases = (
            ((), "--obs is required"),
            (("--obs", "rover.obs"), "--nav is required"),
            (
                ("--obs", "rover.obs", "--nav", "base.nav"),
                "--seed-pos is required",
            ),
        )
        for variant in VARIANTS:
            for options, message in cases:
                with self.subTest(binary=variant.binary_name, message=message):
                    self.assert_usage_error(variant, options, message)

    def test_parser_diagnostics_are_stable(self) -> None:
        cases = (
            (("--unknown",), "unknown argument: --unknown"),
            (("--obs",), "missing value for --obs"),
            (
                ("--max-epochs", "not-an-integer"),
                "invalid integer for --max-epochs: not-an-integer",
            ),
            (
                ("--huber-threshold", "not-a-number"),
                "invalid number for --huber-threshold: not-a-number",
            ),
        )
        for variant in VARIANTS:
            for options, message in cases:
                with self.subTest(binary=variant.binary_name, message=message):
                    self.assert_usage_error(variant, options, message)

    def test_numeric_validation_contracts_are_stable(self) -> None:
        common_cases = [
            ((option, "-1"), "epoch and iteration limits must be non-negative")
            for option in INTEGER_LIMIT_OPTIONS
        ]
        common_cases.extend(
            [
                (
                    ("--seed-match-tolerance", "-1"),
                    "sigma, threshold, and tolerance values must be positive",
                ),
                (
                    ("--huber-threshold", "0"),
                    "sigma, threshold, and tolerance values must be positive",
                ),
            ]
        )
        for variant in VARIANTS:
            cases = list(common_cases)
            if variant.has_tdcp:
                cases.append(
                    (
                        ("--tdcp-sigma-zenith", "0"),
                        "sigma, threshold, and tolerance values must be positive",
                    )
                )
            if variant.has_velocity:
                cases.append(
                    (
                        ("--velocity-prior-sigma", "0"),
                        "sigma, threshold, and tolerance values must be positive",
                    )
                )

            required = (
                "--obs",
                "missing.obs",
                "--nav",
                "missing.nav",
                "--seed-pos",
                "missing.pos",
            )
            for extra_options, message in cases:
                with self.subTest(
                    binary=variant.binary_name,
                    option=extra_options[0],
                ):
                    self.assert_usage_error(
                        variant,
                        (*required, *extra_options),
                        message,
                    )

    def test_supported_tuning_options_reach_runtime(self) -> None:
        with tempfile.TemporaryDirectory(prefix="gnss_observable_cli_") as temp_dir:
            missing_observation = Path(temp_dir) / "missing.obs"
            for variant in VARIANTS:
                tuning_options = [(option, "0") for option in INTEGER_LIMIT_OPTIONS]
                tuning_options.extend(COMMON_TUNING_OPTIONS)
                if variant.has_tdcp:
                    tuning_options.append(TDCP_TUNING_OPTION)
                if variant.has_velocity:
                    tuning_options.extend(VELOCITY_TUNING_OPTIONS)
                flattened = tuple(
                    value
                    for option_and_value in tuning_options
                    for value in option_and_value
                )

                with self.subTest(binary=variant.binary_name):
                    self.assert_missing_observation_error(
                        variant,
                        missing_observation,
                        *flattened,
                        "--quiet",
                    )

    def test_mode_specific_options_remain_restricted(self) -> None:
        all_mode_options = (TDCP_TUNING_OPTION, *VELOCITY_TUNING_OPTIONS)
        required = (
            "--obs",
            "missing.obs",
            "--nav",
            "missing.nav",
            "--seed-pos",
            "missing.pos",
        )
        for variant in VARIANTS:
            supported = set()
            if variant.has_tdcp:
                supported.add(TDCP_TUNING_OPTION[0])
            if variant.has_velocity:
                supported.update(option for option, _ in VELOCITY_TUNING_OPTIONS)

            for option, value in all_mode_options:
                if option in supported:
                    continue
                with self.subTest(binary=variant.binary_name, option=option):
                    self.assert_usage_error(
                        variant,
                        (*required, option, value),
                        f"unknown argument: {option}",
                    )

    def test_debug_success_outputs_and_factor_numerics_are_stable(self) -> None:
        with tempfile.TemporaryDirectory(prefix="gnss_observable_success_") as temp_dir:
            temp_path = Path(temp_dir)
            observation_path, navigation_path, seed_path = self.write_success_fixture(
                temp_path
            )

            for variant in VARIANTS:
                output_path = temp_path / f"{variant.binary_name}.csv"
                factor_path = temp_path / f"{variant.binary_name}.factors.csv"
                graph_path = temp_path / f"{variant.binary_name}.graph.csv"
                summary_path = temp_path / f"{variant.binary_name}.json"

                with self.subTest(binary=variant.binary_name):
                    result = self.run_cli(
                        variant,
                        "--obs",
                        str(observation_path),
                        "--nav",
                        str(navigation_path),
                        "--seed-pos",
                        str(seed_path),
                        "--out-csv",
                        str(output_path),
                        "--factor-debug-csv",
                        str(factor_path),
                        "--graph-csv",
                        str(graph_path),
                        "--summary-json",
                        str(summary_path),
                        "--debug-problem-only",
                        "--quiet",
                    )

                    self.assertEqual(result.returncode, 0, msg=result.stderr)
                    self.assertEqual(result.stdout, "")
                    self.assertEqual(result.stderr, "")

                    summary = json.loads(summary_path.read_text(encoding="utf-8"))
                    expected_summary = self.expected_summary(variant)
                    self.assertEqual(set(summary), set(expected_summary))
                    for key, expected_value in expected_summary.items():
                        if key in {
                            "initial_cost",
                            "final_cost",
                            "residual_rms_mps",
                        }:
                            self.assert_contract_float(summary[key], float(expected_value))
                        else:
                            self.assertEqual(summary[key], expected_value, msg=key)

                    with output_path.open(newline="", encoding="utf-8") as stream:
                        output_reader = csv.DictReader(stream)
                        output_rows = list(output_reader)
                    expected_output_columns = (
                        VELOCITY_OUTPUT_COLUMNS
                        if variant.has_velocity
                        else POSITION_OUTPUT_COLUMNS
                    )
                    self.assertEqual(output_reader.fieldnames, expected_output_columns)
                    self.assertEqual(len(output_rows), 2)
                    for index, output_row in enumerate(output_rows):
                        self.assertEqual(output_row["epoch"], str(index + 1))
                        self.assertEqual(output_row["gps_week"], "2200")
                        self.assert_contract_float(
                            output_row["gps_tow"], 345_600.0 + index
                        )
                        self.assert_contract_float(output_row["spp_y_m"], RECEIVER_Y_M)
                        self.assert_contract_float(output_row["fgo_y_m"], RECEIVER_Y_M)
                        self.assertEqual(output_row["clock_jump"], "0")
                        zero_columns = set(expected_output_columns) - {
                            "epoch",
                            "gps_week",
                            "gps_tow",
                            "spp_y_m",
                            "fgo_y_m",
                            "clock_jump",
                        }
                        for column in zero_columns:
                            self.assert_contract_float(output_row[column], 0.0)

                    with factor_path.open(newline="", encoding="utf-8") as stream:
                        factor_reader = csv.DictReader(stream)
                        factor_rows = list(factor_reader)
                    self.assertEqual(
                        factor_reader.fieldnames,
                        self.expected_factor_columns(variant),
                    )
                    expected_factor_types = ["P", "P"]
                    expected_factor_types.extend(
                        ["D"] * (2 if variant.has_velocity else 1)
                    )
                    if variant.has_tdcp:
                        expected_factor_types.append("C")
                    self.assertEqual(
                        [row["factor_type"] for row in factor_rows],
                        expected_factor_types,
                    )

                    first_pseudorange = factor_rows[0]
                    self.assertEqual(first_pseudorange["epoch_index"], "0")
                    self.assertEqual(first_pseudorange["satellite"], "G03")
                    self.assertEqual(first_pseudorange["signal"], "GPS_L1CA")
                    self.assert_contract_float(first_pseudorange["snr_dbhz"], 45.0)
                    self.assert_contract_float(
                        first_pseudorange["elevation_deg"], 51.61724761403688
                    )
                    self.assert_contract_float(
                        first_pseudorange["res_pc_m"], 1_038_974.2389265299
                    )
                    self.assert_contract_float(
                        first_pseudorange["final_residual_m"],
                        -1_038_974.2389265299,
                    )

                    first_doppler = next(
                        row for row in factor_rows if row["factor_type"] == "D"
                    )
                    numerics = EXPECTED_NUMERICS[variant.binary_name]
                    self.assert_contract_float(
                        first_doppler["res_d_mps"],
                        numerics["first_doppler_residual_mps"],
                    )
                    self.assert_contract_float(
                        first_doppler["measured_range_rate_mps"],
                        numerics["first_measured_range_rate_mps"],
                    )
                    self.assert_contract_float(
                        first_doppler["final_residual_m"],
                        -numerics["first_doppler_residual_mps"],
                    )

                    if variant.has_tdcp:
                        tdcp = next(
                            row for row in factor_rows if row["factor_type"] == "C"
                        )
                        self.assert_contract_float(
                            tdcp["res_tdcp_m"], -291.25052121281624
                        )
                        self.assert_contract_float(
                            tdcp["wavelength_m"], 0.190293672798365
                        )
                        self.assert_contract_float(
                            tdcp["final_residual_m"], 291.25052121281624
                        )

                    with graph_path.open(newline="", encoding="utf-8") as stream:
                        graph_reader = csv.DictReader(stream)
                        graph_rows = list(graph_reader)
                    expected_graph_columns = [
                        "n",
                        "nsat",
                        "graph_factors",
                        "graph_values",
                        "initial_cost",
                        "final_cost",
                        "optimizer_error",
                        "iterations",
                        "valid_position_epochs",
                        (
                            "valid_velocity_epochs"
                            if variant.has_velocity
                            else "valid_clock_epochs"
                        ),
                    ]
                    self.assertEqual(graph_reader.fieldnames, expected_graph_columns)
                    self.assertEqual(len(graph_rows), 1)
                    graph_row = graph_rows[0]
                    self.assertEqual(graph_row["n"], "2")
                    self.assertEqual(graph_row["nsat"], "1")
                    self.assertEqual(
                        graph_row["graph_factors"],
                        str(expected_summary["graph_factors"]),
                    )
                    self.assertEqual(
                        graph_row["graph_values"],
                        str(expected_summary["graph_values"]),
                    )
                    self.assertEqual(graph_row["iterations"], "0")
                    for cost_column in (
                        "initial_cost",
                        "final_cost",
                        "optimizer_error",
                    ):
                        self.assert_contract_float(
                            graph_row[cost_column],
                            numerics["initial_cost"],
                        )

    def test_success_seed_formats_and_interpolation_are_stable(self) -> None:
        cases = (
            (
                "rtklib",
                "2022/03/10 00:00:00.000 0.0 90.0 0.0\n"
                "2022/03/10 00:00:01.000 0.0 90.0 0.0\n",
                (RECEIVER_Y_M, RECEIVER_Y_M),
                0,
            ),
            (
                "interpolated",
                "2200 345599 0 6378134 0\n"
                "2200 345602 0 6378143 0\n",
                (RECEIVER_Y_M, RECEIVER_Y_M + 3.0),
                2,
            ),
        )
        with tempfile.TemporaryDirectory(prefix="gnss_observable_seed_") as temp_dir:
            temp_path = Path(temp_dir)
            observation_path, navigation_path, seed_path = self.write_success_fixture(
                temp_path
            )

            for case_name, seed_text, expected_y_m, interpolated_epochs in cases:
                seed_path.write_text(seed_text, encoding="ascii")
                for variant in VARIANTS:
                    output_path = (
                        temp_path / f"{case_name}-{variant.binary_name}.csv"
                    )
                    summary_path = (
                        temp_path / f"{case_name}-{variant.binary_name}.json"
                    )
                    with self.subTest(
                        case=case_name,
                        binary=variant.binary_name,
                    ):
                        result = self.run_cli(
                            variant,
                            "--obs",
                            str(observation_path),
                            "--nav",
                            str(navigation_path),
                            "--seed-pos",
                            str(seed_path),
                            "--out-csv",
                            str(output_path),
                            "--summary-json",
                            str(summary_path),
                            "--debug-problem-only",
                            "--quiet",
                        )

                        self.assertEqual(result.returncode, 0, msg=result.stderr)
                        self.assertEqual(result.stdout, "")
                        self.assertEqual(result.stderr, "")

                        summary = json.loads(
                            summary_path.read_text(encoding="utf-8")
                        )
                        self.assertEqual(summary["optimized_epochs"], 2)
                        self.assertEqual(summary["seed_matched_epochs"], 2)
                        self.assertEqual(
                            summary["seed_interpolated_epochs"],
                            interpolated_epochs,
                        )

                        with output_path.open(
                            newline="", encoding="utf-8"
                        ) as stream:
                            output_rows = list(csv.DictReader(stream))
                        self.assertEqual(len(output_rows), 2)
                        for row, expected in zip(output_rows, expected_y_m):
                            self.assert_contract_float(row["spp_y_m"], expected)
                            self.assert_contract_float(row["fgo_y_m"], expected)

    def test_runtime_input_failure_does_not_create_outputs(self) -> None:
        with tempfile.TemporaryDirectory(prefix="gnss_observable_cli_") as temp_dir:
            temp_path = Path(temp_dir)
            missing_observation = temp_path / "missing.obs"
            for variant in VARIANTS:
                output_paths = (
                    temp_path / f"{variant.binary_name}.csv",
                    temp_path / f"{variant.binary_name}.factors.csv",
                    temp_path / f"{variant.binary_name}.graph.csv",
                    temp_path / f"{variant.binary_name}.json",
                )
                with self.subTest(binary=variant.binary_name):
                    self.assert_missing_observation_error(
                        variant,
                        missing_observation,
                        "--out-csv",
                        str(output_paths[0]),
                        "--factor-debug-csv",
                        str(output_paths[1]),
                        "--graph-csv",
                        str(output_paths[2]),
                        "--summary-json",
                        str(output_paths[3]),
                        "--debug-problem-only",
                        "--quiet",
                    )
                    for output_path in output_paths:
                        self.assertFalse(output_path.exists(), msg=str(output_path))


if __name__ == "__main__":
    unittest.main()
