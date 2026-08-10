"""Characterize help, parsing, and early failures for observable-native CLIs."""

from __future__ import annotations

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
