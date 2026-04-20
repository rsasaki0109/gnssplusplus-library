"""Python bindings for selected libgnss++ inspection and file-based solver APIs."""

from __future__ import annotations

from pathlib import Path
import shutil
import subprocess
import sys
import tempfile

from ._libgnsspp import (
    CorrectedMeasurement,
    GNSSTime,
    PositionSolution,
    Solution,
    SolutionStatistics,
    SolutionStatus,
    ecef_to_geodetic_deg,
    geodetic_deg_to_ecef,
    load_solution,
    preprocess_spp_file,
    read_rinex_header,
    read_rinex_observation_epochs,
    solution_status_name,
)


def _resolve_gnss_command() -> list[str]:
    package_path = Path(__file__).resolve()
    for parent in package_path.parents:
        source_dispatcher = parent / "apps" / "gnss.py"
        if source_dispatcher.exists():
            return [sys.executable, str(source_dispatcher)]

    installed_dispatcher = shutil.which("gnss")
    if installed_dispatcher is not None:
        return [installed_dispatcher]

    raise RuntimeError("failed to locate the gnss dispatcher")


def _run_gnss_and_load_solution(arguments: list[str]) -> Solution:
    command = _resolve_gnss_command()
    with tempfile.TemporaryDirectory(prefix="libgnsspp_py_") as temp_dir:
        output_path = Path(temp_dir) / "solution.pos"
        result = subprocess.run(
            command + arguments + ["--out", str(output_path)],
            capture_output=True,
            text=True,
        )
        if result.returncode != 0:
            message = result.stderr.strip() or result.stdout.strip() or "unknown error"
            raise RuntimeError(f"gnss command failed: {message}")
        return load_solution(str(output_path))


def solve_spp_file(obs_path: str, nav_path: str, max_epochs: int = 0) -> Solution:
    arguments = [
        "spp",
        "--obs",
        obs_path,
        "--nav",
        nav_path,
        "--quiet",
    ]
    if max_epochs > 0:
        arguments.extend(["--max-epochs", str(max_epochs)])
    return _run_gnss_and_load_solution(arguments)


def solve_ppp_file(
    obs_path: str,
    nav_path: str,
    max_epochs: int = 0,
    kinematic_mode: bool = False,
    enable_ar: bool = False,
    sp3_path: str = "",
    clk_path: str = "",
) -> Solution:
    arguments = [
        "ppp",
        "--obs",
        obs_path,
        "--nav",
        nav_path,
        "--quiet",
        "--kinematic" if kinematic_mode else "--static",
    ]
    if enable_ar:
        arguments.append("--enable-ar")
    if sp3_path:
        arguments.extend(["--sp3", sp3_path])
    if clk_path:
        arguments.extend(["--clk", clk_path])
    if max_epochs > 0:
        arguments.extend(["--max-epochs", str(max_epochs)])
    return _run_gnss_and_load_solution(arguments)


def solve_rtk_file(
    rover_obs_path: str,
    base_obs_path: str,
    nav_path: str,
    max_epochs: int = 0,
    iono_mode: str = "off",
) -> Solution:
    arguments = [
        "solve",
        "--rover",
        rover_obs_path,
        "--base",
        base_obs_path,
        "--nav",
        nav_path,
        "--no-kml",
        "--iono",
        iono_mode,
    ]
    if max_epochs > 0:
        arguments.extend(["--max-epochs", str(max_epochs)])
    return _run_gnss_and_load_solution(arguments)


__all__ = [
    "CorrectedMeasurement",
    "GNSSTime",
    "PositionSolution",
    "Solution",
    "SolutionStatistics",
    "SolutionStatus",
    "ecef_to_geodetic_deg",
    "geodetic_deg_to_ecef",
    "load_solution",
    "preprocess_spp_file",
    "read_rinex_header",
    "read_rinex_observation_epochs",
    "solve_ppp_file",
    "solve_rtk_file",
    "solve_spp_file",
    "solution_status_name",
]
