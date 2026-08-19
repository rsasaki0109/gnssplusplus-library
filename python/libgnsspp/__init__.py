"""Python bindings for selected libgnss++ inspection and file-based solver APIs."""

from __future__ import annotations

from pathlib import Path
import shutil
import subprocess
import sys
import tempfile

from . import artifacts
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
    ssr_path: str = "",
    config_path: str = "",
    ar_method: str = "",
    ar_ratio_threshold: float = 0.0,
    use_dynamics_model: bool = False,
    clas_osr: bool = False,
    estimate_ionosphere: bool = False,
    estimate_troposphere: bool = True,
    extra_args: list[str] | None = None,
) -> Solution:
    """Run gnss_ppp on RINEX observation files and return the Solution.

    Parameters
    ----------
    obs_path:
        Path to the rover RINEX observation file.
    nav_path:
        Path to the broadcast navigation file.
    ssr_path:
        Path to the expanded SSR corrections CSV (CLAS/MADOCA).
    config_path:
        Optional TOML config file (``--config``). CLI arguments override it.
    kinematic_mode:
        Enable kinematic processing (``--kinematic``).
    use_dynamics_model:
        Enable the dynamics / IMU-free kinematic model (``--use-dynamics-model``).
    enable_ar:
        Enable ambiguity resolution (``--enable-ar``).
    ar_method:
        AR method string, e.g. ``"wlnl"`` or ``"lambda"`` (``--ar-method``).
    ar_ratio_threshold:
        AR ratio threshold; 0.0 leaves the default (``--ar-ratio-threshold``).
    clas_osr:
        Enable the CLAS OSR PPP-RTK filter path (``--clas-osr``).
    estimate_ionosphere:
        Estimate per-satellite ionosphere states (``--estimate-ionosphere``).
    estimate_troposphere:
        Estimate zenith troposphere (default True).
    sp3_path:
        Precise orbit SP3 file.
    clk_path:
        Precise clock CLK file.
    extra_args:
        Additional raw CLI arguments appended verbatim.
    """
    arguments = ["ppp", "--obs", obs_path, "--nav", nav_path, "--quiet"]
    if config_path:
        arguments.extend(["--config", config_path])
    if kinematic_mode:
        arguments.append("--kinematic")
    if use_dynamics_model:
        arguments.append("--use-dynamics-model")
    if enable_ar:
        arguments.append("--enable-ar")
    if ar_method:
        arguments.extend(["--ar-method", ar_method])
    if ar_ratio_threshold > 0.0:
        arguments.extend(["--ar-ratio-threshold", str(ar_ratio_threshold)])
    if clas_osr:
        arguments.extend(["--clas-osr", "--no-ionosphere-free"])
    if estimate_ionosphere:
        arguments.append("--estimate-ionosphere")
    if not estimate_troposphere:
        arguments.append("--no-estimate-troposphere")
    if ssr_path:
        arguments.extend(["--ssr", ssr_path])
    if sp3_path:
        arguments.extend(["--sp3", sp3_path])
    if clk_path:
        arguments.extend(["--clk", clk_path])
    if max_epochs > 0:
        arguments.extend(["--max-epochs", str(max_epochs)])
    if extra_args:
        arguments.extend(extra_args)
    return _run_gnss_and_load_solution(arguments)


def solve_clas_kinematic_file(
    obs_path: str,
    nav_path: str,
    ssr_path: str,
    max_epochs: int = 0,
    config_path: str = "",
    extra_args: list[str] | None = None,
) -> Solution:
    """Convenience wrapper for CLAS PPP-RTK kinematic positioning.

    Equivalent to::

        gnss_ppp --obs <obs> --nav <nav> --ssr <ssr> \\
                 --kinematic --use-dynamics-model --clas-osr \\
                 --no-ionosphere-free --estimate-ionosphere \\
                 --estimate-troposphere --enable-ar --ar-method wlnl \\
                 --ar-ratio-threshold 3.0

    Parameters
    ----------
    obs_path:
        Path to the rover RINEX observation file.
    nav_path:
        Path to the broadcast navigation file.
    ssr_path:
        Path to the expanded CLAS SSR corrections CSV.
    config_path:
        Optional TOML config file to override defaults.
    extra_args:
        Additional raw CLI arguments.
    """
    return solve_ppp_file(
        obs_path=obs_path,
        nav_path=nav_path,
        ssr_path=ssr_path,
        config_path=config_path,
        kinematic_mode=True,
        use_dynamics_model=True,
        enable_ar=True,
        ar_method="wlnl",
        ar_ratio_threshold=3.0,
        clas_osr=True,
        estimate_ionosphere=True,
        estimate_troposphere=True,
        max_epochs=max_epochs,
        extra_args=extra_args,
    )


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
    "artifacts",
    "ecef_to_geodetic_deg",
    "geodetic_deg_to_ecef",
    "load_solution",
    "preprocess_spp_file",
    "read_rinex_header",
    "read_rinex_observation_epochs",
    "solve_clas_kinematic_file",
    "solve_ppp_file",
    "solve_rtk_file",
    "solve_spp_file",
    "solution_status_name",
]
