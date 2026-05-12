#!/usr/bin/env python3
"""Convert VMF site-wise tidal APL coefficients to libgnss++ ATL files."""

from __future__ import annotations

import argparse
import math
import os
from pathlib import Path
import sys
from urllib.request import urlopen


DEFAULT_VMF_GNSS_TIDAL_URL = (
    "https://vmf.geo.tuwien.ac.at/APL_products/TIDAL/"
    "s1_s2_s3_cm_noib_gnss.dat"
)


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(prog=os.environ.get("GNSS_CLI_NAME"))
    parser.add_argument(
        "--source",
        default=DEFAULT_VMF_GNSS_TIDAL_URL,
        help="VMF GNSS tidal APL file path or URL. Defaults to the VMF "
             "site-wise GNSS tidal APL product.",
    )
    parser.add_argument(
        "--station",
        action="append",
        required=True,
        help="Four-character station code to extract. Repeat for multiple sites.",
    )
    parser.add_argument(
        "--output-dir",
        type=Path,
        default=Path("test_data") / "iers",
        help="Directory for generated .atl files.",
    )
    parser.add_argument(
        "--suffix",
        default="vmf",
        help="Output filename suffix: <station>_<suffix>.atl.",
    )
    return parser.parse_args()


def read_text(source: str) -> str:
    if source.startswith(("http://", "https://")):
        with urlopen(source, timeout=30) as response:
            return response.read().decode("ascii")
    return Path(source).read_text(encoding="ascii")


def parse_vmf_rows(text: str) -> dict[str, list[float]]:
    rows: dict[str, list[float]] = {}
    for line in text.splitlines():
        if not line.strip() or line.lstrip().startswith(("#", "$")):
            continue
        parts = line.split()
        if len(parts) < 19:
            continue
        station = parts[0].upper()
        try:
            rows[station] = [float(value) for value in parts[1:19]]
        except ValueError:
            continue
    return rows


def amplitude_phase_m(cos_mm: float, sin_mm: float) -> tuple[float, float]:
    """Return A, phi for A*cos(angle - phi) from C*cos(angle)+S*sin(angle)."""
    amplitude_m = math.hypot(cos_mm, sin_mm) / 1000.0
    phase_deg = math.degrees(math.atan2(sin_mm, cos_mm))
    return amplitude_m, phase_deg


def convert_row(station: str, values: list[float]) -> str:
    # VMF columns after station:
    # 0..5 radial S1/S2/S3 cos/sin, 6..11 east, 12..17 north, all in mm.
    indices = {
        "S1": (0, 1, 6, 7, 12, 13),
        "S2": (2, 3, 8, 9, 14, 15),
    }
    lines = [
        "$$ VMF site-wise GNSS tidal atmospheric pressure loading coefficients",
        "$$ Source: " + DEFAULT_VMF_GNSS_TIDAL_URL,
        "$$ Converted from VMF cos/sin mm coefficients to libgnss++",
        "$$ amplitude/phase meters. East/north are stored as west/south.",
        "$$ Format:",
        "$$   <constituent>  radial_amp_m  west_amp_m  south_amp_m  "
        "radial_phase_deg  west_phase_deg  south_phase_deg",
        station.upper(),
    ]
    for constituent, (r_c, r_s, e_c, e_s, n_c, n_s) in indices.items():
        radial_amp, radial_phase = amplitude_phase_m(values[r_c], values[r_s])
        west_amp, west_phase = amplitude_phase_m(-values[e_c], -values[e_s])
        south_amp, south_phase = amplitude_phase_m(-values[n_c], -values[n_s])
        lines.append(
            f"{constituent}  "
            f"{radial_amp:.8f}  {west_amp:.8f}  {south_amp:.8f}  "
            f"{radial_phase:.3f}  {west_phase:.3f}  {south_phase:.3f}"
        )
    lines.append("$$ END")
    return "\n".join(lines) + "\n"


def main() -> int:
    args = parse_args()
    rows = parse_vmf_rows(read_text(args.source))
    args.output_dir.mkdir(parents=True, exist_ok=True)
    missing: list[str] = []
    for raw_station in args.station:
        station = raw_station.upper()
        values = rows.get(station)
        if values is None:
            missing.append(station)
            continue
        output = args.output_dir / f"{station.lower()}_{args.suffix}.atl"
        output.write_text(convert_row(station, values), encoding="ascii")
        print(output)
    if missing:
        print("missing stations: " + ", ".join(missing), file=sys.stderr)
        return 1
    return 0


if __name__ == "__main__":
    sys.exit(main())
