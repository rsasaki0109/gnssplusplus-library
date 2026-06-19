#!/usr/bin/env python3
"""Normalize disposable CLASLIB component dumps for native CLAS ZD diffing."""

from __future__ import annotations

import argparse
import csv
import sys
from pathlib import Path
from typing import Iterable, Optional


SYS_PREFIX_BY_ID = {
    1: "G",  # GPS
    2: "S",  # SBAS
    4: "R",  # GLONASS
    8: "E",  # Galileo
    16: "J",  # QZSS
    32: "C",  # BeiDou
    64: "I",  # NavIC/IRNSS
}

RTKLIB_CODE_TO_SUFFIX = {
    1: "1C",
    2: "1P",
    3: "1W",
    4: "1Y",
    5: "1M",
    6: "1N",
    7: "1S",
    8: "1L",
    9: "1E",
    10: "1A",
    11: "1B",
    12: "1X",
    13: "1Z",
    14: "2C",
    15: "2D",
    16: "2S",
    17: "2L",
    18: "2X",
    19: "2P",
    20: "2W",
    21: "2Y",
    22: "2M",
    23: "2N",
    24: "5I",
    25: "5Q",
    26: "5X",
    27: "7I",
    28: "7Q",
    29: "7X",
    30: "6A",
    31: "6B",
    32: "6C",
    33: "6X",
    34: "6Z",
    35: "6S",
    36: "6L",
    37: "8I",
    38: "8Q",
    39: "8X",
    40: "2I",
    41: "2Q",
    42: "6I",
    43: "6Q",
    44: "3I",
    45: "3Q",
    46: "3X",
    47: "1I",
    48: "1Q",
}

FIELDNAMES = [
    "record",
    "stage",
    "week",
    "tow",
    "sat",
    "row_type",
    "freq",
    "signal",
    "pseudorange_rinex_code",
    "carrier_rinex_code",
    "pseudorange_rtklib_code",
    "carrier_rtklib_code",
    "source_sys",
    "source_prn",
    "source_satno",
    "source_rtklib_code",
    "raw_p_m",
    "corrected_p_m",
    "raw_l_m",
    "corrected_l_m",
    "applied_pr_corr_m",
    "carrier_correction_m",
    "prc_m",
    "cpc_m",
    "trop_correction_m",
    "iono_l1_m",
    "iono_scaled_m",
    "code_bias_m",
    "phase_bias_m",
    "receiver_antenna_m",
    "relativity_m",
    "windup_m",
    "orbit_projection_m",
    "clock_correction_m",
    "residual_m",
    "geo_m",
    "sat_clk_m",
    "model_m",
    "az_rad",
    "el_rad",
    "rx_x_m",
    "rx_y_m",
    "rx_z_m",
]


class ExportError(ValueError):
    """Raised for unsupported CLASLIB dump content."""


def first_present(row: dict[str, str], names: Iterable[str], default: str = "") -> str:
    for name in names:
        value = row.get(name, "")
        if value != "":
            return value
    return default


def parse_int(value: str, *, field: str) -> int:
    try:
        return int(float(value))
    except ValueError as exc:
        raise ExportError(f"invalid {field}: {value!r}") from exc


def normalize_qzss_prn(prn: int) -> int:
    return prn - 192 if prn >= 193 else prn


def sat_label(sys_id: int, prn: int) -> str:
    prefix = SYS_PREFIX_BY_ID.get(sys_id)
    if prefix is None:
        raise ExportError(f"unsupported CLASLIB system id: {sys_id}")
    if prefix == "J":
        prn = normalize_qzss_prn(prn)
    return f"{prefix}{prn:02d}"


def rinex_pair(rtklib_code: int) -> tuple[str, str]:
    suffix = RTKLIB_CODE_TO_SUFFIX.get(rtklib_code)
    if suffix is None:
        raise ExportError(f"unsupported RTKLIB observation code: {rtklib_code}")
    return f"C{suffix}", f"L{suffix}"


def detect_row_type(row: dict[str, str]) -> str:
    value = first_present(row, ("row_type", "record", "type", "kind"), "").strip().lower()
    if value in {"code", "pseudorange", "pr"} or value.startswith("code"):
        return "code"
    if value in {"phase", "carrier", "carrier_phase"} or value.startswith("phase"):
        return "phase"
    raise ExportError(f"unsupported row type: {value!r}")


def normalized_record(row_type: str) -> str:
    return "CODE" if row_type == "code" else "PHASE"


def normalize_row(
    row: dict[str, str],
    *,
    row_number: int,
    stage_label: Optional[str],
) -> dict[str, str]:
    sys_id = parse_int(first_present(row, ("sys", "system")), field=f"row {row_number} sys")
    prn = parse_int(first_present(row, ("prn",)), field=f"row {row_number} prn")
    rtklib_code = parse_int(first_present(row, ("code", "rtklib_code")), field=f"row {row_number} code")
    row_type = detect_row_type(row)
    pseudorange_code, carrier_code = rinex_pair(rtklib_code)
    row_signal = pseudorange_code if row_type == "code" else carrier_code
    stage = row.get("stage", "") or (stage_label or "")

    output = {field: "" for field in FIELDNAMES}
    output.update(
        {
            "record": first_present(row, ("record",), normalized_record(row_type)).upper(),
            "stage": stage,
            "week": first_present(row, ("week",)),
            "tow": first_present(row, ("tow",)),
            "sat": sat_label(sys_id, prn),
            "row_type": row_type,
            "freq": first_present(row, ("freq", "f"), "0"),
            "signal": row_signal,
            "pseudorange_rinex_code": pseudorange_code,
            "carrier_rinex_code": carrier_code,
            "pseudorange_rtklib_code": str(rtklib_code),
            "carrier_rtklib_code": str(rtklib_code),
            "source_sys": str(sys_id),
            "source_prn": str(prn),
            "source_satno": first_present(row, ("sat", "satno")),
            "source_rtklib_code": str(rtklib_code),
            "raw_p_m": first_present(row, ("raw_p_m", "raw_m")),
            "corrected_p_m": first_present(row, ("corrected_p_m", "corr_meas_m")),
            "raw_l_m": first_present(row, ("raw_l_m",)),
            "corrected_l_m": first_present(row, ("corrected_l_m",)),
            "applied_pr_corr_m": first_present(row, ("applied_pr_corr_m",)),
            "carrier_correction_m": first_present(row, ("carrier_correction_m",)),
            "prc_m": first_present(row, ("prc_m", "PRC", "PRC_m")),
            "cpc_m": first_present(row, ("cpc_m", "CPC", "CPC_m")),
            "trop_correction_m": first_present(row, ("trop_correction_m", "trop_m")),
            "iono_l1_m": first_present(row, ("iono_l1_m", "l1_iono_m")),
            "iono_scaled_m": first_present(row, ("iono_scaled_m",)),
            "code_bias_m": first_present(row, ("code_bias_m", "code_bias", "cbias_m")),
            "phase_bias_m": first_present(row, ("phase_bias_m", "phase_bias", "pbias_m", "comp_m")),
            "receiver_antenna_m": first_present(row, ("receiver_antenna_m", "receiver_ant_m", "receiver_ant")),
            "relativity_m": first_present(row, ("relativity_m", "relativity_correction_m")),
            "windup_m": first_present(row, ("windup_m",)),
            "orbit_projection_m": first_present(row, ("orbit_projection_m", "orb_m")),
            "clock_correction_m": first_present(row, ("clock_correction_m", "clk_m")),
            "residual_m": first_present(row, ("residual_m", "zd_residual_m")),
            "geo_m": first_present(row, ("geo_m",)),
            "sat_clk_m": first_present(row, ("sat_clk_m",)),
            "model_m": first_present(row, ("model_m",)),
            "az_rad": first_present(row, ("az_rad",)),
            "el_rad": first_present(row, ("el_rad",)),
            "rx_x_m": first_present(row, ("rx_x_m",)),
            "rx_y_m": first_present(row, ("rx_y_m",)),
            "rx_z_m": first_present(row, ("rx_z_m",)),
        }
    )
    return output


def export_csv(input_path: Path, output_path: Path, *, stage_label: Optional[str]) -> int:
    with input_path.open(newline="", encoding="utf-8") as input_handle:
        reader = csv.DictReader(input_handle)
        if reader.fieldnames is None:
            raise ExportError(f"{input_path}: missing CSV header")
        rows = [
            normalize_row(row, row_number=index, stage_label=stage_label)
            for index, row in enumerate(reader, start=2)
        ]

    output_path.parent.mkdir(parents=True, exist_ok=True)
    with output_path.open("w", newline="", encoding="utf-8") as output_handle:
        writer = csv.DictWriter(output_handle, fieldnames=FIELDNAMES)
        writer.writeheader()
        writer.writerows(rows)
    return len(rows)


def parse_args(argv: Optional[list[str]] = None) -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description=(
            "Normalize disposable CLASLIB ZD/code component dumps into the "
            "CSV schema consumed by clas_zd_component_diff.py."
        )
    )
    parser.add_argument("input_csv", type=Path, help="Raw CLASLIB component dump CSV")
    parser.add_argument("--output", type=Path, required=True, help="Normalized output CSV")
    parser.add_argument(
        "--stage-label",
        default=None,
        help="Stage label to use only for input rows that do not already have a stage column",
    )
    return parser.parse_args(argv)


def main(argv: Optional[list[str]] = None) -> int:
    args = parse_args(argv)
    try:
        rows_written = export_csv(args.input_csv, args.output, stage_label=args.stage_label)
    except ExportError as exc:
        print(f"claslib_zd_component_export: {exc}", file=sys.stderr)
        return 2
    print(f"claslib_zd_component_export: wrote {rows_written} rows to {args.output}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
