#!/usr/bin/env python3
"""Normalize disposable CLASLIB component dumps for native CLAS ZD diffing."""

from __future__ import annotations

import argparse
import csv
from dataclasses import dataclass
import math
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
    "stec_tecu",
    "iono_scaled_m",
    "iono_scale",
    "code_bias_m",
    "phase_bias_m",
    "receiver_antenna_m",
    "relativity_m",
    "atmos_ref_week",
    "atmos_ref_tow",
    "clock_ref_week",
    "clock_ref_tow",
    "atmos_clock_gap_s",
    "atmos_network_id",
    "atmos_grid_no",
    "atmos_grid_distance_m",
    "atmos_grid_count",
    "atmos_grid1_no",
    "atmos_grid1_weight",
    "atmos_grid2_no",
    "atmos_grid2_weight",
    "atmos_grid3_no",
    "atmos_grid3_weight",
    "atmos_grid4_no",
    "atmos_grid4_weight",
    "windup_m",
    "phase_compensation_m",
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

CLASLIB_OSR_REQUIRED_COLUMNS = {
    "msg",
    "tow",
    "sys",
    "prn",
    "pbias1",
    "pbias2",
    "pbias5",
    "cbias1",
    "cbias2",
    "cbias5",
    "CPC1",
    "CPC2",
    "CPC5",
    "PRC1",
    "PRC2",
    "PRC5",
}

CLASLIB_OSR_GPS_SLOTS = (
    (0, "1", 1),   # GPS L1C
    (1, "2", 20),  # GPS L2W, the current CLAS A4b target
    (2, "5", 26),  # GPS L5X
)

GPS_IONO_SCALE_BY_SUFFIX = {
    "1": 1.0,
    "2": (1575.42 / 1227.60) ** 2,
    "5": (1575.42 / 1176.45) ** 2,
}
GPS_L1_TECU_TO_METERS = 40.3e16 / ((1575.42 * 1e6) ** 2)
WGS84_A = 6378137.0
DEGREES_TO_RADIANS = math.pi / 180.0
CLAS_MAX_GRID_DISTANCE_M = 120000.0
CLAS_NEAREST_GRID_THRESHOLD_M = 1000.0
CLAS_MULTI_GRID_NETWORK_MAX = 12


class ExportError(ValueError):
    """Raised for unsupported CLASLIB dump content."""


@dataclass(frozen=True)
class ClasGridPoint:
    network_id: int
    grid_no: int
    latitude_deg: float
    longitude_deg: float
    height_m: float


@dataclass(frozen=True)
class GridCandidate:
    point: ClasGridPoint
    distance_m: float


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


def is_claslib_osr_header(fieldnames: Iterable[str]) -> bool:
    return CLASLIB_OSR_REQUIRED_COLUMNS.issubset(set(fieldnames))


def is_invalid_osr_value(value: str) -> bool:
    if value == "":
        return True
    try:
        parsed = float(value)
    except ValueError:
        return True
    return abs(parsed + 10000.0) < 0.0005


def parse_osr_float(value: str) -> Optional[float]:
    if is_invalid_osr_value(value):
        return None
    try:
        return float(value)
    except ValueError:
        return None


def format_component(value: Optional[float]) -> str:
    if value is None:
        return ""
    return f"{value:.15g}"


def load_clas_grid_def(path: Path) -> list[ClasGridPoint]:
    if not path.is_file():
        raise ExportError(f"CLAS grid definition is missing: {path}")
    points: list[ClasGridPoint] = []
    with path.open(encoding="utf-8") as handle:
        for line_number, line in enumerate(handle, start=1):
            parts = line.split()
            if len(parts) < 5:
                continue
            try:
                points.append(
                    ClasGridPoint(
                        network_id=int(parts[0]),
                        grid_no=int(parts[1]),
                        latitude_deg=float(parts[2]),
                        longitude_deg=float(parts[3]),
                        height_m=float(parts[4]),
                    )
                )
            except ValueError:
                if line_number == 1:
                    continue
                raise ExportError(f"{path}: invalid CLAS grid row {line_number}: {line.strip()!r}")
    if not points:
        raise ExportError(f"{path}: no CLAS grid points found")
    return points


def clas_grid_metric_distance_m(lat_deg: float, lon_deg: float, point: ClasGridPoint) -> float:
    receiver_lat_rad = lat_deg * DEGREES_TO_RADIANS
    grid_lat_rad = point.latitude_deg * DEGREES_TO_RADIANS
    grid_lon_rad = point.longitude_deg * DEGREES_TO_RADIANS
    dlat_m = WGS84_A * (receiver_lat_rad - grid_lat_rad)
    dlon_m = WGS84_A * (lon_deg * DEGREES_TO_RADIANS - grid_lon_rad) * math.cos(receiver_lat_rad)
    return max(math.hypot(dlat_m, dlon_m), 1.0)


def non_collinear_2d(ax: float, ay: float, bx: float, by: float) -> bool:
    norm_product = math.hypot(ax, ay) * math.hypot(bx, by)
    if norm_product <= 0.0:
        return False
    return abs((ax * bx + ay * by) / norm_product) < 1.0


def select_claslib_surrounding_grids(candidates: list[GridCandidate]) -> list[GridCandidate]:
    if not candidates:
        return []
    n = len(candidates)
    first_index = (
        1
        if n > 2
        and candidates[1].point.network_id == candidates[2].point.network_id
        and candidates[0].point.network_id != candidates[1].point.network_id
        else 0
    )
    selected = [candidates[first_index]]
    grid2: GridCandidate | None = None
    grid3: GridCandidate | None = None
    dd12_lat = dd12_lon = dd13_lat = dd13_lon = 0.0

    for index, candidate in enumerate(candidates):
        if index == first_index or candidate.point.network_id != selected[0].point.network_id:
            continue
        grid2 = candidate
        selected.append(candidate)
        dd12_lat = candidate.point.latitude_deg - selected[0].point.latitude_deg
        dd12_lon = candidate.point.longitude_deg - selected[0].point.longitude_deg
        break
    if grid2 is None:
        return selected[:1]

    for index, candidate in enumerate(candidates):
        if (
            index == first_index
            or candidate is grid2
            or candidate.point.network_id != selected[0].point.network_id
        ):
            continue
        dlat = candidate.point.latitude_deg - selected[0].point.latitude_deg
        dlon = candidate.point.longitude_deg - selected[0].point.longitude_deg
        if non_collinear_2d(dd12_lat, dd12_lon, dlat, dlon):
            grid3 = candidate
            selected.append(candidate)
            dd13_lat = dlat
            dd13_lon = dlon
            break
    if grid3 is None:
        return selected[:1]

    for index, candidate in enumerate(candidates):
        if (
            index == first_index
            or candidate is grid2
            or candidate is grid3
            or candidate.point.network_id != selected[0].point.network_id
        ):
            continue
        dlat = candidate.point.latitude_deg - selected[0].point.latitude_deg
        dlon = candidate.point.longitude_deg - selected[0].point.longitude_deg
        rectangular = (
            abs(grid2.point.longitude_deg - candidate.point.longitude_deg) < 0.04
            and abs(grid3.point.latitude_deg - candidate.point.latitude_deg) < 0.04
        ) or (
            abs(grid2.point.latitude_deg - candidate.point.latitude_deg) < 0.04
            and abs(grid3.point.longitude_deg - candidate.point.longitude_deg) < 0.04
        )
        if (
            non_collinear_2d(dd12_lat, dd12_lon, dlat, dlon)
            and non_collinear_2d(dd13_lat, dd13_lon, dlat, dlon)
            and rectangular
        ):
            selected.append(candidate)
            return selected[:4]
    return selected[:3]


def solve_linear_4x4(matrix: list[list[float]]) -> Optional[list[float]]:
    for col in range(4):
        pivot = max(range(col, 4), key=lambda row: abs(matrix[row][col]))
        if abs(matrix[pivot][col]) <= 1e-14:
            return None
        if pivot != col:
            matrix[col], matrix[pivot] = matrix[pivot], matrix[col]
        scale = matrix[col][col]
        for c in range(col, 5):
            matrix[col][c] /= scale
        for row in range(4):
            if row == col:
                continue
            factor = matrix[row][col]
            for c in range(col, 5):
                matrix[row][c] -= factor * matrix[col][c]
    return [matrix[row][4] for row in range(4)]


def four_grid_weights(selected: list[GridCandidate], user_dlat_deg: float, user_dlon_deg: float) -> Optional[list[float]]:
    matrix = [[0.0 for _ in range(5)] for _ in range(4)]
    for grid, candidate in enumerate(selected[:4]):
        dlat = candidate.point.latitude_deg - selected[0].point.latitude_deg
        dlon = candidate.point.longitude_deg - selected[0].point.longitude_deg
        matrix[0][grid] = dlat
        matrix[1][grid] = dlon
        matrix[2][grid] = dlat * dlon
        matrix[3][grid] = 1.0
    matrix[0][4] = user_dlat_deg
    matrix[1][4] = user_dlon_deg
    matrix[2][4] = user_dlat_deg * user_dlon_deg
    matrix[3][4] = 1.0
    return solve_linear_4x4(matrix)


def three_grid_weights(selected: list[GridCandidate], user_dlat_deg: float, user_dlon_deg: float) -> Optional[list[float]]:
    a00 = selected[1].point.latitude_deg - selected[0].point.latitude_deg
    a01 = selected[2].point.latitude_deg - selected[0].point.latitude_deg
    a10 = selected[1].point.longitude_deg - selected[0].point.longitude_deg
    a11 = selected[2].point.longitude_deg - selected[0].point.longitude_deg
    det = a00 * a11 - a01 * a10
    if abs(det) <= 1e-14:
        return None
    w1 = (a11 * user_dlat_deg - a01 * user_dlon_deg) / det
    w2 = (-a10 * user_dlat_deg + a00 * user_dlon_deg) / det
    return [1.0 - w1 - w2, w1, w2, 0.0]


def inverse_distance_weights(selected: list[GridCandidate]) -> list[float]:
    inverse_sum = sum(1.0 / candidate.distance_m for candidate in selected)
    weights = [(1.0 / candidate.distance_m) / inverse_sum for candidate in selected]
    return weights + [0.0] * (4 - len(weights))


def claslib_grid_provenance(
    grid_points: Optional[list[ClasGridPoint]],
    lat_text: str,
    lon_text: str,
) -> dict[str, str]:
    if grid_points is None or lat_text == "" or lon_text == "":
        return {}
    try:
        lat_deg = float(lat_text)
        lon_deg = float(lon_text)
    except ValueError:
        return {}
    candidates = [
        GridCandidate(point, clas_grid_metric_distance_m(lat_deg, lon_deg, point))
        for point in grid_points
        if abs(point.height_m) <= 0.0001
    ]
    candidates = [
        candidate
        for candidate in candidates
        if candidate.distance_m <= CLAS_MAX_GRID_DISTANCE_M
    ]
    candidates.sort(key=lambda candidate: candidate.distance_m)
    if not candidates:
        return {}

    selected = [candidates[0]]
    weights = [1.0, 0.0, 0.0, 0.0]
    if (
        len(candidates) >= 3
        and candidates[0].point.network_id <= CLAS_MULTI_GRID_NETWORK_MAX
        and candidates[0].distance_m > CLAS_NEAREST_GRID_THRESHOLD_M
    ):
        selected = select_claslib_surrounding_grids(candidates)
        if len(selected) == 4:
            user_dlat = lat_deg - selected[0].point.latitude_deg
            user_dlon = lon_deg - selected[0].point.longitude_deg
            computed = four_grid_weights(selected, user_dlat, user_dlon)
            if computed is None:
                selected = [candidates[0]]
            else:
                weights = computed
        elif len(selected) == 3:
            user_dlat = lat_deg - selected[0].point.latitude_deg
            user_dlon = lon_deg - selected[0].point.longitude_deg
            computed = three_grid_weights(selected, user_dlat, user_dlon)
            if computed is None:
                selected = [candidates[0]]
            else:
                weights = computed
        elif len(selected) > 1:
            weights = inverse_distance_weights(selected)

    if len(selected) == 1:
        weights = [1.0, 0.0, 0.0, 0.0]
    fields = {
        "atmos_network_id": str(selected[0].point.network_id),
        "atmos_grid_no": str(selected[0].point.grid_no),
        "atmos_grid_distance_m": format_component(selected[0].distance_m),
        "atmos_grid_count": str(len(selected)),
    }
    for index in range(4):
        if index < len(selected):
            fields[f"atmos_grid{index + 1}_no"] = str(selected[index].point.grid_no)
            fields[f"atmos_grid{index + 1}_weight"] = format_component(weights[index])
        else:
            fields[f"atmos_grid{index + 1}_no"] = ""
            fields[f"atmos_grid{index + 1}_weight"] = ""
    return fields


def prc_iono_scaled_from_osrres(row: dict[str, str], suffix: str) -> Optional[float]:
    prc = parse_osr_float(first_present(row, (f"PRC{suffix}",)))
    trop = parse_osr_float(first_present(row, ("trop", "trop_m")))
    cbias = parse_osr_float(first_present(row, (f"cbias{suffix}",)))
    antr = parse_osr_float(first_present(row, (f"antr{suffix}",)))
    relativity = parse_osr_float(
        first_present(row, ("relatv", "relativity_m", "relativity_correction_m"))
    )
    if prc is None or trop is None or cbias is None or antr is None or relativity is None:
        return None
    return prc - (trop + antr + relativity + cbias)


def prc_iono_l1_from_osrres(row: dict[str, str], suffix: str) -> str:
    scaled = prc_iono_scaled_from_osrres(row, suffix)
    scale = GPS_IONO_SCALE_BY_SUFFIX.get(suffix, 0.0)
    if scaled is not None and scale > 0.0:
        return format_component(scaled / scale)
    return first_present(row, ("iono", "iono_l1_m", "l1_iono_m"))


def stec_tecu_from_iono_l1_m(value: str) -> str:
    iono_l1_m = parse_osr_float(value)
    if iono_l1_m is None or GPS_L1_TECU_TO_METERS <= 0.0:
        return ""
    return format_component(iono_l1_m / GPS_L1_TECU_TO_METERS)


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
    grid_points: Optional[list[ClasGridPoint]] = None,
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
            "stec_tecu": first_present(row, ("stec_tecu",)),
            "iono_scaled_m": first_present(row, ("iono_scaled_m",)),
            "iono_scale": first_present(row, ("iono_scale",)),
            "code_bias_m": first_present(row, ("code_bias_m", "code_bias", "cbias_m")),
            "phase_bias_m": first_present(row, ("phase_bias_m", "phase_bias", "pbias_m", "comp_m")),
            "receiver_antenna_m": first_present(row, ("receiver_antenna_m", "receiver_ant_m", "receiver_ant")),
            "relativity_m": first_present(row, ("relativity_m", "relativity_correction_m")),
            "atmos_ref_week": first_present(row, ("atmos_ref_week", "atmos_reference_week")),
            "atmos_ref_tow": first_present(row, ("atmos_ref_tow", "atmos_reference_tow")),
            "clock_ref_week": first_present(row, ("clock_ref_week", "clock_reference_week")),
            "clock_ref_tow": first_present(row, ("clock_ref_tow", "clock_reference_tow")),
            "atmos_clock_gap_s": first_present(row, ("atmos_clock_gap_s", "atmos_clock_dt_s")),
            "atmos_network_id": first_present(row, ("atmos_network_id",)),
            "atmos_grid_no": first_present(row, ("atmos_grid_no",)),
            "atmos_grid_distance_m": first_present(row, ("atmos_grid_distance_m",)),
            "atmos_grid_count": first_present(row, ("atmos_grid_count",)),
            "atmos_grid1_no": first_present(row, ("atmos_grid1_no",)),
            "atmos_grid1_weight": first_present(row, ("atmos_grid1_weight",)),
            "atmos_grid2_no": first_present(row, ("atmos_grid2_no",)),
            "atmos_grid2_weight": first_present(row, ("atmos_grid2_weight",)),
            "atmos_grid3_no": first_present(row, ("atmos_grid3_no",)),
            "atmos_grid3_weight": first_present(row, ("atmos_grid3_weight",)),
            "atmos_grid4_no": first_present(row, ("atmos_grid4_no",)),
            "atmos_grid4_weight": first_present(row, ("atmos_grid4_weight",)),
            "windup_m": first_present(row, ("windup_m",)),
            "phase_compensation_m": first_present(row, ("phase_compensation_m", "comp_m")),
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
    if output["atmos_network_id"] == "":
        output.update(claslib_grid_provenance(grid_points, first_present(row, ("lat",)), first_present(row, ("lon",))))
    return output


def osr_week(row: dict[str, str], *, gps_week: Optional[int], input_path: Path, row_number: int) -> str:
    week = first_present(row, ("week",))
    if week:
        return week
    if gps_week is None:
        raise ExportError(
            f"{input_path}: CLASLIB OSR row {row_number} has no GPS week; pass --gps-week"
        )
    return str(gps_week)


def normalize_osrres_rows(
    row: dict[str, str],
    *,
    row_number: int,
    stage_label: Optional[str],
    gps_week: Optional[int],
    input_path: Path,
    grid_points: Optional[list[ClasGridPoint]] = None,
) -> list[dict[str, str]]:
    sys_id = parse_int(first_present(row, ("sys", "system")), field=f"row {row_number} sys")
    prn = parse_int(first_present(row, ("prn",)), field=f"row {row_number} prn")
    if sys_id != 1:
        return []

    stage = row.get("stage", "") or (stage_label or "")
    week = osr_week(row, gps_week=gps_week, input_path=input_path, row_number=row_number)
    sat = sat_label(sys_id, prn)
    output_rows: list[dict[str, str]] = []
    grid_provenance = claslib_grid_provenance(
        grid_points,
        first_present(row, ("lat", "receiver_lat_deg")),
        first_present(row, ("lon", "receiver_lon_deg")),
    )

    for freq_index, suffix, rtklib_code in CLASLIB_OSR_GPS_SLOTS:
        pseudorange_code, carrier_code = rinex_pair(rtklib_code)
        effective_iono_l1_m = prc_iono_l1_from_osrres(row, suffix)
        effective_iono_scaled_m = format_component(prc_iono_scaled_from_osrres(row, suffix))
        common = {
            "stage": stage,
            "week": week,
            "tow": first_present(row, ("tow",)),
            "sat": sat,
            "freq": str(freq_index),
            "pseudorange_rinex_code": pseudorange_code,
            "carrier_rinex_code": carrier_code,
            "pseudorange_rtklib_code": str(rtklib_code),
            "carrier_rtklib_code": str(rtklib_code),
            "source_sys": str(sys_id),
            "source_prn": str(prn),
            "source_satno": first_present(row, ("sat", "satno")),
            "source_rtklib_code": str(rtklib_code),
            "trop_correction_m": first_present(row, ("trop", "trop_m")),
            "iono_l1_m": effective_iono_l1_m,
            "stec_tecu": stec_tecu_from_iono_l1_m(effective_iono_l1_m),
            "iono_scaled_m": effective_iono_scaled_m,
            "iono_scale": format_component(GPS_IONO_SCALE_BY_SUFFIX[suffix]),
            "receiver_antenna_m": first_present(row, (f"antr{suffix}",)),
            "relativity_m": first_present(row, ("relatv", "relativity_m", "relativity_correction_m")),
            "windup_m": first_present(row, (f"wup{suffix}",)),
            "phase_compensation_m": first_present(row, (f"compI{suffix}",)),
            "orbit_projection_m": first_present(row, ("orb", "orb_m")),
            "clock_correction_m": first_present(row, ("clk", "clk_m")),
            **grid_provenance,
        }

        prc = first_present(row, (f"PRC{suffix}",))
        cbias = first_present(row, (f"cbias{suffix}",))
        if not is_invalid_osr_value(prc) and not is_invalid_osr_value(cbias):
            code_row = {field: "" for field in FIELDNAMES}
            code_row.update(
                {
                    **common,
                    "record": "CODE",
                    "row_type": "code",
                    "signal": pseudorange_code,
                    "applied_pr_corr_m": prc,
                    "prc_m": prc,
                    "code_bias_m": cbias,
                }
            )
            output_rows.append(code_row)

        cpc = first_present(row, (f"CPC{suffix}",))
        pbias = first_present(row, (f"pbias{suffix}",))
        if not is_invalid_osr_value(cpc) and not is_invalid_osr_value(pbias):
            phase_row = {field: "" for field in FIELDNAMES}
            phase_row.update(
                {
                    **common,
                    "record": "PHASE",
                    "row_type": "phase",
                    "signal": carrier_code,
                    "carrier_correction_m": cpc,
                    "cpc_m": cpc,
                    "phase_bias_m": pbias,
                }
            )
            output_rows.append(phase_row)

    return output_rows


def export_csv(
    input_path: Path,
    output_path: Path,
    *,
    stage_label: Optional[str],
    gps_week: Optional[int] = None,
    clas_grid_def: Optional[Path] = None,
) -> int:
    grid_points = load_clas_grid_def(clas_grid_def) if clas_grid_def is not None else None
    with input_path.open(newline="", encoding="utf-8") as input_handle:
        reader = csv.DictReader(input_handle)
        if reader.fieldnames is None:
            raise ExportError(f"{input_path}: missing CSV header")
        if is_claslib_osr_header(reader.fieldnames):
            rows = []
            for index, row in enumerate(reader, start=2):
                rows.extend(
                    normalize_osrres_rows(
                        row,
                        row_number=index,
                        stage_label=stage_label,
                        gps_week=gps_week,
                        input_path=input_path,
                        grid_points=grid_points,
                    )
                )
        else:
            rows = [
                normalize_row(row, row_number=index, stage_label=stage_label, grid_points=grid_points)
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
    parser.add_argument(
        "--gps-week",
        type=int,
        default=None,
        help="GPS week to attach to CLASLIB .osr/OSRRES rows, which carry TOW only",
    )
    parser.add_argument(
        "--clas-grid-def",
        type=Path,
        default=None,
        help="Optional CLAS grid definition used to add CLASLIB-side grid provenance for .osr rows",
    )
    return parser.parse_args(argv)


def main(argv: Optional[list[str]] = None) -> int:
    args = parse_args(argv)
    try:
        rows_written = export_csv(
            args.input_csv,
            args.output,
            stage_label=args.stage_label,
            gps_week=args.gps_week,
            clas_grid_def=args.clas_grid_def,
        )
    except ExportError as exc:
        print(f"claslib_zd_component_export: {exc}", file=sys.stderr)
        return 2
    print(f"claslib_zd_component_export: wrote {rows_written} rows to {args.output}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
