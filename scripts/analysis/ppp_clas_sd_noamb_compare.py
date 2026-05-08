#!/usr/bin/env python3
"""Compare libgnss++ CLAS SD no-ambiguity residuals with CLASLIB ddres rows."""

from __future__ import annotations

import argparse
import math
from dataclasses import dataclass
from pathlib import Path
from typing import Dict, Iterable, List, Optional, Sequence, Tuple


PairKey = Tuple[str, str, int]
TimePairKey = Tuple[int, str, str, int]
UdKey = Tuple[int, str, int]
MeasRowKey = Tuple[int, int]


@dataclass(frozen=True)
class ReceiverPosition:
    tow: float
    x_m: float
    y_m: float
    z_m: float
    clock_m: float = math.nan


@dataclass(frozen=True)
class LibUdRow:
    tow: float
    sat: str
    freq_group: int
    raw_phase_m: float
    carrier_phase_correction_m: float
    l_corr_m: float
    geometric_range_no_sagnac_m: float
    sagnac_m: float
    geometric_range_m: float
    satellite_clock_m: float
    trop_m: float
    iono_scale: float
    iono_state_m: float
    osr_trop_correction_m: float
    osr_relativity_m: float
    osr_receiver_antenna_m: float
    osr_iono_l1_m: float
    osr_code_bias_m: float
    osr_phase_bias_m: float
    osr_windup_m: float
    osr_prc_m: float
    osr_cpc_m: float
    osr_phase_compensation_m: float

    @property
    def key(self) -> UdKey:
        return (_tow_key(self.tow), self.sat, self.freq_group)

    @property
    def model_noamb_m(self) -> float:
        return (
            self.geometric_range_m
            - self.satellite_clock_m
            + self.trop_m
            + self.iono_scale * self.iono_state_m
        )

    @property
    def phase_iono_state_m(self) -> float:
        return self.iono_scale * self.iono_state_m

    @property
    def geo_clock_m(self) -> float:
        return self.geometric_range_m - self.satellite_clock_m


@dataclass(frozen=True)
class LibRow:
    tow: float
    ref: str
    sat: str
    freq_group: int
    sd_noamb_m: float
    sd_residual_m: float
    sd_ambiguity_term_m: float

    @property
    def pair_key(self) -> PairKey:
        return (self.ref, self.sat, self.freq_group)

    @property
    def time_key(self) -> TimePairKey:
        return (_tow_key(self.tow), self.ref, self.sat, self.freq_group)


@dataclass(frozen=True)
class ClaslibRow:
    tow: float
    ref: str
    sat: str
    freq_group: int
    y_m: float
    amb_m: float
    obs_m: float
    rho_m: float
    dts_m: float
    rel_m: float
    sagnac_m: float
    trop_model_m: float
    iono_grid_m: float
    iono_state_term_m: float
    trop_grid_m: float
    cpc_m: float
    phase_bias_m: float
    phase_comp_m: float
    windup_m: float
    pco_pcv_m: float
    components_are_sd: bool = True

    @property
    def noamb_m(self) -> float:
        return self.y_m + self.amb_m

    @property
    def time_key(self) -> TimePairKey:
        return (_tow_key(self.tow), self.ref, self.sat, self.freq_group)


@dataclass(frozen=True)
class LegacyMeasRow:
    tow: float
    row_index: int
    ref: str
    sat: str
    is_phase: bool
    freq_group: int

    @property
    def key(self) -> MeasRowKey:
        return (_tow_key(self.tow), self.row_index)


def _tow_key(tow: float) -> int:
    return int(round(tow * 1000.0))


def _parse_token_map(line: str) -> Dict[str, str]:
    out: Dict[str, str] = {}
    for token in line.strip().split():
        if "=" not in token:
            continue
        key, value = token.split("=", 1)
        out[key] = value
    return out


def _to_float(value: Optional[str]) -> float:
    if value is None:
        return math.nan
    try:
        return float(value)
    except ValueError:
        return math.nan


def _to_int(value: Optional[str], default: int = 0) -> int:
    if value is None:
        return default
    try:
        return int(float(value))
    except ValueError:
        return default


def _first_float(tokens: Dict[str, str], keys: Sequence[str]) -> float:
    for key in keys:
        if key in tokens:
            return _to_float(tokens[key])
    return math.nan


def _sum_present(tokens: Dict[str, str], keys: Sequence[str]) -> float:
    values = [_to_float(tokens.get(key)) for key in keys if key in tokens]
    if not values:
        return math.nan
    return sum(values)


def parse_libgnss_ddres(
    path: Path,
) -> Tuple[List[LibRow], Dict[UdKey, LibUdRow], Dict[int, ReceiverPosition]]:
    rows: List[LibRow] = []
    ud_rows: Dict[UdKey, LibUdRow] = {}
    receiver_positions: Dict[int, ReceiverPosition] = {}
    section = ""
    tow = math.nan
    receiver_x = math.nan
    receiver_y = math.nan
    receiver_z = math.nan
    receiver_clock = math.nan

    def record_receiver_position() -> None:
        if not all(
            math.isfinite(value)
            for value in (tow, receiver_x, receiver_y, receiver_z)
        ):
            return
        receiver_positions[_tow_key(tow)] = ReceiverPosition(
            tow=tow,
            x_m=receiver_x,
            y_m=receiver_y,
            z_m=receiver_z,
            clock_m=receiver_clock,
        )

    with path.open("r", encoding="utf-8") as handle:
        for raw_line in handle:
            line = raw_line.strip()
            if not line:
                continue
            if line.startswith("[") and line.endswith("]"):
                section = line[1:-1]
                continue
            if line.startswith("strict_clas_ddres_dump="):
                section = ""
                tow = math.nan
                receiver_x = math.nan
                receiver_y = math.nan
                receiver_z = math.nan
                receiver_clock = math.nan
                continue
            if not section and line.startswith("time_tow="):
                tow = _to_float(line.split("=", 1)[1])
                record_receiver_position()
                continue
            if not section and line.startswith("receiver_x_m="):
                receiver_x = _to_float(line.split("=", 1)[1])
                record_receiver_position()
                continue
            if not section and line.startswith("receiver_y_m="):
                receiver_y = _to_float(line.split("=", 1)[1])
                record_receiver_position()
                continue
            if not section and line.startswith("receiver_z_m="):
                receiver_z = _to_float(line.split("=", 1)[1])
                record_receiver_position()
                continue
            if not section and line.startswith("receiver_clock_m="):
                receiver_clock = _to_float(line.split("=", 1)[1])
                record_receiver_position()
                continue
            if section == "ud_phase" and line.startswith("i="):
                tokens = _parse_token_map(line)
                ud_row = LibUdRow(
                    tow=tow,
                    sat=tokens["sat"],
                    freq_group=int(tokens["freq_group"]),
                    raw_phase_m=_to_float(tokens.get("raw_phase_m")),
                    carrier_phase_correction_m=_to_float(
                        tokens.get("carrier_phase_correction_m")
                    ),
                    l_corr_m=_to_float(tokens.get("l_corr_m")),
                    geometric_range_no_sagnac_m=_to_float(
                        tokens.get("geometric_range_no_sagnac_m")
                    ),
                    sagnac_m=_to_float(tokens.get("sagnac_m")),
                    geometric_range_m=_to_float(tokens.get("geometric_range_m")),
                    satellite_clock_m=_to_float(tokens.get("satellite_clock_m")),
                    trop_m=_to_float(tokens.get("trop_m")),
                    iono_scale=_to_float(tokens.get("iono_scale")),
                    iono_state_m=_to_float(tokens.get("iono_state_m")),
                    osr_trop_correction_m=_to_float(
                        tokens.get("osr_trop_correction_m")
                    ),
                    osr_relativity_m=_to_float(tokens.get("osr_relativity_m")),
                    osr_receiver_antenna_m=_to_float(
                        tokens.get("osr_receiver_antenna_m")
                    ),
                    osr_iono_l1_m=_to_float(tokens.get("osr_iono_l1_m")),
                    osr_code_bias_m=_to_float(tokens.get("osr_code_bias_m")),
                    osr_phase_bias_m=_to_float(tokens.get("osr_phase_bias_m")),
                    osr_windup_m=_to_float(tokens.get("osr_windup_m")),
                    osr_prc_m=_to_float(tokens.get("osr_prc_m")),
                    osr_cpc_m=_to_float(tokens.get("osr_cpc_m")),
                    osr_phase_compensation_m=_to_float(
                        tokens.get("osr_phase_compensation_m")
                    ),
                )
                ud_rows[ud_row.key] = ud_row
                continue
            if section != "sd_phase" or not line.startswith("k="):
                continue
            tokens = _parse_token_map(line)
            rows.append(
                LibRow(
                    tow=tow,
                    ref=tokens["ref"],
                    sat=tokens["sat"],
                    freq_group=int(tokens["freq_group"]),
                    sd_noamb_m=_to_float(tokens.get("sd_noamb_m")),
                    sd_residual_m=_to_float(tokens.get("sd_residual_m")),
                    sd_ambiguity_term_m=_to_float(tokens.get("sd_ambiguity_term_m")),
                )
            )
    return rows, ud_rows, receiver_positions


def parse_claslib_model_comp(
    path: Path,
) -> Tuple[Dict[TimePairKey, ClaslibRow], Dict[int, ReceiverPosition]]:
    rows: Dict[TimePairKey, ClaslibRow] = {}
    receiver_positions: Dict[int, ReceiverPosition] = {}
    with path.open("r", encoding="utf-8") as handle:
        for raw_line in handle:
            line = raw_line.strip()
            if not line.startswith("[CLAS-MODEL-COMP]"):
                continue
            tokens = _parse_token_map(line)
            if tokens.get("type") != "phase":
                continue
            tow = _to_float(tokens.get("tow"))
            receiver_x = _to_float(tokens.get("rr_x"))
            receiver_y = _to_float(tokens.get("rr_y"))
            receiver_z = _to_float(tokens.get("rr_z"))
            if all(
                math.isfinite(value)
                for value in (tow, receiver_x, receiver_y, receiver_z)
            ):
                receiver_positions.setdefault(
                    _tow_key(tow),
                    ReceiverPosition(
                        tow=tow,
                        x_m=receiver_x,
                        y_m=receiver_y,
                        z_m=receiver_z,
                    ),
                )
            pco_pcv = (
                _to_float(tokens.get("pco_rcv"))
                + _to_float(tokens.get("pco_sat"))
                + _to_float(tokens.get("pcv_rcv"))
                + _to_float(tokens.get("pcv_sat"))
            )
            row = ClaslibRow(
                tow=tow,
                ref=tokens["ref"],
                sat=tokens["sat"],
                freq_group=int(tokens["freq_group"]),
                y_m=_to_float(tokens.get("y")),
                amb_m=_to_float(tokens.get("amb_m")),
                obs_m=_to_float(tokens.get("obs")),
                rho_m=_to_float(tokens.get("rho")),
                dts_m=_to_float(tokens.get("dts")),
                rel_m=_to_float(tokens.get("rel")),
                sagnac_m=_to_float(tokens.get("sagnac")),
                trop_model_m=_to_float(tokens.get("trop_model")),
                iono_grid_m=_to_float(tokens.get("iono_grid")),
                iono_state_term_m=_to_float(tokens.get("iono_state_term")),
                trop_grid_m=_to_float(tokens.get("trop_grid")),
                cpc_m=_to_float(tokens.get("cpc")),
                phase_bias_m=_to_float(tokens.get("phase_bias")),
                phase_comp_m=_to_float(tokens.get("phase_comp")),
                windup_m=_to_float(tokens.get("windup")),
                pco_pcv_m=pco_pcv,
            )
            rows[row.time_key] = row
    return rows, receiver_positions


def parse_legacy_measrow_dump(path: Path) -> Dict[MeasRowKey, LegacyMeasRow]:
    rows: Dict[MeasRowKey, LegacyMeasRow] = {}
    with path.open("r", encoding="utf-8") as handle:
        for raw_line in handle:
            line = raw_line.strip()
            if not line.startswith("tow="):
                continue
            tokens = _parse_token_map(line)
            tow = _to_float(tokens.get("tow"))
            row_index = _to_int(tokens.get("row"), -1)
            ref = tokens.get("ref", "")
            sat = tokens.get("sat", "")
            if (
                not math.isfinite(tow)
                or row_index < 0
                or not ref
                or ref == "none"
                or not sat
                or sat == "none"
            ):
                continue
            row = LegacyMeasRow(
                tow=tow,
                row_index=row_index,
                ref=ref,
                sat=sat,
                is_phase=_to_int(tokens.get("is_phase")) == 1,
                freq_group=_to_int(tokens.get("freq")),
            )
            rows[row.key] = row
    return rows


def parse_claslib_legacy_model_comp(
    model_comp_path: Path,
    measrow_path: Path,
) -> Tuple[Dict[TimePairKey, ClaslibRow], Dict[int, ReceiverPosition]]:
    meas_rows = parse_legacy_measrow_dump(measrow_path)
    rows: Dict[TimePairKey, ClaslibRow] = {}
    row_index_by_tow: Dict[int, int] = {}
    with model_comp_path.open("r", encoding="utf-8") as handle:
        for raw_line in handle:
            line = raw_line.strip()
            if not line.startswith("tow="):
                continue
            tokens = _parse_token_map(line)
            tow = _to_float(tokens.get("tow"))
            if not math.isfinite(tow):
                continue
            tow_key = _tow_key(tow)
            row_index = _to_int(tokens.get("row"), row_index_by_tow.get(tow_key, 0))
            row_index_by_tow[tow_key] = row_index + 1
            meas_row = meas_rows.get((tow_key, row_index))
            if meas_row is None or not meas_row.is_phase:
                continue
            if _to_int(tokens.get("is_phase")) != 1:
                continue
            freq_group = _to_int(tokens.get("freq"), meas_row.freq_group)
            if freq_group != meas_row.freq_group:
                continue
            trop_model = _first_float(tokens, ("trop_model",))
            if not math.isfinite(trop_model):
                trop_model = _sum_present(tokens, ("trop_d", "trop_w"))
            pco_pcv = _sum_present(
                tokens,
                ("pco_rcv", "pco_sat", "pcv_rcv", "pcv_sat"),
            )
            if not math.isfinite(pco_pcv):
                pco_pcv = _sum_present(tokens, ("pco_r", "pco_s", "pcv_r", "pcv_s"))
            row = ClaslibRow(
                tow=tow,
                ref=meas_row.ref,
                sat=meas_row.sat,
                freq_group=freq_group,
                y_m=_to_float(tokens.get("y")),
                amb_m=_first_float(tokens, ("amb_m", "amb")),
                obs_m=_to_float(tokens.get("obs")),
                rho_m=_to_float(tokens.get("rho")),
                dts_m=_to_float(tokens.get("dts")),
                rel_m=_to_float(tokens.get("rel")),
                sagnac_m=_to_float(tokens.get("sagnac")),
                trop_model_m=trop_model,
                iono_grid_m=_to_float(tokens.get("iono_grid")),
                iono_state_term_m=_first_float(
                    tokens, ("iono_state_term", "iono_state")
                ),
                trop_grid_m=_to_float(tokens.get("trop_grid")),
                cpc_m=_to_float(tokens.get("cpc")),
                phase_bias_m=_first_float(tokens, ("phase_bias", "pb")),
                phase_comp_m=_first_float(tokens, ("phase_comp", "pcomp")),
                windup_m=_first_float(tokens, ("windup", "wu")),
                pco_pcv_m=pco_pcv,
                components_are_sd=True,
            )
            rows[row.time_key] = row
    return rows, {}


def parse_libgnss_pos(path: Path) -> Dict[int, ReceiverPosition]:
    positions: Dict[int, ReceiverPosition] = {}
    with path.open("r", encoding="utf-8") as handle:
        for raw_line in handle:
            line = raw_line.strip()
            if not line or line.startswith("%"):
                continue
            parts = line.split()
            if len(parts) < 5:
                continue
            tow = _to_float(parts[1])
            x_m = _to_float(parts[2])
            y_m = _to_float(parts[3])
            z_m = _to_float(parts[4])
            if not all(math.isfinite(value) for value in (tow, x_m, y_m, z_m)):
                continue
            positions[_tow_key(tow)] = ReceiverPosition(
                tow=tow,
                x_m=x_m,
                y_m=y_m,
                z_m=z_m,
            )
    return positions


def parse_claslib_stat_pos(path: Path) -> Dict[int, ReceiverPosition]:
    positions: Dict[int, ReceiverPosition] = {}
    with path.open("r", encoding="utf-8") as handle:
        for raw_line in handle:
            line = raw_line.strip()
            if not line.startswith("$POS,"):
                continue
            parts = line.split(",")
            if len(parts) < 7:
                continue
            tow = _to_float(parts[2])
            x_m = _to_float(parts[4])
            y_m = _to_float(parts[5])
            z_m = _to_float(parts[6])
            if not all(math.isfinite(value) for value in (tow, x_m, y_m, z_m)):
                continue
            positions[_tow_key(tow)] = ReceiverPosition(
                tow=tow,
                x_m=x_m,
                y_m=y_m,
                z_m=z_m,
            )
    return positions


def parse_claslib_state_history(
    path: Path,
) -> Dict[str, Dict[int, ReceiverPosition]]:
    positions_by_stage: Dict[str, Dict[int, ReceiverPosition]] = {}
    with path.open("r", encoding="utf-8") as handle:
        for raw_line in handle:
            line = raw_line.strip()
            if not line.startswith("[CLAS-RCV-POS]"):
                continue
            tokens = _parse_token_map(line)
            stage = tokens.get("stage", "")
            tow = _to_float(tokens.get("tow"))
            if stage == "after_pntpos":
                x_m = _to_float(tokens.get("spp_x"))
                y_m = _to_float(tokens.get("spp_y"))
                z_m = _to_float(tokens.get("spp_z"))
                clock_m = _to_float(tokens.get("spp_clk_m"))
            else:
                x_m = _to_float(tokens.get("filter_x"))
                y_m = _to_float(tokens.get("filter_y"))
                z_m = _to_float(tokens.get("filter_z"))
                clock_m = _to_float(tokens.get("filter_clk_m"))
            if not stage or not all(
                math.isfinite(value) for value in (tow, x_m, y_m, z_m)
            ):
                continue
            positions_by_stage.setdefault(stage, {})[_tow_key(tow)] = (
                ReceiverPosition(
                    tow=tow,
                    x_m=x_m,
                    y_m=y_m,
                    z_m=z_m,
                    clock_m=clock_m,
                )
            )
    return positions_by_stage


def filter_pairs(rows: Iterable[LibRow], pairs: Sequence[str]) -> List[LibRow]:
    if not pairs:
        return list(rows)
    wanted = set()
    for pair in pairs:
        parts = pair.replace(":", ",").split(",")
        if len(parts) != 3:
            raise ValueError(f"bad --pair '{pair}', expected REF,SAT,FREQ")
        wanted.add((parts[0], parts[1], int(parts[2])))
    return [row for row in rows if row.pair_key in wanted]


def filter_tow_window(
    rows: Iterable[LibRow],
    tow_min: Optional[float],
    tow_max: Optional[float],
) -> List[LibRow]:
    filtered: List[LibRow] = []
    for row in rows:
        if tow_min is not None and row.tow < tow_min:
            continue
        if tow_max is not None and row.tow > tow_max:
            continue
        filtered.append(row)
    return filtered


def filter_receiver_positions(
    positions: Dict[int, ReceiverPosition],
    tow_min: Optional[float],
    tow_max: Optional[float],
) -> Dict[int, ReceiverPosition]:
    filtered: Dict[int, ReceiverPosition] = {}
    for tow_key, position in positions.items():
        if tow_min is not None and position.tow < tow_min:
            continue
        if tow_max is not None and position.tow > tow_max:
            continue
        filtered[tow_key] = position
    return filtered


def lookup_claslib(
    rows: Dict[TimePairKey, ClaslibRow], lib_row: LibRow
) -> Tuple[Optional[ClaslibRow], int, str]:
    key = lib_row.time_key
    direct = rows.get(key)
    if direct is not None:
        return direct, 1, "direct"
    tow, ref, sat, freq = key
    reversed_row = rows.get((tow, sat, ref, freq))
    if reversed_row is not None:
        return reversed_row, -1, "reversed"
    return None, 1, "missing"


def _rms(values: Sequence[float]) -> float:
    finite = [value for value in values if math.isfinite(value)]
    if not finite:
        return math.nan
    return math.sqrt(sum(value * value for value in finite) / len(finite))


def _position_delta_norm(lhs: ReceiverPosition, rhs: ReceiverPosition) -> float:
    return math.sqrt(
        (lhs.x_m - rhs.x_m) * (lhs.x_m - rhs.x_m)
        + (lhs.y_m - rhs.y_m) * (lhs.y_m - rhs.y_m)
        + (lhs.z_m - rhs.z_m) * (lhs.z_m - rhs.z_m)
    )


def _sd(ref_ud: LibUdRow, sat_ud: LibUdRow, field: str) -> float:
    return getattr(ref_ud, field) - getattr(sat_ud, field)


def summarize(
    lib_rows: List[LibRow],
    lib_ud_rows: Dict[UdKey, LibUdRow],
    lib_receiver_positions: Dict[int, ReceiverPosition],
    clas_rows: Dict[TimePairKey, ClaslibRow],
    clas_receiver_positions: Dict[int, ReceiverPosition],
    top: int,
) -> str:
    compared = []
    missing = 0
    missing_ud = 0
    for lib_row in lib_rows:
        clas_row, sign, orientation = lookup_claslib(clas_rows, lib_row)
        if clas_row is None:
            missing += 1
            continue
        tow_key = _tow_key(lib_row.tow)
        ref_ud = lib_ud_rows.get((tow_key, lib_row.ref, lib_row.freq_group))
        sat_ud = lib_ud_rows.get((tow_key, lib_row.sat, lib_row.freq_group))
        if ref_ud is None or sat_ud is None:
            missing_ud += 1
        clas_noamb = sign * clas_row.noamb_m
        clas_y = sign * clas_row.y_m
        clas_amb = sign * clas_row.amb_m
        delta_noamb = lib_row.sd_noamb_m - clas_noamb
        delta_amb = lib_row.sd_ambiguity_term_m - clas_amb
        delta_resid = lib_row.sd_residual_m - clas_y
        values = {
            "abs_delta_noamb": abs(delta_noamb),
            "delta_noamb": delta_noamb,
            "delta_amb": delta_amb,
            "delta_resid": delta_resid,
            "lib_corrected_phase_sd": math.nan,
            "clas_raw_phase_sd": (
                sign * clas_row.obs_m if clas_row.components_are_sd else math.nan
            ),
            "clas_corrected_phase_sd": math.nan,
            "delta_raw_phase_sd": math.nan,
            "delta_applied_corr_sd": math.nan,
            "delta_corrected_phase_sd": math.nan,
            "delta_model_raw": math.nan,
            "delta_model_corrected": math.nan,
            "delta_geo_clock_sd": math.nan,
            "delta_geo_no_sagnac_sd": math.nan,
            "delta_sagnac_sd": math.nan,
            "delta_sat_clock_sd": math.nan,
            "delta_trop_grid_sd": math.nan,
            "delta_cpc_sd": math.nan,
            "delta_cpc_no_trop_sd": math.nan,
            "delta_phase_bias_sd": math.nan,
            "delta_phase_comp_sd": math.nan,
            "delta_windup_sd": math.nan,
            "delta_trop_model_sd": math.nan,
            "delta_phase_iono_state_sd": math.nan,
            "lib_model_corrected": math.nan,
            "clas_model_corrected": math.nan,
            "lib_row": lib_row,
            "clas_row": clas_row,
            "sign": sign,
            "orientation": orientation,
        }
        if ref_ud is not None and sat_ud is not None and clas_row.components_are_sd:
            lib_raw_phase_sd = _sd(ref_ud, sat_ud, "raw_phase_m")
            lib_applied_corr_sd = _sd(ref_ud, sat_ud, "carrier_phase_correction_m")
            lib_corrected_phase_sd = _sd(ref_ud, sat_ud, "l_corr_m")
            lib_osr_cpc_sd = _sd(ref_ud, sat_ud, "osr_cpc_m")
            lib_osr_trop_sd = _sd(ref_ud, sat_ud, "osr_trop_correction_m")
            lib_osr_cpc_no_trop_sd = lib_osr_cpc_sd - lib_osr_trop_sd
            lib_geo_clock_sd = _sd(ref_ud, sat_ud, "geo_clock_m")
            lib_geo_no_sagnac_sd = _sd(
                ref_ud, sat_ud, "geometric_range_no_sagnac_m"
            )
            lib_sagnac_sd = _sd(ref_ud, sat_ud, "sagnac_m")
            lib_sat_clock_sd = _sd(ref_ud, sat_ud, "satellite_clock_m")
            lib_trop_model_sd = _sd(ref_ud, sat_ud, "trop_m")
            lib_phase_iono_state_sd = (
                ref_ud.phase_iono_state_m - sat_ud.phase_iono_state_m
            )

            clas_raw_phase_sd = sign * clas_row.obs_m
            clas_cpc_sd = sign * clas_row.cpc_m
            clas_trop_grid_sd = sign * clas_row.trop_grid_m
            clas_cpc_no_trop_sd = clas_cpc_sd - clas_trop_grid_sd
            clas_corrected_phase_sd = clas_raw_phase_sd - clas_cpc_no_trop_sd
            clas_geo_clock_sd = sign * (
                clas_row.rho_m + clas_row.sagnac_m - clas_row.dts_m
            )
            clas_geo_no_sagnac_sd = sign * clas_row.rho_m
            clas_sagnac_sd = sign * clas_row.sagnac_m
            clas_sat_clock_sd = sign * clas_row.dts_m
            clas_trop_model_sd = sign * clas_row.trop_model_m
            clas_phase_iono_state_sd = sign * clas_row.iono_state_term_m
            lib_model_corrected = lib_corrected_phase_sd - lib_row.sd_noamb_m
            clas_model_corrected = clas_corrected_phase_sd - clas_noamb
            values.update(
                {
                    "lib_corrected_phase_sd": lib_corrected_phase_sd,
                    "clas_corrected_phase_sd": clas_corrected_phase_sd,
                    "delta_raw_phase_sd": lib_raw_phase_sd - clas_raw_phase_sd,
                    "delta_applied_corr_sd": (
                        lib_applied_corr_sd - clas_cpc_no_trop_sd
                    ),
                    "delta_corrected_phase_sd": (
                        lib_corrected_phase_sd - clas_corrected_phase_sd
                    ),
                    "delta_model_raw": (
                        lib_model_corrected
                        - sign * (clas_row.obs_m - clas_row.noamb_m)
                    ),
                    "delta_model_corrected": (
                        lib_model_corrected - clas_model_corrected
                    ),
                    "delta_geo_clock_sd": lib_geo_clock_sd - clas_geo_clock_sd,
                    "delta_geo_no_sagnac_sd": (
                        lib_geo_no_sagnac_sd - clas_geo_no_sagnac_sd
                    ),
                    "delta_sagnac_sd": lib_sagnac_sd - clas_sagnac_sd,
                    "delta_sat_clock_sd": lib_sat_clock_sd - clas_sat_clock_sd,
                    "delta_trop_grid_sd": lib_osr_trop_sd - clas_trop_grid_sd,
                    "delta_cpc_sd": lib_osr_cpc_sd - clas_cpc_sd,
                    "delta_cpc_no_trop_sd": (
                        lib_osr_cpc_no_trop_sd - clas_cpc_no_trop_sd
                    ),
                    "delta_phase_bias_sd": (
                        _sd(ref_ud, sat_ud, "osr_phase_bias_m")
                        - sign * clas_row.phase_bias_m
                    ),
                    "delta_phase_comp_sd": (
                        _sd(ref_ud, sat_ud, "osr_phase_compensation_m")
                        - sign * clas_row.phase_comp_m
                    ),
                    "delta_windup_sd": (
                        _sd(ref_ud, sat_ud, "osr_windup_m")
                        - sign * clas_row.windup_m
                    ),
                    "delta_trop_model_sd": (
                        lib_trop_model_sd - clas_trop_model_sd
                    ),
                    "delta_phase_iono_state_sd": (
                        lib_phase_iono_state_sd - clas_phase_iono_state_sd
                    ),
                    "lib_model_corrected": lib_model_corrected,
                    "clas_model_corrected": clas_model_corrected,
                }
            )
        compared.append(values)

    lines: List[str] = []
    lines.append(f"lib_rows={len(lib_rows)}")
    lines.append(f"claslib_phase_rows={len(clas_rows)}")
    lines.append(
        "claslib_non_sd_component_rows="
        f"{sum(1 for row in clas_rows.values() if not row.components_are_sd)}"
    )
    lines.append(f"matched_rows={len(compared)}")
    lines.append(f"missing_rows={missing}")
    lines.append(f"missing_ud_rows={missing_ud}")
    common_position_tows = sorted(
        set(lib_receiver_positions).intersection(clas_receiver_positions)
    )
    position_deltas = [
        _position_delta_norm(
            lib_receiver_positions[tow_key],
            clas_receiver_positions[tow_key],
        )
        for tow_key in common_position_tows
    ]
    lines.append(f"lib_receiver_positions={len(lib_receiver_positions)}")
    lines.append(f"claslib_receiver_positions={len(clas_receiver_positions)}")
    lines.append(f"receiver_position_matched={len(common_position_tows)}")
    if position_deltas:
        lines.append(f"receiver_position_rms_delta_m={_rms(position_deltas):.12g}")
    if compared:
        lines.append(f"sd_noamb_rms_delta_m={_rms([item['delta_noamb'] for item in compared]):.12g}")
        lines.append(f"sd_ambiguity_rms_delta_m={_rms([item['delta_amb'] for item in compared]):.12g}")
        lines.append(f"sd_residual_rms_delta_m={_rms([item['delta_resid'] for item in compared]):.12g}")
        lines.append(f"raw_phase_sd_rms_delta_m={_rms([item['delta_raw_phase_sd'] for item in compared]):.12g}")
        lines.append(f"applied_corr_sd_rms_delta_m={_rms([item['delta_applied_corr_sd'] for item in compared]):.12g}")
        lines.append(f"corrected_phase_sd_rms_delta_m={_rms([item['delta_corrected_phase_sd'] for item in compared]):.12g}")
        lines.append(f"model_corrected_noamb_rms_delta_m={_rms([item['delta_model_corrected'] for item in compared]):.12g}")
        lines.append(f"model_raw_noamb_rms_delta_m={_rms([item['delta_model_raw'] for item in compared]):.12g}")
        lines.append(f"geo_clock_sd_rms_delta_m={_rms([item['delta_geo_clock_sd'] for item in compared]):.12g}")
        lines.append(f"cpc_no_trop_sd_rms_delta_m={_rms([item['delta_cpc_no_trop_sd'] for item in compared]):.12g}")
        lines.append(f"phase_bias_sd_rms_delta_m={_rms([item['delta_phase_bias_sd'] for item in compared]):.12g}")
        lines.append(f"phase_comp_sd_rms_delta_m={_rms([item['delta_phase_comp_sd'] for item in compared]):.12g}")
    if common_position_tows:
        lines.append("")
        lines.append("receiver position deltas:")
        for tow_key in common_position_tows:
            lib_pos = lib_receiver_positions[tow_key]
            clas_pos = clas_receiver_positions[tow_key]
            lines.append(
                f"  tow={lib_pos.tow:.3f} "
                f"lib=({lib_pos.x_m:.6f},{lib_pos.y_m:.6f},{lib_pos.z_m:.6f}) "
                f"clas=({clas_pos.x_m:.6f},{clas_pos.y_m:.6f},{clas_pos.z_m:.6f}) "
                f"delta=({lib_pos.x_m - clas_pos.x_m:.6f},"
                f"{lib_pos.y_m - clas_pos.y_m:.6f},"
                f"{lib_pos.z_m - clas_pos.z_m:.6f}) "
                f"norm={_position_delta_norm(lib_pos, clas_pos):.6f}"
            )
    lines.append("")
    lines.append(f"top {top} sd_noamb deltas:")
    for item in sorted(
        compared, key=lambda value: value["abs_delta_noamb"], reverse=True
    )[:top]:
        delta_noamb = item["delta_noamb"]
        delta_amb = item["delta_amb"]
        delta_resid = item["delta_resid"]
        lib_row = item["lib_row"]
        clas_row = item["clas_row"]
        sign = item["sign"]
        orientation = item["orientation"]
        clas_noamb = sign * clas_row.noamb_m
        clas_y = sign * clas_row.y_m
        clas_obs_sd = item["clas_raw_phase_sd"]
        lines.append(
            f"  tow={lib_row.tow:.3f} {lib_row.ref}->{lib_row.sat}/f{lib_row.freq_group} "
            f"lib_noamb={lib_row.sd_noamb_m:.12f} clas_noamb={clas_noamb:.12f} "
            f"delta_noamb={delta_noamb:.12f} "
            f"lib_corr_phase_sd={item['lib_corrected_phase_sd']:.12f} "
            f"clas_raw_phase_sd={clas_obs_sd:.12f} "
            f"clas_corr_phase_sd={item['clas_corrected_phase_sd']:.12f} "
            f"delta_raw_phase={item['delta_raw_phase_sd']:.12f} "
            f"delta_applied_corr={item['delta_applied_corr_sd']:.12f} "
            f"delta_corr_phase={item['delta_corrected_phase_sd']:.12f} "
            f"lib_model_corr={item['lib_model_corrected']:.12f} "
            f"clas_model_corr={item['clas_model_corrected']:.12f} "
            f"delta_model_corr={item['delta_model_corrected']:.12f} "
            f"delta_geo_clock={item['delta_geo_clock_sd']:.12f} "
            f"delta_cpc_no_trop={item['delta_cpc_no_trop_sd']:.12f} "
            f"delta_phase_bias={item['delta_phase_bias_sd']:.12f} "
            f"delta_phase_comp={item['delta_phase_comp_sd']:.12f} "
            f"lib_amb={lib_row.sd_ambiguity_term_m:.12f} "
            f"clas_amb={sign * clas_row.amb_m:.12f} "
            f"delta_amb={delta_amb:.12f} "
            f"lib_resid={lib_row.sd_residual_m:.12f} clas_y={clas_y:.12f} "
            f"delta_resid={delta_resid:.12f} orientation={orientation} "
            f"clas_cpc={sign * clas_row.cpc_m:.12f} "
            f"clas_phase_bias={sign * clas_row.phase_bias_m:.12f} "
            f"clas_phase_comp={sign * clas_row.phase_comp_m:.12f} "
            f"clas_iono_grid={sign * clas_row.iono_grid_m:.12f} "
            f"clas_iono_state={sign * clas_row.iono_state_term_m:.12f} "
            f"clas_trop_grid={sign * clas_row.trop_grid_m:.12f}"
        )
    return "\n".join(lines)


def summarize_position_comparison(
    title: str,
    lhs_label: str,
    lhs_positions: Dict[int, ReceiverPosition],
    rhs_label: str,
    rhs_positions: Dict[int, ReceiverPosition],
    top: int,
) -> str:
    common_tows = sorted(set(lhs_positions).intersection(rhs_positions))
    deltas = [
        _position_delta_norm(lhs_positions[tow_key], rhs_positions[tow_key])
        for tow_key in common_tows
    ]
    lines: List[str] = []
    lines.append(title)
    lines.append(f"{lhs_label}_positions={len(lhs_positions)}")
    lines.append(f"{rhs_label}_positions={len(rhs_positions)}")
    lines.append(f"matched_positions={len(common_tows)}")
    if deltas:
        max_index = max(range(len(deltas)), key=lambda index: deltas[index])
        max_tow = lhs_positions[common_tows[max_index]].tow
        lines.append(f"position_rms_delta_m={_rms(deltas):.12g}")
        lines.append(f"position_max_delta_m={deltas[max_index]:.12g} tow={max_tow:.3f}")
        lines.append("position deltas:")
        for tow_key in common_tows[:top]:
            lhs = lhs_positions[tow_key]
            rhs = rhs_positions[tow_key]
            lines.append(
                f"  tow={lhs.tow:.3f} "
                f"{lhs_label}=({lhs.x_m:.6f},{lhs.y_m:.6f},{lhs.z_m:.6f}) "
                f"{rhs_label}=({rhs.x_m:.6f},{rhs.y_m:.6f},{rhs.z_m:.6f}) "
                f"delta=({lhs.x_m - rhs.x_m:.6f},"
                f"{lhs.y_m - rhs.y_m:.6f},"
                f"{lhs.z_m - rhs.z_m:.6f}) "
                f"norm={_position_delta_norm(lhs, rhs):.6f}"
            )
    return "\n".join(lines)


def parse_args(argv: Optional[Sequence[str]] = None) -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--libgnss-ddres", type=Path, required=True)
    parser.add_argument("--claslib-model-comp", type=Path, required=True)
    parser.add_argument(
        "--claslib-measrow-dump",
        type=Path,
        default=None,
        help=(
            "GNSS_PPP_MEASROW_DUMP file used to add REF/SAT metadata to legacy "
            "GNSS_PPP_MODEL_COMPONENTS_DUMP rows."
        ),
    )
    parser.add_argument("--libgnss-pos", type=Path, default=None)
    parser.add_argument("--libgnss-spp-pos", type=Path, default=None)
    parser.add_argument(
        "--claslib-pos",
        type=Path,
        default=None,
        help="CLASLIB regular .pos solution file.",
    )
    parser.add_argument("--claslib-pos-stat", type=Path, default=None)
    parser.add_argument("--claslib-state-history", type=Path, default=None)
    parser.add_argument("--pair", action="append", default=[], help="REF,SAT,FREQ or REF:SAT:FREQ")
    parser.add_argument("--tow-min", type=float, default=None)
    parser.add_argument("--tow-max", type=float, default=None)
    parser.add_argument("--top", type=int, default=16)
    return parser.parse_args(argv)


def main(argv: Optional[Sequence[str]] = None) -> int:
    args = parse_args(argv)
    lib_all_rows, lib_ud_rows, lib_receiver_positions = parse_libgnss_ddres(
        args.libgnss_ddres
    )
    lib_rows = filter_tow_window(
        filter_pairs(lib_all_rows, args.pair), args.tow_min, args.tow_max
    )
    lib_receiver_positions = filter_receiver_positions(
        lib_receiver_positions, args.tow_min, args.tow_max
    )
    if args.claslib_measrow_dump is not None:
        clas_rows, clas_receiver_positions = parse_claslib_legacy_model_comp(
            args.claslib_model_comp,
            args.claslib_measrow_dump,
        )
    else:
        clas_rows, clas_receiver_positions = parse_claslib_model_comp(
            args.claslib_model_comp
        )
    clas_receiver_positions = filter_receiver_positions(
        clas_receiver_positions, args.tow_min, args.tow_max
    )
    summaries = [
        summarize(
            lib_rows,
            lib_ud_rows,
            lib_receiver_positions,
            clas_rows,
            clas_receiver_positions,
            args.top,
        )
    ]
    if args.claslib_state_history is not None:
        claslib_state_positions_by_stage = parse_claslib_state_history(
            args.claslib_state_history
        )
        if args.libgnss_spp_pos is not None:
            lib_spp_positions = filter_receiver_positions(
                parse_libgnss_pos(args.libgnss_spp_pos), args.tow_min, args.tow_max
            )
            claslib_after_pntpos_positions = filter_receiver_positions(
                claslib_state_positions_by_stage.get("after_pntpos", {}),
                args.tow_min,
                args.tow_max,
            )
            if lib_spp_positions and claslib_after_pntpos_positions:
                summaries.append(
                    summarize_position_comparison(
                        "lib SPP vs CLASLIB after_pntpos:",
                        "lib_spp",
                        lib_spp_positions,
                        "claslib_after_pntpos",
                        claslib_after_pntpos_positions,
                        args.top,
                    )
                )
        for stage, positions in sorted(claslib_state_positions_by_stage.items()):
            filtered_positions = filter_receiver_positions(
                positions, args.tow_min, args.tow_max
            )
            if not filtered_positions:
                continue
            summaries.append(
                summarize_position_comparison(
                    f"lib prior vs CLASLIB state stage={stage}:",
                    "lib_prior",
                    lib_receiver_positions,
                    f"claslib_{stage}",
                    filtered_positions,
                    args.top,
                )
            )
    if args.libgnss_pos is not None and (
        args.claslib_pos is not None or args.claslib_pos_stat is not None
    ):
        lib_solution_positions = filter_receiver_positions(
            parse_libgnss_pos(args.libgnss_pos), args.tow_min, args.tow_max
        )
        claslib_solution_positions: Dict[int, ReceiverPosition] = {}
        if args.claslib_pos_stat is not None:
            claslib_solution_positions.update(
                parse_claslib_stat_pos(args.claslib_pos_stat)
            )
        if args.claslib_pos is not None:
            claslib_solution_positions.update(parse_libgnss_pos(args.claslib_pos))
        claslib_solution_positions = filter_receiver_positions(
            claslib_solution_positions, args.tow_min, args.tow_max
        )
        if claslib_solution_positions:
            summaries.append(
                summarize_position_comparison(
                    "lib prior vs CLASLIB solution:",
                    "lib_prior",
                    lib_receiver_positions,
                    "claslib_solution",
                    claslib_solution_positions,
                    args.top,
                )
            )
        summaries.append(
            summarize_position_comparison(
                "lib solution vs CLASLIB solution:",
                "lib_solution",
                lib_solution_positions,
                "claslib_solution",
                claslib_solution_positions,
                args.top,
            )
        )
    print("\n\n".join(summaries))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
