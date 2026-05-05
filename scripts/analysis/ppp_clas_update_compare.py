#!/usr/bin/env python3
"""Summarize libgnss++ and CLASLIB CLAS update-row dumps."""

from __future__ import annotations

import argparse
import math
import statistics
from collections import Counter, defaultdict
from dataclasses import dataclass
from pathlib import Path
from typing import DefaultDict, Dict, Iterable, List, Optional, Sequence, Tuple


TowKey = int
RowKey = Tuple[TowKey, str, str, str, int]


@dataclass(frozen=True)
class UpdateSummary:
    source: str
    tow: float
    nobs: int
    dx_kf_x: float = math.nan
    dx_kf_y: float = math.nan
    dx_kf_z: float = math.nan
    dx_anchor_x: float = math.nan
    dx_anchor_y: float = math.nan
    dx_anchor_z: float = math.nan

    @property
    def tow_key(self) -> TowKey:
        return _tow_key(self.tow)

    @property
    def dx_kf_norm(self) -> float:
        return _norm3(self.dx_kf_x, self.dx_kf_y, self.dx_kf_z)

    @property
    def dx_anchor_norm(self) -> float:
        return _norm3(self.dx_anchor_x, self.dx_anchor_y, self.dx_anchor_z)


@dataclass(frozen=True)
class UpdateRow:
    source: str
    tow: float
    row: int
    sat: str
    ref: str
    freq_group: int
    row_type: str
    y_m: float
    r_m2: float
    h_pos_norm: float = math.nan
    h_iono: float = math.nan
    h_trop: float = math.nan
    h_amb: float = math.nan
    h_amb_eff: float = math.nan
    selected_count: int = 0
    total_rows: int = 0
    focus_sat: str = ""
    focus_role: str = ""
    dx_state_cycles: float = math.nan
    state_before_cycles: float = math.nan
    state_after_cycles: float = math.nan

    @property
    def tow_key(self) -> TowKey:
        return _tow_key(self.tow)

    @property
    def class_type(self) -> str:
        if self.source == "libgnss" and self.row_type == "code" and self.freq_group < 0:
            return "iono_prior"
        return self.row_type

    @property
    def key(self) -> RowKey:
        return (self.tow_key, self.ref, self.sat, self.class_type, self.freq_group)


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
        return int(value)
    except ValueError:
        return default


def _tow_key(tow: float) -> TowKey:
    return int(round(tow * 1000.0))


def _tow_from_key(tow_key: TowKey) -> float:
    return tow_key / 1000.0


def _norm3(x: float, y: float, z: float) -> float:
    if not all(math.isfinite(value) for value in (x, y, z)):
        return math.nan
    return math.sqrt(x * x + y * y + z * z)


def _rms(values: Sequence[float]) -> float:
    finite = [value for value in values if math.isfinite(value)]
    if not finite:
        return math.nan
    return math.sqrt(sum(value * value for value in finite) / len(finite))


def _fmt(value: float, digits: int = 6) -> str:
    if not math.isfinite(value):
        return "nan"
    return f"{value:.{digits}f}"


def _format_counts(rows: Sequence[UpdateRow]) -> str:
    counts = Counter(row.class_type for row in rows)
    return " ".join(f"{name}={counts[name]}" for name in sorted(counts))


def _stats(values: Iterable[float]) -> Tuple[int, float, float, float, float]:
    finite = [value for value in values if math.isfinite(value)]
    if not finite:
        return 0, math.nan, math.nan, math.nan, math.nan
    return (
        len(finite),
        min(finite),
        statistics.median(finite),
        max(finite),
        _rms(finite),
    )


def parse_update_dump(
    path: Path,
    source_hint: Optional[str] = None,
) -> Tuple[List[UpdateSummary], List[UpdateRow], List[UpdateRow]]:
    summaries: List[UpdateSummary] = []
    rows: List[UpdateRow] = []
    focus_rows: List[UpdateRow] = []

    with path.open("r", encoding="utf-8") as handle:
        for raw_line in handle:
            line = raw_line.strip()
            if not line:
                continue
            if line.startswith("[CLAS-UPDATE-SUM]"):
                tokens = _parse_token_map(line)
                summaries.append(
                    UpdateSummary(
                        source=tokens.get("source", source_hint or ""),
                        tow=_to_float(tokens.get("tow")),
                        nobs=_to_int(tokens.get("nobs")),
                        dx_kf_x=_to_float(tokens.get("dx_kf_x")),
                        dx_kf_y=_to_float(tokens.get("dx_kf_y")),
                        dx_kf_z=_to_float(tokens.get("dx_kf_z")),
                        dx_anchor_x=_to_float(tokens.get("dx_anchor_x")),
                        dx_anchor_y=_to_float(tokens.get("dx_anchor_y")),
                        dx_anchor_z=_to_float(tokens.get("dx_anchor_z")),
                    )
                )
                continue
            if line.startswith("[CLAS-ROWS-SELECTED]"):
                tokens = _parse_token_map(line)
                rows.append(_row_from_tokens(tokens, source_hint or "claslib"))
                continue
            if line.startswith("[CLAS-UPDATE-ROW]"):
                tokens = _parse_token_map(line)
                row = _row_from_tokens(tokens, source_hint)
                if row.source == "libgnss":
                    rows.append(row)
                else:
                    focus_rows.append(row)
                continue
    return summaries, rows, focus_rows


def _row_from_tokens(tokens: Dict[str, str], source_hint: Optional[str]) -> UpdateRow:
    source = tokens.get("source", source_hint or "")
    ref = tokens.get("ref", "")
    if not ref and source == "libgnss":
        amb_sat = tokens.get("amb_sat", "")
        if amb_sat != "none":
            ref = amb_sat
    return UpdateRow(
        source=source,
        tow=_to_float(tokens.get("tow")),
        row=_to_int(tokens.get("row"), -1),
        sat=tokens.get("sat", ""),
        ref=ref,
        freq_group=_to_int(tokens.get("freq_group"), _to_int(tokens.get("f"), -1)),
        row_type=tokens.get("type", ""),
        y_m=_to_float(tokens.get("y")),
        r_m2=_to_float(tokens.get("R")),
        h_pos_norm=_to_float(tokens.get("h_pos_norm")),
        h_iono=_to_float(tokens.get("h_iono")),
        h_trop=_to_float(tokens.get("h_trop")),
        h_amb=_to_float(tokens.get("h_amb")),
        h_amb_eff=_to_float(tokens.get("h_amb_eff")),
        selected_count=_to_int(tokens.get("selected_count")),
        total_rows=_to_int(tokens.get("total_rows")),
        focus_sat=tokens.get("focus_sat", ""),
        focus_role=tokens.get("focus_role", ""),
        dx_state_cycles=_to_float(tokens.get("dx_state_cycles")),
        state_before_cycles=_to_float(tokens.get("state_before_cycles")),
        state_after_cycles=_to_float(tokens.get("state_after_cycles")),
    )


def _filter_tows(
    tow_keys: Iterable[TowKey],
    tow_min: Optional[float],
    tow_max: Optional[float],
) -> List[TowKey]:
    out = []
    for tow_key in sorted(set(tow_keys)):
        tow = _tow_from_key(tow_key)
        if tow_min is not None and tow < tow_min:
            continue
        if tow_max is not None and tow > tow_max:
            continue
        out.append(tow_key)
    return out


def _group_by_tow(rows: Sequence[UpdateRow]) -> DefaultDict[TowKey, List[UpdateRow]]:
    grouped: DefaultDict[TowKey, List[UpdateRow]] = defaultdict(list)
    for row in rows:
        grouped[row.tow_key].append(row)
    return grouped


def _summaries_by_tow(
    summaries: Sequence[UpdateSummary],
) -> Dict[TowKey, UpdateSummary]:
    return {summary.tow_key: summary for summary in summaries}


def _print_row_stats(label: str, rows: Sequence[UpdateRow]) -> None:
    print(f"{label}: total={len(rows)} {_format_counts(rows)}")
    grouped: DefaultDict[Tuple[str, int], List[UpdateRow]] = defaultdict(list)
    for row in rows:
        grouped[(row.class_type, row.freq_group)].append(row)
    for key in sorted(grouped):
        subset = grouped[key]
        r_count, r_min, r_med, r_max, _ = _stats(row.r_m2 for row in subset)
        y_count, _, _, _, y_rms = _stats(row.y_m for row in subset)
        print(
            "  "
            f"{key[0]}/f{key[1]} n={len(subset)} "
            f"R[min/med/max]={_fmt(r_min, 9)}/{_fmt(r_med, 9)}/{_fmt(r_max, 9)} "
            f"y_rms={_fmt(y_rms, 6)} "
            f"finite_R={r_count} finite_y={y_count}"
        )


def _print_focus_stats(label: str, rows: Sequence[UpdateRow]) -> None:
    if not rows:
        return
    by_type: DefaultDict[str, List[UpdateRow]] = defaultdict(list)
    for row in rows:
        by_type[row.class_type].append(row)
    print(f"{label}: focus_entries={len(rows)}")
    for row_type in sorted(by_type):
        subset = by_type[row_type]
        _, dx_min, dx_med, dx_max, dx_rms = _stats(
            row.dx_state_cycles for row in subset
        )
        print(
            "  "
            f"{row_type} n={len(subset)} "
            f"dx_cycles[min/med/max/rms]="
            f"{_fmt(dx_min, 6)}/{_fmt(dx_med, 6)}/{_fmt(dx_max, 6)}/{_fmt(dx_rms, 6)}"
        )


def _print_common_phase_delta(
    lib_rows: Sequence[UpdateRow],
    clas_rows: Sequence[UpdateRow],
    top: int,
) -> None:
    lib_by_key = {row.key: row for row in lib_rows if row.class_type == "phase"}
    clas_by_key = {row.key: row for row in clas_rows if row.class_type == "phase"}
    common_keys = sorted(set(lib_by_key) & set(clas_by_key))
    deltas = []
    for key in common_keys:
        lib_row = lib_by_key[key]
        clas_row = clas_by_key[key]
        deltas.append((lib_row.y_m - clas_row.y_m, lib_row, clas_row))
    finite_deltas = [delta for delta, _, _ in deltas if math.isfinite(delta)]
    print(
        "common_phase_rows: "
        f"matched={len(common_keys)} "
        f"y_delta_rms={_fmt(_rms(finite_deltas), 6)}"
    )
    for delta, lib_row, clas_row in sorted(
        deltas, key=lambda item: abs(item[0]) if math.isfinite(item[0]) else -1.0,
        reverse=True,
    )[:top]:
        print(
            "  "
            f"{lib_row.ref}->{lib_row.sat}/f{lib_row.freq_group} "
            f"lib_y={_fmt(lib_row.y_m, 6)} "
            f"clas_y={_fmt(clas_row.y_m, 6)} "
            f"delta={_fmt(delta, 6)} "
            f"lib_R={_fmt(lib_row.r_m2, 9)} "
            f"clas_R={_fmt(clas_row.r_m2, 9)}"
        )


def compare(args: argparse.Namespace) -> None:
    lib_summaries, lib_rows, _ = parse_update_dump(args.libgnss_update, "libgnss")
    _, clas_rows, clas_focus_rows = parse_update_dump(args.claslib_update, "claslib")

    lib_by_tow = _group_by_tow(lib_rows)
    clas_by_tow = _group_by_tow(clas_rows)
    focus_by_tow = _group_by_tow(clas_focus_rows)
    lib_summary_by_tow = _summaries_by_tow(lib_summaries)

    tow_keys = _filter_tows(
        set(lib_by_tow) | set(clas_by_tow) | set(focus_by_tow),
        args.tow_min,
        args.tow_max,
    )
    if args.max_tows:
        tow_keys = tow_keys[: args.max_tows]

    print(
        f"parsed lib_rows={len(lib_rows)} clas_selected_rows={len(clas_rows)} "
        f"clas_focus_rows={len(clas_focus_rows)}"
    )
    for tow_key in tow_keys:
        print(f"\n[tow {_tow_from_key(tow_key):.3f}]")
        summary = lib_summary_by_tow.get(tow_key)
        if summary is not None:
            print(
                "lib_update: "
                f"nobs={summary.nobs} "
                f"dx_kf_norm={_fmt(summary.dx_kf_norm, 6)} "
                f"dx_anchor_norm={_fmt(summary.dx_anchor_norm, 6)}"
            )
        _print_row_stats("lib_rows", lib_by_tow.get(tow_key, []))
        _print_row_stats("clas_selected", clas_by_tow.get(tow_key, []))
        _print_focus_stats("clas_focus_update", focus_by_tow.get(tow_key, []))
        _print_common_phase_delta(
            lib_by_tow.get(tow_key, []),
            clas_by_tow.get(tow_key, []),
            args.top,
        )


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("--libgnss-update", type=Path, required=True)
    parser.add_argument("--claslib-update", type=Path, required=True)
    parser.add_argument("--tow-min", type=float)
    parser.add_argument("--tow-max", type=float)
    parser.add_argument("--max-tows", type=int, default=0)
    parser.add_argument("--top", type=int, default=5)
    compare(parser.parse_args())


if __name__ == "__main__":
    main()
