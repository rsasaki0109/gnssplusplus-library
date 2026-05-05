#!/usr/bin/env python3
"""Summarize CLAS ddres-style SD phase dumps.

Reads dumps emitted by GNSS_PPP_CLAS_DDRES_DUMP and optionally compares each
SD ambiguity value to a CLASLIB/libgnss++ first-AR dump [dd_pairs] target.
"""

from __future__ import annotations

import argparse
import math
from dataclasses import dataclass
from pathlib import Path
from typing import Dict, Iterable, List, Optional, Sequence, Tuple


PairKey = Tuple[str, str, int]


@dataclass(frozen=True)
class SdRow:
    tow: float
    ref: str
    sat: str
    freq_group: int
    sd_x_cycles: float
    sd_residual_m: float
    sd_noamb_m: float
    sd_ambiguity_term_m: float

    @property
    def key(self) -> PairKey:
        return (self.ref, self.sat, self.freq_group)


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


def parse_ddres_dump(path: Path) -> List[SdRow]:
    rows: List[SdRow] = []
    section = ""
    tow = math.nan

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
                continue
            if not section and line.startswith("time_tow="):
                tow = _to_float(line.split("=", 1)[1])
                continue
            if section != "sd_phase" or not line.startswith("k="):
                continue
            tokens = _parse_token_map(line)
            rows.append(
                SdRow(
                    tow=tow,
                    ref=tokens["ref"],
                    sat=tokens["sat"],
                    freq_group=int(tokens["freq_group"]),
                    sd_x_cycles=_to_float(tokens.get("sd_x_cycles")),
                    sd_residual_m=_to_float(tokens.get("sd_residual_m")),
                    sd_noamb_m=_to_float(tokens.get("sd_noamb_m")),
                    sd_ambiguity_term_m=_to_float(tokens.get("sd_ambiguity_term_m")),
                )
            )
    return rows


def parse_ar_targets(path: Path) -> Dict[PairKey, float]:
    targets: Dict[PairKey, float] = {}
    section = ""
    with path.open("r", encoding="utf-8") as handle:
        for raw_line in handle:
            line = raw_line.strip()
            if not line:
                continue
            if line.startswith("[") and line.endswith("]"):
                section = line[1:-1]
                continue
            if section == "dd_pairs" and line.startswith("k="):
                tokens = _parse_token_map(line)
                key = (tokens["ref"], tokens["sat"], int(tokens["freq_group"]))
                targets[key] = _to_float(tokens.get("dd_float"))
    return targets


def target_for_key(targets: Dict[PairKey, float], key: PairKey) -> Tuple[float, str]:
    if key in targets:
        return targets[key], "direct"
    ref, sat, freq_group = key
    reversed_key = (sat, ref, freq_group)
    if reversed_key in targets:
        return -targets[reversed_key], "reversed"
    return math.nan, "missing"


def filter_rows(rows: Iterable[SdRow], pairs: Sequence[str]) -> List[SdRow]:
    if not pairs:
        return list(rows)
    wanted = set()
    for pair in pairs:
        parts = pair.replace(":", ",").split(",")
        if len(parts) != 3:
            raise ValueError(f"bad --pair '{pair}', expected REF,SAT,FREQ")
        wanted.add((parts[0], parts[1], int(parts[2])))
    return [row for row in rows if row.key in wanted]


def summarize(rows: List[SdRow], targets: Dict[PairKey, float], top: int) -> str:
    compared = []
    for row in rows:
        target, orientation = target_for_key(targets, row.key) if targets else (math.nan, "")
        delta = row.sd_x_cycles - target if math.isfinite(target) else math.nan
        compared.append((abs(delta) if math.isfinite(delta) else -1.0, delta, row, target, orientation))

    lines: List[str] = []
    lines.append(f"rows={len(rows)}")
    if targets:
        comparable = [item for item in compared if math.isfinite(item[1])]
        lines.append(f"comparable_to_ar={len(comparable)}")
        if comparable:
            rms = math.sqrt(sum(item[1] * item[1] for item in comparable) / len(comparable))
            lines.append(f"sd_x_vs_ar_rms_delta_cycles={rms:.12g}")
    lines.append("")
    lines.append(f"top {top} rows:")
    for _, delta, row, target, orientation in sorted(
        compared, key=lambda item: item[0], reverse=True
    )[:top]:
        target_text = "nan" if not math.isfinite(target) else f"{target:.12f}"
        delta_text = "nan" if not math.isfinite(delta) else f"{delta:.12f}"
        lines.append(
            f"  tow={row.tow:.3f} {row.ref}->{row.sat}/f{row.freq_group} "
            f"sd_x={row.sd_x_cycles:.12f} target={target_text} "
            f"delta={delta_text} orientation={orientation or 'none'} "
            f"sd_resid_m={row.sd_residual_m:.12f} "
            f"sd_noamb_m={row.sd_noamb_m:.12f} "
            f"sd_amb_term_m={row.sd_ambiguity_term_m:.12f}"
        )
    return "\n".join(lines)


def parse_args(argv: Optional[Sequence[str]] = None) -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--ddres", type=Path, required=True)
    parser.add_argument("--ar-dump", type=Path)
    parser.add_argument("--pair", action="append", default=[], help="REF,SAT,FREQ or REF:SAT:FREQ")
    parser.add_argument("--top", type=int, default=12)
    return parser.parse_args(argv)


def main(argv: Optional[Sequence[str]] = None) -> int:
    args = parse_args(argv)
    rows = filter_rows(parse_ddres_dump(args.ddres), args.pair)
    targets = parse_ar_targets(args.ar_dump) if args.ar_dump else {}
    print(summarize(rows, targets, args.top))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
