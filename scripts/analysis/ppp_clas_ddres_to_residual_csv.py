#!/usr/bin/env python3
"""Convert CLAS DD residual dumps into canonical PPP residual CSV rows."""

from __future__ import annotations

import argparse
import csv
import math
from dataclasses import dataclass
from pathlib import Path
from typing import Dict, Iterable, List, Optional, Sequence


FIELDNAMES = [
    "week",
    "tow",
    "iteration",
    "row_index",
    "sat",
    "row_type",
    "observation_m",
    "predicted_m",
    "residual_m",
    "variance_m2",
    "elevation_deg",
    "iono_state_m",
    "solution_status",
    "primary_signal",
    "secondary_signal",
    "primary_observation_code",
    "secondary_observation_code",
    "frequency_index",
    "ionosphere_coefficient",
]


@dataclass(frozen=True)
class ResidualRow:
    week: int
    tow: float
    ref: str
    sat: str
    frequency_index: int
    residual_m: float
    variance_m2: float = math.nan
    observation_m: float = math.nan
    predicted_m: float = math.nan
    source_index: int = 0

    def output_sat(self, canonicalize_pairs: bool) -> tuple[str, float]:
        if not canonicalize_pairs or self.ref <= self.sat:
            return f"{self.ref}>{self.sat}", self.residual_m
        return f"{self.sat}>{self.ref}", -self.residual_m


def parse_token_map(line: str) -> Dict[str, str]:
    tokens: Dict[str, str] = {}
    for token in line.strip().split():
        if "=" not in token:
            continue
        key, value = token.split("=", 1)
        tokens[key] = value
    return tokens


def to_float(value: Optional[str], default: float = math.nan) -> float:
    if value is None or value == "":
        return default
    try:
        parsed = float(value)
    except ValueError:
        return default
    return parsed if math.isfinite(parsed) else default


def to_int(value: Optional[str], default: int = 0) -> int:
    if value is None or value == "":
        return default
    try:
        return int(float(value))
    except ValueError:
        return default


def parse_libgnss_ddres(path: Path, *, default_week: int = 0) -> List[ResidualRow]:
    rows: List[ResidualRow] = []
    section = ""
    week = default_week
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
                week = default_week
                tow = math.nan
                continue
            if not section and line.startswith("time_week="):
                week = to_int(line.split("=", 1)[1], default_week)
                continue
            if not section and line.startswith("time_tow="):
                tow = to_float(line.split("=", 1)[1])
                continue
            if section != "sd_phase" or not line.startswith("k="):
                continue
            tokens = parse_token_map(line)
            rows.append(
                ResidualRow(
                    week=week,
                    tow=tow,
                    ref=tokens["ref"],
                    sat=tokens["sat"],
                    frequency_index=to_int(
                        tokens.get("freq_group"), to_int(tokens.get("freq_index"))
                    ),
                    residual_m=to_float(tokens.get("sd_residual_m")),
                    variance_m2=to_float(tokens.get("variance_m2")),
                    source_index=to_int(tokens.get("k"), len(rows)),
                )
            )
    return rows


def parse_claslib_model_comp(path: Path, *, default_week: int = 0) -> List[ResidualRow]:
    rows: List[ResidualRow] = []
    with path.open("r", encoding="utf-8") as handle:
        for raw_line in handle:
            line = raw_line.strip()
            if not line.startswith("[CLAS-MODEL-COMP]"):
                continue
            tokens = parse_token_map(line)
            if tokens.get("type") != "phase":
                continue
            residual = to_float(tokens.get("y"))
            observation = to_float(tokens.get("obs"))
            predicted = observation - residual if math.isfinite(observation) and math.isfinite(residual) else math.nan
            rows.append(
                ResidualRow(
                    week=to_int(tokens.get("week"), default_week),
                    tow=to_float(tokens.get("tow")),
                    ref=tokens["ref"],
                    sat=tokens["sat"],
                    frequency_index=to_int(tokens.get("freq_group")),
                    residual_m=residual,
                    variance_m2=to_float(tokens.get("var")),
                    observation_m=observation,
                    predicted_m=predicted,
                    source_index=len(rows),
                )
            )
    return rows


def parse_measrow_dump(path: Path, *, default_week: int = 0) -> List[ResidualRow]:
    rows: List[ResidualRow] = []
    with path.open("r", encoding="utf-8") as handle:
        for raw_line in handle:
            line = raw_line.strip()
            if not line.startswith("tow="):
                continue
            tokens = parse_token_map(line)
            if to_int(tokens.get("is_phase")) != 1:
                continue
            ref = tokens.get("ref", "")
            sat = tokens.get("sat", "")
            if not ref or ref == "none" or not sat or sat == "none":
                continue
            rows.append(
                ResidualRow(
                    week=to_int(tokens.get("week"), default_week),
                    tow=to_float(tokens.get("tow")),
                    ref=ref,
                    sat=sat,
                    frequency_index=to_int(tokens.get("freq")),
                    residual_m=to_float(tokens.get("z")),
                    variance_m2=to_float(tokens.get("R")),
                    source_index=to_int(tokens.get("row"), len(rows)),
                )
            )
    return rows


def finite_text(value: float) -> str:
    return f"{value:.15g}" if math.isfinite(value) else ""


def row_to_csv_dict(
    row: ResidualRow,
    *,
    row_index: int,
    canonicalize_pairs: bool,
) -> Dict[str, str]:
    sat, residual = row.output_sat(canonicalize_pairs)
    predicted = (
        row.observation_m - residual
        if math.isfinite(row.observation_m) and not math.isfinite(row.predicted_m)
        else row.predicted_m
    )
    return {
        "week": str(row.week),
        "tow": f"{row.tow:.3f}",
        "iteration": "0",
        "row_index": str(row_index),
        "sat": sat,
        "row_type": "phase",
        "observation_m": finite_text(row.observation_m),
        "predicted_m": finite_text(predicted),
        "residual_m": finite_text(residual),
        "variance_m2": finite_text(row.variance_m2),
        "elevation_deg": "",
        "iono_state_m": "",
        "solution_status": "",
        "primary_signal": "",
        "secondary_signal": "",
        "primary_observation_code": "",
        "secondary_observation_code": "",
        "frequency_index": str(row.frequency_index),
        "ionosphere_coefficient": "1",
    }


def write_residual_csv(
    rows: Iterable[ResidualRow],
    path: Path,
    *,
    canonicalize_pairs: bool,
) -> int:
    path.parent.mkdir(parents=True, exist_ok=True)
    count = 0
    with path.open("w", newline="", encoding="utf-8") as handle:
        writer = csv.DictWriter(handle, fieldnames=FIELDNAMES)
        writer.writeheader()
        for count, row in enumerate(rows, start=1):
            writer.writerow(
                row_to_csv_dict(
                    row,
                    row_index=count - 1,
                    canonicalize_pairs=canonicalize_pairs,
                )
            )
    return count


def parse_args(argv: Optional[Sequence[str]] = None) -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    source = parser.add_mutually_exclusive_group(required=True)
    source.add_argument("--libgnss-ddres", type=Path)
    source.add_argument("--claslib-model-comp", type=Path)
    source.add_argument("--measrow-dump", type=Path)
    parser.add_argument("--out", type=Path, required=True)
    parser.add_argument(
        "--week",
        type=int,
        default=0,
        help="GPS week used when the input dump does not carry a week field.",
    )
    parser.add_argument(
        "--canonicalize-pairs",
        action="store_true",
        help="Sort REF/SAT pair labels and flip residual sign when orientation changes.",
    )
    return parser.parse_args(argv)


def main(argv: Optional[Sequence[str]] = None) -> int:
    args = parse_args(argv)
    if args.libgnss_ddres is not None:
        rows = parse_libgnss_ddres(args.libgnss_ddres, default_week=args.week)
    elif args.claslib_model_comp is not None:
        rows = parse_claslib_model_comp(args.claslib_model_comp, default_week=args.week)
    else:
        rows = parse_measrow_dump(args.measrow_dump, default_week=args.week)
    count = write_residual_csv(
        rows,
        args.out,
        canonicalize_pairs=args.canonicalize_pairs,
    )
    print(f"wrote {count} residual rows: {args.out}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
