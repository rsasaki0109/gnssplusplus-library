#!/usr/bin/env python3
"""Compare PPP-AR first-dump DD vectors and Qb matrices.

The parser understands the text dumps emitted by the CLASLIB parity
instrumentation and libgnss++ GNSS_PPP_FIRST_AR_DUMP.
"""

from __future__ import annotations

import argparse
import math
from dataclasses import dataclass
from pathlib import Path
from typing import Dict, Iterable, List, Optional, Sequence, Tuple


PairKey = Tuple[str, str, int]
StateKey = Tuple[str, int]


@dataclass(frozen=True)
class DdPair:
    index: int
    ref: str
    sat: str
    freq_group: int
    dd_float: float
    dd_fixed: Optional[float]
    qdiag: float

    @property
    def key(self) -> PairKey:
        return (self.ref, self.sat, self.freq_group)


@dataclass(frozen=True)
class StateEntry:
    sat: str
    freq_group: int
    x_cycles: float
    pdiag_cycles2: float
    lock: Optional[int]
    fix: Optional[int]

    @property
    def key(self) -> StateKey:
        return (self.sat, self.freq_group)


@dataclass
class ArDump:
    path: Path
    header: Dict[str, str]
    pairs: List[DdPair]
    qa: List[List[float]]
    states: List[StateEntry]

    @property
    def pair_map(self) -> Dict[PairKey, DdPair]:
        return {pair.key: pair for pair in self.pairs}

    @property
    def state_map(self) -> Dict[StateKey, StateEntry]:
        return {state.key: state for state in self.states}


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


def parse_dump(path: Path) -> ArDump:
    header: Dict[str, str] = {}
    pairs: List[DdPair] = []
    states: List[StateEntry] = []
    rows: Dict[int, List[float]] = {}
    section = ""

    with path.open("r", encoding="utf-8") as handle:
        for raw_line in handle:
            line = raw_line.strip()
            if not line:
                continue
            if line.startswith("[") and line.endswith("]"):
                section = line[1:-1]
                continue
            if not section and "=" in line:
                key, value = line.split("=", 1)
                header[key] = value
                continue
            if section == "dd_pairs" and line.startswith("k="):
                tokens = _parse_token_map(line)
                dd_fixed = tokens.get("dd_fixed")
                pairs.append(
                    DdPair(
                        index=int(tokens["k"]),
                        ref=tokens["ref"],
                        sat=tokens["sat"],
                        freq_group=int(tokens["freq_group"]),
                        dd_float=float(tokens["dd_float"]),
                        dd_fixed=None if dd_fixed is None else float(dd_fixed),
                        qdiag=float(tokens["qdiag"]),
                    )
                )
                continue
            if section in {"phase_states", "eligible_states"} and "sat=" in line:
                tokens = _parse_token_map(line)
                if "freq_group" not in tokens:
                    continue
                sat = tokens.get("sat", "")
                x_cycles = _to_float(tokens.get("x_cycles"))
                pdiag_cycles2 = _to_float(tokens.get("pdiag_cycles2"))
                if not math.isfinite(x_cycles):
                    x_cycles = _to_float(tokens.get("x"))
                if not math.isfinite(pdiag_cycles2):
                    pdiag_cycles2 = _to_float(tokens.get("pdiag"))
                lock = tokens.get("lock")
                fix = tokens.get("fix", tokens.get("fixed"))
                states.append(
                    StateEntry(
                        sat=sat,
                        freq_group=int(tokens["freq_group"]),
                        x_cycles=x_cycles,
                        pdiag_cycles2=pdiag_cycles2,
                        lock=None if lock is None else int(lock),
                        fix=None if fix is None else int(fix),
                    )
                )
                continue
            if section == "lambda_Qa" and line.startswith("row="):
                tokens = _parse_token_map(line)
                row_index = int(tokens["row"])
                values: List[float] = []
                q_index = 0
                while f"q{q_index}" in tokens:
                    values.append(float(tokens[f"q{q_index}"]))
                    q_index += 1
                rows[row_index] = values

    qa = [rows[index] for index in sorted(rows)]
    return ArDump(path=path, header=header, pairs=pairs, qa=qa, states=states)


def _rms(values: Iterable[float]) -> float:
    finite = [value for value in values if math.isfinite(value)]
    if not finite:
        return math.nan
    return math.sqrt(sum(value * value for value in finite) / len(finite))


def _format_pair(key: PairKey) -> str:
    ref, sat, freq_group = key
    return f"{ref}->{sat}/f{freq_group}"


def _matrix_value(matrix: Sequence[Sequence[float]], row: int, col: int) -> float:
    if row < 0 or col < 0 or row >= len(matrix) or col >= len(matrix[row]):
        return math.nan
    return matrix[row][col]


def compare_dumps(base: ArDump, candidate: ArDump, top: int) -> str:
    base_pairs = base.pair_map
    candidate_pairs = candidate.pair_map
    base_states = base.state_map
    candidate_states = candidate.state_map
    common_keys = [pair.key for pair in base.pairs if pair.key in candidate_pairs]
    missing_keys = [pair.key for pair in base.pairs if pair.key not in candidate_pairs]
    extra_keys = [pair.key for pair in candidate.pairs if pair.key not in base_pairs]
    common_state_keys = [
        state.key
        for state in base.states
        if state.key in candidate_states
        and math.isfinite(state.x_cycles)
        and math.isfinite(candidate_states[state.key].x_cycles)
        and (state.fix is None or state.fix > 0)
    ]

    dd_deltas = []
    diag_deltas = []
    for key in common_keys:
        base_pair = base_pairs[key]
        candidate_pair = candidate_pairs[key]
        dd_deltas.append((candidate_pair.dd_float - base_pair.dd_float, key, base_pair, candidate_pair))
        diag_deltas.append((candidate_pair.qdiag - base_pair.qdiag, key, base_pair, candidate_pair))

    cov_deltas = []
    for row_key in common_keys:
        base_row = base_pairs[row_key].index
        candidate_row = candidate_pairs[row_key].index
        for col_key in common_keys:
            base_col = base_pairs[col_key].index
            candidate_col = candidate_pairs[col_key].index
            base_value = _matrix_value(base.qa, base_row, base_col)
            candidate_value = _matrix_value(candidate.qa, candidate_row, candidate_col)
            cov_deltas.append((candidate_value - base_value, row_key, col_key, base_value, candidate_value))

    state_x_deltas = []
    state_pdiag_deltas = []
    for key in common_state_keys:
        base_state = base_states[key]
        candidate_state = candidate_states[key]
        state_x_deltas.append(
            (candidate_state.x_cycles - base_state.x_cycles, key, base_state, candidate_state)
        )
        state_pdiag_deltas.append(
            (
                candidate_state.pdiag_cycles2 - base_state.pdiag_cycles2,
                key,
                base_state,
                candidate_state,
            )
        )

    lines = []
    lines.append(f"base: {base.path}")
    lines.append(f"candidate: {candidate.path}")
    lines.append(
        "header: "
        f"base_nb={base.header.get('nb', '?')} "
        f"candidate_nb={candidate.header.get('nb', '?')} "
        f"base_ratio={base.header.get('ratio', '?')} "
        f"candidate_ratio={candidate.header.get('ratio', '?')}"
    )
    lines.append(
        "pairs: "
        f"common={len(common_keys)} "
        f"missing_in_candidate={len(missing_keys)} "
        f"extra_in_candidate={len(extra_keys)}"
    )
    lines.append(f"states: common_fix_or_eligible={len(common_state_keys)}")
    if missing_keys:
        lines.append("missing_in_candidate: " + ", ".join(_format_pair(key) for key in missing_keys))
    if extra_keys:
        lines.append("extra_in_candidate: " + ", ".join(_format_pair(key) for key in extra_keys))

    lines.append(
        "summary: "
        f"dd_float_rms_delta={_rms(delta for delta, *_ in dd_deltas):.12g} "
        f"qdiag_rms_delta={_rms(delta for delta, *_ in diag_deltas):.12g} "
        f"qmatrix_rms_delta={_rms(delta for delta, *_ in cov_deltas):.12g} "
        f"state_x_rms_delta={_rms(delta for delta, *_ in state_x_deltas):.12g}"
    )

    lines.append("")
    lines.append(f"top {top} |dd_float_delta|:")
    for delta, key, base_pair, candidate_pair in sorted(
        dd_deltas, key=lambda item: abs(item[0]), reverse=True
    )[:top]:
        lines.append(
            f"  {_format_pair(key):14s} "
            f"base={base_pair.dd_float:.12f} "
            f"candidate={candidate_pair.dd_float:.12f} "
            f"delta={delta:.12f}"
        )

    lines.append("")
    lines.append(f"top {top} |state_x_delta| cycles:")
    for delta, key, base_state, candidate_state in sorted(
        state_x_deltas, key=lambda item: abs(item[0]), reverse=True
    )[:top]:
        sat, freq_group = key
        lines.append(
            f"  {sat}/f{freq_group:<2d} "
            f"base={base_state.x_cycles:.12f} "
            f"candidate={candidate_state.x_cycles:.12f} "
            f"delta={delta:.12f} "
            f"base_lock={base_state.lock} candidate_lock={candidate_state.lock}"
        )

    lines.append("")
    lines.append(f"top {top} |state_pdiag_delta| cycles^2:")
    for delta, key, base_state, candidate_state in sorted(
        state_pdiag_deltas, key=lambda item: abs(item[0]), reverse=True
    )[:top]:
        sat, freq_group = key
        lines.append(
            f"  {sat}/f{freq_group:<2d} "
            f"base={base_state.pdiag_cycles2:.12f} "
            f"candidate={candidate_state.pdiag_cycles2:.12f} "
            f"delta={delta:.12f}"
        )

    lines.append("")
    lines.append(f"top {top} |qdiag_delta|:")
    for delta, key, base_pair, candidate_pair in sorted(
        diag_deltas, key=lambda item: abs(item[0]), reverse=True
    )[:top]:
        lines.append(
            f"  {_format_pair(key):14s} "
            f"base={base_pair.qdiag:.12f} "
            f"candidate={candidate_pair.qdiag:.12f} "
            f"delta={delta:.12f}"
        )

    lines.append("")
    lines.append(f"top {top} |Qb_delta| common-submatrix entries:")
    for delta, row_key, col_key, base_value, candidate_value in sorted(
        cov_deltas, key=lambda item: abs(item[0]), reverse=True
    )[:top]:
        lines.append(
            f"  {_format_pair(row_key):14s} x {_format_pair(col_key):14s} "
            f"base={base_value:.12f} "
            f"candidate={candidate_value:.12f} "
            f"delta={delta:.12f}"
        )

    return "\n".join(lines)


def parse_args(argv: Optional[Sequence[str]] = None) -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--base", type=Path, required=True)
    parser.add_argument("--candidate", type=Path, required=True)
    parser.add_argument("--top", type=int, default=12)
    return parser.parse_args(argv)


def main(argv: Optional[Sequence[str]] = None) -> int:
    args = parse_args(argv)
    print(compare_dumps(parse_dump(args.base), parse_dump(args.candidate), args.top))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
