#!/usr/bin/env python3
"""Compare PPP residual-log row state update contributions."""

from __future__ import annotations

import argparse
import csv
import json
import math
from collections import defaultdict
from dataclasses import dataclass, field
from decimal import Decimal, InvalidOperation, ROUND_HALF_UP
from pathlib import Path
from typing import Any, DefaultDict, Dict, Iterable, List, Optional, Sequence, Tuple


REQUIRED_COLUMNS = [
    "week",
    "tow",
    "iteration",
    "sat",
    "row_type",
    "primary_signal",
    "secondary_signal",
    "primary_observation_code",
    "secondary_observation_code",
    "frequency_index",
    "ionosphere_coefficient",
    "position_update_contribution_x_m",
    "position_update_contribution_y_m",
    "position_update_contribution_z_m",
    "position_update_contribution_3d_m",
    "receiver_clock_update_contribution_m",
    "ionosphere_update_contribution_m",
    "ambiguity_update_contribution_m",
]

OPTIONAL_SUM_COLUMNS = [
    ("innovation_variance_m2", "innovation_variance_sum_m2"),
    ("innovation_inverse_diagonal_1_per_m2", "innovation_inverse_diagonal_sum_1_per_m2"),
    ("innovation_covariance_code_coupling_abs_m2", "innovation_covariance_code_coupling_abs_sum_m2"),
    ("innovation_covariance_phase_coupling_abs_m2", "innovation_covariance_phase_coupling_abs_sum_m2"),
    (
        "innovation_covariance_ionosphere_constraint_coupling_abs_m2",
        "innovation_covariance_ionosphere_constraint_coupling_abs_sum_m2",
    ),
    (
        "innovation_inverse_code_coupling_abs_1_per_m2",
        "innovation_inverse_code_coupling_abs_sum_1_per_m2",
    ),
    (
        "innovation_inverse_phase_coupling_abs_1_per_m2",
        "innovation_inverse_phase_coupling_abs_sum_1_per_m2",
    ),
    (
        "innovation_inverse_ionosphere_constraint_coupling_abs_1_per_m2",
        "innovation_inverse_ionosphere_constraint_coupling_abs_sum_1_per_m2",
    ),
]

OPTIONAL_GAIN_COLUMNS = [
    ("position_x_kalman_gain", "position_x_kalman_gain_abs_sum"),
    ("position_y_kalman_gain", "position_y_kalman_gain_abs_sum"),
    ("position_z_kalman_gain", "position_z_kalman_gain_abs_sum"),
    ("receiver_clock_kalman_gain", "receiver_clock_kalman_gain_abs_sum"),
    ("ionosphere_kalman_gain", "ionosphere_kalman_gain_abs_sum"),
    ("ambiguity_kalman_gain", "ambiguity_kalman_gain_abs_sum"),
]

Row = Dict[str, str]
Identity = Tuple[str, int, str, str, str, str, str]
WindowSpec = Tuple[str, Optional[float], Optional[float]]


def tow_to_millis(value: str) -> int:
    try:
        tow = Decimal(str(value))
    except InvalidOperation as exc:
        raise ValueError(f"invalid tow value: {value}") from exc
    return int((tow * Decimal("1000")).to_integral_value(rounding=ROUND_HALF_UP))


def to_float(value: str) -> float:
    if value == "":
        return 0.0
    try:
        parsed = float(value)
    except ValueError:
        return 0.0
    return parsed if math.isfinite(parsed) else 0.0


def to_int(value: str) -> int:
    if value == "":
        return 0
    try:
        return int(float(value))
    except ValueError:
        return 0


def read_csv(path: Path) -> List[Row]:
    with path.open(newline="", encoding="utf-8") as handle:
        reader = csv.DictReader(handle)
        if reader.fieldnames is None:
            raise ValueError(f"{path}: missing CSV header")
        missing = [column for column in REQUIRED_COLUMNS if column not in reader.fieldnames]
        if missing:
            raise ValueError(f"{path}: missing required columns: {', '.join(missing)}")
        return [dict(row) for row in reader]


def phase_identity(row: Row) -> Identity:
    return (
        row.get("sat", ""),
        to_int(row.get("frequency_index", "0")),
        row.get("primary_signal", ""),
        row.get("secondary_signal", ""),
        row.get("primary_observation_code", ""),
        row.get("secondary_observation_code", ""),
        f"{to_float(row.get('ionosphere_coefficient', '1')):.12g}",
    )


def identity_label(identity: Identity) -> str:
    sat, freq, primary_signal, secondary_signal, primary_code, secondary_code, iono_coeff = identity
    signal = primary_signal if not secondary_signal else f"{primary_signal}/{secondary_signal}"
    code = primary_code if not secondary_code else f"{primary_code}/{secondary_code}"
    return f"{sat}:f{freq}:{signal}:{code}:iono={iono_coeff}"


def parse_window(text: str) -> WindowSpec:
    parts = text.split(":")
    if len(parts) != 3 or not parts[0]:
        raise ValueError("window must look like name:start:end")
    start = None if parts[1] == "" else float(parts[1])
    end = None if parts[2] == "" else float(parts[2])
    if start is not None and end is not None and end < start:
        raise ValueError("window end must be >= start")
    return (parts[0], start, end)


def in_window(row: Row, window: WindowSpec) -> bool:
    _, start, end = window
    tow = float(row["tow"])
    if start is not None and tow < start:
        return False
    if end is not None and tow > end:
        return False
    return True


@dataclass
class ContributionStats:
    rows: int = 0
    epochs: set[Tuple[int, int]] = field(default_factory=set)
    position_x_m: float = 0.0
    position_y_m: float = 0.0
    position_z_m: float = 0.0
    position_3d_abs_sum_m: float = 0.0
    receiver_clock_m: float = 0.0
    ionosphere_m: float = 0.0
    ambiguity_m: float = 0.0
    residual_abs_sum_m: float = 0.0
    residual_abs_max_m: float = 0.0
    optional_metric_sums: Dict[str, float] = field(default_factory=dict)

    def add(self, row: Row) -> None:
        self.rows += 1
        self.epochs.add((int(float(row["week"])), tow_to_millis(row["tow"])))
        px = to_float(row.get("position_update_contribution_x_m", "0"))
        py = to_float(row.get("position_update_contribution_y_m", "0"))
        pz = to_float(row.get("position_update_contribution_z_m", "0"))
        p3 = to_float(row.get("position_update_contribution_3d_m", "0"))
        residual_abs = abs(to_float(row.get("residual_m", "0")))
        self.position_x_m += px
        self.position_y_m += py
        self.position_z_m += pz
        self.position_3d_abs_sum_m += abs(p3)
        self.receiver_clock_m += to_float(row.get("receiver_clock_update_contribution_m", "0"))
        self.ionosphere_m += to_float(row.get("ionosphere_update_contribution_m", "0"))
        self.ambiguity_m += to_float(row.get("ambiguity_update_contribution_m", "0"))
        self.residual_abs_sum_m += residual_abs
        self.residual_abs_max_m = max(self.residual_abs_max_m, residual_abs)
        for column, key in OPTIONAL_SUM_COLUMNS:
            self.optional_metric_sums[key] = self.optional_metric_sums.get(key, 0.0) + to_float(
                row.get(column, "0")
            )
        position_gain_abs = 0.0
        for column, key in OPTIONAL_GAIN_COLUMNS:
            value_abs = abs(to_float(row.get(column, "0")))
            self.optional_metric_sums[key] = self.optional_metric_sums.get(key, 0.0) + value_abs
            if column.startswith("position_"):
                position_gain_abs += value_abs
        self.optional_metric_sums["position_kalman_gain_abs_sum"] = (
            self.optional_metric_sums.get("position_kalman_gain_abs_sum", 0.0) + position_gain_abs
        )

    @property
    def position_signed_norm_m(self) -> float:
        return math.sqrt(
            self.position_x_m * self.position_x_m
            + self.position_y_m * self.position_y_m
            + self.position_z_m * self.position_z_m
        )

    def to_dict(self) -> Dict[str, Any]:
        result: Dict[str, Any] = {
            "rows": self.rows,
            "epochs": len(self.epochs),
            "position_update_sum_x_m": self.position_x_m,
            "position_update_sum_y_m": self.position_y_m,
            "position_update_sum_z_m": self.position_z_m,
            "position_update_signed_norm_m": self.position_signed_norm_m,
            "position_update_3d_abs_sum_m": self.position_3d_abs_sum_m,
            "receiver_clock_update_sum_m": self.receiver_clock_m,
            "ionosphere_update_sum_m": self.ionosphere_m,
            "ambiguity_update_sum_m": self.ambiguity_m,
            "residual_abs_mean_m": self.residual_abs_sum_m / self.rows if self.rows else 0.0,
            "residual_abs_max_m": self.residual_abs_max_m,
        }
        for key, value in sorted(self.optional_metric_sums.items()):
            result[key] = value
            if self.rows:
                result[key.replace("_sum", "_mean")] = value / self.rows
        return result


def selected_rows(
    rows: Iterable[Row],
    *,
    row_type: str,
    iteration: Optional[int],
    window: WindowSpec,
) -> List[Row]:
    selected: List[Row] = []
    for row in rows:
        if row.get("row_type", "") != row_type:
            continue
        if iteration is not None and to_int(row.get("iteration", "0")) != iteration:
            continue
        if not in_window(row, window):
            continue
        selected.append(row)
    return selected


def summarize_rows(rows: Sequence[Row]) -> ContributionStats:
    stats = ContributionStats()
    for row in rows:
        stats.add(row)
    return stats


def summarize_by_identity(rows: Sequence[Row]) -> Dict[Identity, ContributionStats]:
    grouped: DefaultDict[Identity, ContributionStats] = defaultdict(ContributionStats)
    for row in rows:
        grouped[phase_identity(row)].add(row)
    return dict(grouped)


def diff_stats(base: ContributionStats, candidate: ContributionStats) -> Dict[str, Any]:
    base_dict = base.to_dict()
    candidate_dict = candidate.to_dict()
    delta: Dict[str, Any] = {}
    for key, base_value in base_dict.items():
        candidate_value = candidate_dict[key]
        if isinstance(base_value, (int, float)) and isinstance(candidate_value, (int, float)):
            delta[key] = candidate_value - base_value
    return {"base": base_dict, "candidate": candidate_dict, "delta": delta}


def identity_summary(
    identity: Identity,
    base: ContributionStats,
    candidate: ContributionStats,
) -> Dict[str, Any]:
    summary = diff_stats(base, candidate)
    delta = summary["delta"]
    return {
        "identity": {
            "label": identity_label(identity),
            "sat": identity[0],
            "frequency_index": identity[1],
            "primary_signal": identity[2],
            "secondary_signal": identity[3],
            "primary_observation_code": identity[4],
            "secondary_observation_code": identity[5],
            "ionosphere_coefficient": identity[6],
        },
        **summary,
        "sort_key": abs(delta.get("position_update_signed_norm_m", 0.0))
        + abs(delta.get("ionosphere_update_sum_m", 0.0))
        + abs(delta.get("ambiguity_update_sum_m", 0.0))
        + abs(delta.get("receiver_clock_update_sum_m", 0.0)),
    }


def summarize_window(
    base_rows: Sequence[Row],
    candidate_rows: Sequence[Row],
    *,
    row_type: str,
    window: WindowSpec,
    iteration: Optional[int],
    top: int,
) -> Dict[str, Any]:
    base_selected = selected_rows(base_rows, row_type=row_type, iteration=iteration, window=window)
    candidate_selected = selected_rows(
        candidate_rows,
        row_type=row_type,
        iteration=iteration,
        window=window,
    )
    base_by_identity = summarize_by_identity(base_selected)
    candidate_by_identity = summarize_by_identity(candidate_selected)
    identities = sorted(set(base_by_identity) | set(candidate_by_identity))
    top_identities = [
        identity_summary(
            identity,
            base_by_identity.get(identity, ContributionStats()),
            candidate_by_identity.get(identity, ContributionStats()),
        )
        for identity in identities
    ]
    top_identities.sort(key=lambda item: item["sort_key"], reverse=True)
    for item in top_identities:
        item.pop("sort_key", None)

    return {
        "window": {"name": window[0], "start_tow": window[1], "end_tow": window[2]},
        "row_type": row_type,
        "iteration": iteration,
        "totals": diff_stats(summarize_rows(base_selected), summarize_rows(candidate_selected)),
        "top_identity_deltas": top_identities[:top],
    }


def summarize_contribution_diff(
    base_rows: Sequence[Row],
    candidate_rows: Sequence[Row],
    *,
    row_type: str = "phase",
    windows: Sequence[WindowSpec],
    iteration: Optional[int],
    top: int,
) -> Dict[str, Any]:
    return {
        "row_type": row_type,
        "iteration": iteration,
        "windows": [
            summarize_window(
                base_rows,
                candidate_rows,
                row_type=row_type,
                window=window,
                iteration=iteration,
                top=top,
            )
            for window in windows
        ],
    }


def print_summary(report: Dict[str, Any]) -> None:
    print("ppp_contribution_diff:")
    print(f"  row_type_filter: {report['row_type']}")
    print(f"  iteration_filter: {report['iteration'] if report['iteration'] is not None else 'all'}")
    for window in report["windows"]:
        name = window["window"]["name"]
        totals = window["totals"]
        base = totals["base"]
        candidate = totals["candidate"]
        delta = totals["delta"]
        print(f"  window {name}:")
        print(
            "    rows: "
            f"{base['rows']} -> {candidate['rows']} "
            f"delta={delta['rows']:+.0f}"
        )
        print(
            "    position signed norm: "
            f"{base['position_update_signed_norm_m']:.6g} -> "
            f"{candidate['position_update_signed_norm_m']:.6g} "
            f"delta={delta['position_update_signed_norm_m']:+.6g}"
        )
        print(
            "    clock/iono/ambiguity delta sums: "
            f"clock={delta['receiver_clock_update_sum_m']:+.6g} "
            f"iono={delta['ionosphere_update_sum_m']:+.6g} "
            f"amb={delta['ambiguity_update_sum_m']:+.6g}"
        )
        if "innovation_inverse_phase_coupling_abs_sum_1_per_m2" in delta:
            print(
                "    coupling/gain delta sums: "
                f"Sphase={delta['innovation_covariance_phase_coupling_abs_sum_m2']:+.6g} "
                f"Sinv_phase={delta['innovation_inverse_phase_coupling_abs_sum_1_per_m2']:+.6g} "
                f"Sinv_code={delta['innovation_inverse_code_coupling_abs_sum_1_per_m2']:+.6g} "
                f"pos_gain={delta['position_kalman_gain_abs_sum']:+.6g} "
                f"clock_gain={delta['receiver_clock_kalman_gain_abs_sum']:+.6g} "
                f"iono_gain={delta['ionosphere_kalman_gain_abs_sum']:+.6g}"
            )
        print("    top identity deltas:")
        for item in window["top_identity_deltas"]:
            identity = item["identity"]
            item_delta = item["delta"]
            print(
                "      "
                f"{identity['label']} rows={item['base']['rows']}->{item['candidate']['rows']} "
                f"pos_norm_delta={item_delta['position_update_signed_norm_m']:+.6g} "
                f"pos_xyz_delta=({item_delta['position_update_sum_x_m']:+.6g},"
                f"{item_delta['position_update_sum_y_m']:+.6g},"
                f"{item_delta['position_update_sum_z_m']:+.6g}) "
                f"clock={item_delta['receiver_clock_update_sum_m']:+.6g} "
                f"iono={item_delta['ionosphere_update_sum_m']:+.6g} "
                f"amb={item_delta['ambiguity_update_sum_m']:+.6g}"
            )


def main(argv: Optional[Sequence[str]] = None) -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("base_residual_log", type=Path)
    parser.add_argument("candidate_residual_log", type=Path)
    parser.add_argument(
        "--iteration",
        type=int,
        help="restrict to one filter iteration; default uses all iterations",
    )
    parser.add_argument(
        "--row-type",
        choices=("phase", "code"),
        default="phase",
        help="residual-log row_type to compare; default is accepted phase rows",
    )
    parser.add_argument(
        "--window",
        action="append",
        default=[],
        help="window name:start_tow:end_tow; may repeat",
    )
    parser.add_argument("--top", type=int, default=8, help="top identity deltas per window")
    parser.add_argument("--json-out", type=Path)
    args = parser.parse_args(argv)

    windows = [parse_window(item) for item in args.window]
    if not windows:
        windows = [("all", None, None)]

    report = summarize_contribution_diff(
        read_csv(args.base_residual_log),
        read_csv(args.candidate_residual_log),
        row_type=args.row_type,
        windows=windows,
        iteration=args.iteration,
        top=args.top,
    )
    print_summary(report)
    if args.json_out is not None:
        args.json_out.write_text(json.dumps(report, indent=2, sort_keys=True) + "\n", encoding="utf-8")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
