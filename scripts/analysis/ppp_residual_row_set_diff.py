#!/usr/bin/env python3
"""Compare row identities between two gnss_ppp --ppp-residual-log CSV files."""

from __future__ import annotations

import argparse
import csv
import json
from collections import Counter, defaultdict
from decimal import Decimal, InvalidOperation, ROUND_HALF_UP
from pathlib import Path
from typing import Any, DefaultDict, Dict, Iterable, List, Optional, Sequence, Tuple


REQUIRED_COLUMNS = [
    "week",
    "tow",
    "iteration",
    "row_index",
    "sat",
    "row_type",
    "residual_m",
    "iono_state_m",
]

Row = Dict[str, str]
GroupKey = Tuple[int, int, int, str]
RowIdentity = Tuple[str, str, str, str, str, int, str]

RowOccurrence = Tuple[GroupKey, RowIdentity]
PhaseCandidateKey = Tuple[int, int, int, RowIdentity]


def tow_to_millis(value: str) -> int:
    try:
        tow = Decimal(str(value))
    except InvalidOperation as exc:
        raise ValueError(f"invalid tow value: {value}") from exc
    return int((tow * Decimal("1000")).to_integral_value(rounding=ROUND_HALF_UP))


def to_int(value: str) -> int:
    if value == "":
        return 0
    try:
        return int(float(value))
    except ValueError:
        return 0


def normalize_float_text(value: str, default: str) -> str:
    if value == "":
        return default
    try:
        parsed = float(value)
    except ValueError:
        return value
    return f"{parsed:.12g}"


def read_csv(path: Path) -> List[Row]:
    with path.open(newline="", encoding="utf-8") as handle:
        reader = csv.DictReader(handle)
        if reader.fieldnames is None:
            raise ValueError(f"{path}: missing CSV header")
        missing = [column for column in REQUIRED_COLUMNS if column not in reader.fieldnames]
        if missing:
            raise ValueError(f"{path}: missing required columns: {', '.join(missing)}")
        return [dict(row) for row in reader]


def group_key(row: Row) -> GroupKey:
    return (
        int(float(row["week"])),
        tow_to_millis(row["tow"]),
        to_int(row.get("iteration", "0")),
        row.get("row_type", ""),
    )


def row_identity(row: Row) -> RowIdentity:
    return (
        row.get("sat", ""),
        row.get("primary_signal", ""),
        row.get("secondary_signal", ""),
        row.get("primary_observation_code", ""),
        row.get("secondary_observation_code", ""),
        to_int(row.get("frequency_index", "0")),
        normalize_float_text(row.get("ionosphere_coefficient", "1"), "1"),
    )


def row_occurrences(
    rows: Sequence[Row],
    *,
    row_type: Optional[str],
    iteration: Optional[int],
) -> List[RowOccurrence]:
    occurrences: List[RowOccurrence] = []
    for row in rows:
        if row_type is not None and row.get("row_type", "") != row_type:
            continue
        if iteration is not None and to_int(row.get("iteration", "0")) != iteration:
            continue
        occurrences.append((group_key(row), row_identity(row)))
    return occurrences


def group_counter(occurrences: Iterable[RowOccurrence]) -> Dict[GroupKey, Counter[RowIdentity]]:
    grouped: Dict[GroupKey, Counter[RowIdentity]] = {}
    for group, identity in occurrences:
        grouped.setdefault(group, Counter())[identity] += 1
    return grouped


def identity_to_dict(identity: RowIdentity, count: int) -> Dict[str, Any]:
    return {
        "sat": identity[0],
        "primary_signal": identity[1],
        "secondary_signal": identity[2],
        "primary_observation_code": identity[3],
        "secondary_observation_code": identity[4],
        "frequency_index": identity[5],
        "ionosphere_coefficient": identity[6],
        "count": count,
    }


def group_to_dict(group: GroupKey) -> Dict[str, Any]:
    return {
        "week": group[0],
        "tow": group[1] / 1000.0,
        "iteration": group[2],
        "row_type": group[3],
    }


def counter_only_rows(
    left: Counter[RowIdentity],
    right: Counter[RowIdentity],
) -> Counter[RowIdentity]:
    only: Counter[RowIdentity] = Counter()
    for identity, count in left.items():
        delta = count - right.get(identity, 0)
        if delta > 0:
            only[identity] = delta
    return only


def phase_candidate_key(row: Row) -> PhaseCandidateKey:
    return (
        int(float(row["week"])),
        tow_to_millis(row["tow"]),
        to_int(row.get("iteration", "0")),
        row_identity(row),
    )


def explain_base_only_phase_candidates(
    base_grouped: Dict[GroupKey, Counter[RowIdentity]],
    candidate_grouped: Dict[GroupKey, Counter[RowIdentity]],
    candidate_rows: Sequence[Row],
    *,
    top_examples: int = 20,
) -> Dict[str, Any]:
    candidate_phase_candidates: DefaultDict[PhaseCandidateKey, List[Row]] = defaultdict(list)
    for row in candidate_rows:
        if row.get("row_type", "") == "phase_candidate":
            candidate_phase_candidates[phase_candidate_key(row)].append(row)

    reason_counts: Counter[str] = Counter()
    reason_sat_counts: Counter[Tuple[str, str]] = Counter()
    examples: List[Dict[str, Any]] = []
    explained_rows = 0
    unexplained_rows = 0

    for group in sorted(set(base_grouped) | set(candidate_grouped)):
        if group[3] != "phase":
            continue
        base_counter = base_grouped.get(group, Counter())
        candidate_counter = candidate_grouped.get(group, Counter())
        base_only = counter_only_rows(base_counter, candidate_counter)
        for identity, count in sorted(base_only.items()):
            key = (group[0], group[1], group[2], identity)
            diagnostic_rows = candidate_phase_candidates.get(key, [])
            matched_rows = diagnostic_rows[:count]
            explained_rows += len(matched_rows)
            unexplained = count - len(matched_rows)
            unexplained_rows += unexplained
            for row in matched_rows:
                reason = row.get("phase_skip_reason", "") or "unknown"
                reason_counts[reason] += 1
                reason_sat_counts[(reason, identity[0])] += 1
                if len(examples) < top_examples:
                    examples.append(
                        {
                            **group_to_dict(group),
                            **identity_to_dict(identity, 1),
                            "phase_skip_reason": reason,
                            "residual_m": row.get("residual_m", ""),
                            "phase_limit_m": row.get("phase_limit_m", ""),
                            "ambiguity_lock_count": row.get("ambiguity_lock_count", ""),
                            "required_lock_count": row.get("required_lock_count", ""),
                        }
                    )
            if unexplained > 0:
                reason_counts["no_phase_candidate"] += unexplained
                reason_sat_counts[("no_phase_candidate", identity[0])] += unexplained
                if len(examples) < top_examples:
                    examples.append(
                        {
                            **group_to_dict(group),
                            **identity_to_dict(identity, unexplained),
                            "phase_skip_reason": "no_phase_candidate",
                            "residual_m": "",
                            "phase_limit_m": "",
                            "ambiguity_lock_count": "",
                            "required_lock_count": "",
                        }
                    )

    return {
        "explained_rows": explained_rows,
        "unexplained_rows": unexplained_rows,
        "skip_reasons": [
            {"reason": reason, "rows": rows}
            for reason, rows in reason_counts.most_common()
        ],
        "skip_reason_satellites": [
            {"reason": reason, "sat": sat, "rows": rows}
            for (reason, sat), rows in reason_sat_counts.most_common()
        ],
        "examples": examples,
    }


def compare_row_sets(
    base_rows: Sequence[Row],
    candidate_rows: Sequence[Row],
    *,
    row_type: Optional[str] = None,
    iteration: Optional[int] = None,
    top_groups: int = 10,
    top_satellites: int = 10,
    explain_phase_candidates: bool = False,
) -> Dict[str, Any]:
    base_grouped = group_counter(row_occurrences(base_rows, row_type=row_type, iteration=iteration))
    candidate_grouped = group_counter(
        row_occurrences(candidate_rows, row_type=row_type, iteration=iteration)
    )
    all_groups = sorted(set(base_grouped) | set(candidate_grouped))

    base_only_sats: Counter[str] = Counter()
    candidate_only_sats: Counter[str] = Counter()
    base_rows_count = 0
    candidate_rows_count = 0
    common_rows = 0
    base_only_rows = 0
    candidate_only_rows = 0
    group_reports: List[Dict[str, Any]] = []

    for group in all_groups:
        base_counter = base_grouped.get(group, Counter())
        candidate_counter = candidate_grouped.get(group, Counter())
        group_base_rows = sum(base_counter.values())
        group_candidate_rows = sum(candidate_counter.values())
        base_rows_count += group_base_rows
        candidate_rows_count += group_candidate_rows

        base_only = counter_only_rows(base_counter, candidate_counter)
        candidate_only = counter_only_rows(candidate_counter, base_counter)
        group_common = sum((base_counter & candidate_counter).values())
        group_base_only = sum(base_only.values())
        group_candidate_only = sum(candidate_only.values())

        common_rows += group_common
        base_only_rows += group_base_only
        candidate_only_rows += group_candidate_only
        for identity, count in base_only.items():
            base_only_sats[identity[0]] += count
        for identity, count in candidate_only.items():
            candidate_only_sats[identity[0]] += count

        if group_base_only or group_candidate_only:
            group_report = {
                **group_to_dict(group),
                "base_rows": group_base_rows,
                "candidate_rows": group_candidate_rows,
                "common_rows": group_common,
                "base_only_rows": group_base_only,
                "candidate_only_rows": group_candidate_only,
                "base_only": [
                    identity_to_dict(identity, count)
                    for identity, count in sorted(base_only.items())
                ],
                "candidate_only": [
                    identity_to_dict(identity, count)
                    for identity, count in sorted(candidate_only.items())
                ],
            }
            group_reports.append(group_report)

    group_reports.sort(
        key=lambda item: (
            -(item["base_only_rows"] + item["candidate_only_rows"]),
            item["week"],
            item["tow"],
            item["iteration"],
            item["row_type"],
        )
    )
    first_difference = min(
        group_reports,
        key=lambda item: (item["week"], item["tow"], item["iteration"], item["row_type"]),
        default=None,
    )

    report: Dict[str, Any] = {
        "row_type_filter": row_type,
        "iteration_filter": iteration,
        "groups_compared": len(all_groups),
        "groups_with_differences": len(group_reports),
        "base_rows": base_rows_count,
        "candidate_rows": candidate_rows_count,
        "common_rows": common_rows,
        "base_only_rows": base_only_rows,
        "candidate_only_rows": candidate_only_rows,
        "base_only_satellites": [
            {"sat": sat, "rows": count}
            for sat, count in base_only_sats.most_common(top_satellites)
        ],
        "candidate_only_satellites": [
            {"sat": sat, "rows": count}
            for sat, count in candidate_only_sats.most_common(top_satellites)
        ],
        "first_difference": first_difference,
        "top_different_groups": group_reports[:top_groups],
    }
    if explain_phase_candidates:
        report["base_only_phase_candidate_explanations"] = (
            explain_base_only_phase_candidates(
                base_grouped,
                candidate_grouped,
                candidate_rows,
                top_examples=top_groups,
            )
        )
    return report


def write_details_csv(path: Path, report: Dict[str, Any]) -> None:
    fieldnames = [
        "side",
        "week",
        "tow",
        "iteration",
        "row_type",
        "sat",
        "primary_signal",
        "secondary_signal",
        "primary_observation_code",
        "secondary_observation_code",
        "frequency_index",
        "ionosphere_coefficient",
        "count",
    ]
    rows: List[Dict[str, Any]] = []
    for group in report.get("top_different_groups", []):
        common = {
            "week": group["week"],
            "tow": f"{group['tow']:.3f}",
            "iteration": group["iteration"],
            "row_type": group["row_type"],
        }
        for side in ["base_only", "candidate_only"]:
            for item in group.get(side, []):
                rows.append({"side": side, **common, **item})

    path.parent.mkdir(parents=True, exist_ok=True)
    with path.open("w", newline="", encoding="utf-8") as handle:
        writer = csv.DictWriter(handle, fieldnames=fieldnames)
        writer.writeheader()
        writer.writerows(rows)


def print_summary(report: Dict[str, Any]) -> None:
    print("residual_row_set_diff:")
    for key in [
        "row_type_filter",
        "iteration_filter",
        "groups_compared",
        "groups_with_differences",
        "base_rows",
        "candidate_rows",
        "common_rows",
        "base_only_rows",
        "candidate_only_rows",
    ]:
        print(f"  {key}: {report[key]}")
    first = report.get("first_difference")
    if first is not None:
        print(
            "  first_difference: "
            f"{first['week']}:{first['tow']:.3f} iter={first['iteration']} "
            f"{first['row_type']} base_only={first['base_only_rows']} "
            f"candidate_only={first['candidate_only_rows']}"
        )
    print("  base_only_satellites:")
    print("    " + ", ".join(f"{item['sat']}={item['rows']}" for item in report["base_only_satellites"]))
    print("  candidate_only_satellites:")
    print(
        "    "
        + ", ".join(f"{item['sat']}={item['rows']}" for item in report["candidate_only_satellites"])
    )
    explanations = report.get("base_only_phase_candidate_explanations")
    if explanations is not None:
        print("  base_only_phase_candidate_explanations:")
        print(f"    explained_rows: {explanations['explained_rows']}")
        print(f"    unexplained_rows: {explanations['unexplained_rows']}")
        print(
            "    skip_reasons: "
            + ", ".join(
                f"{item['reason']}={item['rows']}"
                for item in explanations.get("skip_reasons", [])
            )
        )
    print("  top_different_groups:")
    for group in report["top_different_groups"]:
        print(
            "    "
            f"{group['week']}:{group['tow']:.3f} iter={group['iteration']} "
            f"{group['row_type']} base={group['base_rows']} candidate={group['candidate_rows']} "
            f"base_only={group['base_only_rows']} candidate_only={group['candidate_only_rows']}"
        )


def parse_args(argv: Optional[Sequence[str]] = None) -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description="Compare row identities between two gnss_ppp residual logs."
    )
    parser.add_argument("base_residual_log", type=Path)
    parser.add_argument("candidate_residual_log", type=Path)
    parser.add_argument("--row-type", help="compare only one row_type, for example phase")
    parser.add_argument("--iteration", type=int, help="compare only one filter iteration")
    parser.add_argument("--top-groups", type=int, default=10)
    parser.add_argument("--top-satellites", type=int, default=10)
    parser.add_argument("--json-out", type=Path)
    parser.add_argument("--details-csv", type=Path)
    parser.add_argument(
        "--explain-phase-candidates",
        action="store_true",
        help="explain base-only phase rows using candidate phase_candidate diagnostics",
    )
    return parser.parse_args(argv)


def main(argv: Optional[Sequence[str]] = None) -> int:
    args = parse_args(argv)
    base_rows = read_csv(args.base_residual_log)
    candidate_rows = read_csv(args.candidate_residual_log)
    report = compare_row_sets(
        base_rows,
        candidate_rows,
        row_type=args.row_type,
        iteration=args.iteration,
        top_groups=args.top_groups,
        top_satellites=args.top_satellites,
        explain_phase_candidates=args.explain_phase_candidates,
    )
    report["base_residual_log"] = str(args.base_residual_log)
    report["candidate_residual_log"] = str(args.candidate_residual_log)
    print_summary(report)
    if args.json_out is not None:
        args.json_out.parent.mkdir(parents=True, exist_ok=True)
        args.json_out.write_text(
            json.dumps(report, indent=2, sort_keys=True) + "\n",
            encoding="utf-8",
        )
    if args.details_csv is not None:
        write_details_csv(args.details_csv, report)
    return 0


if __name__ == "__main__":
    try:
        raise SystemExit(main())
    except BrokenPipeError:
        raise SystemExit(1)
