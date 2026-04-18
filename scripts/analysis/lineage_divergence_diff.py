#!/usr/bin/env python3
"""Compare RTK lineage telemetry CSVs over an early-divergence window."""

from __future__ import annotations

import argparse
import csv
import os
import statistics
import sys
from decimal import Decimal, InvalidOperation, ROUND_HALF_UP
from typing import Dict, Iterable, List, Optional, Sequence, Tuple


HELD_KEYS_COL = "hold_held_pair_keys"

REQUIRED_COLUMNS = [
    "gps_tow",
    "final_status",
    "accepted_fix_source",
    "accepted_candidate_path",
    "accepted_overwrite_before_pair_keys",
    "accepted_overwrite_after_pair_keys",
    "accepted_overlap_ratio",
    "accepted_overwrite_lineage_id",
    "accepted_overwrite_lineage_origin_tow",
    "accepted_preserved_prior_hold_state",
    "hold_used_preserved_backup",
    HELD_KEYS_COL,
]

DETAIL_COLUMNS = [
    "accepted_fix_source",
    "accepted_candidate_path",
    "accepted_overwrite_before_pair_keys",
    "accepted_overwrite_after_pair_keys",
    "hold_used_preserved_backup",
    "accepted_overwrite_lineage_id",
]

TRAJECTORY_COLUMNS = [
    "accepted_overwrite_lineage_id",
    "accepted_overwrite_lineage_origin_tow",
    "accepted_fix_source",
    "accepted_candidate_path",
]


Row = Dict[str, str]


def parse_tow_key(value: str) -> int:
    try:
        tow = Decimal(str(value))
    except InvalidOperation as exc:
        raise ValueError(f"invalid gps_tow value: {value}") from exc
    return int((tow * Decimal("1000")).to_integral_value(rounding=ROUND_HALF_UP))


def format_tow(key: Optional[int]) -> str:
    if key is None:
        return "none"
    return f"{key / 1000.0:.3f}"


def to_float(value: str) -> Optional[float]:
    if value == "":
        return None
    try:
        return float(value)
    except ValueError:
        return None


def to_int(value: str) -> Optional[int]:
    if value == "":
        return None
    try:
        return int(float(value))
    except ValueError:
        return None


def truthy(value: str) -> bool:
    return str(value).strip().lower() in {"1", "true", "yes", "y"}


def pair_set(value: str) -> set[str]:
    if value is None:
        return set()
    return {item for item in (part.strip() for part in value.split("|")) if item}


def overlap_ratio(left: set[str], right: set[str]) -> float:
    if not left and not right:
        return 1.0
    if not left or not right:
        return 0.0
    return len(left & right) / max(len(left), len(right))


def read_csv_rows(path: str) -> List[Row]:
    with open(path, newline="", encoding="utf-8") as handle:
        reader = csv.DictReader(handle)
        if reader.fieldnames is None:
            raise ValueError(f"{path}: missing CSV header")
        missing = [col for col in REQUIRED_COLUMNS if col not in reader.fieldnames]
        if missing:
            print(f"{path}: missing required columns: {', '.join(missing)}", file=sys.stderr)
            raise KeyError(", ".join(missing))
        rows: List[Row] = []
        for row in reader:
            row["_tow_key"] = str(parse_tow_key(row["gps_tow"]))
            rows.append(row)
        return rows


def filter_window(rows: Sequence[Row], start_key: Optional[int], end_key: Optional[int]) -> List[Row]:
    filtered = []
    for row in rows:
        key = int(row["_tow_key"])
        if start_key is not None and key < start_key:
            continue
        if end_key is not None and key > end_key:
            continue
        filtered.append(row)
    return filtered


def index_by_tow(rows: Sequence[Row]) -> Dict[int, Row]:
    indexed: Dict[int, Row] = {}
    for row in rows:
        indexed[int(row["_tow_key"])] = row
    return indexed


def first_divergence(common_tows: Sequence[int], base_by_tow: Dict[int, Row], backup_by_tow: Dict[int, Row]) -> Optional[int]:
    for tow in common_tows:
        if pair_set(base_by_tow[tow][HELD_KEYS_COL]) != pair_set(backup_by_tow[tow][HELD_KEYS_COL]):
            return tow
    return None


def first_disjoint(common_tows: Sequence[int], base_by_tow: Dict[int, Row], backup_by_tow: Dict[int, Row], column: str) -> Optional[int]:
    for tow in common_tows:
        base_keys = pair_set(base_by_tow[tow][column])
        backup_keys = pair_set(backup_by_tow[tow][column])
        if base_keys and backup_keys and not (base_keys & backup_keys):
            return tow
    return None


def compute_overlap_rows(common_tows: Sequence[int], base_by_tow: Dict[int, Row], backup_by_tow: Dict[int, Row]) -> List[Dict[str, object]]:
    rows: List[Dict[str, object]] = []
    for tow in common_tows:
        base_keys = pair_set(base_by_tow[tow][HELD_KEYS_COL])
        backup_keys = pair_set(backup_by_tow[tow][HELD_KEYS_COL])
        rows.append(
            {
                "tow": tow,
                "ratio": overlap_ratio(base_keys, backup_keys),
                "base_count": len(base_keys),
                "backup_count": len(backup_keys),
                "intersection_count": len(base_keys & backup_keys),
            }
        )
    return rows


def first_nonrecovering_lineage_advance(
    overlap_rows: Sequence[Dict[str, object]],
    base_by_tow: Dict[int, Row],
    backup_by_tow: Dict[int, Row],
) -> Optional[Tuple[int, int]]:
    low_start: Optional[int] = None
    prev_base_id: Optional[int] = None
    prev_backup_id: Optional[int] = None
    for item in overlap_rows:
        tow = int(item["tow"])
        ratio = float(item["ratio"])
        base_id = to_int(base_by_tow[tow]["accepted_overwrite_lineage_id"])
        backup_id = to_int(backup_by_tow[tow]["accepted_overwrite_lineage_id"])
        if ratio < 0.5:
            if low_start is None:
                low_start = tow
            base_advanced = prev_base_id is not None and base_id is not None and base_id > prev_base_id
            backup_advanced = prev_backup_id is not None and backup_id is not None and backup_id > prev_backup_id
            if base_advanced or backup_advanced:
                return tow, low_start
        else:
            low_start = None
        prev_base_id = base_id
        prev_backup_id = backup_id
    return None


def lineage_change_points(
    run_name: str,
    rows: Sequence[Row],
    overlap_by_tow: Dict[int, float],
) -> List[Dict[str, str]]:
    points: List[Dict[str, str]] = []
    prev_key: Optional[Tuple[str, str]] = None
    for row in sorted(rows, key=lambda item: int(item["_tow_key"])):
        key = (
            row["accepted_overwrite_lineage_id"],
            row["accepted_overwrite_lineage_origin_tow"],
        )
        if prev_key is None or key != prev_key:
            tow = int(row["_tow_key"])
            points.append(
                {
                    "run": run_name,
                    "tow": format_tow(tow),
                    "id": row["accepted_overwrite_lineage_id"] or "none",
                    "origin_tow": row["accepted_overwrite_lineage_origin_tow"] or "none",
                    "source": row["accepted_fix_source"] or "none",
                    "path": row["accepted_candidate_path"] or "none",
                    "overlap_ratio": f"{overlap_by_tow.get(tow, float('nan')):.3f}",
                }
            )
        prev_key = key
    return points


def fix_summary(rows: Sequence[Row]) -> Tuple[int, int, float]:
    fixed = sum(1 for row in rows if row.get("final_status") == "4")
    total = len(rows)
    rate = fixed / total if total else 0.0
    return fixed, total, rate


def backup_use_rows(rows: Sequence[Row]) -> List[Row]:
    return [row for row in rows if truthy(row.get("hold_used_preserved_backup", ""))]


def describe_run_at(row: Row) -> List[str]:
    return [row.get(col, "") or "none" for col in DETAIL_COLUMNS]


def make_table(headers: Sequence[str], rows: Sequence[Sequence[str]]) -> List[str]:
    lines = [
        "| " + " | ".join(headers) + " |",
        "| " + " | ".join("---" for _ in headers) + " |",
    ]
    for row in rows:
        lines.append("| " + " | ".join(row) + " |")
    return lines


def one_line_keys(value: str, max_len: int = 180) -> str:
    if value == "":
        return "none"
    if len(value) <= max_len:
        return value
    return value[: max_len - 3] + "..."


def decisive_candidates(
    first_div: Optional[int],
    first_below_half: Optional[int],
    nonrecovering_advance: Optional[Tuple[int, int]],
    accepted_after_disjoint: Optional[int],
    base_by_tow: Dict[int, Row],
    backup_by_tow: Dict[int, Row],
) -> List[str]:
    candidates: List[str] = []
    if first_div is not None:
        backup_row = backup_by_tow[first_div]
        reason = "first held-set divergence"
        if truthy(backup_row.get("hold_used_preserved_backup", "")):
            reason += " with hold_used_preserved_backup=1"
        elif truthy(backup_row.get("accepted_preserved_prior_hold_state", "")):
            reason += " with accepted_preserved_prior_hold_state=1"
        candidates.append(f"- {format_tow(first_div)}: {reason}; lineage id base={base_by_tow[first_div]['accepted_overwrite_lineage_id']} backup={backup_row['accepted_overwrite_lineage_id']}.")
    if first_below_half is not None and first_below_half != first_div:
        candidates.append(f"- {format_tow(first_below_half)}: base/backup held-set overlap first drops below 0.5, making the drift externally visible.")
    if accepted_after_disjoint is not None:
        candidates.append(f"- {format_tow(accepted_after_disjoint)}: accepted_overwrite_after_pair_keys first becomes fully disjoint across runs.")
    if nonrecovering_advance is not None:
        advance_tow, low_start = nonrecovering_advance
        text = f"- {format_tow(advance_tow)}: lineage advances during a continuous <0.5 overlap stretch that began at {format_tow(low_start)}."
        if text not in candidates:
            candidates.append(text)
    return candidates[:3] or ["- none: no branch candidate was visible from the requested telemetry."]


def analyze_rows(
    base_rows: Sequence[Row],
    backup_rows: Sequence[Row],
    window_start: Optional[int],
    window_end: Optional[int],
) -> Dict[str, object]:
    base_window = filter_window(base_rows, window_start, window_end)
    backup_window = filter_window(backup_rows, window_start, window_end)
    base_by_tow = index_by_tow(base_window)
    backup_by_tow = index_by_tow(backup_window)
    common_tows = sorted(set(base_by_tow) & set(backup_by_tow))

    overlap_rows = compute_overlap_rows(common_tows, base_by_tow, backup_by_tow)
    overlap_by_tow = {int(item["tow"]): float(item["ratio"]) for item in overlap_rows}
    ratios = [float(item["ratio"]) for item in overlap_rows]

    first_div = first_divergence(common_tows, base_by_tow, backup_by_tow)
    first_below_half = next((int(item["tow"]) for item in overlap_rows if float(item["ratio"]) < 0.5), None)
    first_below_one = next((int(item["tow"]) for item in overlap_rows if float(item["ratio"]) < 1.0), None)
    held_disjoint = first_disjoint(common_tows, base_by_tow, backup_by_tow, HELD_KEYS_COL)
    accepted_after_disjoint = first_disjoint(common_tows, base_by_tow, backup_by_tow, "accepted_overwrite_after_pair_keys")
    nonrecovering_advance = first_nonrecovering_lineage_advance(overlap_rows, base_by_tow, backup_by_tow)

    base_fix = fix_summary(base_window)
    backup_fix = fix_summary(backup_window)
    backup_events = backup_use_rows(backup_window)
    base_points = lineage_change_points("base", base_window, overlap_by_tow)
    backup_points = lineage_change_points("backup", backup_window, overlap_by_tow)

    return {
        "base_window": base_window,
        "backup_window": backup_window,
        "base_by_tow": base_by_tow,
        "backup_by_tow": backup_by_tow,
        "common_tows": common_tows,
        "overlap_rows": overlap_rows,
        "ratios": ratios,
        "first_divergence": first_div,
        "first_below_half": first_below_half,
        "first_below_one": first_below_one,
        "held_disjoint": held_disjoint,
        "accepted_after_disjoint": accepted_after_disjoint,
        "nonrecovering_advance": nonrecovering_advance,
        "base_fix": base_fix,
        "backup_fix": backup_fix,
        "backup_events": backup_events,
        "base_points": base_points,
        "backup_points": backup_points,
    }


def render_markdown(result: Dict[str, object], window_start: Optional[int], window_end: Optional[int]) -> str:
    base_window: Sequence[Row] = result["base_window"]  # type: ignore[assignment]
    backup_window: Sequence[Row] = result["backup_window"]  # type: ignore[assignment]
    base_by_tow: Dict[int, Row] = result["base_by_tow"]  # type: ignore[assignment]
    backup_by_tow: Dict[int, Row] = result["backup_by_tow"]  # type: ignore[assignment]
    common_tows: Sequence[int] = result["common_tows"]  # type: ignore[assignment]
    overlap_rows: Sequence[Dict[str, object]] = result["overlap_rows"]  # type: ignore[assignment]
    ratios: Sequence[float] = result["ratios"]  # type: ignore[assignment]
    first_div: Optional[int] = result["first_divergence"]  # type: ignore[assignment]
    first_below_half: Optional[int] = result["first_below_half"]  # type: ignore[assignment]
    first_below_one: Optional[int] = result["first_below_one"]  # type: ignore[assignment]
    held_disjoint: Optional[int] = result["held_disjoint"]  # type: ignore[assignment]
    accepted_after_disjoint: Optional[int] = result["accepted_after_disjoint"]  # type: ignore[assignment]
    nonrecovering_advance: Optional[Tuple[int, int]] = result["nonrecovering_advance"]  # type: ignore[assignment]
    base_fix: Tuple[int, int, float] = result["base_fix"]  # type: ignore[assignment]
    backup_fix: Tuple[int, int, float] = result["backup_fix"]  # type: ignore[assignment]
    backup_events: Sequence[Row] = result["backup_events"]  # type: ignore[assignment]
    base_points: Sequence[Dict[str, str]] = result["base_points"]  # type: ignore[assignment]
    backup_points: Sequence[Dict[str, str]] = result["backup_points"]  # type: ignore[assignment]

    lines: List[str] = [
        "# Lineage divergence diff",
        "",
        f"Window: {format_tow(window_start)} to {format_tow(window_end)}",
        f"Held pair source column: `{HELD_KEYS_COL}`",
        "",
        "## Window summary",
        "",
        f"- base epochs in window: {len(base_window)}",
        f"- backup epochs in window: {len(backup_window)}",
        f"- common epochs compared: {len(common_tows)}",
        f"- base fix rate: {base_fix[0]}/{base_fix[1]} ({base_fix[2] * 100.0:.2f}%)",
        f"- backup fix rate: {backup_fix[0]}/{backup_fix[1]} ({backup_fix[2] * 100.0:.2f}%)",
        f"- backup-use count: {len(backup_events)}",
        "",
        "## First divergence",
        "",
    ]

    if first_div is None:
        lines.append("none")
    else:
        lines.append(f"- first held_pair_keys divergence: {format_tow(first_div)}")
        headers = ["run", "tow"] + DETAIL_COLUMNS + ["held_pair_keys"]
        rows = []
        for run_name, by_tow in (("base", base_by_tow), ("backup", backup_by_tow)):
            row = by_tow[first_div]
            rows.append([run_name, format_tow(first_div)] + describe_run_at(row) + [one_line_keys(row[HELD_KEYS_COL])])
        lines.extend(make_table(headers, rows))

    min_ratio = min(ratios) if ratios else 0.0
    mean_ratio = statistics.fmean(ratios) if ratios else 0.0
    median_ratio = statistics.median(ratios) if ratios else 0.0
    lines.extend(
        [
            "",
            "## Overlap trajectory",
            "",
            f"- min overlap ratio: {min_ratio:.3f}",
            f"- mean overlap ratio: {mean_ratio:.3f}",
            f"- median overlap ratio: {median_ratio:.3f}",
            f"- first overlap < 1.0: {format_tow(first_below_one)}",
            f"- first overlap < 0.5: {format_tow(first_below_half)}",
        ]
    )
    if nonrecovering_advance is None:
        lines.append("- first lineage advance without recovery to >=0.5: none")
    else:
        advance_tow, low_start = nonrecovering_advance
        lines.append(
            f"- first lineage advance without recovery to >=0.5: {format_tow(advance_tow)} "
            f"(low stretch start: {format_tow(low_start)})"
        )
    if overlap_rows:
        lowest = sorted(overlap_rows, key=lambda item: (float(item["ratio"]), int(item["tow"])))[:5]
        lines.append("- lowest overlap epochs:")
        for item in lowest:
            lines.append(
                f"  - {format_tow(int(item['tow']))}: ratio={float(item['ratio']):.3f}, "
                f"base={item['base_count']}, backup={item['backup_count']}, common={item['intersection_count']}"
            )

    lines.extend(["", "## Lineage ID trajectory", ""])
    for label, points in (("base", base_points), ("backup", backup_points)):
        lines.append(f"### {label}")
        if not points:
            lines.append("none")
        else:
            for point in points:
                lines.append(
                    f"- ({point['tow']}, {point['id']}, {point['origin_tow']}, "
                    f"{point['source']}, {point['path']}, {point['overlap_ratio']})"
                )
        lines.append("")

    lines.extend(["## Backup-use timeline", ""])
    if not backup_events:
        lines.append("none")
    else:
        timeline_rows = []
        for row in backup_events:
            timeline_rows.append(
                [
                    format_tow(int(row["_tow_key"])),
                    row["accepted_fix_source"] or "none",
                    row["accepted_candidate_path"] or "none",
                    one_line_keys(row["accepted_overwrite_before_pair_keys"]),
                    one_line_keys(row["accepted_overwrite_after_pair_keys"]),
                    row["accepted_overwrite_lineage_id"] or "none",
                ]
            )
        lines.extend(
            make_table(
                ["tow", "source", "path", "before pair keys", "after pair keys", "lineage ID"],
                timeline_rows,
            )
        )

    lines.extend(["", "## Disjoint point", ""])
    lines.append(f"- first held_pair_keys disjoint point: {format_tow(held_disjoint)}")
    if held_disjoint is not None:
        lines.extend(
            make_table(
                ["run", "tow", "source", "path", "held_pair_keys"],
                [
                    [
                        "base",
                        format_tow(held_disjoint),
                        base_by_tow[held_disjoint]["accepted_fix_source"] or "none",
                        base_by_tow[held_disjoint]["accepted_candidate_path"] or "none",
                        one_line_keys(base_by_tow[held_disjoint][HELD_KEYS_COL]),
                    ],
                    [
                        "backup",
                        format_tow(held_disjoint),
                        backup_by_tow[held_disjoint]["accepted_fix_source"] or "none",
                        backup_by_tow[held_disjoint]["accepted_candidate_path"] or "none",
                        one_line_keys(backup_by_tow[held_disjoint][HELD_KEYS_COL]),
                    ],
                ],
            )
        )
    lines.append(f"- first accepted_overwrite_after_pair_keys disjoint point: {format_tow(accepted_after_disjoint)}")
    if accepted_after_disjoint is not None:
        lines.extend(
            make_table(
                ["run", "tow", "source", "path", "accepted_overwrite_after_pair_keys"],
                [
                    [
                        "base",
                        format_tow(accepted_after_disjoint),
                        base_by_tow[accepted_after_disjoint]["accepted_fix_source"] or "none",
                        base_by_tow[accepted_after_disjoint]["accepted_candidate_path"] or "none",
                        one_line_keys(base_by_tow[accepted_after_disjoint]["accepted_overwrite_after_pair_keys"]),
                    ],
                    [
                        "backup",
                        format_tow(accepted_after_disjoint),
                        backup_by_tow[accepted_after_disjoint]["accepted_fix_source"] or "none",
                        backup_by_tow[accepted_after_disjoint]["accepted_candidate_path"] or "none",
                        one_line_keys(backup_by_tow[accepted_after_disjoint]["accepted_overwrite_after_pair_keys"]),
                    ],
                ],
            )
        )

    lines.extend(["", "## Decisive branch candidates", ""])
    candidates = decisive_candidates(
        first_div,
        first_below_half,
        nonrecovering_advance,
        accepted_after_disjoint,
        base_by_tow,
        backup_by_tow,
    )
    lines.extend(candidates)
    lines.append("")
    return "\n".join(lines)


def synthetic_rows() -> Tuple[List[Row], List[Row]]:
    def row(tow: str, status: str, held: str, after: str, lineage: str, backup_used: str = "0") -> Row:
        data = {col: "" for col in REQUIRED_COLUMNS}
        data.update(
            {
                "gps_tow": tow,
                "final_status": status,
                "accepted_fix_source": "lambda",
                "accepted_candidate_path": "full",
                "accepted_overwrite_before_pair_keys": held,
                "accepted_overwrite_after_pair_keys": after,
                "accepted_overlap_ratio": "1.0",
                "accepted_overwrite_lineage_id": lineage,
                "accepted_overwrite_lineage_origin_tow": "0.0",
                "accepted_preserved_prior_hold_state": "0",
                "hold_used_preserved_backup": backup_used,
                HELD_KEYS_COL: held,
                "_tow_key": str(parse_tow_key(tow)),
            }
        )
        return data

    base = [
        row("0.0", "4", "A|B", "A|B", "1"),
        row("0.2", "4", "A|B", "A|B", "1"),
        row("0.4", "4", "A|B", "A|B", "1"),
        row("0.6", "4", "A|B", "A|B", "1"),
        row("0.8", "4", "A|B", "A|B", "1"),
        row("1.0", "3", "A|B", "A|B", "1"),
        row("1.2", "4", "A|B", "A|B", "2"),
        row("1.4", "4", "A|B", "A|B", "2"),
    ]
    backup = [
        row("0.0", "4", "A|B", "A|B", "1"),
        row("0.2", "4", "A|C", "A|C", "1", "1"),
        row("0.4", "4", "C|D", "A|B", "1"),
        row("0.6", "4", "C|D", "A|B", "2"),
        row("0.8", "4", "A|B", "A|B", "2"),
        row("1.0", "3", "A|B", "A|B", "2"),
        row("1.2", "4", "A|B", "C|D", "2"),
        row("1.4", "4", "A|B", "A|B", "2"),
    ]
    return base, backup


def run_self_check() -> int:
    base, backup = synthetic_rows()
    result = analyze_rows(base, backup, None, None)
    overlap_rows: Sequence[Dict[str, object]] = result["overlap_rows"]  # type: ignore[assignment]
    by_tow = {format_tow(int(item["tow"])): float(item["ratio"]) for item in overlap_rows}
    assert by_tow["0.000"] == 1.0, by_tow
    assert by_tow["0.200"] == 0.5, by_tow
    assert by_tow["0.400"] == 0.0, by_tow
    assert result["first_divergence"] == parse_tow_key("0.2"), result["first_divergence"]
    assert result["held_disjoint"] == parse_tow_key("0.4"), result["held_disjoint"]
    assert result["accepted_after_disjoint"] == parse_tow_key("1.2"), result["accepted_after_disjoint"]
    assert result["nonrecovering_advance"] == (parse_tow_key("0.6"), parse_tow_key("0.4")), result["nonrecovering_advance"]
    print("self-check passed")
    return 0


def parse_args(argv: Optional[Sequence[str]] = None) -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--base-csv")
    parser.add_argument("--backup-csv")
    parser.add_argument("--out-md")
    parser.add_argument("--window-start", default="177050")
    parser.add_argument("--window-end", default="177770")
    parser.add_argument("--self-check", action="store_true")
    return parser.parse_args(argv)


def main(argv: Optional[Sequence[str]] = None) -> int:
    args = parse_args(argv)
    if args.self_check:
        return run_self_check()

    missing_args = [name for name in ("base_csv", "backup_csv", "out_md") if getattr(args, name) is None]
    if missing_args:
        print(f"missing required arguments: {', '.join('--' + name.replace('_', '-') for name in missing_args)}", file=sys.stderr)
        return 2

    window_start = parse_tow_key(args.window_start) if args.window_start is not None else None
    window_end = parse_tow_key(args.window_end) if args.window_end is not None else None
    base_rows = read_csv_rows(args.base_csv)
    backup_rows = read_csv_rows(args.backup_csv)
    result = analyze_rows(base_rows, backup_rows, window_start, window_end)
    markdown = render_markdown(result, window_start, window_end)

    parent = os.path.dirname(args.out_md)
    if parent:
        os.makedirs(parent, exist_ok=True)
    with open(args.out_md, "w", encoding="utf-8") as handle:
        handle.write(markdown)
        handle.write("\n")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
