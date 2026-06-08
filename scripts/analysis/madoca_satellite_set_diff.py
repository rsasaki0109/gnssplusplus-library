#!/usr/bin/env python3
"""Compare MADOCALIB $SAT satellite sets with native PPP correction logs."""

from __future__ import annotations

import argparse
import csv
import json
import math
from collections import Counter, defaultdict
from decimal import Decimal, InvalidOperation, ROUND_HALF_UP
from pathlib import Path
from typing import Any, Dict, Iterable, List, Optional, Sequence, Set, Tuple


EpochKey = Tuple[int, int]
Row = Dict[str, str]
SatelliteSets = Dict[EpochKey, Set[str]]
BridgeFrequencySets = Dict[Tuple[str, int], Set[EpochKey]]

BRIDGE_SAT_COLUMNS = 17

CORRECTION_REQUIRED_COLUMNS = [
    "week",
    "tow",
    "sat",
    "stec_tecu",
    "iono_m",
    "ionosphere_estimation_constraint",
    "elevation_deg",
    "valid_after_corrections",
]

RESIDUAL_REQUIRED_COLUMNS = [
    "week",
    "tow",
    "iteration",
    "sat",
    "row_type",
    "residual_m",
]


def tow_to_millis(value: str) -> int:
    try:
        tow = Decimal(str(value))
    except InvalidOperation as exc:
        raise ValueError(f"invalid tow value: {value}") from exc
    return int((tow * Decimal("1000")).to_integral_value(rounding=ROUND_HALF_UP))


def epoch_key_from_values(week: str, tow: str) -> EpochKey:
    return (int(float(week)), tow_to_millis(tow))


def epoch_key(row: Row) -> EpochKey:
    return epoch_key_from_values(row["week"], row["tow"])


def format_epoch(key: EpochKey) -> str:
    return f"{key[0]}:{key[1] / 1000.0:.3f}"


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


def truthy(value: str) -> bool:
    return str(value).strip().lower() in {"1", "true", "yes", "y"}


def constellation(sat: str) -> str:
    return sat[:1] if sat else ""


def sorted_epoch_keys(keys: Iterable[EpochKey]) -> List[EpochKey]:
    return sorted(keys, key=lambda item: (item[0], item[1]))


def read_bridge_stat(path: Path) -> Dict[str, Any]:
    valid_sets: SatelliteSets = defaultdict(set)
    all_sets: SatelliteSets = defaultdict(set)
    valid_frequency_sets: BridgeFrequencySets = defaultdict(set)
    sat_rows = 0
    malformed_rows = 0
    valid_frequency_rows = 0

    with path.open(newline="", encoding="utf-8") as handle:
        reader = csv.reader(handle)
        for parts in reader:
            if not parts or parts[0] != "$SAT":
                continue
            sat_rows += 1
            if len(parts) < BRIDGE_SAT_COLUMNS:
                malformed_rows += 1
                continue
            key = epoch_key_from_values(parts[1], parts[2])
            sat = parts[3]
            all_sets[key].add(sat)
            if truthy(parts[9]):
                valid_frequency_rows += 1
                valid_sets[key].add(sat)
                valid_frequency_sets[(sat, to_int(parts[4]))].add(key)

    return {
        "path": str(path),
        "valid_sets": dict(valid_sets),
        "all_sets": dict(all_sets),
        "valid_frequency_sets": dict(valid_frequency_sets),
        "sat_rows": sat_rows,
        "malformed_rows": malformed_rows,
        "valid_frequency_rows": valid_frequency_rows,
    }


def read_csv_rows(path: Path, required_columns: Sequence[str]) -> List[Row]:
    with path.open(newline="", encoding="utf-8") as handle:
        reader = csv.DictReader(handle)
        if reader.fieldnames is None:
            raise ValueError(f"{path}: missing CSV header")
        missing = [column for column in required_columns if column not in reader.fieldnames]
        if missing:
            raise ValueError(f"{path}: missing required columns: {', '.join(missing)}")
        return [dict(row) for row in reader]


def native_valid_sets(
    correction_rows: Sequence[Row],
    *,
    require_carrier_phase: bool = False,
) -> SatelliteSets:
    sets: SatelliteSets = defaultdict(set)
    for row in correction_rows:
        if not truthy(row.get("valid_after_corrections", "0")):
            continue
        if require_carrier_phase and not truthy(row.get("has_carrier_phase", "0")):
            continue
        sets[epoch_key(row)].add(row["sat"])
    return dict(sets)


def rows_have_column(rows: Sequence[Row], column: str) -> bool:
    return bool(rows) and column in rows[0]


def relation_for_sat(
    key: EpochKey,
    sat: str,
    bridge_sets: SatelliteSets,
    native_sets: SatelliteSets,
) -> Optional[str]:
    in_bridge = sat in bridge_sets.get(key, set())
    in_native = sat in native_sets.get(key, set())
    if in_bridge and in_native:
        return "common"
    if in_native:
        return "native_only"
    if in_bridge:
        return "bridge_only"
    return None


def compare_satellite_sets(
    bridge_sets: SatelliteSets,
    native_sets: SatelliteSets,
    *,
    top_satellites: int,
) -> Dict[str, Any]:
    bridge_epochs = set(bridge_sets)
    native_epochs = set(native_sets)
    common_epochs = sorted_epoch_keys(bridge_epochs & native_epochs)
    bridge_only_epochs = sorted_epoch_keys(bridge_epochs - native_epochs)
    native_only_epochs = sorted_epoch_keys(native_epochs - bridge_epochs)

    per_epoch: List[Dict[str, Any]] = []
    bridge_only_counter: Counter[str] = Counter()
    native_only_counter: Counter[str] = Counter()
    common_satellite_counter: Counter[str] = Counter()
    bridge_counts: List[int] = []
    native_counts: List[int] = []
    common_counts: List[int] = []

    for key in common_epochs:
        bridge = bridge_sets.get(key, set())
        native = native_sets.get(key, set())
        common = bridge & native
        bridge_only = bridge - native
        native_only = native - bridge
        bridge_counts.append(len(bridge))
        native_counts.append(len(native))
        common_counts.append(len(common))
        bridge_only_counter.update(bridge_only)
        native_only_counter.update(native_only)
        common_satellite_counter.update(common)
        per_epoch.append(
            {
                "week": key[0],
                "tow": key[1] / 1000.0,
                "bridge_count": len(bridge),
                "native_count": len(native),
                "common_count": len(common),
                "bridge_only_count": len(bridge_only),
                "native_only_count": len(native_only),
                "bridge_only": sorted(bridge_only),
                "native_only": sorted(native_only),
            }
        )

    def mean(values: Sequence[int]) -> float:
        return sum(values) / len(values) if values else 0.0

    def counter_items(counter: Counter[str]) -> List[Dict[str, Any]]:
        return [
            {"sat": sat, "epochs": count}
            for sat, count in sorted(counter.items(), key=lambda item: (-item[1], item[0]))[:top_satellites]
        ]

    return {
        "bridge_epochs": len(bridge_epochs),
        "native_epochs": len(native_epochs),
        "common_epochs": len(common_epochs),
        "bridge_only_epochs": [format_epoch(key) for key in bridge_only_epochs],
        "native_only_epochs": [format_epoch(key) for key in native_only_epochs],
        "mean_bridge_satellites": mean(bridge_counts),
        "mean_native_satellites": mean(native_counts),
        "mean_common_satellites": mean(common_counts),
        "bridge_only_satellite_appearances": sum(bridge_only_counter.values()),
        "native_only_satellite_appearances": sum(native_only_counter.values()),
        "top_bridge_only_satellites": counter_items(bridge_only_counter),
        "top_native_only_satellites": counter_items(native_only_counter),
        "top_common_satellites": counter_items(common_satellite_counter),
        "per_epoch": per_epoch,
    }


def new_numeric_group() -> Dict[str, Any]:
    return {
        "rows": 0,
        "epochs": set(),
        "satellites": set(),
        "stec_rows": 0,
        "ionosphere_constraint_rows": 0,
        "phase_capable_rows": 0,
        "iono_nonzero_rows": 0,
        "frequency_index_counts": Counter(),
        "sum_abs_iono_m": 0.0,
        "sum_elevation_deg": 0.0,
        "min_elevation_deg": None,
        "max_elevation_deg": None,
    }


def update_numeric_group(group: Dict[str, Any], row: Row) -> None:
    elevation = to_float(row.get("elevation_deg", "0"))
    iono = abs(to_float(row.get("iono_m", "0")))
    group["rows"] += 1
    group["epochs"].add(epoch_key(row))
    group["satellites"].add(row.get("sat", ""))
    if abs(to_float(row.get("stec_tecu", "0"))) > 0.0:
        group["stec_rows"] += 1
    if truthy(row.get("ionosphere_estimation_constraint", "0")):
        group["ionosphere_constraint_rows"] += 1
    if truthy(row.get("has_carrier_phase", "0")):
        group["phase_capable_rows"] += 1
    if "frequency_index" in row:
        group["frequency_index_counts"][str(to_int(row.get("frequency_index", "0")))] += 1
    if iono > 0.0:
        group["iono_nonzero_rows"] += 1
    group["sum_abs_iono_m"] += iono
    group["sum_elevation_deg"] += elevation
    if group["min_elevation_deg"] is None or elevation < group["min_elevation_deg"]:
        group["min_elevation_deg"] = elevation
    if group["max_elevation_deg"] is None or elevation > group["max_elevation_deg"]:
        group["max_elevation_deg"] = elevation


def finish_numeric_group(group: Dict[str, Any]) -> Dict[str, Any]:
    rows = max(group["rows"], 1)
    return {
        "rows": group["rows"],
        "epochs": len(group["epochs"]),
        "satellites": len(group["satellites"]),
        "stec_rows": group["stec_rows"],
        "ionosphere_constraint_rows": group["ionosphere_constraint_rows"],
        "phase_capable_rows": group["phase_capable_rows"],
        "iono_nonzero_rows": group["iono_nonzero_rows"],
        "frequency_index_counts": dict(sorted(
            group["frequency_index_counts"].items(),
            key=lambda item: (to_int(item[0]), item[0]),
        )),
        "mean_abs_iono_m": group["sum_abs_iono_m"] / rows,
        "mean_elevation_deg": group["sum_elevation_deg"] / rows,
        "min_elevation_deg": group["min_elevation_deg"] or 0.0,
        "max_elevation_deg": group["max_elevation_deg"] or 0.0,
    }


def summarize_native_corrections(
    correction_rows: Sequence[Row],
    bridge_sets: SatelliteSets,
    native_sets: SatelliteSets,
    *,
    top_satellites: int,
) -> Dict[str, Any]:
    common_epochs = set(bridge_sets) & set(native_sets)
    by_relation: Dict[str, Dict[str, Any]] = defaultdict(new_numeric_group)
    by_relation_constellation: Dict[Tuple[str, str], Dict[str, Any]] = defaultdict(new_numeric_group)
    by_relation_satellite: Dict[Tuple[str, str], Dict[str, Any]] = defaultdict(new_numeric_group)

    for row in correction_rows:
        key = epoch_key(row)
        if key not in common_epochs or not truthy(row.get("valid_after_corrections", "0")):
            continue
        relation = relation_for_sat(key, row["sat"], bridge_sets, native_sets)
        if relation not in {"common", "native_only"}:
            continue
        update_numeric_group(by_relation[relation], row)
        update_numeric_group(by_relation_constellation[(relation, constellation(row["sat"]))], row)
        update_numeric_group(by_relation_satellite[(relation, row["sat"])], row)

    by_satellite: Dict[str, List[Dict[str, Any]]] = defaultdict(list)
    for (relation, sat), group in by_relation_satellite.items():
        item = finish_numeric_group(group)
        item["sat"] = sat
        item["constellation"] = constellation(sat)
        by_satellite[relation].append(item)

    return {
        "by_relation": {
            relation: finish_numeric_group(group)
            for relation, group in sorted(by_relation.items())
        },
        "by_relation_constellation": [
            {
                "relation": relation,
                "constellation": system,
                **finish_numeric_group(group),
            }
            for (relation, system), group in sorted(by_relation_constellation.items())
        ],
        "top_satellites_by_relation": {
            relation: sorted(
                items,
                key=lambda item: (-item["rows"], item["stec_rows"], item["sat"]),
            )[:top_satellites]
            for relation, items in sorted(by_satellite.items())
        },
    }


def select_final_iteration_rows(rows: Sequence[Row]) -> List[Row]:
    max_iteration_by_epoch: Dict[EpochKey, int] = {}
    for row in rows:
        key = epoch_key(row)
        max_iteration_by_epoch[key] = max(max_iteration_by_epoch.get(key, -1), to_int(row.get("iteration", "0")))
    return [
        row
        for row in rows
        if to_int(row.get("iteration", "0")) == max_iteration_by_epoch[epoch_key(row)]
    ]


def new_residual_group() -> Dict[str, Any]:
    return {
        "rows": 0,
        "epochs": set(),
        "satellites": set(),
        "sum_sq_residual_m2": 0.0,
        "max_abs_residual_m": 0.0,
        "max_abs_week": 0,
        "max_abs_tow": 0.0,
        "max_abs_sat": "",
    }


def update_residual_group(group: Dict[str, Any], row: Row) -> None:
    residual_abs = abs(to_float(row.get("residual_m", "0")))
    key = epoch_key(row)
    group["rows"] += 1
    group["epochs"].add(key)
    group["satellites"].add(row.get("sat", ""))
    group["sum_sq_residual_m2"] += residual_abs * residual_abs
    if residual_abs > group["max_abs_residual_m"]:
        group["max_abs_residual_m"] = residual_abs
        group["max_abs_week"] = key[0]
        group["max_abs_tow"] = key[1] / 1000.0
        group["max_abs_sat"] = row.get("sat", "")


def finish_residual_group(group: Dict[str, Any]) -> Dict[str, Any]:
    rows = max(group["rows"], 1)
    return {
        "rows": group["rows"],
        "epochs": len(group["epochs"]),
        "satellites": len(group["satellites"]),
        "rms_residual_m": math.sqrt(group["sum_sq_residual_m2"] / rows),
        "max_abs_residual_m": group["max_abs_residual_m"],
        "max_abs_week": group["max_abs_week"],
        "max_abs_tow": group["max_abs_tow"],
        "max_abs_sat": group["max_abs_sat"],
    }


def summarize_residuals(
    residual_rows: Sequence[Row],
    bridge_sets: SatelliteSets,
    native_sets: SatelliteSets,
    *,
    top_satellites: int,
) -> Dict[str, Any]:
    common_epochs = set(bridge_sets) & set(native_sets)
    final_rows = select_final_iteration_rows(residual_rows)
    groups: Dict[Tuple[str, str, str], Dict[str, Any]] = defaultdict(new_residual_group)
    satellite_groups: Dict[Tuple[str, str, str], Dict[str, Any]] = defaultdict(new_residual_group)
    used_rows = 0
    skipped_rows = 0

    for row in final_rows:
        key = epoch_key(row)
        if key not in common_epochs:
            skipped_rows += 1
            continue
        relation = relation_for_sat(key, row["sat"], bridge_sets, native_sets)
        if relation not in {"common", "native_only"}:
            skipped_rows += 1
            continue
        row_type = row.get("row_type", "")
        system = constellation(row["sat"])
        update_residual_group(groups[(relation, row_type, system)], row)
        update_residual_group(satellite_groups[(relation, row_type, row["sat"])], row)
        used_rows += 1

    top_by_relation_type: Dict[str, Dict[str, List[Dict[str, Any]]]] = defaultdict(lambda: defaultdict(list))
    for (relation, row_type, sat), group in satellite_groups.items():
        item = finish_residual_group(group)
        item["sat"] = sat
        item["constellation"] = constellation(sat)
        top_by_relation_type[relation][row_type].append(item)

    return {
        "input_rows": len(residual_rows),
        "final_iteration_rows": len(final_rows),
        "classified_rows": used_rows,
        "skipped_rows": skipped_rows,
        "by_relation_row_type_constellation": [
            {
                "relation": relation,
                "row_type": row_type,
                "constellation": system,
                **finish_residual_group(group),
            }
            for (relation, row_type, system), group in sorted(groups.items())
        ],
        "top_satellites_by_relation_type": {
            relation: {
                row_type: sorted(
                    items,
                    key=lambda item: (-item["max_abs_residual_m"], -item["rms_residual_m"], item["sat"]),
                )[:top_satellites]
                for row_type, items in sorted(by_type.items())
            }
            for relation, by_type in sorted(top_by_relation_type.items())
        },
    }


def phase_identity(row: Row) -> str:
    primary_code = row.get("primary_observation_code", "")
    secondary_code = row.get("secondary_observation_code", "")
    return f"f{to_int(row.get('frequency_index', '0'))}:{primary_code}/{secondary_code}"


def phase_candidate_sort_key(row: Row) -> Tuple[int, int, int, int]:
    return (
        int(float(row.get("week", "0"))),
        tow_to_millis(row.get("tow", "0")),
        to_int(row.get("iteration", "0")),
        to_int(row.get("row_index", "0")),
    )


def phase_candidate_snapshot(row: Row) -> Dict[str, Any]:
    residual = to_float(row.get("residual_m", "0"))
    return {
        "week": int(float(row.get("week", "0"))),
        "tow": tow_to_millis(row.get("tow", "0")) / 1000.0,
        "iteration": to_int(row.get("iteration", "0")),
        "residual_m": residual,
        "abs_residual_m": abs(residual),
        "phase_limit_m": to_float(row.get("phase_limit_m", "0")),
        "phase_skip_reason": row.get("phase_skip_reason", ""),
        "ambiguity_lock_count": to_int(row.get("ambiguity_lock_count", "0")),
        "required_lock_count": to_int(row.get("required_lock_count", "0")),
    }


def new_phase_candidate_group() -> Dict[str, Any]:
    return {
        "sat": "",
        "frequency_index": 0,
        "identity": "",
        "candidate_rows": 0,
        "candidate_epochs": set(),
        "ready_candidate_rows": 0,
        "ready_candidate_epochs": set(),
        "accepted_phase_rows": 0,
        "accepted_phase_epochs": set(),
        "skip_reasons": Counter(),
        "first_candidate_row": None,
        "min_candidate_row": None,
        "first_ready_row": None,
        "min_ready_row": None,
        "first_accepted_row": None,
        "min_accepted_row": None,
    }


def update_first_min_phase_candidate(group: Dict[str, Any], row: Row) -> None:
    first_candidate = group["first_candidate_row"]
    if first_candidate is None or phase_candidate_sort_key(row) < phase_candidate_sort_key(first_candidate):
        group["first_candidate_row"] = row
    min_candidate = group["min_candidate_row"]
    row_abs = abs(to_float(row.get("residual_m", "0")))
    min_abs = (
        abs(to_float(min_candidate.get("residual_m", "0")))
        if min_candidate is not None
        else math.inf
    )
    if min_candidate is None or row_abs < min_abs:
        group["min_candidate_row"] = row


def update_first_min_ready_candidate(group: Dict[str, Any], row: Row) -> None:
    if not truthy(row.get("phase_ready", "0")):
        return
    first_ready = group["first_ready_row"]
    if first_ready is None or phase_candidate_sort_key(row) < phase_candidate_sort_key(first_ready):
        group["first_ready_row"] = row
    min_ready = group["min_ready_row"]
    row_abs = abs(to_float(row.get("residual_m", "0")))
    min_abs = abs(to_float(min_ready.get("residual_m", "0"))) if min_ready is not None else math.inf
    if min_ready is None or row_abs < min_abs:
        group["min_ready_row"] = row


def update_first_min_accepted_phase(group: Dict[str, Any], row: Row) -> None:
    first_accepted = group["first_accepted_row"]
    if first_accepted is None or phase_candidate_sort_key(row) < phase_candidate_sort_key(first_accepted):
        group["first_accepted_row"] = row
    min_accepted = group["min_accepted_row"]
    row_abs = abs(to_float(row.get("residual_m", "0")))
    min_abs = (
        abs(to_float(min_accepted.get("residual_m", "0")))
        if min_accepted is not None
        else math.inf
    )
    if min_accepted is None or row_abs < min_abs:
        group["min_accepted_row"] = row


def summarize_phase_candidates(
    residual_rows: Sequence[Row],
    *,
    top_satellites: int,
) -> Dict[str, Any]:
    groups: Dict[Tuple[str, int, str], Dict[str, Any]] = defaultdict(new_phase_candidate_group)
    total_candidate_rows = 0
    total_ready_candidate_rows = 0
    total_accepted_phase_rows = 0

    for row in residual_rows:
        row_type = row.get("row_type", "")
        if row_type not in {"phase_candidate", "phase"}:
            continue
        sat = row.get("sat", "")
        frequency_index = to_int(row.get("frequency_index", "0"))
        identity = phase_identity(row)
        group = groups[(sat, frequency_index, identity)]
        group["sat"] = sat
        group["frequency_index"] = frequency_index
        group["identity"] = identity
        key = epoch_key(row)
        if row_type == "phase_candidate":
            total_candidate_rows += 1
            group["candidate_rows"] += 1
            group["candidate_epochs"].add(key)
            reason = row.get("phase_skip_reason", "") or "unknown"
            group["skip_reasons"][reason] += 1
            update_first_min_phase_candidate(group, row)
            if truthy(row.get("phase_ready", "0")):
                total_ready_candidate_rows += 1
                group["ready_candidate_rows"] += 1
                group["ready_candidate_epochs"].add(key)
            update_first_min_ready_candidate(group, row)
        elif row_type == "phase":
            total_accepted_phase_rows += 1
            group["accepted_phase_rows"] += 1
            group["accepted_phase_epochs"].add(key)
            update_first_min_accepted_phase(group, row)

    items: List[Dict[str, Any]] = []
    for group in groups.values():
        first_candidate = group["first_candidate_row"]
        min_candidate = group["min_candidate_row"]
        first_ready = group["first_ready_row"]
        min_ready = group["min_ready_row"]
        first_accepted = group["first_accepted_row"]
        min_accepted = group["min_accepted_row"]
        items.append(
            {
                "sat": group["sat"],
                "constellation": constellation(group["sat"]),
                "frequency_index": group["frequency_index"],
                "identity": group["identity"],
                "candidate_rows": group["candidate_rows"],
                "candidate_epochs": len(group["candidate_epochs"]),
                "ready_candidate_rows": group["ready_candidate_rows"],
                "ready_candidate_epochs": len(group["ready_candidate_epochs"]),
                "accepted_phase_rows": group["accepted_phase_rows"],
                "accepted_phase_epochs": len(group["accepted_phase_epochs"]),
                "skip_reasons": [
                    {"reason": reason, "rows": rows}
                    for reason, rows in group["skip_reasons"].most_common()
                ],
                "first_candidate": phase_candidate_snapshot(first_candidate)
                if first_candidate is not None
                else None,
                "min_candidate": phase_candidate_snapshot(min_candidate)
                if min_candidate is not None
                else None,
                "first_ready_candidate": phase_candidate_snapshot(first_ready)
                if first_ready is not None
                else None,
                "min_ready_candidate": phase_candidate_snapshot(min_ready)
                if min_ready is not None
                else None,
                "first_accepted_phase": phase_candidate_snapshot(first_accepted)
                if first_accepted is not None
                else None,
                "min_accepted_phase": phase_candidate_snapshot(min_accepted)
                if min_accepted is not None
                else None,
            }
        )

    return {
        "total_phase_candidate_rows": total_candidate_rows,
        "total_ready_phase_candidate_rows": total_ready_candidate_rows,
        "total_accepted_phase_rows": total_accepted_phase_rows,
        "top_satellites": sorted(
            items,
            key=lambda item: (
                -item["candidate_rows"],
                -item["ready_candidate_rows"],
                -item["accepted_phase_rows"],
                item["sat"],
                item["frequency_index"],
                item["identity"],
            ),
        )[:top_satellites],
    }


def summarize_phase_frequency_overlap(
    residual_rows: Sequence[Row],
    bridge_frequency_sets: BridgeFrequencySets,
    *,
    comparison_epochs: Optional[Set[EpochKey]],
    top_satellites: int,
) -> Dict[str, Any]:
    final_phase_rows = [
        row
        for row in select_final_iteration_rows(residual_rows)
        if row.get("row_type", "") == "phase"
    ]

    bridge_by_satellite: Dict[str, Dict[str, Any]] = defaultdict(
        lambda: {
            "epochs": set(),
            "frequency_epoch_counts": Counter(),
            "frequency_rows": 0,
        }
    )
    total_bridge_valid_frequency_rows = 0
    for (sat, frequency), epochs in bridge_frequency_sets.items():
        filtered_epochs = set(epochs)
        if comparison_epochs is not None:
            filtered_epochs &= comparison_epochs
        if not filtered_epochs:
            continue
        group = bridge_by_satellite[sat]
        group["epochs"].update(filtered_epochs)
        group["frequency_epoch_counts"][str(frequency)] += len(filtered_epochs)
        group["frequency_rows"] += len(filtered_epochs)
        total_bridge_valid_frequency_rows += len(filtered_epochs)

    native_by_satellite: Dict[str, Dict[str, Any]] = defaultdict(
        lambda: {
            "epochs": set(),
            "identity_counts": Counter(),
            "identity_epochs": defaultdict(set),
            "rows": 0,
        }
    )
    for row in final_phase_rows:
        sat = row.get("sat", "")
        identity = phase_identity(row)
        key = epoch_key(row)
        group = native_by_satellite[sat]
        group["epochs"].add(key)
        group["identity_counts"][identity] += 1
        group["identity_epochs"][identity].add(key)
        group["rows"] += 1

    satellites = sorted(set(bridge_by_satellite) | set(native_by_satellite))
    items: List[Dict[str, Any]] = []
    for sat in satellites:
        bridge_group = bridge_by_satellite.get(sat, {})
        native_group = native_by_satellite.get(sat, {})
        bridge_epochs = len(bridge_group.get("epochs", set()))
        bridge_frequency_rows = int(bridge_group.get("frequency_rows", 0))
        native_epochs = len(native_group.get("epochs", set()))
        native_rows = int(native_group.get("rows", 0))
        identity_counts: Counter[str] = native_group.get("identity_counts", Counter())
        identity_epochs = native_group.get("identity_epochs", {})
        item = {
            "sat": sat,
            "constellation": constellation(sat),
            "bridge_valid_epochs": bridge_epochs,
            "bridge_valid_frequency_rows": bridge_frequency_rows,
            "bridge_frequency_epoch_counts": dict(
                sorted(
                    bridge_group.get("frequency_epoch_counts", Counter()).items(),
                    key=lambda entry: (to_int(entry[0]), entry[0]),
                )
            ),
            "native_phase_epochs": native_epochs,
            "native_final_phase_rows": native_rows,
            "native_phase_identities": [
                {
                    "identity": identity,
                    "rows": rows,
                    "epochs": len(identity_epochs.get(identity, set())),
                }
                for identity, rows in sorted(
                    identity_counts.items(),
                    key=lambda entry: (-entry[1], entry[0]),
                )
            ],
        }
        item["mismatch_score"] = (
            abs(item["bridge_valid_epochs"] - item["native_phase_epochs"])
            + abs(item["bridge_valid_frequency_rows"] - item["native_final_phase_rows"])
        )
        items.append(item)

    return {
        "bridge_satellites": len(bridge_by_satellite),
        "native_phase_satellites": len(native_by_satellite),
        "total_bridge_valid_frequency_rows": total_bridge_valid_frequency_rows,
        "total_native_final_phase_rows": len(final_phase_rows),
        "top_satellites": sorted(
            items,
            key=lambda item: (
                -item["mismatch_score"],
                -item["native_final_phase_rows"],
                item["sat"],
            ),
        )[:top_satellites],
    }


def correction_identity_key(row: Row) -> Tuple[EpochKey, str, int, str, str]:
    return (
        epoch_key(row),
        row.get("sat", ""),
        to_int(row.get("frequency_index", "0")),
        row.get("primary_observation_code", ""),
        row.get("secondary_observation_code", ""),
    )


def new_initial_phase_admission_group() -> Dict[str, Any]:
    return {
        "rows": 0,
        "correction_matches": 0,
        "constellation_counts": Counter(),
        "frequency_index_counts": Counter(),
        "skip_reasons": Counter(),
        "ssr_available_rows": 0,
        "orbit_clock_applied_rows": 0,
        "valid_after_corrections_rows": 0,
        "stec_nonzero_rows": 0,
        "iono_nonzero_rows": 0,
        "ionosphere_constraint_rows": 0,
        "phase_bias_nonzero_rows": 0,
        "atmos_token_rows": 0,
        "sum_abs_residual_m": 0.0,
        "min_abs_residual_m": None,
        "max_abs_residual_m": 0.0,
        "sum_elevation_deg": 0.0,
        "elevation_rows": 0,
    }


def update_initial_phase_admission_group(
    group: Dict[str, Any],
    row: Row,
    correction_row: Optional[Row],
) -> None:
    residual_abs = abs(to_float(row.get("residual_m", "0")))
    group["rows"] += 1
    group["constellation_counts"][constellation(row.get("sat", ""))] += 1
    group["frequency_index_counts"][str(to_int(row.get("frequency_index", "0")))] += 1
    reason = row.get("phase_skip_reason", "")
    if reason:
        group["skip_reasons"][reason] += 1
    group["sum_abs_residual_m"] += residual_abs
    if group["min_abs_residual_m"] is None or residual_abs < group["min_abs_residual_m"]:
        group["min_abs_residual_m"] = residual_abs
    if residual_abs > group["max_abs_residual_m"]:
        group["max_abs_residual_m"] = residual_abs

    if correction_row is None:
        return
    group["correction_matches"] += 1
    if truthy(correction_row.get("ssr_available", "0")):
        group["ssr_available_rows"] += 1
    if truthy(correction_row.get("orbit_clock_applied", "0")):
        group["orbit_clock_applied_rows"] += 1
    if truthy(correction_row.get("valid_after_corrections", "0")):
        group["valid_after_corrections_rows"] += 1
    if abs(to_float(correction_row.get("stec_tecu", "0"))) > 0.0:
        group["stec_nonzero_rows"] += 1
    if abs(to_float(correction_row.get("iono_m", "0"))) > 0.0:
        group["iono_nonzero_rows"] += 1
    if truthy(correction_row.get("ionosphere_estimation_constraint", "0")):
        group["ionosphere_constraint_rows"] += 1
    if abs(to_float(correction_row.get("phase_bias_m", "0"))) > 0.0:
        group["phase_bias_nonzero_rows"] += 1
    if to_int(correction_row.get("atmos_token_count", "0")) > 0:
        group["atmos_token_rows"] += 1
    if "elevation_deg" in correction_row:
        group["sum_elevation_deg"] += to_float(correction_row.get("elevation_deg", "0"))
        group["elevation_rows"] += 1


def finish_initial_phase_admission_group(group: Dict[str, Any]) -> Dict[str, Any]:
    rows = max(group["rows"], 1)
    elevation_rows = max(group["elevation_rows"], 1)
    return {
        "rows": group["rows"],
        "correction_matches": group["correction_matches"],
        "constellation_counts": dict(sorted(group["constellation_counts"].items())),
        "frequency_index_counts": dict(
            sorted(group["frequency_index_counts"].items(), key=lambda item: (to_int(item[0]), item[0]))
        ),
        "skip_reasons": [
            {"reason": reason, "rows": rows}
            for reason, rows in group["skip_reasons"].most_common()
        ],
        "ssr_available_rows": group["ssr_available_rows"],
        "orbit_clock_applied_rows": group["orbit_clock_applied_rows"],
        "valid_after_corrections_rows": group["valid_after_corrections_rows"],
        "stec_nonzero_rows": group["stec_nonzero_rows"],
        "iono_nonzero_rows": group["iono_nonzero_rows"],
        "ionosphere_constraint_rows": group["ionosphere_constraint_rows"],
        "phase_bias_nonzero_rows": group["phase_bias_nonzero_rows"],
        "atmos_token_rows": group["atmos_token_rows"],
        "mean_abs_residual_m": group["sum_abs_residual_m"] / rows,
        "min_abs_residual_m": group["min_abs_residual_m"] or 0.0,
        "max_abs_residual_m": group["max_abs_residual_m"],
        "mean_elevation_deg": group["sum_elevation_deg"] / elevation_rows,
    }


def summarize_initial_phase_admission(
    residual_rows: Sequence[Row],
    correction_rows: Sequence[Row],
) -> Dict[str, Any]:
    phase_rows = [
        row
        for row in residual_rows
        if row.get("row_type", "") in {"phase", "phase_candidate"}
    ]
    if not phase_rows:
        return {
            "week": 0,
            "tow": 0.0,
            "iteration": 0,
            "rows": 0,
            "accepted_rows": 0,
            "skipped_rows": 0,
            "by_status": {},
        }

    first_key = min(
        (epoch_key(row), to_int(row.get("iteration", "0")))
        for row in phase_rows
    )
    initial_rows = [
        row
        for row in phase_rows
        if epoch_key(row) == first_key[0]
        and to_int(row.get("iteration", "0")) == first_key[1]
    ]
    correction_by_identity = {
        correction_identity_key(row): row
        for row in correction_rows
    }
    groups: Dict[str, Dict[str, Any]] = {
        "accepted": new_initial_phase_admission_group(),
        "skipped": new_initial_phase_admission_group(),
    }

    for row in initial_rows:
        status = "accepted" if row.get("row_type", "") == "phase" else "skipped"
        update_initial_phase_admission_group(
            groups[status],
            row,
            correction_by_identity.get(correction_identity_key(row)),
        )

    accepted_rows = groups["accepted"]["rows"]
    skipped_rows = groups["skipped"]["rows"]
    return {
        "week": first_key[0][0],
        "tow": first_key[0][1] / 1000.0,
        "iteration": first_key[1],
        "rows": len(initial_rows),
        "accepted_rows": accepted_rows,
        "skipped_rows": skipped_rows,
        "by_status": {
            status: finish_initial_phase_admission_group(group)
            for status, group in groups.items()
            if group["rows"] > 0
        },
    }


def build_report(args: argparse.Namespace) -> Dict[str, Any]:
    bridge = read_bridge_stat(args.bridge_stat)
    correction_rows = read_csv_rows(args.native_correction_log, CORRECTION_REQUIRED_COLUMNS)
    bridge_sets = bridge["valid_sets"]
    native_sets = native_valid_sets(correction_rows)

    report: Dict[str, Any] = {
        "bridge_stat": args.bridge_stat.as_posix(),
        "native_correction_log": args.native_correction_log.as_posix(),
        "bridge_summary": {
            "sat_rows": bridge["sat_rows"],
            "malformed_rows": bridge["malformed_rows"],
            "valid_frequency_rows": bridge["valid_frequency_rows"],
            "valid_epochs": len(bridge_sets),
        },
        "native_correction_rows": len(correction_rows),
        "satellite_set_comparison": compare_satellite_sets(
            bridge_sets,
            native_sets,
            top_satellites=args.top_satellites,
        ),
        "native_correction_summary": summarize_native_corrections(
            correction_rows,
            bridge_sets,
            native_sets,
            top_satellites=args.top_satellites,
        ),
    }

    if rows_have_column(correction_rows, "has_carrier_phase"):
        native_phase_sets = native_valid_sets(correction_rows, require_carrier_phase=True)
        report["satellite_set_comparison_phase_capable"] = compare_satellite_sets(
            bridge_sets,
            native_phase_sets,
            top_satellites=args.top_satellites,
        )

    if args.residual_log is not None:
        residual_rows = read_csv_rows(args.residual_log, RESIDUAL_REQUIRED_COLUMNS)
        report["native_residual_log"] = args.residual_log.as_posix()
        report["native_residual_summary"] = summarize_residuals(
            residual_rows,
            bridge_sets,
            native_sets,
            top_satellites=args.top_satellites,
        )
        report["phase_frequency_overlap"] = summarize_phase_frequency_overlap(
            residual_rows,
            bridge["valid_frequency_sets"],
            comparison_epochs=set(bridge_sets) & set(native_sets),
            top_satellites=args.top_satellites,
        )
        report["phase_candidate_summary"] = summarize_phase_candidates(
            residual_rows,
            top_satellites=args.top_satellites,
        )
        report["initial_phase_admission_summary"] = summarize_initial_phase_admission(
            residual_rows,
            correction_rows,
        )

    return report


def write_epoch_csv(path: Path, per_epoch: Sequence[Dict[str, Any]]) -> None:
    fieldnames = [
        "week",
        "tow",
        "bridge_count",
        "native_count",
        "common_count",
        "bridge_only_count",
        "native_only_count",
        "bridge_only",
        "native_only",
    ]
    with path.open("w", newline="", encoding="utf-8") as handle:
        writer = csv.DictWriter(handle, fieldnames=fieldnames)
        writer.writeheader()
        for row in per_epoch:
            output = dict(row)
            output["bridge_only"] = " ".join(row["bridge_only"])
            output["native_only"] = " ".join(row["native_only"])
            writer.writerow(output)


def print_satellite_set_block(label: str, sets: Dict[str, Any], *, top_epochs: int) -> None:
    print(f"{label}:")
    for key in [
        "bridge_epochs",
        "native_epochs",
        "common_epochs",
        "mean_bridge_satellites",
        "mean_native_satellites",
        "mean_common_satellites",
        "bridge_only_satellite_appearances",
        "native_only_satellite_appearances",
    ]:
        print(f"  {key}: {sets[key]}")
    print(f"  top_bridge_only_satellites: {sets['top_bridge_only_satellites']}")
    print(f"  top_native_only_satellites: {sets['top_native_only_satellites']}")
    print("  epoch_differences:")
    for row in sets["per_epoch"][:top_epochs]:
        print(
            "    "
            f"{row['week']}:{row['tow']:.3f} "
            f"bridge={row['bridge_count']} native={row['native_count']} common={row['common_count']} "
            f"bridge_only={row['bridge_only']} native_only={row['native_only']}"
        )


def print_report(report: Dict[str, Any], *, top_epochs: int) -> None:
    print_satellite_set_block(
        "satellite_sets",
        report["satellite_set_comparison"],
        top_epochs=top_epochs,
    )
    phase_sets = report.get("satellite_set_comparison_phase_capable")
    if phase_sets is not None:
        print_satellite_set_block(
            "satellite_sets_phase_capable",
            phase_sets,
            top_epochs=top_epochs,
        )

    phase_frequency = report.get("phase_frequency_overlap")
    if phase_frequency is not None:
        print("phase_frequency_overlap:")
        print(
            "  "
            f"bridge_freq_rows={phase_frequency['total_bridge_valid_frequency_rows']} "
            f"native_phase_rows={phase_frequency['total_native_final_phase_rows']}"
        )
        for item in phase_frequency["top_satellites"][:3]:
            identities = ", ".join(
                f"{identity['identity']}={identity['epochs']}ep"
                for identity in item.get("native_phase_identities", [])[:2]
            )
            print(
                "  "
                f"{item['sat']}: bridge_epochs={item['bridge_valid_epochs']} "
                f"native_epochs={item['native_phase_epochs']} identities={identities}"
            )

    phase_candidates = report.get("phase_candidate_summary")
    if phase_candidates is not None:
        print("phase_candidate_summary:")
        print(
            "  "
            f"candidates={phase_candidates['total_phase_candidate_rows']} "
            f"ready={phase_candidates['total_ready_phase_candidate_rows']} "
            f"accepted_phase={phase_candidates['total_accepted_phase_rows']}"
        )
        for item in phase_candidates["top_satellites"][:3]:
            reasons = ", ".join(
                f"{reason['reason']}={reason['rows']}"
                for reason in item.get("skip_reasons", [])[:2]
            )
            first_candidate = item.get("first_candidate")
            min_candidate = item.get("min_candidate")
            candidate_text = "min_candidate=none"
            if min_candidate is not None:
                candidate_text = (
                    f"min_candidate={min_candidate['residual_m']:.3f}/"
                    f"{min_candidate['phase_limit_m']:.3f}"
                    f"/{min_candidate['phase_skip_reason'] or 'accepted'}"
                )
            first_ready = item.get("first_ready_candidate")
            first_ready_text = "first_ready=none"
            if first_ready is not None:
                first_ready_text = (
                    f"first_ready={first_ready['residual_m']:.3f}/"
                    f"{first_ready['phase_limit_m']:.3f}"
                )
            first_accepted = item.get("first_accepted_phase")
            first_accepted_text = "first_accepted=none"
            if first_accepted is not None:
                first_accepted_text = f"first_accepted={first_accepted['residual_m']:.3f}"
            print(
                "  "
                f"{item['sat']} {item['identity']}: "
                f"cand={item['candidate_rows']} ready={item['ready_candidate_rows']} "
                f"accepted={item['accepted_phase_rows']} {candidate_text} "
                f"{first_ready_text} {first_accepted_text} reasons={reasons}"
            )

    initial_phase = report.get("initial_phase_admission_summary")
    if initial_phase is not None and initial_phase.get("rows", 0) > 0:
        print("initial_phase_admission_summary:")
        print(
            "  "
            f"epoch={initial_phase['week']}:{initial_phase['tow']:.3f} "
            f"iteration={initial_phase['iteration']} rows={initial_phase['rows']} "
            f"accepted={initial_phase['accepted_rows']} skipped={initial_phase['skipped_rows']}"
        )
        for status, item in initial_phase.get("by_status", {}).items():
            systems = ",".join(
                f"{system}:{count}"
                for system, count in item.get("constellation_counts", {}).items()
            )
            frequencies = ",".join(
                f"f{frequency}:{count}"
                for frequency, count in item.get("frequency_index_counts", {}).items()
            )
            reasons = ",".join(
                f"{reason['reason']}:{reason['rows']}"
                for reason in item.get("skip_reasons", [])
            )
            print(
                "  "
                f"{status}: rows={item['rows']} corr={item['correction_matches']} "
                f"systems={systems} freq={frequencies} ssr={item['ssr_available_rows']} "
                f"orbit={item['orbit_clock_applied_rows']} stec={item['stec_nonzero_rows']} "
                f"iono_constraint={item['ionosphere_constraint_rows']} "
                f"phase_bias_nonzero={item['phase_bias_nonzero_rows']} "
                f"abs_residual={item['min_abs_residual_m']:.3f}..{item['max_abs_residual_m']:.3f} "
                f"reasons={reasons or '-'}"
            )

    residual = report.get("native_residual_summary")
    if residual is None:
        return
    print("residuals:")
    print(
        "  "
        f"final_rows={residual['final_iteration_rows']} "
        f"classified={residual['classified_rows']} skipped={residual['skipped_rows']}"
    )
    for item in residual["by_relation_row_type_constellation"]:
        print(
            "  "
            f"{item['relation']} {item['row_type']} {item['constellation']}: "
            f"n={item['rows']} rms={item['rms_residual_m']:.3f} "
            f"max={item['max_abs_residual_m']:.3f}"
        )
    native_only = residual.get("top_satellites_by_relation_type", {}).get("native_only", {})
    if native_only:
        print("  top_native_only_residual_satellites:")
        for row_type, items in native_only.items():
            formatted = ", ".join(
                f"{item['sat']} n={item['rows']} rms={item['rms_residual_m']:.3f} "
                f"max={item['max_abs_residual_m']:.3f}"
                for item in items[:3]
            )
            print(f"    {row_type}: {formatted}")


def parse_args(argv: Optional[Sequence[str]] = None) -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description="Compare MADOCALIB .stat $SAT satellite sets with native PPP logs."
    )
    parser.add_argument("bridge_stat", type=Path, help="MADOCALIB .pos.stat file")
    parser.add_argument("native_correction_log", type=Path, help="native gnss_ppp --ppp-correction-log CSV")
    parser.add_argument("--residual-log", type=Path, help="native gnss_ppp --ppp-residual-log CSV")
    parser.add_argument("--json-out", type=Path, help="write full report JSON")
    parser.add_argument("--epoch-csv", type=Path, help="write per-epoch satellite-set differences")
    parser.add_argument("--top-satellites", type=int, default=10)
    parser.add_argument("--top-epochs", type=int, default=20)
    return parser.parse_args(argv)


def main(argv: Optional[Sequence[str]] = None) -> int:
    args = parse_args(argv)
    report = build_report(args)
    if args.json_out is not None:
        args.json_out.parent.mkdir(parents=True, exist_ok=True)
        args.json_out.write_text(json.dumps(report, indent=2, sort_keys=True), encoding="utf-8")
    if args.epoch_csv is not None:
        args.epoch_csv.parent.mkdir(parents=True, exist_ok=True)
        write_epoch_csv(args.epoch_csv, report["satellite_set_comparison"]["per_epoch"])
    print_report(report, top_epochs=args.top_epochs)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
