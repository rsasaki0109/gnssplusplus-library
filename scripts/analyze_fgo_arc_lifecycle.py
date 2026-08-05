#!/usr/bin/env python3
"""Audit causal ambiguity-lifecycle events around wrong FGO FIX onsets."""

from __future__ import annotations

import argparse
import csv
import json
import math
from collections import defaultdict
from pathlib import Path
from statistics import median
from typing import Any


FROZEN_LOOKBACK_EPOCHS = 5
MINIMUM_WRONG_ONSETS = 20
MINIMUM_CORRECT_FIX_EPOCHS = 100
MINIMUM_ONSET_COVERAGE = 0.20
MAXIMUM_CORRECT_EXPOSURE = 0.05
MINIMUM_RELATIVE_RISK = 4.0
MAXIMUM_CAUSAL_GAP_MS = 250


def tow_key(value: str) -> int:
    return round(float(value) * 1000.0)


def as_int(row: dict[str, str], name: str) -> int:
    value = row.get(name, "")
    return int(float(value)) if value else 0


def as_float(row: dict[str, str], name: str) -> float:
    value = row.get(name, "")
    return float(value) if value else 0.0


def load_candidates(path: Path) -> dict[int, list[dict[str, str]]]:
    candidates: dict[int, list[dict[str, str]]] = defaultdict(list)
    with path.open(newline="", encoding="utf-8-sig") as stream:
        for row in csv.DictReader(stream):
            if row.get("disposition") == "0":
                candidates[tow_key(row["tow"])].append(row)
    return candidates


def candidate_lifecycle(
    epoch_keys: list[int], candidates: dict[int, list[dict[str, str]]]
) -> list[dict[str, Any]]:
    first_seen: dict[int, int] = {}
    previous_ids: set[int] = set()
    previous_references: dict[tuple[str, str], str] = {}
    result: list[dict[str, Any]] = []
    for epoch_index, key in enumerate(epoch_keys):
        delta_ms = key - epoch_keys[epoch_index - 1] if epoch_index > 0 else 0
        continuous = (
            epoch_index > 0
            and 0 < delta_ms <= MAXIMUM_CAUSAL_GAP_MS
        )
        if epoch_index > 0 and not continuous:
            first_seen.clear()
        rows = candidates.get(key, [])
        ambiguity_ids = {int(row["ambiguity_index"]) for row in rows}
        references = {
            (row["system"], row["signal"]): row["reference_satellite"]
            for row in rows
        }
        for ambiguity_id in ambiguity_ids:
            first_seen.setdefault(ambiguity_id, epoch_index)
        ages = [epoch_index - first_seen[value] + 1 for value in ambiguity_ids]
        new_ids = ambiguity_ids - previous_ids if continuous else ambiguity_ids
        union = ambiguity_ids | previous_ids
        churn = (
            1.0 - len(ambiguity_ids & previous_ids) / len(union)
            if continuous and union
            else 0.0
        )
        reference_changes = 0
        if continuous:
            groups = set(references) | set(previous_references)
            reference_changes = sum(
                references.get(group) != previous_references.get(group)
                for group in groups
            )
        result.append(
            {
                "eligible_ambiguities": len(ambiguity_ids),
                "new_ambiguities": len(new_ids),
                "new_ambiguity_fraction": (
                    len(new_ids) / len(ambiguity_ids) if ambiguity_ids else 0.0
                ),
                "ambiguity_churn_fraction": churn,
                "minimum_arc_age_epochs": min(ages) if ages else 0,
                "median_arc_age_epochs": median(ages) if ages else 0.0,
                "reference_groups": len(references),
                "reference_changes": reference_changes,
            }
        )
        previous_ids = ambiguity_ids
        previous_references = references
    return result


def label_and_events(
    epoch_rows: list[dict[str, str]], lifecycle: list[dict[str, Any]]
) -> tuple[list[str], list[set[str]], list[bool], list[bool]]:
    event_names = [
        "clock_jump",
        "reference_change",
        "high_ambiguity_churn",
        "young_arc",
        "geometry_free_slip",
        "doppler_slip",
        "generation_bump",
        "fde_exclusion",
        "any_lifecycle_discontinuity",
    ]
    events: list[set[str]] = []
    wrong_fix: list[bool] = []
    correct_fix: list[bool] = []
    for row, arc in zip(epoch_rows, lifecycle):
        error = math.sqrt(
            sum(float(row[name]) ** 2 for name in ("e_err_m", "n_err_m", "u_err_m"))
        )
        fixed = row["status"].upper() == "FIXED"
        wrong_fix.append(fixed and error > 0.5)
        correct_fix.append(fixed and error <= 0.5)

        current: set[str] = set()
        has_signed_clock_delta = as_int(row, "clock_common_delta_satellites") > 0
        if (
            has_signed_clock_delta
            and abs(as_float(row, "clock_common_delta_m")) > 1e5
        ) or (not has_signed_clock_delta and as_int(row, "clock_jump") > 0):
            current.add("clock_jump")
        if arc["reference_changes"] > 0:
            current.add("reference_change")
        if arc["ambiguity_churn_fraction"] >= 0.25:
            current.add("high_ambiguity_churn")
        if arc["eligible_ambiguities"] > 0 and arc["minimum_arc_age_epochs"] <= 2:
            current.add("young_arc")
        if as_int(row, "gf_slip_events") > 0:
            current.add("geometry_free_slip")
        if as_int(row, "doppler_slip_signals") > 0:
            current.add("doppler_slip")
        if sum(
            as_int(row, name)
            for name in (
                "gen_bump_hold",
                "gen_bump_fde",
                "gen_bump_reset",
                "gen_bump_warm_reset",
                "gen_bump_stale_pin",
            )
        ) > 0:
            current.add("generation_bump")
        if as_int(row, "amb_excl_fde") > 0:
            current.add("fde_exclusion")
        if current:
            current.add("any_lifecycle_discontinuity")
        events.append(current)
    return event_names, events, wrong_fix, correct_fix


def exposure(
    events: list[set[str]],
    index: int,
    name: str,
    lookback: int,
    epoch_keys: list[int] | None = None,
) -> bool:
    begin = max(0, index - lookback)
    for position in range(index, begin - 1, -1):
        if (
            epoch_keys is not None
            and position < index
            and not (
                0
                < epoch_keys[position + 1] - epoch_keys[position]
                <= MAXIMUM_CAUSAL_GAP_MS
            )
        ):
            break
        if name in events[position]:
            return True
    return False


def score(
    event_names: list[str],
    events: list[set[str]],
    wrong_fix: list[bool],
    correct_fix: list[bool],
    epoch_keys: list[int] | None = None,
    lookbacks: tuple[int, ...] = (0, 1, 3, FROZEN_LOOKBACK_EPOCHS),
) -> dict[str, Any]:
    wrong_onsets = [
        index
        for index, value in enumerate(wrong_fix)
        if value
        and (
            index == 0
            or not wrong_fix[index - 1]
            or (
                epoch_keys is not None
                and not (
                    0
                    < epoch_keys[index] - epoch_keys[index - 1]
                    <= MAXIMUM_CAUSAL_GAP_MS
                )
            )
        )
    ]
    correct_indexes = [index for index, value in enumerate(correct_fix) if value]
    table: dict[str, dict[str, Any]] = {}
    for name in event_names:
        rows: dict[str, Any] = {}
        for lookback in lookbacks:
            onset_count = sum(
                exposure(events, i, name, lookback, epoch_keys) for i in wrong_onsets
            )
            correct_count = sum(
                exposure(events, i, name, lookback, epoch_keys) for i in correct_indexes
            )
            onset_coverage = onset_count / len(wrong_onsets) if wrong_onsets else None
            correct_exposure = (
                correct_count / len(correct_indexes) if correct_indexes else None
            )
            relative_risk = (
                onset_coverage / correct_exposure
                if onset_coverage is not None and correct_exposure not in (None, 0.0)
                else None
            )
            rows[str(lookback)] = {
                "wrong_onsets_exposed": onset_count,
                "wrong_onset_coverage": onset_coverage,
                "correct_fix_exposed": correct_count,
                "correct_fix_exposure": correct_exposure,
                "relative_risk": relative_risk,
            }
        frozen = rows[str(FROZEN_LOOKBACK_EPOCHS)]
        rows["gate_passed"] = (
            len(wrong_onsets) >= MINIMUM_WRONG_ONSETS
            and len(correct_indexes) >= MINIMUM_CORRECT_FIX_EPOCHS
            and frozen["wrong_onset_coverage"] is not None
            and frozen["wrong_onset_coverage"] >= MINIMUM_ONSET_COVERAGE
            and frozen["correct_fix_exposure"] is not None
            and frozen["correct_fix_exposure"] <= MAXIMUM_CORRECT_EXPOSURE
            and (
                (frozen["correct_fix_exposure"] == 0.0 and frozen["wrong_onset_coverage"] > 0.0)
                or (
                    frozen["relative_risk"] is not None
                    and frozen["relative_risk"] >= MINIMUM_RELATIVE_RISK
                )
            )
        )
        table[name] = rows
    return {
        "support": {
            "wrong_fix_epochs": sum(wrong_fix),
            "wrong_fix_onsets": len(wrong_onsets),
            "correct_fix_epochs": len(correct_indexes),
        },
        "requirements": {
            "frozen_lookback_epochs": FROZEN_LOOKBACK_EPOCHS,
            "maximum_causal_gap_ms": MAXIMUM_CAUSAL_GAP_MS,
            "minimum_wrong_onsets": MINIMUM_WRONG_ONSETS,
            "minimum_correct_fix_epochs": MINIMUM_CORRECT_FIX_EPOCHS,
            "minimum_wrong_onset_coverage": MINIMUM_ONSET_COVERAGE,
            "maximum_correct_fix_exposure": MAXIMUM_CORRECT_EXPOSURE,
            "minimum_relative_risk": MINIMUM_RELATIVE_RISK,
        },
        "events": table,
        "qualifying_events": [name for name, rows in table.items() if rows["gate_passed"]],
    }


def render_markdown(summary: dict[str, Any]) -> str:
    support = summary["support"]
    lines = [
        "# FGO ambiguity lifecycle wrong-FIX audit",
        "",
        f"Wrong FIX: **{support['wrong_fix_epochs']} epochs / "
        f"{support['wrong_fix_onsets']} causal onsets**; correct FIX controls: "
        f"**{support['correct_fix_epochs']} epochs**.",
        "",
        "The frozen verdict uses only each onset epoch and the preceding five epochs. "
        "Future events are never visible to the classifier, and observation gaps over "
        f"{MAXIMUM_CAUSAL_GAP_MS} ms terminate the lookback.",
        "",
        "| Event | Wrong onsets exposed | Coverage | Correct FIX exposure | Relative risk | Gate |",
        "|---|---:|---:|---:|---:|---:|",
    ]
    for name, event in summary["events"].items():
        frozen = event[str(FROZEN_LOOKBACK_EPOCHS)]
        risk = frozen["relative_risk"]
        risk_text = f"{risk:.3f}" if risk is not None else "n/a"
        onset_coverage = frozen["wrong_onset_coverage"]
        correct_exposure = frozen["correct_fix_exposure"]
        onset_text = (
            f"{100.0 * onset_coverage:.3f}%" if onset_coverage is not None else "n/a"
        )
        correct_text = (
            f"{100.0 * correct_exposure:.3f}%" if correct_exposure is not None else "n/a"
        )
        lines.append(
            f"| `{name}` | {frozen['wrong_onsets_exposed']} | "
            f"{onset_text} | {correct_text} | "
            f"{risk_text} | "
            f"{'PASS' if event['gate_passed'] else 'FAIL'} |"
        )
    lines.extend(
        [
            "",
            "Qualifying events: "
            + (", ".join(f"`{name}`" for name in summary["qualifying_events"]) or "none"),
            "",
        ]
    )
    return "\n".join(lines)


def analyze(epoch_csv: Path, candidate_csv: Path) -> dict[str, Any]:
    with epoch_csv.open(newline="", encoding="utf-8-sig") as stream:
        rows = list(csv.DictReader(stream))
    keys = [tow_key(row["tow"]) for row in rows]
    lifecycle = candidate_lifecycle(keys, load_candidates(candidate_csv))
    event_names, events, wrong_fix, correct_fix = label_and_events(rows, lifecycle)
    summary = score(event_names, events, wrong_fix, correct_fix, keys)
    summary["rows"] = len(rows)
    summary["clock_jump_available"] = (
        bool(rows)
        and (
            "clock_jump" in rows[0]
            or {
                "clock_common_delta_m",
                "clock_common_delta_satellites",
            }.issubset(rows[0])
        )
    )
    return summary


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--epoch-csv", type=Path, required=True)
    parser.add_argument("--candidate-csv", type=Path)
    parser.add_argument("--json", type=Path)
    parser.add_argument("--markdown", type=Path)
    args = parser.parse_args()
    candidate_csv = args.candidate_csv or Path(str(args.epoch_csv) + ".ar_candidates.csv")
    summary = analyze(args.epoch_csv, candidate_csv)
    rendered = json.dumps(summary, indent=2, sort_keys=True, allow_nan=False)
    print(rendered)
    if args.json:
        args.json.write_text(rendered + "\n", encoding="utf-8")
    if args.markdown:
        args.markdown.write_text(render_markdown(summary), encoding="utf-8")
    return 0 if summary["qualifying_events"] else 1


if __name__ == "__main__":
    raise SystemExit(main())
