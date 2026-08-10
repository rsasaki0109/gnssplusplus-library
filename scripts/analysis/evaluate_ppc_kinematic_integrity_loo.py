#!/usr/bin/env python3
"""Six-fold LOO evaluation for the truth-free PPC kinematic integrity gate."""

from __future__ import annotations

import argparse
import itertools
import json
import math
from pathlib import Path
import sys
from typing import Any

import numpy as np


ROOT_DIR = Path(__file__).resolve().parents[2]
SCRIPTS_DIR = ROOT_DIR / "scripts"
ANALYSIS_DIR = SCRIPTS_DIR / "analysis"
for script_dir in (SCRIPTS_DIR, ANALYSIS_DIR):
    if str(script_dir) not in sys.path:
        sys.path.insert(0, str(script_dir))

import analyze_ppc_wrong_fix_residuals as analysis  # noqa: E402
import apply_ppc_status_demotion as demotion  # noqa: E402


RUN_PATHS = dict(analysis.RUNS)
JUMP_GRID_M = (8.0, 10.0, 12.0, 15.0, 20.0)
ACCELERATION_GRID_MPS2 = (50.0, 100.0, 200.0, 400.0)
HOLD_GRID_EPOCHS = (1, 2, 3, 5)
PLATEAU_GRID_M = (0.05, 0.1, 0.2)
MAX_HOLD_GRID_EPOCHS = (5, 8)
SECONDARY_PREFIT_GRID_M = (5.0, 8.0, 10.0)
SECONDARY_RATIO_GRID = (8.0, 10.0, 15.0)
SECONDARY_OUTLIER_GRID = (10, 15, 20)
SECONDARY_SATELLITE_GRID = (12, 13, 15)
PRIMARY_PRODUCTION = (12.0, 200.0, 3)
ADVANCED_PRODUCTION = (0.1, 8, 5.0, 10.0, 10, 13)


def parse_specs(specs: list[str]) -> dict[str, Path]:
    paths: dict[str, Path] = {}
    for spec in specs:
        key, separator, value = spec.partition("=")
        if not separator or key not in RUN_PATHS or key in paths:
            raise SystemExit(f"invalid or duplicate --pos value: {spec!r}")
        paths[key] = Path(value)
    missing = sorted(set(RUN_PATHS) - set(paths))
    if missing:
        raise SystemExit(f"missing --pos runs: {', '.join(missing)}")
    return paths


def data_parts(path: Path) -> list[list[str]]:
    with path.open(encoding="utf-8") as handle:
        return [
            line.split()
            for line in handle
            if line.strip() and not line.startswith("%") and len(line.split()) >= 9
        ]


def load_run(
    key: str,
    path: Path,
    dataset_root: Path,
    base_rule: demotion.DemotionRule,
) -> np.ndarray:
    reference = analysis.load_reference(dataset_root / RUN_PATHS[key] / "reference.csv")
    epochs = analysis.load_solution(path)
    parts = data_parts(path)
    if len(epochs) != len(parts):
        raise SystemExit(f"{path}: parsed epoch/row count mismatch")
    rows: list[tuple[float, ...]] = []
    previous: analysis.SolutionEpoch | None = None
    previous_velocity: np.ndarray | None = None
    for epoch, fields in zip(epochs, parts, strict=True):
        velocity: np.ndarray | None = None
        jump_m = math.nan
        acceleration_mps2 = math.nan
        if previous is not None and epoch.week == previous.week:
            dt = epoch.tow_s - previous.tow_s
            if 0.0 < dt <= 1.0:
                velocity = (np.asarray(epoch.ecef) - np.asarray(previous.ecef)) / dt
                jump_m = float(np.linalg.norm(np.asarray(epoch.ecef) - previous.ecef))
                if previous_velocity is not None:
                    acceleration_mps2 = float(
                        np.linalg.norm(velocity - previous_velocity) / dt
                    )
        reference_ecef = reference.get((epoch.week, epoch.tow_s))
        error_m = (
            analysis.error_3d_m(epoch.ecef, reference_ecef)
            if reference_ecef is not None
            else math.nan
        )
        base_allowed = epoch.status == 4 and (
            not demotion.should_demote(fields, base_rule)
            or demotion.should_exonerate(fields, base_rule)
        )
        telemetry = tuple(
            value
            if (value := demotion.optional_float(fields, index)) is not None
            else math.nan
            for index in (18, 11, 17, 9)
        )
        rows.append(
            (float(base_allowed), jump_m, acceleration_mps2, error_m, *telemetry)
        )
        previous = epoch
        previous_velocity = velocity
    return np.asarray(rows, dtype=float)


def candidate_mask(
    rows: np.ndarray,
    jump_m: float,
    acceleration_mps2: float,
    hold_epochs: int,
) -> np.ndarray:
    remaining = 0
    mask = np.zeros(len(rows), dtype=bool)
    for index, row in enumerate(rows):
        jump, acceleration = row[1:3]
        if jump > jump_m and acceleration > acceleration_mps2:
            remaining = max(remaining, hold_epochs)
        if remaining > 0:
            mask[index] = True
            remaining -= 1
    return mask & rows[:, 0].astype(bool)


def advanced_candidate_mask(
    rows: np.ndarray,
    plateau_max_jump_m: float,
    max_hold_epochs: int,
    secondary: tuple[float, float, int, int] | None,
) -> np.ndarray:
    """Replay the production primary gate plus optional advanced extensions."""
    hold_remaining = 0
    quarantine_age = 0
    quarantine_active = False
    mask = np.zeros(len(rows), dtype=bool)
    for index, row in enumerate(rows):
        jump, acceleration = row[1:3]
        primary_trigger = jump > 12.0 and acceleration > 200.0
        secondary_trigger = False
        if secondary is not None:
            prefit_min, ratio_max, outliers_min, satellites_max = secondary
            prefit, ratio, outliers, satellites = row[4:8]
            secondary_trigger = (
                jump > 5.0
                and acceleration > 100.0
                and prefit > prefit_min
                and ratio <= ratio_max
                and outliers >= outliers_min
                and satellites <= satellites_max
            )
        if primary_trigger or secondary_trigger:
            hold_remaining = max(hold_remaining, 3)
            quarantine_age = 0
            quarantine_active = True
        keep_quarantine = False
        if quarantine_active:
            keep_quarantine = hold_remaining > 0
            if (
                not keep_quarantine
                and quarantine_age < max_hold_epochs
                and jump <= plateau_max_jump_m
            ):
                keep_quarantine = True
            if keep_quarantine:
                quarantine_age += 1
                hold_remaining = max(0, hold_remaining - 1)
            else:
                quarantine_active = False
        mask[index] = keep_quarantine
    return mask & rows[:, 0].astype(bool)


def score_mask(rows: np.ndarray, mask: np.ndarray) -> dict[str, int]:
    errors = rows[:, 3]
    return {
        "demoted": int(np.sum(mask)),
        "correct_fix_harmed": int(np.sum(mask & (errors <= 0.5))),
        "wrong_fix_caught": int(np.sum(mask & (errors > 0.5))),
        "wrong_fix_gt5m_caught": int(np.sum(mask & (errors > 5.0))),
        "wrong_fix_gt10m_caught": int(np.sum(mask & (errors > 10.0))),
    }


def evaluate(
    runs: dict[str, np.ndarray],
    max_training_correct_harm_rate: float,
    production: tuple[float, float, int],
) -> dict[str, Any]:
    candidates: list[dict[str, Any]] = []
    for jump, acceleration, hold in itertools.product(
        JUMP_GRID_M, ACCELERATION_GRID_MPS2, HOLD_GRID_EPOCHS
    ):
        per_run = {
            key: score_mask(rows, candidate_mask(rows, jump, acceleration, hold))
            for key, rows in runs.items()
        }
        candidates.append(
            {
                "jump_m": jump,
                "acceleration_mps2": acceleration,
                "hold_epochs": hold,
                "runs": per_run,
            }
        )

    folds: list[dict[str, Any]] = []
    for held_out in runs:
        training = [key for key in runs if key != held_out]
        training_correct = sum(
            int(np.sum(rows[:, 0].astype(bool) & (rows[:, 3] <= 0.5)))
            for key, rows in runs.items()
            if key in training
        )
        eligible: list[tuple[tuple[float, ...], dict[str, Any]]] = []
        for candidate in candidates:
            train_stats = {
                field: sum(candidate["runs"][key][field] for key in training)
                for field in next(iter(candidate["runs"].values()))
            }
            harm_rate = train_stats["correct_fix_harmed"] / max(training_correct, 1)
            if harm_rate > max_training_correct_harm_rate:
                continue
            # Accuracy first; conservative thresholds break exact ties.
            rank = (
                train_stats["wrong_fix_gt10m_caught"],
                train_stats["wrong_fix_caught"],
                -train_stats["correct_fix_harmed"],
                -candidate["hold_epochs"],
                candidate["jump_m"],
                candidate["acceleration_mps2"],
            )
            eligible.append((rank, {**candidate, "training": train_stats}))
        if not eligible:
            raise SystemExit(f"no eligible candidate for held-out run {held_out}")
        selected = max(eligible, key=lambda item: item[0])[1]
        folds.append(
            {
                "held_out": held_out,
                "selected": {
                    key: selected[key]
                    for key in ("jump_m", "acceleration_mps2", "hold_epochs")
                },
                "training": selected["training"],
                "held_out_metrics": selected["runs"][held_out],
            }
        )

    production_candidate = next(
        candidate
        for candidate in candidates
        if (
            candidate["jump_m"],
            candidate["acceleration_mps2"],
            candidate["hold_epochs"],
        )
        == production
    )
    fields = next(iter(production_candidate["runs"].values()))
    production_total = {
        field: sum(row[field] for row in production_candidate["runs"].values())
        for field in fields
    }

    primary_masks = {
        key: candidate_mask(rows, *PRIMARY_PRODUCTION) for key, rows in runs.items()
    }
    extension_candidates: list[dict[str, Any]] = []
    secondary_grid: list[tuple[float, float, int, int] | None] = [None]
    secondary_grid.extend(
        itertools.product(
            SECONDARY_PREFIT_GRID_M,
            SECONDARY_RATIO_GRID,
            SECONDARY_OUTLIER_GRID,
            SECONDARY_SATELLITE_GRID,
        )
    )
    for plateau, max_hold, secondary in itertools.product(
        PLATEAU_GRID_M, MAX_HOLD_GRID_EPOCHS, secondary_grid
    ):
        masks = {
            key: advanced_candidate_mask(rows, plateau, max_hold, secondary)
            for key, rows in runs.items()
        }
        extension_candidates.append(
            {
                "plateau_max_jump_m": plateau,
                "max_hold_epochs": max_hold,
                "secondary": secondary,
                "runs": {
                    key: score_mask(rows, masks[key]) for key, rows in runs.items()
                },
                "extension_runs": {
                    key: score_mask(rows, masks[key] & ~primary_masks[key])
                    for key, rows in runs.items()
                },
            }
        )

    extension_folds: list[dict[str, Any]] = []
    for held_out in runs:
        training = [key for key in runs if key != held_out]
        training_correct = sum(
            int(np.sum(rows[:, 0].astype(bool) & (rows[:, 3] <= 0.5)))
            for key, rows in runs.items()
            if key in training
        )
        eligible: list[tuple[tuple[float, ...], dict[str, Any]]] = []
        for candidate in extension_candidates:
            extension_train = {
                field: sum(candidate["extension_runs"][key][field] for key in training)
                for field in fields
            }
            harm_rate = extension_train["correct_fix_harmed"] / max(training_correct, 1)
            if harm_rate > max_training_correct_harm_rate:
                continue
            secondary = candidate["secondary"]
            rank = (
                extension_train["wrong_fix_gt10m_caught"],
                extension_train["wrong_fix_caught"],
                -extension_train["correct_fix_harmed"],
                secondary is None,
                -candidate["max_hold_epochs"],
                -candidate["plateau_max_jump_m"],
            )
            eligible.append((rank, {**candidate, "training": extension_train}))
        if not eligible:
            raise SystemExit(
                f"no eligible extension candidate for held-out run {held_out}"
            )
        selected = max(eligible, key=lambda item: item[0])[1]
        extension_folds.append(
            {
                "held_out": held_out,
                "selected": {
                    "plateau_max_jump_m": selected["plateau_max_jump_m"],
                    "max_hold_epochs": selected["max_hold_epochs"],
                    "secondary": selected["secondary"],
                },
                "training_extension": selected["training"],
                "held_out_extension": selected["extension_runs"][held_out],
                "held_out_combined": selected["runs"][held_out],
            }
        )

    advanced_candidate = next(
        candidate
        for candidate in extension_candidates
        if (
            candidate["plateau_max_jump_m"],
            candidate["max_hold_epochs"],
            *(candidate["secondary"] or ()),
        )
        == ADVANCED_PRODUCTION
    )
    advanced_total = {
        field: sum(row[field] for row in advanced_candidate["runs"].values())
        for field in fields
    }
    advanced_extension_total = {
        field: sum(row[field] for row in advanced_candidate["extension_runs"].values())
        for field in fields
    }
    return {
        "runtime_truth_used": False,
        "reference_truth_role": "offline labels and LOO scoring only",
        "candidate_grid": {
            "jump_m": JUMP_GRID_M,
            "acceleration_mps2": ACCELERATION_GRID_MPS2,
            "hold_epochs": HOLD_GRID_EPOCHS,
        },
        "max_training_correct_harm_rate": max_training_correct_harm_rate,
        "folds": folds,
        "extension_candidate_grid": {
            "plateau_max_jump_m": PLATEAU_GRID_M,
            "max_hold_epochs": MAX_HOLD_GRID_EPOCHS,
            "secondary_prefit_min_m": SECONDARY_PREFIT_GRID_M,
            "secondary_ratio_max": SECONDARY_RATIO_GRID,
            "secondary_outliers_min": SECONDARY_OUTLIER_GRID,
            "secondary_satellites_max": SECONDARY_SATELLITE_GRID,
            "secondary_off_included": True,
        },
        "extension_harm_limit_applies_to": "increment beyond fixed primary gate",
        "extension_folds": extension_folds,
        "production": {
            "jump_m": production[0],
            "acceleration_mps2": production[1],
            "hold_epochs": production[2],
            "runs": production_candidate["runs"],
            "total": production_total,
        },
        "advanced_production": {
            "primary": {
                "jump_m": PRIMARY_PRODUCTION[0],
                "acceleration_mps2": PRIMARY_PRODUCTION[1],
                "hold_epochs": PRIMARY_PRODUCTION[2],
            },
            "plateau_max_jump_m": advanced_candidate["plateau_max_jump_m"],
            "max_hold_epochs": advanced_candidate["max_hold_epochs"],
            "secondary": advanced_candidate["secondary"],
            "runs": advanced_candidate["runs"],
            "extension_runs": advanced_candidate["extension_runs"],
            "total": advanced_total,
            "extension_total": advanced_extension_total,
        },
    }


def markdown(payload: dict[str, Any]) -> str:
    lines = [
        "# PPC kinematic integrity six-fold LOO",
        "",
        "Reference truth is used only for offline labels and candidate scoring. Runtime inputs are consecutive POS positions and timestamps.",
        "",
        "| Held out | Selected jump / acceleration / hold | Train >10m caught | Train correct harm | Held >10m caught | Held correct harm |",
        "|---|---:|---:|---:|---:|---:|",
    ]
    for fold in payload["folds"]:
        selected = fold["selected"]
        lines.append(
            f"| {fold['held_out']} | {selected['jump_m']:.0f} m / "
            f"{selected['acceleration_mps2']:.0f} m/s2 / {selected['hold_epochs']} | "
            f"{fold['training']['wrong_fix_gt10m_caught']} | "
            f"{fold['training']['correct_fix_harmed']} | "
            f"{fold['held_out_metrics']['wrong_fix_gt10m_caught']} | "
            f"{fold['held_out_metrics']['correct_fix_harmed']} |"
        )
    production = payload["production"]
    lines.extend(
        [
            "",
            f"Production: **{production['jump_m']:.0f} m / "
            f"{production['acceleration_mps2']:.0f} m/s2 / "
            f"{production['hold_epochs']} epochs**.",
            "",
            f"It catches **{production['total']['wrong_fix_gt10m_caught']}** >10 m "
            f"wrong FIX and **{production['total']['wrong_fix_caught']}** total wrong FIX "
            f"while harming **{production['total']['correct_fix_harmed']}** correct FIX epochs.",
            "",
        ]
    )
    lines.extend(
        [
            "## Advanced extension LOO",
            "",
            "The primary production gate is fixed. Each fold selects only plateau continuation and the telemetry-backed secondary trigger from the other five runs.",
            "",
            "| Held out | Plateau / max age / secondary | Train added >10m | Train added correct harm | Held added >10m | Held added correct harm |",
            "|---|---:|---:|---:|---:|---:|",
        ]
    )
    for fold in payload["extension_folds"]:
        selected = fold["selected"]
        secondary = selected["secondary"]
        secondary_text = (
            "off"
            if secondary is None
            else f"prefit>{secondary[0]:g}, ratio<={secondary[1]:g}, outliers>={secondary[2]}, nsat<={secondary[3]}"
        )
        lines.append(
            f"| {fold['held_out']} | {selected['plateau_max_jump_m']:g} m / "
            f"{selected['max_hold_epochs']} / {secondary_text} | "
            f"{fold['training_extension']['wrong_fix_gt10m_caught']} | "
            f"{fold['training_extension']['correct_fix_harmed']} | "
            f"{fold['held_out_extension']['wrong_fix_gt10m_caught']} | "
            f"{fold['held_out_extension']['correct_fix_harmed']} |"
        )
    advanced = payload["advanced_production"]
    lines.extend(
        [
            "",
            f"Advanced production catches **{advanced['total']['wrong_fix_gt10m_caught']}** >10 m and "
            f"**{advanced['total']['wrong_fix_caught']}** total wrong FIX while harming "
            f"**{advanced['total']['correct_fix_harmed']}** correct FIX epochs. The extension alone adds "
            f"{advanced['extension_total']['wrong_fix_gt10m_caught']} / "
            f"{advanced['extension_total']['wrong_fix_caught']} catches at a cost of "
            f"{advanced['extension_total']['correct_fix_harmed']} correct epochs.",
            "",
        ]
    )
    return "\n".join(lines)


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--dataset-root", type=Path, required=True)
    parser.add_argument("--pos", action="append", required=True, metavar="RUN_KEY=PATH")
    parser.add_argument("--output-json", type=Path, required=True)
    parser.add_argument("--markdown-output", type=Path, required=True)
    parser.add_argument("--max-training-correct-harm-rate", type=float, default=0.0005)
    args = parser.parse_args()
    paths = parse_specs(args.pos)
    base_rule = demotion.DemotionRule(
        max_ratio=None,
        min_baseline_m=None,
        max_baseline_m=None,
        max_nis_per_obs=None,
        max_post_rms_m=None,
        min_satellites=8,
        low_satellite_ceiling=11,
        low_satellite_max_ratio=15.0,
        exonerate_min_satellites=11,
        exonerate_max_prefit_rms_m=0.5,
        exonerate_max_nis_per_obs=0.2,
    )
    runs = {
        key: load_run(key, path, args.dataset_root, base_rule)
        for key, path in paths.items()
    }
    payload = evaluate(runs, args.max_training_correct_harm_rate, (12.0, 200.0, 3))
    payload["positions"] = {key: str(path) for key, path in paths.items()}
    args.output_json.parent.mkdir(parents=True, exist_ok=True)
    args.output_json.write_text(json.dumps(payload, indent=2) + "\n", encoding="utf-8")
    args.markdown_output.parent.mkdir(parents=True, exist_ok=True)
    args.markdown_output.write_text(markdown(payload), encoding="utf-8")
    print(json.dumps(payload["production"]["total"], indent=2))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
