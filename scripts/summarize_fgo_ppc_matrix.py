#!/usr/bin/env python3
"""Score six gnss_fgo_parity CSVs with the common PPC metric definitions."""

from __future__ import annotations

import argparse
import json
from pathlib import Path
import sys


ROOT = Path(__file__).resolve().parents[1]
COMMANDS = ROOT / "apps" / "commands"
for group in ("benchmarks", "positioning", "products", "receivers"):
    sys.path.insert(0, str(COMMANDS / group))
sys.path.insert(0, str(COMMANDS))
sys.path.insert(0, str(ROOT / "scripts"))

import generate_driving_comparison as comparison  # noqa: E402
import gnss_ppc_metrics as metrics  # noqa: E402


RUNS = tuple(
    (city, f"run{run}") for city in ("tokyo", "nagoya") for run in range(1, 4)
)


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--dataset-root", type=Path, required=True)
    source = parser.add_mutually_exclusive_group(required=True)
    source.add_argument(
        "--csv-template",
        help="Path template containing {city} and {run}, for example output/fgo_{city}_{run}.csv",
    )
    source.add_argument(
        "--pos",
        action="append",
        metavar="CITY/RUN=PATH",
        help="Selected .pos solution; repeat once for each of the six PPC runs.",
    )
    parser.add_argument("--output-json", type=Path, required=True)
    parser.add_argument("--markdown-output", type=Path)
    parser.add_argument("--match-tolerance-s", type=float, default=0.11)
    return parser.parse_args()


def macro_mean(rows: list[dict[str, object]], key: str) -> float:
    return round(sum(float(row[key]) for row in rows) / len(rows), 6)


def parse_pos_specs(specs: list[str] | None) -> dict[tuple[str, str], Path]:
    selected: dict[tuple[str, str], Path] = {}
    for spec in specs or []:
        try:
            run_key, path_text = spec.split("=", 1)
            city, run = run_key.lower().split("/", 1)
        except ValueError as error:
            raise SystemExit(f"invalid --pos value {spec!r}; expected CITY/RUN=PATH") from error
        key = (city, run)
        if key not in RUNS:
            raise SystemExit(f"invalid PPC run in --pos: {run_key!r}")
        if key in selected:
            raise SystemExit(f"duplicate --pos for {city}/{run}")
        selected[key] = Path(path_text)
    if selected and set(selected) != set(RUNS):
        missing = ", ".join(f"{city}/{run}" for city, run in RUNS if (city, run) not in selected)
        raise SystemExit(f"--pos requires all six PPC runs; missing: {missing}")
    return selected


def render_markdown(payload: dict[str, object]) -> str:
    rows = payload["runs"]
    assert isinstance(rows, list)
    lines = [
        "| Run | FIX | Wrong FIX/FIX | Correct FIX/ref | 50 cm/ref | Official | P95 H |",
        "|---|---:|---:|---:|---:|---:|---:|",
    ]
    for row in rows:
        assert isinstance(row, dict)
        lines.append(
            f"| {row['label']} | {row['fix_rate_pct']:.3f}% | "
            f"{row['wrong_fix_rate_pct']:.3f}% | {row['correct_fix_ref_pct']:.3f}% | "
            f"{row['ppc_score_3d_50cm_ref_pct']:.3f}% | "
            f"{row['ppc_official_score_pct']:.3f}% | {row['p95_h_m']:.3f} m |"
        )
    aggregate = payload["macro_mean"]
    assert isinstance(aggregate, dict)
    lines.append(
        f"| **Macro mean** | **{aggregate['fix_rate_pct']:.3f}%** | "
        f"**{aggregate['wrong_fix_rate_pct']:.3f}%** | "
        f"**{aggregate['correct_fix_ref_pct']:.3f}%** | "
        f"**{aggregate['ppc_score_3d_50cm_ref_pct']:.3f}%** | "
        f"**{aggregate['ppc_official_score_pct']:.3f}%** | "
        f"**{aggregate['p95_h_m']:.3f} m** |"
    )
    return "\n".join(lines) + "\n"


def main() -> int:
    args = parse_args()
    selected_pos = parse_pos_specs(args.pos)
    rows: list[dict[str, object]] = []
    for city, run in RUNS:
        reference_path = args.dataset_root / city / run / "reference.csv"
        reference = comparison.read_reference_csv(reference_path)
        if selected_pos:
            solution_path = selected_pos[(city, run)]
            if not solution_path.is_absolute():
                solution_path = ROOT / solution_path
            solution = comparison.read_libgnss_pos(solution_path)
            source_key = "pos"
        else:
            solution_path = Path(args.csv_template.format(city=city, run=run))
            if not solution_path.is_absolute():
                solution_path = ROOT / solution_path
            solution = metrics.load_fgo_parity_csv(solution_path, reference)
            source_key = "csv"
        row = metrics.summarize_solution_epochs(
            reference,
            solution,
            fixed_status=4,
            label=f"FGO {city} {run}",
            match_tolerance_s=args.match_tolerance_s,
            solver_wall_time_s=None,
        )
        row["label"] = f"{city}_{run}"
        row[source_key] = str(solution_path)
        rows.append(row)

    keys = (
        "fix_rate_pct",
        "wrong_fix_rate_pct",
        "correct_fix_ref_pct",
        "ppc_score_3d_50cm_ref_pct",
        "ppc_official_score_pct",
        "p95_h_m",
        "p95_abs_up_m",
    )
    payload: dict[str, object] = {
        "runs": rows,
        "macro_mean": {key: macro_mean(rows, key) for key in keys},
        "weighted_official_score_pct": round(
            100.0
            * sum(float(row["ppc_official_score_distance_m"]) for row in rows)
            / sum(float(row["ppc_official_total_distance_m"]) for row in rows),
            6,
        ),
    }
    args.output_json.parent.mkdir(parents=True, exist_ok=True)
    args.output_json.write_text(json.dumps(payload, indent=2) + "\n", encoding="utf-8")
    if args.markdown_output is not None:
        args.markdown_output.parent.mkdir(parents=True, exist_ok=True)
        args.markdown_output.write_text(render_markdown(payload), encoding="utf-8")
    print(
        json.dumps(
            {
                "macro_mean": payload["macro_mean"],
                "weighted_official_score_pct": payload["weighted_official_score_pct"],
            },
            indent=2,
        )
    )
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
