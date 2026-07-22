#!/usr/bin/env python3
"""Build the audited PPC/Kaiyodai/GICI goal summary and README figures."""

from __future__ import annotations

import argparse
import json
from pathlib import Path


RUNS = (
    ("tokyo_run1", "Tokyo 1", "tokyo1_compare.json"),
    ("tokyo_run2", "Tokyo 2", "tokyo2_compare.json"),
    ("tokyo_run3", "Tokyo 3", "tokyo3_compare.json"),
    ("nagoya_run1", "Nagoya 1", "nagoya1_compare.json"),
    ("nagoya_run2", "Nagoya 2", "nagoya2_compare.json"),
    ("nagoya_run3", "Nagoya 3", "nagoya3_compare.json"),
)

KAIYODAI_PAPER = (
    "https://www.denshi.e.kaiyodai.ac.jp/wp-content/uploads/pdf/content/"
    "2024okada,sasaki,ando.pdf"
)
TURING_SLIDES = (
    "https://www.denshi.e.kaiyodai.ac.jp/wp-content/uploads/2025/01/"
    "Turing-Inc.-Tight-coupling-Factor-Graph-%E4%BA%95%E4%B8%8A%E6%A7%98-"
    "%E5%9C%A7%E7%B8%AE.pdf"
)
GICI_REPOSITORY = "https://github.com/inuex35/gici-open"


def parse_args(argv: list[str] | None = None) -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--lib-matrix", type=Path, required=True)
    gici = parser.add_mutually_exclusive_group(required=True)
    gici.add_argument("--gici-matrix", type=Path)
    gici.add_argument("--gici-results-dir", type=Path)
    parser.add_argument("--nagoya1-public-summary", type=Path, required=True)
    parser.add_argument("--summary-json", type=Path, required=True)
    parser.add_argument("--comparison-png", type=Path, required=True)
    parser.add_argument("--targets-png", type=Path, required=True)
    parser.add_argument(
        "--gici-commit", default="e7666110a88d22e08aad24345a253564af9b8024"
    )
    return parser.parse_args(argv)


def read_object(path: Path) -> dict[str, object]:
    payload = json.loads(path.read_text(encoding="utf-8"))
    if not isinstance(payload, dict):
        raise SystemExit(f"{path}: expected a JSON object")
    return payload


def number(mapping: dict[str, object], key: str, context: str) -> float:
    value = mapping.get(key)
    if not isinstance(value, (int, float)):
        raise SystemExit(f"{context}: missing numeric {key!r}")
    return float(value)


def macro(rows: list[dict[str, object]], key: str) -> float:
    return sum(number(row, key, "run") for row in rows) / len(rows)


def portable_artifact_path(value: object) -> str:
    text = str(value or "").replace("\\", "/")
    marker = "/output/"
    if marker in text:
        return "output/" + text.split(marker, 1)[1]
    return text


def build_payload(args: argparse.Namespace) -> dict[str, object]:
    matrix = read_object(args.lib_matrix)
    raw_lib_runs = matrix.get("runs")
    if not isinstance(raw_lib_runs, list):
        raise SystemExit(f"{args.lib_matrix}: missing runs list")
    lib_by_label = {
        str(row.get("label")): row for row in raw_lib_runs if isinstance(row, dict)
    }
    gici_by_label: dict[str, dict[str, object]] = {}
    if args.gici_matrix is not None:
        gici_matrix = read_object(args.gici_matrix)
        raw_gici_runs = gici_matrix.get("runs")
        if not isinstance(raw_gici_runs, list):
            raise SystemExit(f"{args.gici_matrix}: missing runs list")
        gici_by_label = {
            str(row.get("label")): row
            for row in raw_gici_runs
            if isinstance(row, dict)
        }

    runs: list[dict[str, object]] = []
    gici_rows: list[dict[str, object]] = []
    for key, label, filename in RUNS:
        lib = lib_by_label.get(key)
        if lib is None:
            raise SystemExit(f"{args.lib_matrix}: missing {key}")
        if args.gici_matrix is not None:
            gici = gici_by_label.get(key)
            if gici is None:
                raise SystemExit(f"{args.gici_matrix}: missing {key}")
        else:
            competitor = read_object(args.gici_results_dir / filename)
            solutions = competitor.get("solutions")
            if (
                not isinstance(solutions, list)
                or len(solutions) != 1
                or not isinstance(solutions[0], dict)
            ):
                raise SystemExit(f"{filename}: expected exactly one GICI solution")
            gici = solutions[0]
        gici_rows.append(gici)
        runs.append(
            {
                "key": key,
                "label": label,
                "libgnss": {
                    "pos": portable_artifact_path(lib.get("pos", "")),
                    **{
                        metric: number(lib, metric, key)
                        for metric in (
                            "fix_rate_pct",
                            "wrong_fix_rate_pct",
                            "correct_fix_ref_pct",
                            "ppc_score_3d_50cm_ref_pct",
                            "ppc_official_score_pct",
                            "p95_h_m",
                            "p95_abs_up_m",
                        )
                    },
                },
                "gici": {
                    "path": portable_artifact_path(gici.get("path", "")),
                    **{
                        metric: number(gici, metric, filename)
                        for metric in (
                            "fix_rate_pct",
                            "wrong_fix_rate_pct",
                            "correct_fix_ref_pct",
                            "ppc_score_3d_50cm_ref_pct",
                            "ppc_official_score_pct",
                            "p95_h_m",
                            "p95_abs_up_m",
                        )
                    },
                },
            }
        )

    lib_macro_raw = matrix.get("macro_mean")
    if not isinstance(lib_macro_raw, dict):
        raise SystemExit(f"{args.lib_matrix}: missing macro_mean")
    metric_keys = (
        "fix_rate_pct",
        "wrong_fix_rate_pct",
        "correct_fix_ref_pct",
        "ppc_score_3d_50cm_ref_pct",
        "ppc_official_score_pct",
        "p95_h_m",
        "p95_abs_up_m",
    )
    lib_macro = {key: number(lib_macro_raw, key, "lib macro") for key in metric_keys}
    gici_macro = {key: macro(gici_rows, key) for key in metric_keys}

    nagoya_summary = read_object(args.nagoya1_public_summary)
    targets = [
        {
            "key": "tokyo1_fix",
            "label": "Tokyo 1 FIX",
            "target_pct": 80.8,
            "achieved_pct": number(lib_by_label["tokyo_run1"], "fix_rate_pct", "Tokyo 1"),
            "profile": "six-course selected matrix",
        },
        {
            "key": "nagoya1_fix",
            "label": "Nagoya 1 FIX",
            "target_pct": 85.1,
            "achieved_pct": number(nagoya_summary, "fix_rate_pct", "Nagoya 1 public profile"),
            "profile": "FIX-target profile (separate from six-course matrix)",
            "wrong_fix_rate_pct": number(
                nagoya_summary, "wrong_fix_rate_pct", "Nagoya 1 public profile"
            ),
            "p95_h_m": number(nagoya_summary, "p95_h_m", "Nagoya 1 public profile"),
        },
        {
            "key": "ppc_official",
            "label": "PPC official",
            "target_pct": 78.7,
            "achieved_pct": number(matrix, "weighted_official_score_pct", "matrix"),
            "profile": "distance-weighted six-course selected matrix",
        },
    ]
    return {
        "schema_version": 1,
        "evaluation": {
            "dataset": "taroz/PPC-Dataset, Tokyo/Nagoya runs 1-3",
            "fixed_status": 4,
            "correct_fix_threshold_3d_m": 0.5,
            "official_score_threshold_3d_m": 0.5,
            "reference_used_by_runtime_selector": False,
            "status_labels_preserved_by_position_selectors": True,
        },
        "provenance": {
            "lib_matrix": str(args.lib_matrix),
            "gici_matrix": str(args.gici_matrix) if args.gici_matrix is not None else None,
            "gici_results_dir": (
                str(args.gici_results_dir) if args.gici_results_dir is not None else None
            ),
            "gici_repository": GICI_REPOSITORY,
            "gici_commit": args.gici_commit,
            "gici_license_boundary": "GPL-3.0 external executable; NMEA output only",
            "kaiyodai_paper": KAIYODAI_PAPER,
            "turing_slides": TURING_SLIDES,
        },
        "runs": runs,
        "macro_mean": {"libgnss": lib_macro, "gici": gici_macro},
        "weighted_official_score_pct": number(
            matrix, "weighted_official_score_pct", "matrix"
        ),
        "targets": targets,
    }


def render_comparison(payload: dict[str, object], output: Path) -> None:
    import matplotlib.pyplot as plt
    import numpy as np

    runs = payload["runs"]
    assert isinstance(runs, list)
    aggregate = payload["macro_mean"]
    assert isinstance(aggregate, dict)
    labels = [str(row["label"]) for row in runs] + ["Macro"]
    metrics = (
        ("fix_rate_pct", "FIX rate"),
        ("correct_fix_ref_pct", "Correct FIX / reference"),
        ("ppc_official_score_pct", "PPC official score"),
    )
    fig, axes = plt.subplots(1, 3, figsize=(15.5, 5.8), dpi=120, sharey=True)
    x = np.arange(len(labels))
    width = 0.38
    for ax, (key, title) in zip(axes, metrics):
        lib_values = [float(row["libgnss"][key]) for row in runs]
        gici_values = [float(row["gici"][key]) for row in runs]
        lib_values.append(float(aggregate["libgnss"][key]))
        gici_values.append(float(aggregate["gici"][key]))
        ax.bar(x - width / 2, lib_values, width, label="libgnss++", color="#0f766e")
        ax.bar(x + width / 2, gici_values, width, label="gici-open", color="#64748b")
        ax.set_title(title, weight="bold")
        ax.set_xticks(x, labels, rotation=35, ha="right")
        ax.set_ylim(0, 100)
        ax.grid(axis="y", alpha=0.22)
        ax.set_axisbelow(True)
        ax.set_ylabel("Percent")
    axes[0].legend(loc="upper left", frameon=False)
    fig.suptitle(
        "PPC 2024 six-run reproduction — same data and metric definitions",
        fontsize=16,
        weight="bold",
    )
    fig.text(
        0.5,
        0.015,
        "Runtime selection is reference-free; baseline status labels are preserved.",
        ha="center",
        color="#475467",
    )
    fig.tight_layout(rect=(0, 0.055, 1, 0.92))
    output.parent.mkdir(parents=True, exist_ok=True)
    fig.savefig(output, bbox_inches="tight")
    plt.close(fig)


def render_targets(payload: dict[str, object], output: Path) -> None:
    import matplotlib.pyplot as plt
    import numpy as np

    targets = payload["targets"]
    assert isinstance(targets, list)
    labels = [str(row["label"]) for row in targets]
    target_values = [float(row["target_pct"]) for row in targets]
    achieved = [float(row["achieved_pct"]) for row in targets]
    y = np.arange(len(labels))
    fig, ax = plt.subplots(figsize=(11.5, 5.2), dpi=120)
    ax.barh(y, target_values, height=0.58, color="#d0d5dd", label="Published target")
    ax.barh(y, achieved, height=0.34, color="#0f766e", label="libgnss++ achieved")
    for index, (target, actual) in enumerate(zip(target_values, achieved)):
        ax.text(actual + 0.25, index, f"{actual:.3f}%", va="center", weight="bold")
        ax.plot([target, target], [index - 0.34, index + 0.34], color="#b42318", lw=2)
    ax.set_yticks(y, labels)
    ax.invert_yaxis()
    ax.set_xlim(0, 92)
    ax.set_xlabel("Percent")
    ax.grid(axis="x", alpha=0.22)
    ax.set_axisbelow(True)
    handles, legend_labels = ax.get_legend_handles_labels()
    fig.legend(
        handles,
        legend_labels,
        loc="lower center",
        ncol=2,
        frameon=False,
        bbox_to_anchor=(0.5, 0.005),
    )
    fig.suptitle("PPC public targets cleared", y=0.96, fontsize=17, weight="bold")
    fig.text(
        0.5,
        0.885,
        "Nagoya 1 uses the audited FIX-target profile; PPC official uses the six-run distance-weighted matrix.",
        ha="center",
        color="#475467",
    )
    fig.tight_layout(rect=(0, 0.08, 1, 0.85))
    output.parent.mkdir(parents=True, exist_ok=True)
    fig.savefig(output, bbox_inches="tight")
    plt.close(fig)


def main(argv: list[str] | None = None) -> int:
    args = parse_args(argv)
    payload = build_payload(args)
    args.summary_json.parent.mkdir(parents=True, exist_ok=True)
    args.summary_json.write_text(
        json.dumps(payload, indent=2, sort_keys=True) + "\n", encoding="utf-8"
    )
    render_comparison(payload, args.comparison_png)
    render_targets(payload, args.targets_png)
    print(
        json.dumps(
            {
                "weighted_official_score_pct": payload["weighted_official_score_pct"],
                "lib_macro_fix_pct": payload["macro_mean"]["libgnss"]["fix_rate_pct"],
                "gici_macro_fix_pct": payload["macro_mean"]["gici"]["fix_rate_pct"],
            },
            indent=2,
        )
    )
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
