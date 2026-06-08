#!/usr/bin/env python3
"""Summarize per-DD RTK residual diagnostics emitted by gnss solve.

Thin CLI orchestrator: argument parsing, dispatch into the
``gnss_dd_residuals_*`` helper modules, and result printing.  The actual
record schema, statistics, and rendering live in the sibling modules so each
layer can be unit-tested in isolation:

| Concern                | Module                                |
|------------------------|---------------------------------------|
| Record schema helpers  | ``gnss_dd_residuals_records``         |
| CSV I/O                | ``gnss_dd_residuals_io``              |
| Filtering              | ``gnss_dd_residuals_filtering``       |
| Statistics             | ``gnss_dd_residuals_statistics``      |
| Summary assembly       | ``gnss_dd_residuals_summary``         |
| HTML/SVG rendering     | ``gnss_dd_residuals_html``            |
"""

from __future__ import annotations

import argparse
import json
import os
from pathlib import Path
from typing import Any

from gnss_dd_residuals_filtering import filter_records
from gnss_dd_residuals_html import write_html_report
from gnss_dd_residuals_io import read_records, write_top_pairs_csv
from gnss_dd_residuals_summary import summarize_records


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(prog=os.environ.get("GNSS_CLI_NAME"))
    parser.add_argument("input_csv", type=Path, help="Input --dd-residuals-csv file.")
    parser.add_argument("--summary-json", type=Path, default=None, help="Optional summary JSON path.")
    parser.add_argument("--top-pairs-csv", type=Path, default=None, help="Optional worst-pair CSV path.")
    parser.add_argument(
        "--html-report", type=Path, default=None, help="Optional self-contained HTML report path."
    )
    parser.add_argument("--top-n", type=int, default=10, help="Number of worst pairs to keep.")
    parser.add_argument(
        "--kind",
        choices=("all", "phase", "code"),
        default="all",
        help="Filter measurements before summarizing.",
    )
    parser.add_argument(
        "--frequency-index",
        type=int,
        default=None,
        help="Filter one frequency index before summarizing.",
    )
    parser.add_argument(
        "--require-phase-p95-max",
        type=float,
        default=None,
        help="Fail if phase p95 absolute residual exceeds this value in meters.",
    )
    parser.add_argument(
        "--require-code-p95-max",
        type=float,
        default=None,
        help="Fail if code p95 absolute residual exceeds this value in meters.",
    )
    parser.add_argument(
        "--require-max-abs-residual-max",
        type=float,
        default=None,
        help="Fail if max absolute residual exceeds this value in meters.",
    )
    parser.add_argument(
        "--require-suppressed-rows-max",
        type=int,
        default=None,
        help="Fail if outlier-threshold suppressed row count exceeds this value.",
    )
    return parser.parse_args()


def enforce_requirements(summary: dict[str, Any], args: argparse.Namespace) -> None:
    """Validate the summary against the optional CI thresholds.

    Raises ``SystemExit`` with the concatenated failure list when any of
    the configured ``--require-*`` thresholds are exceeded.  Threshold
    failures are reported as a single error block so CI surfaces all of
    them in one shot instead of one-per-rerun.
    """

    failures: list[str] = []

    phase_p95 = summary["by_kind"]["phase"]["p95_abs_residual_m"]
    if args.require_phase_p95_max is not None:
        if phase_p95 is None or float(phase_p95) > args.require_phase_p95_max:
            observed = "n/a" if phase_p95 is None else f"{float(phase_p95):.6f}"
            failures.append(
                f"phase p95 abs residual {observed} m > {args.require_phase_p95_max:.6f} m"
            )

    code_p95 = summary["by_kind"]["code"]["p95_abs_residual_m"]
    if args.require_code_p95_max is not None:
        if code_p95 is None or float(code_p95) > args.require_code_p95_max:
            observed = "n/a" if code_p95 is None else f"{float(code_p95):.6f}"
            failures.append(
                f"code p95 abs residual {observed} m > {args.require_code_p95_max:.6f} m"
            )

    max_abs = summary["overall"]["max_abs_residual_m"]
    if args.require_max_abs_residual_max is not None:
        if max_abs is None or float(max_abs) > args.require_max_abs_residual_max:
            observed = "n/a" if max_abs is None else f"{float(max_abs):.6f}"
            failures.append(
                f"max abs residual {observed} m > {args.require_max_abs_residual_max:.6f} m"
            )

    suppressed = int(summary["overall"]["suppressed_rows"])
    if args.require_suppressed_rows_max is not None and suppressed > args.require_suppressed_rows_max:
        failures.append(
            f"suppressed rows {suppressed} > {args.require_suppressed_rows_max}"
        )

    if failures:
        raise SystemExit(
            "DD residual checks failed:\n" + "\n".join(f"  - {item}" for item in failures)
        )


def main() -> int:
    args = parse_args()
    if args.top_n < 0:
        raise SystemExit("--top-n must be non-negative")
    records = filter_records(read_records(args.input_csv), args.kind, args.frequency_index)
    if not records:
        raise SystemExit(f"No DD residual rows matched {args.input_csv}")

    summary = summarize_records(records, top_n=args.top_n)
    if args.summary_json is not None:
        args.summary_json.parent.mkdir(parents=True, exist_ok=True)
        args.summary_json.write_text(
            json.dumps(summary, indent=2, sort_keys=True) + "\n",
            encoding="utf-8",
        )
    if args.top_pairs_csv is not None:
        write_top_pairs_csv(args.top_pairs_csv, list(summary["top_pairs"]))
    if args.html_report is not None:
        write_html_report(args.html_report, summary)

    enforce_requirements(summary, args)

    print("DD residual summary")
    print(f"  rows: {summary['overall']['rows']}")
    print(f"  epochs: {summary['coverage']['epochs']}")
    print(f"  satellite_pairs: {summary['coverage']['satellite_pairs']}")
    print(f"  pair_frequency_kind_tracks: {summary['coverage']['pair_frequency_kind_tracks']}")
    print(f"  phase_p95_abs_m: {summary['by_kind']['phase']['p95_abs_residual_m']}")
    print(f"  code_p95_abs_m: {summary['by_kind']['code']['p95_abs_residual_m']}")
    print(f"  phase_p95_sigma: {summary['by_kind']['phase']['p95_abs_normalized_residual']}")
    print(f"  code_p95_sigma: {summary['by_kind']['code']['p95_abs_normalized_residual']}")
    print(f"  max_abs_m: {summary['overall']['max_abs_residual_m']}")
    if args.summary_json is not None:
        print(f"  summary: {args.summary_json}")
    if args.top_pairs_csv is not None:
        print(f"  top_pairs: {args.top_pairs_csv}")
    if args.html_report is not None:
        print(f"  html_report: {args.html_report}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
