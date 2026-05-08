#!/usr/bin/env python3
"""Multi-site driver around ``gnss_ppp_iers_pole_tide_bench``.

Runs the Phase D-1 pole-tide truth-bench harness across an arbitrary
list of IGS stations (each with its own RINEX observation file) and
aggregates the per-site comparison JSONs into one report. Used for
flip-default validation: the single-site bench at TSKB establishes
order-of-magnitude correctness, but flipping `use_iers_pole_tide`
to default-on needs evidence at multiple latitudes — pole tide
amplitude scales with sin(2θ) and changes sign across the equator,
so a robust multi-site distribution gates the rollout.

Site list is supplied via ``--sites <site_config.json>`` whose
schema is::

    {
      "common": {
        "nav": "data/igs_2026105/BRDC.rnx",
        "sp3": "data/igs_2026105/IGS_final.sp3",
        "clk": "data/igs_2026105/IGS_final.clk",
        "eop_c04": "data/iers/finals2000A_extended.txt"
      },
      "sites": [
        { "name": "TSKB", "obs": "data/igs_2026105/TSKB.rnx" },
        { "name": "ALGO", "obs": "data/igs_2026105/ALGO00CAN.rnx" },
        ...
      ]
    }

A site row may also override any of ``nav`` / ``sp3`` / ``clk`` /
``eop_c04`` from ``common``. This is how multi-day campaigns are
expressed: each (station, day) becomes its own site row carrying
the day-specific products::

    {
      "common": { "eop_c04": "data/iers/finals2000A.daily" },
      "sites": [
        { "name": "TSKB-103", "obs": "data/igs_2026103/TSKB.rnx",
          "nav": "data/igs_2026103/BRDC.rnx",
          "sp3": "data/igs_2026103/IGS_final.sp3",
          "clk": "data/igs_2026103/IGS_final.clk" },
        { "name": "TSKB-105", ... },
        { "name": "TSKB-107", ... }
      ]
    }

The script writes:
    <output-dir>/per_site/<NAME>/legacy.pos     — single-site bench output
    <output-dir>/per_site/<NAME>/iers.pos
    <output-dir>/per_site/<NAME>/comparison.json
    <output-dir>/multisite_summary.json         — aggregate report

See ``docs/iers-integration-plan.md`` for the rollout plan.
"""

from __future__ import annotations

import argparse
import json
import os
import statistics
import subprocess
import sys
from pathlib import Path

ROOT_DIR = Path(__file__).resolve().parent.parent


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(prog=os.environ.get("GNSS_CLI_NAME"))
    parser.add_argument("--sites", type=Path, required=True,
                        help="JSON site configuration (see module doc).")
    parser.add_argument("--output-dir", type=Path,
                        default=ROOT_DIR / "output" /
                                "iers_pole_tide_multisite_bench",
                        help="Directory for per-site outputs and the "
                             "aggregate summary.")
    parser.add_argument("--max-epochs", type=int, default=0,
                        help="Per-site PPP epoch cap (0 == all).")
    parser.add_argument("--mode", choices=("static", "kinematic"),
                        default="static",
                        help="PPP motion model. (default: static)")
    return parser.parse_args()


def run_single_site(
    config: dict,
    site: dict,
    common: dict,
    output_dir: Path,
    mode: str,
    max_epochs: int,
) -> dict | None:
    """Invoke ppp-iers-pole-tide-bench for one site. Returns the
    per-site comparison.json dict, or None on failure."""
    name = site["name"]
    site_dir = output_dir / "per_site" / name
    site_dir.mkdir(parents=True, exist_ok=True)
    # Per-site values override the campaign-wide `common` block when
    # present — this is the multi-day extension: each site row can
    # carry its own nav/sp3/clk if a campaign spans multiple days.
    nav  = site.get("nav",  common.get("nav"))
    sp3  = site.get("sp3",  common.get("sp3"))
    clk  = site.get("clk",  common.get("clk"))
    eop  = site.get("eop_c04", common.get("eop_c04"))
    cmd = [
        "python3",
        str(ROOT_DIR / "apps" / "gnss_ppp_iers_pole_tide_bench.py"),
        "--obs", str(site["obs"]),
        "--eop-c04", str(eop),
        "--output-dir", str(site_dir),
        "--mode", mode,
    ]
    if nav is not None: cmd += ["--nav", str(nav)]
    if sp3 is not None: cmd += ["--sp3", str(sp3)]
    if clk is not None: cmd += ["--clk", str(clk)]
    if max_epochs > 0:
        cmd += ["--max-epochs", str(max_epochs)]

    print(f"\n[multisite] === running site {name} ===", file=sys.stderr)
    env = os.environ.copy()
    # Make the per-site script find gnss_runtime via PYTHONPATH.
    env["PYTHONPATH"] = (str(ROOT_DIR / "apps")
                        + os.pathsep + env.get("PYTHONPATH", ""))
    try:
        subprocess.run(cmd, check=True, env=env)
    except subprocess.CalledProcessError as exc:
        print(f"[multisite] site {name} failed: {exc}", file=sys.stderr)
        return None

    cmp_path = site_dir / "comparison.json"
    if not cmp_path.exists():
        print(f"[multisite] site {name} produced no comparison.json",
              file=sys.stderr)
        return None
    with cmp_path.open() as fh:
        return json.load(fh)


def aggregate(per_site: dict[str, dict]) -> dict:
    """Pull headline statistics from each per-site comparison.json into
    a single distribution-level summary."""
    metrics = ("max_displacement_m",
               "p95_displacement_m",
               "median_displacement_m",
               "first_epoch_displacement_m",
               "median_dx_m", "median_dy_m", "median_dz_m")
    aggregate_summary: dict = {
        "n_sites": len(per_site),
        "site_names": sorted(per_site.keys()),
        "per_site_headline": {},
        "distribution": {},
    }
    for name in sorted(per_site):
        rec = per_site[name]
        aggregate_summary["per_site_headline"][name] = {
            m: rec.get(m) for m in metrics
        }
    for m in metrics:
        values = [per_site[n].get(m) for n in per_site]
        values = [v for v in values if v is not None]
        if not values:
            continue
        aggregate_summary["distribution"][m] = {
            "min":    min(values),
            "max":    max(values),
            "mean":   statistics.fmean(values),
            "median": statistics.median(values),
            "abs_max":      max(abs(v) for v in values),
            "abs_median":   statistics.median([abs(v) for v in values]),
        }
    return aggregate_summary


def render_table(per_site: dict[str, dict]) -> str:
    """Plain-text per-site headline table for human-readable logs."""
    cols = ("max", "p95", "median", "first", "median_dz")
    keys = ("max_displacement_m",
            "p95_displacement_m",
            "median_displacement_m",
            "first_epoch_displacement_m",
            "median_dz_m")
    lines = ["{:<10}  {:>10}  {:>10}  {:>10}  {:>10}  {:>10}".format(
        "site", *cols)]
    for name in sorted(per_site):
        rec = per_site[name]
        row = [name]
        for k in keys:
            v = rec.get(k)
            row.append(f"{v*1e3:+.3f}mm" if isinstance(v, (int, float))
                       else "    n/a")
        lines.append("{:<10}  {:>10}  {:>10}  {:>10}  {:>10}  {:>10}".format(
            *row))
    return "\n".join(lines)


def main() -> int:
    args = parse_args()
    if not args.sites.exists():
        print(f"sites config not found: {args.sites}", file=sys.stderr)
        return 1
    with args.sites.open() as fh:
        config = json.load(fh)
    common = config.get("common", {})
    sites = config.get("sites", [])
    if not sites:
        print("sites config has no 'sites' list", file=sys.stderr)
        return 1

    args.output_dir.mkdir(parents=True, exist_ok=True)

    per_site: dict[str, dict] = {}
    for site in sites:
        name = site["name"]
        result = run_single_site(
            config, site, common, args.output_dir, args.mode, args.max_epochs)
        if result is not None:
            per_site[name] = result

    summary = aggregate(per_site)
    summary["output_dir"] = str(args.output_dir)
    summary_path = args.output_dir / "multisite_summary.json"
    summary_path.write_text(json.dumps(summary, indent=2, sort_keys=True)
                             + "\n", encoding="utf-8")
    print(json.dumps(summary, indent=2, sort_keys=True))

    if per_site:
        print("\n" + render_table(per_site))
    print(f"\n[multisite] aggregate written to {summary_path}",
          file=sys.stderr)
    return 0 if len(per_site) == len(sites) else 2


if __name__ == "__main__":
    sys.exit(main())
