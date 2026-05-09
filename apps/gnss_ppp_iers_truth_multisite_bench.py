#!/usr/bin/env python3
"""Multi-site driver around ``gnss_ppp_iers_truth_bench``.

Runs the IERS Conventions 2010 end-to-end truth bench across an
arbitrary list of IGS stations and aggregates the per-site
residual JSONs into one report. Mirrors the pole-tide /
sub-daily-EOP / atm-tidal multi-site drivers (PR #69 / #75 / #76).

The site config schema mirrors the other multi-site drivers; each
site row may override any of ``nav`` / ``sp3`` / ``clk`` /
``eop_c04`` / ``blq`` / ``ocean_loading_station`` from the
campaign-wide ``common`` block::

    {
      "common": {
        "nav": "data/igs_2026105/BRDC.rnx",
        "sp3": "data/igs_2026105/IGS_final.sp3",
        "clk": "data/igs_2026105/IGS_final.clk",
        "eop_c04": "data/iers/finals2000A.daily"
      },
      "sites": [
        { "name": "TSKB", "obs": "data/igs_2026105/TSKB.rnx" },
        { "name": "GRAZ", "obs": "data/igs_2026105/GRAZ00AUT.rnx" }
      ]
    }

Each site row may also carry an explicit ``reference_xyz`` if the
RINEX header's ``APPROX POSITION XYZ`` is not the right truth
reference (e.g. for non-IGS stations or when an ITRF2020 update
needs to be applied).

The script writes:
    <output-dir>/per_site/<NAME>/iers_on.pos
    <output-dir>/per_site/<NAME>/iers_off.pos    (when --ab)
    <output-dir>/per_site/<NAME>/comparison.json
    <output-dir>/multisite_summary.json
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
                                "iers_truth_multisite_bench",
                        help="Directory for per-site outputs and the "
                             "aggregate summary.")
    parser.add_argument("--max-epochs", type=int, default=0,
                        help="Per-site PPP epoch cap (0 == all).")
    parser.add_argument("--converged-tail-epochs", type=int, default=600,
                        help="Average over the last N epochs only "
                             "(default 600).")
    parser.add_argument("--mode", choices=("static", "kinematic"),
                        default="static",
                        help="PPP motion model. (default: static)")
    parser.add_argument("--ab", action="store_true",
                        help="Per site, also run an all-IERS-OFF arm "
                             "and report on/off residual delta.")
    return parser.parse_args()


def run_single_site(
    site: dict,
    common: dict,
    output_dir: Path,
    mode: str,
    max_epochs: int,
    tail: int,
    ab: bool,
) -> dict | None:
    """Invoke ppp-iers-truth-bench for one site. Returns the
    per-site comparison.json dict, or None on failure."""
    name = site["name"]
    site_dir = output_dir / "per_site" / name
    site_dir.mkdir(parents=True, exist_ok=True)
    nav  = site.get("nav",  common.get("nav"))
    sp3  = site.get("sp3",  common.get("sp3"))
    clk  = site.get("clk",  common.get("clk"))
    eop  = site.get("eop_c04", common.get("eop_c04"))
    blq  = site.get("blq",  common.get("blq"))
    blq_station = site.get("ocean_loading_station",
                           common.get("ocean_loading_station"))
    ref_xyz = site.get("reference_xyz", common.get("reference_xyz"))
    cmd = [
        "python3",
        str(ROOT_DIR / "apps" / "gnss_ppp_iers_truth_bench.py"),
        "--obs", str(site["obs"]),
        "--output-dir", str(site_dir),
        "--mode", mode,
        "--converged-tail-epochs", str(tail),
    ]
    if nav is not None: cmd += ["--nav", str(nav)]
    if sp3 is not None: cmd += ["--sp3", str(sp3)]
    if clk is not None: cmd += ["--clk", str(clk)]
    if eop is not None: cmd += ["--eop-c04", str(eop)]
    if blq is not None:
        cmd += ["--blq", str(blq)]
        if blq_station is not None:
            cmd += ["--ocean-loading-station", str(blq_station)]
    if ref_xyz is not None:
        cmd += ["--reference-xyz", str(ref_xyz[0]), str(ref_xyz[1]),
                str(ref_xyz[2])]
    if max_epochs > 0:
        cmd += ["--max-epochs", str(max_epochs)]
    if ab:
        cmd += ["--ab"]

    print(f"\n[multisite] === running site {name} ===", file=sys.stderr)
    env = os.environ.copy()
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


def aggregate(per_site: dict[str, dict], ab: bool) -> dict:
    """Pull headline statistics from each per-site comparison.json into
    a single distribution-level summary."""
    headline_metrics = ("horizontal_enu_m", "vertical_enu_m",
                        "distance_3d_m")
    aggregate_summary: dict = {
        "n_sites": len(per_site),
        "site_names": sorted(per_site.keys()),
        "per_site_headline": {},
        "distribution_iers_on": {},
    }
    if ab:
        aggregate_summary["distribution_iers_off"] = {}
        aggregate_summary["distribution_delta"] = {}
    for name in sorted(per_site):
        rec = per_site[name]
        on = rec.get("iers_on", {})
        off = rec.get("iers_off", {})
        delta = rec.get("delta", {})
        aggregate_summary["per_site_headline"][name] = {
            "iers_on": {m: on.get(m) for m in headline_metrics},
        }
        if ab:
            aggregate_summary["per_site_headline"][name]["iers_off"] = {
                m: off.get(m) for m in headline_metrics
            }
            aggregate_summary["per_site_headline"][name]["delta"] = {
                "horizontal_enu_m_off_minus_on":
                    delta.get("horizontal_enu_m_off_minus_on"),
                "vertical_enu_m_off_minus_on":
                    delta.get("vertical_enu_m_off_minus_on"),
                "distance_3d_m_off_minus_on":
                    delta.get("distance_3d_m_off_minus_on"),
                "iers_on_closer_to_truth":
                    delta.get("iers_on_closer_to_truth"),
            }

    def _populate(block_name: str, source_key: str) -> None:
        for m in headline_metrics:
            values = [per_site[n].get(source_key, {}).get(m)
                      for n in per_site]
            values = [v for v in values if v is not None]
            if not values:
                continue
            aggregate_summary[block_name][m] = {
                "min":          min(values),
                "max":          max(values),
                "mean":         statistics.fmean(values),
                "median":       statistics.median(values),
                "abs_max":      max(abs(v) for v in values),
                "abs_median":   statistics.median([abs(v) for v in values]),
            }

    _populate("distribution_iers_on", "iers_on")
    if ab:
        _populate("distribution_iers_off", "iers_off")
        for m in ("horizontal_enu_m_off_minus_on",
                  "vertical_enu_m_off_minus_on",
                  "distance_3d_m_off_minus_on"):
            values = [per_site[n].get("delta", {}).get(m)
                      for n in per_site]
            values = [v for v in values if v is not None]
            if not values:
                continue
            aggregate_summary["distribution_delta"][m] = {
                "min":          min(values),
                "max":          max(values),
                "mean":         statistics.fmean(values),
                "median":       statistics.median(values),
                "abs_max":      max(abs(v) for v in values),
                "abs_median":   statistics.median([abs(v) for v in values]),
            }
    return aggregate_summary


def render_table(per_site: dict[str, dict], ab: bool) -> str:
    """Plain-text per-site headline table for human-readable logs."""
    if ab:
        cols = ("on_h", "on_v", "on_3d", "off_3d", "delta_3d", "winner")
    else:
        cols = ("on_h", "on_v", "on_3d")
    lines = ["{:<10}  ".format("site") +
             "  ".join("{:>10}".format(c) for c in cols)]
    for name in sorted(per_site):
        rec = per_site[name]
        on = rec.get("iers_on", {})
        off = rec.get("iers_off", {})
        delta = rec.get("delta", {})
        if ab:
            cells = [
                f"{on.get('horizontal_enu_m', float('nan')):.4f}m",
                f"{on.get('vertical_enu_m', float('nan')):.4f}m",
                f"{on.get('distance_3d_m', float('nan')):.4f}m",
                f"{off.get('distance_3d_m', float('nan')):.4f}m",
                f"{delta.get('distance_3d_m_off_minus_on', float('nan'))*1e3:+.3f}mm",
                "iers" if delta.get("iers_on_closer_to_truth")
                       else ("legacy" if delta.get("iers_on_closer_to_truth")
                                         is False else "n/a"),
            ]
        else:
            cells = [
                f"{on.get('horizontal_enu_m', float('nan')):.4f}m",
                f"{on.get('vertical_enu_m', float('nan')):.4f}m",
                f"{on.get('distance_3d_m', float('nan')):.4f}m",
            ]
        lines.append("{:<10}  ".format(name) +
                     "  ".join("{:>10}".format(c) for c in cells))
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
            site, common, args.output_dir, args.mode,
            args.max_epochs, args.converged_tail_epochs, args.ab)
        if result is not None:
            per_site[name] = result

    summary = aggregate(per_site, args.ab)
    summary["output_dir"] = str(args.output_dir)
    summary_path = args.output_dir / "multisite_summary.json"
    summary_path.write_text(json.dumps(summary, indent=2, sort_keys=True)
                             + "\n", encoding="utf-8")
    print(json.dumps(summary, indent=2, sort_keys=True))

    if per_site:
        print("\n" + render_table(per_site, args.ab))
    print(f"\n[multisite] aggregate written to {summary_path}",
          file=sys.stderr)
    return 0 if len(per_site) == len(sites) else 2


if __name__ == "__main__":
    sys.exit(main())
