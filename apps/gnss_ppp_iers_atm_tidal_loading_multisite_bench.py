#!/usr/bin/env python3
"""Multi-site driver around ``gnss_ppp_iers_atm_tidal_loading_bench``.

Runs the Phase D-3 atmospheric tidal-loading paired PPP harness across
an arbitrary list of stations and aggregates the per-site comparison
JSONs into one report. ATL coefficients are station-specific, so each
site may provide its own ``atm_tidal_loading`` file; a common file may
be supplied only for synthetic smoke benches.

Site list is supplied via ``--sites <site_config.json>`` whose schema is::

    {
      "common": {
        "nav": "data/igs_2026105/BRDC.rnx",
        "sp3": "data/igs_2026105/IGS_final.sp3",
        "clk": "data/igs_2026105/IGS_final.clk"
      },
      "sites": [
        {
          "name": "TSKB",
          "obs": "data/igs_2026105/TSKB.rnx",
          "atm_tidal_loading": "data/iers/tskb.atl"
        },
        ...
      ]
    }

The script writes:
    <output-dir>/per_site/<NAME>/legacy.pos       — single-site bench output
    <output-dir>/per_site/<NAME>/iers.pos
    <output-dir>/per_site/<NAME>/comparison.json
    <output-dir>/multisite_summary.json           — aggregate report

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
                                "iers_atm_tidal_loading_multisite_bench",
                        help="Directory for per-site outputs and the "
                             "aggregate summary.")
    parser.add_argument("--max-epochs", type=int, default=0,
                        help="Per-site PPP epoch cap (0 == all).")
    parser.add_argument("--mode", choices=("static", "kinematic"),
                        default="static",
                        help="PPP motion model. (default: static)")
    return parser.parse_args()


def resolve_config_path(value: str | os.PathLike[str], config_dir: Path) -> Path:
    """Resolve site-config paths relative to the JSON file or its parents."""
    path = Path(value).expanduser()
    if path.is_absolute():
        return path
    for base in (config_dir, *config_dir.parents, ROOT_DIR):
        candidate = base / path
        if candidate.exists():
            return candidate
    return config_dir / path


def resolve_config_paths(config: dict, config_dir: Path) -> None:
    common = config.get("common", {})
    for key in (
        "nav", "sp3", "clk", "ionex", "dcb", "antex", "blq",
        "atm_tidal_loading",
    ):
        if key in common:
            common[key] = resolve_config_path(common[key], config_dir)
    for site in config.get("sites", []):
        for key in ("obs", "atm_tidal_loading", "blq"):
            if key in site:
                site[key] = resolve_config_path(site[key], config_dir)


def site_value(site: dict, common: dict, key: str):
    return site[key] if key in site else common.get(key)


def run_single_site(
    site: dict,
    common: dict,
    output_dir: Path,
    mode: str,
    max_epochs: int,
) -> dict | None:
    """Invoke ppp-iers-atm-tidal-loading-bench for one site."""
    name = site["name"]
    site_dir = output_dir / "per_site" / name
    site_dir.mkdir(parents=True, exist_ok=True)

    atm_path = site_value(site, common, "atm_tidal_loading")
    if atm_path is None:
        print(f"[multisite-atl] site {name} has no atm_tidal_loading file",
              file=sys.stderr)
        return None

    cmd = [
        "python3",
        str(ROOT_DIR / "apps" / "gnss_ppp_iers_atm_tidal_loading_bench.py"),
        "--obs", str(site["obs"]),
        "--atm-tidal-loading", str(atm_path),
        "--output-dir", str(site_dir),
        "--mode", mode,
    ]
    for key, flag in (
        ("nav", "--nav"),
        ("sp3", "--sp3"),
        ("clk", "--clk"),
        ("ionex", "--ionex"),
        ("dcb", "--dcb"),
        ("antex", "--antex"),
        ("blq", "--blq"),
    ):
        value = site_value(site, common, key)
        if value is not None:
            cmd += [flag, str(value)]

    ocean_station = site_value(site, common, "ocean_loading_station")
    if ocean_station is not None:
        cmd += ["--ocean-loading-station", str(ocean_station)]
    if max_epochs > 0:
        cmd += ["--max-epochs", str(max_epochs)]

    print(f"\n[multisite-atl] === running site {name} ===", file=sys.stderr)
    env = os.environ.copy()
    env["PYTHONPATH"] = (str(ROOT_DIR / "apps")
                        + os.pathsep + env.get("PYTHONPATH", ""))
    try:
        subprocess.run(cmd, check=True, env=env)
    except subprocess.CalledProcessError as exc:
        print(f"[multisite-atl] site {name} failed: {exc}", file=sys.stderr)
        return None

    cmp_path = site_dir / "comparison.json"
    if not cmp_path.exists():
        print(f"[multisite-atl] site {name} produced no comparison.json",
              file=sys.stderr)
        return None
    with cmp_path.open() as fh:
        return json.load(fh)


def aggregate(per_site: dict[str, dict]) -> dict:
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
            "min": min(values),
            "max": max(values),
            "mean": statistics.fmean(values),
            "median": statistics.median(values),
            "abs_max": max(abs(v) for v in values),
            "abs_median": statistics.median([abs(v) for v in values]),
        }
    return aggregate_summary


def render_table(per_site: dict[str, dict]) -> str:
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
    resolve_config_paths(config, args.sites.resolve().parent)

    args.output_dir.mkdir(parents=True, exist_ok=True)

    per_site: dict[str, dict] = {}
    for site in sites:
        name = site["name"]
        result = run_single_site(
            site, common, args.output_dir, args.mode, args.max_epochs)
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
    print(f"\n[multisite-atl] aggregate written to {summary_path}",
          file=sys.stderr)
    return 0 if len(per_site) == len(sites) else 2


if __name__ == "__main__":
    sys.exit(main())
