#!/usr/bin/env python3
"""Recompute the truth-free per-drive picks of the GSDC official recipe (since Kaggle ref 57733487).

The recipe pins each drive's argv; this script audits how its 22 changed drives
were chosen.  From the 57678818 output and the native variant outputs of every
drive it applies the rule that was fixed on train before it touched test (see
"Truth-free rule on train and road snap" in docs/gsdc2023_native_taroz_goal.md):

1. Height map: a drive with a height map drops it when the median height
   spread of the map points within 15 m of its base trajectory is >= 0.3 m;
   it then chooses between ``nomap`` and ``nomap+motion``.
2. XXVV motion: sm-a325f, samsunga325g and sm-a205u always take it.
3. Otherwise the candidate with the lowest prres wins (base, motion, tdcp;
   a tie keeps the earlier one).

prres is the median over epochs of the median |corrected raw pseudorange -
geometric range| after removing a per-epoch clock per constellation and
signal (elevation > 15 deg, C/N0 > 25 dB-Hz, at least 6 rows per epoch).  The
trajectory height comes from the drive's GNSS-first stage, or from the raw
WLS positions where no stage was exported.  No ground truth is read.

Layout of ``--variants-dir``: ``<variant>/<course>/<phone>/solution.csv`` with
variant in base, motion, tdcp, nomap, nomap+motion; a missing file means the
candidate was not run.  ``base/<course>/<phone>/summary.json.gnss-first.csv``
supplies the height when present.  ``--maps-dir`` holds the height maps of the
57678818 recipe as ``<course>__<phone>.csv``.
"""

from __future__ import annotations

import argparse
import json
from pathlib import Path
import sys
from typing import Any, Mapping

ROOT_DIR = Path(__file__).resolve().parents[3]
DEFAULT_RECIPE = ROOT_DIR / "configs" / "reproduce" / "gsdc_official_recipe.json"
MOTION_PHONES = {"sm-a325f", "samsunga325g", "sm-a205u"}
MAP_SPREAD_LIMIT_M = 0.3
LABELS = {
    "motion": "XXVV motion",
    "tdcp": "source TDCP metre sigma + Phase184 Huber",
    "nomap": "no height map",
    "nomap+motion": "no height map + XXVV motion",
}
C, OMEGA = 299792458.0, 7.2921151467e-5
WGS84_A, WGS84_F = 6378137.0, 1 / 298.257223563
EARTH_RADIUS_M = 6371008.8


def choose(phone: str, prres: Mapping[str, float], map_spread_m: float | None) -> str:
    """The frozen rule: ``prres`` maps candidate name to its metric."""
    if map_spread_m is not None and map_spread_m >= MAP_SPREAD_LIMIT_M:
        names, motion = ["nomap", "nomap+motion"], "nomap+motion"
    else:
        names, motion = [name for name in ("base", "motion", "tdcp") if name in prres], "motion"
    if phone in MOTION_PHONES:
        return motion
    scores = {name: prres[name] for name in names}
    return min(scores, key=scores.get)


def _llh_to_ecef(lat_deg: Any, lon_deg: Any, height_m: Any) -> Any:
    import numpy as np  # noqa: PLC0415

    e2 = WGS84_F * (2 - WGS84_F)
    lat, lon = np.radians(lat_deg), np.radians(lon_deg)
    n = WGS84_A / np.sqrt(1 - e2 * np.sin(lat) ** 2)
    return np.c_[(n + height_m) * np.cos(lat) * np.cos(lon), (n + height_m) * np.cos(lat) * np.sin(lon),
                 (n * (1 - e2) + height_m) * np.sin(lat)]


def _ecef_height(xyz: Any) -> Any:
    import numpy as np  # noqa: PLC0415

    e2 = WGS84_F * (2 - WGS84_F)
    p = np.hypot(xyz[:, 0], xyz[:, 1])
    lat = np.arctan2(xyz[:, 2], p * (1 - e2))
    height = p * 0.0
    for _ in range(5):
        n = WGS84_A / np.sqrt(1 - e2 * np.sin(lat) ** 2)
        height = p / np.cos(lat) - n
        lat = np.arctan2(xyz[:, 2], p * (1 - e2 * n / (n + height)))
    return height


def wls_heights(device_gnss: Path) -> Any:
    """Per-epoch median raw WLS position, in the GNSS-first stage CSV layout."""
    import pandas as pd  # noqa: PLC0415

    columns = ["utcTimeMillis", "WlsPositionXEcefMeters", "WlsPositionYEcefMeters", "WlsPositionZEcefMeters"]
    frame = pd.read_csv(device_gnss, usecols=columns).groupby("utcTimeMillis").median().reset_index()
    frame.insert(0, "phone", "")
    return frame.rename(columns={"utcTimeMillis": "UnixTimeMillis"})


def prres(device_gnss: Path, heights: Any, solution: Path) -> float:
    """Median over epochs of the median |pseudorange residual| (see the module docstring)."""
    import numpy as np  # noqa: PLC0415
    import pandas as pd  # noqa: PLC0415

    columns = ["utcTimeMillis", "ConstellationType", "SignalType", "RawPseudorangeMeters", "SvClockBiasMeters",
               "IsrbMeters", "IonosphericDelayMeters", "TroposphericDelayMeters", "SvPositionXEcefMeters",
               "SvPositionYEcefMeters", "SvPositionZEcefMeters", "SvElevationDegrees", "Cn0DbHz"]
    raw = pd.read_csv(device_gnss, usecols=columns)
    raw = raw[(raw.SvElevationDegrees > 15) & (raw.Cn0DbHz > 25) & raw.RawPseudorangeMeters.notna()]
    keys = np.sort(raw.utcTimeMillis.unique())
    height = pd.Series(_ecef_height(heights.iloc[:, 2:5].values), index=heights.UnixTimeMillis.values)
    height = height[~height.index.duplicated()].reindex(keys).interpolate(limit_direction="both")
    sat = raw[["SvPositionXEcefMeters", "SvPositionYEcefMeters", "SvPositionZEcefMeters"]].values
    corrected = (raw.RawPseudorangeMeters + raw.SvClockBiasMeters - raw.IsrbMeters
                 - raw.IonosphericDelayMeters - raw.TroposphericDelayMeters).values
    group = (raw.ConstellationType.astype(str) + raw.SignalType).values
    index = pd.Series(np.arange(len(keys)), index=keys)[raw.utcTimeMillis.values].values
    track = pd.read_csv(solution).drop_duplicates("UnixTimeMillis").set_index("UnixTimeMillis").reindex(keys)
    position = _llh_to_ecef(track.LatitudeDegrees.values, track.LongitudeDegrees.values, height.values)[index]
    ranges = (np.linalg.norm(sat - position, axis=1)
              + OMEGA * (sat[:, 0] * position[:, 1] - sat[:, 1] * position[:, 0]) / C)
    residual = pd.DataFrame({"k": index, "g": group, "r": corrected - ranges}).dropna()
    residual["r"] -= residual.groupby(["k", "g"]).r.transform("median")
    residual = residual[residual.groupby("k").r.transform("size") >= 6]
    return float(residual.assign(a=residual.r.abs()).groupby("k").a.median().median())


def map_spread(height_map: Path, solution: Path) -> float:
    """Median height spread (max - min) of the map points within 15 m of each epoch."""
    import numpy as np  # noqa: PLC0415
    import pandas as pd  # noqa: PLC0415
    from scipy.spatial import cKDTree  # noqa: PLC0415

    points = pd.read_csv(height_map)
    lat0 = np.radians(points.lat_deg.median())

    def xy(lat: Any, lon: Any) -> Any:
        return np.c_[np.radians(lon) * EARTH_RADIUS_M * np.cos(lat0), np.radians(lat) * EARTH_RADIUS_M]

    tree = cKDTree(xy(points.lat_deg.values, points.lon_deg.values))
    heights = points.height_m.values
    track = pd.read_csv(solution).drop_duplicates("UnixTimeMillis")
    near = tree.query_ball_point(xy(track.LatitudeDegrees.values, track.LongitudeDegrees.values), 15.0)
    spread = np.array([np.ptp(heights[i]) if len(i) > 1 else np.nan for i in near])
    return float(np.nanmedian(spread))


def audit(recipe: Mapping[str, Any], gsdc_root: Path, variants_dir: Path, maps_dir: Path) -> dict[str, Any]:
    drives = []
    for entry in recipe["drives"]:
        course, phone = entry["id"].split("/")
        available = {name: variants_dir / name / course / phone / "solution.csv"
                     for name in ("base", "motion", "tdcp", "nomap", "nomap+motion")}
        available = {name: path for name, path in available.items() if path.is_file()}
        if "base" not in available:
            raise SystemExit(f"{entry['id']}: no base output under {variants_dir}")
        stage = variants_dir / "base" / course / phone / "summary.json.gnss-first.csv"
        device_gnss = gsdc_root / "test" / course / phone / "device_gnss.csv"
        import pandas as pd  # noqa: PLC0415

        heights = pd.read_csv(stage) if stage.is_file() else wls_heights(device_gnss)
        height_map = maps_dir / f"{course}__{phone}.csv"
        spread = map_spread(height_map, available["base"]) if height_map.is_file() else None
        dropped = spread is not None and spread >= MAP_SPREAD_LIMIT_M
        considered = ["nomap", "nomap+motion"] if dropped else [n for n in ("base", "motion", "tdcp") if n in available]
        metrics = {name: prres(device_gnss, heights, available[name]) for name in considered}
        pick = choose(phone, metrics, spread)
        recorded = entry.get("research_source", {}).get("truth_free", {}).get("variant")
        drives.append({"id": entry["id"], "pick": pick, "map_spread_m": spread,
                       "prres_m": {name: round(value, 4) for name, value in metrics.items()},
                       "recipe_variant": recorded,
                       "matches_recipe": LABELS.get(pick) == recorded if pick != "base" else recorded is None})
    return {"drives": drives, "changed": sum(1 for d in drives if d["pick"] != "base"),
            "matches_recipe": sum(1 for d in drives if d["matches_recipe"])}


def main(argv: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("--recipe", type=Path, default=DEFAULT_RECIPE)
    parser.add_argument("--gsdc-root", type=Path, required=True, help="taroz dataset_2023 root (holds test/<drive>/)")
    parser.add_argument("--variants-dir", type=Path, required=True)
    parser.add_argument("--maps-dir", type=Path, required=True)
    parser.add_argument("--out", type=Path, default=None, help="Write the per-drive report as JSON.")
    args = parser.parse_args(argv)
    report = audit(json.loads(args.recipe.read_text(encoding="utf-8")), args.gsdc_root, args.variants_dir, args.maps_dir)
    for drive in report["drives"]:
        flag = "" if drive["matches_recipe"] else "  <- differs from the recipe"
        print(f"{drive['id']:50s} {drive['pick']:13s} "
              + " ".join(f"{name}={value:.4f}" for name, value in drive["prres_m"].items()) + flag)
    print(f"changed {report['changed']} drives; {report['matches_recipe']}/{len(report['drives'])} match the recipe")
    if args.out:
        args.out.write_text(json.dumps(report, indent=1) + "\n", encoding="utf-8")
    return 0 if report["matches_recipe"] == len(report["drives"]) else 1


if __name__ == "__main__":
    sys.exit(main())
