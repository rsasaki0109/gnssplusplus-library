#!/usr/bin/env python3
"""Run a smartLoc receiver-fix sign-off from public dataset CSV files."""

from __future__ import annotations

import argparse
import json
import os
from pathlib import Path
import sys
import zipfile


ROOT_DIR = Path(__file__).resolve().parent.parent
SCRIPTS_DIR = ROOT_DIR / "scripts"

if str(SCRIPTS_DIR) not in sys.path:
    sys.path.insert(0, str(SCRIPTS_DIR))

import generate_driving_comparison as comparison  # noqa: E402
import gnss_ppc_commercial as ppc_commercial  # noqa: E402
import gnss_ppc_metrics as ppc_metrics  # noqa: E402
import gnss_ppc_demo as ppc_demo  # noqa: E402
import gnss_smartloc_adapter as smartloc_adapter  # noqa: E402

DEFAULT_SMARTLOC_ZIP_URL = smartloc_adapter.DEFAULT_SMARTLOC_ZIP_URL
download_cache_filename = smartloc_adapter.download_cache_filename
RINEX_NAV_EXTENSIONS = (".nav", ".n", ".g", ".p", ".q", ".l")
PRECISE_ORBIT_SUFFIXES = (".sp3", ".sp3.gz", ".sp3.z")
PRECISE_CLOCK_SUFFIXES = (".clk", ".clk.gz", ".clk.z")


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(prog=os.environ.get("GNSS_CLI_NAME"))
    parser.add_argument(
        "--input",
        type=Path,
        default=None,
        help="smartLoc dataset zip or extracted directory.",
    )
    parser.add_argument(
        "--input-url",
        default=DEFAULT_SMARTLOC_ZIP_URL,
        help="Public smartLoc dataset zip URL used when --input/direct CSV paths are omitted.",
    )
    parser.add_argument(
        "--download-cache-dir",
        type=Path,
        default=ROOT_DIR / "output" / "downloads",
        help="Directory for downloaded smartLoc zips.",
    )
    parser.add_argument(
        "--force-download",
        action="store_true",
        help="Re-fetch --input-url even when the cached zip already exists.",
    )
    parser.add_argument("--nav-posllh", type=Path, default=None)
    parser.add_argument("--rawx", type=Path, default=None)
    parser.add_argument("--output-dir", type=Path, default=ROOT_DIR / "output" / "smartloc")
    parser.add_argument("--reference-csv", type=Path, default=None)
    parser.add_argument("--receiver-csv", type=Path, default=None)
    parser.add_argument("--raw-csv", type=Path, default=None)
    parser.add_argument("--obs-rinex", type=Path, default=None)
    parser.add_argument("--matched-csv", type=Path, default=None)
    parser.add_argument("--summary-json", type=Path, default=None)
    parser.add_argument("--receiver-label", default="smartloc_ublox_evk_m8t")
    parser.add_argument("--match-tolerance-s", type=float, default=0.25)
    parser.add_argument("--max-rows", type=int, default=-1)
    parser.add_argument("--raw-max-epochs", type=int, default=-1)
    parser.add_argument("--skip-raw-export", action="store_true")
    parser.add_argument("--require-matched-epochs-min", type=int, default=None)
    parser.add_argument("--require-mean-h-max", type=float, default=None)
    parser.add_argument("--require-median-h-max", type=float, default=None)
    parser.add_argument("--require-p95-h-max", type=float, default=None)
    parser.add_argument("--require-max-h-max", type=float, default=None)
    parser.add_argument("--require-p95-up-max", type=float, default=None)
    parser.add_argument("--require-raw-epochs-min", type=int, default=None)
    parser.add_argument("--require-raw-observations-min", type=int, default=None)
    parser.add_argument("--require-solver-inputs-available", action="store_true")
    return parser.parse_args()


def default_paths(args: argparse.Namespace) -> dict[str, Path]:
    output_dir = args.output_dir
    return {
        "reference_csv": args.reference_csv or output_dir / "smartloc_reference.csv",
        "receiver_csv": args.receiver_csv or output_dir / "smartloc_receiver.csv",
        "raw_csv": args.raw_csv or output_dir / "smartloc_rawx.csv",
        "obs_rinex": args.obs_rinex or output_dir / "smartloc_rover.obs",
        "matched_csv": args.matched_csv or output_dir / "smartloc_receiver_matches.csv",
        "summary_json": args.summary_json or output_dir / "smartloc_signoff_summary.json",
    }


def summarize_receiver_fix(
    *,
    reference_csv: Path,
    receiver_csv: Path,
    matched_csv: Path,
    receiver_label: str,
    match_tolerance_s: float,
) -> dict[str, object]:
    reference = ppc_demo.read_flexible_reference_csv(reference_csv)
    receiver_epochs, receiver_format = ppc_commercial.read_commercial_solution_epochs(
        receiver_csv,
        "csv",
    )
    metrics = ppc_metrics.summarize_solution_epochs(
        reference,
        receiver_epochs,
        1,
        receiver_label,
        match_tolerance_s,
        None,
    )
    matches = comparison.match_to_reference(receiver_epochs, reference, match_tolerance_s)
    ppc_metrics.write_reference_matches_csv(matched_csv, matches)
    metrics.update(
        {
            "label": receiver_label,
            "source": "smartloc_nav_posllh_receiver_fix",
            "solution_pos": str(receiver_csv),
            "format": receiver_format,
            "matched_csv": str(matched_csv),
        }
    )
    return metrics


def list_input_members(input_path: Path | None) -> list[str]:
    if input_path is None:
        return []
    if input_path.is_dir():
        return sorted(
            path.relative_to(input_path).as_posix()
            for path in input_path.rglob("*")
            if path.is_file()
        )
    if input_path.suffix.lower() == ".zip":
        with zipfile.ZipFile(input_path) as archive:
            return sorted(name for name in archive.namelist() if not name.endswith("/"))
    if input_path.exists():
        return [input_path.name]
    return []


def is_rinex_navigation_candidate(name: str) -> bool:
    lower = name.lower()
    path = Path(lower)
    if any(lower.endswith(suffix) for suffix in RINEX_NAV_EXTENSIONS):
        return True
    if path.suffix == ".rnx" and any(token in path.name for token in ("brdc", "nav", "navigation")):
        return True
    return bool(re_match_old_rinex_nav_extension(path.name))


def re_match_old_rinex_nav_extension(name: str) -> bool:
    if len(name) < 4:
        return False
    suffix = name[-4:]
    return (
        suffix[0] == "."
        and suffix[1:3].isdigit()
        and suffix[3] in {"n", "g", "p", "q", "l"}
    )


def is_base_observation_candidate(name: str) -> bool:
    lower = name.lower()
    path = Path(lower)
    if not any(token in path.name for token in ("base", "station")):
        return False
    return lower.endswith((".obs", ".rnx", ".o", ".ubx", ".rtcm", ".rtcm3")) or re_match_old_rinex_obs_extension(path.name)


def re_match_old_rinex_obs_extension(name: str) -> bool:
    if len(name) < 4:
        return False
    suffix = name[-4:]
    return suffix[0] == "." and suffix[1:3].isdigit() and suffix[3] == "o"


def endswith_any(name: str, suffixes: tuple[str, ...]) -> bool:
    lower = name.lower()
    return any(lower.endswith(suffix) for suffix in suffixes)


def build_solver_preflight(
    *,
    input_path: Path | None,
    paths: dict[str, Path],
    raw_summary: dict[str, object] | None,
) -> dict[str, object]:
    members = list_input_members(input_path)
    broadcast_nav = [name for name in members if is_rinex_navigation_candidate(name)]
    base_observations = [name for name in members if is_base_observation_candidate(name)]
    precise_orbits = [name for name in members if endswith_any(name, PRECISE_ORBIT_SUFFIXES)]
    precise_clocks = [name for name in members if endswith_any(name, PRECISE_CLOCK_SUFFIXES)]
    has_rover_obs = raw_summary is not None and paths["obs_rinex"].exists()

    rtk_blockers: list[str] = []
    if not has_rover_obs:
        rtk_blockers.append("missing generated rover RINEX observations from RXM-RAWX")
    if not broadcast_nav:
        rtk_blockers.append("missing broadcast navigation RINEX in smartLoc input")
    if not base_observations:
        rtk_blockers.append("missing base-station observation stream in smartLoc input")

    ppp_blockers: list[str] = []
    if not has_rover_obs:
        ppp_blockers.append("missing generated rover RINEX observations from RXM-RAWX")
    if not precise_orbits:
        ppp_blockers.append("missing precise SP3 orbit product")
    if not precise_clocks:
        ppp_blockers.append("missing precise CLK clock product")

    return {
        "status": "ready" if not rtk_blockers else "blocked",
        "input_path": str(input_path) if input_path is not None else None,
        "archive_member_count": len(members),
        "archive_members": members,
        "rover_obs_rinex": str(paths["obs_rinex"]) if has_rover_obs else None,
        "broadcast_nav_candidates": broadcast_nav,
        "base_observation_candidates": base_observations,
        "precise_orbit_candidates": precise_orbits,
        "precise_clock_candidates": precise_clocks,
        "rtk_signoff_available": not rtk_blockers,
        "rtk_blockers": rtk_blockers,
        "spp_smoke_available": has_rover_obs and bool(broadcast_nav),
        "ppp_smoke_available": has_rover_obs and bool(precise_orbits) and bool(precise_clocks),
        "ppp_blockers": ppp_blockers,
    }


def enforce_requirements(payload: dict[str, object], args: argparse.Namespace) -> None:
    failures: list[str] = []
    receiver = payload["receiver_fix"]
    assert isinstance(receiver, dict)
    if (
        args.require_matched_epochs_min is not None
        and int(receiver["matched_epochs"]) < args.require_matched_epochs_min
    ):
        failures.append(
            f"matched epochs {int(receiver['matched_epochs'])} < {args.require_matched_epochs_min}"
        )
    if args.require_mean_h_max is not None and float(receiver["mean_h_m"]) > args.require_mean_h_max:
        failures.append(
            f"mean horizontal error {float(receiver['mean_h_m']):.6f} m > {args.require_mean_h_max:.6f} m"
        )
    if args.require_median_h_max is not None and float(receiver["median_h_m"]) > args.require_median_h_max:
        failures.append(
            f"median horizontal error {float(receiver['median_h_m']):.6f} m > {args.require_median_h_max:.6f} m"
        )
    if args.require_p95_h_max is not None and float(receiver["p95_h_m"]) > args.require_p95_h_max:
        failures.append(
            f"p95 horizontal error {float(receiver['p95_h_m']):.6f} m > {args.require_p95_h_max:.6f} m"
        )
    if args.require_max_h_max is not None and float(receiver["max_h_m"]) > args.require_max_h_max:
        failures.append(
            f"max horizontal error {float(receiver['max_h_m']):.6f} m > {args.require_max_h_max:.6f} m"
        )
    if args.require_p95_up_max is not None and float(receiver["p95_abs_up_m"]) > args.require_p95_up_max:
        failures.append(
            f"p95 absolute up error {float(receiver['p95_abs_up_m']):.6f} m > {args.require_p95_up_max:.6f} m"
        )

    raw = payload.get("raw_adapter")
    if args.require_raw_epochs_min is not None:
        if raw is None:
            failures.append("raw adapter summary is unavailable")
        elif int(raw["raw_epochs"]) < args.require_raw_epochs_min:
            failures.append(
                f"raw epochs {int(raw['raw_epochs'])} < {args.require_raw_epochs_min}"
            )
    if args.require_raw_observations_min is not None:
        if raw is None:
            failures.append("raw adapter summary is unavailable")
        elif int(raw["raw_observations"]) < args.require_raw_observations_min:
            failures.append(
                "raw observations "
                f"{int(raw['raw_observations'])} < {args.require_raw_observations_min}"
            )
    if args.require_solver_inputs_available:
        solver_preflight = payload.get("solver_preflight")
        if not isinstance(solver_preflight, dict) or not bool(solver_preflight.get("rtk_signoff_available", False)):
            blockers: list[str] = []
            if isinstance(solver_preflight, dict):
                raw_blockers = solver_preflight.get("rtk_blockers", [])
                if isinstance(raw_blockers, list):
                    blockers = [str(item) for item in raw_blockers]
            blocker_text = "; ".join(blockers) if blockers else "unknown blocker"
            failures.append(f"solver inputs unavailable: {blocker_text}")

    if failures:
        raise SystemExit("smartLoc sign-off checks failed:\n" + "\n".join(f"  - {item}" for item in failures))


def build_payload(
    *,
    args: argparse.Namespace,
    paths: dict[str, Path],
    nav_summary: dict[str, object],
    raw_summary: dict[str, object] | None,
    receiver_fix: dict[str, object],
    solver_preflight: dict[str, object],
) -> dict[str, object]:
    rtk_available = bool(solver_preflight["rtk_signoff_available"])
    rtk_blockers = solver_preflight.get("rtk_blockers", [])
    return {
        "dataset": "smartLoc",
        "signoff_profile": "smartloc-receiver-fix",
        "input": str(args.input) if args.input is not None else None,
        "input_url": getattr(args, "resolved_input_url", None),
        "downloaded_input": str(args.resolved_input) if getattr(args, "resolved_input_url", None) else None,
        "nav_posllh": str(args.nav_posllh) if args.nav_posllh is not None else None,
        "rawx": str(args.rawx) if args.rawx is not None else None,
        "reference_csv": str(paths["reference_csv"]),
        "receiver_csv": str(paths["receiver_csv"]),
        "raw_csv": str(paths["raw_csv"]) if raw_summary is not None else None,
        "obs_rinex": str(paths["obs_rinex"]) if raw_summary is not None else None,
        "matched_csv": str(paths["matched_csv"]),
        "summary_json": str(paths["summary_json"]),
        "match_tolerance_s": ppc_metrics.rounded(args.match_tolerance_s),
        "receiver_label": args.receiver_label,
        "adapter": nav_summary,
        "raw_adapter": raw_summary,
        "receiver_fix": receiver_fix,
        "solver_preflight": solver_preflight,
        "solver_signoff_available": rtk_available,
        "solver_signoff_blocker": None if rtk_available else "; ".join(str(item) for item in rtk_blockers),
    }


def main() -> int:
    args = parse_args()
    args.resolved_input, args.resolved_input_url = smartloc_adapter.ensure_local_input(
        input_path=args.input,
        nav_posllh=args.nav_posllh,
        rawx=args.rawx,
        need_nav=True,
        need_raw=not args.skip_raw_export,
        input_url=args.input_url,
        download_cache_dir=args.download_cache_dir,
        force_download=args.force_download,
    )
    paths = default_paths(args)
    for path in paths.values():
        path.parent.mkdir(parents=True, exist_ok=True)

    nav_source, nav_member = smartloc_adapter.resolve_nav_posllh_path(args.resolved_input, args.nav_posllh)
    nav_summary = smartloc_adapter.convert_nav_posllh(
        source_path=nav_source,
        archive_member=nav_member,
        reference_csv=paths["reference_csv"],
        receiver_csv=paths["receiver_csv"],
        receiver_label=args.receiver_label,
        max_rows=args.max_rows,
    )

    raw_summary: dict[str, object] | None = None
    if not args.skip_raw_export:
        raw_source, raw_member = smartloc_adapter.resolve_rawx_path(args.resolved_input, args.rawx)
        raw_summary = smartloc_adapter.convert_rawx(
            source_path=raw_source,
            archive_member=raw_member,
            raw_csv=paths["raw_csv"],
            obs_rinex=paths["obs_rinex"],
            max_epochs=args.raw_max_epochs,
        )

    receiver_fix = summarize_receiver_fix(
        reference_csv=paths["reference_csv"],
        receiver_csv=paths["receiver_csv"],
        matched_csv=paths["matched_csv"],
        receiver_label=args.receiver_label,
        match_tolerance_s=args.match_tolerance_s,
    )
    solver_preflight = build_solver_preflight(
        input_path=args.resolved_input,
        paths=paths,
        raw_summary=raw_summary,
    )
    payload = build_payload(
        args=args,
        paths=paths,
        nav_summary=nav_summary,
        raw_summary=raw_summary,
        receiver_fix=receiver_fix,
        solver_preflight=solver_preflight,
    )
    enforce_requirements(payload, args)
    paths["summary_json"].write_text(json.dumps(payload, indent=2, sort_keys=True) + "\n", encoding="utf-8")

    print("Finished smartLoc sign-off.")
    print(f"  receiver matched epochs: {receiver_fix['matched_epochs']}")
    print(f"  receiver p95_h_m: {receiver_fix['p95_h_m']}")
    if raw_summary is not None:
        print(f"  raw epochs: {raw_summary['raw_epochs']}")
        print(f"  raw observations: {raw_summary['raw_observations']}")
    print(f"  solver preflight: {solver_preflight['status']}")
    if solver_preflight["rtk_blockers"]:
        print(f"  solver blockers: {payload['solver_signoff_blocker']}")
    print(f"  summary: {paths['summary_json']}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
