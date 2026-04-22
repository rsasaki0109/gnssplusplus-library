#!/usr/bin/env python3
"""Run libgnss++ against an external PPC-Dataset run and summarize the result."""

from __future__ import annotations

import argparse
import csv
from datetime import datetime, timedelta
import json
import os
from pathlib import Path
import subprocess
import sys
import tempfile
import time

from gnss_runtime import ensure_input_exists, resolve_gnss_command


ROOT_DIR = Path(__file__).resolve().parent.parent
SCRIPTS_DIR = ROOT_DIR / "scripts"
GPS_EPOCH = datetime(1980, 1, 6)
NONFIX_DRIFT_GUARD_DEFAULTS = {
    "max_anchor_gap_s": 120.0,
    "max_anchor_speed_mps": 1.0,
    "max_residual_m": 30.0,
    "min_horizontal_residual_m": 0.0,
    "min_segment_epochs": 20,
    "max_segment_epochs": 0,
}
SPP_HEIGHT_STEP_GUARD_DEFAULTS = {
    "min_step_m": 30.0,
    "max_rate_mps": 4.0,
}
FLOAT_BRIDGE_TAIL_GUARD_DEFAULTS = {
    "max_anchor_gap_s": 120.0,
    "min_anchor_speed_mps": 0.4,
    "max_anchor_speed_mps": 1.0,
    "max_residual_m": 12.0,
    "min_segment_epochs": 20,
}
FIXED_BRIDGE_BURST_GUARD_DEFAULTS = {
    "max_anchor_gap_s": 30.0,
    "min_boundary_gap_s": 1.0,
    "max_residual_m": 20.0,
    "max_segment_epochs": 12,
}

sys.path.insert(0, str(SCRIPTS_DIR))

import generate_driving_comparison as comparison  # noqa: E402
import gnss_ppc_commercial as ppc_commercial  # noqa: E402
import gnss_ppc_metrics as ppc_metrics  # noqa: E402

PPC_RECEIVER_HARDWARE: dict[str, dict[str, object]] = {
    "tokyo": {
        "vehicle_receiver": "Septentrio mosaic-X5",
        "vehicle_antenna": "Trimble AT1675",
        "reference_station_receiver": "Trimble Alloy",
        "reference_station_antenna": "Trimble Zephyr Geodetic 2",
        "reference_station_phase_center_llh_deg_m": [35.66633426, 139.79220181, 59.82],
    },
    "nagoya": {
        "vehicle_receiver": "Septentrio mosaic-X5",
        "vehicle_antenna": "Trimble Zephyr 3 Rover",
        "reference_station_receiver": "Trimble NetR9",
        "reference_station_antenna": "Trimble Zephyr 3 Base",
        "reference_station_phase_center_llh_deg_m": [35.13470947, 136.97757427, 104.718],
    },
}


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(prog=os.environ.get("GNSS_CLI_NAME"))
    parser.add_argument(
        "--dataset-root",
        type=Path,
        default=None,
        help="Root of an extracted PPC-Dataset tree.",
    )
    parser.add_argument(
        "--city",
        choices=("tokyo", "nagoya"),
        default=None,
        help="City directory under --dataset-root.",
    )
    parser.add_argument(
        "--run",
        default="run1",
        help="Run directory under --dataset-root/<city> (default: run1).",
    )
    parser.add_argument(
        "--run-dir",
        type=Path,
        default=None,
        help="Explicit PPC run directory containing rover.obs, base.obs, base.nav, and reference.csv.",
    )
    parser.add_argument(
        "--solver",
        choices=("rtk", "ppp"),
        default="rtk",
        help="libgnss++ solver path to run against the PPC data.",
    )
    parser.add_argument("--rover", type=Path, default=None, help="Override rover.obs path.")
    parser.add_argument("--base", type=Path, default=None, help="Override base.obs path for RTK.")
    parser.add_argument("--nav", type=Path, default=None, help="Override base.nav path.")
    parser.add_argument(
        "--reference-csv",
        type=Path,
        default=None,
        help="Override reference.csv path.",
    )
    parser.add_argument(
        "--out",
        type=Path,
        default=None,
        help="Output .pos path (default: output/ppc_<city>_<run>_<solver>.pos).",
    )
    parser.add_argument(
        "--summary-json",
        type=Path,
        default=None,
        help="Output summary JSON path (default: output/ppc_<city>_<run>_<solver>_summary.json).",
    )
    parser.add_argument(
        "--max-epochs",
        type=int,
        default=120,
        help="Stop after N epochs (default: 120). Use -1 for the full run.",
    )
    parser.add_argument(
        "--match-tolerance-s",
        type=float,
        default=0.25,
        help="Reference matching tolerance in seconds (default: 0.25).",
    )
    parser.add_argument(
        "--use-existing-solution",
        action="store_true",
        help="Do not rerun the solver; only summarize an existing --out file.",
    )
    parser.add_argument(
        "--solver-wall-time-s",
        type=float,
        default=None,
        help="Optional solver wall time in seconds. If omitted, actual runtime is recorded when the solver is executed.",
    )
    parser.add_argument("--sp3", type=Path, default=None, help="Optional SP3 precise orbit file for PPP.")
    parser.add_argument("--clk", type=Path, default=None, help="Optional CLK precise clock file for PPP.")
    parser.add_argument("--ionex", type=Path, default=None, help="Optional IONEX TEC map file for PPP.")
    parser.add_argument("--dcb", type=Path, default=None, help="Optional DCB / Bias-SINEX file for PPP.")
    parser.add_argument("--antex", type=Path, default=None, help="Optional ANTEX file for PPP.")
    parser.add_argument("--blq", type=Path, default=None, help="Optional BLQ loading coefficients for PPP.")
    parser.add_argument(
        "--ppp-summary-json",
        type=Path,
        default=None,
        help="Optional gnss ppp summary JSON output path when --solver ppp is executed.",
    )
    parser.add_argument(
        "--preset",
        choices=("survey", "low-cost", "moving-base"),
        default=None,
        help="Optional RTK tuning preset passed through to gnss solve when --solver rtk.",
    )
    parser.add_argument(
        "--iono",
        choices=("auto", "off", "iflc", "est"),
        default=None,
        help="Optional RTK ionosphere option passed through to gnss solve when --solver rtk.",
    )
    parser.add_argument(
        "--ratio",
        type=float,
        default=None,
        help="Optional RTK ambiguity ratio threshold passed through to gnss solve when --solver rtk.",
    )
    parser.add_argument(
        "--max-hold-div",
        type=float,
        default=None,
        help="Optional RTK hold-fix divergence guard passed through to gnss solve when --solver rtk.",
    )
    parser.add_argument(
        "--max-pos-jump",
        type=float,
        default=None,
        help="Optional RTK fixed-position jump guard passed through to gnss solve when --solver rtk.",
    )
    parser.add_argument(
        "--max-pos-jump-min",
        type=float,
        default=None,
        help="Optional RTK adaptive fixed-position jump floor passed through to gnss solve.",
    )
    parser.add_argument(
        "--max-pos-jump-rate",
        type=float,
        default=None,
        help="Optional RTK adaptive fixed-position jump rate passed through to gnss solve.",
    )
    parser.add_argument(
        "--max-float-spp-div",
        type=float,
        default=None,
        help="Optional RTK FLOAT-vs-SPP divergence guard passed through to gnss solve.",
    )
    parser.add_argument(
        "--max-float-prefit-rms",
        type=float,
        default=None,
        help="Optional RTK FLOAT prefit DD residual RMS reset/fallback threshold in meters.",
    )
    parser.add_argument(
        "--max-float-prefit-max",
        type=float,
        default=None,
        help="Optional RTK FLOAT prefit DD residual magnitude reset/fallback threshold in meters.",
    )
    parser.add_argument(
        "--max-consec-float-reset",
        type=int,
        default=None,
        help="Optional RTK ambiguity reset after N consecutive FLOAT epochs.",
    )
    parser.add_argument(
        "--max-consec-nonfix-reset",
        type=int,
        default=None,
        help="Optional RTK ambiguity reset after N consecutive non-FIX epochs.",
    )
    parser.add_argument(
        "--max-postfix-rms",
        type=float,
        default=None,
        help="Optional RTK post-fix residual RMS rejection threshold in meters.",
    )
    parser.add_argument(
        "--enable-wide-lane-ar",
        action="store_true",
        help="Enable the RTK Melbourne-Wubbena wide-lane AR pre-step.",
    )
    parser.add_argument(
        "--wide-lane-threshold",
        type=float,
        default=None,
        help="Optional wide-lane integer acceptance threshold in cycles.",
    )
    parser.add_argument("--arfilter", dest="arfilter", action="store_true", help="Enable AR filter for RTK.")
    parser.add_argument("--no-arfilter", dest="arfilter", action="store_false", help="Disable AR filter for RTK.")
    parser.set_defaults(arfilter=None)
    parser.add_argument("--arfilter-margin", type=float, default=None, help="Optional AR filter margin for RTK.")
    parser.add_argument("--min-hold-count", type=int, default=None, help="Optional min hold count for RTK.")
    parser.add_argument(
        "--hold-ratio-threshold",
        type=float,
        default=None,
        help="Optional ratio threshold used while hold is active for RTK.",
    )
    parser.add_argument(
        "--no-kinematic-post-filter",
        action="store_true",
        help=(
            "For RTK runs, keep valid SPP/float fallback epochs instead of applying "
            "the precision-oriented kinematic post-filter."
        ),
    )
    parser.add_argument(
        "--no-nonfix-drift-guard",
        action="store_true",
        help="Disable the low-speed non-FIX drift guard in gnss solve.",
    )
    parser.add_argument("--nonfix-drift-max-anchor-gap", type=float, default=None)
    parser.add_argument("--nonfix-drift-max-anchor-speed", type=float, default=None)
    parser.add_argument("--nonfix-drift-max-residual", type=float, default=None)
    parser.add_argument("--nonfix-drift-min-horizontal-residual", type=float, default=None)
    parser.add_argument("--nonfix-drift-min-segment-epochs", type=int, default=None)
    parser.add_argument("--nonfix-drift-max-segment-epochs", type=int, default=None)
    parser.add_argument(
        "--no-spp-height-step-guard",
        action="store_true",
        help="Disable the RTK SPP vertical spike guard in gnss solve.",
    )
    parser.add_argument("--spp-height-step-min", type=float, default=None)
    parser.add_argument("--spp-height-step-rate", type=float, default=None)
    float_bridge_tail_group = parser.add_mutually_exclusive_group()
    float_bridge_tail_group.add_argument(
        "--float-bridge-tail-guard",
        dest="float_bridge_tail_guard",
        action="store_true",
        help="Enable the slow FLOAT bridge-tail guard in gnss solve (default).",
    )
    float_bridge_tail_group.add_argument(
        "--no-float-bridge-tail-guard",
        dest="float_bridge_tail_guard",
        action="store_false",
        help="Disable the slow FLOAT bridge-tail guard in gnss solve.",
    )
    parser.set_defaults(float_bridge_tail_guard=True)
    parser.add_argument("--float-bridge-tail-max-anchor-gap", type=float, default=None)
    parser.add_argument("--float-bridge-tail-min-anchor-speed", type=float, default=None)
    parser.add_argument("--float-bridge-tail-max-anchor-speed", type=float, default=None)
    parser.add_argument("--float-bridge-tail-max-residual", type=float, default=None)
    parser.add_argument("--float-bridge-tail-min-segment-epochs", type=int, default=None)
    parser.add_argument(
        "--fixed-bridge-burst-guard",
        action="store_true",
        help="Enable opt-in short FIX burst rejection against surrounding FIX-anchor bridge.",
    )
    parser.add_argument(
        "--fixed-bridge-burst-max-anchor-gap",
        type=float,
        default=None,
        help="Maximum surrounding FIX-anchor gap in seconds for the short FIX burst guard.",
    )
    parser.add_argument(
        "--fixed-bridge-burst-min-boundary-gap",
        type=float,
        default=None,
        help="Minimum non-contiguous gap in seconds on each side of a candidate FIX burst.",
    )
    parser.add_argument(
        "--fixed-bridge-burst-max-residual",
        type=float,
        default=None,
        help="Reject burst FIX epochs farther than this from the surrounding FIX-anchor bridge.",
    )
    parser.add_argument(
        "--fixed-bridge-burst-max-segment-epochs",
        type=int,
        default=None,
        help="Maximum contiguous FIX epochs considered a short burst.",
    )
    parser.add_argument(
        "--rtklib-bin",
        type=Path,
        default=None,
        help="Optional RTKLIB rnx2rtkp binary for side-by-side comparison.",
    )
    parser.add_argument(
        "--rtklib-config",
        type=Path,
        default=ROOT_DIR / "scripts/rtklib_odaiba.conf",
        help="RTKLIB configuration file used when --rtklib-bin is set.",
    )
    parser.add_argument(
        "--rtklib-pos",
        type=Path,
        default=None,
        help="Optional RTKLIB .pos path. If omitted, one is generated next to the summary when --rtklib-bin is used.",
    )
    parser.add_argument(
        "--use-existing-rtklib-solution",
        action="store_true",
        help="Do not rerun RTKLIB; only summarize an existing --rtklib-pos file.",
    )
    parser.add_argument(
        "--rtklib-solver-wall-time-s",
        type=float,
        default=None,
        help="Optional RTKLIB wall time in seconds. If omitted, actual runtime is recorded when RTKLIB is executed.",
    )
    parser.add_argument(
        "--commercial-pos",
        type=Path,
        default=None,
        help=(
            "Optional commercial receiver solution, as libgnss .pos or normalized CSV, "
            "to summarize against the PPC reference."
        ),
    )
    parser.add_argument(
        "--commercial-rover",
        type=Path,
        default=None,
        help=(
            "Optional commercial receiver rover observation RINEX. It is solved with libgnss++ "
            "against the same reference for public datasets such as UrbanNav Tokyo."
        ),
    )
    parser.add_argument(
        "--commercial-base",
        type=Path,
        default=None,
        help="Base observation file for --commercial-rover (default: the primary RTK base).",
    )
    parser.add_argument(
        "--commercial-nav",
        type=Path,
        default=None,
        help="Navigation file for --commercial-rover (default: the primary navigation file).",
    )
    parser.add_argument(
        "--commercial-out",
        type=Path,
        default=None,
        help="Output .pos path for --commercial-rover (default: next to --summary-json).",
    )
    parser.add_argument(
        "--use-existing-commercial-solution",
        action="store_true",
        help="Do not solve --commercial-rover; summarize an existing --commercial-out file.",
    )
    parser.add_argument(
        "--commercial-format",
        choices=("auto", "pos", "csv"),
        default="auto",
        help="Commercial receiver solution format (default: auto from suffix).",
    )
    parser.add_argument(
        "--commercial-label",
        default="commercial_receiver",
        help="Label stored in the commercial receiver summary.",
    )
    parser.add_argument(
        "--commercial-matched-csv",
        type=Path,
        default=None,
        help="Optional CSV of commercial receiver epochs matched to the PPC reference.",
    )
    parser.add_argument(
        "--commercial-solver-wall-time-s",
        type=float,
        default=None,
        help="Optional commercial receiver wall time in seconds for realtime metrics.",
    )
    parser.add_argument(
        "--commercial-preset",
        choices=("survey", "low-cost", "moving-base"),
        default=None,
        help="Optional RTK preset for solving --commercial-rover.",
    )
    parser.add_argument("--commercial-arfilter", dest="commercial_arfilter", action="store_true")
    parser.add_argument("--no-commercial-arfilter", dest="commercial_arfilter", action="store_false")
    parser.set_defaults(commercial_arfilter=None)
    parser.add_argument("--commercial-arfilter-margin", type=float, default=None)
    parser.add_argument("--commercial-min-hold-count", type=int, default=None)
    parser.add_argument("--commercial-hold-ratio-threshold", type=float, default=None)
    parser.add_argument(
        "--enable-ar",
        action="store_true",
        help="Enable PPP ambiguity resolution.",
    )
    parser.add_argument(
        "--low-dynamics",
        action="store_true",
        help="Use the quasi-static low-dynamics PPP profile.",
    )
    parser.add_argument("--require-valid-epochs-min", type=int, default=None)
    parser.add_argument("--require-matched-epochs-min", type=int, default=None)
    parser.add_argument("--require-positioning-rate-min", type=float, default=None)
    parser.add_argument("--require-fix-rate-min", type=float, default=None)
    parser.add_argument("--require-ppc-official-score-min", type=float, default=None)
    parser.add_argument("--require-ppc-score-3d-50cm-ref-min", type=float, default=None)
    parser.add_argument("--require-median-h-max", type=float, default=None)
    parser.add_argument("--require-p95-h-max", type=float, default=None)
    parser.add_argument("--require-max-h-max", type=float, default=None)
    parser.add_argument("--require-p95-up-max", type=float, default=None)
    parser.add_argument("--require-mean-sats-min", type=float, default=None)
    parser.add_argument("--require-solver-wall-time-max", type=float, default=None)
    parser.add_argument("--require-realtime-factor-min", type=float, default=None)
    parser.add_argument("--require-effective-epoch-rate-min", type=float, default=None)
    parser.add_argument("--require-lib-fix-rate-vs-rtklib-min-delta", type=float, default=None)
    parser.add_argument("--require-lib-median-h-vs-rtklib-max-delta", type=float, default=None)
    parser.add_argument("--require-lib-p95-h-vs-rtklib-max-delta", type=float, default=None)
    return parser.parse_args()


def run_command(command: list[str]) -> None:
    print("+", " ".join(command))
    subprocess.run(command, check=True)


def rounded(value: float) -> float:
    return round(value, 6)


def nonfix_drift_guard_value(
    args: argparse.Namespace,
    attr_name: str,
    default_name: str,
) -> float | int:
    value = getattr(args, attr_name, None)
    if value is None:
        return NONFIX_DRIFT_GUARD_DEFAULTS[default_name]
    return value


def nonfix_drift_guard_config(args: argparse.Namespace) -> dict[str, float | int]:
    return {
        "max_anchor_gap_s": rounded(
            float(
                nonfix_drift_guard_value(
                    args,
                    "nonfix_drift_max_anchor_gap",
                    "max_anchor_gap_s",
                )
            )
        ),
        "max_anchor_speed_mps": rounded(
            float(
                nonfix_drift_guard_value(
                    args,
                    "nonfix_drift_max_anchor_speed",
                    "max_anchor_speed_mps",
                )
            )
        ),
        "max_residual_m": rounded(
            float(
                nonfix_drift_guard_value(
                    args,
                    "nonfix_drift_max_residual",
                    "max_residual_m",
                )
            )
        ),
        "min_horizontal_residual_m": rounded(
            float(
                nonfix_drift_guard_value(
                    args,
                    "nonfix_drift_min_horizontal_residual",
                    "min_horizontal_residual_m",
                )
            )
        ),
        "min_segment_epochs": int(
            nonfix_drift_guard_value(
                args,
                "nonfix_drift_min_segment_epochs",
                "min_segment_epochs",
            )
        ),
        "max_segment_epochs": int(
            nonfix_drift_guard_value(
                args,
                "nonfix_drift_max_segment_epochs",
                "max_segment_epochs",
            )
        ),
    }


def spp_height_step_guard_value(
    args: argparse.Namespace,
    attr_name: str,
    default_name: str,
) -> float:
    value = getattr(args, attr_name, None)
    if value is None:
        return SPP_HEIGHT_STEP_GUARD_DEFAULTS[default_name]
    return value


def spp_height_step_guard_config(args: argparse.Namespace) -> dict[str, float]:
    return {
        "min_step_m": rounded(
            float(
                spp_height_step_guard_value(
                    args,
                    "spp_height_step_min",
                    "min_step_m",
                )
            )
        ),
        "max_rate_mps": rounded(
            float(
                spp_height_step_guard_value(
                    args,
                    "spp_height_step_rate",
                    "max_rate_mps",
                )
            )
        ),
    }


def float_bridge_tail_guard_value(
    args: argparse.Namespace,
    attr_name: str,
    default_name: str,
) -> float | int:
    value = getattr(args, attr_name, None)
    if value is None:
        return FLOAT_BRIDGE_TAIL_GUARD_DEFAULTS[default_name]
    return value


def float_bridge_tail_guard_config(args: argparse.Namespace) -> dict[str, float | int]:
    return {
        "max_anchor_gap_s": rounded(
            float(
                float_bridge_tail_guard_value(
                    args,
                    "float_bridge_tail_max_anchor_gap",
                    "max_anchor_gap_s",
                )
            )
        ),
        "min_anchor_speed_mps": rounded(
            float(
                float_bridge_tail_guard_value(
                    args,
                    "float_bridge_tail_min_anchor_speed",
                    "min_anchor_speed_mps",
                )
            )
        ),
        "max_anchor_speed_mps": rounded(
            float(
                float_bridge_tail_guard_value(
                    args,
                    "float_bridge_tail_max_anchor_speed",
                    "max_anchor_speed_mps",
                )
            )
        ),
        "max_residual_m": rounded(
            float(
                float_bridge_tail_guard_value(
                    args,
                    "float_bridge_tail_max_residual",
                    "max_residual_m",
                )
            )
        ),
        "min_segment_epochs": int(
            float_bridge_tail_guard_value(
                args,
                "float_bridge_tail_min_segment_epochs",
                "min_segment_epochs",
            )
        ),
    }


def fixed_bridge_burst_guard_value(
    args: argparse.Namespace,
    attr_name: str,
    default_name: str,
) -> float | int:
    value = getattr(args, attr_name, None)
    if value is None:
        return FIXED_BRIDGE_BURST_GUARD_DEFAULTS[default_name]
    return value


def fixed_bridge_burst_guard_config(args: argparse.Namespace) -> dict[str, float | int]:
    return {
        "max_anchor_gap_s": rounded(
            float(
                fixed_bridge_burst_guard_value(
                    args,
                    "fixed_bridge_burst_max_anchor_gap",
                    "max_anchor_gap_s",
                )
            )
        ),
        "min_boundary_gap_s": rounded(
            float(
                fixed_bridge_burst_guard_value(
                    args,
                    "fixed_bridge_burst_min_boundary_gap",
                    "min_boundary_gap_s",
                )
            )
        ),
        "max_residual_m": rounded(
            float(
                fixed_bridge_burst_guard_value(
                    args,
                    "fixed_bridge_burst_max_residual",
                    "max_residual_m",
                )
            )
        ),
        "max_segment_epochs": int(
            fixed_bridge_burst_guard_value(
                args,
                "fixed_bridge_burst_max_segment_epochs",
                "max_segment_epochs",
            )
        ),
    }


def normalize_header(name: str) -> str:
    normalized = name.strip().lower()
    for old, new in (
        (" ", "_"),
        ("-", "_"),
        ("/", "_"),
        ("(", ""),
        (")", ""),
        ("[", ""),
        ("]", ""),
    ):
        normalized = normalized.replace(old, new)
    while "__" in normalized:
        normalized = normalized.replace("__", "_")
    return normalized


def gps_datetime_to_week_tow(stamp: datetime) -> tuple[int, float]:
    delta = stamp - GPS_EPOCH
    total_seconds = delta.total_seconds()
    week = int(total_seconds // 604800)
    tow = total_seconds - week * 604800
    return week, tow


def parse_reference_timestamp(text: str) -> tuple[int, float]:
    candidates = (
        "%Y/%m/%d %H:%M:%S.%f",
        "%Y/%m/%d %H:%M:%S",
        "%Y-%m-%d %H:%M:%S.%f",
        "%Y-%m-%d %H:%M:%S",
        "%Y-%m-%dT%H:%M:%S.%f",
        "%Y-%m-%dT%H:%M:%S",
    )
    stripped = text.strip()
    for pattern in candidates:
        try:
            return gps_datetime_to_week_tow(datetime.strptime(stripped, pattern))
        except ValueError:
            continue
    raise SystemExit(f"Unsupported PPC reference timestamp format: {text}")


def read_flexible_reference_csv(path: Path) -> list[comparison.ReferenceEpoch]:
    with path.open(newline="", encoding="utf-8") as handle:
        reader = csv.DictReader(handle)
        if not reader.fieldnames:
            return comparison.read_reference_csv(path)

        normalized_fields = {normalize_header(name): name for name in reader.fieldnames if name}
        if "gps_week" not in normalized_fields and "week" not in normalized_fields:
            if "gps_tow_s" not in normalized_fields and "tow" not in normalized_fields:
                return comparison.read_reference_csv(path)

        def get(row: dict[str, str], *names: str) -> str | None:
            for name in names:
                original = normalized_fields.get(name)
                if original is None:
                    continue
                value = row.get(original)
                if value is not None and value.strip():
                    return value.strip()
            return None

        rows: list[comparison.ReferenceEpoch] = []
        for row in reader:
            week_token = get(row, "gps_week", "week")
            tow_token = get(row, "gps_tow_s", "gps_tow", "tow_s", "tow", "gpstow")
            if week_token is not None and tow_token is not None:
                week = int(float(week_token))
                tow = float(tow_token)
            else:
                timestamp_token = get(row, "gpst", "gps_time", "timestamp")
                if timestamp_token is None:
                    date_token = get(row, "date")
                    time_token = get(row, "time")
                    if date_token is None or time_token is None:
                        raise SystemExit(
                            f"Could not infer GPS week/tow from {path}; supported columns are gps_week/gps_tow_s or timestamp/date+time"
                        )
                    timestamp_token = f"{date_token} {time_token}"
                week, tow = parse_reference_timestamp(timestamp_token)

            lat_token = get(row, "lat_deg", "latitude_deg", "lat", "latitude")
            lon_token = get(row, "lon_deg", "longitude_deg", "lon", "longitude")
            height_token = get(
                row,
                "height_m",
                "altitude_m",
                "alt_m",
                "height",
                "altitude",
            )
            ecef_x_token = get(row, "ecef_x_m", "x_m", "ecef_x", "x")
            ecef_y_token = get(row, "ecef_y_m", "y_m", "ecef_y", "y")
            ecef_z_token = get(row, "ecef_z_m", "z_m", "ecef_z", "z")

            if lat_token is not None and lon_token is not None and height_token is not None:
                lat_deg = float(lat_token)
                lon_deg = float(lon_token)
                height_m = float(height_token)
                ecef = comparison.llh_to_ecef(lat_deg, lon_deg, height_m)
            elif ecef_x_token is not None and ecef_y_token is not None and ecef_z_token is not None:
                ecef_x_m = float(ecef_x_token)
                ecef_y_m = float(ecef_y_token)
                ecef_z_m = float(ecef_z_token)
                lat_deg, lon_deg, height_m = ppc_metrics.llh_from_ecef(ecef_x_m, ecef_y_m, ecef_z_m)
                ecef = comparison.llh_to_ecef(lat_deg, lon_deg, height_m)
            else:
                raise SystemExit(
                    f"Could not infer reference coordinates from {path}; expected lat/lon/height or ECEF XYZ columns"
                )

            rows.append(
                comparison.ReferenceEpoch(
                    week=week,
                    tow=tow,
                    lat_deg=lat_deg,
                    lon_deg=lon_deg,
                    height_m=height_m,
                    ecef=ecef,
                )
            )
        return rows


def gps_week_tow_to_datetime_strings(week: int, tow: float) -> tuple[str, str]:
    stamp = GPS_EPOCH + timedelta(weeks=week, seconds=tow)
    return stamp.strftime("%Y/%m/%d"), stamp.strftime("%H:%M:%S.%f")[:-3]


def rtklib_config_text(config_path: Path, solver: str) -> str:
    text = config_path.read_text(encoding="utf-8")
    replacements = {
        "pos1-navsys        =1": "pos1-navsys        =61",
    }
    if solver == "rtk":
        replacements["pos1-posmode       =kinematic"] = "pos1-posmode       =kinematic"
    else:
        replacements["pos1-posmode       =kinematic"] = "pos1-posmode       =ppp-kine"
        replacements["pos2-armode        =continuous"] = "pos2-armode        =off"
    for old, new in replacements.items():
        text = text.replace(old, new)
    return text


def run_rtklib_solver(
    args: argparse.Namespace,
    rover: Path,
    base: Path | None,
    nav: Path,
    reference: list[comparison.ReferenceEpoch],
    rtklib_pos: Path,
) -> float:
    if args.solver != "rtk":
        raise SystemExit("RTKLIB comparison in ppc-demo is currently supported for --solver rtk only")
    if base is None:
        raise SystemExit("RTKLIB comparison requires a base observation file")
    if not reference:
        raise SystemExit("RTKLIB comparison requires at least one reference epoch")

    rtklib_bin = Path(args.rtklib_bin)
    command = [str(rtklib_bin)]
    with tempfile.TemporaryDirectory(prefix="ppc_rtklib_conf_") as temp_dir:
        temp_dir_path = Path(temp_dir)
        config_path = temp_dir_path / "ppc_rtklib.conf"
        config_path.write_text(rtklib_config_text(Path(args.rtklib_config), args.solver), encoding="utf-8")
        command.extend(["-k", str(config_path), "-o", str(rtklib_pos)])
        window_reference = reference
        if args.max_epochs > 0:
            window_reference = reference[:args.max_epochs]
        start_day, start_time = gps_week_tow_to_datetime_strings(
            window_reference[0].week, window_reference[0].tow
        )
        end_day, end_time = gps_week_tow_to_datetime_strings(
            window_reference[-1].week, window_reference[-1].tow
        )
        command.extend(["-ts", start_day, start_time, "-te", end_day, end_time])
        command.extend([str(rover), str(base), str(nav)])
        start = time.perf_counter()
        run_command(command)
        return time.perf_counter() - start


def resolve_run_dir(args: argparse.Namespace) -> Path:
    if args.run_dir is not None:
        return args.run_dir
    if args.dataset_root is None or args.city is None:
        raise SystemExit("Provide either --run-dir or both --dataset-root and --city")
    return args.dataset_root / args.city / args.run


def dataset_label(args: argparse.Namespace, run_dir: Path) -> tuple[str, str, str]:
    city = args.city or run_dir.parent.name
    run_name = args.run if args.run_dir is None else run_dir.name
    slug = f"{city}_{run_name}".replace("-", "_")
    return city, run_name, slug


def solver_fixed_status(solver: str) -> int:
    if solver == "ppp":
        return 6
    return 4


def ppc_receiver_observation_provenance(city: str) -> dict[str, object]:
    normalized = city.strip().lower()
    hardware = PPC_RECEIVER_HARDWARE.get(normalized)
    payload: dict[str, object] = {
        "city": normalized,
        "vehicle_observation_format": "RINEX 3.04",
        "vehicle_observation_rate_hz": 5.0,
        "reference_station_observation_format": "RINEX 3.04",
        "reference_station_observation_rate_hz": 1.0,
        "receiver_engine_solution_available": False,
        "receiver_engine_solution_role": "not used as benchmark target",
        "benchmark_role": "survey-grade receiver observations plus reference trajectory truth",
    }
    if hardware is not None:
        payload.update(hardware)
    else:
        payload["hardware_status"] = "unknown city; PPC README documents tokyo and nagoya hardware"
    return payload


def resolve_paths(args: argparse.Namespace) -> tuple[Path, Path | None, Path, Path, Path, Path]:
    run_dir = resolve_run_dir(args)
    city, run_name, slug = dataset_label(args, run_dir)
    rover = args.rover or (run_dir / "rover.obs")
    base = args.base or (run_dir / "base.obs")
    nav = args.nav or (run_dir / "base.nav")
    reference_csv = args.reference_csv or (run_dir / "reference.csv")
    out = args.out or (ROOT_DIR / "output" / f"ppc_{slug}_{args.solver}.pos")
    summary_json = args.summary_json or (ROOT_DIR / "output" / f"ppc_{slug}_{args.solver}_summary.json")
    args._dataset_city = city
    args._dataset_run = run_name
    return rover, base if args.solver == "rtk" else None, nav, reference_csv, out, summary_json


def run_solver(
    args: argparse.Namespace,
    rover: Path,
    base: Path | None,
    nav: Path,
    out: Path,
) -> float:
    gnss_command = resolve_gnss_command(ROOT_DIR)
    if args.solver == "rtk":
        assert base is not None
        command = [
            *gnss_command,
            "solve",
            "--rover",
            str(rover),
            "--base",
            str(base),
            "--nav",
            str(nav),
            "--out",
            str(out),
            "--mode",
            "kinematic",
            "--no-kml",
        ]
        if args.preset is not None:
            command.extend(["--preset", args.preset])
        if getattr(args, "iono", None) is not None:
            command.extend(["--iono", args.iono])
        if getattr(args, "ratio", None) is not None:
            command.extend(["--ratio", str(args.ratio)])
        if getattr(args, "max_hold_div", None) is not None:
            command.extend(["--max-hold-div", str(args.max_hold_div)])
        if getattr(args, "max_pos_jump", None) is not None:
            command.extend(["--max-pos-jump", str(args.max_pos_jump)])
        if getattr(args, "max_pos_jump_min", None) is not None:
            command.extend(["--max-pos-jump-min", str(args.max_pos_jump_min)])
        if getattr(args, "max_pos_jump_rate", None) is not None:
            command.extend(["--max-pos-jump-rate", str(args.max_pos_jump_rate)])
        if getattr(args, "max_float_spp_div", None) is not None:
            command.extend(["--max-float-spp-div", str(args.max_float_spp_div)])
        if getattr(args, "max_float_prefit_rms", None) is not None:
            command.extend(["--max-float-prefit-rms", str(args.max_float_prefit_rms)])
        if getattr(args, "max_float_prefit_max", None) is not None:
            command.extend(["--max-float-prefit-max", str(args.max_float_prefit_max)])
        if getattr(args, "max_consec_float_reset", None) is not None:
            command.extend(["--max-consec-float-reset", str(args.max_consec_float_reset)])
        if getattr(args, "max_consec_nonfix_reset", None) is not None:
            command.extend(["--max-consec-nonfix-reset", str(args.max_consec_nonfix_reset)])
        if getattr(args, "max_postfix_rms", None) is not None:
            command.extend(["--max-postfix-rms", str(args.max_postfix_rms)])
        if getattr(args, "enable_wide_lane_ar", False):
            command.append("--enable-wide-lane-ar")
        if getattr(args, "wide_lane_threshold", None) is not None:
            command.extend(["--wide-lane-threshold", str(args.wide_lane_threshold)])
        if args.arfilter is True:
            command.append("--arfilter")
        elif args.arfilter is False:
            command.append("--no-arfilter")
        if args.arfilter_margin is not None:
            command.extend(["--arfilter-margin", str(args.arfilter_margin)])
        if args.min_hold_count is not None:
            command.extend(["--min-hold-count", str(args.min_hold_count)])
        if args.hold_ratio_threshold is not None:
            command.extend(["--hold-ratio-threshold", str(args.hold_ratio_threshold)])
        if getattr(args, "no_kinematic_post_filter", False):
            command.append("--no-kinematic-post-filter")
        if getattr(args, "no_nonfix_drift_guard", False):
            command.append("--no-nonfix-drift-guard")
        if getattr(args, "nonfix_drift_max_anchor_gap", None) is not None:
            command.extend(["--nonfix-drift-max-anchor-gap", str(args.nonfix_drift_max_anchor_gap)])
        if getattr(args, "nonfix_drift_max_anchor_speed", None) is not None:
            command.extend(["--nonfix-drift-max-anchor-speed", str(args.nonfix_drift_max_anchor_speed)])
        if getattr(args, "nonfix_drift_max_residual", None) is not None:
            command.extend(["--nonfix-drift-max-residual", str(args.nonfix_drift_max_residual)])
        if getattr(args, "nonfix_drift_min_horizontal_residual", None) is not None:
            command.extend([
                "--nonfix-drift-min-horizontal-residual",
                str(args.nonfix_drift_min_horizontal_residual),
            ])
        if getattr(args, "nonfix_drift_min_segment_epochs", None) is not None:
            command.extend(["--nonfix-drift-min-segment-epochs", str(args.nonfix_drift_min_segment_epochs)])
        if getattr(args, "nonfix_drift_max_segment_epochs", None) is not None:
            command.extend(["--nonfix-drift-max-segment-epochs", str(args.nonfix_drift_max_segment_epochs)])
        if getattr(args, "no_spp_height_step_guard", False):
            command.append("--no-spp-height-step-guard")
        if getattr(args, "spp_height_step_min", None) is not None:
            command.extend(["--spp-height-step-min", str(args.spp_height_step_min)])
        if getattr(args, "spp_height_step_rate", None) is not None:
            command.extend(["--spp-height-step-rate", str(args.spp_height_step_rate)])
        if getattr(args, "float_bridge_tail_guard", True):
            command.append("--float-bridge-tail-guard")
        else:
            command.append("--no-float-bridge-tail-guard")
        if getattr(args, "float_bridge_tail_max_anchor_gap", None) is not None:
            command.extend(
                [
                    "--float-bridge-tail-max-anchor-gap",
                    str(args.float_bridge_tail_max_anchor_gap),
                ]
            )
        if getattr(args, "float_bridge_tail_min_anchor_speed", None) is not None:
            command.extend(
                [
                    "--float-bridge-tail-min-anchor-speed",
                    str(args.float_bridge_tail_min_anchor_speed),
                ]
            )
        if getattr(args, "float_bridge_tail_max_anchor_speed", None) is not None:
            command.extend(
                [
                    "--float-bridge-tail-max-anchor-speed",
                    str(args.float_bridge_tail_max_anchor_speed),
                ]
            )
        if getattr(args, "float_bridge_tail_max_residual", None) is not None:
            command.extend(
                [
                    "--float-bridge-tail-max-residual",
                    str(args.float_bridge_tail_max_residual),
                ]
            )
        if getattr(args, "float_bridge_tail_min_segment_epochs", None) is not None:
            command.extend(
                [
                    "--float-bridge-tail-min-segment-epochs",
                    str(args.float_bridge_tail_min_segment_epochs),
                ]
            )
        if getattr(args, "fixed_bridge_burst_guard", False):
            command.append("--fixed-bridge-burst-guard")
        if getattr(args, "fixed_bridge_burst_max_anchor_gap", None) is not None:
            command.extend([
                "--fixed-bridge-burst-max-anchor-gap",
                str(args.fixed_bridge_burst_max_anchor_gap),
            ])
        if getattr(args, "fixed_bridge_burst_min_boundary_gap", None) is not None:
            command.extend([
                "--fixed-bridge-burst-min-boundary-gap",
                str(args.fixed_bridge_burst_min_boundary_gap),
            ])
        if getattr(args, "fixed_bridge_burst_max_residual", None) is not None:
            command.extend([
                "--fixed-bridge-burst-max-residual",
                str(args.fixed_bridge_burst_max_residual),
            ])
        if getattr(args, "fixed_bridge_burst_max_segment_epochs", None) is not None:
            command.extend([
                "--fixed-bridge-burst-max-segment-epochs",
                str(args.fixed_bridge_burst_max_segment_epochs),
            ])
    else:
        command = [
            *gnss_command,
            "ppp",
            "--kinematic",
            "--obs",
            str(rover),
            "--nav",
            str(nav),
            "--out",
            str(out),
        ]
        if args.low_dynamics:
            command.append("--low-dynamics")
        if args.sp3 is not None:
            command.extend(["--sp3", str(args.sp3)])
        if args.clk is not None:
            command.extend(["--clk", str(args.clk)])
        if args.ionex is not None:
            command.extend(["--ionex", str(args.ionex)])
        if args.dcb is not None:
            command.extend(["--dcb", str(args.dcb)])
        if args.antex is not None:
            command.extend(["--antex", str(args.antex)])
        if args.blq is not None:
            command.extend(["--blq", str(args.blq)])
        if args.enable_ar:
            command.append("--enable-ar")
        if args.ppp_summary_json is not None:
            command.extend(["--summary-json", str(args.ppp_summary_json)])
    if args.max_epochs > 0:
        command.extend(["--max-epochs", str(args.max_epochs)])
    start = time.perf_counter()
    run_command(command)
    return time.perf_counter() - start


def build_summary_payload(
    args: argparse.Namespace,
    run_dir: Path,
    rover: Path,
    base: Path | None,
    nav: Path,
    reference_csv: Path,
    out: Path,
    summary_json: Path,
    solver_wall_time_s: float | None = None,
) -> dict[str, object]:
    reference = read_flexible_reference_csv(reference_csv)
    solution_epochs = comparison.read_libgnss_pos(out)
    lib_metrics = ppc_metrics.summarize_solution_epochs(
        reference,
        solution_epochs,
        solver_fixed_status(args.solver),
        args.solver,
        args.match_tolerance_s,
        solver_wall_time_s,
    )

    payload = {
        "dataset": f"PPC-Dataset {args._dataset_city} {args._dataset_run}",
        "solver": args.solver,
        "run_dir": str(run_dir),
        "rover": str(rover),
        "base": str(base) if base is not None else None,
        "nav": str(nav),
        "reference_csv": str(reference_csv),
        "receiver_observation_provenance": ppc_receiver_observation_provenance(args._dataset_city),
        "rtk_iono": getattr(args, "iono", None) if args.solver == "rtk" else None,
        "rtk_ratio_threshold": getattr(args, "ratio", None) if args.solver == "rtk" else None,
        "rtk_max_hold_divergence_m": getattr(args, "max_hold_div", None) if args.solver == "rtk" else None,
        "rtk_max_position_jump_m": getattr(args, "max_pos_jump", None) if args.solver == "rtk" else None,
        "rtk_max_position_jump_min_m": (
            getattr(args, "max_pos_jump_min", None) if args.solver == "rtk" else None
        ),
        "rtk_max_position_jump_rate_mps": (
            getattr(args, "max_pos_jump_rate", None) if args.solver == "rtk" else None
        ),
        "rtk_max_float_spp_divergence_m": (
            getattr(args, "max_float_spp_div", None) if args.solver == "rtk" else None
        ),
        "rtk_max_float_prefit_residual_rms_m": (
            getattr(args, "max_float_prefit_rms", None) if args.solver == "rtk" else None
        ),
        "rtk_max_float_prefit_residual_max_m": (
            getattr(args, "max_float_prefit_max", None) if args.solver == "rtk" else None
        ),
        "rtk_max_consecutive_float_for_reset": (
            getattr(args, "max_consec_float_reset", None) if args.solver == "rtk" else None
        ),
        "rtk_max_consecutive_nonfix_for_reset": (
            getattr(args, "max_consec_nonfix_reset", None) if args.solver == "rtk" else None
        ),
        "rtk_max_postfix_residual_rms_m": (
            getattr(args, "max_postfix_rms", None) if args.solver == "rtk" else None
        ),
        "rtk_wide_lane_ar_enabled": bool(
            args.solver == "rtk" and getattr(args, "enable_wide_lane_ar", False)
        ),
        "rtk_wide_lane_threshold": (
            getattr(args, "wide_lane_threshold", None) if args.solver == "rtk" else None
        ),
        "rtk_output_profile": "coverage" if getattr(args, "no_kinematic_post_filter", False) else "precision",
        "kinematic_post_filter_enabled": not getattr(args, "no_kinematic_post_filter", False),
        "nonfix_drift_guard_enabled": args.solver == "rtk" and not getattr(args, "no_nonfix_drift_guard", False),
        "nonfix_drift_guard": nonfix_drift_guard_config(args) if args.solver == "rtk" else None,
        "spp_height_step_guard_enabled": (
            args.solver == "rtk" and not getattr(args, "no_spp_height_step_guard", False)
        ),
        "spp_height_step_guard": spp_height_step_guard_config(args) if args.solver == "rtk" else None,
        "float_bridge_tail_guard_enabled": (
            args.solver == "rtk" and getattr(args, "float_bridge_tail_guard", True)
        ),
        "float_bridge_tail_guard": float_bridge_tail_guard_config(args) if args.solver == "rtk" else None,
        "fixed_bridge_burst_guard_enabled": (
            args.solver == "rtk" and getattr(args, "fixed_bridge_burst_guard", False)
        ),
        "fixed_bridge_burst_guard": fixed_bridge_burst_guard_config(args) if args.solver == "rtk" else None,
        "solution_pos": str(out),
        "summary_json": str(summary_json),
        "generated_solution": not args.use_existing_solution,
        "valid_epochs": lib_metrics["valid_epochs"],
        "reference_epochs": len(reference),
        "matched_epochs": lib_metrics["matched_epochs"],
        "fixed_epochs": lib_metrics["fixed_epochs"],
        "positioning_rate_pct": lib_metrics["positioning_rate_pct"],
        "fix_rate_pct": lib_metrics["fix_rate_pct"],
        "mean_h_m": lib_metrics["mean_h_m"],
        "median_h_m": lib_metrics["median_h_m"],
        "p95_h_m": lib_metrics["p95_h_m"],
        "max_h_m": lib_metrics["max_h_m"],
        "ppc_score_3d_50cm_epochs": lib_metrics["ppc_score_3d_50cm_epochs"],
        "ppc_score_3d_50cm_matched_pct": lib_metrics["ppc_score_3d_50cm_matched_pct"],
        "ppc_score_3d_50cm_ref_pct": lib_metrics["ppc_score_3d_50cm_ref_pct"],
        "ppc_official_score_threshold_m": lib_metrics["ppc_official_score_threshold_m"],
        "ppc_official_total_distance_m": lib_metrics["ppc_official_total_distance_m"],
        "ppc_official_matched_distance_m": lib_metrics["ppc_official_matched_distance_m"],
        "ppc_official_score_distance_m": lib_metrics["ppc_official_score_distance_m"],
        "ppc_official_score_pct": lib_metrics["ppc_official_score_pct"],
        "median_abs_up_m": lib_metrics["median_abs_up_m"],
        "p95_abs_up_m": lib_metrics["p95_abs_up_m"],
        "mean_up_m": lib_metrics["mean_up_m"],
        "mean_satellites": lib_metrics["mean_satellites"],
        "match_tolerance_s": rounded(args.match_tolerance_s),
        "solution_span_s": lib_metrics["solution_span_s"],
        "solver_wall_time_s": lib_metrics["solver_wall_time_s"],
        "realtime_factor": lib_metrics["realtime_factor"],
        "effective_epoch_rate_hz": lib_metrics["effective_epoch_rate_hz"],
    }

    if args.solver == "ppp" and args.ppp_summary_json is not None and args.ppp_summary_json.exists():
        ppp_run_summary = json.loads(args.ppp_summary_json.read_text(encoding="utf-8"))
        if isinstance(ppp_run_summary, dict):
            payload.update(
                {
                    "ppp_run_summary": ppp_run_summary,
                    "ppp_converged": bool(ppp_run_summary.get("converged", False)),
                    "ppp_convergence_time_s": rounded(
                        float(ppp_run_summary.get("convergence_time_s", 0.0))
                    ),
                    "ppp_solution_rate_pct": rounded(
                        float(ppp_run_summary.get("solution_rate_pct", 0.0))
                    ),
                    "ionex_corrections": int(ppp_run_summary.get("ionex_corrections", 0)),
                    "ionex_meters": rounded(float(ppp_run_summary.get("ionex_meters", 0.0))),
                    "dcb_corrections": int(ppp_run_summary.get("dcb_corrections", 0)),
                    "dcb_meters": rounded(float(ppp_run_summary.get("dcb_meters", 0.0))),
                }
            )

    rtklib_pos = getattr(args, "rtklib_pos", None)
    if rtklib_pos is not None and Path(rtklib_pos).exists():
        rtklib_metrics = ppc_metrics.summarize_solution_epochs(
            reference,
            comparison.read_rtklib_pos(Path(rtklib_pos)),
            1,
            "RTKLIB",
            args.match_tolerance_s,
            getattr(args, "rtklib_solver_wall_time_s", None),
        )
        payload["rtklib_pos"] = str(rtklib_pos)
        payload["rtklib_generated_solution"] = not getattr(args, "use_existing_rtklib_solution", False)
        payload["rtklib"] = rtklib_metrics
        payload["delta_vs_rtklib"] = {
            "fix_rate_pct": rounded(float(payload["fix_rate_pct"]) - float(rtklib_metrics["fix_rate_pct"])),
            "positioning_rate_pct": rounded(
                float(payload["positioning_rate_pct"]) - float(rtklib_metrics["positioning_rate_pct"])
            ),
            "median_h_m": rounded(float(payload["median_h_m"]) - float(rtklib_metrics["median_h_m"])),
            "p95_h_m": rounded(float(payload["p95_h_m"]) - float(rtklib_metrics["p95_h_m"])),
            "max_h_m": rounded(float(payload["max_h_m"]) - float(rtklib_metrics["max_h_m"])),
            "ppc_score_3d_50cm_matched_pct": rounded(
                float(payload["ppc_score_3d_50cm_matched_pct"])
                - float(rtklib_metrics["ppc_score_3d_50cm_matched_pct"])
            ),
            "ppc_score_3d_50cm_ref_pct": rounded(
                float(payload["ppc_score_3d_50cm_ref_pct"]) - float(rtklib_metrics["ppc_score_3d_50cm_ref_pct"])
            ),
            "ppc_official_score_pct": rounded(
                float(payload["ppc_official_score_pct"]) - float(rtklib_metrics["ppc_official_score_pct"])
            ),
            "ppc_official_matched_distance_m": rounded(
                float(payload["ppc_official_matched_distance_m"])
                - float(rtklib_metrics["ppc_official_matched_distance_m"])
            ),
            "ppc_official_score_distance_m": rounded(
                float(payload["ppc_official_score_distance_m"])
                - float(rtklib_metrics["ppc_official_score_distance_m"])
            ),
            "solver_wall_time_s": (
                rounded(float(payload["solver_wall_time_s"]) - float(rtklib_metrics["solver_wall_time_s"]))
                if payload["solver_wall_time_s"] is not None and rtklib_metrics["solver_wall_time_s"] is not None
                else None
            ),
            "realtime_factor": (
                rounded(float(payload["realtime_factor"]) - float(rtklib_metrics["realtime_factor"]))
                if payload["realtime_factor"] is not None and rtklib_metrics["realtime_factor"] is not None
                else None
            ),
        }

    if args.commercial_pos is not None and Path(args.commercial_pos).exists():
        commercial_metrics = ppc_commercial.summarize_existing_receiver_solution(
            reference=reference,
            solution_pos=Path(args.commercial_pos),
            requested_format=getattr(args, "commercial_format", "auto"),
            label=getattr(args, "commercial_label", "commercial_receiver"),
            matched_csv=getattr(args, "commercial_matched_csv", None),
            match_tolerance_s=args.match_tolerance_s,
            solver_wall_time_s=getattr(args, "commercial_solver_wall_time_s", None),
        )
        payload["commercial_receiver"] = commercial_metrics
        payload["delta_vs_commercial_receiver"] = ppc_metrics.solution_metric_delta(
            payload,
            commercial_metrics,
        )
    elif args.commercial_rover is not None and args.commercial_out is not None and args.commercial_out.exists():
        commercial_metrics = ppc_commercial.summarize_solved_receiver_observations(
            reference=reference,
            solution_pos=args.commercial_out,
            label=getattr(args, "commercial_label", "commercial_receiver"),
            matched_csv=getattr(args, "commercial_matched_csv", None),
            match_tolerance_s=args.match_tolerance_s,
            solver_wall_time_s=getattr(args, "commercial_solver_wall_time_s", None),
            generated_solution=not bool(getattr(args, "use_existing_commercial_solution", False)),
            rover=args.commercial_rover,
            base=args.commercial_base or base,
            nav=args.commercial_nav or nav,
        )
        payload["commercial_receiver"] = commercial_metrics
        payload["delta_vs_commercial_receiver"] = ppc_metrics.solution_metric_delta(
            payload,
            commercial_metrics,
        )

    summary_json.parent.mkdir(parents=True, exist_ok=True)
    summary_json.write_text(json.dumps(payload, indent=2, sort_keys=True) + "\n", encoding="utf-8")
    return payload


def enforce_summary_requirements(payload: dict[str, object], args: argparse.Namespace) -> None:
    failures: list[str] = []
    if (
        args.require_valid_epochs_min is not None
        and int(payload["valid_epochs"]) < args.require_valid_epochs_min
    ):
        failures.append(
            f"valid epochs {int(payload['valid_epochs'])} < {args.require_valid_epochs_min}"
        )
    if (
        args.require_matched_epochs_min is not None
        and int(payload["matched_epochs"]) < args.require_matched_epochs_min
    ):
        failures.append(
            f"matched epochs {int(payload['matched_epochs'])} < {args.require_matched_epochs_min}"
        )
    if (
        args.require_fix_rate_min is not None
        and float(payload["fix_rate_pct"]) < args.require_fix_rate_min
    ):
        failures.append(
            f"fix rate {float(payload['fix_rate_pct']):.6f}% < {args.require_fix_rate_min:.6f}%"
        )
    if (
        getattr(args, "require_positioning_rate_min", None) is not None
        and float(payload["positioning_rate_pct"]) < args.require_positioning_rate_min
    ):
        failures.append(
            "positioning rate "
            f"{float(payload['positioning_rate_pct']):.6f}% < {args.require_positioning_rate_min:.6f}%"
        )
    if (
        getattr(args, "require_ppc_official_score_min", None) is not None
        and float(payload["ppc_official_score_pct"]) < args.require_ppc_official_score_min
    ):
        failures.append(
            "PPC official distance score "
            f"{float(payload['ppc_official_score_pct']):.6f}% < "
            f"{args.require_ppc_official_score_min:.6f}%"
        )
    if (
        getattr(args, "require_ppc_score_3d_50cm_ref_min", None) is not None
        and float(payload["ppc_score_3d_50cm_ref_pct"]) < args.require_ppc_score_3d_50cm_ref_min
    ):
        failures.append(
            "PPC 3D<=50cm reference score "
            f"{float(payload['ppc_score_3d_50cm_ref_pct']):.6f}% < "
            f"{args.require_ppc_score_3d_50cm_ref_min:.6f}%"
        )
    if (
        args.require_median_h_max is not None
        and float(payload["median_h_m"]) > args.require_median_h_max
    ):
        failures.append(
            f"median horizontal error {float(payload['median_h_m']):.6f} m > {args.require_median_h_max:.6f} m"
        )
    if (
        args.require_p95_h_max is not None
        and float(payload["p95_h_m"]) > args.require_p95_h_max
    ):
        failures.append(
            f"p95 horizontal error {float(payload['p95_h_m']):.6f} m > {args.require_p95_h_max:.6f} m"
        )
    if (
        args.require_max_h_max is not None
        and float(payload["max_h_m"]) > args.require_max_h_max
    ):
        failures.append(
            f"max horizontal error {float(payload['max_h_m']):.6f} m > {args.require_max_h_max:.6f} m"
        )
    if (
        args.require_p95_up_max is not None
        and float(payload["p95_abs_up_m"]) > args.require_p95_up_max
    ):
        failures.append(
            f"p95 absolute up error {float(payload['p95_abs_up_m']):.6f} m > {args.require_p95_up_max:.6f} m"
        )
    if (
        args.require_mean_sats_min is not None
        and float(payload["mean_satellites"]) < args.require_mean_sats_min
    ):
        failures.append(
            f"mean satellites {float(payload['mean_satellites']):.6f} < {args.require_mean_sats_min:.6f}"
        )
    if args.require_solver_wall_time_max is not None:
        solver_wall_time_s = payload["solver_wall_time_s"]
        if solver_wall_time_s is None:
            failures.append("solver wall time is unavailable")
        elif float(solver_wall_time_s) > args.require_solver_wall_time_max:
            failures.append(
                f"solver wall time {float(solver_wall_time_s):.6f} s > {args.require_solver_wall_time_max:.6f} s"
            )
    if args.require_realtime_factor_min is not None:
        realtime_factor = payload["realtime_factor"]
        if realtime_factor is None:
            failures.append("realtime factor is unavailable")
        elif float(realtime_factor) < args.require_realtime_factor_min:
            failures.append(
                f"realtime factor {float(realtime_factor):.6f} < {args.require_realtime_factor_min:.6f}"
            )
    if args.require_effective_epoch_rate_min is not None:
        effective_epoch_rate_hz = payload["effective_epoch_rate_hz"]
        if effective_epoch_rate_hz is None:
            failures.append("effective epoch rate is unavailable")
        elif float(effective_epoch_rate_hz) < args.require_effective_epoch_rate_min:
            failures.append(
                f"effective epoch rate {float(effective_epoch_rate_hz):.6f} Hz < {args.require_effective_epoch_rate_min:.6f} Hz"
            )

    if (
        args.require_lib_fix_rate_vs_rtklib_min_delta is not None
        or args.require_lib_median_h_vs_rtklib_max_delta is not None
        or args.require_lib_p95_h_vs_rtklib_max_delta is not None
    ):
        if "delta_vs_rtklib" not in payload:
            failures.append("RTKLIB comparison summary is unavailable")
        else:
            deltas = payload["delta_vs_rtklib"]
            if args.require_lib_fix_rate_vs_rtklib_min_delta is not None:
                if float(deltas["fix_rate_pct"]) < args.require_lib_fix_rate_vs_rtklib_min_delta:
                    failures.append(
                        "lib fix rate vs RTKLIB delta "
                        f"{float(deltas['fix_rate_pct']):.6f}% < "
                        f"{args.require_lib_fix_rate_vs_rtklib_min_delta:.6f}%"
                    )
            if args.require_lib_median_h_vs_rtklib_max_delta is not None:
                if float(deltas["median_h_m"]) > args.require_lib_median_h_vs_rtklib_max_delta:
                    failures.append(
                        "lib median horizontal vs RTKLIB delta "
                        f"{float(deltas['median_h_m']):.6f} m > "
                        f"{args.require_lib_median_h_vs_rtklib_max_delta:.6f} m"
                    )
            if args.require_lib_p95_h_vs_rtklib_max_delta is not None:
                if float(deltas["p95_h_m"]) > args.require_lib_p95_h_vs_rtklib_max_delta:
                    failures.append(
                        "lib p95 horizontal vs RTKLIB delta "
                        f"{float(deltas['p95_h_m']):.6f} m > "
                        f"{args.require_lib_p95_h_vs_rtklib_max_delta:.6f} m"
                    )

    if failures:
        raise SystemExit(
            "PPC demo checks failed:\n" + "\n".join(f"  - {failure}" for failure in failures)
        )


def main() -> int:
    args = parse_args()
    rover, base, nav, reference_csv, out, summary_json = resolve_paths(args)
    run_dir = resolve_run_dir(args)
    rtklib_pos = args.rtklib_pos
    if rtklib_pos is None and args.rtklib_bin is not None:
        rtklib_pos = summary_json.with_name(summary_json.stem.replace("_summary", "_rtklib") + ".pos")
        args.rtklib_pos = rtklib_pos
    if args.commercial_rover is not None and args.commercial_out is None:
        args.commercial_out = ppc_commercial.default_commercial_out(summary_json)

    ensure_input_exists(reference_csv, "PPC reference CSV", ROOT_DIR)
    if args.commercial_pos is not None and args.commercial_rover is not None:
        raise SystemExit("Use either --commercial-pos or --commercial-rover, not both")
    if args.commercial_pos is not None:
        ensure_input_exists(args.commercial_pos, "commercial receiver solution", ROOT_DIR)
    elif args.commercial_rover is not None:
        if args.solver != "rtk":
            raise SystemExit("--commercial-rover is currently supported only with --solver rtk")
        if base is None:
            raise SystemExit("--commercial-rover requires a primary or explicit base observation file")
        assert args.commercial_out is not None
        ensure_input_exists(args.commercial_rover, "commercial rover observation file", ROOT_DIR)
        ensure_input_exists(args.commercial_base or base, "commercial base observation file", ROOT_DIR)
        ensure_input_exists(args.commercial_nav or nav, "commercial navigation file", ROOT_DIR)
        if args.use_existing_commercial_solution:
            ensure_input_exists(args.commercial_out, "existing commercial receiver solution", ROOT_DIR)
    elif args.commercial_matched_csv is not None:
        raise SystemExit("--commercial-matched-csv requires --commercial-pos or --commercial-rover")
    elif args.use_existing_commercial_solution:
        raise SystemExit("--use-existing-commercial-solution requires --commercial-rover")
    if args.max_epochs == 0:
        raise SystemExit("--max-epochs must be positive or -1")

    if args.use_existing_solution:
        ensure_input_exists(out, "existing solution file", ROOT_DIR)
        if args.solver == "ppp" and args.ppp_summary_json is not None:
            ensure_input_exists(args.ppp_summary_json, "existing PPP summary JSON", ROOT_DIR)
    else:
        ensure_input_exists(rover, "PPC rover observation file", ROOT_DIR)
        ensure_input_exists(nav, "PPC navigation file", ROOT_DIR)
        if base is not None:
            ensure_input_exists(base, "PPC base observation file", ROOT_DIR)
        if args.sp3 is not None:
            ensure_input_exists(args.sp3, "PPP SP3 file", ROOT_DIR)
        if args.clk is not None:
            ensure_input_exists(args.clk, "PPP CLK file", ROOT_DIR)
        if args.ionex is not None:
            ensure_input_exists(args.ionex, "PPP IONEX file", ROOT_DIR)
        if args.dcb is not None:
            ensure_input_exists(args.dcb, "PPP DCB file", ROOT_DIR)
        if args.antex is not None:
            ensure_input_exists(args.antex, "PPP ANTEX file", ROOT_DIR)
        if args.blq is not None:
            ensure_input_exists(args.blq, "PPP BLQ file", ROOT_DIR)
        if args.ppp_summary_json is not None:
            args.ppp_summary_json.parent.mkdir(parents=True, exist_ok=True)

        out.parent.mkdir(parents=True, exist_ok=True)
        measured_wall_time_s = run_solver(args, rover, base, nav, out)
        if args.solver_wall_time_s is None:
            args.solver_wall_time_s = measured_wall_time_s

    if args.rtklib_bin is not None or args.use_existing_rtklib_solution:
        if args.solver != "rtk":
            raise SystemExit("PPC RTKLIB comparison is currently supported only for --solver rtk")
        if args.rtklib_bin is not None:
            ensure_input_exists(args.rtklib_bin, "RTKLIB binary", ROOT_DIR)
            ensure_input_exists(args.rtklib_config, "RTKLIB config", ROOT_DIR)
        if args.use_existing_rtklib_solution:
            if rtklib_pos is None:
                raise SystemExit("--use-existing-rtklib-solution requires --rtklib-pos")
            ensure_input_exists(rtklib_pos, "existing RTKLIB solution file", ROOT_DIR)
        else:
            reference = read_flexible_reference_csv(reference_csv)
            if rtklib_pos is None:
                raise SystemExit("missing RTKLIB output path")
            rtklib_pos.parent.mkdir(parents=True, exist_ok=True)
            measured_rtklib_wall_time_s = run_rtklib_solver(args, rover, base, nav, reference, rtklib_pos)
            if args.rtklib_solver_wall_time_s is None:
                args.rtklib_solver_wall_time_s = measured_rtklib_wall_time_s

    if args.commercial_rover is not None and not args.use_existing_commercial_solution:
        assert base is not None
        assert args.commercial_out is not None
        args.commercial_out.parent.mkdir(parents=True, exist_ok=True)
        commercial_solve = ppc_commercial.CommercialRoverSolve(
            rover=args.commercial_rover,
            base=args.commercial_base or base,
            nav=args.commercial_nav or nav,
            out=args.commercial_out,
            max_epochs=args.max_epochs,
            tuning=ppc_commercial.CommercialRoverTuning(
                preset=args.commercial_preset,
                arfilter=args.commercial_arfilter,
                arfilter_margin=args.commercial_arfilter_margin,
                min_hold_count=args.commercial_min_hold_count,
                hold_ratio_threshold=args.commercial_hold_ratio_threshold,
            ),
        )
        measured_commercial_wall_time_s = ppc_commercial.run_commercial_rover_solver(
            resolve_gnss_command(ROOT_DIR),
            commercial_solve,
            run_command,
        )
        if args.commercial_solver_wall_time_s is None:
            args.commercial_solver_wall_time_s = measured_commercial_wall_time_s

    payload = build_summary_payload(
        args,
        run_dir,
        rover,
        base,
        nav,
        reference_csv,
        out,
        summary_json,
        solver_wall_time_s=args.solver_wall_time_s,
    )
    enforce_summary_requirements(payload, args)

    print("Finished PPC-Dataset demo.")
    print(f"  dataset: {payload['dataset']}")
    print(f"  solver: {args.solver}")
    print(f"  solution: {out}")
    print(f"  summary: {summary_json}")
    if payload["solver_wall_time_s"] is not None:
        print(
            "  performance:"
            f" wall={payload['solver_wall_time_s']} s"
            f", span={payload['solution_span_s']} s"
            f", rtf={payload['realtime_factor']}"
            f", rate={payload['effective_epoch_rate_hz']} Hz"
        )
    if "rtklib" in payload:
        rtklib = payload["rtklib"]
        print(f"  rtklib: {payload['rtklib_pos']}")
        print(
            "  rtklib performance:"
            f" wall={rtklib['solver_wall_time_s']} s"
            f", span={rtklib['solution_span_s']} s"
            f", rtf={rtklib['realtime_factor']}"
            f", rate={rtklib['effective_epoch_rate_hz']} Hz"
        )
        print(
            "  vs rtklib:"
            f" fix={payload['delta_vs_rtklib']['fix_rate_pct']} %"
            f", positioning={payload['delta_vs_rtklib']['positioning_rate_pct']} %"
            f", ppc_official={payload['delta_vs_rtklib']['ppc_official_score_pct']} %"
            f", 3d50_ref={payload['delta_vs_rtklib']['ppc_score_3d_50cm_ref_pct']} %"
            f", p95_h={payload['delta_vs_rtklib']['p95_h_m']} m"
            f", wall={payload['delta_vs_rtklib']['solver_wall_time_s']} s"
        )
    if "commercial_receiver" in payload:
        commercial = payload["commercial_receiver"]
        print(f"  commercial_receiver: {commercial['solution_pos']}")
        print(
            "  vs commercial_receiver:"
            f" fix={payload['delta_vs_commercial_receiver']['fix_rate_pct']} %"
            f", p95_h={payload['delta_vs_commercial_receiver']['p95_h_m']} m"
        )
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
