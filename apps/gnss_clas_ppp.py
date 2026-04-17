#!/usr/bin/env python3
"""Run PPP with a named RTCM-carried CLAS/MADOCA correction profile."""

from __future__ import annotations

import argparse
import datetime as dt
import json
import os
from pathlib import Path
import socket
import subprocess
import sys
import tempfile

if os.name != "nt":
    import termios

from gnss_runtime import resolve_gnss_command
import gnss_qzss_l6_info as qzss_l6_info

ROOT_DIR = Path(__file__).resolve().parent.parent
PPP_STATUSES = {5, 6}
DEFAULT_SERIAL_BAUD = 115200
COMPACT_FILE_COLUMNS = (
    "week,tow,system,prn,dx,dy,dz,dclock_m"
    "[,high_rate_clock_m][,ura_sigma_m=<m>][,cbias:<id>=<m>...][,pbias:<id>=<m>...]"
    "[,bias_network_id=<n>][,atmos_<name>=<value>...]"
)


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(prog=os.environ.get("GNSS_CLI_NAME"))
    parser.add_argument(
        "--profile",
        choices=("clas", "madoca"),
        default="clas",
        help="Named RTCM-carried correction profile.",
    )
    parser.add_argument("--obs", type=Path, required=True, help="Rover observation RINEX file.")
    parser.add_argument("--nav", type=Path, required=True, help="Broadcast navigation RINEX file.")
    parser.add_argument(
        "--ssr-rtcm",
        default=None,
        help="RTCM SSR source: local file or ntrip:// / tcp:// / serial:// URI.",
    )
    parser.add_argument(
        "--compact-ssr",
        default=None,
        help=(
            "Compact sampled correction source: local file or tcp:// / serial:// URI. "
            f"Expected columns: {COMPACT_FILE_COLUMNS}"
        ),
    )
    parser.add_argument(
        "--qzss-l6",
        default=None,
        help="Direct QZSS L6 frame source: local file or serial:// URI.",
    )
    parser.add_argument(
        "--qzss-gps-week",
        type=int,
        default=None,
        help="Optional GPS week override used when decoding raw QZSS L6 corrections.",
    )
    parser.add_argument(
        "--compact-flush-policy",
        choices=qzss_l6_info.COMPACT_SSR_FLUSH_POLICIES,
        default=qzss_l6_info.COMPACT_SSR_FLUSH_POLICY_LAG_TOLERANT,
        help="Compact SSR row emission policy used when expanding raw QZSS L6 into sampled SSR CSV.",
    )
    parser.add_argument(
        "--compact-atmos-merge-policy",
        choices=qzss_l6_info.COMPACT_ATMOS_MERGE_POLICIES,
        default=qzss_l6_info.COMPACT_ATMOS_MERGE_POLICY_STEC_COEFF_CARRY,
        help="Compact SSR atmosphere-token merge policy used when expanding raw QZSS L6 into sampled SSR CSV.",
    )
    parser.add_argument(
        "--compact-atmos-subtype-merge-policy",
        choices=qzss_l6_info.COMPACT_ATMOS_SUBTYPE_MERGE_POLICIES,
        default=qzss_l6_info.COMPACT_ATMOS_SUBTYPE_MERGE_POLICY_UNION,
        help="Compact SSR atmosphere subtype precedence policy used when expanding raw QZSS L6 into sampled SSR CSV.",
    )
    parser.add_argument(
        "--compact-phase-bias-merge-policy",
        choices=qzss_l6_info.COMPACT_PHASE_BIAS_MERGE_POLICIES,
        default=qzss_l6_info.COMPACT_PHASE_BIAS_MERGE_POLICY_LATEST_UNION,
        help="Compact SSR phase-bias merge policy used when expanding raw QZSS L6 into sampled SSR CSV.",
    )
    parser.add_argument(
        "--compact-phase-bias-source-policy",
        choices=qzss_l6_info.COMPACT_PHASE_BIAS_SOURCE_POLICIES,
        default=qzss_l6_info.COMPACT_PHASE_BIAS_SOURCE_POLICY_ARRIVAL_ORDER,
        help="Compact SSR phase-bias source-row precedence policy used when expanding raw QZSS L6 into sampled SSR CSV.",
    )
    parser.add_argument(
        "--compact-code-bias-composition-policy",
        choices=qzss_l6_info.COMPACT_CODE_BIAS_COMPOSITION_POLICIES,
        default=qzss_l6_info.COMPACT_CODE_BIAS_COMPOSITION_POLICY_DIRECT,
        help="Compact SSR code-bias composition policy used when expanding raw QZSS L6 into sampled SSR CSV.",
    )
    parser.add_argument(
        "--compact-code-bias-bank-policy",
        choices=qzss_l6_info.COMPACT_CODE_BIAS_BANK_POLICIES,
        default=qzss_l6_info.COMPACT_CODE_BIAS_BANK_POLICY_PENDING_EPOCH,
        help="Compact SSR code-bias base-bank lookup policy used when expanding raw QZSS L6 into sampled SSR CSV.",
    )
    parser.add_argument(
        "--compact-phase-bias-composition-policy",
        choices=qzss_l6_info.COMPACT_PHASE_BIAS_COMPOSITION_POLICIES,
        default=qzss_l6_info.COMPACT_PHASE_BIAS_COMPOSITION_POLICY_DIRECT,
        help="Compact SSR phase-bias composition policy used when expanding raw QZSS L6 into sampled SSR CSV.",
    )
    parser.add_argument(
        "--compact-phase-bias-bank-policy",
        choices=qzss_l6_info.COMPACT_PHASE_BIAS_BANK_POLICIES,
        default=qzss_l6_info.COMPACT_PHASE_BIAS_BANK_POLICY_PENDING_EPOCH,
        help="Compact SSR phase-bias base-bank lookup policy used when expanding raw QZSS L6 into sampled SSR CSV.",
    )
    parser.add_argument(
        "--compact-bias-row-materialization",
        choices=qzss_l6_info.COMPACT_BIAS_ROW_MATERIALIZATION_POLICIES,
        default=qzss_l6_info.COMPACT_BIAS_ROW_MATERIALIZATION_POLICY_OVERLAP_ONLY,
        help="Compact SSR row materialization policy used when subtype-6 network rows extend missing base-bias signal rows.",
    )
    parser.add_argument(
        "--compact-row-construction-policy",
        choices=qzss_l6_info.COMPACT_ROW_CONSTRUCTION_POLICIES,
        default=qzss_l6_info.COMPACT_ROW_CONSTRUCTION_POLICY_INDEPENDENT,
        help="Compact SSR row construction policy controlling how subtype-4/subtype-6 row and value construction interact.",
    )
    parser.add_argument("--out", type=Path, required=True, help="Output PPP .pos file.")
    parser.add_argument("--summary-json", type=Path, default=None, help="Optional summary JSON path.")
    parser.add_argument("--sp3", type=Path, default=None, help="Optional precise SP3 file.")
    parser.add_argument("--clk", type=Path, default=None, help="Optional precise CLK file.")
    parser.add_argument("--kml", type=Path, default=None, help="Optional KML output path.")
    parser.add_argument("--max-epochs", type=int, default=0, help="Stop after N epochs.")
    parser.add_argument(
        "--convergence-min-epochs",
        type=int,
        default=None,
        help="Minimum epochs before PPP convergence/AR checks.",
    )
    parser.add_argument(
        "--ssr-step-seconds",
        type=float,
        default=1.0,
        help="Sampling step for RTCM SSR conversion.",
    )
    parser.add_argument("--static", dest="kinematic", action="store_false", help="Use static PPP.")
    parser.add_argument("--kinematic", dest="kinematic", action="store_true", help="Use kinematic PPP.")
    parser.set_defaults(kinematic=False)
    parser.add_argument("--enable-ar", action="store_true", help="Enable PPP ambiguity resolution.")
    parser.add_argument(
        "--ar-ratio-threshold",
        type=float,
        default=3.0,
        help="AR ratio threshold when ambiguity fixing is enabled.",
    )
    parser.add_argument(
        "--no-estimate-troposphere",
        dest="estimate_troposphere",
        action="store_false",
        help="Disable zenith troposphere estimation.",
    )
    parser.add_argument(
        "--estimate-troposphere",
        dest="estimate_troposphere",
        action="store_true",
        help="Enable zenith troposphere estimation.",
    )
    parser.set_defaults(estimate_troposphere=True)
    parser.add_argument(
        "--require-valid-epochs-min",
        type=int,
        default=None,
        help="Fail if valid epochs are below this count.",
    )
    parser.add_argument(
        "--require-ppp-solution-rate-min",
        type=float,
        default=None,
        help="Fail if PPP solution rate is below this percentage.",
    )
    parser.add_argument(
        "--require-ppp-fixed-epochs-min",
        type=int,
        default=None,
        help="Fail if PPP fixed epochs are below this count.",
    )
    parser.add_argument(
        "--require-atmos-messages-min",
        type=int,
        default=None,
        help="Fail if decoded atmospheric messages are below this count.",
    )
    parser.add_argument(
        "--require-ppp-atmos-trop-corrections-min",
        type=int,
        default=None,
        help="Fail if PPP-applied atmospheric troposphere corrections are below this count.",
    )
    parser.add_argument(
        "--require-ppp-atmos-ionosphere-corrections-min",
        type=int,
        default=None,
        help="Fail if PPP-applied atmospheric ionosphere corrections are below this count.",
    )
    return parser.parse_args()


def ensure_exists(path: Path | None, description: str) -> None:
    if path is None:
        return
    if not path.exists():
        raise SystemExit(f"Missing {description}: {path}")


def classify_transport(source: str) -> str:
    if "://" not in source:
        return "file"
    return source.split("://", 1)[0].lower()


def classify_encoding(args: argparse.Namespace) -> str:
    if args.qzss_l6:
        return "qzss_l6"
    return "compact" if args.compact_ssr else "rtcm"


def selected_correction_source(args: argparse.Namespace) -> str:
    if args.qzss_l6:
        return args.qzss_l6
    return args.compact_ssr if args.compact_ssr else args.ssr_rtcm


def parse_serial_path(path: str) -> tuple[str, int]:
    raw = path[len("serial://") :] if path.startswith("serial://") else path
    if "?baud=" in raw:
        device, baud_text = raw.split("?baud=", 1)
        return device, int(baud_text)
    return raw, DEFAULT_SERIAL_BAUD


def configure_serial(fd: int, baud: int) -> None:
    if os.name == "nt":
        raise RuntimeError("serial compact correction input is not supported on this platform")
    baud_map = {
        9600: termios.B9600,
        19200: termios.B19200,
        38400: termios.B38400,
        57600: termios.B57600,
        115200: termios.B115200,
        230400: termios.B230400,
    }
    if baud not in baud_map:
        raise ValueError(f"unsupported serial baud rate: {baud}")
    attrs = termios.tcgetattr(fd)
    attrs[0] = 0
    attrs[1] = 0
    attrs[2] = termios.CLOCAL | termios.CREAD | termios.CS8
    attrs[3] = 0
    attrs[4] = baud_map[baud]
    attrs[5] = baud_map[baud]
    attrs[6][termios.VMIN] = 1
    attrs[6][termios.VTIME] = 0
    termios.tcsetattr(fd, termios.TCSANOW, attrs)


def read_compact_source(source: str) -> str:
    if "://" not in source:
        return Path(source).read_text(encoding="ascii")

    scheme, remainder = source.split("://", 1)
    scheme = scheme.lower()
    if scheme == "tcp":
        host, port_text = remainder.rsplit(":", 1)
        with socket.create_connection((host, int(port_text)), timeout=5.0) as conn:
            chunks: list[bytes] = []
            while True:
                chunk = conn.recv(4096)
                if not chunk:
                    break
                chunks.append(chunk)
        return b"".join(chunks).decode("ascii")

    if scheme == "serial":
        device, baud = parse_serial_path(source)
        fd = os.open(device, os.O_RDONLY | os.O_NOCTTY)
        try:
            configure_serial(fd, baud)
            chunks: list[bytes] = []
            while True:
                chunk = os.read(fd, 4096)
                if not chunk:
                    break
                chunks.append(chunk)
        finally:
            os.close(fd)
        return b"".join(chunks).decode("ascii")

    raise SystemExit(f"Unsupported compact correction transport: {scheme}")


def normalize_system_token(token: str) -> str:
    normalized = token.strip().upper()
    aliases = {
        "GPS": "G",
        "GLONASS": "R",
        "GALILEO": "E",
        "BEIDOU": "C",
        "BDS": "C",
        "QZSS": "J",
        "SBAS": "S",
        "NAVIC": "I",
    }
    if normalized in aliases:
        return aliases[normalized]
    if len(normalized) == 1 and normalized in {"G", "R", "E", "C", "J", "S", "I"}:
        return normalized
    raise SystemExit(f"Unsupported compact correction system token: {token}")


def expand_compact_ssr_text(text: str, output_path: Path) -> dict[str, object]:
    rows_written = 0
    systems: set[str] = set()
    output_path.parent.mkdir(parents=True, exist_ok=True)
    with output_path.open("w", encoding="ascii") as handle:
        handle.write("# week,tow,sat,dx,dy,dz,dclock_m\n")
        for raw_line in text.splitlines():
            line = raw_line.strip()
            if not line or line.startswith("#"):
                continue
            columns = [column.strip() for column in line.split(",")]
            if len(columns) < 8:
                raise SystemExit(
                    "Compact correction rows must have "
                    f"{COMPACT_FILE_COLUMNS}: {line}"
                )
            week = int(columns[0])
            tow = float(columns[1])
            system = normalize_system_token(columns[2])
            prn = int(columns[3])
            dx = float(columns[4])
            dy = float(columns[5])
            dz = float(columns[6])
            dclock_m = float(columns[7])
            high_rate_clock_m = 0.0
            ura_sigma_token = None
            iode_tokens: list[str] = []
            code_bias_tokens: list[str] = []
            phase_bias_tokens: list[str] = []
            bias_network_tokens: list[str] = []
            atmos_tokens: list[str] = []
            extras = columns[8:]
            if extras and "=" not in extras[0] and not extras[0].startswith("cbias:"):
                high_rate_clock_m = float(extras[0])
                extras = extras[1:]
            for token in extras:
                if token.startswith("ura_sigma_m="):
                    ura_sigma_token = token
                    continue
                if token.startswith("iode="):
                    iode_tokens.append(token)
                    continue
                if token.startswith("cbias:") and "=" in token:
                    code_bias_tokens.append(token)
                    continue
                if token.startswith("pbias:") and "=" in token:
                    phase_bias_tokens.append(token)
                    continue
                if token.startswith("bias_network_id="):
                    bias_network_tokens.append(token)
                    continue
                if token.startswith("atmos_") and "=" in token:
                    atmos_tokens.append(token)
                    continue
                raise SystemExit(
                    "Compact correction rows must have "
                    f"{COMPACT_FILE_COLUMNS}: {line}"
                )
            sat_token = f"{system}{prn:02d}"
            systems.add(system)
            merged_clock = dclock_m + high_rate_clock_m
            output_tokens = [
                str(week),
                f"{tow:.3f}",
                sat_token,
                f"{dx:.6f}",
                f"{dy:.6f}",
                f"{dz:.6f}",
                f"{merged_clock:.6f}",
            ]
            if ura_sigma_token is not None:
                output_tokens.append(ura_sigma_token)
            output_tokens.extend(iode_tokens)
            output_tokens.extend(code_bias_tokens)
            output_tokens.extend(phase_bias_tokens)
            output_tokens.extend(bias_network_tokens)
            output_tokens.extend(atmos_tokens)
            handle.write(",".join(output_tokens) + "\n")
            rows_written += 1
    if rows_written == 0:
        raise SystemExit("No compact correction rows found")
    return {
        "rows_written": rows_written,
        "systems": sorted(systems),
        "expanded_csv": str(output_path),
    }


def infer_gps_week_from_obs(obs_path: Path) -> int:
    gps_epoch = dt.datetime(1980, 1, 6, tzinfo=dt.timezone.utc)
    with obs_path.open(encoding="ascii", errors="ignore") as handle:
        for line in handle:
            if not line.startswith(">"):
                continue
            parts = line[1:].split()
            if len(parts) < 6:
                continue
            year = int(parts[0])
            month = int(parts[1])
            day = int(parts[2])
            hour = int(parts[3])
            minute = int(parts[4])
            second = float(parts[5])
            whole_seconds = int(second)
            microseconds = int(round((second - whole_seconds) * 1_000_000))
            epoch = dt.datetime(
                year,
                month,
                day,
                hour,
                minute,
                whole_seconds,
                microseconds,
                tzinfo=dt.timezone.utc,
            )
            delta = epoch - gps_epoch
            return delta.days // 7
    raise SystemExit(f"Could not infer GPS week from observation file: {obs_path}")


def expand_qzss_l6_source(
    source: str,
    gps_week: int,
    output_path: Path,
    *,
    compact_flush_policy: str = qzss_l6_info.COMPACT_SSR_FLUSH_POLICY_LAG_TOLERANT,
    compact_atmos_merge_policy: str = qzss_l6_info.COMPACT_ATMOS_MERGE_POLICY_STEC_COEFF_CARRY,
    compact_atmos_subtype_merge_policy: str = qzss_l6_info.COMPACT_ATMOS_SUBTYPE_MERGE_POLICY_UNION,
    compact_phase_bias_merge_policy: str = qzss_l6_info.COMPACT_PHASE_BIAS_MERGE_POLICY_LATEST_UNION,
    compact_phase_bias_source_policy: str = qzss_l6_info.COMPACT_PHASE_BIAS_SOURCE_POLICY_ARRIVAL_ORDER,
    compact_code_bias_composition_policy: str = qzss_l6_info.COMPACT_CODE_BIAS_COMPOSITION_POLICY_DIRECT,
    compact_code_bias_bank_policy: str = qzss_l6_info.COMPACT_CODE_BIAS_BANK_POLICY_PENDING_EPOCH,
    compact_phase_bias_composition_policy: str = qzss_l6_info.COMPACT_PHASE_BIAS_COMPOSITION_POLICY_DIRECT,
    compact_phase_bias_bank_policy: str = qzss_l6_info.COMPACT_PHASE_BIAS_BANK_POLICY_PENDING_EPOCH,
    compact_bias_row_materialization: str = qzss_l6_info.COMPACT_BIAS_ROW_MATERIALIZATION_POLICY_OVERLAP_ONLY,
    compact_row_construction_policy: str = qzss_l6_info.COMPACT_ROW_CONSTRUCTION_POLICY_INDEPENDENT,
) -> dict[str, object]:
    frames, subframes, _stats = qzss_l6_info.decode_source(source)
    messages, corrections, _service_info_packets = qzss_l6_info.decode_cssr_messages(
        subframes,
        gps_week=gps_week,
        flush_policy=compact_flush_policy,
        atmos_merge_policy=compact_atmos_merge_policy,
        atmos_subtype_merge_policy=compact_atmos_subtype_merge_policy,
        phase_bias_merge_policy=compact_phase_bias_merge_policy,
        phase_bias_source_policy=compact_phase_bias_source_policy,
        code_bias_composition_policy=compact_code_bias_composition_policy,
        code_bias_bank_policy=compact_code_bias_bank_policy,
        phase_bias_composition_policy=compact_phase_bias_composition_policy,
        phase_bias_bank_policy=compact_phase_bias_bank_policy,
        bias_row_materialization_policy=compact_bias_row_materialization,
        row_construction_policy=compact_row_construction_policy,
    )
    compact_source = output_path.parent / "qzss_l6_compact.csv"
    qzss_l6_info.write_compact_corrections(compact_source, corrections)
    expand_compact_ssr_text(compact_source.read_text(encoding="ascii"), output_path)
    atmos_messages = sum(
        1
        for message in messages
        if message.subtype in (
            qzss_l6_info.CSSR_SUBTYPE_STEC,
            qzss_l6_info.CSSR_SUBTYPE_GRIDDED,
            qzss_l6_info.CSSR_SUBTYPE_ATMOS,
        )
    )
    atmos_rows = sum(
        1
        for correction in corrections
        if correction.atmos_network_id is not None
    )
    return {
        "frames": len(frames),
        "subframes": len(subframes),
        "messages": len(messages),
        "rows_written": len(corrections),
        "atmos_messages": atmos_messages,
        "atmos_rows": atmos_rows,
        "compact_csv": str(compact_source),
        "expanded_csv": str(output_path),
        "compact_atmos_merge_policy": compact_atmos_merge_policy,
        "compact_atmos_subtype_merge_policy": compact_atmos_subtype_merge_policy,
        "compact_phase_bias_merge_policy": compact_phase_bias_merge_policy,
        "compact_phase_bias_source_policy": compact_phase_bias_source_policy,
        "compact_code_bias_composition_policy": compact_code_bias_composition_policy,
        "compact_code_bias_bank_policy": compact_code_bias_bank_policy,
        "compact_phase_bias_composition_policy": compact_phase_bias_composition_policy,
        "compact_phase_bias_bank_policy": compact_phase_bias_bank_policy,
        "compact_bias_row_materialization": compact_bias_row_materialization,
        "compact_row_construction_policy": compact_row_construction_policy,
    }


def run_command(command: list[str]) -> subprocess.CompletedProcess[str]:
    print("+", " ".join(command))
    result = subprocess.run(command, check=True, text=True, capture_output=True)
    if result.stdout:
        print(result.stdout, end="")
    if result.stderr:
        print(result.stderr, end="", file=sys.stderr)
    return result


def read_pos_records(path: Path) -> list[dict[str, float | int]]:
    records: list[dict[str, float | int]] = []
    with path.open(encoding="ascii") as handle:
        for line in handle:
            if not line.strip() or line.startswith("%"):
                continue
            parts = line.split()
            if len(parts) < 10:
                continue
            records.append(
                {
                    "week": int(float(parts[0])),
                    "tow": float(parts[1]),
                    "status": int(parts[8]),
                    "satellites": int(parts[9]),
                }
            )
    return records


def rounded(value: float) -> float:
    return round(value, 6)


def _parse_ppp_summary_counts(ppp_stdout: str) -> dict[str, int | float]:
    parsed = {
        "ppp_atmospheric_trop_corrections": 0,
        "ppp_atmospheric_ionosphere_corrections": 0,
        "ppp_atmospheric_trop_meters": 0.0,
        "ppp_atmospheric_ionosphere_meters": 0.0,
    }
    for line in ppp_stdout.splitlines():
        stripped = line.strip()
        if stripped.startswith("atmospheric trop corrections:"):
            parsed["ppp_atmospheric_trop_corrections"] = int(stripped.rsplit(":", 1)[1].strip())
        elif stripped.startswith("atmospheric trop meters:"):
            parsed["ppp_atmospheric_trop_meters"] = float(stripped.rsplit(":", 1)[1].strip())
        elif stripped.startswith("atmospheric ionosphere corrections:"):
            parsed["ppp_atmospheric_ionosphere_corrections"] = int(
                stripped.rsplit(":", 1)[1].strip()
            )
        elif stripped.startswith("atmospheric ionosphere meters:"):
            parsed["ppp_atmospheric_ionosphere_meters"] = float(
                stripped.rsplit(":", 1)[1].strip()
            )
    return parsed


def build_summary_payload(
    args: argparse.Namespace,
    compact_summary: dict[str, object] | None = None,
    ppp_summary_counts: dict[str, int | float] | None = None,
) -> dict[str, object]:
    records = read_pos_records(args.out)
    if not records:
        raise SystemExit(f"No solution epochs found in {args.out}")

    ppp_float_epochs = sum(1 for record in records if int(record["status"]) == 5)
    ppp_fixed_epochs = sum(1 for record in records if int(record["status"]) == 6)
    ppp_epochs = sum(1 for record in records if int(record["status"]) in PPP_STATUSES)
    mean_satellites = sum(int(record["satellites"]) for record in records) / len(records)
    payload = {
        "dataset": "CLAS/MADOCA PPP",
        "correction_profile": args.profile,
        "ssr_transport": classify_transport(selected_correction_source(args)),
        "correction_encoding": classify_encoding(args),
        "obs": str(args.obs),
        "nav": str(args.nav),
        "sp3": str(args.sp3) if args.sp3 is not None else None,
        "clk": str(args.clk) if args.clk is not None else None,
        "ssr_rtcm": args.ssr_rtcm,
        "compact_ssr": args.compact_ssr,
        "qzss_l6": args.qzss_l6,
        "solution_pos": str(args.out),
        "epochs": len(records),
        "ppp_float_epochs": ppp_float_epochs,
        "ppp_fixed_epochs": ppp_fixed_epochs,
        "fallback_epochs": len(records) - ppp_epochs,
        "ppp_solution_rate_pct": rounded(100.0 * ppp_epochs / len(records)),
        "mean_satellites": rounded(mean_satellites),
        "ambiguity_resolution_enabled": bool(args.enable_ar),
        "ar_ratio_threshold": rounded(args.ar_ratio_threshold),
        "mode": "kinematic" if args.kinematic else "static",
        "compact_atmos_merge_policy": args.compact_atmos_merge_policy,
        "compact_atmos_subtype_merge_policy": args.compact_atmos_subtype_merge_policy,
        "compact_phase_bias_merge_policy": args.compact_phase_bias_merge_policy,
        "compact_phase_bias_source_policy": args.compact_phase_bias_source_policy,
        "compact_code_bias_composition_policy": args.compact_code_bias_composition_policy,
        "compact_code_bias_bank_policy": args.compact_code_bias_bank_policy,
        "compact_phase_bias_composition_policy": args.compact_phase_bias_composition_policy,
        "compact_phase_bias_bank_policy": args.compact_phase_bias_bank_policy,
        "compact_bias_row_materialization": args.compact_bias_row_materialization,
        "compact_row_construction_policy": args.compact_row_construction_policy,
        "atmos_messages": 0,
        "atmos_rows": 0,
        "ppp_atmospheric_trop_corrections": 0,
        "ppp_atmospheric_ionosphere_corrections": 0,
        "ppp_atmospheric_trop_meters": 0.0,
        "ppp_atmospheric_ionosphere_meters": 0.0,
    }
    if compact_summary is not None:
        payload["atmos_messages"] = int(compact_summary.get("atmos_messages", 0))
        payload["atmos_rows"] = int(compact_summary.get("atmos_rows", 0))
    if ppp_summary_counts is not None:
        payload["ppp_atmospheric_trop_corrections"] = int(
            ppp_summary_counts.get("ppp_atmospheric_trop_corrections", 0)
        )
        payload["ppp_atmospheric_ionosphere_corrections"] = int(
            ppp_summary_counts.get("ppp_atmospheric_ionosphere_corrections", 0)
        )
        payload["ppp_atmospheric_trop_meters"] = rounded(
            float(ppp_summary_counts.get("ppp_atmospheric_trop_meters", 0.0))
        )
        payload["ppp_atmospheric_ionosphere_meters"] = rounded(
            float(ppp_summary_counts.get("ppp_atmospheric_ionosphere_meters", 0.0))
        )
    if args.summary_json is not None:
        args.summary_json.parent.mkdir(parents=True, exist_ok=True)
        args.summary_json.write_text(json.dumps(payload, indent=2, sort_keys=True) + "\n", encoding="utf-8")
    return payload


def enforce_requirements(payload: dict[str, object], args: argparse.Namespace) -> None:
    failures: list[str] = []
    if args.require_valid_epochs_min is not None and int(payload["epochs"]) < args.require_valid_epochs_min:
        failures.append(f"valid epochs {payload['epochs']} < {args.require_valid_epochs_min}")
    if (
        args.require_ppp_solution_rate_min is not None
        and float(payload["ppp_solution_rate_pct"]) < args.require_ppp_solution_rate_min
    ):
        failures.append(
            f"PPP solution rate {float(payload['ppp_solution_rate_pct']):.6f}% < "
            f"{args.require_ppp_solution_rate_min:.6f}%"
        )
    if (
        args.require_ppp_fixed_epochs_min is not None
        and int(payload["ppp_fixed_epochs"]) < args.require_ppp_fixed_epochs_min
    ):
        failures.append(
            f"PPP fixed epochs {int(payload['ppp_fixed_epochs'])} < "
            f"{args.require_ppp_fixed_epochs_min}"
        )
    if (
        args.require_atmos_messages_min is not None
        and int(payload["atmos_messages"]) < args.require_atmos_messages_min
    ):
        failures.append(
            f"atmospheric messages {int(payload['atmos_messages'])} < "
            f"{args.require_atmos_messages_min}"
        )
    if (
        args.require_ppp_atmos_trop_corrections_min is not None
        and int(payload["ppp_atmospheric_trop_corrections"])
        < args.require_ppp_atmos_trop_corrections_min
    ):
        failures.append(
            f"PPP atmospheric trop corrections {int(payload['ppp_atmospheric_trop_corrections'])} < "
            f"{args.require_ppp_atmos_trop_corrections_min}"
        )
    if (
        args.require_ppp_atmos_ionosphere_corrections_min is not None
        and int(payload["ppp_atmospheric_ionosphere_corrections"])
        < args.require_ppp_atmos_ionosphere_corrections_min
    ):
        failures.append(
            "PPP atmospheric ionosphere corrections "
            f"{int(payload['ppp_atmospheric_ionosphere_corrections'])} < "
            f"{args.require_ppp_atmos_ionosphere_corrections_min}"
        )
    if failures:
        message = "CLAS/MADOCA PPP checks failed:\n" + "\n".join(f"  - {failure}" for failure in failures)
        raise SystemExit(message)


def main() -> int:
    args = parse_args()
    gnss_command = resolve_gnss_command(ROOT_DIR)
    ensure_exists(args.obs, "observation file")
    ensure_exists(args.nav, "navigation file")
    ensure_exists(args.sp3, "SP3 file")
    ensure_exists(args.clk, "CLK file")
    selected_sources = [bool(args.ssr_rtcm), bool(args.compact_ssr), bool(args.qzss_l6)]
    if sum(1 for selected in selected_sources if selected) != 1:
        raise SystemExit("Specify exactly one of --ssr-rtcm, --compact-ssr, or --qzss-l6")

    args.out.parent.mkdir(parents=True, exist_ok=True)
    if args.summary_json is not None:
        args.summary_json.parent.mkdir(parents=True, exist_ok=True)

    with tempfile.TemporaryDirectory(prefix="gnss_clas_ppp_") as temp_dir:
        compact_summary: dict[str, object] | None = None
        command = [
            *gnss_command,
            "ppp",
            "--obs",
            str(args.obs),
            "--nav",
            str(args.nav),
            "--out",
            str(args.out),
        ]
        if args.qzss_l6 is not None:
            compact_csv = Path(temp_dir) / "qzss_l6_expanded.csv"
            gps_week = args.qzss_gps_week if args.qzss_gps_week is not None else infer_gps_week_from_obs(args.obs)
            compact_summary = expand_qzss_l6_source(
                args.qzss_l6,
                gps_week,
                compact_csv,
                compact_flush_policy=args.compact_flush_policy,
                compact_atmos_merge_policy=args.compact_atmos_merge_policy,
                compact_atmos_subtype_merge_policy=args.compact_atmos_subtype_merge_policy,
                compact_phase_bias_merge_policy=args.compact_phase_bias_merge_policy,
                compact_phase_bias_source_policy=args.compact_phase_bias_source_policy,
                compact_code_bias_composition_policy=args.compact_code_bias_composition_policy,
                compact_code_bias_bank_policy=args.compact_code_bias_bank_policy,
                compact_phase_bias_composition_policy=args.compact_phase_bias_composition_policy,
                compact_phase_bias_bank_policy=args.compact_phase_bias_bank_policy,
                compact_bias_row_materialization=args.compact_bias_row_materialization,
                compact_row_construction_policy=args.compact_row_construction_policy,
            )
            print(
                "decoded qzss l6 corrections:",
                f"frames={compact_summary['frames']}",
                f"subframes={compact_summary['subframes']}",
                f"messages={compact_summary['messages']}",
                f"rows={compact_summary['rows_written']}",
                f"atmos_messages={compact_summary['atmos_messages']}",
                f"atmos_rows={compact_summary['atmos_rows']}",
                f"csv={compact_summary['expanded_csv']}",
            )
            command.extend(["--ssr", str(compact_csv)])
        elif args.compact_ssr is not None:
            compact_csv = Path(temp_dir) / "compact_expanded.csv"
            compact_summary = expand_compact_ssr_text(read_compact_source(args.compact_ssr), compact_csv)
            print(
                "expanded compact corrections:",
                f"rows={compact_summary['rows_written']}",
                f"systems={','.join(compact_summary['systems'])}",
                f"csv={compact_summary['expanded_csv']}",
            )
            command.extend(["--ssr", str(compact_csv)])
        else:
            command.extend(["--ssr-rtcm", args.ssr_rtcm, "--ssr-step-seconds", str(args.ssr_step_seconds)])

        command.append("--kinematic" if args.kinematic else "--static")
        command.append("--estimate-troposphere" if args.estimate_troposphere else "--no-estimate-troposphere")
        # When atmospheric corrections are available from CLAS, disable IFLC and
        # enable per-satellite ionosphere estimation with STEC constraints.
        command.append("--no-ionosphere-free")
        command.append("--estimate-ionosphere")
        if args.sp3 is not None:
            command.extend(["--sp3", str(args.sp3)])
        if args.clk is not None:
            command.extend(["--clk", str(args.clk)])
        if args.kml is not None:
            command.extend(["--kml", str(args.kml)])
        if args.max_epochs > 0:
            command.extend(["--max-epochs", str(args.max_epochs)])
        if args.convergence_min_epochs is not None:
            command.extend(["--convergence-min-epochs", str(args.convergence_min_epochs)])
        if args.enable_ar:
            command.extend(["--enable-ar", "--ar-ratio-threshold", str(args.ar_ratio_threshold)])

        ppp_result = run_command(command)
    payload = build_summary_payload(args, compact_summary, _parse_ppp_summary_counts(ppp_result.stdout))
    enforce_requirements(payload, args)

    print("Finished CLAS/MADOCA PPP run.")
    print(f"  profile: {args.profile}")
    print(f"  transport: {payload['ssr_transport']}")
    print(f"  encoding: {payload['correction_encoding']}")
    print(f"  solution: {args.out}")
    if args.summary_json is not None:
        print(f"  summary: {args.summary_json}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
