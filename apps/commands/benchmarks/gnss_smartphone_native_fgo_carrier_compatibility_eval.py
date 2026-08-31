#!/usr/bin/env python3
"""Development-only carrier compatibility selector for native smartphone FGO.

The sealed all-device float-carrier experiment remains unchanged.  This lane
only repairs the already demonstrated adapter contract at its boundary: an
Android ADR row which is invalid, reset, cycle-slipped, clock-discontinuous,
or separated by more than 1500 ms is emitted with a RINEX loss-of-lock marker
and the carrier field is blanked.  The original adapter and its frozen hash are
never rewritten.

Each route runs the exact native-FGO v1 pseudorange + ordinary TDCP + motion
graph as ``baseline8``.  ``carrier_float50`` adds only no-base float carrier
factors and the sanitized observation file.  A route may select the candidate
only when all truth-free physical/solver checks pass; otherwise its selected
file is a byte-for-byte copy of baseline8.  Truth is intentionally unavailable
until ``train-score`` verifies the sealed truth-free manifest, and only the
development cohort is allowed there.
"""

from __future__ import annotations

import argparse
import csv
from collections import defaultdict
import hashlib
import json
import math
import os
from pathlib import Path
import resource
import shutil
import subprocess
import sys
import tempfile
import time
from typing import Any
import zipfile

_COMMANDS_DIR = Path(__file__).resolve().parents[1]
if str(_COMMANDS_DIR) not in sys.path:
    sys.path.insert(0, str(_COMMANDS_DIR))
from support.gnss_runtime import application_root  # noqa: E402

ROOT = application_root(__file__)
BENCHMARK_DIR = ROOT / "apps" / "commands" / "benchmarks"
if str(BENCHMARK_DIR) not in sys.path:
    sys.path.insert(0, str(BENCHMARK_DIR))

import gnss_smartphone_native_fgo_eval as native_eval  # noqa: E402
import gnss_smartphone_native_fgo_optimizer_stop_eval as optimizer_eval  # noqa: E402
import gnss_smartphone_trajectory_smoother_eval as smoother_eval  # noqa: E402


FREEZE = ROOT / "docs/use_cases/records/smartphone_r5_gsdc2023_native_fgo_carrier_compatibility_freeze_v1.json"
FREEZE_MANIFEST = ROOT / "docs/use_cases/records/smartphone_r5_gsdc2023_native_fgo_carrier_compatibility_freeze_v1_manifest.json"
ROLE_INVENTORY = ROOT / "docs/use_cases/records/smartphone_r5_gsdc2023_native_fgo_carrier_compatibility_role_inventory_v1.json"
ARCHIVE = ROOT / "data/gsdc2023/cache/dataset_2023.zip"
CENTRAL_INVENTORY = ROOT / "output/smartphone-r5/generalization-v6/archive_inventory.json"
PROFILE = ROOT / "configs/benchmarks/smartphone_r5_gsdc2023.json"
FGO_BINARY = ROOT / "build/apps/gnss_fgo"
SPP_BINARY = ROOT / "build/apps/gnss_spp"
DEFAULT_OUTPUT = ROOT / "output/smartphone-r5/native-fgo-carrier-compatibility-v1"

ARCHIVE_SHA256 = "bda30ab456e6fd6f83550c246e8dbd287306d5385f1f1069c99c16298e647408"
CENTRAL_INVENTORY_SHA256 = "f4e68109885eecfc14b2bd5e8fab87e18d73473c04bb7f53da31b7040a8e90a7"
CENTRAL_METADATA_SHA256 = "5f619be94a00c33c3067f13b1cd3351d96f2804f07fa04eda8fb027739fb0992"
PROFILE_SHA256 = "273dfcc4e4636940d5216cca793a55773d9a04f668d1bfa5fcdc0013f4768776"
FGO_BINARY_SHA256 = "9190455cb34a9e818b79ed2ed6b023af5711fbef545215d4588bfed87f5c03e4"
SPP_BINARY_SHA256 = "4d272940437c2ab2dcffef31b7541c8c01212e97d2d2320edcf7bd8f80ea3c12"
ROLE_INVENTORY_SHA256 = "dc41e76bf4fa956d589af4675a44fee1be1c8bf54fdfe52cf437b02e07866cd1"
# The companion manifest is the source of truth for the freeze hash.  Keeping
# that value out of this source file avoids a self-hash cycle when the script
# itself is one of the frozen source artifacts.

FGO_SOURCE_HASHES = {
    "apps/native/gnss_fgo.cpp": "2d9e53485c427d1e0d43b07dd611aaf3644e2cd51d5e175c2977038ca05e3e32",
    "src/algorithms/fgo.cpp": "4805a30741e3bcc53a2a29d832429924a3cc46c0fac5aa530446cedd224af3b2",
    "include/libgnss++/algorithms/fgo.hpp": "67845f6365eb1f621afed19c52b82ad48d00ac1623e99b3bf7b96cd944263307",
    "include/libgnss++/algorithms/fgo_config.hpp": "8a9390e6709f4c4a55ca4f9d4d43fd451465234d1aa55cd4dd662b06f0872d80",
    "apps/commands/benchmarks/gnss_smartphone_gnss_adapter.py": "4a26dbbef0d5eff4a4840c43b600d790d573accc9977a9993754386ad086466b",
    "configs/benchmarks/smartphone_r5_gsdc2023.json": PROFILE_SHA256,
}

# This list is deliberately a closed development-only cohort.  The identities
# and their role inventory are repeated in the freeze record, so an accidental
# validation/holdout/test path cannot be introduced by a command-line typo.
COHORT = (
    "2021-03-10-23-13-us-ca-mtv-h/pixel5",
    "2021-08-24-20-32-us-ca-mtv-h/pixel5",
    "2022-04-01-18-22-us-ca-lax-t/pixel5",
    "2021-01-04-21-50-us-ca-e1highway280driveroutea/pixel5",
    "2021-07-14-20-50-us-ca-mtv-e/sm-g988b",
    "2021-03-16-18-59-us-ca-mtv-a/pixel5",
    "2021-07-27-19-49-us-ca-mtv-b/pixel4",
    "2022-02-24-18-29-us-ca-lax-o/pixel5",
)

SOURCE_ROOTS = (
    ROOT / "output/smartphone-r5/native-fgo-optimizer-stop-v1/train",
    ROOT / "output/smartphone-r5/native-fgo-convergence-selector-v1/train",
    ROOT / "output/smartphone-r5/native-fgo-v2-processed/routes",
)
LEAP_SECONDS = 18
MAX_RUNTIME_SECONDS = 900
MAX_ADDRESS_SPACE_BYTES = 8 * 1024 * 1024 * 1024
MAX_OUTPUT_BYTES = 1024 * 1024 * 1024
SPEED_BOUND_MPS = 70.0
RAW_ARC_GAP_MS = 1500
MAX_TDCP_GAP_S = 2.0
MIN_ARC_DURATION_S = 5.0
CARRIER_SIGMA_M = 0.01
CARRIER_HUBER_THRESHOLD_SIGMA = 4.0
MIN_ELEVATION_DEG = 10.0
POSTFIT_RMS_BOUND_M = (
    CARRIER_SIGMA_M * CARRIER_HUBER_THRESHOLD_SIGMA
    / math.sin(math.radians(MIN_ELEVATION_DEG))
)
PREFIT_RATE_RMS_BOUND_MPS = 2.0 * POSTFIT_RMS_BOUND_M
DIAGNOSTIC_KEYS = tuple(native_eval.DIAGNOSTIC_KEYS)


class CarrierCompatibilityError(ValueError):
    """Raised when the compatibility-lane contract is violated."""


def sha256(path: Path) -> str:
    if not path.is_file():
        raise CarrierCompatibilityError(f"missing file: {path}")
    digest = hashlib.sha256()
    with path.open("rb") as handle:
        for chunk in iter(lambda: handle.read(1024 * 1024), b""):
            digest.update(chunk)
    return digest.hexdigest()


def relative(path: Path) -> str:
    try:
        return str(path.resolve().relative_to(ROOT.resolve()))
    except ValueError:
        return str(path)


def load_json(path: Path, label: str) -> dict[str, Any]:
    try:
        payload = json.loads(path.read_text(encoding="utf-8"))
    except (OSError, json.JSONDecodeError) as exc:
        raise CarrierCompatibilityError(f"invalid {label}: {path}") from exc
    if not isinstance(payload, dict):
        raise CarrierCompatibilityError(f"{label} must be a JSON object")
    return payload


def atomic_bytes(path: Path, content: bytes) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    descriptor, temporary_name = tempfile.mkstemp(
        prefix=f".{path.name}.", suffix=".tmp", dir=str(path.parent)
    )
    temporary = Path(temporary_name)
    try:
        with os.fdopen(descriptor, "wb") as handle:
            handle.write(content)
            handle.flush()
            os.fsync(handle.fileno())
        os.replace(temporary, path)
    finally:
        temporary.unlink(missing_ok=True)


def atomic_json(path: Path, payload: dict[str, Any]) -> None:
    atomic_bytes(path, (json.dumps(payload, indent=2, sort_keys=True) + "\n").encode("utf-8"))


def safe_id(dataset_id: str) -> str:
    if dataset_id.count("/") != 1 or any(ch.isspace() for ch in dataset_id):
        raise CarrierCompatibilityError(f"invalid dataset id: {dataset_id}")
    return dataset_id.replace("/", "__")


def _finite(value: Any) -> bool:
    try:
        return math.isfinite(float(value))
    except (TypeError, ValueError):
        return False


def _float_or_none(value: Any) -> float | None:
    if value is None or str(value).strip() == "":
        return None
    try:
        number = float(value)
    except (TypeError, ValueError):
        return None
    return number if math.isfinite(number) else None


def _int_field(row: dict[str, str], name: str) -> int:
    try:
        return int(row[name].strip())
    except (KeyError, TypeError, ValueError) as exc:
        raise CarrierCompatibilityError(f"invalid raw integer field: {name}") from exc


def _raw_records(path: Path) -> tuple[list[dict[str, Any]], dict[int, dict[tuple[str, int], dict[str, Any]]]]:
    required = {
        "utcTimeMillis",
        "Svid",
        "AccumulatedDeltaRangeState",
        "AccumulatedDeltaRangeMeters",
        "PseudorangeRateMetersPerSecond",
        "HardwareClockDiscontinuityCount",
        "SignalType",
    }
    by_time: dict[int, dict[tuple[str, int], dict[str, Any]]] = {}
    with path.open(newline="", encoding="utf-8") as handle:
        reader = csv.DictReader(handle)
        missing = sorted(required - set(reader.fieldnames or ()))
        if missing:
            raise CarrierCompatibilityError("raw GNSS schema missing: " + ", ".join(missing))
        for row in reader:
            signal = row.get("SignalType", "").strip()
            if signal not in {"GPS_L1_CA", "GAL_E1_C_P"}:
                continue
            timestamp = _int_field(row, "utcTimeMillis")
            svid = _int_field(row, "Svid")
            if svid <= 0 or svid > 99:
                raise CarrierCompatibilityError("raw Svid is outside RINEX two-digit range")
            constellation = "G" if signal == "GPS_L1_CA" else "E"
            key = (constellation, svid)
            epoch = by_time.setdefault(timestamp, {})
            if key in epoch:
                raise CarrierCompatibilityError(
                    f"duplicate supported raw observation at {timestamp}/{key}"
                )
            state = _int_field(row, "AccumulatedDeltaRangeState")
            adr = _float_or_none(row.get("AccumulatedDeltaRangeMeters"))
            rate = _float_or_none(row.get("PseudorangeRateMetersPerSecond"))
            clock = _int_field(row, "HardwareClockDiscontinuityCount")
            if clock < 0:
                raise CarrierCompatibilityError("negative hardware clock counter")
            epoch[key] = {
                "timestamp_ms": timestamp,
                "constellation": constellation,
                "svid": svid,
                "signal": signal,
                "state": state,
                "adr_m": adr,
                "rate_mps": rate,
                "clock": clock,
                "valid": bool(state & 1) and adr is not None and abs(adr) < 1e9,
            }
    if not by_time:
        raise CarrierCompatibilityError("raw GNSS has no supported carrier rows")
    epochs = sorted(by_time)
    records = [row for timestamp in epochs for row in by_time[timestamp].values()]
    return records, {index: by_time[timestamp] for index, timestamp in enumerate(epochs)}


def _rinex_epochs(path: Path) -> tuple[list[str], list[list[tuple[int, tuple[str, int]]]]]:
    try:
        lines = path.read_text(encoding="ascii").splitlines(keepends=True)
    except (OSError, UnicodeDecodeError) as exc:
        raise CarrierCompatibilityError(f"RINEX is not ASCII/readable: {path}") from exc
    header_end = None
    for index, line in enumerate(lines):
        if line.rstrip("\r\n").endswith("END OF HEADER"):
            header_end = index + 1
            break
    if header_end is None:
        raise CarrierCompatibilityError("RINEX header lacks END OF HEADER")
    epoch_lines: list[str] = []
    observations: list[list[tuple[int, tuple[str, int]]]] = []
    index = header_end
    while index < len(lines):
        line = lines[index]
        if not line.startswith(">"):
            index += 1
            continue
        tokens = line[1:].split()
        if len(tokens) < 8:
            raise CarrierCompatibilityError("RINEX epoch line is incomplete")
        try:
            count = int(tokens[7])
        except ValueError as exc:
            raise CarrierCompatibilityError("RINEX epoch satellite count is invalid") from exc
        if count < 0 or count > 100:
            raise CarrierCompatibilityError("RINEX epoch satellite count is unsafe")
        epoch_lines.append(line)
        epoch_obs: list[tuple[int, tuple[str, int]]] = []
        for offset in range(1, count + 1):
            line_index = index + offset
            if line_index >= len(lines):
                raise CarrierCompatibilityError("RINEX epoch is truncated")
            satellite_line = lines[line_index]
            if len(satellite_line) < 35 or satellite_line[0] not in {"G", "E"}:
                raise CarrierCompatibilityError("RINEX satellite line is incomplete/unsupported")
            try:
                svid = int(satellite_line[1:3])
            except ValueError as exc:
                raise CarrierCompatibilityError("RINEX satellite identifier is invalid") from exc
            epoch_obs.append((line_index, (satellite_line[0], svid)))
        observations.append(epoch_obs)
        index += count + 1
    if not observations:
        raise CarrierCompatibilityError("RINEX has no epochs")
    return lines, observations


def _prefit_rate_stats(by_epoch: dict[int, dict[tuple[str, int], dict[str, Any]]]) -> dict[str, Any]:
    previous: dict[tuple[str, int], dict[str, Any]] = {}
    residuals: list[float] = []
    for epoch_index in sorted(by_epoch):
        for key, current in by_epoch[epoch_index].items():
            prior = previous.get(key)
            if prior and current["valid"] and prior["valid"]:
                delta_ms = current["timestamp_ms"] - prior["timestamp_ms"]
                if 0 < delta_ms <= RAW_ARC_GAP_MS:
                    dt = delta_ms / 1000.0
                    if (
                        current["rate_mps"] is not None
                        and prior["rate_mps"] is not None
                        and not (current["state"] & 6)
                        and not (prior["state"] & 6)
                    ):
                        residuals.append(
                            (current["adr_m"] - prior["adr_m"]) / dt
                            - 0.5 * (current["rate_mps"] + prior["rate_mps"])
                        )
            previous[key] = current
    absolute = sorted(abs(value) for value in residuals)
    rms = math.sqrt(sum(value * value for value in residuals) / len(residuals)) if residuals else math.inf
    p95 = absolute[min(len(absolute) - 1, int(math.ceil(0.95 * len(absolute)) - 1))] if absolute else math.inf
    return {
        "pair_count": len(residuals),
        "rms_mps": rms,
        "absolute_p95_mps": p95,
        "absolute_max_mps": max(absolute) if absolute else math.inf,
        "bound_mps": PREFIT_RATE_RMS_BOUND_MPS,
    }


def _percentile(values: list[float], fraction: float) -> float | None:
    if not values:
        return None
    ordered = sorted(values)
    index = min(len(ordered) - 1, max(0, int(math.ceil(fraction * len(ordered)) - 1)))
    return ordered[index]


def _carrier_stats(
    by_epoch: dict[int, dict[tuple[str, int], dict[str, Any]]],
) -> tuple[dict[tuple[int, tuple[str, int]], str], dict[str, Any]]:
    marks: dict[tuple[int, tuple[str, int]], str] = {}
    awaiting_next: set[tuple[str, int]] = set()
    previous: dict[tuple[str, int], dict[str, Any]] = {}
    reasons: defaultdict[str, int] = defaultdict(int)
    valid_rows = invalid_rows = reset_rows = slip_rows = 0
    max_arc_s = 0.0
    arc_start: dict[tuple[str, int], int] = {}
    next_valid_marks = 0
    for epoch_index in sorted(by_epoch):
        for key, current in by_epoch[epoch_index].items():
            if current["valid"]:
                valid_rows += 1
            else:
                invalid_rows += 1
            if current["state"] & 2:
                reset_rows += 1
            if current["state"] & 4:
                slip_rows += 1
            prior = previous.get(key)
            boundary_reason: str | None = None
            if key in awaiting_next and current["valid"]:
                marks[(epoch_index, key)] = "next-valid-after-boundary"
                awaiting_next.remove(key)
                next_valid_marks += 1
            if not current["valid"]:
                boundary_reason = "adr-invalid"
            elif current["state"] & 2:
                boundary_reason = "adr-reset"
            elif current["state"] & 4:
                boundary_reason = "cycle-slip"
            if prior is not None:
                delta_ms = current["timestamp_ms"] - prior["timestamp_ms"]
                if delta_ms <= 0:
                    boundary_reason = "nonpositive-time"
                elif delta_ms > RAW_ARC_GAP_MS:
                    boundary_reason = "gap-over-1500ms"
                elif current["clock"] != prior["clock"]:
                    boundary_reason = "hardware-clock-discontinuity"
                elif not prior["valid"]:
                    boundary_reason = "after-invalid-adr"
            if boundary_reason is not None:
                marks[(epoch_index, key)] = boundary_reason
                reasons[boundary_reason] += 1
                awaiting_next.add(key)
                arc_start.pop(key, None)
            if current["valid"]:
                if key not in arc_start:
                    arc_start[key] = current["timestamp_ms"]
                max_arc_s = max(max_arc_s, (current["timestamp_ms"] - arc_start[key]) / 1000.0)
            else:
                arc_start.pop(key, None)
            previous[key] = current
    prefit = _prefit_rate_stats(by_epoch)
    stats = {
        "supported_rows": valid_rows + invalid_rows,
        "valid_rows": valid_rows,
        "invalid_rows": invalid_rows,
        "reset_rows": reset_rows,
        "cycle_slip_rows": slip_rows,
        "boundary_rows": len(marks),
        "next_valid_boundary_rows": next_valid_marks,
        "boundary_reason_counts": dict(sorted(reasons.items())),
        "max_arc_duration_s": max_arc_s,
        "supported_key_count": len({key for epoch in by_epoch.values() for key in epoch}),
        "prefit_rate": prefit,
        "raw_gap_contract_ms": RAW_ARC_GAP_MS,
    }
    return marks, stats


def sanitize_rinex_carrier_boundaries(
    raw_csv: Path, source_rinex: Path, target_rinex: Path
) -> dict[str, Any]:
    """Sanitize ADR arc boundaries without changing code/Doppler/geometry."""

    _, by_epoch = _raw_records(raw_csv)
    lines, rinex_epochs = _rinex_epochs(source_rinex)
    if len(by_epoch) != len(rinex_epochs):
        raise CarrierCompatibilityError(
            f"raw/RINEX epoch count mismatch: {len(by_epoch)} != {len(rinex_epochs)}"
        )
    marks, stats = _carrier_stats(by_epoch)
    changed = 0
    for epoch_index, epoch_obs in enumerate(rinex_epochs):
        raw_keys = set(by_epoch[epoch_index])
        rinex_keys = {key for _, key in epoch_obs}
        if raw_keys != rinex_keys:
            raise CarrierCompatibilityError(
                f"raw/RINEX supported key mismatch at epoch {epoch_index}: "
                f"raw={sorted(raw_keys)} rinex={sorted(rinex_keys)}"
            )
        for line_index, key in epoch_obs:
            if (epoch_index, key) not in marks:
                continue
            original = lines[line_index]
            if len(original.rstrip("\r\n")) < 35:
                raise CarrierCompatibilityError("RINEX phase field is too short")
            newline = "\n" if original.endswith("\n") else ""
            body = original[:-1] if newline else original
            if body.endswith("\r"):
                body = body[:-1]
                newline = "\r\n"
            chars = list(body)
            chars[19:35] = [" "] * 16
            chars[33] = "1"
            replacement = "".join(chars) + newline
            if replacement != original:
                lines[line_index] = replacement
                changed += 1
    content = "".join(lines).encode("ascii")
    atomic_bytes(target_rinex, content)
    stats.update(
        {
            "source_rinex_sha256": sha256(source_rinex),
            "sanitized_rinex_sha256": sha256(target_rinex),
            "sanitized_rows": changed,
            "rinex_epoch_count": len(rinex_epochs),
            "rinex_observation_rows": sum(len(epoch) for epoch in rinex_epochs),
        }
    )
    return stats


def _common_fgo_command(
    obs: Path,
    nav: Path,
    seed: Path,
    output: Path,
    *,
    max_iterations: int,
    relative_threshold: float,
) -> list[str]:
    return [
        str(FGO_BINARY),
        "--obs", str(obs),
        "--nav", str(nav),
        "--seed-pos", str(seed),
        "--out", str(output / "fgo.pos"),
        "--summary-json", str(output / "fgo_summary.json"),
        "--epoch-debug-csv", str(output / "fgo_epoch_debug.csv"),
        "--factor-debug-csv", str(output / "fgo_factor_debug.csv"),
        "--cost-trace-csv", str(output / "fgo_cost_trace.csv"),
        "--preset", "default",
        "--backend", "eigen",
        "--skip-epochs", "0",
        "--max-epochs", "0",
        "--max-iterations", str(max_iterations),
        "--relative-cost-threshold", format(relative_threshold, ".17g"),
        "--absolute-cost-threshold", "0",
        "--pseudorange-sigma", "3",
        "--pseudorange-elevation-power", "1",
        "--motion-sigma", "50",
        "--clock-motion-sigma", "300",
        "--velocity-prior-sigma", "100",
        "--velocity-motion-sigma", "0.01",
        "--position-prior-sigma", "0",
        "--clock-prior-sigma", "0",
        "--tdcp-sigma", "0.03",
        "--carrier-phase-sigma", "0.01",
        "--pseudorange-huber-threshold", "4",
        "--carrier-phase-huber-threshold", "4",
        "--tdcp-huber-threshold", "4",
        "--max-tdcp-gap", "2",
        "--seed-match-tolerance", "0.01",
        "--seed-interpolation-max-gap", "0",
        "--tdcp-slip-threshold", "10",
        "--min-elevation", "10",
        "--min-snr", "0",
        "--min-satellites-per-epoch", "4",
        "--no-dd-factors",
        "--ionosphere-model",
        "--troposphere-model",
        "--quiet",
    ]


def baseline_command(obs: Path, nav: Path, seed: Path, output: Path) -> list[str]:
    """Exact v1 graph with the frozen eight-iteration stopping policy."""

    return _common_fgo_command(obs, nav, seed, output, max_iterations=8, relative_threshold=0.0)


def carrier_float_command(obs: Path, nav: Path, seed: Path, output: Path) -> list[str]:
    """Exact v1 graph plus no-base float carrier factors."""

    command = _common_fgo_command(obs, nav, seed, output, max_iterations=50, relative_threshold=1e-6)
    command.extend(("--carrier-phase-factors", "--reject-rover-carrier-lli"))
    return command


def command_contract(command: list[str]) -> dict[str, Any]:
    values = set(command)
    forbidden = {
        "--base": "base observations",
        "--fix-ambiguities": "integer ambiguity fixing",
        "--fix-all-ambiguities": "integer ambiguity fixing",
        "--sd-doppler-factors": "single-difference Doppler",
        "--sd-tdcp-factors": "single-difference TDCP",
        "--ambiguity-between-factors": "ambiguity-between factors",
    }
    violations = [reason for flag, reason in forbidden.items() if flag in values]
    return {
        "carrier_phase_enabled": "--carrier-phase-factors" in values,
        "reject_rover_carrier_lli": "--reject-rover-carrier-lli" in values,
        "no_double_difference": "--no-dd-factors" in values,
        "backend": command[command.index("--backend") + 1] if "--backend" in values else None,
        "max_iterations": int(command[command.index("--max-iterations") + 1]) if "--max-iterations" in values else None,
        "relative_cost_threshold": float(command[command.index("--relative-cost-threshold") + 1]) if "--relative-cost-threshold" in values else None,
        "violations": sorted(set(violations)),
    }


def validate_command(command: list[str], *, candidate: bool) -> None:
    contract = command_contract(command)
    if contract["violations"]:
        raise CarrierCompatibilityError("forbidden FGO flags: " + ", ".join(contract["violations"]))
    if contract["backend"] != "eigen" or not contract["no_double_difference"]:
        raise CarrierCompatibilityError("compatibility lane requires Eigen/no-DD")
    if candidate:
        if not contract["carrier_phase_enabled"] or not contract["reject_rover_carrier_lli"]:
            raise CarrierCompatibilityError("carrier lane lacks carrier factors or LLI rejection")
        if contract["max_iterations"] != 50 or contract["relative_cost_threshold"] != 1e-6:
            raise CarrierCompatibilityError("carrier stopping policy changed")
    else:
        if contract["carrier_phase_enabled"] or contract["max_iterations"] != 8 or contract["relative_cost_threshold"] != 0.0:
            raise CarrierCompatibilityError("baseline v1 stopping/measurement policy changed")


def _limits() -> None:
    resource.setrlimit(resource.RLIMIT_AS, (MAX_ADDRESS_SPACE_BYTES, MAX_ADDRESS_SPACE_BYTES))
    resource.setrlimit(resource.RLIMIT_CPU, (MAX_RUNTIME_SECONDS, MAX_RUNTIME_SECONDS + 1))


def run_child(command: list[str], stage_dir: Path, label: str) -> dict[str, Any]:
    stage_dir.mkdir(parents=True, exist_ok=True)
    started = time.perf_counter()
    before_rss = resource.getrusage(resource.RUSAGE_CHILDREN).ru_maxrss
    environment = os.environ.copy()
    loader = "/home/sasaki/.local/lib:/opt/ros/jazzy/lib"
    environment["LD_LIBRARY_PATH"] = loader + (":" + environment["LD_LIBRARY_PATH"] if environment.get("LD_LIBRARY_PATH") else "")
    try:
        completed = subprocess.run(
            command,
            cwd=stage_dir,
            env=environment,
            capture_output=True,
            text=True,
            timeout=MAX_RUNTIME_SECONDS,
            preexec_fn=_limits,
            check=False,
        )
        timed_out = False
    except subprocess.TimeoutExpired as exc:
        completed = None
        timed_out = True
        stdout = exc.stdout or ""
        stderr = exc.stderr or ""
    wall = time.perf_counter() - started
    if completed is not None:
        stdout = completed.stdout
        stderr = completed.stderr
        return_code = completed.returncode
    else:
        return_code = -1
    atomic_bytes(stage_dir / f"{label}.stdout.log", stdout.encode("utf-8", errors="replace"))
    atomic_bytes(stage_dir / f"{label}.stderr.log", stderr.encode("utf-8", errors="replace"))
    after_rss = resource.getrusage(resource.RUSAGE_CHILDREN).ru_maxrss
    return {
        "label": label,
        "return_code": return_code,
        "timed_out": timed_out,
        "wall_seconds": wall,
        "peak_rss_kb": max(before_rss, after_rss),
        "stdout": relative(stage_dir / f"{label}.stdout.log"),
        "stderr": relative(stage_dir / f"{label}.stderr.log"),
    }


def _source_route(dataset_id: str) -> Path:
    safe = safe_id(dataset_id)
    for root in SOURCE_ROOTS:
        candidate = root / safe
        if all((candidate / item).is_file() for item in ("inputs/device_gnss.csv", "inputs/brdc.nav", "adapter/rover.obs")):
            return candidate
    raise CarrierCompatibilityError(f"no pre-materialized development input route: {dataset_id}")


def _artifact(path: Path) -> dict[str, Any]:
    if not path.is_file():
        raise CarrierCompatibilityError(f"missing artifact: {path}")
    size = path.stat().st_size
    if size < 0 or size > MAX_OUTPUT_BYTES:
        raise CarrierCompatibilityError(f"artifact size is unsafe: {path}")
    return {"path": relative(path), "size": size, "sha256": sha256(path)}


def _parse_position_rows(path: Path) -> list[dict[str, float]]:
    rows: list[dict[str, float]] = []
    with path.open(encoding="utf-8") as handle:
        for line in handle:
            if not line.strip() or line.startswith("%"):
                continue
            fields = line.split()
            if len(fields) < 8:
                raise CarrierCompatibilityError(f"position row is incomplete: {path}")
            try:
                week, tow = float(fields[0]), float(fields[1])
                x, y, z = (float(fields[index]) for index in (2, 3, 4))
                lat, lon, height = (float(fields[index]) for index in (5, 6, 7))
            except (TypeError, ValueError) as exc:
                raise CarrierCompatibilityError(f"position row is non-numeric: {path}") from exc
            if not all(math.isfinite(value) for value in (week, tow, x, y, z, lat, lon, height)):
                raise CarrierCompatibilityError(f"position row is non-finite: {path}")
            if not (-90.0 <= lat <= 90.0 and -180.0 <= lon <= 180.0):
                raise CarrierCompatibilityError(f"position latitude/longitude out of range: {path}")
            rows.append({"week": week, "tow": tow, "x": x, "y": y, "z": z, "lat": lat, "lon": lon, "height": height})
    if not rows:
        raise CarrierCompatibilityError(f"position output has no rows: {path}")
    return rows


def _position_stats(path: Path) -> dict[str, Any]:
    rows = _parse_position_rows(path)
    jumps = 0
    max_speed = 0.0
    for prior, current in zip(rows, rows[1:]):
        dt = (current["week"] - prior["week"]) * 604800.0 + current["tow"] - prior["tow"]
        distance = math.sqrt(sum((current[key] - prior[key]) ** 2 for key in ("x", "y", "z")))
        if dt <= 0.0:
            raise CarrierCompatibilityError("position timestamps are not strictly increasing")
        speed = distance / dt
        max_speed = max(max_speed, speed)
        if speed > SPEED_BOUND_MPS:
            jumps += 1
    return {"rows": len(rows), "above_speed_bound": jumps, "max_speed_mps": max_speed, "artifact": _artifact(path)}


def _validate_trace(path: Path, max_iterations: int) -> dict[str, Any]:
    if not path.is_file():
        raise CarrierCompatibilityError(f"missing cost trace: {path}")
    rows: list[dict[str, Any]] = []
    previous_iteration = -1
    previous_cost = math.inf
    increases: list[float] = []
    converged_rows = 0
    with path.open(newline="", encoding="utf-8") as handle:
        reader = csv.DictReader(handle)
        if not {"global_iteration", "cost", "converged"}.issubset(reader.fieldnames or set()):
            raise CarrierCompatibilityError("cost trace schema is incomplete")
        for row in reader:
            try:
                iteration = int(row["global_iteration"])
                cost = float(row["cost"])
            except (TypeError, ValueError) as exc:
                raise CarrierCompatibilityError("cost trace has invalid number") from exc
            if iteration < 0 or iteration > max_iterations or not math.isfinite(cost):
                raise CarrierCompatibilityError("cost trace iteration/cost is invalid")
            if iteration <= previous_iteration:
                raise CarrierCompatibilityError("cost trace iteration is not increasing")
            increase = max(0.0, cost - previous_cost) if math.isfinite(previous_cost) else 0.0
            increases.append(increase)
            if str(row["converged"]).strip() in {"1", "true", "True"}:
                converged_rows += 1
            rows.append({"iteration": iteration, "cost": cost, "increase": increase})
            previous_iteration = iteration
            previous_cost = cost
    if not rows:
        raise CarrierCompatibilityError("cost trace has no rows")
    return {
        "rows": len(rows),
        "initial_cost": rows[0]["cost"],
        "final_cost": rows[-1]["cost"],
        "max_increase": max(increases),
        "monotonic_non_increasing": max(increases) <= 1e-9 * max(1.0, abs(rows[0]["cost"])),
        "converged_rows": converged_rows,
        "artifact": _artifact(path),
    }


def _summary_contract(summary: dict[str, Any], *, candidate: bool) -> dict[str, Any]:
    required = {
        "backend", "preset", "input_epochs", "optimized_epochs", "valid_solutions",
        "pseudorange_factors", "tdcp_factors", "tdcp_factors_inserted", "motion_factors",
        "carrier_phase_factors", "ambiguity_states", "double_difference_pseudorange_factors",
        "double_difference_carrier_factors", "single_difference_doppler_factors",
        "single_difference_tdcp_factors", "ambiguity_between_factors", "fixed_ambiguities",
        "fixed_solution", "iterations", "max_iterations", "initial_cost", "final_cost",
        "converged", "graph_factors", "graph_values", "residual_rms_m",
        "carrier_phase_residual_rms_m",
    }
    missing = sorted(required - set(summary))
    if missing:
        raise CarrierCompatibilityError("FGO summary missing: " + ", ".join(missing))
    if summary["backend"] != "eigen" or summary["preset"] != "default":
        raise CarrierCompatibilityError("FGO backend/preset changed")
    for key in required - {"backend", "preset", "fixed_solution", "converged"}:
        if not _finite(summary[key]) or float(summary[key]) < 0:
            raise CarrierCompatibilityError(f"FGO summary non-finite/negative: {key}")
    if not isinstance(summary["fixed_solution"], bool) or not isinstance(summary["converged"], bool):
        raise CarrierCompatibilityError("FGO summary boolean schema changed")
    positive = ("input_epochs", "optimized_epochs", "valid_solutions", "pseudorange_factors", "tdcp_factors", "motion_factors", "graph_factors", "graph_values")
    if any(float(summary[key]) <= 0 for key in positive):
        raise CarrierCompatibilityError("FGO summary lacks required factors/outputs")
    if int(summary["tdcp_factors_inserted"]) != int(summary["tdcp_factors"]):
        raise CarrierCompatibilityError("TDCP insertion count differs")
    for key in ("double_difference_pseudorange_factors", "double_difference_carrier_factors", "single_difference_doppler_factors", "single_difference_tdcp_factors", "ambiguity_between_factors"):
        if int(summary[key]) != 0:
            raise CarrierCompatibilityError(f"forbidden factor count is nonzero: {key}")
    if int(summary["fixed_ambiguities"]) != 0 or summary["fixed_solution"]:
        raise CarrierCompatibilityError("float compatibility lane fixed an ambiguity")
    if candidate and (int(summary["carrier_phase_factors"]) <= 0 or int(summary["ambiguity_states"]) <= 0):
        raise CarrierCompatibilityError("carrier candidate has no ambiguity factors/states")
    condition_proxy = float(summary["graph_factors"]) / max(1.0, float(summary["graph_values"]))
    ambiguity_proxy = float(summary["graph_factors"]) / max(1.0, float(summary["ambiguity_states"]))
    if not math.isfinite(condition_proxy) or not math.isfinite(ambiguity_proxy):
        raise CarrierCompatibilityError("ambiguity conditioning proxy is non-finite")
    return {
        "input_epochs": int(summary["input_epochs"]),
        "optimized_epochs": int(summary["optimized_epochs"]),
        "valid_solutions": int(summary["valid_solutions"]),
        "carrier_phase_factors": int(summary["carrier_phase_factors"]),
        "ambiguity_states": int(summary["ambiguity_states"]),
        "converged": bool(summary["converged"]),
        "iterations": int(summary["iterations"]),
        "max_iterations": int(summary["max_iterations"]),
        "initial_cost": float(summary["initial_cost"]),
        "final_cost": float(summary["final_cost"]),
        "carrier_phase_residual_rms_m": float(summary["carrier_phase_residual_rms_m"]),
        "residual_rms_m": float(summary["residual_rms_m"]),
        "ambiguity_condition_proxy": ambiguity_proxy,
        "graph_condition_proxy": condition_proxy,
        "covariance_contract": "native summary exports no covariance matrix; finite graph/ambiguity condition proxies are required and reported explicitly",
    }


def _candidate_gate(summary: dict[str, Any], trace: dict[str, Any], carrier: dict[str, Any], position: dict[str, Any]) -> tuple[bool, list[str], dict[str, Any]]:
    reasons: list[str] = []
    if not summary["converged"]:
        reasons.append("summary_not_converged")
    if int(summary["iterations"]) > 50 or int(summary["max_iterations"]) != 50:
        reasons.append("iteration_bound_mismatch")
    if not trace["monotonic_non_increasing"] or trace["converged_rows"] <= 0:
        reasons.append("cost_trace_not_monotonic_or_converged")
    if carrier["valid_rows"] <= 0 or carrier["max_arc_duration_s"] < MIN_ARC_DURATION_S:
        reasons.append("insufficient_valid_carrier_arc")
    if not _finite(carrier["prefit_rate"]["rms_mps"]) or carrier["prefit_rate"]["rms_mps"] > PREFIT_RATE_RMS_BOUND_MPS:
        reasons.append("prefit_rate_rms_exceeds_physical_bound")
    if summary["carrier_phase_residual_rms_m"] > POSTFIT_RMS_BOUND_M:
        reasons.append("postfit_carrier_rms_exceeds_physical_bound")
    if position["above_speed_bound"]:
        reasons.append("physical_continuity_violation")
    checks = {
        "converged": bool(summary["converged"]),
        "cost_trace_monotonic": bool(trace["monotonic_non_increasing"]),
        "cost_trace_convergence_row": trace["converged_rows"] > 0,
        "sufficient_valid_carrier_rows": carrier["valid_rows"] > 0,
        "max_arc_duration_s": carrier["max_arc_duration_s"],
        "min_arc_duration_s": MIN_ARC_DURATION_S,
        "prefit_rate_rms_mps": carrier["prefit_rate"]["rms_mps"],
        "prefit_rate_bound_mps": PREFIT_RATE_RMS_BOUND_MPS,
        "postfit_carrier_rms_m": summary["carrier_phase_residual_rms_m"],
        "postfit_carrier_bound_m": POSTFIT_RMS_BOUND_M,
        "ambiguity_condition_proxy_finite": math.isfinite(summary["ambiguity_condition_proxy"]),
        "physical_continuity_above_70mps": position["above_speed_bound"],
    }
    return not reasons, reasons, checks


def _copy_atomic(source: Path, target: Path) -> None:
    atomic_bytes(target, source.read_bytes())


def _spp_command(obs: Path, nav: Path, output: Path) -> list[str]:
    return [
        str(SPP_BINARY), "--obs", str(obs), "--nav", str(nav),
        "--out", str(output / "libgnsspp_spp.pos"),
        "--summary-json", str(output / "libgnsspp_spp_summary.json"),
        "--clock-csv", str(output / "libgnsspp_spp_clock.csv"),
        "--timing-csv", str(output / "libgnsspp_spp_timing.csv"), "--quiet",
    ]


def _run_route(dataset_id: str, output_root: Path) -> dict[str, Any]:
    source = _source_route(dataset_id)
    inputs = source / "inputs"
    adapter = source / "adapter"
    route_parent = output_root / "routes"
    route_parent.mkdir(parents=True, exist_ok=True)
    final_route = route_parent / safe_id(dataset_id)
    if final_route.exists():
        raise CarrierCompatibilityError(f"refusing to overwrite route artifact: {dataset_id}")
    stage = Path(tempfile.mkdtemp(prefix=f".{safe_id(dataset_id)}.", dir=str(route_parent)))
    try:
        compatibility = stage / "carrier_compat"
        compatibility.mkdir()
        carrier_stats = sanitize_rinex_carrier_boundaries(
            inputs / "device_gnss.csv", adapter / "rover.obs", compatibility / "rover.obs"
        )
        spp_dir = stage / "spp"
        spp_run = run_child(_spp_command(adapter / "rover.obs", inputs / "brdc.nav", spp_dir), spp_dir, "spp")
        if spp_run["return_code"] != 0 or spp_run["timed_out"]:
            raise CarrierCompatibilityError("SPP failed")
        seed = spp_dir / "libgnsspp_spp.pos"
        _position_stats(seed)
        baseline_dir = stage / "baseline8"
        baseline_cmd = baseline_command(adapter / "rover.obs", inputs / "brdc.nav", seed, baseline_dir)
        validate_command(baseline_cmd, candidate=False)
        baseline_run = run_child(baseline_cmd, baseline_dir, "fgo")
        if baseline_run["return_code"] != 0 or baseline_run["timed_out"]:
            raise CarrierCompatibilityError("baseline FGO failed")
        baseline_summary = load_json(baseline_dir / "fgo_summary.json", "baseline summary")
        baseline_diag = _summary_contract(baseline_summary, candidate=False)
        baseline_trace = _validate_trace(baseline_dir / "fgo_cost_trace.csv", 8)
        baseline_position_stats = _position_stats(baseline_dir / "fgo.pos")
        candidate_dir = stage / "carrier_float50"
        candidate_cmd = carrier_float_command(compatibility / "rover.obs", inputs / "brdc.nav", seed, candidate_dir)
        validate_command(candidate_cmd, candidate=True)
        candidate_run = run_child(candidate_cmd, candidate_dir, "fgo")
        candidate_errors: list[str] = []
        candidate_summary: dict[str, Any] | None = None
        candidate_diag: dict[str, Any] | None = None
        candidate_trace: dict[str, Any] | None = None
        candidate_position_stats: dict[str, Any] | None = None
        candidate_checks: dict[str, Any] = {}
        passed = False
        if candidate_run["return_code"] != 0 or candidate_run["timed_out"]:
            candidate_errors.append("candidate_process_failed")
        else:
            try:
                candidate_summary = load_json(candidate_dir / "fgo_summary.json", "carrier candidate summary")
                candidate_diag = _summary_contract(candidate_summary, candidate=True)
                candidate_trace = _validate_trace(candidate_dir / "fgo_cost_trace.csv", 50)
                candidate_position_stats = _position_stats(candidate_dir / "fgo.pos")
                passed, candidate_errors, candidate_checks = _candidate_gate(
                    candidate_diag, candidate_trace, carrier_stats, candidate_position_stats
                )
            except CarrierCompatibilityError as exc:
                candidate_errors.append(str(exc))
                passed = False
        selected_lane = "carrier_float50" if passed else "baseline8"
        selected_source = candidate_dir / "fgo.pos" if passed else baseline_dir / "fgo.pos"
        selected_dir = stage / "selected"
        selected_dir.mkdir()
        _copy_atomic(selected_source, selected_dir / "fgo.pos")
        selected_artifact = _artifact(selected_dir / "fgo.pos")
        result = {
            "schema_version": "smartphone-r5-native-fgo-carrier-compatibility-route.v1",
            "dataset_id": dataset_id,
            "status": "passed" if passed else "fallback-baseline8",
            "truth_free": True,
            "truth_opened": False,
            "source_route": relative(source),
            "source_hashes": {
                "device_gnss": sha256(inputs / "device_gnss.csv"),
                "broadcast_nav": sha256(inputs / "brdc.nav"),
                "original_rover_obs": sha256(adapter / "rover.obs"),
            },
            "spp": {"run": spp_run, "seed": _artifact(seed)},
            "carrier_boundary_sanitizer": carrier_stats,
            "baseline8": {
                "command": baseline_cmd,
                "command_contract": command_contract(baseline_cmd),
                "run": baseline_run,
                "summary": baseline_summary,
                "diagnostics": baseline_diag,
                "cost_trace": baseline_trace,
                "position": baseline_position_stats,
                "artifacts": {name: _artifact(baseline_dir / name) for name in ("fgo.pos", "fgo_summary.json", "fgo_epoch_debug.csv", "fgo_factor_debug.csv", "fgo_cost_trace.csv")},
            },
            "carrier_float50": {
                "command": candidate_cmd,
                "command_contract": command_contract(candidate_cmd),
                "run": candidate_run,
                "summary": candidate_summary,
                "diagnostics": candidate_diag,
                "cost_trace": candidate_trace,
                "position": candidate_position_stats,
                "truth_free_checks": candidate_checks,
                "candidate_errors": candidate_errors,
                "artifacts": {name: _artifact(candidate_dir / name) for name in ("fgo.pos", "fgo_summary.json", "fgo_epoch_debug.csv", "fgo_factor_debug.csv", "fgo_cost_trace.csv") if (candidate_dir / name).is_file()},
            },
            "selector": {
                "selected_lane": selected_lane,
                "reason": "all predeclared physics/solver checks passed" if passed else "candidate rejected fail-closed; exact baseline8 selected",
                "candidate_rejection_reasons": candidate_errors,
                "selected_output": selected_artifact,
                "baseline_output_sha256": sha256(baseline_dir / "fgo.pos"),
                "selected_equals_baseline": selected_artifact["sha256"] == sha256(baseline_dir / "fgo.pos"),
                "selected_equals_candidate": selected_artifact["sha256"] == sha256(candidate_dir / "fgo.pos") if (candidate_dir / "fgo.pos").is_file() else False,
            },
            "continuity": {
                "speed_bound_mps": SPEED_BOUND_MPS,
                "baseline_above_bound": baseline_position_stats["above_speed_bound"],
                "candidate_above_bound": candidate_position_stats["above_speed_bound"] if candidate_position_stats else None,
            },
            "truth_policy": {"truth_opened": False, "validation_opened": False, "holdout_opened": False, "test_opened": False},
        }
        atomic_json(stage / "route_manifest.json", result)
        route_hash = sha256(stage / "route_manifest.json")
        atomic_bytes(stage / "route_manifest.sha256", f"{route_hash}  route_manifest.json\n".encode("ascii"))
        os.replace(stage, final_route)
        result["route_manifest_sha256"] = route_hash
        return result
    except Exception:
        shutil.rmtree(stage, ignore_errors=True)
        raise


def verify_freeze() -> dict[str, Any]:
    freeze = load_json(FREEZE, "carrier compatibility freeze")
    if freeze.get("schema_version") != "smartphone-r5-gsdc2023-native-fgo-carrier-compatibility-freeze.v1":
        raise CarrierCompatibilityError("compatibility freeze schema mismatch")
    if freeze.get("status") != "frozen-before-carrier-compatibility-truth-free-run":
        raise CarrierCompatibilityError("compatibility freeze is not pre-truth")
    manifest = load_json(FREEZE_MANIFEST, "carrier compatibility freeze manifest")
    expected_freeze_hash = manifest.get("freeze_record_sha256")
    if manifest.get("freeze_record") != relative(FREEZE) or not isinstance(expected_freeze_hash, str):
        raise CarrierCompatibilityError("compatibility freeze hash mismatch")
    if sha256(FREEZE) != expected_freeze_hash:
        raise CarrierCompatibilityError("compatibility freeze record is not sealed")
    if manifest.get("role_inventory_sha256") != ROLE_INVENTORY_SHA256 or sha256(ROLE_INVENTORY) != ROLE_INVENTORY_SHA256:
        raise CarrierCompatibilityError("role inventory hash mismatch")
    if manifest.get("truth_open_count_before_freeze") != 0 or manifest.get("validation_or_holdout_materialized_before_freeze") is not False:
        raise CarrierCompatibilityError("truth/validation/holdout was opened before freeze")
    if sha256(ARCHIVE) != ARCHIVE_SHA256 or sha256(CENTRAL_INVENTORY) != CENTRAL_INVENTORY_SHA256 or sha256(PROFILE) != PROFILE_SHA256:
        raise CarrierCompatibilityError("archive/inventory/profile hash mismatch")
    if manifest.get("central_metadata_canonical_sha256") != CENTRAL_METADATA_SHA256:
        raise CarrierCompatibilityError("central metadata hash mismatch")
    if sha256(FGO_BINARY) != FGO_BINARY_SHA256 or sha256(SPP_BINARY) != SPP_BINARY_SHA256:
        raise CarrierCompatibilityError("Release binary hash mismatch")
    source_hashes = freeze.get("source_hashes")
    if not isinstance(source_hashes, dict):
        raise CarrierCompatibilityError("source hash contract missing")
    for path_string, expected in FGO_SOURCE_HASHES.items():
        if source_hashes.get(path_string) != expected or sha256(ROOT / path_string) != expected:
            raise CarrierCompatibilityError(f"source hash mismatch: {path_string}")
    compat_sources = freeze.get("compatibility_lane_sources", {})
    for path_string, expected in compat_sources.items():
        if expected in {None, "", "__FILL_AFTER_FREEZE__"} or sha256(ROOT / path_string) != expected:
            raise CarrierCompatibilityError(f"compatibility source hash mismatch: {path_string}")
    if tuple(freeze.get("cohort", {}).get("identities", ())) != COHORT:
        raise CarrierCompatibilityError("cohort changed")
    role = load_json(ROLE_INVENTORY, "role inventory")
    ids = tuple(item.get("dataset_id") for item in role.get("development_cv_cohort", ()))
    if ids != COHORT:
        raise CarrierCompatibilityError("role inventory cohort mismatch")
    if freeze.get("prior_all_device_carrier_no_go", {}).get("reinterpreted") is not False:
        raise CarrierCompatibilityError("prior all-device No-Go was reinterpreted")
    return freeze


def _verify_truth_free(output_root: Path) -> dict[str, Any]:
    manifest_path = output_root / "truth_free_manifest.json"
    manifest = load_json(manifest_path, "truth-free manifest")
    if manifest.get("schema_version") != "smartphone-r5-native-fgo-carrier-compatibility-truth-free.v1":
        raise CarrierCompatibilityError("truth-free manifest schema mismatch")
    if manifest.get("truth_open_count") != 0 or manifest.get("validation_or_holdout_opened") is not False:
        raise CarrierCompatibilityError("truth-free manifest truth policy failed")
    if tuple(manifest.get("cohort", ())) != COHORT:
        raise CarrierCompatibilityError("truth-free cohort mismatch")
    route_reports = {}
    for dataset_id in COHORT:
        route = output_root / "routes" / safe_id(dataset_id)
        route_manifest_path = route / "route_manifest.json"
        route_manifest = load_json(route_manifest_path, f"route manifest {dataset_id}")
        if route_manifest.get("truth_opened") is not False:
            raise CarrierCompatibilityError("route truth policy failed")
        expected = manifest.get("route_manifest_sha256", {}).get(dataset_id)
        observed = sha256(route_manifest_path)
        seal = (route / "route_manifest.sha256").read_text(encoding="ascii").split()[0]
        if expected != observed or seal != observed:
            raise CarrierCompatibilityError(f"route manifest hash mismatch: {dataset_id}")
        route_reports[dataset_id] = route_manifest
    return {"top": manifest, "routes": route_reports, "manifest_sha256": sha256(manifest_path)}


def truth_free_run(output_root: Path) -> dict[str, Any]:
    verify_freeze()
    if output_root.exists():
        raise CarrierCompatibilityError(f"refusing to overwrite output root: {output_root}")
    output_root.mkdir(parents=True)
    reports: dict[str, Any] = {}
    started = time.perf_counter()
    try:
        for dataset_id in COHORT:
            reports[dataset_id] = _run_route(dataset_id, output_root)
        report = {
            "schema_version": "smartphone-r5-native-fgo-carrier-compatibility-truth-free.v1",
            "status": "passed" if all(item["status"] in {"passed", "fallback-baseline8"} for item in reports.values()) else "failed-closed",
            "candidate_id": "native-fgo-v1-carrier-compatibility-selector-v1",
            "truth_free": True,
            "truth_open_count": 0,
            "validation_or_holdout_opened": False,
            "cohort": list(COHORT),
            "route_manifest_sha256": {dataset_id: item["route_manifest_sha256"] for dataset_id, item in reports.items()},
            "routes": reports,
            "elapsed_wall_seconds": time.perf_counter() - started,
            "freeze_record": relative(FREEZE),
            "freeze_record_sha256": sha256(FREEZE),
            "no_post_truth_tuning": True,
        }
        atomic_json(output_root / "truth_free_manifest.json", report)
        top_hash = sha256(output_root / "truth_free_manifest.json")
        atomic_json(output_root / "truth_free_manifest.seal.json", {"manifest": relative(output_root / "truth_free_manifest.json"), "manifest_sha256": top_hash, "truth_open_count": 0, "validation_or_holdout_opened": False})
        return {"status": report["status"], "truth_open_count": 0, "manifest_sha256": top_hash, "routes": reports}
    except Exception:
        shutil.rmtree(output_root, ignore_errors=True)
        raise


def _materialize_truth(dataset_id: str, output_root: Path) -> Path:
    route, phone = dataset_id.split("/", 1)
    member = f"dataset_2023/train/{route}/{phone}/ground_truth.csv"
    destination = output_root / "train_truth" / safe_id(dataset_id) / "ground_truth.csv"
    if destination.exists():
        raise CarrierCompatibilityError(f"truth already materialized for this phase: {dataset_id}")
    with zipfile.ZipFile(ARCHIVE) as archive:
        try:
            info = archive.getinfo(member)
        except KeyError as exc:
            raise CarrierCompatibilityError(f"declared development truth member is missing: {member}") from exc
        if info.file_size > MAX_OUTPUT_BYTES:
            raise CarrierCompatibilityError("truth member exceeds safety limit")
        with archive.open(info) as source:
            atomic_bytes(destination, source.read())
    if sha256(destination) != hashlib.sha256(destination.read_bytes()).hexdigest():
        raise CarrierCompatibilityError("truth hash could not be verified")
    return destination


def _strict_comparison(candidate: dict[str, Any], baseline: dict[str, Any]) -> dict[str, Any]:
    failures: list[str] = []
    for key in DIAGNOSTIC_KEYS:
        if not (float(candidate["kaggle_diagnostic_score_variants_m"][key]) < float(baseline["kaggle_diagnostic_score_variants_m"][key]) - 1e-9):
            failures.append(f"{key}_not_strictly_improved")
    if not float(candidate["kaggle_diagnostic_mean_m"]) < float(baseline["kaggle_diagnostic_mean_m"]) - 1e-9:
        failures.append("diagnostic_mean_not_strictly_improved")
    if float(candidate["availability_ratio"]) < float(baseline["availability_ratio"]) - 1e-12:
        failures.append("availability_regression")
    if float(candidate["truth_coverage_ratio"]) < float(baseline["truth_coverage_ratio"]) - 1e-12:
        failures.append("truth_coverage_regression")
    if float(candidate["vertical_p95_abs_m"] or math.inf) > float(baseline["vertical_p95_abs_m"] or math.inf) + 0.25:
        failures.append("vertical_safety_margin_regression")
    return {"passed": not failures, "failures": failures}


def _score_route(route_manifest: dict[str, Any], truth_path: Path, output_root: Path, dataset_id: str) -> dict[str, Any]:
    truth = smoother_eval._read_truth(truth_path)
    route = output_root / "routes" / safe_id(dataset_id)
    device = _source_route(dataset_id) / "inputs/device_gnss.csv"
    baseline = optimizer_eval._score_optimizer_position(route / "baseline8/fgo.pos", device, truth)
    candidate = optimizer_eval._score_optimizer_position(route / "carrier_float50/fgo.pos", device, truth)
    selected = optimizer_eval._score_optimizer_position(route / "selected/fgo.pos", device, truth)
    return {
        "baseline8": baseline,
        "carrier_float50": candidate,
        "selected": selected,
        "selected_lane": route_manifest["selector"]["selected_lane"],
        "gate_selected_vs_baseline": _strict_comparison(selected, baseline),
        "truth_path": relative(truth_path),
        "truth_sha256": sha256(truth_path),
    }


def _aggregate(metrics: list[dict[str, Any]]) -> dict[str, Any]:
    return native_eval._aggregate(metrics)


def train_score(output_root: Path) -> dict[str, Any]:
    verify_freeze()
    sealed = _verify_truth_free(output_root)
    reports: dict[str, Any] = {}
    truth_hashes: dict[str, str] = {}
    for dataset_id in COHORT:
        # Truth is opened only after every truth-free route manifest has been
        # hash-verified.  No validation/holdout/test member is addressable here.
        truth_path = _materialize_truth(dataset_id, output_root)
        report = _score_route(sealed["routes"][dataset_id], truth_path, output_root, dataset_id)
        reports[dataset_id] = report
        truth_hashes[dataset_id] = report["truth_sha256"]
    selected_metrics = [report["selected"] for report in reports.values()]
    baseline_metrics = [report["baseline8"] for report in reports.values()]
    candidate_metrics = [report["carrier_float50"] for report in reports.values()]
    aggregate = {
        "baseline8": _aggregate(baseline_metrics),
        "carrier_float50": _aggregate(candidate_metrics),
        "selected": _aggregate(selected_metrics),
    }
    aggregate["gate_selected_vs_baseline"] = _strict_comparison(aggregate["selected"], aggregate["baseline8"])
    route_gates = {dataset_id: report["gate_selected_vs_baseline"] for dataset_id, report in reports.items()}
    route_groups = {dataset_id: next(item["route_group"] for item in load_json(ROLE_INVENTORY, "role inventory")["development_cv_cohort"] if item["dataset_id"] == dataset_id) for dataset_id in COHORT}
    families = {dataset_id: dataset_id.split("/", 1)[1] for dataset_id in COHORT}
    folds: dict[str, Any] = {"leave_one_route_group_out": {}, "leave_one_phone_family_out": {}}
    for fold_name, groups in (("leave_one_route_group_out", route_groups), ("leave_one_phone_family_out", families)):
        for group in sorted(set(groups.values())):
            ids = [dataset_id for dataset_id in COHORT if groups[dataset_id] == group]
            base = _aggregate([reports[dataset_id]["baseline8"] for dataset_id in ids])
            selected = _aggregate([reports[dataset_id]["selected"] for dataset_id in ids])
            folds[fold_name][group] = {
                "identities": ids,
                "baseline": base,
                "selected": selected,
                "gate": _strict_comparison(selected, base),
            }
    all_route_gates_pass = all(gate["passed"] for gate in route_gates.values())
    all_fold_gates_pass = all(fold["gate"]["passed"] for kind in folds.values() for fold in kind.values())
    passed = all_route_gates_pass and all_fold_gates_pass and aggregate["gate_selected_vs_baseline"]["passed"]
    report = {
        "schema_version": "smartphone-r5-native-fgo-carrier-compatibility-train-evaluation.v1",
        "status": "promote-development-only" if passed else "no-go-train-gate",
        "candidate_id": "native-fgo-v1-carrier-compatibility-selector-v1",
        "truth_access": {
            "truth_free_artifacts_sealed_before_truth": True,
            "train_truth_open_count": len(COHORT),
            "train_truth_read_count": len(COHORT),
            "validation_truth_open_count": 0,
            "holdout_truth_open_count": 0,
            "test_truth_open_count": 0,
            "truth_hashes": truth_hashes,
        },
        "routes": reports,
        "aggregate": aggregate,
        "route_gates": route_gates,
        "cv_folds": folds,
        "gate": {
            "all_four_official_diagnostics_and_mean_strictly_improve_routewise": all_route_gates_pass,
            "all_leave_out_folds_strictly_improve": all_fold_gates_pass,
            "aggregate_strict_improvement": aggregate["gate_selected_vs_baseline"],
            "availability_vertical_continuity_non_regression": all_route_gates_pass,
            "passed": passed,
        },
        "policy": {
            "truth_features_used_by_selector": False,
            "leaderboard_used_for_tuning": False,
            "prior_holdout_or_validation_used": False,
            "post_truth_tuning": False,
            "fresh_validation_opened": False,
            "future_holdout_opened": False,
            "kaggle_submission": False,
            "production_default_changed": False,
            "next_action": "require a genuinely new validation asset" if passed else "seal No-Go; do not open validation or holdout",
        },
        "freeze": {"record": relative(FREEZE), "record_sha256": sha256(FREEZE), "manifest": relative(FREEZE_MANIFEST), "manifest_sha256": sha256(FREEZE_MANIFEST), "truth_free_manifest_sha256": sealed["manifest_sha256"]},
    }
    atomic_json(output_root / "train_evaluation.json", report)
    atomic_json(output_root / "train_evaluation.manifest.json", {
        "schema_version": "smartphone-r5-native-fgo-carrier-compatibility-train-manifest.v1",
        "report": {"path": relative(output_root / "train_evaluation.json"), "sha256": sha256(output_root / "train_evaluation.json")},
        "truth_free_manifest_sha256": sealed["manifest_sha256"],
        "truth_open_count": len(COHORT),
        "validation_truth_open_count": 0,
        "holdout_truth_open_count": 0,
        "test_truth_open_count": 0,
        "no_post_truth_tuning": True,
    })
    return report


def parse_args(argv: list[str] | None = None) -> argparse.Namespace:
    parser = argparse.ArgumentParser(prog="gnss_smartphone_native_fgo_carrier_compatibility_eval")
    subparsers = parser.add_subparsers(dest="mode", required=True)
    verify = subparsers.add_parser("verify-freeze")
    verify.set_defaults(output_root=DEFAULT_OUTPUT)
    run = subparsers.add_parser("truth-free-run")
    run.add_argument("--output-root", type=Path, default=DEFAULT_OUTPUT)
    score = subparsers.add_parser("train-score")
    score.add_argument("--output-root", type=Path, default=DEFAULT_OUTPUT)
    return parser.parse_args(argv)


def main(argv: list[str] | None = None) -> int:
    args = parse_args(argv)
    try:
        if args.mode == "verify-freeze":
            freeze = verify_freeze()
            print(json.dumps({"status": "verified", "candidate_id": freeze["candidate_id"], "freeze_record_sha256": sha256(FREEZE)}, sort_keys=True))
            return 0
        output_root = args.output_root if args.output_root.is_absolute() else ROOT / args.output_root
        if args.mode == "truth-free-run":
            result = truth_free_run(output_root.resolve())
            print(json.dumps({"status": result["status"], "truth_open_count": 0, "manifest_sha256": result["manifest_sha256"]}, sort_keys=True))
            return 0
        report = train_score(output_root.resolve())
        print(json.dumps({"status": report["status"], "truth_open_count": report["truth_access"]["train_truth_open_count"], "validation_truth_open_count": 0, "holdout_truth_open_count": 0}, sort_keys=True))
        return 0 if report["status"] == "promote-development-only" else 1
    except CarrierCompatibilityError as exc:
        print(f"carrier compatibility contract error: {exc}", file=sys.stderr)
        return 2


if __name__ == "__main__":
    raise SystemExit(main())
