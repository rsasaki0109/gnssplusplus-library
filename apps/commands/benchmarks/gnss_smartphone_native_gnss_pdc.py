#!/usr/bin/env python3
"""Run the native raw-Android GNSS/PDC route with a provenance boundary.

The estimator remains ``build/apps/gnss_pos_vel_pdc``.  This command only
validates the raw CSV and broadcast navigation, invokes that executable with
``--android-raw`` (there is no Python/RINEX/coordinate stage), and atomically
publishes the generated artifacts together with hashes of the approved inputs.
Any MATLAB data path, saved result, truth, sample, or prior submission is
rejected before opening it.
"""

from __future__ import annotations

import argparse
import hashlib
import json
import os
from pathlib import Path
import subprocess
import sys
import tempfile
import time
from typing import Any


BENCHMARK_DIR = Path(__file__).resolve().parent
ROOT = BENCHMARK_DIR.parents[2]
if str(BENCHMARK_DIR) not in sys.path:
    sys.path.insert(0, str(BENCHMARK_DIR))
import gnss_smartphone_native_only as native_only  # noqa: E402


SCHEMA_VERSION = "smartphone-r5-native-gnss-pdc-run-manifest.v1"
ARTIFACT_NAMES = (
    "per_epoch.csv",
    "keyed.csv",
    "factor.csv",
    "graph.csv",
    "summary.json",
)


class NativeGnssPdcError(ValueError):
    """Raised when the raw-only orchestration contract cannot be met."""


def sha256_file(path: Path) -> str:
    if not path.is_file():
        raise NativeGnssPdcError(f"missing regular file: {path}")
    digest = hashlib.sha256()
    try:
        with path.open("rb") as handle:
            for block in iter(lambda: handle.read(1024 * 1024), b""):
                digest.update(block)
    except OSError as exc:
        raise NativeGnssPdcError(f"failed to hash {path}: {exc}") from exc
    return digest.hexdigest()


def _reject_path(path: Path, label: str) -> None:
    try:
        native_only._reject_path(path, label)
    except native_only.NativeOnlyInputError as exc:
        raise NativeGnssPdcError(str(exc)) from exc
    except AttributeError as exc:  # pragma: no cover - contract module drift
        raise NativeGnssPdcError("native-only path contract is unavailable") from exc


def preflight(raw_path: Path, nav_path: Path, binary: Path) -> dict[str, Any]:
    """Validate paths and raw headers before any estimator process starts."""

    _reject_path(raw_path, "device_gnss.csv")
    _reject_path(nav_path, "broadcast navigation")
    _reject_path(binary, "native PDC binary")
    if raw_path.name.lower() != "device_gnss.csv":
        raise NativeGnssPdcError("raw GNSS input must be named device_gnss.csv")
    if nav_path.suffix.lower() not in {".nav", ".rnx", ".rinex", ".gz"}:
        raise NativeGnssPdcError("broadcast navigation must be a RINEX navigation file")
    if not binary.is_file():
        raise NativeGnssPdcError(f"native PDC binary is missing: {binary}")
    try:
        gnss = native_only.inspect_raw_gnss(raw_path)
        nav = native_only.inspect_broadcast_nav(nav_path)
    except native_only.NativeOnlyInputError as exc:
        raise NativeGnssPdcError(str(exc)) from exc
    return {
        "gnss": gnss,
        "broadcast_nav": nav,
        "binary": {"path": str(binary), "sha256": sha256_file(binary)},
    }


def _atomic_json(path: Path, payload: dict[str, Any]) -> None:
    content = (json.dumps(payload, indent=2, sort_keys=True) + "\n").encode("utf-8")
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


def _resolve(path: Path) -> Path:
    return path if path.is_absolute() else ROOT / path


def run(args: argparse.Namespace) -> dict[str, Any]:
    raw_path = _resolve(args.android_raw)
    nav_path = _resolve(args.nav)
    binary = _resolve(args.binary)
    output_dir = _resolve(args.output_dir)
    _reject_path(output_dir, "native PDC output directory")
    if output_dir.exists():
        raise NativeGnssPdcError(
            f"output directory already exists; refusing non-atomic overwrite: {output_dir}"
        )
    preflight_data = preflight(raw_path, nav_path, binary)

    output_dir.parent.mkdir(parents=True, exist_ok=True)
    temporary_dir = Path(
        tempfile.mkdtemp(prefix=f".{output_dir.name}.", dir=str(output_dir.parent))
    )
    started = time.monotonic()
    command = [
        str(binary),
        "--android-raw",
        str(raw_path),
        "--nav",
        str(nav_path),
        "--trip-id",
        args.trip_id,
        "--keyed-out-csv",
        str(temporary_dir / "keyed.csv"),
        "--out-csv",
        str(temporary_dir / "per_epoch.csv"),
        "--factor-debug-csv",
        str(temporary_dir / "factor.csv"),
        "--graph-csv",
        str(temporary_dir / "graph.csv"),
        "--summary-json",
        str(temporary_dir / "summary.json"),
        "--quiet",
    ]
    if args.device_model:
        trip_flag_index = command.index("--trip-id")
        command[trip_flag_index:trip_flag_index] = ["--device-model", args.device_model]
    if args.upstream_residual_snr:
        command.append("--upstream-residual-snr")
    if args.upstream_state_contract:
        command.append("--upstream-state-contract")
    environment = os.environ.copy()
    try:
        completed = subprocess.run(
            command,
            cwd=ROOT,
            env=environment,
            stdin=subprocess.DEVNULL,
            stdout=subprocess.PIPE,
            stderr=subprocess.PIPE,
            text=True,
            timeout=args.timeout_seconds,
            check=False,
        )
    except (OSError, subprocess.TimeoutExpired) as exc:
        raise NativeGnssPdcError(f"native PDC process failed: {exc}") from exc
    wall_seconds = time.monotonic() - started
    if completed.returncode != 0:
        raise NativeGnssPdcError(
            f"native PDC returned {completed.returncode}: {completed.stderr.strip()}"
        )

    summary_path = temporary_dir / "summary.json"
    try:
        summary = json.loads(summary_path.read_text(encoding="utf-8"))
    except (OSError, json.JSONDecodeError) as exc:
        raise NativeGnssPdcError("native PDC did not produce valid summary JSON") from exc
    if summary.get("input_mode") != "android_raw" or summary.get(
        "device_wls_seed_consumed"
    ) is not False:
        raise NativeGnssPdcError("native PDC summary violates raw-only seed contract")
    artifacts: dict[str, Any] = {}
    for name in ARTIFACT_NAMES:
        path = temporary_dir / name
        artifacts[name] = {
            "path": name,
            "bytes": path.stat().st_size,
            "sha256": sha256_file(path),
        }
    manifest: dict[str, Any] = {
        "schema_version": SCHEMA_VERSION,
        "status": "truth-free-artifacts-sealed",
        "truth_free": True,
        "trip_id": args.trip_id,
        "preflight": preflight_data,
        "command": command,
        "approved_inputs": ["raw device_gnss.csv", "broadcast RINEX navigation"],
        "preprocessing": {
            "upstream_residual_snr_opt_in": args.upstream_residual_snr,
            "rules": (
                "taroz exobs_residuals P-D/L-D pair masks and residual gates; "
                "obserrmodel MATLAB prctile(85) SNR weights"
            )
            if args.upstream_residual_snr
            else "legacy native PDC weighting and masks",
            "base_compensation": "not applied; base observations are forbidden",
        },
        "state_backend": {
            "upstream_state_contract_opt_in": args.upstream_state_contract,
            "contract": (
                "actual raw epoch dt in midpoint motion/clock terms; "
                "temporal edges require 0 < dt < 1.5 s; ECEF/five-clock "
                "Eigen state remains unchanged"
            )
            if args.upstream_state_contract
            else "legacy ECEF/five-clock Eigen state with unit-interval temporal terms",
            "full_upstream_equivalence": False,
        },
        "forbidden_input_policy": {
            "mat_paths_rejected_before_open": True,
            "result_coordinates_read": False,
            "ground_truth_read": False,
            "sample_coordinates_read": False,
            "v5_output_read": False,
            "python_coordinate_or_rinex_stage": False,
            "base_or_double_difference_factors": False,
        },
        "artifacts": artifacts,
        "summary": summary,
        "wall_seconds": wall_seconds,
        "atomic_publish": True,
        "no_external_mutation": True,
    }
    _atomic_json(temporary_dir / "run_manifest.json", manifest)
    manifest_sha256 = sha256_file(temporary_dir / "run_manifest.json")
    (temporary_dir / "run_manifest.sha256").write_text(
        f"{manifest_sha256}  run_manifest.json\n", encoding="ascii"
    )
    manifest["run_manifest_sha256"] = manifest_sha256
    os.replace(temporary_dir, output_dir)
    return manifest


def _parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--android-raw", type=Path, required=True)
    parser.add_argument("--nav", type=Path, required=True)
    parser.add_argument("--trip-id", required=True)
    parser.add_argument("--output-dir", type=Path, required=True)
    parser.add_argument(
        "--binary", type=Path, default=ROOT / "build" / "apps" / "gnss_pos_vel_pdc"
    )
    parser.add_argument("--device-model", default="")
    parser.add_argument(
        "--upstream-residual-snr",
        action="store_true",
        help=(
            "opt in to the raw-only taroz exobs_residuals/obserrmodel port; "
            "the default lane is unchanged"
        ),
    )
    parser.add_argument(
        "--upstream-state-contract",
        action="store_true",
        help=(
            "opt in to actual raw epoch intervals for temporal motion/clock "
            "terms; the legacy ECEF/five-clock state remains unchanged"
        ),
    )
    parser.add_argument("--timeout-seconds", type=float, default=900.0)
    return parser.parse_args()


def main() -> int:
    try:
        manifest = run(_parse_args())
    except (NativeGnssPdcError, native_only.NativeOnlyInputError) as exc:
        print(f"native raw GNSS/PDC rejected: {exc}", file=sys.stderr)
        return 2
    print(json.dumps(manifest, sort_keys=True))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
