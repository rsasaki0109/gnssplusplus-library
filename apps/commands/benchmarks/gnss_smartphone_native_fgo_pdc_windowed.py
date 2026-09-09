#!/usr/bin/env python3
"""Run the opt-in, truth-free smartphone PDC graph in bounded windows.

The published taroz scripts select a fixed-interval slice with ``skip`` and
``max``; they do not define a state handoff, overlap stitch, or marginalization
contract.  This wrapper therefore solves independent 120-epoch windows with a
predeclared 60-epoch overlap.  Every window starts with the native initializer
and resets clock/TDCP/carrier arcs.  Overlap rows are selected by a deterministic
distance-from-window-edge rule.  A failed or physically invalid window falls
back to the already sealed native-FGO route position file; no sample or truth
coordinate is ever used.

This is development-only.  It is deliberately not imported by the production
smartphone workflow and does not score truth.
"""

from __future__ import annotations

import argparse
from dataclasses import dataclass
import hashlib
import json
import math
import os
from pathlib import Path
import resource
import subprocess
import sys
import tempfile
import time
from typing import Any, Iterable, Sequence


_COMMANDS_DIR = Path(__file__).resolve().parents[1]
if str(_COMMANDS_DIR) not in sys.path:
    sys.path.insert(0, str(_COMMANDS_DIR))
from support.gnss_runtime import application_root  # noqa: E402


ROOT = application_root(__file__)
DEFAULT_BINARY = ROOT / "build/apps/gnss_fgo"
DEFAULT_ROUTE = ROOT / (
    "output/smartphone-r5/native-fgo-v2-processed/routes/"
    "2021-03-16-18-59-us-ca-mtv-a__pixel5"
)
DEFAULT_OBS = DEFAULT_ROUTE / "adapter/rover.obs"
DEFAULT_NAV = DEFAULT_ROUTE / "inputs/brdc.nav"
DEFAULT_FALLBACK = ROOT / (
    "output/smartphone-r5/native-fgo-carrier-compatibility-v1/routes/"
    "2021-03-16-18-59-us-ca-mtv-a__pixel5/baseline8/fgo.pos"
)
DEFAULT_OUTPUT = ROOT / "output/smartphone-r5/native-fgo-pdc-windowed-v1"

SCHEMA = "smartphone-r5-gsdc2023-native-fgo-pdc-windowed.v1"
WINDOW_EPOCHS = 120
WINDOW_OVERLAP = 60
WINDOW_STRIDE = WINDOW_EPOCHS - WINDOW_OVERLAP
MAX_ITERATIONS = 1000
RELATIVE_COST_THRESHOLD = "1e-6"
ABSOLUTE_COST_THRESHOLD = "1e-9"
MAX_RUNTIME_SECONDS = 900
MAX_ADDRESS_SPACE_BYTES = 8 * 1024 * 1024 * 1024
MAX_ARTIFACT_BYTES = 1024 * 1024 * 1024
MAX_TRANSITION_SPEED_MPS = 70.0
MIN_ECEF_NORM_M = 6_000_000.0
MAX_ECEF_NORM_M = 7_500_000.0
SEGMENT_GAP_SECONDS = 2.0
MAX_D_RESIDUAL_RMS_MPS = 4.0

# This is the existing native no-base P+D+TDCP+float-carrier recipe.  Keep the
# tuple literal auditable: changing any factor, noise, robust threshold, or
# initialization flag requires a new freeze record.
FIXED_RECIPE: tuple[str, ...] = (
    "--preset", "default", "--backend", "eigen",
    "--max-iterations", str(MAX_ITERATIONS),
    "--relative-cost-threshold", RELATIVE_COST_THRESHOLD,
    "--absolute-cost-threshold", ABSOLUTE_COST_THRESHOLD,
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
    "--undifferenced-doppler-sigma", "0.2",
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
    "--undifferenced-doppler-factors",
    "--corrected-undifferenced-doppler-factors",
    "--doppler-velocity-wls-initialization",
    "--velocity-states",
    "--velocity-motion-factors",
    "--carrier-phase-factors",
    "--ambiguity-between-factors",
    "--quiet",
)


class WindowedPdcError(ValueError):
    """Raised when the truth-free window contract cannot be verified."""


@dataclass(frozen=True)
class PosRow:
    key: tuple[int, int]
    week: int
    tow: float
    ecef: tuple[float, float, float]
    text: str


@dataclass(frozen=True)
class WindowSpec:
    index: int
    segment_index: int
    start_epoch: int
    count: int


@dataclass(frozen=True)
class WindowResult:
    spec: WindowSpec
    source: str
    rows: tuple[PosRow, ...]
    summary: dict[str, Any] | None
    command: tuple[str, ...]
    return_code: int
    wall_seconds: float
    peak_rss_kib: int
    reason: str | None
    artifact_hashes: dict[str, str]


def sha256(path: Path) -> str:
    if not path.is_file():
        raise WindowedPdcError(f"missing file: {path}")
    digest = hashlib.sha256()
    with path.open("rb") as handle:
        for chunk in iter(lambda: handle.read(1024 * 1024), b""):
            digest.update(chunk)
    return digest.hexdigest()


def relative(path: Path) -> str:
    try:
        return str(path.resolve().relative_to(ROOT.resolve()))
    except ValueError:
        return str(path.resolve())


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
        if temporary.exists():
            temporary.unlink()


def atomic_json(path: Path, value: dict[str, Any]) -> None:
    atomic_bytes(path, (json.dumps(value, indent=2, sort_keys=True) + "\n").encode("utf-8"))


def _finite(value: object) -> float:
    result = float(value)
    if not math.isfinite(result):
        raise WindowedPdcError(f"nonfinite numeric value: {value!r}")
    return result


def _key(week: int, tow: float) -> tuple[int, int]:
    return int(week), int(round(tow * 1000.0))


def read_pos(path: Path) -> list[PosRow]:
    if not path.is_file() or path.stat().st_size > MAX_ARTIFACT_BYTES:
        raise WindowedPdcError(f"invalid position artifact: {path}")
    rows: list[PosRow] = []
    seen: set[tuple[int, int]] = set()
    for raw in path.read_text(encoding="utf-8").splitlines():
        line = raw.strip()
        if not line or line.startswith("%"):
            continue
        fields = line.split()
        if len(fields) < 14:
            raise WindowedPdcError(f"position row has too few columns: {path}")
        week = int(fields[0])
        tow = _finite(fields[1])
        xyz = tuple(_finite(fields[index]) for index in (2, 3, 4))
        norm = math.sqrt(sum(value * value for value in xyz))
        if not MIN_ECEF_NORM_M <= norm <= MAX_ECEF_NORM_M:
            raise WindowedPdcError(f"ECEF outside physical range in {path}: {norm}")
        # Validate the exported geodetic columns too; retain the original line
        # so stitching does not introduce a formatting/rounding transform.
        _finite(fields[5])
        _finite(fields[6])
        key = _key(week, tow)
        if key in seen:
            raise WindowedPdcError(f"duplicate position epoch in {path}: {key}")
        seen.add(key)
        rows.append(PosRow(key, week, tow, xyz, line))
    if not rows:
        raise WindowedPdcError(f"empty position artifact: {path}")
    rows.sort(key=lambda row: row.key)
    return rows


def epoch_seconds(left: PosRow, right: PosRow) -> float:
    return ((right.key[0] - left.key[0]) * 604800000.0 + right.key[1] - left.key[1]) / 1000.0


def ecef_speed(left: PosRow, right: PosRow) -> float:
    seconds = epoch_seconds(left, right)
    if seconds <= 0.0:
        raise WindowedPdcError(f"non-increasing epoch keys: {left.key}, {right.key}")
    distance = math.sqrt(sum((a - b) ** 2 for a, b in zip(left.ecef, right.ecef)))
    return distance / seconds


def segment_ranges(rows: Sequence[PosRow]) -> list[tuple[int, int]]:
    """Return half-open contiguous ranges, resetting at clock/time gaps."""
    ranges: list[tuple[int, int]] = []
    start = 0
    for index in range(1, len(rows)):
        if epoch_seconds(rows[index - 1], rows[index]) > SEGMENT_GAP_SECONDS:
            ranges.append((start, index))
            start = index
    ranges.append((start, len(rows)))
    return ranges


def window_specs(rows: Sequence[PosRow]) -> list[WindowSpec]:
    specs: list[WindowSpec] = []
    for segment_index, (segment_start, segment_end) in enumerate(segment_ranges(rows)):
        length = segment_end - segment_start
        if length <= 0:
            continue
        final_start = max(segment_start, segment_end - WINDOW_EPOCHS)
        starts: list[int] = []
        start = segment_start
        # Walk regular strides only until the final full window.  Replacing a
        # short penultimate window with that final window keeps the contract
        # sorted and avoids duplicate/three-way tails on short segments.
        while start < final_start:
            starts.append(start)
            start += WINDOW_STRIDE
        if not starts or starts[-1] != final_start:
            starts.append(final_start)
        for start in starts:
            specs.append(WindowSpec(
                index=len(specs),
                segment_index=segment_index,
                start_epoch=start,
                count=min(WINDOW_EPOCHS, segment_end - start),
            ))
    return specs


def _required_summary(summary: dict[str, Any], count: int) -> None:
    integer_fields = (
        "input_epochs", "optimized_epochs", "valid_solutions", "iterations",
        "pseudorange_factors", "undifferenced_doppler_factors", "tdcp_factors",
        "carrier_phase_factors", "ambiguity_states", "single_difference_doppler_factors",
        "single_difference_tdcp_factors", "double_difference_pseudorange_factors",
        "double_difference_carrier_factors",
    )
    for field in integer_fields:
        if field not in summary:
            raise WindowedPdcError(f"window summary missing {field}")
        value = int(summary[field])
        if value < 0:
            raise WindowedPdcError(f"window summary has negative {field}")
    if int(summary["input_epochs"]) != count or int(summary["optimized_epochs"]) != count:
        raise WindowedPdcError("window epoch limit was not honored")
    if int(summary["valid_solutions"]) != count:
        raise WindowedPdcError("window did not produce one solution per epoch")
    if int(summary["iterations"]) > MAX_ITERATIONS or not bool(summary.get("converged")):
        raise WindowedPdcError("window optimizer did not converge within the frozen bound")
    if int(summary["pseudorange_factors"]) <= 0 or int(summary["undifferenced_doppler_factors"]) <= 0:
        raise WindowedPdcError("window lacks P/D factors")
    if int(summary["tdcp_factors"]) <= 0 or int(summary["carrier_phase_factors"]) <= 0:
        raise WindowedPdcError("window lacks TDCP/float-carrier factors")
    if int(summary["ambiguity_states"]) <= 0:
        raise WindowedPdcError("window lacks float ambiguity states")
    if any(int(summary[field]) != 0 for field in (
        "single_difference_doppler_factors", "single_difference_tdcp_factors",
        "double_difference_pseudorange_factors", "double_difference_carrier_factors",
    )):
        raise WindowedPdcError("base-dependent SD/DD factors appeared")
    initial_cost = _finite(summary["initial_cost"])
    final_cost = _finite(summary["final_cost"])
    if final_cost > initial_cost:
        raise WindowedPdcError("window cost increased")
    d_rms = _finite(summary["undifferenced_doppler_residual_rms_mps"])
    if d_rms > MAX_D_RESIDUAL_RMS_MPS:
        raise WindowedPdcError(f"D residual exceeds fixed structural ceiling: {d_rms}")
    for field in (
        "tdcp_residual_rms_m", "carrier_phase_residual_rms_m",
        "doppler_velocity_wls_max_velocity_norm_mps",
        "doppler_velocity_wls_max_clock_rate_abs_mps",
    ):
        _finite(summary[field])
    if _finite(summary["doppler_velocity_wls_max_velocity_norm_mps"]) > MAX_TRANSITION_SPEED_MPS:
        raise WindowedPdcError("Doppler initializer exceeds receiver speed bound")


def _child_limits() -> None:
    resource.setrlimit(resource.RLIMIT_AS, (MAX_ADDRESS_SPACE_BYTES, MAX_ADDRESS_SPACE_BYTES))
    resource.setrlimit(resource.RLIMIT_CPU, (MAX_RUNTIME_SECONDS, MAX_RUNTIME_SECONDS + 1))


def build_command(binary: Path, obs: Path, nav: Path, spec: WindowSpec, out_dir: Path) -> list[str]:
    return [
        str(binary), "--obs", str(obs), "--nav", str(nav),
        *FIXED_RECIPE,
        "--skip-epochs", str(spec.start_epoch), "--max-epochs", str(spec.count),
        "--out", str(out_dir / "fgo.pos"),
        "--summary-json", str(out_dir / "fgo_summary.json"),
        "--epoch-debug-csv", str(out_dir / "fgo_epoch_debug.csv"),
        "--factor-debug-csv", str(out_dir / "fgo_factor_debug.csv"),
        "--cost-trace-csv", str(out_dir / "fgo_cost_trace.csv"),
    ]


def _run_child(command: Sequence[str], out_dir: Path) -> tuple[int, float, int]:
    environment = os.environ.copy()
    library_path = "/home/sasaki/.local/lib:/opt/ros/jazzy/lib"
    if environment.get("LD_LIBRARY_PATH"):
        library_path += os.pathsep + environment["LD_LIBRARY_PATH"]
    environment["LD_LIBRARY_PATH"] = library_path
    started = time.perf_counter()
    with (out_dir / "stdout.log").open("wb") as stdout, (out_dir / "stderr.log").open("wb") as stderr:
        try:
            completed = subprocess.run(
                list(command), cwd=ROOT, env=environment, stdin=subprocess.DEVNULL,
                stdout=stdout, stderr=stderr, timeout=MAX_RUNTIME_SECONDS + 5,
                preexec_fn=_child_limits, check=False,
            )
            return_code = int(completed.returncode)
        except subprocess.TimeoutExpired:
            return_code = -int(getattr(subprocess, "ETIMEDOUT", 124))
    # ru_maxrss is a process-wide high-water mark on Linux.  It is still a
    # conservative, useful bound for this serial child lane and avoids
    # claiming an unavailable per-thread measurement.
    peak_rss_kib = int(resource.getrusage(resource.RUSAGE_CHILDREN).ru_maxrss)
    return return_code, time.perf_counter() - started, peak_rss_kib


def _artifact_hashes(directory: Path) -> dict[str, str]:
    result: dict[str, str] = {}
    for name in ("fgo.pos", "fgo_summary.json", "fgo_epoch_debug.csv", "fgo_factor_debug.csv", "fgo_cost_trace.csv"):
        path = directory / name
        if path.is_file():
            result[name] = sha256(path)
    return result


def run_window(binary: Path, obs: Path, nav: Path, spec: WindowSpec, directory: Path) -> WindowResult:
    directory.mkdir(parents=True, exist_ok=False)
    command = build_command(binary, obs, nav, spec, directory)
    return_code, wall_seconds, peak_rss_kib = _run_child(command, directory)
    summary: dict[str, Any] | None = None
    rows: tuple[PosRow, ...] = ()
    reason: str | None = None
    try:
        summary_path = directory / "fgo_summary.json"
        position_path = directory / "fgo.pos"
        if return_code != 0:
            raise WindowedPdcError(f"native process returned {return_code}")
        summary_value = json.loads(summary_path.read_text(encoding="utf-8"))
        if not isinstance(summary_value, dict):
            raise WindowedPdcError("window summary is not an object")
        summary = summary_value
        _required_summary(summary, spec.count)
        parsed = read_pos(position_path)
        if len(parsed) != spec.count:
            raise WindowedPdcError("window position row count mismatch")
        rows = tuple(parsed)
    except (OSError, json.JSONDecodeError, TypeError, ValueError, WindowedPdcError) as exc:
        reason = str(exc)
    source = "windowed_native_fgo" if reason is None else "sealed_native_fgo_fallback"
    return WindowResult(
        spec=spec, source=source, rows=rows, summary=summary,
        command=tuple(command), return_code=return_code,
        wall_seconds=wall_seconds, peak_rss_kib=peak_rss_kib,
        reason=reason, artifact_hashes=_artifact_hashes(directory),
    )


def _candidate_map(results: Sequence[WindowResult]) -> dict[tuple[int, int], list[tuple[int, PosRow]]]:
    candidates: dict[tuple[int, int], list[tuple[int, PosRow]]] = {}
    for result in results:
        if result.source != "windowed_native_fgo":
            continue
        for offset, row in enumerate(result.rows):
            candidates.setdefault(row.key, []).append((offset, row))
    return candidates


def stitch_rows(canonical: Sequence[PosRow], results: Sequence[WindowResult]) -> tuple[list[PosRow], dict[str, Any]]:
    """Stitch by max distance from an independent window edge, then index."""
    candidates = _candidate_map(results)
    selected: list[PosRow] = []
    labels: list[str] = []
    fallback_keys: list[tuple[int, int]] = []
    for baseline in canonical:
        entries = candidates.get(baseline.key, [])
        if not entries:
            selected.append(baseline)
            labels.append("sealed_native_fgo_fallback")
            fallback_keys.append(baseline.key)
            continue
        # Reconstruct the window-local distance from its edge.  The candidate
        # list's order is result order, hence the final tie-break is stable.
        ranked: list[tuple[int, int, PosRow]] = []
        for result_index, result in enumerate(results):
            for offset, row in enumerate(result.rows):
                if row.key == baseline.key:
                    ranked.append((min(offset, result.spec.count - 1 - offset), -result_index, row))
        _, _, chosen = max(ranked, key=lambda item: (item[0], item[1]))
        selected.append(chosen)
        labels.append("windowed_native_fgo")
    if len(selected) != len(canonical) or len({row.key for row in selected}) != len(canonical):
        raise WindowedPdcError("stitched key set is not one-to-one with fallback key set")
    speeds = [ecef_speed(left, right) for left, right in zip(selected, selected[1:])]
    above = [speed for speed in speeds if speed > MAX_TRANSITION_SPEED_MPS]
    # A physically unsafe independent-window seam is fail-closed as a whole
    # route.  The caller then emits the exact sealed fallback rows.
    continuity_ok = not above
    counts = {
        "windowed_native_fgo": labels.count("windowed_native_fgo"),
        "sealed_native_fgo_fallback": labels.count("sealed_native_fgo_fallback"),
    }
    return selected, {
        "source_counts": counts,
        "fallback_key_count": len(fallback_keys),
        "fallback_keys_preview": [list(key) for key in fallback_keys[:5]],
        "max_transition_speed_mps": max(speeds, default=0.0),
        "above_transition_speed_count": len(above),
        "continuity_ok": continuity_ok,
    }


def _write_pos(path: Path, rows: Sequence[PosRow]) -> None:
    payload = (
        "% LibGNSS++ Position Solution\n"
        "% Format: pos\n"
        "% Columns: GPS_Week GPS_TOW X(m) Y(m) Z(m) Lat(deg) Lon(deg) Height(m) Status Satellites PDOP Ratio FixedAmbiguities Iterations\n"
        + "\n".join(row.text for row in rows) + "\n"
    ).encode("utf-8")
    atomic_bytes(path, payload)


def _window_manifest(result: WindowResult, directory: Path) -> dict[str, Any]:
    return {
        "schema_version": SCHEMA + ".window",
        "status": "truth-free-window-sealed",
        "window": {
            "index": result.spec.index,
            "segment_index": result.spec.segment_index,
            "start_epoch": result.spec.start_epoch,
            "count": result.spec.count,
        },
        "source": result.source,
        "reason": result.reason,
        "command": list(result.command),
        "return_code": result.return_code,
        "wall_seconds": result.wall_seconds,
        "peak_rss_kib": result.peak_rss_kib,
        "summary": result.summary,
        "artifacts": {name: {"path": relative(directory / name), "sha256": value}
                      for name, value in result.artifact_hashes.items()},
        "truth_policy": {"truth_opened": False, "validation_opened": False, "holdout_opened": False},
    }


def _repair_manifest_paths(output_root: Path, value: dict[str, Any]) -> dict[str, Any]:
    """Repair only temp-directory paths after an atomic directory rename.

    The first sealed run recorded the temporary staging prefix in nested
    artifact references.  Hashes and bytes are already sealed; this repair
    rewrites references to the final content-addressed directory and nothing
    else.  It is intentionally explicit and never invokes the solver.
    """
    window_manifests = value.get("window_manifests")
    if not isinstance(window_manifests, list):
        raise WindowedPdcError("existing manifest has no window manifest list")
    for item in window_manifests:
        if not isinstance(item, dict) or not isinstance(item.get("window"), dict):
            raise WindowedPdcError("existing window manifest metadata is malformed")
        index = int(item["window"]["index"])
        final_directory = output_root / "windows" / f"window-{index:04d}"
        artifacts = item.get("artifacts", {})
        if isinstance(artifacts, dict):
            for name, artifact in artifacts.items():
                if isinstance(artifact, dict):
                    artifact["path"] = relative(final_directory / name)
        nested = final_directory / "window_manifest.json"
        if nested.is_file():
            nested_value = json.loads(nested.read_text(encoding="utf-8"))
            if not isinstance(nested_value, dict):
                raise WindowedPdcError(f"nested window manifest is malformed: {nested}")
            nested_artifacts = nested_value.get("artifacts", {})
            if isinstance(nested_artifacts, dict):
                for name, artifact in nested_artifacts.items():
                    if isinstance(artifact, dict):
                        artifact["path"] = relative(final_directory / name)
            atomic_json(nested, nested_value)
    value["manifest_paths_repaired"] = True
    return value


def _validate_existing(output_root: Path, repair_manifest_paths: bool = False) -> dict[str, Any]:
    manifest = output_root / "windowed_manifest.json"
    output = output_root / "windowed.pos"
    if not manifest.is_file() or not output.is_file():
        raise WindowedPdcError(f"existing output is not atomically sealed: {output_root}")
    value = json.loads(manifest.read_text(encoding="utf-8"))
    if not isinstance(value, dict) or value.get("schema_version") != SCHEMA:
        raise WindowedPdcError("existing windowed manifest schema mismatch")
    if value.get("windowed_position_sha256") != sha256(output):
        raise WindowedPdcError("existing windowed output hash mismatch")
    if repair_manifest_paths:
        value = _repair_manifest_paths(output_root, value)
        atomic_json(manifest, value)
        atomic_bytes(
            output_root / "windowed_manifest.sha256",
            f"{sha256(manifest)}  windowed_manifest.json\n".encode("ascii"),
        )
    return value


def run(
    *,
    binary: Path = DEFAULT_BINARY,
    obs: Path = DEFAULT_OBS,
    nav: Path = DEFAULT_NAV,
    fallback_pos: Path = DEFAULT_FALLBACK,
    output_root: Path = DEFAULT_OUTPUT,
    repair_manifest_paths: bool = False,
) -> dict[str, Any]:
    if output_root.exists():
        return _validate_existing(output_root, repair_manifest_paths)
    for path in (binary, obs, nav, fallback_pos):
        if not path.is_file():
            raise WindowedPdcError(f"required input missing: {path}")
    canonical = read_pos(fallback_pos)
    specs = window_specs(canonical)
    if not specs:
        raise WindowedPdcError("no windows generated")
    if int(os.environ.get("GNSSPP_WINDOWED_PDC_NO_TRUTH", "0")) != 1:
        # This lane is intentionally truth-free; make accidental evaluator use
        # impossible unless a test explicitly sets the assertion variable.
        raise WindowedPdcError("truth-free guard requires GNSSPP_WINDOWED_PDC_NO_TRUTH=1")
    output_root.parent.mkdir(parents=True, exist_ok=True)
    temporary_root = Path(tempfile.mkdtemp(prefix=f".{output_root.name}.", dir=str(output_root.parent)))
    started = time.perf_counter()
    results: list[WindowResult] = []
    try:
        window_root = temporary_root / "windows"
        window_root.mkdir()
        for spec in specs:
            directory = window_root / f"window-{spec.index:04d}"
            result = run_window(binary, obs, nav, spec, directory)
            expected = tuple(row.key for row in canonical[spec.start_epoch:spec.start_epoch + spec.count])
            if result.source == "windowed_native_fgo" and tuple(row.key for row in result.rows) != expected:
                result = WindowResult(
                    spec=result.spec, source="sealed_native_fgo_fallback", rows=(),
                    summary=result.summary, command=result.command,
                    return_code=result.return_code, wall_seconds=result.wall_seconds,
                    peak_rss_kib=result.peak_rss_kib, reason="window key set/order mismatch",
                    artifact_hashes=result.artifact_hashes,
                )
            results.append(result)
        stitched, continuity = stitch_rows(canonical, results)
        structural_ok = all(result.source == "windowed_native_fgo" for result in results) and bool(continuity["continuity_ok"])
        if not structural_ok:
            # Fallback is the unmodified sealed route position artifact, not a
            # rerun and not an official sample coordinate.
            stitched = canonical
        _write_pos(temporary_root / "windowed.pos", stitched)
        window_manifests = []
        for result in results:
            directory = window_root / f"window-{result.spec.index:04d}"
            manifest = _window_manifest(result, directory)
            atomic_json(directory / "window_manifest.json", manifest)
            window_manifests.append(manifest)
        output_hash = sha256(temporary_root / "windowed.pos")
        manifest: dict[str, Any] = {
            "schema_version": SCHEMA,
            "status": "truth-free-windowed-pdc-sealed",
            "candidate": "native_fgo_pdc_independent_window_overlap_v1",
            "route": "2021-03-16-18-59-us-ca-mtv-a/pixel5",
            "truth_policy": {
                "truth_free": True, "truth_opened": False,
                "validation_opened": False, "holdout_opened": False,
                "test_data_used": False,
            },
            "inputs": {
                "obs": {"path": relative(obs), "sha256": sha256(obs)},
                "nav": {"path": relative(nav), "sha256": sha256(nav)},
                "sealed_fallback_position": {"path": relative(fallback_pos), "sha256": sha256(fallback_pos)},
                "binary": {"path": relative(binary), "sha256": sha256(binary)},
            },
            "recipe": {
                "flags": list(FIXED_RECIPE),
                "factor_contract": "P+D(corrected undifferenced)+TDCP+float carrier+velocity/clock/position motion",
                "no_base_or_double_difference": True,
                "robust_loss": {"native": "Huber", "pseudorange_sigma_units": 4, "tdcp_sigma_units": 4, "carrier_sigma_units": 4},
                "optimizer": {"max_iterations": MAX_ITERATIONS, "relative_cost_threshold": RELATIVE_COST_THRESHOLD, "absolute_cost_threshold": ABSOLUTE_COST_THRESHOLD},
            },
            "published_window_contract": {
                "taroz_source": "output/reproducibility-cache/gsdc2023/run_fgo.m:fgo_gnss_imu independent invocation; no window/stitch handoff",
                "local_window_epochs": WINDOW_EPOCHS,
                "local_overlap_epochs": WINDOW_OVERLAP,
                "local_stride_epochs": WINDOW_STRIDE,
                "state_handoff": "none; each window reinitializes and resets clock/TDCP/carrier arcs",
                "ambiguity_policy": "float native ambiguity states; no carry across window boundary",
                "clock_policy": "native per-window initialization; reset at segment/window boundary",
                "marginalization": "not used; existing fixed-lag API requires IMU+Pose3 and DD factors",
                "stitch_rule": "maximum distance from window edge; stable lower window index tie-break",
                "segment_reset_gap_seconds": SEGMENT_GAP_SECONDS,
            },
            "fallback_contract": {
                "on_window_error": "exact sealed fallback rows",
                "on_stitch_physical_failure": "exact sealed fallback rows for entire route",
                "fallback_is_sample_or_truth": False,
                "earth_ecef_norm_m": [MIN_ECEF_NORM_M, MAX_ECEF_NORM_M],
                "max_transition_speed_mps": MAX_TRANSITION_SPEED_MPS,
            },
            "window_count": len(specs),
            "window_manifests": window_manifests,
            "continuity": continuity,
            "structural_gate": {
                "all_windows_converged_and_complete": all(result.source == "windowed_native_fgo" for result in results),
                "all_windows_finite": all(result.rows for result in results if result.source == "windowed_native_fgo"),
                "d_residual_rms_ceiling_mps": MAX_D_RESIDUAL_RMS_MPS,
                "no_transition_above_bound": bool(continuity["continuity_ok"]),
                "passed": structural_ok,
            },
            "timing": {
                "wall_seconds": time.perf_counter() - started,
                "max_child_rss_kib": max((result.peak_rss_kib for result in results), default=0),
                "sum_child_wall_seconds": sum(result.wall_seconds for result in results),
            },
            "source_counts": continuity["source_counts"],
            "windowed_position_sha256": output_hash,
            "windowed_position_rows": len(stitched),
            "atomic_publish": True,
            "no_post_truth_tuning": True,
        }
        atomic_json(temporary_root / "windowed_manifest.json", manifest)
        atomic_bytes(temporary_root / "windowed_manifest.sha256", f"{sha256(temporary_root / 'windowed_manifest.json')}  windowed_manifest.json\n".encode("ascii"))
        os.replace(temporary_root, output_root)
        return manifest
    except Exception:
        # Keep a failed run out of the authoritative output namespace.  The
        # per-window temporary directory may be inspected by the caller's OS
        # cleanup; no partial published artifact is accepted.
        import shutil
        shutil.rmtree(temporary_root, ignore_errors=True)
        raise


def main(argv: Iterable[str] | None = None) -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--binary", type=Path, default=DEFAULT_BINARY)
    parser.add_argument("--obs", type=Path, default=DEFAULT_OBS)
    parser.add_argument("--nav", type=Path, default=DEFAULT_NAV)
    parser.add_argument("--fallback-pos", type=Path, default=DEFAULT_FALLBACK)
    parser.add_argument("--output-root", type=Path, default=DEFAULT_OUTPUT)
    parser.add_argument("--no-truth", action="store_true", help="assert this run must not access truth")
    parser.add_argument(
        "--repair-manifest-paths", action="store_true",
        help="repair final artifact references in an existing sealed run without rerunning the solver",
    )
    args = parser.parse_args(list(argv) if argv is not None else None)
    if args.no_truth:
        os.environ["GNSSPP_WINDOWED_PDC_NO_TRUTH"] = "1"
    try:
        manifest = run(
            binary=args.binary, obs=args.obs, nav=args.nav,
            fallback_pos=args.fallback_pos, output_root=args.output_root,
            repair_manifest_paths=args.repair_manifest_paths,
        )
    except (OSError, ValueError, WindowedPdcError) as exc:
        print(f"windowed PDC failed closed: {exc}", file=sys.stderr)
        return 2
    print(json.dumps({
        "output_root": relative(args.output_root),
        "status": manifest.get("status"),
        "structural_passed": manifest.get("structural_gate", {}).get("passed"),
        "window_count": manifest.get("window_count"),
        "source_counts": manifest.get("source_counts"),
        "windowed_position_sha256": manifest.get("windowed_position_sha256"),
    }, sort_keys=True))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
