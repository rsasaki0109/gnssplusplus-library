#!/usr/bin/env python3
"""Truth-free contract and smoke runner for no-base float carrier phase.

The native Eigen FGO already has an undifferenced carrier-phase path.  This
research-only wrapper enables that path with the existing CLI and checks the
contract around it; it deliberately does not change the estimator, defaults,
or any numeric parameter.  Carrier observations are accepted only after the
adapter's finite/ADR-valid processing, with Hatch-30 E1 input and rover LLI
rejection.  Every arc is keyed by (constellation, PRN, signal) and a gap,
loss-of-lock, or clock discontinuity retires the arc.

No-base, no SD/DD factors, and no integer fixing are allowed.  A failed
candidate is fail-closed and callers may retain the exact v1 output.  The
current archive is exhausted for a strict new three-identity train split, so
the command intentionally stops before truth scoring; ``structural-smoke``
only reuses three prior truth-free materializations and never reads truth.
"""

from __future__ import annotations

import argparse
import csv
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
from typing import Any


_COMMANDS_DIR = Path(__file__).resolve().parents[1]
if str(_COMMANDS_DIR) not in sys.path:
    sys.path.insert(0, str(_COMMANDS_DIR))
from support.gnss_runtime import application_root  # noqa: E402


ROOT = application_root(__file__)
FREEZE = ROOT / "docs/use_cases/records/smartphone_r5_gsdc2023_native_fgo_float_carrier_freeze_v1.json"
FREEZE_MANIFEST = ROOT / "docs/use_cases/records/smartphone_r5_gsdc2023_native_fgo_float_carrier_freeze_v1_manifest.json"
ARCHIVE = ROOT / "data/gsdc2023/cache/dataset_2023.zip"
CENTRAL_INVENTORY = ROOT / "output/smartphone-r5/generalization-v6/archive_inventory.json"
PROFILE = ROOT / "configs/benchmarks/smartphone_r5_gsdc2023.json"
FGO_BINARY = ROOT / "build/apps/gnss_fgo"
SPP_BINARY = ROOT / "build/apps/gnss_spp"
OUTPUT_ROOT = ROOT / "output/smartphone-r5/native-fgo-float-carrier-v1"

FREEZE_SHA256 = "ad0481e4833ed9299f3967dad4f30ea482ecf84f6f72ecd21978d09f84163b20"
ARCHIVE_SHA256 = "bda30ab456e6fd6f83550c246e8dbd287306d5385f1f1069c99c16298e647408"
CENTRAL_INVENTORY_SHA256 = "f4e68109885eecfc14b2bd5e8fab87e18d73473c04bb7f53da31b7040a8e90a7"
CENTRAL_METADATA_SHA256 = "5f619be94a00c33c3067f13b1cd3351d96f2804f07fa04eda8fb027739fb0992"
PROFILE_SHA256 = "273dfcc4e4636940d5216cca793a55773d9a04f668d1bfa5fcdc0013f4768776"
FGO_BINARY_SHA256 = "9190455cb34a9e818b79ed2ed6b023af5711fbef545215d4588bfed87f5c03e4"
SPP_BINARY_SHA256 = "4d272940437c2ab2dcffef31b7541c8c01212e97d2d2320edcf7bd8f80ea3c12"
FGO_SOURCE_HASHES = {
    "apps/native/gnss_fgo.cpp": "2d9e53485c427d1e0d43b07dd611aaf3644e2cd51d5e175c2977038ca05e3e32",
    "src/algorithms/fgo.cpp": "4805a30741e3bcc53a2a29d832429924a3cc46c0fac5aa530446cedd224af3b2",
    "include/libgnss++/algorithms/fgo.hpp": "67845f6365eb1f621afed19c52b82ad48d00ac1623e99b3bf7b96cd944263307",
    "include/libgnss++/algorithms/fgo_config.hpp": "8a9390e6709f4c4a55ca4f9d4d43fd451465234d1aa55cd4dd662b06f0872d80",
    "apps/commands/benchmarks/gnss_smartphone_gnss_adapter.py": "4a26dbbef0d5eff4a4840c43b600d790d573accc9977a9993754386ad086466b",
    "configs/benchmarks/smartphone_r5_gsdc2023.json": PROFILE_SHA256,
}
STRUCTURAL_SMOKE_IDS = (
    "2021-03-16-18-59-us-ca-mtv-a/pixel5",
    "2021-07-27-19-49-us-ca-mtv-b/pixel4",
    "2022-02-24-18-29-us-ca-lax-o/pixel5",
)
SMOKE_INPUT_ROOT = ROOT / "output/smartphone-r5/native-fgo-v2-processed/routes"
LEAP_SECONDS = 18
MAX_RUNTIME_SECONDS = 900
MAX_ADDRESS_SPACE_BYTES = 8 * 1024 * 1024 * 1024
MAX_OUTPUT_BYTES = 1024 * 1024 * 1024
SPEED_BOUND_MPS = 70.0
MAX_TDCP_GAP_S = 2.0


class FloatCarrierError(ValueError):
    """Raised when the frozen float-carrier contract is violated."""


def sha256(path: Path) -> str:
    if not path.is_file():
        raise FloatCarrierError(f"missing file: {path}")
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
        value = json.loads(path.read_text(encoding="utf-8"))
    except (OSError, json.JSONDecodeError) as exc:
        raise FloatCarrierError(f"invalid {label}: {path}") from exc
    if not isinstance(value, dict):
        raise FloatCarrierError(f"{label} must be an object: {path}")
    return value


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
    atomic_bytes(
        path,
        (json.dumps(payload, indent=2, sort_keys=True) + "\n").encode("utf-8"),
    )


def safe_id(dataset_id: str) -> str:
    if dataset_id.count("/") != 1 or any(ch.isspace() for ch in dataset_id):
        raise FloatCarrierError(f"invalid dataset id: {dataset_id}")
    return dataset_id.replace("/", "__")


def verify_freeze() -> dict[str, Any]:
    """Verify the pre-materialization freeze and all immutable inputs."""

    freeze = load_json(FREEZE, "float-carrier freeze")
    if freeze.get("schema_version") != "smartphone-r5-gsdc2023-native-fgo-float-carrier-freeze.v1":
        raise FloatCarrierError("float-carrier freeze schema mismatch")
    if freeze.get("status") != "frozen-before-float-carrier-materialization":
        raise FloatCarrierError("float-carrier freeze is not pre-materialization")
    manifest = load_json(FREEZE_MANIFEST, "float-carrier freeze manifest")
    if manifest.get("schema_version") != "smartphone-r5-gsdc2023-native-fgo-float-carrier-freeze-manifest.v1":
        raise FloatCarrierError("float-carrier freeze manifest schema mismatch")
    if manifest.get("freeze_record") != relative(FREEZE):
        raise FloatCarrierError("float-carrier freeze record path mismatch")
    if manifest.get("freeze_record_sha256") != FREEZE_SHA256 or sha256(FREEZE) != FREEZE_SHA256:
        raise FloatCarrierError("float-carrier freeze record hash mismatch")
    if manifest.get("truth_open_count_before_freeze") != 0:
        raise FloatCarrierError("new truth was opened before freeze")
    if manifest.get("new_payload_materialized_before_freeze") is not False:
        raise FloatCarrierError("new payload was materialized before freeze")
    if manifest.get("new_validation_materialized_before_freeze") is not False:
        raise FloatCarrierError("new validation was materialized before freeze")
    if manifest.get("new_holdout_materialized_before_freeze") is not False:
        raise FloatCarrierError("new holdout was materialized before freeze")
    archive = freeze.get("archive")
    if not isinstance(archive, dict):
        raise FloatCarrierError("archive contract is missing")
    if archive.get("sha256") != ARCHIVE_SHA256 or sha256(ARCHIVE) != ARCHIVE_SHA256:
        raise FloatCarrierError("archive hash mismatch")
    if archive.get("central_inventory_sha256") != CENTRAL_INVENTORY_SHA256 or sha256(CENTRAL_INVENTORY) != CENTRAL_INVENTORY_SHA256:
        raise FloatCarrierError("central inventory hash mismatch")
    if archive.get("central_metadata_canonical_sha256") != CENTRAL_METADATA_SHA256:
        raise FloatCarrierError("central metadata hash mismatch")
    source_hashes = freeze.get("source_hashes")
    if not isinstance(source_hashes, dict):
        raise FloatCarrierError("source hash contract is missing")
    for path_string, expected in FGO_SOURCE_HASHES.items():
        if source_hashes.get(path_string) != expected or sha256(ROOT / path_string) != expected:
            raise FloatCarrierError(f"source hash mismatch: {path_string}")
    binaries = freeze.get("binary_hashes")
    if not isinstance(binaries, dict):
        raise FloatCarrierError("binary hash contract is missing")
    if binaries.get("build/apps/gnss_fgo") != FGO_BINARY_SHA256 or sha256(FGO_BINARY) != FGO_BINARY_SHA256:
        raise FloatCarrierError("FGO binary hash mismatch")
    if binaries.get("build/apps/gnss_spp") != SPP_BINARY_SHA256 or sha256(SPP_BINARY) != SPP_BINARY_SHA256:
        raise FloatCarrierError("SPP binary hash mismatch")
    if freeze.get("split_audit", {}).get("status") != "blocked-no-strict-unused-three-way-split":
        raise FloatCarrierError("split audit was unexpectedly changed")
    algorithm = freeze.get("algorithm_contract", {})
    if algorithm.get("backend") != "Eigen native FGO" or algorithm.get("no_base") is True:
        # ``no_base`` is not a required field in the record; this branch only
        # catches an explicitly contradictory future edit.
        raise FloatCarrierError("algorithm backend/base contract changed")
    if sha256(PROFILE) != PROFILE_SHA256:
        raise FloatCarrierError("benchmark profile hash mismatch")
    return freeze


def arc_break_reason(
    previous: dict[str, Any] | None,
    current: dict[str, Any],
    *,
    max_gap_s: float = MAX_TDCP_GAP_S,
) -> str:
    """Return the deterministic arc transition reason.

    This mirrors the native builder's key and gap/loss rules and makes the
    reset boundary independently testable without opening archive payloads.
    """

    if previous is None:
        return "initial"
    if (previous.get("constellation"), previous.get("prn"), previous.get("signal")) != (
        current.get("constellation"), current.get("prn"), current.get("signal")
    ):
        return "identity-change"
    try:
        delta = float(current["timestamp_s"]) - float(previous["timestamp_s"])
    except (KeyError, TypeError, ValueError) as exc:
        raise FloatCarrierError("arc timestamps must be finite numbers") from exc
    if not math.isfinite(delta) or delta <= 0.0:
        return "nonpositive-time"
    if max_gap_s > 0.0 and delta > max_gap_s:
        return "gap"
    if bool(current.get("loss_of_lock")) or bool(current.get("adr_invalid")):
        return "loss-of-lock-or-invalid-adr"
    if bool(current.get("clock_discontinuity")):
        return "hardware-clock-discontinuity"
    return "continue"


def _common_fgo_command(
    obs: Path,
    nav: Path,
    seed: Path,
    output: Path,
    *,
    max_epochs: int = 0,
    skip_epochs: int = 0,
) -> list[str]:
    """Build the frozen v1 command surface shared by both lanes."""

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
        "--skip-epochs", str(skip_epochs),
        "--max-epochs", str(max_epochs),
        "--max-iterations", "8",
        "--relative-cost-threshold", "0",
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


def baseline_fgo_command(
    obs: Path,
    nav: Path,
    seed: Path,
    output: Path,
    *,
    max_epochs: int = 0,
    skip_epochs: int = 0,
) -> list[str]:
    """Return the exact frozen native-FGO v1 baseline command."""

    return _common_fgo_command(
        obs, nav, seed, output, max_epochs=max_epochs, skip_epochs=skip_epochs
    )


def float_carrier_fgo_command(
    obs: Path,
    nav: Path,
    seed: Path,
    output: Path,
    *,
    max_epochs: int = 0,
    skip_epochs: int = 0,
) -> list[str]:
    """Return v1 plus no-base float carrier/ADR-valid CLI switches."""

    command = _common_fgo_command(
        obs, nav, seed, output, max_epochs=max_epochs, skip_epochs=skip_epochs
    )
    command.extend(("--carrier-phase-factors", "--reject-rover-carrier-lli"))
    return command


def command_contract(command: list[str]) -> dict[str, Any]:
    """Inspect a command without executing it."""

    values = set(command)
    base_index = command.index("--base") if "--base" in values else -1
    forbidden = {
        "--base": "base observations",
        "--fix-ambiguities": "integer ambiguity fixing",
        "--fix-all-ambiguities": "integer ambiguity fixing",
        "--sd-doppler-factors": "single-difference Doppler",
        "--sd-tdcp-factors": "single-difference TDCP",
        "--ambiguity-between-factors": "ambiguity-between factors",
    }
    violations = [reason for flag, reason in forbidden.items() if flag in values]
    if base_index >= 0:
        violations.append("base observations")
    return {
        "carrier_phase_enabled": "--carrier-phase-factors" in values,
        "reject_rover_carrier_lli": "--reject-rover-carrier-lli" in values,
        "no_double_difference": "--no-dd-factors" in values,
        "backend": command[command.index("--backend") + 1] if "--backend" in values else None,
        "violations": sorted(set(violations)),
    }


def validate_command(command: list[str], *, candidate: bool) -> None:
    contract = command_contract(command)
    if contract["violations"]:
        raise FloatCarrierError("forbidden FGO command flags: " + ", ".join(contract["violations"]))
    if contract["backend"] != "eigen":
        raise FloatCarrierError("float carrier lane requires Eigen backend")
    if "--no-dd-factors" not in command:
        raise FloatCarrierError("no-base lane must explicitly disable DD factors")
    if candidate and not contract["carrier_phase_enabled"]:
        raise FloatCarrierError("candidate lane did not enable carrier phase factors")
    if candidate and not contract["reject_rover_carrier_lli"]:
        raise FloatCarrierError("candidate lane did not reject rover LLI")
    if not candidate and contract["carrier_phase_enabled"]:
        raise FloatCarrierError("baseline unexpectedly enables carrier phase")


def validate_float_carrier_summary(summary: dict[str, Any], *, require_converged: bool = True) -> dict[str, Any]:
    """Validate finite factor/output diagnostics for a candidate lane."""

    required = (
        "backend", "preset", "input_epochs", "optimized_epochs", "valid_solutions",
        "pseudorange_factors", "tdcp_factors", "tdcp_factors_inserted",
        "motion_factors", "carrier_phase_factors", "ambiguity_states",
        "double_difference_pseudorange_factors", "double_difference_carrier_factors",
        "single_difference_doppler_factors", "single_difference_tdcp_factors",
        "ambiguity_between_factors", "fixed_ambiguities", "fixed_solution",
        "iterations", "max_iterations", "initial_cost", "final_cost",
        "converged", "graph_factors", "graph_values", "residual_rms_m",
        "carrier_phase_residual_rms_m",
    )
    missing = [key for key in required if key not in summary]
    if missing:
        raise FloatCarrierError("FGO summary missing fields: " + ", ".join(missing))
    if summary["backend"] != "eigen" or summary["preset"] != "default":
        raise FloatCarrierError("FGO backend/preset changed")
    bool_fields = ("fixed_solution", "converged")
    for key in bool_fields:
        if not isinstance(summary[key], bool):
            raise FloatCarrierError(f"summary field is not boolean: {key}")
    numeric_fields = [key for key in required if key not in {"backend", "preset", *bool_fields}]
    for key in numeric_fields:
        try:
            value = float(summary[key])
        except (TypeError, ValueError) as exc:
            raise FloatCarrierError(f"summary field is not numeric: {key}") from exc
        if not math.isfinite(value) or value < 0.0:
            raise FloatCarrierError(f"summary field is non-finite/negative: {key}")
    positive = (
        "input_epochs", "optimized_epochs", "valid_solutions", "pseudorange_factors",
        "tdcp_factors", "motion_factors", "carrier_phase_factors", "ambiguity_states",
        "graph_factors", "graph_values",
    )
    if any(float(summary[key]) <= 0.0 for key in positive):
        raise FloatCarrierError("required FGO factor/state count is zero")
    if int(summary["tdcp_factors_inserted"]) != int(summary["tdcp_factors"]):
        raise FloatCarrierError("TDCP inserted/build count differs")
    for key in (
        "double_difference_pseudorange_factors", "double_difference_carrier_factors",
        "single_difference_doppler_factors", "single_difference_tdcp_factors",
        "ambiguity_between_factors",
    ):
        if int(summary[key]) != 0:
            raise FloatCarrierError(f"forbidden factor class is nonzero: {key}")
    if int(summary["fixed_ambiguities"]) != 0 or summary["fixed_solution"]:
        raise FloatCarrierError("float lane produced a fixed ambiguity/solution")
    if require_converged and not summary["converged"]:
        raise FloatCarrierError("float lane did not converge within frozen bound")
    return {
        "epochs": int(summary["optimized_epochs"]),
        "valid_solutions": int(summary["valid_solutions"]),
        "carrier_phase_factors": int(summary["carrier_phase_factors"]),
        "ambiguity_states": int(summary["ambiguity_states"]),
        "converged": summary["converged"],
        "iterations": int(summary["iterations"]),
        "initial_cost": float(summary["initial_cost"]),
        "final_cost": float(summary["final_cost"]),
    }


def inspect_clock_discontinuities(observations_csv: Path) -> dict[str, Any]:
    """Inspect adapter clock counters truth-free and fail closed on bad rows."""

    if not observations_csv.is_file():
        raise FloatCarrierError(f"missing observations CSV: {observations_csv}")
    previous: int | None = None
    transitions = 0
    max_count = 0
    rows = 0
    with observations_csv.open(newline="", encoding="utf-8") as handle:
        reader = csv.DictReader(handle)
        if "HardwareClockDiscontinuityCount" not in (reader.fieldnames or []):
            raise FloatCarrierError("observations CSV lacks HardwareClockDiscontinuityCount")
        for row in reader:
            rows += 1
            token = row.get("HardwareClockDiscontinuityCount", "").strip()
            try:
                count = int(token)
            except (TypeError, ValueError) as exc:
                raise FloatCarrierError("clock discontinuity count is invalid") from exc
            if count < 0 or (previous is not None and count < previous):
                raise FloatCarrierError("clock discontinuity count is not monotonic")
            if previous is not None and count != previous:
                transitions += 1
            previous = count
            max_count = max(max_count, count)
    return {
        "rows": rows,
        "transition_rows": transitions,
        "max_counter": max_count,
        "candidate_safe": transitions == 0,
        "reason": "no hardware clock transition" if transitions == 0 else "clock transition requires arc retirement; native RINEX lane cannot encode the reset",
    }


def validate_cost_trace(path: Path, expected_max_iterations: int = 8) -> dict[str, Any]:
    if not path.is_file():
        raise FloatCarrierError(f"missing cost trace: {path}")
    with path.open(encoding="utf-8", newline="") as handle:
        reader = csv.DictReader(handle)
        required = {"global_iteration", "cost", "converged"}
        if not required.issubset(reader.fieldnames or []):
            raise FloatCarrierError("cost trace schema is incomplete")
        rows: list[dict[str, Any]] = []
        previous_iteration = -1
        previous_cost = math.inf
        converged_rows = 0
        for row in reader:
            try:
                iteration = int(row["global_iteration"])
                cost = float(row["cost"])
            except (TypeError, ValueError) as exc:
                raise FloatCarrierError("invalid cost trace number") from exc
            if iteration < 0 or iteration > expected_max_iterations or not math.isfinite(cost):
                raise FloatCarrierError("cost trace has invalid iteration/cost")
            if iteration <= previous_iteration:
                raise FloatCarrierError("cost trace iterations are not strictly increasing")
            monotonic = cost <= previous_cost + 1e-9 * max(1.0, abs(previous_cost))
            rows.append({"iteration": iteration, "cost": cost, "monotonic": monotonic})
            previous_iteration = iteration
            previous_cost = cost
            if str(row["converged"]).strip().lower() in {"1", "true", "yes"}:
                converged_rows += 1
        if not rows:
            raise FloatCarrierError("cost trace has no rows")
    return {
        "rows": len(rows),
        "initial_cost": rows[0]["cost"],
        "final_cost": rows[-1]["cost"],
        "converged_rows": converged_rows,
        "monotonic_non_increasing": all(row["monotonic"] for row in rows),
        "artifact": {"path": relative(path), "bytes": path.stat().st_size, "sha256": sha256(path)},
    }


def _limits() -> None:
    resource.setrlimit(resource.RLIMIT_AS, (MAX_ADDRESS_SPACE_BYTES, MAX_ADDRESS_SPACE_BYTES))
    resource.setrlimit(resource.RLIMIT_CPU, (MAX_RUNTIME_SECONDS, MAX_RUNTIME_SECONDS + 1))


def run_child(command: list[str], stage_dir: Path, label: str) -> dict[str, Any]:
    stage_dir.mkdir(parents=True, exist_ok=True)
    started = time.perf_counter()
    stdout_path = stage_dir / f"{label}.stdout.log"
    stderr_path = stage_dir / f"{label}.stderr.log"
    environment = os.environ.copy()
    environment["LD_LIBRARY_PATH"] = "/home/sasaki/.local/lib:/opt/ros/jazzy/lib" + (
        os.pathsep + environment["LD_LIBRARY_PATH"] if environment.get("LD_LIBRARY_PATH") else ""
    )
    with stdout_path.open("wb") as stdout, stderr_path.open("wb") as stderr:
        try:
            completed = subprocess.run(
                command,
                cwd=ROOT,
                env=environment,
                stdin=subprocess.DEVNULL,
                stdout=stdout,
                stderr=stderr,
                timeout=MAX_RUNTIME_SECONDS + 5,
                preexec_fn=_limits,
                check=False,
            )
            return_code: int | None = completed.returncode
            timed_out = False
        except subprocess.TimeoutExpired:
            return_code = None
            timed_out = True
    return {
        "command": command,
        "return_code": return_code,
        "timed_out": timed_out,
        "wall_seconds": time.perf_counter() - started,
        "child_max_rss_kib": resource.getrusage(resource.RUSAGE_CHILDREN).ru_maxrss,
        "stdout": relative(stdout_path),
        "stderr": relative(stderr_path),
    }


def artifact(path: Path) -> dict[str, Any]:
    if not path.is_file() or path.stat().st_size > MAX_OUTPUT_BYTES:
        raise FloatCarrierError(f"missing or oversized artifact: {path}")
    return {"path": relative(path), "bytes": path.stat().st_size, "sha256": sha256(path)}


def validate_output_position(path: Path, expected_rows: int) -> dict[str, Any]:
    if not path.is_file():
        raise FloatCarrierError(f"missing FGO position output: {path}")
    rows = []
    previous_time: float | None = None
    previous_ecef: tuple[float, float, float] | None = None
    jumps = 0
    with path.open(encoding="utf-8") as handle:
        for line in handle:
            fields = line.split()
            if len(fields) < 8:
                continue
            try:
                # libgnss++ solution text has GPS week/tow before the ECEF
                # coordinates; use all finite numeric tokens conservatively.
                nums = [float(token) for token in fields]
            except ValueError:
                continue
            if not all(math.isfinite(value) for value in nums):
                raise FloatCarrierError("FGO position output has non-finite values")
            rows.append(nums)
            if len(nums) >= 5:
                time_value = nums[1]
                ecef = tuple(nums[2:5])
                if previous_time is not None and time_value > previous_time and previous_ecef is not None:
                    dt = time_value - previous_time
                    speed = math.sqrt(sum((ecef[i] - previous_ecef[i]) ** 2 for i in range(3))) / dt
                    if speed > SPEED_BOUND_MPS:
                        jumps += 1
                previous_time = time_value
                previous_ecef = ecef
    if not rows:
        raise FloatCarrierError("FGO position output has no rows")
    if expected_rows > 0 and len(rows) != expected_rows:
        raise FloatCarrierError("FGO position/output row count mismatch")
    if jumps:
        raise FloatCarrierError(f"FGO output has {jumps} transitions above {SPEED_BOUND_MPS} m/s")
    return {"rows": len(rows), "above_speed_bound": jumps, "artifact": artifact(path)}


def run_structural_route(dataset_id: str, output_root: Path, *, max_epochs: int = 30) -> dict[str, Any]:
    """Run candidate only on an existing truth-free materialization."""

    verify_freeze()
    route_root = SMOKE_INPUT_ROOT / safe_id(dataset_id)
    inputs = route_root / "inputs"
    adapter = route_root / "adapter"
    for path in (inputs / "brdc.nav", adapter / "rover.obs", adapter / "observations.csv"):
        if not path.is_file():
            raise FloatCarrierError(f"structural smoke input missing: {path}")
    clock = inspect_clock_discontinuities(adapter / "observations.csv")
    if not clock["candidate_safe"]:
        raise FloatCarrierError("clock discontinuity requires an unrepresentable native arc reset")
    route_output = output_root / "structural_smoke" / safe_id(dataset_id)
    if route_output.exists():
        raise FloatCarrierError(f"refusing to overwrite structural output: {route_output}")
    temporary = Path(tempfile.mkdtemp(prefix=f".{safe_id(dataset_id)}.", dir=str(output_root / "structural_smoke")))
    try:
        spp_dir = temporary / "spp"
        spp = run_child(
            [
                str(SPP_BINARY), "--obs", str(adapter / "rover.obs"), "--nav", str(inputs / "brdc.nav"),
                "--out", str(spp_dir / "libgnsspp_spp.pos"),
                "--summary-json", str(spp_dir / "libgnsspp_spp_summary.json"),
                "--clock-csv", str(spp_dir / "libgnsspp_spp_clock.csv"),
                "--timing-csv", str(spp_dir / "libgnsspp_spp_timing.csv"), "--quiet",
            ],
            spp_dir,
            "spp",
        )
        if spp["return_code"] != 0 or spp["timed_out"]:
            raise FloatCarrierError("truth-free SPP smoke failed")
        candidate_dir = temporary / "candidate"
        command = float_carrier_fgo_command(
            adapter / "rover.obs",
            inputs / "brdc.nav",
            spp_dir / "libgnsspp_spp.pos",
            candidate_dir,
            max_epochs=max_epochs,
            skip_epochs=0,
        )
        validate_command(command, candidate=True)
        fgo = run_child(command, candidate_dir, "fgo")
        if fgo["return_code"] != 0 or fgo["timed_out"]:
            raise FloatCarrierError("float-carrier FGO smoke failed")
        summary = load_json(candidate_dir / "fgo_summary.json", "FGO summary")
        diagnostics = validate_float_carrier_summary(summary, require_converged=True)
        trace = validate_cost_trace(candidate_dir / "fgo_cost_trace.csv")
        if not trace["monotonic_non_increasing"] or trace["converged_rows"] <= 0:
            raise FloatCarrierError("float-carrier cost trace failed convergence/monotonicity gate")
        positions = validate_output_position(candidate_dir / "fgo.pos", diagnostics["valid_solutions"])
        result = {
            "dataset_id": dataset_id,
            "status": "passed",
            "truth_opened": False,
            "truth_materialized": False,
            "clock": clock,
            "run": fgo,
            "summary": summary,
            "diagnostics": diagnostics,
            "cost_trace": trace,
            "positions": positions,
            "artifacts": {name: artifact(candidate_dir / name) for name in (
                "fgo.pos", "fgo_summary.json", "fgo_epoch_debug.csv", "fgo_factor_debug.csv", "fgo_cost_trace.csv"
            )},
        }
        atomic_json(temporary / "route_manifest.json", result)
        route_manifest_hash = sha256(temporary / "route_manifest.json")
        atomic_bytes(temporary / "route_manifest.sha256", f"{route_manifest_hash}  route_manifest.json\n".encode("ascii"))
        os.replace(temporary, route_output)
        result["route_manifest_sha256"] = route_manifest_hash
        return result
    except Exception:
        # Keep failed temporary diagnostics available to the caller, but never
        # publish a partial route under its final content-addressed name.
        raise


def structural_smoke(output_root: Path, dataset_ids: tuple[str, ...]) -> dict[str, Any]:
    verify_freeze()
    (output_root / "structural_smoke").mkdir(parents=True, exist_ok=True)
    routes: list[dict[str, Any]] = []
    for dataset_id in dataset_ids:
        started = time.perf_counter()
        try:
            result = run_structural_route(dataset_id, output_root)
        except Exception as exc:  # structural failures are recorded fail-closed
            result = {
                "dataset_id": dataset_id,
                "status": "failed-closed",
                "truth_opened": False,
                "truth_materialized": False,
                "reason": str(exc),
            }
        result["wall_seconds"] = time.perf_counter() - started
        routes.append(result)
    report = {
        "schema_version": "smartphone-r5-native-fgo-float-carrier-structural-smoke.v1",
        "status": "passed" if all(r["status"] == "passed" for r in routes) else "no-go-structural",
        "candidate_id": "native-fgo-v1-float-carrier-no-base-v1",
        "truth_free": True,
        "truth_open_count": 0,
        "dataset_ids": list(dataset_ids),
        "routes": routes,
        "freeze_record": relative(FREEZE),
        "freeze_record_sha256": FREEZE_SHA256,
        "source_hashes": FGO_SOURCE_HASHES,
        "binary_hashes": {
            "build/apps/gnss_fgo": FGO_BINARY_SHA256,
            "build/apps/gnss_spp": SPP_BINARY_SHA256,
        },
        "fallback": "exact native-FGO v1 output is the only permitted route fallback",
        "validation_or_holdout_opened": False,
    }
    atomic_json(output_root / "structural_smoke_manifest.json", report)
    report["structural_smoke_manifest_sha256"] = sha256(output_root / "structural_smoke_manifest.json")
    atomic_json(output_root / "structural_smoke_manifest.seal.json", report)
    return report


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    subparsers = parser.add_subparsers(dest="command", required=True)
    smoke = subparsers.add_parser("structural-smoke", help="run only truth-free short-window smokes")
    smoke.add_argument("--output-root", type=Path, default=OUTPUT_ROOT)
    smoke.add_argument("--dataset-id", action="append", dest="dataset_ids")
    subparsers.add_parser("verify-freeze", help="verify hashes and print the frozen contract")
    subparsers.add_parser("train-score", help="refuse truth scoring while the strict split is blocked")
    return parser.parse_args()


def main() -> int:
    args = parse_args()
    try:
        if args.command == "verify-freeze":
            freeze = verify_freeze()
            print(json.dumps({"status": "verified", "candidate_id": freeze["candidate_id"], "freeze_record_sha256": FREEZE_SHA256}, sort_keys=True))
            return 0
        if args.command == "train-score":
            verify_freeze()
            print("float-carrier train-score is fail-closed: no strict unused three-identity split is available", file=sys.stderr)
            return 2
        dataset_ids = tuple(args.dataset_ids or STRUCTURAL_SMOKE_IDS)
        unknown = sorted(set(dataset_ids) - set(STRUCTURAL_SMOKE_IDS))
        if unknown:
            raise FloatCarrierError("structural smoke identity is not frozen: " + ", ".join(unknown))
        report = structural_smoke(args.output_root.resolve(), dataset_ids)
        print(json.dumps(report, indent=2, sort_keys=True))
        return 0 if report["status"] == "passed" else 1
    except FloatCarrierError as exc:
        print(f"float-carrier contract error: {exc}", file=sys.stderr)
        return 2


if __name__ == "__main__":
    raise SystemExit(main())
