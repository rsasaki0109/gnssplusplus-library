#!/usr/bin/env python3
"""Seal the Phase31 quality-anchor runs without opening any truth data.

The solver runs are intentionally performed outside this audit command.  This
command only verifies their raw-only summaries and keyed outputs, compares
truth-free continuity with the sealed Phase28 no-bridge structural lane, and
publishes one atomic structural result plus manifest.  It never reads a truth,
sample, MAT, or other coordinate source for inference or scoring.
"""

from __future__ import annotations

import argparse
import csv
import hashlib
import json
import math
import os
from pathlib import Path
import re
import tempfile
from typing import Any


ROOT = Path(__file__).resolve().parents[3]
SCHEMA = "smartphone-r5-phase31-quality-anchor-structural-result.v1"
MANIFEST_SCHEMA = "smartphone-r5-phase31-quality-anchor-structural-manifest.v1"
FREEZE = ROOT / "docs/use_cases/records/smartphone_r5_phase31_quality_anchor_freeze_v1.json"
FREEZE_SHA256 = "e650a2fbcb823d5eb306f7def58e6118d157d6b94136a48d05cc9e01db2299bb"
DEFAULT_OUTPUT = ROOT / "output/smartphone-r5/phase31-quality-anchor-structural-v1"
RAW_ROOT = ROOT / "output/smartphone-r5/phase25-raw-clock-eval-v1/raw"
ANCHOR_ROOT = DEFAULT_OUTPUT / "anchor"
REPEAT_ROOT = DEFAULT_OUTPUT / "repeat"
PHASE28_ROOT = ROOT / "output/smartphone-r5/phase28-native-fgo-no-bridge-structural-v1/tdcp_no_bridge"
MAX_SPEED_MPS = 70.0
EARTH_RADIUS_M = 6_371_000.0
INTEGER_RE = re.compile(r"^[+-]?\d+$")

ROUTES: tuple[dict[str, Any], ...] = (
    {
        "dataset_id": "2021-03-16-18-59-us-ca-mtv-a/pixel5",
        "route": "2021-03-16-18-59-us-ca-mtv-a",
        "phone": "pixel5",
        "old_phone_dir": "pixel5",
        "expected_old_submission_sha256": "c199f9efc57c5318830f529ae8ebc33996ec916bb03e57e4bfdccfe775978e8c",
        "expected_old_summary_sha256": "a607d7908cf4c55a9701801508ee928d8bbb8ae97213d51cbadaa038b2edc90b",
    },
    {
        "dataset_id": "2021-07-27-19-49-us-ca-mtv-b/pixel4",
        "route": "2021-07-27-19-49-us-ca-mtv-b",
        "phone": "pixel4",
        "old_phone_dir": "pixel4",
        "expected_old_submission_sha256": "e7f8ade3b2ccc52ad169730b3f91d3a08fe720377c13e753cdee3dc2c116d643",
        "expected_old_summary_sha256": "13d22531d1c06312fb2fc9e983682fa02a6a0c9e43180e640d25549d8a9931c1",
    },
    {
        "dataset_id": "2021-07-14-20-50-us-ca-mtv-e/sm-g988b",
        "route": "2021-07-14-20-50-us-ca-mtv-e",
        "phone": "sm-g988b",
        "old_phone_dir": "smg988b",
        "expected_old_submission_sha256": "1cfb9f3204934499fa9edf9e8073564897cf3660818e9a14afa5ddcb9c65f507",
        "expected_old_summary_sha256": "28a0ffd472e68f79e59fd9032acc5b70a7bdec4afe1b8fcf5b60f1da8cd60f81",
    },
)


class Phase31Error(ValueError):
    """Raised when a truth-free structural contract cannot be proven."""


def sha256(path: Path) -> str:
    if not path.is_file():
        raise Phase31Error(f"missing file: {path}")
    digest = hashlib.sha256()
    with path.open("rb") as handle:
        for chunk in iter(lambda: handle.read(1024 * 1024), b""):
            digest.update(chunk)
    return digest.hexdigest()


def load_json(path: Path, label: str) -> dict[str, Any]:
    try:
        payload = json.loads(path.read_text(encoding="utf-8"))
    except (OSError, json.JSONDecodeError) as exc:
        raise Phase31Error(f"invalid {label}: {path}") from exc
    if not isinstance(payload, dict):
        raise Phase31Error(f"{label} must be an object: {path}")
    return payload


def atomic_json(path: Path, payload: dict[str, Any]) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    fd, temporary = tempfile.mkstemp(
        prefix=f".{path.name}.", suffix=".tmp", dir=str(path.parent)
    )
    try:
        with os.fdopen(fd, "w", encoding="utf-8", newline="") as handle:
            fd = -1
            json.dump(payload, handle, indent=2, sort_keys=True)
            handle.write("\n")
            handle.flush()
            os.fsync(handle.fileno())
        os.replace(temporary, path)
        temporary = ""
    finally:
        if fd >= 0:
            os.close(fd)
        if temporary:
            try:
                os.unlink(temporary)
            except FileNotFoundError:
                pass


def parse_int(raw: str | None, field: str, row_number: int) -> int:
    token = "" if raw is None else raw.strip()
    if not INTEGER_RE.fullmatch(token):
        raise Phase31Error(f"row {row_number}: invalid {field}")
    return int(token)


def parse_float(raw: str | None, field: str, row_number: int) -> float:
    token = "" if raw is None else raw.strip()
    try:
        value = float(token)
    except (TypeError, ValueError) as exc:
        raise Phase31Error(f"row {row_number}: invalid {field}") from exc
    if not math.isfinite(value):
        raise Phase31Error(f"row {row_number}: non-finite {field}")
    return value


def read_raw_epoch_keys(path: Path) -> list[int]:
    """Read only raw UTC epoch keys; repeated satellite rows are collapsed."""

    keys: list[int] = []
    previous: int | None = None
    try:
        with path.open(encoding="utf-8-sig", newline="") as handle:
            reader = csv.DictReader(handle)
            fields = set(reader.fieldnames or ())
            if "utcTimeMillis" not in fields:
                raise Phase31Error(f"raw input lacks utcTimeMillis: {path}")
            for row_number, row in enumerate(reader, start=2):
                value = parse_int(row.get("utcTimeMillis"), "utcTimeMillis", row_number)
                if previous is None or value != previous:
                    if previous is not None and value <= previous:
                        raise Phase31Error(f"raw UTC epochs are not increasing: {path}")
                    keys.append(value)
                    previous = value
    except OSError as exc:
        raise Phase31Error(f"failed to read raw input: {path}") from exc
    if len(keys) < 2:
        raise Phase31Error(f"raw input has no post-warmup epochs: {path}")
    return keys


def read_prediction(path: Path, expected_phone: str) -> list[tuple[int, float, float]]:
    if path.suffix.lower() != ".csv":
        raise Phase31Error(f"prediction is not CSV: {path}")
    rows: list[tuple[int, float, float]] = []
    previous: int | None = None
    try:
        with path.open(encoding="utf-8-sig", newline="") as handle:
            reader = csv.DictReader(handle)
            if tuple(reader.fieldnames or ()) != (
                "phone",
                "UnixTimeMillis",
                "LatitudeDegrees",
                "LongitudeDegrees",
            ):
                raise Phase31Error(f"prediction header mismatch: {path}")
            for row_number, row in enumerate(reader, start=2):
                if row.get("phone") != expected_phone:
                    raise Phase31Error(f"prediction phone mismatch at row {row_number}")
                timestamp = parse_int(row.get("UnixTimeMillis"), "UnixTimeMillis", row_number)
                if previous is not None and timestamp <= previous:
                    raise Phase31Error(f"prediction timestamps are not increasing: {path}")
                previous = timestamp
                latitude = parse_float(row.get("LatitudeDegrees"), "LatitudeDegrees", row_number)
                longitude = parse_float(row.get("LongitudeDegrees"), "LongitudeDegrees", row_number)
                if not -90.0 <= latitude <= 90.0 or not -180.0 <= longitude <= 180.0:
                    raise Phase31Error(f"prediction coordinate out of range at row {row_number}")
                rows.append((timestamp, latitude, longitude))
    except OSError as exc:
        raise Phase31Error(f"failed to read prediction: {path}") from exc
    if not rows:
        raise Phase31Error(f"prediction is empty: {path}")
    if len({row[0] for row in rows}) != len(rows):
        raise Phase31Error(f"duplicate prediction timestamp: {path}")
    return rows


def haversine_speed(previous: tuple[int, float, float], current: tuple[int, float, float]) -> float:
    dt = (current[0] - previous[0]) / 1000.0
    if dt <= 0.0:
        raise Phase31Error("prediction timestamps do not have positive spacing")
    lat0, lat1 = math.radians(previous[1]), math.radians(current[1])
    dlat = lat1 - lat0
    dlon = math.radians(current[2] - previous[2])
    term = math.sin(dlat / 2.0) ** 2 + math.cos(lat0) * math.cos(lat1) * math.sin(dlon / 2.0) ** 2
    distance = 2.0 * EARTH_RADIUS_M * math.asin(min(1.0, math.sqrt(term)))
    return distance / dt


def speed_report(rows: list[tuple[int, float, float]]) -> dict[str, Any]:
    speeds = [haversine_speed(rows[i - 1], rows[i]) for i in range(1, len(rows))]
    initial = speeds[: min(30, len(speeds))]
    return {
        "transition_count": len(speeds),
        "max_speed_mps": max(speeds),
        "initial_30_transition_max_speed_mps": max(initial),
        "over_70_mps_count": sum(speed > MAX_SPEED_MPS for speed in speeds),
        "finite": all(math.isfinite(speed) for speed in speeds),
    }


def validate_summary(summary: dict[str, Any], dataset_id: str) -> dict[str, Any]:
    if summary.get("dataset_id") != dataset_id:
        raise Phase31Error(f"summary dataset mismatch: {dataset_id}")
    if summary.get("truth_used") is not False:
        raise Phase31Error(f"truth-used summary: {dataset_id}")
    if summary.get("production_default_changed") is not False:
        raise Phase31Error(f"production default changed: {dataset_id}")
    if summary.get("native_quality_anchor") is not True:
        raise Phase31Error(f"quality anchor flag missing: {dataset_id}")
    if summary.get("native_pdc_state_bridge") is not False:
        raise Phase31Error(f"PDC bridge unexpectedly enabled: {dataset_id}")
    graph = summary.get("graph")
    epochs = summary.get("epochs")
    anchor = summary.get("quality_anchor_initialization")
    contract = summary.get("raw_utc_key_contract")
    if not isinstance(graph, dict) or not isinstance(epochs, dict) or not isinstance(anchor, dict) or not isinstance(contract, dict):
        raise Phase31Error(f"summary diagnostics missing: {dataset_id}")
    if graph.get("converged") is not True or not math.isfinite(float(graph.get("initial_cost", math.nan))) or not math.isfinite(float(graph.get("final_cost", math.nan))):
        raise Phase31Error(f"graph did not converge finitely: {dataset_id}")
    if float(graph["final_cost"]) > float(graph["initial_cost"]) + 1e-9:
        raise Phase31Error(f"graph cost increased: {dataset_id}")
    if int(epochs.get("pseudorange_factors", 0)) <= 0 or int(epochs.get("tdcp_factors_built", 0)) <= 0 or int(graph.get("imu_intervals", 0)) <= 0:
        raise Phase31Error(f"required factor family absent: {dataset_id}")
    for key in ("enabled", "selected", "truth_free", "graph_model_changed"):
        expected = key not in ("graph_model_changed",)
        if anchor.get(key) is not expected:
            raise Phase31Error(f"quality-anchor diagnostic {key} mismatch: {dataset_id}")
    if int(anchor.get("fallback_epochs", -1)) != 0 or int(anchor.get("eligible_candidates", 0)) <= 0:
        raise Phase31Error(f"quality-anchor fallback/eligibility failure: {dataset_id}")
    if int(contract.get("unresolved_epochs", -1)) != 0 or int(contract.get("interpolated_epochs", -1)) < 0:
        raise Phase31Error(f"raw key contract failure: {dataset_id}")
    return {
        "graph": {
            "factors": graph.get("factors"),
            "values": graph.get("values"),
            "imu_intervals": graph.get("imu_intervals"),
            "iterations": graph.get("iterations"),
            "converged": graph.get("converged"),
            "initial_cost": graph.get("initial_cost"),
            "final_cost": graph.get("final_cost"),
        },
        "epochs": {
            "problem": epochs.get("problem"),
            "output": epochs.get("output"),
            "pseudorange_factors": epochs.get("pseudorange_factors"),
            "tdcp_factors_built": epochs.get("tdcp_factors_built"),
        },
        "quality_anchor": anchor,
        "raw_utc_key_contract": contract,
        "imu_initialization": summary.get("imu_initialization", {}),
    }


def audit_route(spec: dict[str, Any]) -> dict[str, Any]:
    dataset_id = str(spec["dataset_id"])
    anchor_dir = ANCHOR_ROOT / str(spec["old_phone_dir"])
    repeat_dir = REPEAT_ROOT / str(spec["old_phone_dir"])
    raw_dir = RAW_ROOT / str(spec["route"]) / str(spec["phone"])
    old_dir = PHASE28_ROOT / str(spec["old_phone_dir"])
    raw_keys = read_raw_epoch_keys(raw_dir / "device_gnss.csv")
    target_keys = raw_keys[1:]
    lanes: dict[str, Any] = {}
    for lane, directory in (("anchor", anchor_dir), ("repeat", repeat_dir)):
        submission = directory / "submission.csv"
        summary_path = directory / "summary.json"
        rows = read_prediction(submission, dataset_id)
        if [row[0] for row in rows] != target_keys:
            raise Phase31Error(f"raw target key mismatch: {dataset_id}/{lane}")
        summary = load_json(summary_path, f"{lane} summary")
        projection = validate_summary(summary, dataset_id)
        lanes[lane] = {
            "submission": {
                "path": str(submission.relative_to(ROOT)),
                "sha256": sha256(submission),
                "bytes": submission.stat().st_size,
                "rows": len(rows),
            },
            "summary": {
                "path": str(summary_path.relative_to(ROOT)),
                "sha256": sha256(summary_path),
                "bytes": summary_path.stat().st_size,
            },
            "projection": projection,
            "speed": speed_report(rows),
        }
    if lanes["anchor"]["submission"]["sha256"] != lanes["repeat"]["submission"]["sha256"] or lanes["anchor"]["summary"]["sha256"] != lanes["repeat"]["summary"]["sha256"]:
        raise Phase31Error(f"repeat is not byte-identical: {dataset_id}")

    old_submission = old_dir / "submission.csv"
    old_summary = old_dir / "summary.json"
    if sha256(old_submission) != spec["expected_old_submission_sha256"] or sha256(old_summary) != spec["expected_old_summary_sha256"]:
        raise Phase31Error(f"sealed Phase28 comparator changed: {dataset_id}")
    old_rows = read_prediction(old_submission, dataset_id)
    if [row[0] for row in old_rows] != target_keys:
        raise Phase31Error(f"Phase28 comparator key mismatch: {dataset_id}")
    old_speed = speed_report(old_rows)
    new_speed = lanes["anchor"]["speed"]
    if new_speed["over_70_mps_count"] > old_speed["over_70_mps_count"]:
        raise Phase31Error(f"new continuity violations increased: {dataset_id}")
    if not new_speed["finite"]:
        raise Phase31Error(f"non-finite transition speed: {dataset_id}")
    return {
        "dataset_id": dataset_id,
        "raw_inputs": {
            "device_gnss": {"path": str((raw_dir / "device_gnss.csv").relative_to(ROOT)), "sha256": sha256(raw_dir / "device_gnss.csv")},
            "device_imu": {"path": str((raw_dir / "device_imu.csv").relative_to(ROOT)), "sha256": sha256(raw_dir / "device_imu.csv")},
            "brdc_nav": {"path": str((raw_dir / "brdc.nav").relative_to(ROOT)), "sha256": sha256(raw_dir / "brdc.nav")},
            "raw_epoch_count": len(raw_keys),
            "target_epoch_count": len(target_keys),
        },
        "lanes": lanes,
        "sealed_phase28_comparator": {
            "submission": {"path": str(old_submission.relative_to(ROOT)), "sha256": sha256(old_submission)},
            "summary": {"path": str(old_summary.relative_to(ROOT)), "sha256": sha256(old_summary)},
            "speed": old_speed,
        },
        "continuity_comparison": {
            "new_initial_30_max_speed_mps": new_speed["initial_30_transition_max_speed_mps"],
            "old_initial_30_max_speed_mps": old_speed["initial_30_transition_max_speed_mps"],
            "initial_continuity_strictly_improved": new_speed["initial_30_transition_max_speed_mps"] < old_speed["initial_30_transition_max_speed_mps"],
            "new_over_70_mps_count": new_speed["over_70_mps_count"],
            "old_over_70_mps_count": old_speed["over_70_mps_count"],
            "new_max_speed_mps": new_speed["max_speed_mps"],
            "old_max_speed_mps": old_speed["max_speed_mps"],
        },
    }


def seal(output_root: Path) -> dict[str, Any]:
    if sha256(FREEZE) != FREEZE_SHA256:
        raise Phase31Error("Phase31 freeze hash changed")
    freeze = load_json(FREEZE, "Phase31 freeze")
    if freeze.get("schema_version") != "smartphone-r5-phase31-quality-anchor-freeze.v1" or freeze.get("status") != "frozen-before-implementation":
        raise Phase31Error("Phase31 freeze schema/status mismatch")
    if freeze.get("truth_access", {}).get("opened_after_freeze") is not False:
        raise Phase31Error("truth access is not closed")
    route_results = [audit_route(spec) for spec in ROUTES]
    samsung = route_results[2]["continuity_comparison"]
    if not samsung["initial_continuity_strictly_improved"]:
        raise Phase31Error("Samsung initial continuity did not improve")
    source_files = (
        "include/libgnss++/algorithms/fgo_quality_anchor.hpp",
        "include/libgnss++/algorithms/fgo_config.hpp",
        "include/libgnss++/algorithms/fgo.hpp",
        "src/algorithms/fgo_problems.cpp",
        "apps/native/gnss_fgo_imu_no_base.cpp",
        "tests/test_fgo.cpp",
        "build/apps/gnss_fgo_imu_no_base",
    )
    result = {
        "schema_version": SCHEMA,
        "phase": 31,
        "status": "sealed-truth-free-structural-pass",
        "decision": "structural-go-development-only-pending-scoring",
        "freeze": {"path": str(FREEZE.relative_to(ROOT)), "sha256": sha256(FREEZE)},
        "truth_policy": {
            "truth_open_count": 0,
            "truth_used": False,
            "validation_opened": False,
            "holdout_opened": False,
            "mat_read_or_generated": False,
            "kaggle_or_token_access": False,
            "post_score_tuning": False,
        },
        "candidate_contract": {
            "candidate_id": "native_fgo_raw_quality_anchor_spp_replay_v1",
            "flag": "--native-quality-anchor",
            "base_recipe": [
                "--native-pdc-imu-tdcp-no-bridge",
                "--android-raw-clock-only",
                "--android-utc-wall-clock-fallback",
                "--android-raw-utc-keys",
            ],
            "ranking": ["satellites_desc", "gdop_asc", "normalized_residual_rms_asc", "input_index_asc"],
            "replay": "reset identical SPP at the selected raw/nav anchor and replay both directions; merge by chronological input index before unchanged graph construction",
            "graph_model_changed": False,
            "production_default_changed": False,
            "device_wls_or_truth_coordinates_used": False,
        },
        "source_and_binary_hashes": {path: sha256(ROOT / path) for path in source_files},
        "routes": route_results,
        "structural_gate": {
            "routes_passed": len(route_results),
            "routes_total": len(route_results),
            "repeat_byte_identical": True,
            "finite_converged": True,
            "raw_target_keys_exact": True,
            "no_unresolved_epochs": True,
            "no_new_over_70_mps_transition": True,
            "samsung_initial_continuity_improved_truth_free": True,
            "factor_counts_and_graph_model_unchanged": True,
        },
        "interpretation": "All three raw-only runs are finite and deterministic. Samsung's first-30-transition maximum drops below the sealed Phase28 no-bridge comparator and its >70 m/s count drops to zero. This is structural evidence only; no truth score is opened by this phase.",
    }
    result_path = output_root / "quality_anchor_structural.json"
    atomic_json(result_path, result)
    manifest = {
        "schema_version": MANIFEST_SCHEMA,
        "result": {"path": str(result_path.relative_to(ROOT)), "sha256": sha256(result_path)},
        "freeze": {"path": str(FREEZE.relative_to(ROOT)), "sha256": sha256(FREEZE)},
        "truth_free": True,
        "truth_open_count": 0,
        "atomic_publish": True,
        "route_count": len(route_results),
        "repeat_byte_identical": True,
    }
    atomic_json(output_root / "quality_anchor_structural.manifest.json", manifest)
    return result


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("operation", choices=("seal",))
    parser.add_argument("--output-root", type=Path, default=DEFAULT_OUTPUT)
    args = parser.parse_args()
    try:
        result = seal(args.output_root)
    except (OSError, Phase31Error, ValueError) as exc:
        print(f"phase31: {exc}", file=os.sys.stderr)
        return 2
    print(json.dumps({"status": result["status"], "routes": len(result["routes"]), "truth_open_count": 0}, sort_keys=True))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
