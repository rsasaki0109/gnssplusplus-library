#!/usr/bin/env python3
"""Build one versioned CLASLIB/native parity scorecard from a run manifest."""

from __future__ import annotations

import argparse
import hashlib
import json
import math
import sys
from collections import Counter
from pathlib import Path
from typing import Any, Mapping, Sequence


SCRIPT_DIR = Path(__file__).resolve().parent
if str(SCRIPT_DIR) not in sys.path:
    sys.path.insert(0, str(SCRIPT_DIR))

import madoca_solution_diff as solution_diff  # noqa: E402


SCHEMA = "claslib_parity_scorecard.v1"
MANIFEST_SCHEMA = "claslib_parity_manifest.v1"
DEFAULT_THRESHOLDS: dict[str, float] = {
    "dataset_epoch_coverage_min": 1.0,
    "solution_match_coverage_min": 1.0,
    "status_agreement_min": 0.99,
    "native_only_fix_max": 0.0,
    "fix_count_relative_delta_max": 0.01,
    "float_delta_rms_3d_m_max": 0.02,
    "trajectory_delta_rms_3d_m_max": 0.02,
    "trajectory_delta_p95_3d_m_max": 0.03,
    "zd_row_key_coverage_min": 1.0,
    "zd_prc_rms_delta_m_max": 0.01,
    "zd_cpc_rms_delta_m_max": 0.01,
}


def _require_mapping(value: Any, label: str) -> Mapping[str, Any]:
    if not isinstance(value, Mapping):
        raise ValueError(f"{label} must be an object")
    return value


def _require_text(value: Any, label: str) -> str:
    if not isinstance(value, str) or not value.strip():
        raise ValueError(f"{label} must be a non-empty string")
    return value.strip()


def _resolve_path(root: Path, value: Any, label: str) -> Path:
    path = Path(_require_text(value, label)).expanduser()
    return path if path.is_absolute() else root / path


def _sha256(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as handle:
        for chunk in iter(lambda: handle.read(1024 * 1024), b""):
            digest.update(chunk)
    return digest.hexdigest()


def _artifact(path: Path) -> dict[str, Any]:
    if not path.is_file():
        raise ValueError(f"required artifact does not exist: {path}")
    return {
        "path": str(path.resolve()),
        "bytes": path.stat().st_size,
        "sha256": _sha256(path),
    }


def _status_set(section: Mapping[str, Any], key: str) -> set[int]:
    values = section.get(key, [1])
    if not isinstance(values, list) or not values:
        raise ValueError(f"solution.{key} must be a non-empty integer array")
    try:
        return {int(value) for value in values}
    except (TypeError, ValueError) as exc:
        raise ValueError(f"solution.{key} must be a non-empty integer array") from exc


def _fix_source(section: Mapping[str, Any], side: str) -> str:
    source = str(section.get(f"{side}_fix_source", "status"))
    if source not in {"status", "fixed_ambiguities"}:
        raise ValueError(
            f"solution.{side}_fix_source must be status or fixed_ambiguities"
        )
    return source


def _is_fix(epoch: solution_diff.SolutionEpoch, statuses: set[int], source: str) -> bool:
    if source == "fixed_ambiguities":
        return epoch.fixed_ambiguities > 0
    return epoch.status in statuses


def _warmup_pairs(
    pairs: Sequence[solution_diff.MatchedPair], warmup_seconds: float
) -> list[solution_diff.MatchedPair]:
    if not pairs:
        return []
    first_absolute_s = pairs[0].base.week * solution_diff.GPS_WEEK_SECONDS + pairs[0].base.tow
    return [
        pair
        for pair in pairs
        if pair.base.week * solution_diff.GPS_WEEK_SECONDS + pair.base.tow
        >= first_absolute_s + warmup_seconds
    ]


def _delta_metrics(pairs: Sequence[solution_diff.MatchedPair]) -> dict[str, Any]:
    values = [solution_diff.norm_3d(*pair.delta_enu_m) for pair in pairs]
    horizontal = [solution_diff.norm_2d(pair.delta_enu_m[0], pair.delta_enu_m[1]) for pair in pairs]
    vertical = [abs(pair.delta_enu_m[2]) for pair in pairs]
    return {
        "epochs": len(pairs),
        "rms_3d_m": solution_diff.rms(values),
        "p95_3d_m": solution_diff.percentile(values, 95.0),
        "max_3d_m": max(values, default=0.0),
        "rms_horizontal_m": solution_diff.rms(horizontal),
        "p95_horizontal_m": solution_diff.percentile(horizontal, 95.0),
        "rms_vertical_m": solution_diff.rms(vertical),
    }


def _solution_layer(
    report: Mapping[str, Any],
    pairs: Sequence[solution_diff.MatchedPair],
    oracle_fix: set[int],
    candidate_fix: set[int],
    oracle_fix_source: str,
    candidate_fix_source: str,
    warmup_seconds: float,
) -> dict[str, Any]:
    warm = _warmup_pairs(pairs, warmup_seconds)
    oracle_rows = int(_require_mapping(report["base"], "solution base")["rows"])
    candidate_rows = int(_require_mapping(report["candidate"], "solution candidate")["rows"])
    union_rows = len(pairs) + int(report["base_only_epochs"]) + int(report["candidate_only_epochs"])
    raw_matrix = Counter((pair.base.status, pair.candidate.status) for pair in warm)
    semantic_matrix = Counter(
        (
            "fix" if _is_fix(pair.base, oracle_fix, oracle_fix_source) else "float",
            "fix" if _is_fix(pair.candidate, candidate_fix, candidate_fix_source) else "float",
        )
        for pair in warm
    )
    agreements = sum(
        1
        for pair in warm
        if _is_fix(pair.base, oracle_fix, oracle_fix_source)
        == _is_fix(pair.candidate, candidate_fix, candidate_fix_source)
    )
    oracle_fixed = [
        pair for pair in warm if _is_fix(pair.base, oracle_fix, oracle_fix_source)
    ]
    candidate_fixed = [
        pair for pair in warm if _is_fix(pair.candidate, candidate_fix, candidate_fix_source)
    ]
    mutually_fixed = [
        pair
        for pair in warm
        if _is_fix(pair.base, oracle_fix, oracle_fix_source)
        and _is_fix(pair.candidate, candidate_fix, candidate_fix_source)
    ]
    mutually_float = [
        pair
        for pair in warm
        if not _is_fix(pair.base, oracle_fix, oracle_fix_source)
        and not _is_fix(pair.candidate, candidate_fix, candidate_fix_source)
    ]
    native_only_fix = sum(
        1
        for pair in warm
        if not _is_fix(pair.base, oracle_fix, oracle_fix_source)
        and _is_fix(pair.candidate, candidate_fix, candidate_fix_source)
    )
    oracle_fix_count = len(oracle_fixed)
    candidate_fix_count = len(candidate_fixed)
    return {
        "oracle_rows": oracle_rows,
        "candidate_rows": candidate_rows,
        "matched_epochs": len(pairs),
        "union_epochs": union_rows,
        "match_coverage": len(pairs) / union_rows if union_rows else 0.0,
        "oracle_only_epochs": int(report["base_only_epochs"]),
        "candidate_only_epochs": int(report["candidate_only_epochs"]),
        "warmup_seconds": warmup_seconds,
        "evaluated_epochs": len(warm),
        "status_agreement_epochs": agreements,
        "status_agreement_ratio": agreements / len(warm) if warm else 0.0,
        "fix_sources": {
            "oracle": oracle_fix_source,
            "candidate": candidate_fix_source,
        },
        "status_matrix": [
            {"oracle_state": base, "candidate_state": candidate, "epochs": count}
            for (base, candidate), count in sorted(semantic_matrix.items())
        ],
        "raw_status_matrix": [
            {"oracle_status": base, "candidate_status": candidate, "epochs": count}
            for (base, candidate), count in sorted(raw_matrix.items())
        ],
        "oracle_fix_epochs": oracle_fix_count,
        "candidate_fix_epochs": candidate_fix_count,
        "native_only_fix_epochs": native_only_fix,
        "fix_count_relative_delta": abs(candidate_fix_count - oracle_fix_count)
        / max(oracle_fix_count, 1),
        "delta": {
            "full": _delta_metrics(pairs),
            "post_warmup": _delta_metrics(warm),
            # Backward-compatible alias retained for existing v1 consumers.
            "all": _delta_metrics(warm),
            "oracle_fixed": _delta_metrics(oracle_fixed),
            "mutually_fixed": _delta_metrics(mutually_fixed),
            "mutually_float": _delta_metrics(mutually_float),
        },
    }


def _load_json(path: Path, label: str) -> Mapping[str, Any]:
    try:
        payload = json.loads(path.read_text(encoding="utf-8"))
    except (OSError, json.JSONDecodeError) as exc:
        raise ValueError(f"cannot read {label} JSON {path}: {exc}") from exc
    return _require_mapping(payload, label)


def _zd_layer(path: Path) -> dict[str, Any]:
    payload = _load_json(path, "ZD diff")
    if payload.get("schema") != "clas_zd_component_diff.v1":
        raise ValueError(f"{path}: expected schema clas_zd_component_diff.v1")
    common = int(payload.get("common_rows", 0))
    base_only = int(payload.get("base_only_rows", 0))
    candidate_only = int(payload.get("candidate_only_rows", 0))
    union = common + base_only + candidate_only
    components: dict[str, Any] = {}
    per_component = payload.get("per_component", [])
    if not isinstance(per_component, list):
        raise ValueError(f"{path}: per_component must be an array")
    for item in per_component:
        if isinstance(item, Mapping) and isinstance(item.get("component"), str):
            components[str(item["component"])] = dict(item)
    return {
        "artifact": _artifact(path),
        "common_rows": common,
        "union_rows": union,
        "oracle_only_rows": base_only,
        "candidate_only_rows": candidate_only,
        "row_key_coverage": common / union if union else 0.0,
        "components": components,
    }


def _zd_group(paths: Mapping[str, Path]) -> dict[str, Any]:
    """Combine code/phase shards without pretending their row keys are one set."""
    shards = {name: _zd_layer(path) for name, path in sorted(paths.items())}
    components: dict[str, Any] = {}
    for shard_name, shard in shards.items():
        for component, stats in _require_mapping(
            shard["components"], f"ZD shard {shard_name}.components"
        ).items():
            if component in components:
                raise ValueError(
                    f"ZD component {component} appears in more than one shard"
                )
            components[component] = {**dict(stats), "shard": shard_name}
    return {
        "shards": shards,
        # Code and phase exports describe parallel surfaces.  Their minimum
        # coverage is the conservative group coverage; summing would double
        # count the same observation identities.
        "row_key_coverage": min(
            (float(shard["row_key_coverage"]) for shard in shards.values()),
            default=0.0,
        ),
        "components": components,
    }


def _zd_layers(root: Path, layers: Mapping[str, Any]) -> tuple[dict[str, Any], bool]:
    single = layers.get("zd_diff_json")
    multiple = layers.get("zd_diff_jsons")
    if single is not None and multiple is not None:
        raise ValueError("layers must use only one of zd_diff_json or zd_diff_jsons")
    if single is not None:
        return _zd_layer(_resolve_path(root, single, "layers.zd_diff_json")), False
    groups = _require_mapping(multiple, "layers.zd_diff_jsons")
    if not groups:
        raise ValueError("layers.zd_diff_jsons must not be empty")
    systems: dict[str, Any] = {}
    for raw_name, raw_shards in sorted(groups.items()):
        name = _require_text(raw_name, "layers.zd_diff_jsons key")
        if isinstance(raw_shards, str):
            paths = {
                "combined": _resolve_path(
                    root, raw_shards, f"layers.zd_diff_jsons.{name}"
                )
            }
        else:
            shard_values = _require_mapping(
                raw_shards, f"layers.zd_diff_jsons.{name}"
            )
            if not shard_values:
                raise ValueError(f"layers.zd_diff_jsons.{name} must not be empty")
            paths = {
                _require_text(shard_name, f"layers.zd_diff_jsons.{name} key"):
                _resolve_path(
                    root,
                    shard_path,
                    f"layers.zd_diff_jsons.{name}.{shard_name}",
                )
                for shard_name, shard_path in shard_values.items()
            }
        systems[name] = _zd_group(paths)
    return {
        "systems": systems,
        "row_key_coverage": min(
            (float(system["row_key_coverage"]) for system in systems.values()),
            default=0.0,
        ),
    }, True


def _thresholds(manifest: Mapping[str, Any]) -> dict[str, float]:
    configured = manifest.get("thresholds", {})
    configured = _require_mapping(configured, "thresholds")
    unknown = sorted(set(configured) - set(DEFAULT_THRESHOLDS))
    if unknown:
        raise ValueError(f"unknown thresholds: {', '.join(unknown)}")
    result = dict(DEFAULT_THRESHOLDS)
    for key, value in configured.items():
        try:
            numeric = float(value)
        except (TypeError, ValueError) as exc:
            raise ValueError(f"thresholds.{key} must be numeric") from exc
        if not math.isfinite(numeric) or numeric < 0.0:
            raise ValueError(f"thresholds.{key} must be finite and non-negative")
        result[key] = numeric
    return result


def _gate(name: str, actual: float, operator: str, expected: float) -> dict[str, Any]:
    passed = actual >= expected if operator == ">=" else actual <= expected
    return {
        "name": name,
        "actual": actual,
        "operator": operator,
        "expected": expected,
        "status": "passed" if passed else "failed",
    }


def _evaluate_gates(
    solution: Mapping[str, Any],
    zd: Mapping[str, Any],
    thresholds: Mapping[str, float],
    expected_epochs: int,
) -> list[dict[str, Any]]:
    delta = _require_mapping(solution["delta"], "solution.delta")
    full_delta = _require_mapping(delta["full"], "solution.delta.full")
    float_delta = _require_mapping(delta["mutually_float"], "solution.delta.mutually_float")
    gates = [
        _gate(
            "dataset_epoch_coverage",
            min(float(solution["oracle_rows"]), float(solution["candidate_rows"]))
            / expected_epochs,
            ">=",
            thresholds["dataset_epoch_coverage_min"],
        ),
        _gate("solution_match_coverage", float(solution["match_coverage"]), ">=", thresholds["solution_match_coverage_min"]),
        _gate("status_agreement", float(solution["status_agreement_ratio"]), ">=", thresholds["status_agreement_min"]),
        _gate("native_only_fix", float(solution["native_only_fix_epochs"]), "<=", thresholds["native_only_fix_max"]),
        _gate("fix_count_relative_delta", float(solution["fix_count_relative_delta"]), "<=", thresholds["fix_count_relative_delta_max"]),
        _gate("trajectory_delta_rms_3d_m", float(full_delta["rms_3d_m"]), "<=", thresholds["trajectory_delta_rms_3d_m_max"]),
        _gate("trajectory_delta_p95_3d_m", float(full_delta["p95_3d_m"]), "<=", thresholds["trajectory_delta_p95_3d_m_max"]),
    ]
    if int(float_delta["epochs"]) <= 0:
        gates.append(
            {
                "name": "float_delta_rms_3d_m",
                "actual": 0.0,
                "operator": "<=",
                "expected": thresholds["float_delta_rms_3d_m_max"],
                "status": "passed",
                "reason": "not applicable: no mutually FLOAT epochs remain after warmup",
            }
        )
    else:
        gates.append(
            _gate(
                "float_delta_rms_3d_m",
                float(float_delta["rms_3d_m"]),
                "<=",
                thresholds["float_delta_rms_3d_m_max"],
            )
        )
    systems = zd.get("systems")
    zd_groups = (
        _require_mapping(systems, "zd.systems")
        if systems is not None
        else {"": zd}
    )
    for system_name, group_value in zd_groups.items():
        group = _require_mapping(group_value, f"zd.systems.{system_name}")
        prefix = f"zd_{system_name}_" if system_name else "zd_"
        gates.append(
            _gate(
                f"{prefix}row_key_coverage",
                float(group["row_key_coverage"]),
                ">=",
                thresholds["zd_row_key_coverage_min"],
            )
        )
        components = _require_mapping(group["components"], f"{prefix}components")
        for component, threshold_name, suffix in (
            ("prc_m", "zd_prc_rms_delta_m_max", "prc_rms_delta_m"),
            ("cpc_m", "zd_cpc_rms_delta_m_max", "cpc_rms_delta_m"),
        ):
            gate_name = f"{prefix}{suffix}"
            stats = components.get(component)
            if not isinstance(stats, Mapping) or "rms_delta_m" not in stats:
                gates.append(
                    {
                        "name": gate_name,
                        "actual": None,
                        "operator": "<=",
                        "expected": thresholds[threshold_name],
                        "status": "failed",
                        "reason": f"{component} is absent from ZD diff",
                    }
                )
            else:
                gates.append(
                    _gate(
                        gate_name,
                        float(stats["rms_delta_m"]),
                        "<=",
                        thresholds[threshold_name],
                    )
                )
    return gates


def build_scorecard(manifest_path: Path) -> dict[str, Any]:
    manifest_path = manifest_path.resolve()
    manifest = _load_json(manifest_path, "manifest")
    if manifest.get("schema") != MANIFEST_SCHEMA:
        raise ValueError(f"{manifest_path}: expected schema {MANIFEST_SCHEMA}")
    root = manifest_path.parent
    dataset = _require_mapping(manifest.get("dataset"), "dataset")
    oracle = _require_mapping(manifest.get("oracle"), "oracle")
    candidate = _require_mapping(manifest.get("candidate"), "candidate")
    solution_config = _require_mapping(manifest.get("solution"), "solution")
    layers = _require_mapping(manifest.get("layers"), "layers")

    dataset_id = _require_text(dataset.get("id"), "dataset.id")
    try:
        expected_epochs = int(dataset.get("expected_epochs"))
    except (TypeError, ValueError) as exc:
        raise ValueError("dataset.expected_epochs must be a positive integer") from exc
    if expected_epochs <= 0:
        raise ValueError("dataset.expected_epochs must be a positive integer")
    oracle_commit = _require_text(oracle.get("commit"), "oracle.commit")
    candidate_commit = _require_text(candidate.get("commit"), "candidate.commit")
    oracle_pos = _resolve_path(root, oracle.get("solution_pos"), "oracle.solution_pos")
    candidate_pos = _resolve_path(root, candidate.get("solution_pos"), "candidate.solution_pos")
    reference_values = manifest.get("reference_ecef_m")
    if not isinstance(reference_values, list) or len(reference_values) != 3:
        raise ValueError("reference_ecef_m must contain three numbers")
    reference = solution_diff.make_reference_frame([float(value) for value in reference_values])
    tolerance = float(solution_config.get("match_tolerance_s", 0.11))
    warmup_seconds = float(dataset.get("warmup_seconds", 0.0))
    if tolerance < 0.0 or warmup_seconds < 0.0:
        raise ValueError("match tolerance and warmup must be non-negative")

    raw_solution, pairs = solution_diff.build_report(
        oracle_pos,
        candidate_pos,
        reference,
        tolerance,
        0.0,
        "claslib",
        "native",
    )
    solution = _solution_layer(
        raw_solution,
        pairs,
        _status_set(solution_config, "oracle_fix_statuses"),
        _status_set(solution_config, "candidate_fix_statuses"),
        _fix_source(solution_config, "oracle"),
        _fix_source(solution_config, "candidate"),
        warmup_seconds,
    )
    zd, _ = _zd_layers(root, layers)
    thresholds = _thresholds(manifest)
    gates = _evaluate_gates(solution, zd, thresholds, expected_epochs)
    failed = [gate["name"] for gate in gates if gate["status"] != "passed"]

    dataset_artifacts: dict[str, Any] = {}
    artifacts = _require_mapping(dataset.get("artifacts"), "dataset.artifacts")
    for required_name in ("observation", "navigation", "l6"):
        if required_name not in artifacts:
            raise ValueError(f"dataset.artifacts.{required_name} is required")
    for name, value in sorted(artifacts.items()):
        dataset_artifacts[str(name)] = _artifact(
            _resolve_path(root, value, f"dataset.artifacts.{name}")
        )

    return {
        "schema": SCHEMA,
        "status": "passed" if not failed else "failed",
        "failed_gates": failed,
        "manifest": _artifact(manifest_path),
        "contract": {
            "dataset_id": dataset_id,
            "start": dataset.get("start"),
            "end": dataset.get("end"),
            "expected_epochs": expected_epochs,
            "warmup_seconds": warmup_seconds,
            "dataset_artifacts": dataset_artifacts,
            "oracle": {
                "implementation": _require_text(oracle.get("implementation", "CLASLIB"), "oracle.implementation"),
                "version": oracle.get("version"),
                "commit": oracle_commit,
                "solution": _artifact(oracle_pos),
            },
            "candidate": {
                "implementation": _require_text(candidate.get("implementation", "LibGNSS++"), "candidate.implementation"),
                "commit": candidate_commit,
                "solution": _artifact(candidate_pos),
            },
        },
        "thresholds": thresholds,
        "layers": {"solution": solution, "zd": zd},
        "gates": gates,
    }


def _print_summary(scorecard: Mapping[str, Any]) -> None:
    print(f"claslib parity scorecard: {scorecard['status']}")
    print(f"  dataset: {scorecard['contract']['dataset_id']}")
    for gate in scorecard["gates"]:
        actual = "missing" if gate["actual"] is None else f"{gate['actual']:.9g}"
        print(
            f"  {gate['status']:6s} {gate['name']}: "
            f"{actual} {gate['operator']} {gate['expected']:.9g}"
        )


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("manifest", type=Path, help=f"{MANIFEST_SCHEMA} JSON manifest")
    parser.add_argument("--json-out", type=Path, required=True, help="Scorecard JSON output")
    parser.add_argument(
        "--fail-on-gate",
        action="store_true",
        help="Return exit status 1 when one or more parity gates fail",
    )
    return parser.parse_args()


def main() -> int:
    args = parse_args()
    try:
        scorecard = build_scorecard(args.manifest)
    except (OSError, TypeError, ValueError) as exc:
        print(f"error: {exc}", file=sys.stderr)
        return 2
    args.json_out.parent.mkdir(parents=True, exist_ok=True)
    args.json_out.write_text(json.dumps(scorecard, indent=2, sort_keys=True) + "\n", encoding="utf-8")
    _print_summary(scorecard)
    return 1 if args.fail_on_gate and scorecard["status"] != "passed" else 0


if __name__ == "__main__":
    raise SystemExit(main())
