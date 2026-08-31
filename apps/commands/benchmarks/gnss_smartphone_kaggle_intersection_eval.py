#!/usr/bin/env python3
"""Score several frozen submissions on one truth file by key intersection.

This is an evaluation-protocol recovery tool.  It reads the truth CSV once,
parses every requested prediction, and reuses the distance and percentile
implementations from :mod:`gnss_smartphone_kaggle`.  Prediction keys outside
the truth are retained in the report but never contribute to a score.  Missing
truth keys are reported as uncovered; this tool never fills or interpolates
them.
"""

from __future__ import annotations

import argparse
from collections import defaultdict
import hashlib
import json
from pathlib import Path
import sys
from typing import Any, Mapping


try:
    import gnss_smartphone_kaggle as metric
except ModuleNotFoundError:  # pragma: no cover - direct path invocation fallback
    sys.path.insert(0, str(Path(__file__).resolve().parent))
    import gnss_smartphone_kaggle as metric


SCHEMA_VERSION = "smartphone-r5-kaggle-intersection-evaluation.v1"
POLICY_VERSION = "truth-intersection-surplus-count-v1"
COVERAGE_MINIMUM = 0.999
MISSING_MAXIMUM = 1


def _key_digest(keys: set[tuple[str, int]]) -> str:
    canonical = json.dumps(
        [[phone, timestamp] for phone, timestamp in sorted(keys)],
        ensure_ascii=True,
        separators=(",", ":"),
    ).encode("utf-8")
    return hashlib.sha256(canonical).hexdigest()


def _score_prediction(
    prediction_by_key: dict[tuple[str, int], metric.CoordinateRow],
    truth_by_key: dict[tuple[str, int], metric.CoordinateRow],
    matched_keys: set[tuple[str, int]],
) -> dict[str, Any]:
    distance_functions = {
        "wgs84_vincenty": metric._wgs84_horizontal_distance_m,
        "haversine_sphere": metric._haversine_horizontal_distance_m,
    }
    percentile_functions = {
        "linear_n_minus_1": metric._percentile_linear_n_minus_1,
        "nearest_rank_ceiling": metric._percentile_nearest_rank_ceiling,
    }
    distances_by_phone: dict[str, dict[str, list[float]]] = {
        distance_name: defaultdict(list)
        for distance_name in distance_functions
    }
    for key in sorted(matched_keys):
        prediction = prediction_by_key[key]
        reference = truth_by_key[key]
        for distance_name, distance_function in distance_functions.items():
            distances_by_phone[distance_name][prediction.phone].append(
                distance_function(
                    prediction.latitude,
                    prediction.longitude,
                    reference.latitude,
                    reference.longitude,
                )
            )

    phone_metrics: dict[str, dict[str, Any]] = {}
    score_values: dict[str, list[float]] = {
        metric._score_variant_id(distance_name, percentile_name): []
        for distance_name in distance_functions
        for percentile_name in percentile_functions
    }
    for phone in sorted(distances_by_phone["wgs84_vincenty"]):
        variants: dict[str, dict[str, Any]] = {}
        for distance_name, values_by_phone in distances_by_phone.items():
            values = values_by_phone[phone]
            for percentile_name, percentile_function in percentile_functions.items():
                p50 = percentile_function(values, 0.50)
                p95 = percentile_function(values, 0.95)
                variant_id = metric._score_variant_id(distance_name, percentile_name)
                phone_score = (p50 + p95) / 2.0
                score_values[variant_id].append(phone_score)
                variants[variant_id] = {
                    "distance_variant": distance_name,
                    "percentile_variant": percentile_name,
                    "p50_m": p50,
                    "p95_m": p95,
                    "phone_score_m": phone_score,
                }
        phone_metrics[phone] = {
            "matched_rows": len(
                distances_by_phone["wgs84_vincenty"][phone]
            ),
            "score_variants": variants,
        }
    if not phone_metrics:
        raise ValueError("no truth-intersection keys were available for scoring")
    return {
        "matched_rows": len(matched_keys),
        "phone_count_scored": len(phone_metrics),
        "score_variants_m": {
            variant_id: sum(values) / len(values)
            for variant_id, values in sorted(score_values.items())
        },
        "phones": phone_metrics,
    }


def _parse_submission_specs(specs: list[str]) -> dict[str, Path]:
    if len(specs) < 2:
        raise ValueError("at least control=PATH and candidate=PATH are required")
    parsed: dict[str, Path] = {}
    for spec in specs:
        label, separator, path_text = spec.partition("=")
        if not separator or not label or label != label.strip() or not path_text:
            raise ValueError("each --submission must be LABEL=PATH")
        if label in parsed:
            raise ValueError(f"duplicate submission label: {label}")
        parsed[label] = Path(path_text)
    return parsed


def evaluate_submissions(
    truth_path: Path,
    submissions: Mapping[str, Path],
    output_json_path: Path | None = None,
    *,
    phone: str | None = None,
) -> dict[str, Any]:
    """Evaluate all submissions against one in-memory truth read.

    The function deliberately does not hash the truth path after parsing: the
    existing parser performs one file open, and a second hash pass would break
    the one-open contract.  Submission hashes and deterministic key-set
    hashes are still recorded.
    """

    if len(submissions) < 2:
        raise ValueError("at least two submissions are required")
    normalized = {str(label): Path(path) for label, path in submissions.items()}
    labels = list(normalized)
    if any(not label or label != label.strip() for label in labels):
        raise ValueError("submission labels must be non-empty and trimmed")
    input_paths = [("ground-truth CSV", truth_path)] + [
        (f"submission {label}", path) for label, path in normalized.items()
    ]
    if output_json_path is not None:
        metric._reject_output_collisions(output_json_path, None, input_paths)

    # This is the sole truth-file read in this process.  All subsequent
    # submissions are compared with this immutable in-memory key/value set.
    truth = metric._read_ground_truth(truth_path, phone)
    truth_by_key = {(row.phone, row.timestamp): row for row in truth}
    truth_keys = set(truth_by_key)
    truth_phones = {row.phone for row in truth}

    parsed: dict[str, list[metric.CoordinateRow]] = {}
    predictions: dict[str, dict[tuple[str, int], metric.CoordinateRow]] = {}
    metadata: dict[str, dict[str, Any]] = {}
    for label, path in normalized.items():
        rows = metric._read_submission(path)
        prediction_by_key = {(row.phone, row.timestamp): row for row in rows}
        prediction_phones = {row.phone for row in rows}
        if prediction_phones != truth_phones:
            raise ValueError(
                f"submission {label!r} phone mismatch: "
                f"prediction={sorted(prediction_phones)!r}, "
                f"truth={sorted(truth_phones)!r}"
            )
        parsed[label] = rows
        predictions[label] = prediction_by_key
        prediction_keys = set(prediction_by_key)
        extra_keys = prediction_keys - truth_keys
        missing_keys = truth_keys - prediction_keys
        matched_keys = prediction_keys & truth_keys
        metadata[label] = {
            "path": str(path),
            "sha256": metric._sha256(path),
            "prediction_rows": len(rows),
            "prediction_key_count": len(prediction_keys),
            "extra_prediction_keys": len(extra_keys),
            "extra_prediction_keys_sha256": _key_digest(extra_keys),
            "missing_truth_keys": len(missing_keys),
            "missing_truth_keys_sha256": _key_digest(missing_keys),
            "matched_key_count": len(matched_keys),
            "matched_key_set_sha256": _key_digest(matched_keys),
            "coverage": len(matched_keys) / len(truth_keys)
            if truth_keys
            else 0.0,
        }

    control_label = labels[0]
    control_matched = set(predictions[control_label]) & truth_keys
    for label in labels[1:]:
        candidate_matched = set(predictions[label]) & truth_keys
        if candidate_matched != control_matched:
            raise ValueError(
                "control/candidate matched-key-set mismatch: "
                f"{control_label!r} vs {label!r}"
            )

    results: dict[str, dict[str, Any]] = {}
    for label in labels:
        results[label] = _score_prediction(
            predictions[label], truth_by_key, control_matched
        )
        metadata[label]["metrics"] = results[label]

    coverage = (
        len(control_matched) / len(truth_keys) if truth_keys else 0.0
    )
    missing_count = len(truth_keys - control_matched)
    key_gate = (
        bool(truth_keys)
        and coverage >= COVERAGE_MINIMUM
        and missing_count <= MISSING_MAXIMUM
    )
    comparisons: dict[str, dict[str, Any]] = {}
    control_scores = results[control_label]["score_variants_m"]
    for label in labels[1:]:
        candidate_scores = results[label]["score_variants_m"]
        deltas = {
            variant_id: candidate_scores[variant_id] - control_scores[variant_id]
            for variant_id in sorted(control_scores)
        }
        strict_improvement = all(delta < 0.0 for delta in deltas.values())
        comparisons[label] = {
            "control": control_label,
            "candidate": label,
            "score_delta_candidate_minus_control_m": deltas,
            "all_four_strictly_improved": strict_improvement,
            "coverage_equal": metadata[label]["matched_key_set_sha256"]
            == metadata[control_label]["matched_key_set_sha256"],
            "accept": key_gate and strict_improvement,
        }

    report: dict[str, Any] = {
        "schema_version": SCHEMA_VERSION,
        "policy_version": POLICY_VERSION,
        "status": "development-only-intersection-evaluation",
        "truth": {
            "path": str(truth_path),
            "truth_rows": len(truth),
            "truth_key_count": len(truth_keys),
            "phone_count": len(truth_phones),
            "open_count": 1,
            "sha256": None,
            "sha256_note": "not computed after the single truth read",
        },
        "policy": {
            "truth_intersection_scored": True,
            "surplus_prediction_keys": "ignored_for_score_counted_and_hashed",
            "missing_truth_predictions": "counted_only_no_interpolation_or_hold",
            "coverage_definition": "matched_key_count / truth_key_count",
            "control_candidate_matched_key_set_required": True,
            "coverage_minimum": COVERAGE_MINIMUM,
            "missing_maximum": MISSING_MAXIMUM,
            "distance_variants": list(metric.DISTANCE_VARIANT_IDS),
            "percentile_variants": list(metric.PERCENTILE_VARIANT_IDS),
            "duplicate_nonfinite_phone_mismatch": "fail_closed",
        },
        "matched_key_set": {
            "count": len(control_matched),
            "sha256": _key_digest(control_matched),
            "missing_truth_keys": missing_count,
            "coverage": coverage,
            "gate": key_gate,
        },
        "submissions": metadata,
        "metrics": results,
        "comparisons": comparisons,
        "decision": {
            "control": control_label,
            "all_candidates_accept": bool(comparisons)
            and all(item["accept"] for item in comparisons.values()),
            "promotion": "development-only" if comparisons and all(
                item["accept"] for item in comparisons.values()
            ) else "no-go",
            "no_post_score_tuning": True,
        },
    }
    if output_json_path is not None:
        metric._atomic_write(output_json_path, metric._json_bytes(report))
    return report


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(
        prog="gnss-smartphone-kaggle-intersection-eval",
        description=(
            "Read one truth CSV once and score multiple submissions on their "
            "truth-key intersection."
        ),
    )
    parser.add_argument("--truth", type=Path, required=True)
    parser.add_argument(
        "--submission",
        action="append",
        required=True,
        metavar="LABEL=PATH",
        help="repeat at least twice; first label is the control",
    )
    parser.add_argument("--output", type=Path, required=True)
    parser.add_argument("--phone")
    return parser


def main(argv: list[str] | None = None) -> int:
    args = build_parser().parse_args(argv)
    try:
        submissions = _parse_submission_specs(args.submission)
        report = evaluate_submissions(
            args.truth,
            submissions,
            args.output,
            phone=args.phone,
        )
    except (OSError, ValueError) as exc:
        raise SystemExit(f"smartphone intersection evaluation failed: {exc}") from exc
    print(f"Smartphone intersection evaluation: {args.output}")
    print(f"Promotion: {report['decision']['promotion']}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
