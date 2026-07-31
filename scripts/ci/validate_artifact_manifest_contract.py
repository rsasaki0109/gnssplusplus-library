#!/usr/bin/env python3
"""Validate dashboard artifact manifest files before CI upload."""

from __future__ import annotations

import argparse
import csv
import json
import sys
from collections.abc import Iterator, Sequence
from pathlib import Path
from typing import Any
from urllib.parse import urlparse


ROOT_DIR = Path(__file__).resolve().parents[2]
PNG_SIGNATURE = b"\x89PNG\r\n\x1a\n"


def load_json_file(path: Path, label: str) -> Any:
    try:
        return json.loads(path.read_text(encoding="utf-8"))
    except OSError as exc:
        raise AssertionError(f"{label}: failed to read {path}: {exc}") from exc
    except json.JSONDecodeError as exc:
        raise AssertionError(f"{label}: invalid JSON in {path}: {exc}") from exc


def is_url(value: str) -> bool:
    parsed = urlparse(value)
    return parsed.scheme in {"http", "https"} and bool(parsed.netloc)


def resolve_manifest_path(root_dir: Path, value: str, label: str) -> Path:
    candidate = Path(value)
    if candidate.is_absolute():
        resolved = candidate.resolve()
    else:
        resolved = (root_dir / candidate).resolve()
    try:
        resolved.relative_to(root_dir)
    except ValueError as exc:
        raise AssertionError(f"{label}: artifact path escapes manifest root: {value!r}") from exc
    return resolved


def assert_regular_nonempty_file(path: Path, label: str) -> None:
    if not path.is_file():
        raise AssertionError(f"{label}: missing artifact file: {path}")
    size = path.stat().st_size
    if size <= 0:
        raise AssertionError(f"{label}: artifact is empty: {path}")


def assert_parseable_file(path: Path, label: str) -> None:
    suffix = path.suffix.lower()
    if suffix == ".json":
        load_json_file(path, label)
        return
    if suffix == ".csv":
        try:
            with path.open(newline="", encoding="utf-8") as handle:
                first_row = next(csv.reader(handle), None)
        except UnicodeDecodeError as exc:
            raise AssertionError(f"{label}: CSV is not UTF-8 text: {path}") from exc
        except csv.Error as exc:
            raise AssertionError(f"{label}: CSV parse failed: {path}: {exc}") from exc
        if not first_row or not any(cell.strip() for cell in first_row):
            raise AssertionError(f"{label}: CSV header is empty: {path}")
        return
    if suffix == ".png":
        signature = path.read_bytes()[: len(PNG_SIGNATURE)]
        if signature != PNG_SIGNATURE:
            raise AssertionError(f"{label}: PNG signature is invalid: {path}")


def iter_artifact_paths(value: Any, label: str) -> Iterator[tuple[str, str]]:
    if isinstance(value, str) and value:
        yield label, value
        return
    if isinstance(value, dict):
        for key, item in value.items():
            child_label = f"{label}.{key}" if label else str(key)
            yield from iter_artifact_paths(item, child_label)
        return
    if isinstance(value, list):
        for index, item in enumerate(value):
            child_label = f"{label}[{index}]"
            yield from iter_artifact_paths(item, child_label)


def validate_artifact_value(root_dir: Path, label: str, value: str) -> None:
    if is_url(value):
        return
    path = resolve_manifest_path(root_dir, value, label)
    assert_regular_nonempty_file(path, label)
    assert_parseable_file(path, label)


def validate_bundle(root_dir: Path, bundle: Any, index: int) -> None:
    label = f"bundles[{index}]"
    if not isinstance(bundle, dict):
        raise AssertionError(f"{label}: bundle must be an object")
    required_fields = {"category", "label", "summary_json", "status", "headline", "artifacts", "metrics"}
    missing = sorted(required_fields - set(bundle))
    if missing:
        raise AssertionError(f"{label}: missing required fields: {missing}")
    for key in ("category", "label", "summary_json", "status", "headline"):
        if not isinstance(bundle.get(key), str) or not bundle[key].strip():
            raise AssertionError(f"{label}.{key}: must be a non-empty string")
    if not isinstance(bundle.get("artifacts"), dict):
        raise AssertionError(f"{label}.artifacts: must be an object")
    if not isinstance(bundle.get("metrics"), dict):
        raise AssertionError(f"{label}.metrics: must be an object")

    validate_artifact_value(root_dir, f"{label}.summary_json", bundle["summary_json"])
    for artifact_label, artifact_path in iter_artifact_paths(bundle["artifacts"], f"{label}.artifacts"):
        validate_artifact_value(root_dir, artifact_label, artifact_path)


def validate_manifest(payload: Any) -> None:
    if not isinstance(payload, dict):
        raise AssertionError("manifest payload must be an object")
    expected_top_level = {
        "schema_version",
        "contract",
        "root",
        "generated_at_utc",
        "bundle_count",
        "bundles",
    }
    if set(payload) != expected_top_level:
        raise AssertionError(f"manifest has unexpected top-level keys: {sorted(payload)}")
    if payload.get("schema_version") != 1:
        raise AssertionError("manifest schema_version must be 1")
    if payload.get("contract") != "artifact_manifest.v1":
        raise AssertionError("manifest contract must be artifact_manifest.v1")
    if not isinstance(payload.get("root"), str) or not payload["root"].strip():
        raise AssertionError("manifest root must be a non-empty string")
    root_dir = Path(payload["root"]).resolve()
    if not root_dir.is_dir():
        raise AssertionError(f"manifest root is not a directory: {root_dir}")
    if not isinstance(payload.get("generated_at_utc"), str) or not payload["generated_at_utc"].strip():
        raise AssertionError("generated_at_utc must be a non-empty string")
    bundles = payload.get("bundles")
    if not isinstance(bundles, list):
        raise AssertionError("bundles must be a list")
    if payload.get("bundle_count") != len(bundles):
        raise AssertionError("bundle_count does not match bundles length")
    for index, bundle in enumerate(bundles):
        validate_bundle(root_dir, bundle, index)


def parse_args(argv: Sequence[str]) -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        "manifest",
        nargs="?",
        type=Path,
        default=ROOT_DIR / "output" / "artifact_manifest.json",
        help="Artifact manifest JSON path.",
    )
    return parser.parse_args(argv)


def main(argv: Sequence[str] | None = None) -> int:
    args = parse_args(sys.argv[1:] if argv is None else argv)
    manifest_path = args.manifest.resolve()
    assert_regular_nonempty_file(manifest_path, "manifest")
    validate_manifest(load_json_file(manifest_path, "manifest"))
    print("Artifact manifest contract validated.")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
