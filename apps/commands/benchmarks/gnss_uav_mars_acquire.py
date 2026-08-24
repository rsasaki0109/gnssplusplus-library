#!/usr/bin/env python3
"""Acquire a frozen MARS-LVIG ROS1 bag without accepting quota/error pages."""

from __future__ import annotations

import argparse
from concurrent.futures import ThreadPoolExecutor, as_completed
import hashlib
import json
import math
import os
from pathlib import Path
import shutil
import sys
from urllib.error import HTTPError, URLError
from urllib.request import Request, urlopen


SCHEMA_VERSION = "uav-mars-acquire.v1"
ROSBAG_MAGIC = b"#ROSBAG V2.0\n"


def fail(message: str) -> "NoReturn":
    raise SystemExit(message)


def sha256(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as handle:
        for chunk in iter(lambda: handle.read(8 * 1024 * 1024), b""):
            digest.update(chunk)
    return digest.hexdigest()


def load_profile(path: Path) -> dict[str, object]:
    if not path.is_file():
        fail(f"missing R6 profile: {path}")
    try:
        payload = json.loads(path.read_text(encoding="utf-8"))
    except (OSError, json.JSONDecodeError) as exc:
        fail(f"invalid R6 profile: {exc}")
    if not isinstance(payload, dict) or payload.get("schema_version") != "uav-r6-profile.v1":
        fail("R6 profile schema is not uav-r6-profile.v1")
    return payload


def dataset_contract(profile: dict[str, object], role: str) -> dict[str, object]:
    datasets = profile.get("datasets")
    if not isinstance(datasets, dict) or not isinstance(datasets.get(role), dict):
        fail(f"R6 profile has no {role} dataset")
    dataset = dict(datasets[role])
    required = ("id", "filename", "expected_bytes", "sha256", "urls")
    missing = [name for name in required if not dataset.get(name)]
    if missing:
        fail(f"R6 {role} dataset is not frozen: missing {', '.join(missing)}")
    try:
        expected_bytes = int(dataset["expected_bytes"])
    except (TypeError, ValueError):
        fail(f"R6 {role} expected_bytes must be an integer")
    if expected_bytes <= len(ROSBAG_MAGIC):
        fail(f"R6 {role} expected_bytes is implausible")
    digest = str(dataset["sha256"]).lower()
    if len(digest) != 64 or any(char not in "0123456789abcdef" for char in digest):
        fail(f"R6 {role} sha256 must contain 64 hexadecimal characters")
    urls = dataset["urls"]
    if not isinstance(urls, list) or not urls or not all(isinstance(url, str) and url for url in urls):
        fail(f"R6 {role} urls must be a non-empty string list")
    dataset["expected_bytes"] = expected_bytes
    dataset["sha256"] = digest
    return dataset


def validate_bag(path: Path, expected_bytes: int, expected_sha256: str) -> dict[str, object]:
    if not path.is_file():
        fail(f"missing ROS1 bag: {path}")
    actual_bytes = path.stat().st_size
    if actual_bytes != expected_bytes:
        fail(f"ROS1 bag size mismatch: expected {expected_bytes}, got {actual_bytes}")
    with path.open("rb") as handle:
        magic = handle.read(len(ROSBAG_MAGIC))
    if magic != ROSBAG_MAGIC:
        fail("input is not a ROS1 bag (quota/login/error pages are rejected)")
    actual_sha256 = sha256(path)
    if actual_sha256 != expected_sha256:
        fail(f"ROS1 bag SHA-256 mismatch: expected {expected_sha256}, got {actual_sha256}")
    return {"path": str(path), "bytes": actual_bytes, "sha256": actual_sha256}


def download_range(url: str, path: Path, start: int, end: int, total: int) -> None:
    expected = end - start + 1
    if path.is_file() and path.stat().st_size == expected:
        return
    request = Request(url, headers={"Range": f"bytes={start}-{end}", "User-Agent": "libgnss++-uav-r6/1"})
    temporary = path.with_suffix(path.suffix + ".tmp")
    try:
        with urlopen(request, timeout=120) as response, temporary.open("wb") as output:
            if response.status != 206:
                fail(f"server ignored byte range {start}-{end}: HTTP {response.status}")
            content_range = response.headers.get("Content-Range", "")
            if content_range != f"bytes {start}-{end}/{total}":
                fail(f"unexpected Content-Range for {start}-{end}: {content_range!r}")
            shutil.copyfileobj(response, output, length=1024 * 1024)
    except (HTTPError, URLError, TimeoutError, OSError) as exc:
        fail(f"download failed for byte range {start}-{end}: {exc}")
    if temporary.stat().st_size != expected:
        fail(
            f"short download for byte range {start}-{end}: "
            f"expected {expected}, got {temporary.stat().st_size}"
        )
    temporary.replace(path)


def acquire(url: str, destination: Path, total: int, jobs: int) -> None:
    destination.parent.mkdir(parents=True, exist_ok=True)
    parts_dir = destination.parent / f"{destination.stem}.parts"
    parts_dir.mkdir(parents=True, exist_ok=True)
    part_count = min(jobs, max(1, math.ceil(total / (256 * 1024 * 1024))))
    part_size = math.ceil(total / part_count)
    tasks: list[tuple[int, int, Path]] = []
    for index in range(part_count):
        start = index * part_size
        end = min(total - 1, start + part_size - 1)
        if start <= end:
            tasks.append((start, end, parts_dir / f"part-{index:03d}"))
    with ThreadPoolExecutor(max_workers=jobs) as executor:
        futures = {
            executor.submit(download_range, url, path, start, end, total): (start, end)
            for start, end, path in tasks
        }
        for future in as_completed(futures):
            future.result()
    temporary = destination.with_suffix(destination.suffix + ".tmp")
    with temporary.open("wb") as output:
        for _, _, part in tasks:
            with part.open("rb") as source:
                shutil.copyfileobj(source, output, length=8 * 1024 * 1024)
    if temporary.stat().st_size != total:
        fail(f"assembled bag size mismatch: expected {total}, got {temporary.stat().st_size}")
    temporary.replace(destination)


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(prog=os.environ.get("GNSS_CLI_NAME"))
    parser.add_argument("--profile", type=Path, required=True)
    parser.add_argument("--role", choices=("development", "holdout"), required=True)
    parser.add_argument("--output-dir", type=Path, required=True)
    parser.add_argument("--input", type=Path, help="Validate an already acquired bag instead of downloading.")
    parser.add_argument("--jobs", type=int, default=8)
    return parser.parse_args()


def main() -> int:
    args = parse_args()
    if not 1 <= args.jobs <= 32:
        fail("--jobs must be between 1 and 32")
    profile = load_profile(args.profile)
    dataset = dataset_contract(profile, args.role)
    args.output_dir.mkdir(parents=True, exist_ok=True)
    destination = args.input or (args.output_dir / str(dataset["filename"]))
    selected_url: str | None = None
    if args.input is None and not destination.is_file():
        errors: list[str] = []
        for url in dataset["urls"]:
            try:
                acquire(str(url), destination, int(dataset["expected_bytes"]), args.jobs)
                selected_url = str(url)
                break
            except SystemExit as exc:
                errors.append(f"{url}: {exc}")
        if selected_url is None:
            fail("all R6 sources failed:\n" + "\n".join(errors))
    validated = validate_bag(destination, int(dataset["expected_bytes"]), str(dataset["sha256"]))
    payload = {
        "schema_version": SCHEMA_VERSION,
        "dataset": {
            "id": dataset["id"],
            "role": args.role,
            "source_page": dataset.get("source_page"),
            "official_file_id": dataset.get("official_file_id"),
            "selected_url": selected_url,
            "licence_status": dataset.get("licence_status"),
            "redistribution": dataset.get("redistribution"),
        },
        "bag": validated,
        "frame_contract": profile.get("frame_contract"),
        "time_contract": profile.get("time_contract"),
        "command": [os.environ.get("GNSS_CLI_NAME", "uav-mars-acquire"), *sys.argv[1:]],
    }
    summary = args.output_dir / "acquire_summary.json"
    summary.write_text(json.dumps(payload, indent=2, sort_keys=True) + "\n", encoding="utf-8")
    print(json.dumps(payload, indent=2, sort_keys=True))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
