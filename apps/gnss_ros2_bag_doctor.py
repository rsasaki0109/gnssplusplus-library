#!/usr/bin/env python3
"""Read-only ROS2 bag diagnostics for GNSS field logs."""

from __future__ import annotations

import argparse
import json
import os
from pathlib import Path
import sqlite3
import statistics
from typing import Any

try:
    import yaml  # type: ignore[import-not-found]
except ImportError:  # pragma: no cover - optional dependency
    yaml = None  # type: ignore[assignment]


EXPECTED_TOPICS = {
    "raw_binary": {
        "topic": "/gnss/raw_binary",
        "type": "std_msgs/msg/UInt8MultiArray",
        "purpose": "lossless replay and decoder regression",
    },
    "raw": {
        "topic": "/gnss/raw",
        "type": "gnss_raw_driver/msg/GnssRawEpoch",
        "purpose": "parsed GNSS observables for inspection",
    },
    "fix": {
        "topic": "/gnss/fix",
        "type": "sensor_msgs/msg/NavSatFix",
        "purpose": "standard robot localization input",
    },
}


def rounded(value: float | None, digits: int = 6) -> float | None:
    if value is None:
        return None
    return round(float(value), digits)


def int_or_none(value: Any) -> int | None:
    if isinstance(value, bool):
        return None
    if isinstance(value, int):
        return value
    if isinstance(value, str):
        try:
            return int(value)
        except ValueError:
            return None
    return None


def duration_to_seconds(value: Any) -> float | None:
    if isinstance(value, dict):
        duration_ns = int_or_none(value.get("nanoseconds"))
        if duration_ns is not None:
            return duration_ns / 1e9
    duration_ns = int_or_none(value)
    if duration_ns is not None:
        return duration_ns / 1e9
    return None


def canonical_topic(name: str) -> str:
    return "/" + name.lstrip("/")


def add_check(
    checks: list[dict[str, str]],
    name: str,
    status: str,
    detail: str,
    *,
    hint: str | None = None,
) -> None:
    item = {"name": name, "status": status, "detail": detail}
    if hint:
        item["hint"] = hint
    checks.append(item)


def status_rank(status: str) -> int:
    return {"ok": 0, "warn": 1, "missing": 2}.get(status, 1)


def find_storage_files(bag_dir: Path) -> list[Path]:
    patterns = ("*.db3", "*.sqlite3", "*.sqlite")
    files: list[Path] = []
    for pattern in patterns:
        files.extend(path for path in sorted(bag_dir.glob(pattern)) if path.is_file())
    return sorted(set(files))


def find_mcap_files(bag_dir: Path, metadata: dict[str, Any]) -> list[Path]:
    files: list[Path] = []
    relative_paths = metadata.get("relative_file_paths")
    if isinstance(relative_paths, list):
        for relative_path in relative_paths:
            if not isinstance(relative_path, str):
                continue
            candidate = (bag_dir / relative_path).resolve()
            try:
                candidate.relative_to(bag_dir)
            except ValueError:
                continue
            if candidate.is_file() and candidate.suffix.lower() == ".mcap":
                files.append(candidate)
    files.extend(path.resolve() for path in sorted(bag_dir.glob("*.mcap")) if path.is_file())
    return sorted(set(files))


def metadata_topic_summary(item: dict[str, Any]) -> dict[str, Any] | None:
    topic_metadata = item.get("topic_metadata")
    if not isinstance(topic_metadata, dict):
        return None
    name = topic_metadata.get("name")
    if not isinstance(name, str) or not name:
        return None
    message_count = int_or_none(item.get("message_count"))
    return {
        "name": canonical_topic(name),
        "type": topic_metadata.get("type"),
        "serialization_format": topic_metadata.get("serialization_format"),
        "message_count": message_count if message_count is not None else 0,
        "source": "metadata",
        "rate_source": "unavailable",
        "gap_source": "unavailable",
    }


def load_metadata(metadata_yaml: Path) -> dict[str, Any]:
    if not metadata_yaml.exists():
        return {"available": False, "path": str(metadata_yaml), "topics": []}
    payload: dict[str, Any] = {
        "available": True,
        "path": str(metadata_yaml),
        "parsed": False,
        "topics": [],
    }
    if yaml is None:
        payload["parse_error"] = "PyYAML is not installed"
        return payload
    try:
        parsed = yaml.safe_load(metadata_yaml.read_text(encoding="utf-8"))
    except (OSError, ValueError) as exc:
        payload["parse_error"] = str(exc)
        return payload
    if not isinstance(parsed, dict):
        return payload
    info = parsed.get("rosbag2_bagfile_information")
    if not isinstance(info, dict):
        info = parsed
    payload["parsed"] = True
    payload["storage_identifier"] = info.get("storage_identifier")
    payload["message_count"] = int_or_none(info.get("message_count"))
    relative_file_paths = info.get("relative_file_paths")
    if isinstance(relative_file_paths, list):
        payload["relative_file_paths"] = [str(path) for path in relative_file_paths]
    duration_s = duration_to_seconds(info.get("duration"))
    if duration_s is not None:
        payload["duration_s"] = rounded(duration_s)
    starting_time = info.get("starting_time")
    if isinstance(starting_time, dict):
        start_ns = int_or_none(starting_time.get("nanoseconds_since_epoch"))
        if start_ns is not None:
            payload["start_time_ns"] = start_ns
    topics_with_message_count = info.get("topics_with_message_count")
    if isinstance(topics_with_message_count, list):
        topics = [
            topic
            for topic in (metadata_topic_summary(item) for item in topics_with_message_count if isinstance(item, dict))
            if topic is not None
        ]
        topics.sort(key=lambda item: str(item["name"]))
        payload["topics"] = topics
    return payload


def empty_topic_record(
    name: str,
    msg_type: str,
    serialization_format: str | None,
    *,
    source: str = "sqlite",
) -> dict[str, Any]:
    return {
        "name": canonical_topic(name),
        "type": msg_type,
        "serialization_format": serialization_format,
        "timestamps_ns": [],
        "serialized_bytes": 0,
        "source": source,
    }


def read_sqlite_topics(storage_files: list[Path]) -> tuple[dict[str, dict[str, Any]], list[str]]:
    topics: dict[str, dict[str, Any]] = {}
    errors: list[str] = []
    for db_path in storage_files:
        try:
            connection = sqlite3.connect(str(db_path))
        except sqlite3.Error as exc:
            errors.append(f"{db_path}: open failed: {exc}")
            continue
        try:
            cursor = connection.cursor()
            topic_rows = cursor.execute(
                "SELECT id, name, type, serialization_format FROM topics"
            ).fetchall()
            topic_by_id: dict[int, tuple[str, str, str | None]] = {}
            for topic_id, name, msg_type, serialization_format in topic_rows:
                topic_by_id[int(topic_id)] = (str(name), str(msg_type), serialization_format)
                canonical = canonical_topic(str(name))
                topics.setdefault(canonical, empty_topic_record(str(name), str(msg_type), serialization_format))

            for topic_id, timestamp, size_bytes in cursor.execute(
                "SELECT topic_id, timestamp, length(data) FROM messages ORDER BY timestamp"
            ):
                topic = topic_by_id.get(int(topic_id))
                if topic is None:
                    continue
                name, msg_type, serialization_format = topic
                canonical = canonical_topic(name)
                record = topics.setdefault(canonical, empty_topic_record(name, msg_type, serialization_format))
                record["timestamps_ns"].append(int(timestamp))
                if isinstance(size_bytes, int):
                    record["serialized_bytes"] += size_bytes
        except sqlite3.Error as exc:
            errors.append(f"{db_path}: read failed: {exc}")
        finally:
            connection.close()
    return topics, errors


def read_mcap_topics(
    mcap_files: list[Path],
    metadata_topics: dict[str, dict[str, Any]],
) -> tuple[dict[str, dict[str, Any]], list[str]]:
    topics: dict[str, dict[str, Any]] = {}
    errors: list[str] = []
    if not mcap_files:
        return topics, errors
    try:
        from mcap.reader import make_reader  # type: ignore[import-not-found]
    except ImportError as exc:
        return topics, [f"mcap reader unavailable: {exc}"]

    for mcap_path in mcap_files:
        try:
            with mcap_path.open("rb") as stream:
                reader = make_reader(stream)
                for schema, channel, message in reader.iter_messages(log_time_order=True):
                    topic_name = getattr(channel, "topic", None)
                    if not isinstance(topic_name, str) or not topic_name:
                        continue
                    canonical = canonical_topic(topic_name)
                    metadata_topic = metadata_topics.get(canonical, {})
                    schema_name = getattr(schema, "name", None) if schema is not None else None
                    msg_type = schema_name if isinstance(schema_name, str) and schema_name else metadata_topic.get("type")
                    if not isinstance(msg_type, str) or not msg_type:
                        msg_type = "unknown"
                    encoding = getattr(channel, "message_encoding", None)
                    if not isinstance(encoding, str) or not encoding:
                        encoding = metadata_topic.get("serialization_format")
                    if not isinstance(encoding, str):
                        encoding = None
                    record = topics.setdefault(
                        canonical,
                        empty_topic_record(topic_name, msg_type, encoding, source="mcap"),
                    )
                    timestamp_ns = int_or_none(getattr(message, "log_time", None))
                    if timestamp_ns is None:
                        timestamp_ns = int_or_none(getattr(message, "publish_time", None))
                    if timestamp_ns is not None:
                        record["timestamps_ns"].append(timestamp_ns)
                    data = getattr(message, "data", None)
                    if isinstance(data, (bytes, bytearray, memoryview)):
                        record["serialized_bytes"] += len(data)
        except Exception as exc:  # pragma: no cover - depends on optional mcap package internals
            errors.append(f"{mcap_path}: read failed: {exc}")
    return topics, errors


def summarize_topic(record: dict[str, Any], *, gap_factor: float, gap_min_s: float) -> dict[str, Any]:
    timestamps = sorted(int(value) for value in record.get("timestamps_ns", []))
    count = len(timestamps)
    intervals_s = [
        (timestamps[index] - timestamps[index - 1]) / 1e9
        for index in range(1, len(timestamps))
        if timestamps[index] >= timestamps[index - 1]
    ]
    median_period_s = statistics.median(intervals_s) if intervals_s else None
    duration_s = (timestamps[-1] - timestamps[0]) / 1e9 if count >= 2 else 0.0
    mean_rate_hz = (count - 1) / duration_s if count >= 2 and duration_s > 0.0 else None
    gap_threshold_s = gap_min_s
    if median_period_s is not None:
        gap_threshold_s = max(gap_min_s, median_period_s * gap_factor)
    gaps = [
        {
            "after_timestamp_ns": timestamps[index - 1],
            "before_timestamp_ns": timestamps[index],
            "gap_s": rounded(interval_s),
        }
        for index, interval_s in enumerate(intervals_s, start=1)
        if interval_s > gap_threshold_s
    ]
    max_gap_s = max(intervals_s) if intervals_s else None
    return {
        "name": record["name"],
        "type": record.get("type"),
        "serialization_format": record.get("serialization_format"),
        "message_count": count,
        "serialized_bytes": record.get("serialized_bytes", 0),
        "first_timestamp_ns": timestamps[0] if timestamps else None,
        "last_timestamp_ns": timestamps[-1] if timestamps else None,
        "duration_s": rounded(duration_s),
        "mean_rate_hz": rounded(mean_rate_hz),
        "median_period_s": rounded(median_period_s),
        "max_gap_s": rounded(max_gap_s),
        "gap_threshold_s": rounded(gap_threshold_s),
        "gap_count": len(gaps),
        "gaps": gaps[:10],
        "source": record.get("source", "sqlite"),
        "rate_source": record.get("source", "sqlite"),
        "gap_source": record.get("source", "sqlite"),
    }


def summarize_metadata_topic(topic: dict[str, Any]) -> dict[str, Any]:
    return {
        "name": topic["name"],
        "type": topic.get("type"),
        "serialization_format": topic.get("serialization_format"),
        "message_count": topic.get("message_count", 0),
        "serialized_bytes": None,
        "first_timestamp_ns": None,
        "last_timestamp_ns": None,
        "duration_s": None,
        "mean_rate_hz": None,
        "median_period_s": None,
        "max_gap_s": None,
        "gap_threshold_s": None,
        "gap_count": None,
        "gaps": [],
        "source": "metadata",
        "rate_source": "unavailable",
        "gap_source": "unavailable",
    }


def find_expected_topic(topics: list[dict[str, Any]], expected_name: str) -> dict[str, Any] | None:
    expected = canonical_topic(expected_name)
    for topic in topics:
        if canonical_topic(str(topic.get("name", ""))) == expected:
            return topic
    return None


def build_topic_status(topics: list[dict[str, Any]]) -> dict[str, dict[str, Any]]:
    statuses: dict[str, dict[str, Any]] = {}
    for key, expected in EXPECTED_TOPICS.items():
        topic = find_expected_topic(topics, expected["topic"])
        if topic is None:
            statuses[key] = {
                "status": "missing",
                "topic": expected["topic"],
                "expected_type": expected["type"],
                "message_count": 0,
                "purpose": expected["purpose"],
            }
            continue
        actual_type = topic.get("type")
        status = "ok"
        if topic.get("message_count", 0) <= 0:
            status = "missing"
        elif actual_type != expected["type"]:
            status = "warn"
        statuses[key] = {
            "status": status,
            "topic": topic.get("name"),
            "expected_type": expected["type"],
            "actual_type": actual_type,
            "message_count": topic.get("message_count"),
            "mean_rate_hz": topic.get("mean_rate_hz"),
            "max_gap_s": topic.get("max_gap_s"),
            "gap_count": topic.get("gap_count"),
            "source": topic.get("source", "sqlite"),
            "rate_source": topic.get("rate_source", "sqlite"),
            "gap_source": topic.get("gap_source", "sqlite"),
            "purpose": expected["purpose"],
        }
    return statuses


def build_commands(bag_dir: Path, protocol: str, output_pos: Path, output_kml: Path) -> dict[str, str]:
    return {
        "info": f"ros2 bag info {bag_dir}",
        "play": f"ros2 bag play {bag_dir}",
        "decode": (
            "ros2 run gnss_raw_driver gnss_bag_processor_node --ros-args "
            f"-p protocol:={protocol} "
            f"-p output_pos:={output_pos} "
            f"-p output_kml:={output_kml}"
        ),
        "record_next_time": "ros2 bag record /gnss/raw_binary /gnss/raw /gnss/fix",
    }


def build_payload(args: argparse.Namespace) -> dict[str, Any]:
    bag_dir = args.bag.resolve()
    metadata_path = bag_dir / "metadata.yaml"
    storage_files = find_storage_files(bag_dir)
    metadata = load_metadata(metadata_path)
    storage_identifier = metadata.get("storage_identifier")
    is_mcap_storage = isinstance(storage_identifier, str) and storage_identifier.lower() == "mcap"
    mcap_files = find_mcap_files(bag_dir, metadata)
    checks: list[dict[str, str]] = []

    add_check(
        checks,
        "bag directory",
        "ok" if bag_dir.is_dir() else "missing",
        str(bag_dir),
        hint="Pass a ROS2 bag directory, not a parent folder or zip file.",
    )
    add_check(
        checks,
        "metadata.yaml",
        "ok" if metadata_path.exists() else "warn",
        str(metadata_path),
        hint="ROS2 bags normally contain metadata.yaml; sqlite inspection can still work without it.",
    )
    add_check(
        checks,
        "sqlite storage",
        "ok" if storage_files or is_mcap_storage else "missing",
        (
            ", ".join(str(path) for path in storage_files)
            if storage_files
            else ("not required for MCAP storage" if is_mcap_storage else "no .db3/.sqlite3 files found")
        ),
    )
    if is_mcap_storage or mcap_files:
        add_check(
            checks,
            "mcap storage",
            "ok" if mcap_files else "missing",
            ", ".join(str(path) for path in mcap_files) if mcap_files else "no .mcap files found",
        )

    topic_records, sqlite_errors = read_sqlite_topics(storage_files)
    if sqlite_errors:
        for error in sqlite_errors:
            add_check(checks, "sqlite read", "warn", error)
    else:
        add_check(
            checks,
            "sqlite read",
            "ok" if storage_files or is_mcap_storage else "missing",
            (
                f"{len(storage_files)} storage file(s)"
                if storage_files
                else ("skipped; MCAP storage" if is_mcap_storage else "skipped; using metadata.yaml fallback")
            ),
        )

    metadata_topic_records = {
        canonical_topic(str(topic["name"])): topic
        for topic in metadata.get("topics", [])
        if isinstance(topic, dict) and isinstance(topic.get("name"), str)
    }
    mcap_topic_records: dict[str, dict[str, Any]] = {}
    mcap_errors: list[str] = []
    if not topic_records and mcap_files:
        mcap_topic_records, mcap_errors = read_mcap_topics(mcap_files, metadata_topic_records)
        if mcap_errors:
            for error in mcap_errors:
                add_check(
                    checks,
                    "mcap read",
                    "warn",
                    error,
                    hint=(
                        "Install the Python `mcap` package for MCAP message-level rate/gap checks, or export sqlite3 storage."
                        if error.startswith("mcap reader unavailable:")
                        else None
                    ),
                )
        else:
            add_check(checks, "mcap read", "ok", f"{len(mcap_files)} MCAP file(s)")

    sqlite_topics = [
        summarize_topic(record, gap_factor=args.gap_factor, gap_min_s=args.gap_min_s)
        for record in topic_records.values()
    ]
    mcap_topics = [
        summarize_topic(record, gap_factor=args.gap_factor, gap_min_s=args.gap_min_s)
        for record in mcap_topic_records.values()
    ]
    metadata_topics = [
        summarize_metadata_topic(topic)
        for topic in metadata.get("topics", [])
        if isinstance(topic, dict)
    ]
    topics = sqlite_topics if sqlite_topics else (mcap_topics if mcap_topics else metadata_topics)
    topics.sort(key=lambda item: str(item["name"]))
    if sqlite_topics:
        diagnostic_depth = "message-data"
        message_source = "sqlite"
        add_check(checks, "message-level timing", "ok", "rates and timestamp gaps inspected from sqlite messages")
    elif mcap_topics:
        diagnostic_depth = "message-data"
        message_source = "mcap"
        add_check(checks, "message-level timing", "ok", "rates and timestamp gaps inspected from MCAP messages")
    elif metadata_topics:
        diagnostic_depth = "metadata"
        message_source = "metadata"
        add_check(
            checks,
            "metadata topics",
            "ok",
            f"{len(metadata_topics)} topic(s) with message counts from metadata.yaml",
        )
        add_check(
            checks,
            "message-level timing",
            "warn",
            "metadata-only inspection; rates and timestamp gaps were not measured",
            hint="Re-record/export with sqlite3 storage for gap/rate gates, or add an MCAP reader to inspect message timestamps.",
        )
    else:
        diagnostic_depth = "none"
        message_source = "none"
        add_check(checks, "message-level timing", "missing", "no topics found in sqlite storage or metadata.yaml")
    topic_status = build_topic_status(topics)

    for key, status in topic_status.items():
        expected = EXPECTED_TOPICS[key]
        if status["status"] == "ok":
            rate = status.get("mean_rate_hz")
            rate_detail = f"rate={rounded(rate)} Hz" if rate is not None else "rate=n/a"
            detail = (
                f"{status['topic']} count={status['message_count']}, "
                f"{rate_detail}"
            )
            if status.get("source") == "metadata":
                detail += " (metadata)"
            add_check(checks, f"{key} topic", "ok", detail)
        elif status["status"] == "warn":
            add_check(
                checks,
                f"{key} topic",
                "warn",
                f"{status['topic']} type={status.get('actual_type')} expected={expected['type']}",
            )
        else:
            add_check(
                checks,
                f"{key} topic",
                "missing",
                f"{expected['topic']} not found or empty",
                hint=(
                    "Record this topic next time. "
                    "Use `/gnss/raw_binary` when replayable decoder research is required."
                    if key == "raw_binary"
                    else None
                ),
            )

    gap_topics = [
        topic
        for topic in topics
        if isinstance(topic.get("gap_count"), int) and int(topic.get("gap_count", 0)) > 0
    ]
    if diagnostic_depth == "metadata":
        add_check(
            checks,
            "topic gaps",
            "warn",
            "not measured from MCAP metadata-only inspection",
            hint="Use sqlite storage or message-level MCAP reading when realtime gap diagnostics are required.",
        )
    elif gap_topics:
        worst = max(gap_topics, key=lambda item: float(item.get("max_gap_s") or 0.0))
        add_check(
            checks,
            "topic gaps",
            "warn",
            f"{len(gap_topics)} topic(s) with gaps; worst {worst['name']} max_gap={worst.get('max_gap_s')} s",
        )
    else:
        add_check(checks, "topic gaps", "ok" if topics else "missing", "no timestamp gaps above threshold")

    start_ns_values = [topic["first_timestamp_ns"] for topic in topics if topic.get("first_timestamp_ns") is not None]
    end_ns_values = [topic["last_timestamp_ns"] for topic in topics if topic.get("last_timestamp_ns") is not None]
    start_ns = min(start_ns_values) if start_ns_values else None
    end_ns = max(end_ns_values) if end_ns_values else None
    duration_s = (end_ns - start_ns) / 1e9 if start_ns is not None and end_ns is not None else None
    if duration_s is None and diagnostic_depth == "metadata":
        start_ns = metadata.get("start_time_ns") if isinstance(metadata.get("start_time_ns"), int) else None
        metadata_duration_s = metadata.get("duration_s")
        duration_s = float(metadata_duration_s) if isinstance(metadata_duration_s, (int, float)) else None
        end_ns = int(start_ns + duration_s * 1e9) if start_ns is not None and duration_s is not None else None
    total_messages = sum(int(topic.get("message_count", 0)) for topic in topics)
    if total_messages == 0 and isinstance(metadata.get("message_count"), int):
        total_messages = int(metadata["message_count"])
    replayable_raw_binary = topic_status["raw_binary"]["status"] == "ok"

    if diagnostic_depth == "message-data" and replayable_raw_binary and topic_status["fix"]["status"] == "ok":
        status = "ready"
    elif diagnostic_depth == "metadata" and total_messages > 0:
        status = "partial-metadata"
    elif total_messages > 0:
        status = "partial"
    else:
        status = "missing"

    return {
        "tool": "ros2-bag-doctor",
        "bag": str(bag_dir),
        "metadata": metadata,
        "storage_files": [str(path) for path in storage_files],
        "mcap_files": [str(path) for path in mcap_files],
        "storage_identifier": storage_identifier,
        "diagnostic_depth": diagnostic_depth,
        "message_source": message_source,
        "status": status,
        "ok": status == "ready",
        "replayable_raw_binary": replayable_raw_binary,
        "message_count": total_messages,
        "topic_count": len(topics),
        "start_time_ns": start_ns,
        "end_time_ns": end_ns,
        "duration_s": rounded(duration_s),
        "expected_topics": EXPECTED_TOPICS,
        "topic_status": topic_status,
        "topics": topics,
        "checks": checks,
        "commands": build_commands(bag_dir, args.protocol, args.output_pos, args.output_kml),
        "gap_factor": args.gap_factor,
        "gap_min_s": args.gap_min_s,
    }


def render_text(payload: dict[str, Any]) -> str:
    checks = payload["checks"]
    width = max(len(item["status"]) for item in checks)
    lines = [
        "libgnss++ ROS2 bag doctor",
        "",
        f"bag: {payload['bag']}",
        f"status: {payload['status']}",
        f"messages: {payload['message_count']} across {payload['topic_count']} topic(s)",
        f"duration: {payload.get('duration_s') if payload.get('duration_s') is not None else 'n/a'} s",
        "",
    ]
    for item in checks:
        lines.append(f"[{item['status']:<{width}}] {item['name']}: {item['detail']}")
        if item.get("hint"):
            lines.append(f"{'':<{width + 3}} hint: {item['hint']}")
    lines.extend(["", "Topics:"])
    for topic in payload["topics"]:
        lines.append(
            "  "
            f"{topic['name']} ({topic.get('type')}) "
            f"count={topic['message_count']} "
            f"rate={topic.get('mean_rate_hz') if topic.get('mean_rate_hz') is not None else 'n/a'} Hz "
            f"max_gap={topic.get('max_gap_s') if topic.get('max_gap_s') is not None else 'n/a'} s"
        )
    commands = payload["commands"]
    lines.extend(
        [
            "",
            "Next commands:",
            f"  {commands['info']}",
            f"  {commands['play']}",
            f"  {commands['decode']}",
            f"  {commands['record_next_time']}",
        ]
    )
    if not payload["replayable_raw_binary"]:
        lines.extend(
            [
                "",
                "Warning: /gnss/raw_binary is missing or empty, so decoder changes cannot be replayed from this bag.",
            ]
        )
    if payload.get("diagnostic_depth") == "metadata":
        lines.extend(
            [
                "",
                "Note: this is a metadata-only inspection; message rates and timestamp gaps were not measured.",
            ]
        )
    return "\n".join(lines)


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        prog=os.environ.get("GNSS_CLI_NAME"),
        description="Inspect ROS2 GNSS bag topics, rates, gaps, and replayability without ROS2 Python imports.",
    )
    parser.add_argument(
        "--bag",
        type=Path,
        required=True,
        help="ROS2 bag directory containing metadata.yaml plus sqlite storage or MCAP metadata.",
    )
    parser.add_argument("--json", action="store_true", help="Emit machine-readable JSON.")
    parser.add_argument("--summary-json", type=Path, default=None, help="Optional path to write the JSON summary artifact.")
    parser.add_argument("--strict", action="store_true", help="Return non-zero unless the bag has replayable raw binary plus fix topics.")
    parser.add_argument("--gap-factor", type=float, default=3.0, help="Flag gaps larger than median_period * factor.")
    parser.add_argument("--gap-min-s", type=float, default=1.0, help="Minimum timestamp gap in seconds before warning.")
    parser.add_argument("--protocol", choices=("auto", "ubx", "sbf"), default="auto", help="Protocol used in the suggested decoder command.")
    parser.add_argument("--output-pos", type=Path, default=Path("output/ros2_bag_replay.pos"), help="Suggested decoder .pos output.")
    parser.add_argument("--output-kml", type=Path, default=Path("output/ros2_bag_replay.kml"), help="Suggested decoder KML output.")
    return parser.parse_args()


def main() -> int:
    args = parse_args()
    payload = build_payload(args)
    if args.summary_json is not None:
        args.summary_json.parent.mkdir(parents=True, exist_ok=True)
        args.summary_json.write_text(json.dumps(payload, indent=2, sort_keys=True) + "\n", encoding="utf-8")
    if args.json:
        print(json.dumps(payload, indent=2, sort_keys=True))
    else:
        print(render_text(payload))
    if args.strict and payload["status"] != "ready":
        return 1
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
