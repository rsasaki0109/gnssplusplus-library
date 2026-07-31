#!/usr/bin/env python3
"""Validate machine-readable CLI UX contracts for local or Docker commands."""

from __future__ import annotations

import argparse
import json
import subprocess
import sys
from collections.abc import Sequence
from dataclasses import dataclass
from pathlib import Path
from typing import Any


ROOT_DIR = Path(__file__).resolve().parents[2]
LOCAL_CLI = ROOT_DIR / "apps" / "gnss.py"
VALID_COMMAND_KINDS = {"binary", "builtin", "python"}


@dataclass(frozen=True)
class CliInvocation:
    command: tuple[str, ...]
    expected_returncode: int = 0


def run_invocation(invocation: CliInvocation) -> subprocess.CompletedProcess[str]:
    return subprocess.run(
        list(invocation.command),
        cwd=ROOT_DIR,
        check=False,
        capture_output=True,
        text=True,
    )


def load_stdout_json(result: subprocess.CompletedProcess[str], label: str) -> Any:
    if "Traceback (most recent call last)" in result.stdout + result.stderr:
        raise AssertionError(f"{label}: command printed a Python traceback")
    try:
        return json.loads(result.stdout)
    except json.JSONDecodeError as exc:
        raise AssertionError(
            f"{label}: stdout is not valid JSON: {exc}; stdout={result.stdout!r}; stderr={result.stderr!r}"
        ) from exc


def assert_returncode_and_stderr(
    result: subprocess.CompletedProcess[str],
    label: str,
    *,
    expected_returncode: int,
    expect_empty_stderr: bool = True,
) -> None:
    if result.returncode != expected_returncode:
        raise AssertionError(
            f"{label}: expected return code {expected_returncode}, got {result.returncode}; "
            f"stdout={result.stdout!r}; stderr={result.stderr!r}"
        )
    if expect_empty_stderr and result.stderr:
        raise AssertionError(f"{label}: expected empty stderr, got {result.stderr!r}")


def validate_commands_payload(payload: Any, label: str, *, expected_filters: dict[str, Any]) -> set[str]:
    if not isinstance(payload, dict):
        raise AssertionError(f"{label}: payload must be an object")
    if payload.get("schema_version") != 1:
        raise AssertionError(f"{label}: expected schema_version=1")
    if payload.get("filters") != expected_filters:
        raise AssertionError(f"{label}: unexpected filters: {payload.get('filters')!r}")

    commands = payload.get("commands")
    if not isinstance(commands, list):
        raise AssertionError(f"{label}: commands must be a list")
    if payload.get("count") != len(commands):
        raise AssertionError(f"{label}: count does not match command list length")

    names: list[str] = []
    for index, item in enumerate(commands):
        if not isinstance(item, dict):
            raise AssertionError(f"{label}: commands[{index}] must be an object")
        if set(item) != {"kind", "name", "summary"}:
            raise AssertionError(f"{label}: commands[{index}] has unexpected keys {sorted(item)}")
        if item["kind"] not in VALID_COMMAND_KINDS:
            raise AssertionError(f"{label}: commands[{index}] has invalid kind {item['kind']!r}")
        for field in ("name", "summary"):
            if not isinstance(item[field], str) or not item[field].strip():
                raise AssertionError(f"{label}: commands[{index}].{field} must be a non-empty string")
        names.append(item["name"])

    if names != sorted(names):
        raise AssertionError(f"{label}: command names must be sorted")
    if len(names) != len(set(names)):
        raise AssertionError(f"{label}: command names must be unique")
    return set(names)


def validate_doctor_payload(payload: Any, label: str, *, known_commands: set[str]) -> None:
    if not isinstance(payload, dict):
        raise AssertionError(f"{label}: payload must be an object")
    if set(payload) != {"checks", "next_commands", "ok", "root"}:
        raise AssertionError(f"{label}: unexpected keys {sorted(payload)}")
    if not isinstance(payload["ok"], bool):
        raise AssertionError(f"{label}: ok must be a boolean")
    if not isinstance(payload["root"], str) or not payload["root"]:
        raise AssertionError(f"{label}: root must be a non-empty string")

    checks = payload["checks"]
    if not isinstance(checks, list) or not checks:
        raise AssertionError(f"{label}: checks must be a non-empty list")
    for index, check in enumerate(checks):
        if not isinstance(check, dict):
            raise AssertionError(f"{label}: checks[{index}] must be an object")
        for key in ("name", "status", "detail"):
            if not isinstance(check.get(key), str) or not check[key].strip():
                raise AssertionError(f"{label}: checks[{index}].{key} must be a non-empty string")

    next_commands = payload["next_commands"]
    if not isinstance(next_commands, list) or not next_commands:
        raise AssertionError(f"{label}: next_commands must be a non-empty list")
    stale_next_commands: list[str] = []
    for command in next_commands:
        if not isinstance(command, str) or not command.strip():
            raise AssertionError(f"{label}: next_commands entries must be non-empty strings")
        prefix = "python3 apps/gnss.py "
        if not command.startswith(prefix):
            continue
        dispatcher_command = command[len(prefix):].split(None, 1)[0]
        if dispatcher_command not in known_commands:
            stale_next_commands.append(command)
    if stale_next_commands:
        raise AssertionError(f"{label}: stale next_commands: {stale_next_commands}")


def validate_invocation_json(
    invocation: CliInvocation,
    label: str,
    *,
    expected_filters: dict[str, Any] | None = None,
    known_commands: set[str] | None = None,
) -> Any:
    result = run_invocation(invocation)
    assert_returncode_and_stderr(
        result,
        label,
        expected_returncode=invocation.expected_returncode,
    )
    payload = load_stdout_json(result, label)
    if expected_filters is not None:
        return validate_commands_payload(payload, label, expected_filters=expected_filters)
    if known_commands is not None:
        validate_doctor_payload(payload, label, known_commands=known_commands)
    return payload


def build_command(prefix: Sequence[str], *args: str) -> tuple[str, ...]:
    return (*prefix, *args)


def validate_contract(prefix: Sequence[str]) -> None:
    all_commands = validate_invocation_json(
        CliInvocation(build_command(prefix, "commands", "--json")),
        "commands --json",
        expected_filters={"kind": None},
    )
    filtered_commands = validate_invocation_json(
        CliInvocation(build_command(prefix, "commands", "--query", "ppp", "--limit", "3", "--json")),
        "commands --query ppp --limit 3 --json",
        expected_filters={"kind": None, "query": "ppp", "limit": 3},
    )
    if len(filtered_commands) > 3:
        raise AssertionError("filtered commands exceeded --limit 3")
    if not filtered_commands:
        raise AssertionError("filtered commands should include at least one PPP command")

    validate_doctor = CliInvocation(
        build_command(prefix, "doctor", "--root", "/tmp/gnsspp-missing-root", "--json", "--strict"),
        expected_returncode=1,
    )
    validate_invocation_json(validate_doctor, "doctor missing-root --json --strict", known_commands=all_commands)


def parse_args(argv: Sequence[str]) -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        "--cli",
        nargs=argparse.REMAINDER,
        default=[sys.executable, str(LOCAL_CLI)],
        help="CLI prefix to execute, for example: python3 apps/gnss.py",
    )
    return parser.parse_args(argv)


def main(argv: Sequence[str] | None = None) -> int:
    args = parse_args(sys.argv[1:] if argv is None else argv)
    validate_contract(tuple(args.cli))
    print("CLI JSON contract validated.")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
