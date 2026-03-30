#!/usr/bin/env python3
"""Shared TOML config parsing for selected Python CLI entrypoints."""

from __future__ import annotations

import argparse
from pathlib import Path
import sys
import tomllib
from typing import Any


def _extract_config_path(argv: list[str]) -> Path | None:
    for index, token in enumerate(argv):
        if token == "--config-toml":
            if index + 1 >= len(argv):
                raise SystemExit("--config-toml requires a path")
            return Path(argv[index + 1])
        if token.startswith("--config-toml="):
            return Path(token.split("=", 1)[1])
    return None


def _load_section(config_path: Path, section_name: str) -> dict[str, Any]:
    if not config_path.exists():
        raise SystemExit(f"Missing TOML config: {config_path}")
    try:
        with config_path.open("rb") as handle:
            payload = tomllib.load(handle)
    except tomllib.TOMLDecodeError as exc:
        raise SystemExit(f"Failed to parse TOML config {config_path}: {exc}") from exc

    if not isinstance(payload, dict):
        raise SystemExit(f"TOML config must contain a table/object: {config_path}")

    section = payload.get(section_name, payload)
    if not isinstance(section, dict):
        raise SystemExit(
            f"TOML section [{section_name}] must be a table/object in {config_path}"
        )
    return section


def _coerce_scalar(action: argparse.Action, value: Any) -> Any:
    if isinstance(action, (argparse._StoreTrueAction, argparse._StoreFalseAction)):
        return bool(value)
    if action.type is not None:
        try:
            return action.type(value)
        except (TypeError, ValueError) as exc:
            raise SystemExit(
                f"Invalid TOML value for --{action.dest.replace('_', '-')}: {value!r}"
            ) from exc
    if isinstance(action.default, Path):
        return Path(value)
    return value


def _coerce_default(action: argparse.Action, value: Any) -> Any:
    if isinstance(action, argparse._AppendAction):
        if not isinstance(value, list):
            raise SystemExit(
                f"TOML key {action.dest} must be an array because the CLI option is repeatable"
            )
        return [_coerce_scalar(action, item) for item in value]
    return _coerce_scalar(action, value)


def parse_args_with_toml(
    parser: argparse.ArgumentParser,
    section_name: str,
    argv: list[str] | None = None,
) -> argparse.Namespace:
    argv = list(sys.argv[1:] if argv is None else argv)
    config_path = _extract_config_path(argv)
    if config_path is not None:
        section = _load_section(config_path, section_name)
        defaults: dict[str, Any] = {}
        for action in parser._actions:
            if action.dest in ("help", "config_toml") or not action.dest:
                continue
            raw_value: Any | None = None
            found = False
            for key in (action.dest, action.dest.replace("_", "-")):
                if key in section:
                    raw_value = section[key]
                    found = True
                    break
            if not found:
                continue
            defaults[action.dest] = _coerce_default(action, raw_value)
            if getattr(action, "required", False):
                action.required = False
        if defaults:
            parser.set_defaults(**defaults)
    return parser.parse_args(argv)
