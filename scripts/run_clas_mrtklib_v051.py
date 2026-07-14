#!/usr/bin/env python3
"""Run one canonical MRTKLIB v0.5.1 CLAS parity profile."""

from __future__ import annotations

import argparse
import hashlib
import os
import shlex
import subprocess
import sys
from pathlib import Path
from typing import Any

try:
    import tomllib
except ModuleNotFoundError:  # pragma: no cover - Python 3.10 fallback
    import tomli as tomllib


ROOT_DIR = Path(__file__).resolve().parents[1]
DEFAULT_CONFIG = ROOT_DIR / "configs" / "clas_mrtklib_v051.example.toml"


def resolve_path(value: str) -> Path:
    path = Path(os.path.expandvars(os.path.expanduser(value)))
    return path if path.is_absolute() else ROOT_DIR / path


def file_md5(path: Path) -> str:
    digest = hashlib.md5()  # noqa: S324 - fixture identity, not security
    with path.open("rb") as handle:
        for chunk in iter(lambda: handle.read(1024 * 1024), b""):
            digest.update(chunk)
    return digest.hexdigest()


def load_config(path: Path) -> dict[str, Any]:
    try:
        with path.open("rb") as handle:
            payload = tomllib.load(handle)
    except FileNotFoundError as exc:
        raise SystemExit(f"Config not found: {path}") from exc
    except tomllib.TOMLDecodeError as exc:
        raise SystemExit(f"Invalid TOML in {path}: {exc}") from exc
    if not isinstance(payload.get("runner"), dict) or not isinstance(
        payload.get("profiles"), dict
    ):
        raise SystemExit(f"Config must contain [runner] and [profiles.*]: {path}")
    return payload


def profile_command(config: dict[str, Any], profile_name: str) -> tuple[list[str], dict[str, str], Path]:
    profiles = config["profiles"]
    if profile_name not in profiles:
        choices = ", ".join(sorted(profiles))
        raise SystemExit(f"Unknown profile {profile_name!r}; choose one of: {choices}")
    runner = config["runner"]
    profile = profiles[profile_name]
    required = ("obs", "nav", "ssr", "out", "args")
    missing = [key for key in required if key not in profile]
    if missing:
        raise SystemExit(f"Profile {profile_name!r} is missing: {', '.join(missing)}")

    binary = resolve_path(str(runner.get("gnss_ppp", "build/apps/gnss_ppp")))
    obs = resolve_path(str(profile["obs"]))
    nav = resolve_path(str(profile["nav"]))
    ssr = resolve_path(str(profile["ssr"]))
    out = resolve_path(str(profile["out"]))
    for label, path in (("gnss_ppp", binary), ("obs", obs), ("nav", nav), ("ssr", ssr)):
        if not path.exists():
            raise SystemExit(f"{profile_name}: {label} not found: {path}")

    expected_input_md5 = str(profile.get("ssr_md5", "")).lower()
    if expected_input_md5:
        actual = file_md5(ssr)
        if actual != expected_input_md5:
            raise SystemExit(
                f"{profile_name}: SSR MD5 mismatch for {ssr}: "
                f"expected {expected_input_md5}, got {actual}"
            )

    args = profile["args"]
    if not isinstance(args, list) or not all(isinstance(value, str) for value in args):
        raise SystemExit(f"{profile_name}: args must be an array of strings")
    command = [
        str(binary),
        "--obs",
        str(obs),
        "--nav",
        str(nav),
        "--ssr",
        str(ssr),
        "--out",
        str(out),
        *args,
    ]
    env = os.environ.copy()
    env_names = profile.get("env", [])
    if not isinstance(env_names, list) or not all(isinstance(value, str) for value in env_names):
        raise SystemExit(f"{profile_name}: env must be an array of runner.env key names")
    configured_env = runner.get("env", {})
    for name in env_names:
        if name not in configured_env:
            raise SystemExit(f"{profile_name}: runner.env has no value for {name}")
        env[name] = str(configured_env[name])
    return command, env, out


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser()
    parser.add_argument("profile", help="Profile key under [profiles] in the TOML file.")
    parser.add_argument("--config", type=Path, default=DEFAULT_CONFIG)
    parser.add_argument("--print-command", action="store_true")
    parser.add_argument("--skip-output-md5", action="store_true")
    return parser.parse_args()


def main() -> int:
    args = parse_args()
    config = load_config(args.config)
    command, env, out = profile_command(config, args.profile)
    profile = config["profiles"][args.profile]
    env_prefix = [f"{name}={shlex.quote(env[name])}" for name in profile.get("env", [])]
    rendered = " ".join([*env_prefix, *(shlex.quote(token) for token in command)])
    if args.print_command:
        print(rendered)
        return 0

    out.parent.mkdir(parents=True, exist_ok=True)
    print(f"[{args.profile}] {rendered}", flush=True)
    completed = subprocess.run(command, cwd=ROOT_DIR, env=env, check=False)
    if completed.returncode != 0:
        return completed.returncode

    expected_output_md5 = str(profile.get("expected_output_md5", "")).lower()
    if expected_output_md5 and not args.skip_output_md5:
        actual = file_md5(out)
        if actual != expected_output_md5:
            print(
                f"{args.profile}: output MD5 mismatch: expected "
                f"{expected_output_md5}, got {actual}",
                file=sys.stderr,
            )
            return 2
        print(f"[{args.profile}] output MD5 OK: {actual}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
