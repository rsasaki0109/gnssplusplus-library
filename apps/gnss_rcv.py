#!/usr/bin/env python3
"""Config-driven receiver controller for gnss_live."""

from __future__ import annotations

import argparse
import datetime as dt
import glob
import json
import os
import shlex
import signal
import subprocess
import sys
import time
from pathlib import Path


ROOT_DIR = Path(__file__).resolve().parents[1]
APPS_DIR = Path(__file__).resolve().parent
EXE_SUFFIX = ".exe" if os.name == "nt" else ""
BUILD_CONFIGS = ("Release", "RelWithDebInfo", "Debug", "MinSizeRel")
RUNNING_STATES = {"starting", "running", "restarting", "stopping"}
TRUE_VALUES = {"1", "true", "yes", "on"}
CONSOLE_COMMANDS = (
    "status",
    "tail",
    "start",
    "stop",
    "restart",
    "reload",
    "show-config",
    "show-command",
    "help",
    "quit",
    "exit",
)

ALLOWED_KEYS = {
    "rover_rtcm",
    "rover_ubx",
    "rover_ubx_baud",
    "base_rtcm",
    "nav_rinex",
    "out",
    "format",
    "max_epochs",
    "rover_message_limit",
    "base_message_limit",
    "base_hold_seconds",
    "base_ecef",
    "ratio",
    "min_ar_sats",
    "elevation_mask_deg",
    "glonass_ar",
    "quiet",
    "verbose",
    "no_glonass",
    "no_beidou",
    "no_base_interp",
    "auto_restart",
    "restart_delay_seconds",
    "restart_max_attempts",
}


def find_binary(target_name: str) -> str | None:
    filename = target_name + EXE_SUFFIX
    candidates = [
        APPS_DIR / filename,
        ROOT_DIR / "build" / "apps" / filename,
    ]
    for config in BUILD_CONFIGS:
        candidates.extend(
            [
                ROOT_DIR / "build" / "apps" / config / filename,
                ROOT_DIR / "build" / config / "apps" / filename,
                ROOT_DIR / "build" / config / filename,
            ]
        )
    for candidate in candidates:
        if candidate.is_file():
            return str(candidate)

    hits = sorted(
        path for path in glob.glob(str(ROOT_DIR / "build" / "**" / filename), recursive=True)
        if os.path.isfile(path)
    )
    return hits[0] if hits else None


def usage_name() -> str:
    return os.environ.get("GNSS_CLI_NAME", "gnss rcv")


def parse_config_file(path: Path) -> dict[str, str]:
    config: dict[str, str] = {}
    for line_no, raw_line in enumerate(path.read_text(encoding="utf-8").splitlines(), start=1):
        line = raw_line.strip()
        if not line or line.startswith("#") or line.startswith(";"):
            continue
        if "=" not in line:
            raise ValueError(f"{path}:{line_no}: expected key=value")
        key, value = line.split("=", 1)
        key = key.strip()
        value = value.strip()
        if key not in ALLOWED_KEYS:
            raise ValueError(f"{path}:{line_no}: unsupported key `{key}`")
        config[key] = value
    return config


def apply_overrides(config: dict[str, str], overrides: list[str]) -> dict[str, str]:
    merged = dict(config)
    for item in overrides:
        if "=" not in item:
            raise ValueError(f"override must be key=value: {item}")
        key, value = item.split("=", 1)
        key = key.strip()
        value = value.strip()
        if key not in ALLOWED_KEYS:
            raise ValueError(f"unsupported override key `{key}`")
        merged[key] = value
    return merged


def build_live_command(config: dict[str, str]) -> list[str]:
    binary = find_binary("gnss_live")
    if binary is None:
        raise FileNotFoundError(
            "built binary not found for gnss_live. Run `cmake --build build --target gnss_live` first."
        )

    def require(key: str) -> str:
        value = config.get(key, "").strip()
        if not value:
            raise ValueError(f"missing required config key `{key}`")
        return value

    rover_rtcm = config.get("rover_rtcm", "").strip()
    rover_ubx = config.get("rover_ubx", "").strip()
    if bool(rover_rtcm) == bool(rover_ubx):
        raise ValueError("choose exactly one of `rover_rtcm` or `rover_ubx`")

    command = [binary]
    if rover_rtcm:
        command.extend(["--rover-rtcm", rover_rtcm])
    else:
        command.extend(["--rover-ubx", rover_ubx])
        if value := config.get("rover_ubx_baud"):
            command.extend(["--rover-ubx-baud", value])

    command.extend(["--base-rtcm", require("base_rtcm")])

    if value := config.get("nav_rinex"):
        command.extend(["--nav-rinex", value])
    if value := config.get("out"):
        command.extend(["--out", value])
    if value := config.get("format"):
        command.extend(["--format", value])
    if value := config.get("max_epochs"):
        command.extend(["--max-epochs", value])
    if value := config.get("rover_message_limit"):
        command.extend(["--rover-message-limit", value])
    if value := config.get("base_message_limit"):
        command.extend(["--base-message-limit", value])
    if value := config.get("base_hold_seconds"):
        command.extend(["--base-hold-seconds", value])
    if value := config.get("base_ecef"):
        parts = value.split()
        if len(parts) != 3:
            raise ValueError("base_ecef must contain three space-separated numbers")
        command.extend(["--base-ecef", *parts])
    if value := config.get("ratio"):
        command.extend(["--ratio", value])
    if value := config.get("min_ar_sats"):
        command.extend(["--min-ar-sats", value])
    if value := config.get("elevation_mask_deg"):
        command.extend(["--elevation-mask-deg", value])
    if value := config.get("glonass_ar"):
        command.extend(["--glonass-ar", value])

    bool_flags = {
        "quiet": "--quiet",
        "verbose": "--verbose",
        "no_glonass": "--no-glonass",
        "no_beidou": "--no-beidou",
        "no_base_interp": "--no-base-interp",
    }
    for key, flag in bool_flags.items():
        if config.get(key, "").strip().lower() in {"1", "true", "yes", "on"}:
            command.append(flag)

    return command


def write_status(path: Path, payload: dict[str, object]) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    temp_path = path.with_name(path.name + ".tmp")
    temp_path.write_text(json.dumps(payload, indent=2, sort_keys=True) + "\n", encoding="utf-8")
    os.replace(temp_path, path)


def read_status(path: Path) -> dict[str, object]:
    return json.loads(path.read_text(encoding="utf-8"))


def is_truthy(value: str) -> bool:
    return value.strip().lower() in TRUE_VALUES


def resolve_status_path(config_path: Path | None, explicit_path: str | None) -> Path | None:
    if explicit_path:
        return Path(explicit_path)
    if config_path is None:
        return None
    return config_path.with_suffix(".status.json")


def resolve_log_path(config_path: Path | None, explicit_path: str | None) -> Path | None:
    if explicit_path:
        return Path(explicit_path)
    if config_path is None:
        return None
    return config_path.with_suffix(".log")


def process_is_running(pid: int) -> bool:
    if pid <= 0:
        return False
    try:
        os.kill(pid, 0)
    except OSError:
        return False
    return True


def parse_timestamp(value: object) -> dt.datetime | None:
    if not isinstance(value, str) or not value:
        return None
    try:
        return dt.datetime.fromisoformat(value)
    except ValueError:
        return None


def parse_config_float(config: dict[str, str], key: str, default: float) -> float:
    raw = config.get(key, "").strip()
    if not raw:
        return default
    try:
        return float(raw)
    except ValueError as exc:
        raise ValueError(f"config `{key}` must be a number") from exc


def parse_config_int(config: dict[str, str], key: str, default: int) -> int:
    raw = config.get(key, "").strip()
    if not raw:
        return default
    try:
        return int(raw)
    except ValueError as exc:
        raise ValueError(f"config `{key}` must be an integer") from exc


def read_log_tail(path: Path, max_lines: int) -> list[str]:
    if max_lines <= 0 or not path.exists():
        return []
    lines = path.read_text(encoding="utf-8", errors="replace").splitlines()
    if max_lines >= len(lines):
        return lines
    return lines[-max_lines:]


def resolve_managed_config_path(raw_config: str | None, status_path: Path | None) -> Path:
    if raw_config is not None:
        return Path(raw_config)
    if status_path is None or not status_path.exists():
        raise ValueError("`--config` is required when no status file is available")
    payload = read_status(status_path)
    config_path = payload.get("config_path")
    if not isinstance(config_path, str) or not config_path:
        raise ValueError("status file does not contain `config_path`; pass `--config` explicitly")
    return Path(config_path)


def extract_summary_metrics(line: str) -> dict[str, object]:
    metrics: dict[str, object] = {}
    if not line.startswith("summary:"):
        return metrics
    for token in line[len("summary:"):].strip().split():
        if "=" not in token:
            continue
        key, value = token.split("=", 1)
        try:
            if "." in value:
                metrics[key] = float(value)
            else:
                metrics[key] = int(value)
        except ValueError:
            metrics[key] = value
    return metrics


def build_argument_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(
        prog=usage_name(),
        description="Run or manage gnss_live from an rtkrcv-style config file.",
    )
    parser.add_argument(
        "action",
        nargs="?",
        choices=("run", "start", "status", "stop", "restart", "reload", "console"),
        default="run",
        help="Receiver action. Defaults to `run` for backward compatibility.",
    )
    parser.add_argument("--config", help="Path to key=value receiver config file.")
    parser.add_argument(
        "--set",
        dest="overrides",
        action="append",
        default=[],
        help="Override config values with key=value.",
    )
    parser.add_argument(
        "--status-out",
        help="Write or read JSON receiver status from this file. Defaults to <config>.status.json.",
    )
    parser.add_argument(
        "--log-out",
        help="Append run output to this file. Defaults to <config>.log for `start`.",
    )
    parser.add_argument(
        "--dry-run",
        action="store_true",
        help="Print the resolved config and command without executing it.",
    )
    parser.add_argument(
        "--wait-seconds",
        type=float,
        default=0.0,
        help="For `status`, poll until exit or timeout. For `restart`, wait this long after stop before start.",
    )
    parser.add_argument(
        "--poll-seconds",
        type=float,
        default=0.2,
        help="Polling interval in seconds for `status --wait-seconds`.",
    )
    parser.add_argument(
        "--tail-log-lines",
        type=int,
        default=0,
        help="For `status`, include the last N log lines from `log_path` in the JSON output.",
    )
    return parser


def require_config_path(raw_path: str | None) -> Path:
    if raw_path is None:
        raise ValueError("`--config` is required for this action")
    return Path(raw_path)


def load_runtime(config_path: Path, overrides: list[str]) -> tuple[dict[str, str], list[str]]:
    config = parse_config_file(config_path)
    config = apply_overrides(config, overrides)
    command = build_live_command(config)
    return config, command


def build_run_payload(
    config_path: Path,
    config: dict[str, str],
    command: list[str],
    status_path: Path | None,
    log_path: Path | None,
) -> dict[str, object]:
    payload: dict[str, object] = {
        "action": "run",
        "config_path": str(config_path),
        "config": config,
        "command": command,
    }
    if status_path is not None:
        payload["status_path"] = str(status_path)
    if log_path is not None:
        payload["log_path"] = str(log_path)
    return payload


def clone_args(args: argparse.Namespace, action: str, **overrides: object) -> argparse.Namespace:
    payload = vars(args).copy()
    payload["action"] = action
    payload.update(overrides)
    return argparse.Namespace(**payload)


def write_runtime_status(
    status_path: Path | None,
    state: str,
    started_at: str,
    config_path: Path,
    config: dict[str, str],
    command: list[str],
    pid: int,
    log_path: Path | None,
    last_summary: str = "",
    returncode: int | None = None,
    extra_fields: dict[str, object] | None = None,
) -> None:
    if status_path is None:
        return
    payload: dict[str, object] = {
        "state": state,
        "started_at": started_at,
        "pid": pid,
        "config_path": str(config_path),
        "config": config,
        "command": command,
    }
    if log_path is not None:
        payload["log_path"] = str(log_path)
    if returncode is not None:
        payload["returncode"] = returncode
        payload["finished_at"] = dt.datetime.now(dt.timezone.utc).isoformat()
    if last_summary:
        payload["summary_line"] = last_summary
        payload["summary_metrics"] = extract_summary_metrics(last_summary)
    if extra_fields is not None:
        payload.update(extra_fields)
    write_status(status_path, payload)


def run_receiver(args: argparse.Namespace) -> int:
    config_path = require_config_path(args.config)
    config, command = load_runtime(config_path, args.overrides)
    status_path = resolve_status_path(config_path, args.status_out)
    log_path = Path(args.log_out) if args.log_out else None

    resolved = {
        **build_run_payload(config_path, config, command, status_path, log_path),
        "status_path": str(status_path) if status_path is not None else None,
        "log_path": str(log_path) if log_path is not None else None,
    }
    if args.dry_run:
        print(json.dumps(resolved, indent=2, sort_keys=True))
        return 0

    started_at = dt.datetime.now(dt.timezone.utc).isoformat()
    auto_restart = is_truthy(config.get("auto_restart", ""))
    restart_delay_seconds = max(0.0, parse_config_float(config, "restart_delay_seconds", 1.0))
    restart_max_attempts = max(0, parse_config_int(config, "restart_max_attempts", 0))
    restart_count = 0
    write_runtime_status(
        status_path=status_path,
        state="running",
        started_at=started_at,
        config_path=config_path,
        config=config,
        command=command,
        pid=os.getpid(),
        log_path=log_path,
        extra_fields={
            "auto_restart": auto_restart,
            "restart_delay_seconds": restart_delay_seconds,
            "restart_max_attempts": restart_max_attempts,
            "restart_count": restart_count,
        },
    )

    log_handle = None
    if log_path is not None:
        log_path.parent.mkdir(parents=True, exist_ok=True)
        log_handle = log_path.open("a", encoding="utf-8")

    terminate_requested = False
    process: subprocess.Popen[str] | None = None

    def handle_termination(_signum: int, _frame: object) -> None:
        nonlocal terminate_requested, process
        terminate_requested = True
        if process is not None and process.poll() is None:
            process.terminate()

    old_sigterm = signal.getsignal(signal.SIGTERM)
    old_sigint = signal.getsignal(signal.SIGINT)
    signal.signal(signal.SIGTERM, handle_termination)
    signal.signal(signal.SIGINT, handle_termination)

    last_summary = ""
    returncode = 0
    try:
        while True:
            process = subprocess.Popen(
                command,
                cwd=str(ROOT_DIR),
                stdout=subprocess.PIPE,
                stderr=subprocess.STDOUT,
                text=True,
                bufsize=1,
            )

            assert process.stdout is not None
            last_summary = ""
            for line in process.stdout:
                print(line, end="")
                if log_handle is not None:
                    log_handle.write(line)
                    log_handle.flush()
                if line.startswith("summary:"):
                    last_summary = line.strip()

            returncode = process.wait()
            if terminate_requested or returncode == 0 or not auto_restart:
                break
            if restart_max_attempts > 0 and restart_count >= restart_max_attempts:
                break

            restart_count += 1
            next_restart_at = dt.datetime.now(dt.timezone.utc) + dt.timedelta(seconds=restart_delay_seconds)
            write_runtime_status(
                status_path=status_path,
                state="restarting",
                started_at=started_at,
                config_path=config_path,
                config=config,
                command=command,
                pid=os.getpid(),
                log_path=log_path,
                last_summary=last_summary,
                extra_fields={
                    "auto_restart": auto_restart,
                    "restart_delay_seconds": restart_delay_seconds,
                    "restart_max_attempts": restart_max_attempts,
                    "restart_count": restart_count,
                    "last_returncode": returncode,
                    "next_restart_at": next_restart_at.isoformat(),
                },
            )
            deadline = time.monotonic() + restart_delay_seconds
            while time.monotonic() < deadline:
                if terminate_requested:
                    break
                time.sleep(0.05)
            if terminate_requested:
                break
            write_runtime_status(
                status_path=status_path,
                state="running",
                started_at=started_at,
                config_path=config_path,
                config=config,
                command=command,
                pid=os.getpid(),
                log_path=log_path,
                extra_fields={
                    "auto_restart": auto_restart,
                    "restart_delay_seconds": restart_delay_seconds,
                    "restart_max_attempts": restart_max_attempts,
                    "restart_count": restart_count,
                },
            )
    finally:
        signal.signal(signal.SIGTERM, old_sigterm)
        signal.signal(signal.SIGINT, old_sigint)
        if log_handle is not None:
            log_handle.close()

    final_state = "stopped" if terminate_requested else ("completed" if returncode == 0 else "failed")
    normalized_returncode = 0 if terminate_requested else returncode
    write_runtime_status(
        status_path=status_path,
        state=final_state,
        started_at=started_at,
        config_path=config_path,
        config=config,
        command=command,
        pid=os.getpid(),
        log_path=log_path,
        last_summary=last_summary,
        returncode=normalized_returncode,
        extra_fields={
            "auto_restart": auto_restart,
            "restart_delay_seconds": restart_delay_seconds,
            "restart_max_attempts": restart_max_attempts,
            "restart_count": restart_count,
        },
    )
    return normalized_returncode


def start_receiver(args: argparse.Namespace) -> int:
    config_path = require_config_path(args.config)
    config, command = load_runtime(config_path, args.overrides)
    status_path = resolve_status_path(config_path, args.status_out)
    if status_path is None:
        raise ValueError("could not resolve status path")
    log_path = resolve_log_path(config_path, args.log_out)

    if status_path.exists():
        try:
            existing = read_status(status_path)
        except json.JSONDecodeError:
            existing = {}
        existing_pid = int(existing.get("pid", 0) or 0)
        existing_state = str(existing.get("state", ""))
        if existing_state in RUNNING_STATES and process_is_running(existing_pid):
            raise ValueError(
                f"receiver already running with pid {existing_pid}; stop it first or use a different status file"
            )

    launcher_command = [
        sys.executable,
        str(Path(__file__).resolve()),
        "run",
        "--config",
        str(config_path),
        "--status-out",
        str(status_path),
    ]
    if log_path is not None:
        launcher_command.extend(["--log-out", str(log_path)])
    for item in args.overrides:
        launcher_command.extend(["--set", item])

    resolved = {
        "action": "start",
        "config_path": str(config_path),
        "config": config,
        "command": command,
        "status_path": str(status_path),
        "log_path": str(log_path) if log_path is not None else None,
        "launcher_command": launcher_command,
    }
    if args.dry_run:
        print(json.dumps(resolved, indent=2, sort_keys=True))
        return 0

    log_handle = None
    try:
        if log_path is not None:
            log_path.parent.mkdir(parents=True, exist_ok=True)
            log_handle = log_path.open("a", encoding="utf-8")
        child = subprocess.Popen(
            launcher_command,
            cwd=str(ROOT_DIR),
            stdin=subprocess.DEVNULL,
            stdout=log_handle if log_handle is not None else subprocess.DEVNULL,
            stderr=subprocess.STDOUT,
            text=True,
            start_new_session=True,
        )
    finally:
        if log_handle is not None:
            log_handle.close()

    payload = {
        "state": "starting",
        "started_at": dt.datetime.now(dt.timezone.utc).isoformat(),
        "pid": child.pid,
        "config_path": str(config_path),
        "config": config,
        "command": command,
        "status_path": str(status_path),
        "launcher_command": launcher_command,
    }
    if log_path is not None:
        payload["log_path"] = str(log_path)
    payload["auto_restart"] = is_truthy(config.get("auto_restart", ""))
    payload["restart_delay_seconds"] = parse_config_float(config, "restart_delay_seconds", 1.0)
    payload["restart_max_attempts"] = parse_config_int(config, "restart_max_attempts", 0)
    payload["restart_count"] = 0
    write_status(status_path, payload)

    print(json.dumps(payload, indent=2, sort_keys=True))
    return 0


def status_receiver(args: argparse.Namespace) -> int:
    config_path = Path(args.config) if args.config else None
    status_path = resolve_status_path(config_path, args.status_out)
    if status_path is None:
        raise ValueError("`--status-out` or `--config` is required for `status`")
    if not status_path.exists():
        raise FileNotFoundError(f"status file not found: {status_path}")

    deadline = time.time() + max(args.wait_seconds, 0.0)
    payload = read_status(status_path)
    while True:
        pid = int(payload.get("pid", 0) or 0)
        pid_running = process_is_running(pid)
        state = str(payload.get("state", ""))
        payload["pid_running"] = pid_running
        if args.wait_seconds <= 0.0 or time.time() >= deadline:
            break
        if state not in RUNNING_STATES and not pid_running:
            break
        time.sleep(max(args.poll_seconds, 0.05))
        payload = read_status(status_path)

    started_at = parse_timestamp(payload.get("started_at"))
    finished_at = parse_timestamp(payload.get("finished_at"))
    reference_time = finished_at or dt.datetime.now(dt.timezone.utc)
    if started_at is not None:
        payload["uptime_seconds"] = max(
            0.0,
            (reference_time - started_at).total_seconds(),
        )
    next_restart_at = parse_timestamp(payload.get("next_restart_at"))
    if next_restart_at is not None:
        payload["next_restart_in_seconds"] = max(
            0.0,
            (next_restart_at - dt.datetime.now(dt.timezone.utc)).total_seconds(),
        )
    log_path_value = payload.get("log_path")
    if isinstance(log_path_value, str) and log_path_value:
        log_path = Path(log_path_value)
        payload["log_exists"] = log_path.exists()
        if log_path.exists():
            payload["log_size_bytes"] = log_path.stat().st_size
            if args.tail_log_lines > 0:
                payload["log_tail"] = read_log_tail(log_path, args.tail_log_lines)
    print(json.dumps(payload, indent=2, sort_keys=True))
    return 0


def stop_receiver_at_status_path(status_path: Path, emit_output: bool = True) -> int:
    payload = read_status(status_path)
    pid = int(payload.get("pid", 0) or 0)
    if not process_is_running(pid):
        payload["pid_running"] = False
        if payload.get("state") in RUNNING_STATES:
            payload["state"] = "stopped"
            payload["finished_at"] = dt.datetime.now(dt.timezone.utc).isoformat()
            payload["returncode"] = int(payload.get("returncode", 0) or 0)
            write_status(status_path, payload)
        if emit_output:
            print("receiver not running")
        return 0

    os.kill(pid, signal.SIGTERM)
    for _ in range(20):
        if not process_is_running(pid):
            break
        time.sleep(0.1)

    reloaded = read_status(status_path) if status_path.exists() else payload
    reloaded["pid_running"] = process_is_running(pid)
    if reloaded["pid_running"]:
        reloaded["state"] = "stopping"
        reloaded["stop_requested_at"] = dt.datetime.now(dt.timezone.utc).isoformat()
        write_status(status_path, reloaded)
        if emit_output:
            print(f"sent SIGTERM to receiver pid {pid}")
    else:
        if emit_output:
            print(f"stopped receiver pid {pid}")
    return 0


def stop_receiver(args: argparse.Namespace) -> int:
    config_path = Path(args.config) if args.config else None
    status_path = resolve_status_path(config_path, args.status_out)
    if status_path is None:
        raise ValueError("`--status-out` or `--config` is required for `stop`")
    if not status_path.exists():
        raise FileNotFoundError(f"status file not found: {status_path}")
    return stop_receiver_at_status_path(status_path, emit_output=True)


def restart_receiver(args: argparse.Namespace) -> int:
    initial_status_path = resolve_status_path(Path(args.config) if args.config else None, args.status_out)
    config_path = resolve_managed_config_path(args.config, initial_status_path)
    status_path = resolve_status_path(config_path, args.status_out)
    log_path = resolve_log_path(config_path, args.log_out)
    if status_path is None:
        raise ValueError("could not resolve status path")

    config, command = load_runtime(config_path, args.overrides)
    resolved = {
        "action": "restart",
        "config_path": str(config_path),
        "config": config,
        "command": command,
        "status_path": str(status_path),
        "log_path": str(log_path) if log_path is not None else None,
        "wait_seconds": args.wait_seconds,
    }
    if args.dry_run:
        print(json.dumps(resolved, indent=2, sort_keys=True))
        return 0

    if status_path.exists():
        stop_receiver_at_status_path(status_path, emit_output=False)
    if args.wait_seconds > 0.0:
        time.sleep(args.wait_seconds)
    return start_receiver(args)


def reload_receiver(args: argparse.Namespace) -> int:
    initial_status_path = resolve_status_path(Path(args.config) if args.config else None, args.status_out)
    config_path = resolve_managed_config_path(args.config, initial_status_path)
    status_path = resolve_status_path(config_path, args.status_out)
    if status_path is None:
        raise ValueError("could not resolve status path")
    config, command = load_runtime(config_path, args.overrides)
    log_path = resolve_log_path(config_path, args.log_out)

    resolved = {
        "action": "reload",
        "config_path": str(config_path),
        "config": config,
        "command": command,
        "status_path": str(status_path),
        "log_path": str(log_path) if log_path is not None else None,
        "wait_seconds": args.wait_seconds,
    }
    if args.dry_run:
        print(json.dumps(resolved, indent=2, sort_keys=True))
        return 0

    if status_path.exists():
        try:
            existing = read_status(status_path)
        except json.JSONDecodeError:
            existing = {}
        existing.update(
            {
                "state": "reloading",
                "reload_requested_at": dt.datetime.now(dt.timezone.utc).isoformat(),
                "config_path": str(config_path),
                "config": config,
                "command": command,
            }
        )
        if log_path is not None:
            existing["log_path"] = str(log_path)
        write_status(status_path, existing)
        stop_receiver_at_status_path(status_path, emit_output=False)
    if args.wait_seconds > 0.0:
        time.sleep(args.wait_seconds)
    args.config = str(config_path)
    return start_receiver(args)


def print_console_help() -> None:
    print("commands: status | tail [N] | start | stop | restart | reload | show-config | show-command | help | quit")


def console_receiver(args: argparse.Namespace) -> int:
    if args.config is None and args.status_out is None:
        raise ValueError("`console` requires `--config` or `--status-out`")

    print_console_help()
    prompt = "gnss-rcv> "
    interactive = sys.stdin.isatty()

    while True:
        if interactive:
            print(prompt, end="", flush=True)
        line = sys.stdin.readline()
        if not line:
            return 0
        command_line = line.strip()
        if not command_line:
            continue

        try:
            tokens = shlex.split(command_line)
        except ValueError as exc:
            print(f"error: {exc}")
            continue
        if not tokens:
            continue

        command = tokens[0].lower()
        tail_lines = args.tail_log_lines
        if command in {"quit", "exit"}:
            return 0
        if command == "help":
            print_console_help()
            continue

        try:
            if command == "status":
                status_receiver(clone_args(args, "status"))
                continue
            if command == "tail":
                if len(tokens) > 2:
                    raise ValueError("usage: tail [line_count]")
                if len(tokens) == 2:
                    tail_lines = int(tokens[1])
                status_receiver(clone_args(args, "status", tail_log_lines=tail_lines or 20))
                continue
            if command == "start":
                start_receiver(clone_args(args, "start"))
                continue
            if command == "stop":
                stop_receiver(clone_args(args, "stop"))
                continue
            if command == "restart":
                restart_receiver(clone_args(args, "restart"))
                continue
            if command == "reload":
                reload_receiver(clone_args(args, "reload"))
                continue
            if command == "show-config":
                config_path = resolve_managed_config_path(args.config, resolve_status_path(
                    Path(args.config) if args.config else None, args.status_out
                ))
                config = parse_config_file(config_path)
                config = apply_overrides(config, args.overrides)
                print(json.dumps({"config_path": str(config_path), "config": config}, indent=2, sort_keys=True))
                continue
            if command == "show-command":
                config_path = resolve_managed_config_path(args.config, resolve_status_path(
                    Path(args.config) if args.config else None, args.status_out
                ))
                config, command_payload = load_runtime(config_path, args.overrides)
                print(
                    json.dumps(
                        build_run_payload(
                            config_path,
                            config,
                            command_payload,
                            resolve_status_path(config_path, args.status_out),
                            resolve_log_path(config_path, args.log_out),
                        ),
                        indent=2,
                        sort_keys=True,
                    )
                )
                continue
            raise ValueError(
                f"unsupported console command `{command}`; available: {', '.join(CONSOLE_COMMANDS)}"
            )
        except (FileNotFoundError, ValueError) as exc:
            print(f"error: {exc}")


def main() -> int:
    parser = build_argument_parser()
    args = parser.parse_args()

    try:
        if args.action == "run":
            return run_receiver(args)
        if args.action == "start":
            return start_receiver(args)
        if args.action == "status":
            return status_receiver(args)
        if args.action == "stop":
            return stop_receiver(args)
        if args.action == "restart":
            return restart_receiver(args)
        if args.action == "reload":
            return reload_receiver(args)
        if args.action == "console":
            return console_receiver(args)
        raise ValueError(f"unsupported action `{args.action}`")
    except (FileNotFoundError, ValueError) as exc:
        print(f"Error: {exc}", file=sys.stderr)
        return 1


if __name__ == "__main__":
    raise SystemExit(main())
