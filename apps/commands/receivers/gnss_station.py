#!/usr/bin/env python3
"""Small, config-driven station controller built on top of ``gnss rcv``.

``gnss rcv`` is intentionally kept as the lower-level, RTKLIB-like process
controller.  This command is the operational entry point: it validates a
station TOML file, creates a timestamped run directory, starts the existing
controller, and exposes one status/artifact contract for field runs.
"""

from __future__ import annotations

import argparse
import contextlib
import datetime as dt
import hashlib
import io
import json
import os
from pathlib import Path
import re
import shutil
import subprocess
import sys
import time
from typing import Any

try:
    import tomllib
except ModuleNotFoundError:  # pragma: no cover - Python < 3.11 compatibility.
    import tomli as tomllib

import gnss_rcv as rcv

from support.gnss_runtime import application_root


ROOT_DIR = application_root(__file__)
SCHEMA_VERSION = "station.v1"
RUNNING_STATES = {"starting", "running", "restarting", "stopping", "reloading"}
STATION_KEYS = {"name", "run_root", "run_dir", "solution_name"}
SOURCE_KEYS = ("rover_rtcm", "rover_ubx", "base_rtcm", "nav_rinex")
URI_SCHEMES = ("ntrip://", "tcp://", "serial://", "file://", "http://", "https://")
SECRET_KEY_PARTS = ("password", "passwd", "token", "secret", "credential", "api_key")
USERINFO_RE = re.compile(r"(://)([^/@\s]+)@")


def utc_now() -> dt.datetime:
    return dt.datetime.now(dt.timezone.utc)


def iso_now() -> str:
    return utc_now().isoformat()


def write_json_atomic(path: Path, payload: dict[str, Any], *, mode: int | None = None) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    temp_path = path.with_name(path.name + ".tmp")
    temp_path.write_text(json.dumps(payload, indent=2, sort_keys=True) + "\n", encoding="utf-8")
    if mode is not None:
        try:
            os.chmod(temp_path, mode)
        except OSError:
            pass
    os.replace(temp_path, path)


def read_json(path: Path) -> dict[str, Any]:
    payload = json.loads(path.read_text(encoding="utf-8"))
    if not isinstance(payload, dict):
        raise ValueError(f"JSON artifact must contain an object: {path}")
    return payload


def redact_text(value: object) -> str:
    text = str(value)
    # Keep the endpoint and mountpoint useful while never echoing URI userinfo.
    return USERINFO_RE.sub(r"\1***@", text)


def redact_config(config: dict[str, str]) -> dict[str, str]:
    result: dict[str, str] = {}
    for key, value in config.items():
        lowered = key.casefold()
        if any(part in lowered for part in SECRET_KEY_PARTS):
            result[key] = "***"
        else:
            result[key] = redact_text(value)
    return result


def redact_command(command: list[str]) -> list[str]:
    return [redact_text(token) for token in command]


def source_is_remote(value: str) -> bool:
    lowered = value.strip().casefold()
    return lowered.startswith(URI_SCHEMES)


def resolve_source(value: str, base_dir: Path) -> str:
    value = value.strip()
    if not value or source_is_remote(value):
        return value
    path = Path(value).expanduser()
    if not path.is_absolute():
        path = base_dir / path
    return str(path.resolve())


def scalar_to_config(value: object) -> str:
    if isinstance(value, bool):
        return "true" if value else "false"
    if isinstance(value, (list, tuple)):
        return " ".join(scalar_to_config(item) for item in value)
    return str(value)


def load_toml(path: Path) -> tuple[dict[str, Any], dict[str, str]]:
    if not path.is_file():
        raise ValueError(f"station TOML does not exist: {path}")
    try:
        with path.open("rb") as handle:
            document = tomllib.load(handle)
    except tomllib.TOMLDecodeError as exc:
        raise ValueError(f"failed to parse station TOML {path}: {exc}") from exc
    if not isinstance(document, dict):
        raise ValueError(f"station TOML must contain a table: {path}")

    # The preferred form is [station] plus an optional [receiver] table.  A
    # flat [station] table is accepted as a compact form for hand-written files.
    station = document.get("station", document)
    if not isinstance(station, dict):
        raise ValueError(f"[station] must be a TOML table: {path}")
    receiver = station.get("receiver", document.get("receiver", {}))
    if receiver is None:
        receiver = {}
    if not isinstance(receiver, dict):
        raise ValueError(f"[station.receiver] must be a TOML table: {path}")
    live = station.get("live", document.get("live", {}))
    if live is None:
        live = {}
    if not isinstance(live, dict):
        raise ValueError(f"[station.live] must be a TOML table: {path}")

    unknown: list[str] = []
    for table_name, table, allowed in (
        ("station", station, STATION_KEYS | {"receiver", "live"}),
        ("receiver", receiver, rcv.ALLOWED_KEYS),
        ("live", live, rcv.ALLOWED_KEYS),
    ):
        unknown.extend(f"{table_name}.{key}" for key in table if key not in allowed)
    if unknown:
        raise ValueError("unsupported station config key(s): " + ", ".join(sorted(unknown)))

    merged: dict[str, str] = {}
    for table in (live, receiver):
        for key, value in table.items():
            merged[key] = scalar_to_config(value)

    base_dir = path.parent.resolve()
    for key in SOURCE_KEYS:
        if key in merged:
            merged[key] = resolve_source(merged[key], base_dir)

    if "base_ecef" in merged:
        parts = merged["base_ecef"].split()
        if len(parts) != 3:
            raise ValueError("receiver.base_ecef must contain exactly three numbers")
        try:
            [float(part) for part in parts]
        except ValueError as exc:
            raise ValueError("receiver.base_ecef must contain three numbers") from exc

    return dict(station), merged


def find_live_binary() -> str | None:
    binary = rcv.find_binary("gnss_live")
    if binary:
        return binary
    return shutil.which("gnss_live")


def local_input_check(value: str) -> tuple[bool, str]:
    if source_is_remote(value):
        return True, "remote/device endpoint"
    path = Path(value)
    if path.is_file():
        stat = path.stat()
        return True, f"{path} ({stat.st_size} bytes)"
    return False, f"missing local input: {path}"


def validate_station_config(
    source_path: Path,
    station: dict[str, Any],
    config: dict[str, str],
    *,
    allow_missing_input: bool = False,
) -> tuple[list[dict[str, str]], list[str], list[str]]:
    checks: list[dict[str, str]] = []
    errors: list[str] = []
    warnings: list[str] = []

    def add(name: str, ok: bool, detail: str, *, warning: bool = False) -> None:
        if ok:
            status = "ok"
        elif warning:
            status = "warning"
        else:
            status = "error"
        checks.append({"name": name, "status": status, "detail": detail})

    add("config", source_path.is_file(), str(source_path))
    if not source_path.is_file():
        errors.append(f"station TOML does not exist: {source_path}")

    rover_rtcm = config.get("rover_rtcm", "").strip()
    rover_ubx = config.get("rover_ubx", "").strip()
    if bool(rover_rtcm) == bool(rover_ubx):
        detail = "set exactly one of rover_rtcm or rover_ubx"
        add("rover input", False, detail)
        errors.append(detail)
    else:
        key, value = ("rover_rtcm", rover_rtcm) if rover_rtcm else ("rover_ubx", rover_ubx)
        ok, detail = local_input_check(value)
        if not ok and allow_missing_input:
            add(key, False, detail, warning=True)
            warnings.append(detail)
        else:
            add(key, ok, detail)
            if not ok:
                errors.append(detail)

    base_rtcm = config.get("base_rtcm", "").strip()
    if not base_rtcm:
        detail = "base_rtcm is required"
        add("base corrections", False, detail)
        errors.append(detail)
    else:
        ok, detail = local_input_check(base_rtcm)
        if not ok and allow_missing_input:
            add("base_rtcm", False, detail, warning=True)
            warnings.append(detail)
        else:
            add("base_rtcm", ok, detail)
            if not ok:
                errors.append(detail)

    nav = config.get("nav_rinex", "").strip()
    if nav:
        ok, detail = local_input_check(nav)
        if not ok and allow_missing_input:
            add("nav_rinex", False, detail, warning=True)
            warnings.append(detail)
        else:
            add("nav_rinex", ok, detail)
            if not ok:
                errors.append(detail)

    binary = find_live_binary()
    add("gnss_live binary", binary is not None, binary or "build gnss_live or put it on PATH")
    if binary is None:
        errors.append("gnss_live binary was not found")

    for key, default, minimum in (
        ("max_epochs", "0", 0),
        ("rover_message_limit", "0", 0),
        ("base_message_limit", "0", 0),
        ("min_ar_sats", "5", 1),
        ("restart_max_attempts", "0", 0),
    ):
        raw = config.get(key, default)
        try:
            number = int(raw)
            ok = number >= minimum
        except ValueError:
            ok = False
        detail = f"{key}={raw}"
        add(key, ok, detail)
        if not ok:
            errors.append(f"{key} must be an integer >= {minimum}")

    for key, default, minimum in (
        ("ratio", "3.0", 0.0),
        ("base_hold_seconds", "0.5", 0.0),
        ("elevation_mask_deg", "15", 0.0),
        ("restart_delay_seconds", "2.0", 0.0),
    ):
        raw = config.get(key, default)
        try:
            number = float(raw)
            ok = number >= minimum
        except ValueError:
            ok = False
        detail = f"{key}={raw}"
        add(key, ok, detail)
        if not ok:
            errors.append(f"{key} must be a number >= {minimum}")

    if config.get("glonass_ar", "off") not in {"off", "on", "autocal"}:
        detail = "glonass_ar must be off, on, or autocal"
        add("glonass_ar", False, config.get("glonass_ar", ""))
        errors.append(detail)
    else:
        add("glonass_ar", True, config.get("glonass_ar", "off"))

    for key in ("auto_restart", "quiet", "verbose", "no_glonass", "no_beidou", "no_base_interp"):
        if key not in config:
            continue
        raw = config[key].casefold()
        ok = raw in rcv.TRUE_VALUES or raw in {"0", "false", "no", "off"}
        add(key, ok, config[key])
        if not ok:
            errors.append(f"{key} must be a boolean")

    output_format = config.get("format", "pos")
    format_ok = output_format in {"pos", "llh", "xyz"}
    add("format", format_ok, output_format)
    if not format_ok:
        errors.append("format must be pos, llh, or xyz")

    if "rover_ubx_baud" in config:
        try:
            baud_ok = int(config["rover_ubx_baud"]) > 0
        except ValueError:
            baud_ok = False
        add("rover_ubx_baud", baud_ok, config["rover_ubx_baud"])
        if not baud_ok:
            errors.append("rover_ubx_baud must be a positive integer")

    return checks, errors, warnings


def source_config_hash(path: Path) -> str:
    return hashlib.sha256(path.read_bytes()).hexdigest()


def git_revision() -> str | None:
    try:
        completed = subprocess.run(
            ["git", "rev-parse", "HEAD"],
            cwd=ROOT_DIR,
            capture_output=True,
            text=True,
            check=False,
        )
    except OSError:
        return None
    revision = completed.stdout.strip()
    return revision or None


def resolve_run_root(source_path: Path, station: dict[str, Any], override: str | None) -> Path:
    raw = override or str(station.get("run_root", "output/stations"))
    path = Path(raw).expanduser()
    if not path.is_absolute():
        path = source_path.parent / path
    return path.resolve()


def read_latest_run(run_root: Path) -> Path | None:
    pointer = run_root / "latest.json"
    if not pointer.is_file():
        return None
    try:
        payload = read_json(pointer)
        raw = payload.get("run_dir")
        if isinstance(raw, str) and Path(raw).is_dir():
            return Path(raw)
    except (OSError, ValueError, json.JSONDecodeError):
        return None
    return None


def resolve_existing_run(args: argparse.Namespace) -> Path:
    if args.run_dir:
        path = Path(args.run_dir).expanduser()
        if not path.is_absolute():
            path = Path.cwd() / path
        return path.resolve()
    if args.config_toml:
        source_path = Path(args.config_toml).expanduser().resolve()
        station, _ = load_toml(source_path)
        run_root = resolve_run_root(source_path, station, args.run_root)
        latest = read_latest_run(run_root)
        if latest is not None:
            return latest
    raise ValueError("status/stop requires --run-dir or a config with a previous station run")


def choose_run_dir(source_path: Path, station: dict[str, Any], explicit: str | None, override_root: str | None) -> Path:
    if explicit:
        path = Path(explicit).expanduser()
        if not path.is_absolute():
            path = Path.cwd() / path
        return path.resolve()
    configured = station.get("run_dir")
    if configured:
        path = Path(str(configured)).expanduser()
        if not path.is_absolute():
            path = source_path.parent / path
        return path.resolve()
    root = resolve_run_root(source_path, station, override_root)
    stamp = utc_now().strftime("%Y%m%dT%H%M%SZ")
    candidate = root / stamp
    index = 1
    while candidate.exists():
        candidate = root / f"{stamp}-{index:02d}"
        index += 1
    return candidate


def render_receiver_config(config: dict[str, str]) -> str:
    lines = ["# Generated by gnss station; do not edit while the run is active."]
    for key in sorted(config):
        value = config[key].replace("\n", " ").replace("\r", " ")
        lines.append(f"{key}={value}")
    return "\n".join(lines) + "\n"


def input_manifest(config: dict[str, str]) -> dict[str, dict[str, object]]:
    result: dict[str, dict[str, object]] = {}
    for key in SOURCE_KEYS:
        value = config.get(key, "")
        if not value:
            continue
        item: dict[str, object] = {"source": redact_text(value), "remote": source_is_remote(value)}
        if not item["remote"]:
            path = Path(value)
            item["exists"] = path.is_file()
            if path.is_file():
                stat = path.stat()
                item["size_bytes"] = stat.st_size
                item["mtime_ns"] = stat.st_mtime_ns
        result[key] = item
    return result


def process_identity_matches(pid: int, run_dir: Path) -> bool:
    """Avoid signalling a reused PID when procfs exposes the command line."""
    if pid <= 0 or not rcv.process_is_running(pid):
        return False
    proc_cmdline = Path(f"/proc/{pid}/cmdline")
    if not proc_cmdline.is_file():
        # Windows and non-procfs Unix systems do not expose an equivalent
        # without adding a platform-specific process library.  The status
        # artifact still supplies the PID and the caller owns that artifact.
        return True
    try:
        command_line = proc_cmdline.read_bytes().replace(b"\x00", b" ").decode("utf-8", errors="replace")
    except OSError:
        return False
    return "gnss_rcv.py" in command_line and str(run_dir) in command_line


def write_manifest(path: Path, payload: dict[str, Any]) -> None:
    write_json_atomic(path, payload)


def station_artifacts(run_dir: Path, solution_name: str = "solution.pos") -> dict[str, str]:
    return {
        "run_manifest": str(run_dir / "run.json"),
        "receiver_config": str(run_dir / "receiver.conf"),
        "status": str(run_dir / "status.json"),
        "log": str(run_dir / "live.log"),
        "solution": str(run_dir / solution_name),
    }


def update_manifest_from_status(run_dir: Path, status: dict[str, Any]) -> dict[str, Any]:
    manifest_path = run_dir / "run.json"
    manifest = read_json(manifest_path) if manifest_path.is_file() else {}
    manifest["state"] = status.get("state", manifest.get("state", "unknown"))
    manifest["pid"] = status.get("pid", manifest.get("pid"))
    manifest["pid_running"] = status.get("pid_running", False)
    for key in ("pid_identity_ok", "healthy", "started_at", "finished_at", "returncode", "summary_line", "summary_metrics", "restart_count"):
        if key in status:
            manifest[key] = status[key]
    manifest["updated_at"] = iso_now()
    write_manifest(manifest_path, manifest)
    return manifest


def status_payload(run_dir: Path, *, wait_seconds: float = 0.0, tail_log_lines: int = 0) -> dict[str, Any]:
    status_path = run_dir / "status.json"
    if not status_path.is_file():
        raise ValueError(f"station status artifact does not exist: {status_path}")

    deadline = time.monotonic() + max(wait_seconds, 0.0)
    while True:
        status = read_json(status_path)
        pid = int(status.get("pid", 0) or 0)
        status["pid_running"] = rcv.process_is_running(pid)
        status["pid_identity_ok"] = process_identity_matches(pid, run_dir) if status["pid_running"] else False
        if wait_seconds <= 0.0 or time.monotonic() >= deadline:
            break
        if str(status.get("state", "")) not in RUNNING_STATES and not status["pid_running"]:
            break
        time.sleep(0.2)

    manifest = read_json(run_dir / "run.json") if (run_dir / "run.json").is_file() else {}
    if isinstance(status.get("config"), dict):
        status["config"] = redact_config({str(key): str(value) for key, value in status["config"].items()})
    if isinstance(status.get("command"), list):
        status["command"] = redact_command([str(value) for value in status["command"]])
    status["schema_version"] = SCHEMA_VERSION
    status["run_dir"] = str(run_dir)
    status["artifacts"] = manifest.get("artifacts", station_artifacts(run_dir))
    status["healthy"] = (
        str(status.get("state", "")) == "running"
        and bool(status["pid_running"])
        and bool(status["pid_identity_ok"])
    )
    artifact_solution = status["artifacts"].get("solution") if isinstance(status["artifacts"], dict) else None
    solution_path = Path(artifact_solution) if isinstance(artifact_solution, str) else run_dir / "solution.pos"
    log_path = run_dir / "live.log"
    status["solution_exists"] = solution_path.is_file()
    status["solution_size_bytes"] = solution_path.stat().st_size if solution_path.is_file() else 0
    status["log_exists"] = log_path.is_file()
    status["log_size_bytes"] = log_path.stat().st_size if log_path.is_file() else 0
    if tail_log_lines > 0 and log_path.is_file():
        status["log_tail"] = log_path.read_text(encoding="utf-8", errors="replace").splitlines()[-tail_log_lines:]
    update_manifest_from_status(run_dir, status)
    return status


def print_json(payload: dict[str, Any]) -> None:
    print(json.dumps(payload, indent=2, sort_keys=True))


def prepare(
    source_path: Path,
    args: argparse.Namespace,
    *,
    allow_missing_input: bool | None = None,
) -> tuple[dict[str, Any], dict[str, str], list[dict[str, str]], list[str], list[str], list[str]]:
    station, config = load_toml(source_path)
    checks, errors, warnings = validate_station_config(
        source_path,
        station,
        config,
        allow_missing_input=args.allow_missing_input if allow_missing_input is None else allow_missing_input,
    )
    command: list[str] = []
    if not errors:
        try:
            command = rcv.build_live_command(config)
        except (FileNotFoundError, ValueError) as exc:
            errors.append(str(exc))
    return station, config, checks, errors, warnings, command


def check_station(args: argparse.Namespace) -> int:
    if not args.config_toml:
        raise ValueError("check requires --config-toml")
    source_path = Path(args.config_toml).expanduser().resolve()
    station, config, checks, errors, warnings, command = prepare(source_path, args)
    payload = {
        "schema_version": SCHEMA_VERSION,
        "ok": not errors,
        "action": "check",
        "config_path": str(source_path),
        "name": station.get("name", source_path.stem),
        "checks": checks,
        "warnings": warnings,
        "errors": errors,
        "config": redact_config(config),
        "command": redact_command(command),
    }
    print_json(payload)
    return 0 if not errors else 1


def start_station(args: argparse.Namespace) -> int:
    if not args.config_toml:
        raise ValueError("start requires --config-toml")
    source_path = Path(args.config_toml).expanduser().resolve()
    # A check may intentionally allow missing files for a configuration-only
    # dry run; starting a station must always reject them before spawning a
    # background process.
    station, config, checks, errors, warnings, command = prepare(
        source_path,
        args,
        allow_missing_input=False,
    )
    if errors:
        print_json(
            {
                "schema_version": SCHEMA_VERSION,
                "ok": False,
                "action": "start",
                "config_path": str(source_path),
                "checks": checks,
                "warnings": warnings,
                "errors": errors,
            }
        )
        return 1

    run_dir = choose_run_dir(source_path, station, args.run_dir, args.run_root)
    if run_dir.exists() and any(run_dir.iterdir()):
        raise ValueError(f"run directory is not empty: {run_dir}")
    run_dir.mkdir(parents=True, exist_ok=True)

    solution_name = str(station.get("solution_name") or config.get("out") or "solution.pos")
    solution_name = Path(solution_name).name or "solution.pos"
    config = dict(config)
    config["out"] = str(run_dir / solution_name)
    config.setdefault("format", "pos")
    config.setdefault("quiet", "true")
    config.setdefault("auto_restart", "true")
    config.setdefault("restart_delay_seconds", "2.0")
    config.setdefault("restart_max_attempts", "0")
    config.setdefault("status_redact", "true")
    command = rcv.build_live_command(config)

    receiver_config_path = run_dir / "receiver.conf"
    receiver_config_path.write_text(render_receiver_config(config), encoding="utf-8")
    try:
        os.chmod(receiver_config_path, 0o600)
    except OSError:
        pass
    status_path = run_dir / "status.json"
    log_path = run_dir / "live.log"
    manifest_path = run_dir / "run.json"
    artifacts = station_artifacts(run_dir, solution_name)
    manifest: dict[str, Any] = {
        "schema_version": SCHEMA_VERSION,
        "state": "preparing",
        "name": station.get("name", source_path.stem),
        "source_config_path": str(source_path),
        "source_config_sha256": source_config_hash(source_path),
        "created_at": iso_now(),
        "git_revision": git_revision(),
        "config": redact_config(config),
        "command": redact_command(command),
        "inputs": input_manifest(config),
        "artifacts": artifacts,
    }
    write_manifest(manifest_path, manifest)

    namespace = argparse.Namespace(
        action="start",
        config=str(receiver_config_path),
        overrides=[],
        status_out=str(status_path),
        log_out=str(log_path),
        dry_run=False,
        wait_seconds=0.0,
        poll_seconds=0.2,
        tail_log_lines=0,
    )
    captured = io.StringIO()
    try:
        with contextlib.redirect_stdout(captured):
            returncode = rcv.start_receiver(namespace)
    except (FileNotFoundError, ValueError) as exc:
        manifest["state"] = "failed"
        manifest["error"] = str(exc)
        manifest["updated_at"] = iso_now()
        write_manifest(manifest_path, manifest)
        raise
    if returncode != 0:
        raise ValueError(f"failed to start receiver (exit code {returncode})")

    try:
        launcher = json.loads(captured.getvalue())
    except json.JSONDecodeError as exc:
        raise ValueError("gnss rcv did not return a valid start payload") from exc
    manifest["state"] = launcher.get("state", "starting")
    manifest["pid"] = launcher.get("pid")
    manifest["updated_at"] = iso_now()
    write_manifest(manifest_path, manifest)
    run_root = run_dir.parent
    write_json_atomic(run_root / "latest.json", {"schema_version": SCHEMA_VERSION, "run_dir": str(run_dir), "updated_at": iso_now()})
    print_json(
        {
            "schema_version": SCHEMA_VERSION,
            "ok": True,
            "action": "start",
            "state": manifest["state"],
            "name": manifest["name"],
            "run_dir": str(run_dir),
            "pid": manifest.get("pid"),
            "artifacts": artifacts,
            "config": manifest["config"],
        }
    )
    return 0


def status_station(args: argparse.Namespace) -> int:
    run_dir = resolve_existing_run(args)
    payload = status_payload(run_dir, wait_seconds=args.wait_seconds, tail_log_lines=args.tail_log_lines)
    print_json(payload)
    return 0


def stop_station(args: argparse.Namespace) -> int:
    run_dir = resolve_existing_run(args)
    payload = stop_run(run_dir, wait_seconds=args.wait_seconds, tail_log_lines=args.tail_log_lines)
    payload["action"] = "stop"
    print_json(payload)
    return 0


def stop_run(run_dir: Path, *, wait_seconds: float, tail_log_lines: int) -> dict[str, Any]:
    status_path = run_dir / "status.json"
    if not status_path.is_file():
        raise ValueError(f"station status artifact does not exist: {status_path}")
    current = read_json(status_path)
    pid = int(current.get("pid", 0) or 0)
    if rcv.process_is_running(pid) and not process_identity_matches(pid, run_dir):
        raise ValueError(f"refusing to stop pid {pid}: station process identity does not match {run_dir}")
    rcv.stop_receiver_at_status_path(status_path, emit_output=False)
    payload = status_payload(run_dir, wait_seconds=wait_seconds, tail_log_lines=tail_log_lines)
    return payload


def restart_station(args: argparse.Namespace) -> int:
    old_run_dir = resolve_existing_run(args)
    manifest = read_json(old_run_dir / "run.json")
    source = args.config_toml or manifest.get("source_config_path")
    if not isinstance(source, str) or not source:
        raise ValueError("restart could not determine the source station TOML")
    stop_run(old_run_dir, wait_seconds=args.wait_seconds, tail_log_lines=args.tail_log_lines)
    start_args = argparse.Namespace(**vars(args))
    start_args.config_toml = source
    start_args.run_dir = None
    return start_station(start_args)


def build_argument_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(
        prog=os.environ.get("GNSS_CLI_NAME", "gnss station"),
        description="Validate and manage a long-running GNSS RTK station run.",
    )
    parser.add_argument(
        "action",
        nargs="?",
        choices=("check", "start", "status", "stop", "restart"),
        default="check",
        help="Station action (default: check).",
    )
    parser.add_argument("--config-toml", "--config", dest="config_toml", help="Station TOML configuration.")
    parser.add_argument("--run-dir", help="Explicit run directory for start/status/stop.")
    parser.add_argument("--run-root", help="Override the configured run artifact root.")
    parser.add_argument("--allow-missing-input", action="store_true", help="Report missing local inputs as warnings during check.")
    parser.add_argument("--wait-seconds", type=float, default=0.0, help="Wait for a state transition before printing status.")
    parser.add_argument("--tail-log-lines", type=int, default=0, help="Include the last N log lines in status JSON.")
    return parser


def main() -> int:
    args = build_argument_parser().parse_args()
    try:
        if args.action == "check":
            return check_station(args)
        if args.action == "start":
            return start_station(args)
        if args.action == "status":
            return status_station(args)
        if args.action == "stop":
            return stop_station(args)
        if args.action == "restart":
            return restart_station(args)
        raise ValueError(f"unsupported station action: {args.action}")
    except (FileNotFoundError, OSError, ValueError) as exc:
        print(f"Error: {exc}", file=sys.stderr)
        return 1


if __name__ == "__main__":
    raise SystemExit(main())
