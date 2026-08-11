#!/usr/bin/env python3
"""Runtime helpers shared by installed Python command wrappers."""

from __future__ import annotations

import importlib.util
import json
from datetime import date
from pathlib import Path
import shutil
import subprocess
import sys
import threading
from types import ModuleType


_MODULE_MISSING = object()
_MODULE_LOAD_LOCK = threading.RLock()
_MODULES_LOADING: set[str] = set()


def application_root(script_file: str) -> Path:
    """Return the repository root in-tree or install prefix for a command."""
    script_path = Path(script_file).resolve()
    if script_path.parent.parent.name == "commands":
        return script_path.parents[3]
    return script_path.parent.parent


def _module_origin(module: ModuleType) -> Path | None:
    origin = getattr(module, "__file__", None)
    if origin is None:
        origin = getattr(getattr(module, "__spec__", None), "origin", None)
    if not isinstance(origin, (str, Path)):
        return None
    try:
        return Path(origin).resolve()
    except (OSError, RuntimeError, ValueError):
        return None


def load_python_module(module_name: str, module_path: Path) -> ModuleType:
    """Load a Python file by path without changing the process import path."""
    if not isinstance(module_name, str) or not module_name:
        raise ValueError("module_name must be a non-empty string")

    try:
        resolved_path = Path(module_path).expanduser().resolve()
    except (OSError, RuntimeError) as exc:
        raise ImportError(f"Cannot resolve Python module path: {module_path}") from exc

    with _MODULE_LOAD_LOCK:
        existing = sys.modules.get(module_name, _MODULE_MISSING)
        if existing is not _MODULE_MISSING:
            existing_path = _module_origin(existing) if isinstance(existing, ModuleType) else None
            if isinstance(existing, ModuleType) and existing_path == resolved_path:
                if module_name in _MODULES_LOADING:
                    raise ImportError(f"Python module {module_name!r} is still initializing")
                return existing
            origin = str(existing_path) if existing_path is not None else "<unknown file>"
            raise ImportError(
                f"Module name collision: {module_name!r} is already loaded from {origin}; "
                f"cannot load {resolved_path}"
            )

        if not resolved_path.exists():
            raise FileNotFoundError(f"Python module file does not exist: {resolved_path}")
        if not resolved_path.is_file():
            raise ImportError(f"Python module path is not a file: {resolved_path}")

        spec = importlib.util.spec_from_file_location(module_name, resolved_path)
        if spec is None or spec.loader is None or not hasattr(spec.loader, "exec_module"):
            raise ImportError(f"No usable Python loader for {resolved_path}")

        module = importlib.util.module_from_spec(spec)
        sys.modules[module_name] = module
        _MODULES_LOADING.add(module_name)
        try:
            spec.loader.exec_module(module)
        except BaseException:
            if sys.modules.get(module_name) is module:
                del sys.modules[module_name]
            raise
        finally:
            _MODULES_LOADING.discard(module_name)
        return module


def resolve_gnss_command(root_dir: Path) -> list[str]:
    """Return a command prefix that launches the top-level gnss dispatcher."""
    source_dispatcher = root_dir / "apps" / "gnss.py"
    if source_dispatcher.exists():
        return [sys.executable, str(source_dispatcher)]

    sibling_dispatcher = Path(__file__).resolve().parent.parent / "gnss"
    if sibling_dispatcher.exists():
        return [str(sibling_dispatcher)]

    installed_dispatcher = shutil.which("gnss")
    if installed_dispatcher is not None:
        return [installed_dispatcher]

    raise SystemExit(
        "Missing dispatcher: expected source-tree apps/gnss.py or installed gnss on PATH"
    )


def ensure_input_exists(path: Path, description: str, root_dir: Path) -> None:
    """Validate an input path and explain bundled-data behavior for installed prefixes."""
    if path.exists():
        return

    extra = ""
    bundled_roots = (
        root_dir / "data",
        root_dir / "scripts",
        root_dir / "configs",
    )
    if any(str(path).startswith(str(candidate)) for candidate in bundled_roots):
        extra = (
            " Installed prefixes do not ship the sample datasets; "
            "pass explicit --obs/--rover/--base/--nav paths from your own dataset or source tree."
        )

    raise SystemExit(f"Missing {description}: {path}.{extra}")


def parse_summary_metrics(line: str) -> dict[str, object]:
    """Parse a `summary:` line into a dict with int/float/string values."""
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


def _normalize_rinex_year(raw_year: int) -> int:
    if raw_year >= 100:
        return raw_year
    return 2000 + raw_year if raw_year < 80 else 1900 + raw_year


def infer_rinex_first_obs_date(path: Path) -> date:
    with path.open(encoding="ascii", errors="ignore") as handle:
        for line in handle:
            if "TIME OF FIRST OBS" in line:
                fields = line[:43].split()
                if len(fields) < 3:
                    break
                year = _normalize_rinex_year(int(float(fields[0])))
                month = int(float(fields[1]))
                day = int(float(fields[2]))
                return date(year, month, day)
            if "END OF HEADER" in line:
                break
    raise SystemExit(f"Failed to infer TIME OF FIRST OBS from {path}")


def run_fetch_products(
    root_dir: Path,
    obs_path: Path,
    presets: list[str],
    product_specs: list[str],
    product_date_text: str | None = None,
    cache_dir: Path | None = None,
) -> dict[str, object]:
    if not presets and not product_specs:
        raise SystemExit("--fetch-products requires at least one --preset or --product KIND=SOURCE")

    effective_date = product_date_text or infer_rinex_first_obs_date(obs_path).isoformat()
    command = [*resolve_gnss_command(root_dir), "fetch-products", "--date", effective_date]
    for preset in presets:
        command.extend(["--preset", preset])
    for spec in product_specs:
        command.extend(["--product", spec])
    if cache_dir is not None:
        command.extend(["--cache-dir", str(cache_dir)])

    print("+", " ".join(command))
    completed = subprocess.run(
        command,
        cwd=root_dir,
        capture_output=True,
        text=True,
        check=False,
    )
    if completed.returncode != 0:
        raise SystemExit(
            f"gnss fetch-products failed with exit code {completed.returncode}\n"
            f"{completed.stdout}{completed.stderr}"
        )
    try:
        payload = json.loads(completed.stdout)
    except json.JSONDecodeError as exc:
        raise SystemExit(
            "gnss fetch-products did not return valid JSON\n"
            f"{completed.stdout}{completed.stderr}"
        ) from exc
    assert isinstance(payload, dict)
    payload["effective_date"] = effective_date
    return payload
