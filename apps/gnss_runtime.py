#!/usr/bin/env python3
"""Runtime helpers shared by installed Python command wrappers."""

from __future__ import annotations

from pathlib import Path
import shutil
import sys


def resolve_gnss_command(root_dir: Path) -> list[str]:
    """Return a command prefix that launches the top-level gnss dispatcher."""
    source_dispatcher = root_dir / "apps" / "gnss.py"
    if source_dispatcher.exists():
        return [sys.executable, str(source_dispatcher)]

    sibling_dispatcher = Path(__file__).resolve().parent / "gnss"
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
