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
