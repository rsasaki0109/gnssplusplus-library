"""Shared path setup for the PPC experiment entrypoints."""

from __future__ import annotations

from pathlib import Path
import sys


PPC_DIR = Path(__file__).resolve().parent
SCRIPTS_DIR = PPC_DIR.parents[1]
ROOT_DIR = SCRIPTS_DIR.parent
ANALYSIS_DIR = SCRIPTS_DIR / "analysis"
COMMANDS_DIR = ROOT_DIR / "apps" / "commands"
APPS_DIR = COMMANDS_DIR / "benchmarks"

_COMMAND_GROUPS = (
    "benchmarks",
    "positioning",
    "products",
    "receivers",
    "visualization",
    "diagnostics",
)
for _path in (
    PPC_DIR,
    SCRIPTS_DIR,
    ANALYSIS_DIR,
    COMMANDS_DIR,
    *(COMMANDS_DIR / group for group in _COMMAND_GROUPS),
):
    if str(_path) not in sys.path:
        sys.path.insert(0, str(_path))
