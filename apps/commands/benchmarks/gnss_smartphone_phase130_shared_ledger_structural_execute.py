#!/usr/bin/env python3
"""Launch-free facade for Phase130 structural-contract qualification.

This entry point exposes only static and synthetic validation modes.  Raw
materialization and native solver launch require a later independent
authorization artifact and are not reachable here.
"""

from __future__ import annotations

import sys
from pathlib import Path


ROOT = Path(__file__).resolve().parents[3]
sys.path.insert(0, str(ROOT / "apps/commands/benchmarks"))

from gnss_smartphone_phase130_shared_ledger_structural import main  # noqa: E402


if __name__ == "__main__":
    raise SystemExit(main())
