#!/usr/bin/env python3
"""Launch-free facade for Phase128 inventory contract qualification."""

from __future__ import annotations

import sys
from pathlib import Path


ROOT = Path(__file__).resolve().parents[3]
sys.path.insert(0, str(ROOT / "apps/commands/benchmarks"))

from gnss_smartphone_phase128_inventory_first_structural import main  # noqa: E402


if __name__ == "__main__":
    raise SystemExit(main())
