#!/usr/bin/env python3
"""Launch-free facade for Phase131 structural-contract qualification.

Only static pin, manifest, pre-raw, and synthetic in-memory validation modes
are exposed here.  Raw materialization and native execution require a later
independent authorization artifact and are intentionally unreachable from
this facade.
"""

from __future__ import annotations

import sys
from pathlib import Path


ROOT = Path(__file__).resolve().parents[3]
sys.path.insert(0, str(ROOT / "apps/commands/benchmarks"))

from gnss_smartphone_phase131_canonical_correction_structural import main  # noqa: E402


if __name__ == "__main__":
    raise SystemExit(main())
