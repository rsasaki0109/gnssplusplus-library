#!/usr/bin/env python3
"""Launch-free Phase134 qualification entry point.

This facade exposes only static contract verification.  It deliberately has
no raw-input materialization and no subprocess/native-solver path.  A later
independent authorization must use a separately pinned execution wrapper.
"""

from __future__ import annotations

import sys
from pathlib import Path


ROOT = Path(__file__).resolve().parents[3]
sys.path.insert(0, str(ROOT / "apps/commands/benchmarks"))

from gnss_smartphone_phase134_native_summary_bridge import main  # noqa: E402


if __name__ == "__main__":
    raise SystemExit(main())
