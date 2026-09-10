#!/usr/bin/env python3
"""Launch-free Phase132 qualification facade.

This facade exposes only static freeze/manifest/pre-raw verification.  Raw
materialization and native invocation require a future independent
authorization and are intentionally not reachable here.
"""

from __future__ import annotations

import sys
from pathlib import Path


ROOT = Path(__file__).resolve().parents[3]
sys.path.insert(0, str(ROOT / "apps/commands/benchmarks"))

from gnss_smartphone_phase132_typed_canonical_structural import main  # noqa: E402


if __name__ == "__main__":
    raise SystemExit(main())
