#!/usr/bin/env python3
"""Launch-free Phase133 qualification facade.

Only static freeze/manifest/pre-raw verification is exposed.  Raw
materialization and native invocation require a future independent
authorization and are intentionally unreachable from this facade.
"""

from __future__ import annotations

import sys
from pathlib import Path


ROOT = Path(__file__).resolve().parents[3]
sys.path.insert(0, str(ROOT / "apps/commands/benchmarks"))

from gnss_smartphone_phase133_runner_native_selector_boundary import main  # noqa: E402


if __name__ == "__main__":
    raise SystemExit(main())
