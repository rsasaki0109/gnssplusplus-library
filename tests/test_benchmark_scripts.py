#!/usr/bin/env python3
"""Backward-compatible runner for the domain-oriented benchmark tests."""

from __future__ import annotations

import sys
import unittest
from pathlib import Path


TESTS_DIR = Path(__file__).resolve().parent
if str(TESTS_DIR) not in sys.path:
    sys.path.insert(0, str(TESTS_DIR))

from benchmark_cases.clas_iers import *  # noqa: F401,F403,E402
from benchmark_cases.core_metrics import *  # noqa: F401,F403,E402
from benchmark_cases.multi_candidate import *  # noqa: F401,F403,E402
from benchmark_cases.ppc_selectors import *  # noqa: F401,F403,E402
from benchmark_cases.position_bridges import *  # noqa: F401,F403,E402
from benchmark_cases.reports import *  # noqa: F401,F403,E402
from benchmark_cases.signoffs import *  # noqa: F401,F403,E402


if __name__ == "__main__":
    unittest.main()
