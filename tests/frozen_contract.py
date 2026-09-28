"""Skip guards for sealed-artifact contract tests.

Some smartphone evaluation tests read artifacts generated under the
git-ignored ``output/`` tree. Those artifacts do not exist in a clean
checkout, so the tests skip when their precondition is absent instead of
failing CI. When the precondition is present the tests still enforce the full
contract.
"""

from __future__ import annotations

from pathlib import Path
import unittest
from typing import Iterable


def require_files(description: str, paths: Iterable[Path]) -> None:
    """Skip the test when any required generated artifact is missing."""
    missing = [str(path) for path in paths if not Path(path).is_file()]
    if missing:
        raise unittest.SkipTest(
            f"{description} unavailable (missing: {', '.join(missing)})"
        )
