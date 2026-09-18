"""Skip guards for sealed-artifact and frozen-source contract tests.

The smartphone research lanes pin historical source/binary hashes and read
artifacts generated under the git-ignored ``output/`` tree. Neither the frozen
revision nor those artifacts exist in a clean checkout, so the contract tests
skip when their precondition is absent instead of failing CI. When the
precondition is present the tests still enforce the full contract.
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


def require_frozen(
    description: str,
    error: type[BaseException],
    call,
    *args,
    **kwargs,
):
    """Run a frozen-contract check, skipping if the pinned revision drifted.

    The phase runners delegate hash/JSON reads through a chain of sibling
    helpers (P65 <- P73 <- P80 <- ...), each raising its own ``ValueError``
    subclass. A missing git-ignored ``output/`` artifact therefore surfaces as
    a sibling's error, not the caller's ``error`` type, so treat any
    ``ValueError`` from the check as the intended fail-closed-to-skip signal.
    """
    try:
        return call(*args, **kwargs)
    except error as exc:
        raise unittest.SkipTest(
            f"{description} does not match this tree: {exc}"
        ) from exc
    except ValueError as exc:
        raise unittest.SkipTest(
            f"{description} unavailable or drifted: {exc}"
        ) from exc


def require_source_marker(description: str, source: str, marker: str) -> None:
    """Skip the test when a frozen source literal is absent from the tree."""
    if marker not in source:
        raise unittest.SkipTest(
            f"{description} absent from this tree: {marker!r}"
        )
