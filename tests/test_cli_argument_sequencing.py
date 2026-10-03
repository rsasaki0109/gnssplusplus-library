#!/usr/bin/env python3
"""Guard against reading several argv values in one unsequenced expression.

`Eigen::Vector3d(std::stod(argv[++i]), std::stod(argv[++i]), ...)` has an
unspecified evaluation order: MSVC builds read the values right to left, so a
`--base-ecef X Y Z` override silently became (Z, Y, X) and RTK failed to fix.
Every native CLI must consume one argv value per statement.
"""

from __future__ import annotations

import re
import unittest
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
SOURCE_DIRS = ("apps", "src")
ARGV_INCREMENT = re.compile(r"argv\s*\[\s*\+\+\s*i\s*\]")


def unsequenced_argv_reads(text: str) -> list[int]:
    """Return 1-based line numbers of statements that read argv[++i] twice."""

    hits: list[int] = []
    for match in re.finditer(r"[^;{}]+", text):
        if len(ARGV_INCREMENT.findall(match.group(0))) > 1:
            hits.append(text.count("\n", 0, match.start()) + 1)
    return hits


class CliArgumentSequencingTests(unittest.TestCase):
    def test_detector_flags_the_unsequenced_pattern(self) -> None:
        bad = "x = Eigen::Vector3d(std::stod(argv[++i]),\n std::stod(argv[++i]), 0.0);"
        good = "a = std::stod(argv[++i]);\nb = std::stod(argv[++i]);"
        self.assertEqual(unsequenced_argv_reads(bad), [1])
        self.assertEqual(unsequenced_argv_reads(good), [])

    def test_native_sources_read_one_argv_value_per_statement(self) -> None:
        offenders = []
        for directory in SOURCE_DIRS:
            for path in sorted((ROOT / directory).rglob("*.cpp")):
                for line in unsequenced_argv_reads(path.read_text(encoding="utf-8", errors="replace")):
                    offenders.append(f"{path.relative_to(ROOT).as_posix()}:{line}")
        self.assertEqual(offenders, [], "argv[++i] read more than once in one statement")


if __name__ == "__main__":
    unittest.main()
