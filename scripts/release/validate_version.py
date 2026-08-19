#!/usr/bin/env python3
"""Validate an exact release tag against the canonical CMake project version."""

from __future__ import annotations

import argparse
import re
import sys
from pathlib import Path


_VERSION = r"(?:0|[1-9][0-9]*)\.(?:0|[1-9][0-9]*)\.(?:0|[1-9][0-9]*)"
_TAG_PATTERN = re.compile(rf"^v(?P<version>{_VERSION})$")
_CMAKE_PROJECT_PATTERN = re.compile(
    rf"(?im)^\s*project\s*\(\s*gnss_lib\s+VERSION\s+(?P<version>{_VERSION})(?=\s|\))"
)


def version_from_tag(tag: str) -> str:
    """Return the version in an exact ``vMAJOR.MINOR.PATCH`` tag."""

    match = _TAG_PATTERN.fullmatch(tag)
    if match is None:
        raise ValueError("tag must match exact vMAJOR.MINOR.PATCH SemVer")
    return match.group("version")


def version_from_cmake(cmake_file: Path) -> str:
    """Read the single canonical ``project(gnss_lib VERSION ...)`` declaration."""

    text = cmake_file.read_text(encoding="utf-8")
    matches = list(_CMAKE_PROJECT_PATTERN.finditer(text))
    if len(matches) != 1:
        raise ValueError(
            f"expected one project(gnss_lib VERSION ...) declaration in {cmake_file}"
        )
    return matches[0].group("version")


def validate_versions(tag: str, cmake_file: Path) -> str:
    """Validate and return the shared version string."""

    tag_version = version_from_tag(tag)
    cmake_version = version_from_cmake(cmake_file)
    if tag_version != cmake_version:
        raise ValueError(
            f"tag {tag!r} resolves to {tag_version}, but {cmake_file} declares {cmake_version}"
        )
    return tag_version


def main(argv: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--tag", required=True, help="release tag, exactly vMAJOR.MINOR.PATCH")
    parser.add_argument(
        "--cmake-file",
        type=Path,
        default=Path("CMakeLists.txt"),
        help="canonical CMake project file (default: CMakeLists.txt)",
    )
    args = parser.parse_args(argv)

    try:
        version = validate_versions(args.tag, args.cmake_file)
    except (OSError, ValueError) as error:
        print(f"release version validation failed: {error}", file=sys.stderr)
        return 1

    print(f"release version validated: {args.tag} == CMake {version}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
