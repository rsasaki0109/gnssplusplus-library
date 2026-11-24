"""``gnss`` console entry point for pip installs of libgnss++.

The wheel installs the regular CMake install tree inside this package
(``libgnsspp/bin``, ``libgnsspp/share``, ...), so the dispatcher finds the
native tools and Python commands exactly as in a ``cmake --install`` prefix.
"""

from __future__ import annotations

from pathlib import Path
import runpy
import sys


def dispatcher_path() -> Path:
    return Path(__file__).resolve().parent / "bin" / "gnss"


def main() -> None:
    dispatcher = dispatcher_path()
    if not dispatcher.is_file():
        sys.exit(
            f"Error: {dispatcher} is missing; this libgnsspp install has no "
            "bundled command suite (it is present only in pip-installed wheels)."
        )
    sys.argv[0] = str(dispatcher)
    runpy.run_path(str(dispatcher), run_name="__main__")


if __name__ == "__main__":
    main()
