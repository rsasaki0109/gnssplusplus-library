#!/usr/bin/env python3
"""Launch-free Phase126 runner facade.

Before independent authorization this facade only delegates to the structural
validator.  It never materializes an input, opens a payload, starts a native
application, reads a solution, or evaluates truth.  The name is retained so
the later authorized runner has a stable manifest artifact boundary.
"""

from __future__ import annotations

from gnss_smartphone_phase126_raw_base_compound_structural import main


if __name__ == "__main__":
    raise SystemExit(main())
