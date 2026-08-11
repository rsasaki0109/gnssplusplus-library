# Self-contained demo fixture

These three small files are a project-authored deterministic PPP fixture for
the offline `gnss demo` smoke path:

- `synthetic_ppp.obs`: eight RINEX 3.04 observation epochs from six GPS
  satellites;
- `synthetic_ppp.sp3`: two precise-orbit epochs for the same six satellites;
- `synthetic_ppp.clk`: matching zero-clock precise-product records.

The observations are computed from geometric ranges for a receiver at
35°N, 139°E, 45 m ellipsoidal height. They are intentionally synthetic and
must not be read as a field-accuracy benchmark. The fixture contains no
downloaded or credentialed data and is distributed under this repository's
MIT license (`LICENSE`).

The equations and formatting source are the repository's existing
`build_synthetic_ppp_inputs` test helper in
`tests/cli_cases/_support.py`. Keeping the resulting tiny inputs tracked
makes the clean-checkout demo independent of test generation code, Python
packages, and network access.
