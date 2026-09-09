# Phase291 source transmission clock primitive

Starting HEAD `daaf701`; root-only. Previous audit narrowed the next action.

Pinned source: [MALIB ephemeris.c](https://raw.githubusercontent.com/JAXA-SNU/MALIB/159e150d4a54e6b7b15d81128289b8559523ca81/src/ephemeris.c),
SHA-256 `db4d22616a2f708372a5ee893c3c2997183b146bd1bd93ff48da71a83dec193f`.
Read through web and a separate in-memory HTTP fetch; no checkout mutation.

Source findings: satposs selects the first nonzero pseudorange among NFREQ
slots, then uses ephclk before final satpos. eph2clk iterates its polynomial
twice relative to toc; this initial clock excludes relativity and TGD.
The native two-state-evaluation path instead uses its relativistic clock
for that initial time adjustment. This is a concrete parity difference,
not evidence of a material accuracy gain.

Local obs2obs.c:510-530 converts NaN pseudoranges to zero per slot. Exact
signal-to-slot mapping and ephclk ephemeris-selection semantics still need
integration coverage; no first-L1 assumption is encoded here.

Added a header-only finite-checked polynomial primitive with constant,
linear, quadratic and invalid-input synthetic tests. It is not wired into
production paths and does not implement GLONASS/SBAS clock laws or satellite
selection. Those require their own paired adapter coverage. No solver/raw,
truth, candidate, MAT or Kaggle access. No new score.

Validation: gnss_run_tests built successfully (-j4); all 19 base-compensation
tests passed. Synthetic fixture I/O is part of that suite. Not full CTest,
real-data parity, or an end-to-end source-state implementation.
