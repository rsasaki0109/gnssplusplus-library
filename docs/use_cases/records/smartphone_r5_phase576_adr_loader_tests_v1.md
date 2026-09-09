# Phase576: ADR metadata/slip contract passes native loader tests

Extended new ADR-uncertainty fixture to 8 metadata cases times 4 ADR states
(invalid=0, valid=1, valid+reset=3, valid+slip=5). Positive uncertainty is
retained as metadata even on unusable carrier rows; it does not restore
has_carrier_phase. Reset/slip retains loss-of-lock indication. Valid ADR
still has the same carrier metres, without multiplying its uncertainty by
carrier sign or wavelength. Invalid/missing optional metadata leaves rows
admitted under their existing quality rules.

Built a fresh standalone native loader test from source, avoiding ABI mixing
while the full build is still working:

    c++ -std=c++17 -O0 -Iinclude -I/usr/include/eigen3 \
      tests/test_android_raw_gnss.cpp src/io/android_raw_gnss.cpp \
      src/core/observation.cpp src/core/types.cpp \
      -lgtest -lgtest_main -lpthread -o /tmp/phase576_android_focused

`/tmp/phase576_android_focused`: **20/20 tests passed**. Includes raw clock,
signal signs/mapping, quality masks, duplicate rejection, UTC alignment,
ignored enriched pseudorange and MATLAB-path fail-closed cases. Tests use
synthetic temporary CSVs, not MAT payloads or saved positioning inputs.
Temporary fixtures removed by the tests. This is not full CTest or raw
positioning-output invariance.

Full native build session 12146 remains live, last poll compiling PPP/RTK
sources around 77%. No restart or second full build. Next finish that handle,
then freeze a raw-only output-invariance replay before transferring metadata
to actual TDCP endpoint rows. Production weights/factors unchanged; no new
accuracy evaluation/submission, full goal unmet.
