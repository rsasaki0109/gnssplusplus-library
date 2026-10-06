# Python satellite observation analysis

`gnss observation-analysis` reads RINEX through the public Python bindings and
exports satellite and signal time series as CSV, summary JSON and one PNG per
satellite. It uses corrected code, SNR, raw carrier phase, Doppler, satellite
motion and source loss-of-lock metadata. It does not read reference truth or
change a positioning solution.

## Build and run

Configure a native build with the Python interpreter that will run the command.
Install pybind11 and matplotlib in that interpreter's environment. Build the
`_libgnsspp` target; the resulting package lives under `<build>/python`.

```bash
cmake -S . -B build-python -DCMAKE_BUILD_TYPE=Release \
  -DGNSSPP_BUILD_PYTHON_BINDINGS=ON \
  -DPython3_EXECUTABLE=/path/to/python \
  -Dpybind11_DIR=/path/from/python-m-pybind11-cmakedir
cmake --build build-python --target _libgnsspp --parallel 2
python3 apps/gnss.py observation-analysis \
  --obs /datasets/PPC-Dataset/tokyo/run1/rover.obs \
  --nav /datasets/PPC-Dataset/tokyo/run1/base.nav \
  --bindings-dir build-python/python --max-epochs 300 \
  --output-dir output/observations-tokyo1
```

The default analyzes 300 epochs. `--max-epochs 0` reads the complete input.
Output must be new or empty. All satellites are plotted unless
`--plot-satellites G01 E02` selects a subset; CSV and summary always retain all
analyzed satellites. Windows builds use `--config Release` in the build command.
When the extension links GTSAM, add `--runtime-dir <GTSAM DLL directory>`.
On Linux, configure the library search path before launching Python when needed.

The command writes `observations.csv`, `summary.json` and
`satellites/<satellite_id>.png`. Inputs, package source files and the compiled
extension are SHA-256 identified. A passing summary means analysis and plotting
succeeded; it does not establish positioning accuracy or confirm an integer slip.

## Residual and continuity contract

The corrected pseudorange includes the native SPP atmospheric and satellite
clock corrections. At each valid SPP position, subtract the geometric range to
the Sagnac-corrected satellite position. Fit a weighted receiver-clock offset
within each native clock group, then report the remaining code residual.
GPS and QZSS can share a clock group; other systems retain their own group.
These diagnostic residuals include reconstructed corrected rows and are not
the SPP solver's postfit residuals or its outlier-admission decisions. A group
with fewer than two rows has no identifiable satellite residual and reports
null. A small residual is not independent accuracy evidence.

Carrier phase is in cycles, Doppler is in RINEX Hz, SNR is in dB-Hz and satellite
velocity is in ECEF metres per second. Between two samples of the same satellite,
signal and tracking code, the carrier-continuity residual is:

```text
phase_now - phase_previous + 0.5 * (doppler_now + doppler_previous) * elapsed_seconds
```

It vanishes for a continuous carrier with consistent Doppler sign and constant
phase rate. The default discontinuity threshold is 10 cycles. Doppler integration
is approximate: acceleration, coarse cadence and noisy or receiver-derived
Doppler can produce false indications. Change the threshold explicitly for the
receiver and retain the raw evidence when reviewing any indication.

The source LLI loss-of-lock bit, source lock flag and half-cycle ambiguity bit
are preserved separately. Missing carrier resets the arc. Missing Doppler,
unknown/changed carrier frequency, tracking-code changes and gaps over two
seconds withhold the continuity test rather than declaring a clean arc.

A common receiver-clock event candidate requires at least four comparable
satellites in the clock group, with at least 75 percent agreeing on a large
phase/Doppler step after conversion to metres. The median common step is removed
for the satellite-specific continuity test. Both raw and adjusted residuals,
the common step and reason codes are exported. Correlated satellite faults can
also create this pattern; the clock label remains a candidate and does not
exonerate source LLI warnings. Red plot markers denote slip indications, and
orange markers denote common-clock candidates.

## Use the API

```python
import libgnsspp

epochs = libgnsspp.preprocess_spp_file("rover.obs", "base.nav", max_epochs=300)
records, summary = libgnsspp.observations.analyze_epochs(
    epochs, slip_threshold_cycles=10.0, max_gap_s=2.0
)
```

Corrected measurements expose `satellite_id`, `signal_id`, `clock_group`,
`carrier_observation_type`, `carrier_frequency_hz`, `loss_of_lock_indicator`
and `source_loss_of_lock` in addition to the existing observables. The frequency
belongs to the raw source carrier even when the code measurement is an
ionosphere-free combination. No carrier ambiguity should be inferred from that
code-combination flag. The analyzer requires strictly increasing GPS epochs and
rejects duplicate satellite/signal/tracking rows within an epoch.

## Validation

`python3 tests/test_observation_analysis.py` checks weighted clock removal,
constellation clock separation, Doppler sign, individual and common jumps,
source lock flags, gaps, missing measurements, invalid solutions, tracking-code
changes and GPS week rollover. Native binding smoke and a bounded PPC run are
required in addition to these independent synthetic examples.

The 2026-10-06 local validation built the public extension with Python 3.12.10
and analyzed the first 300 epochs of PPC Tokyo run1. All 300 SPP positions were
valid; the export contains 9,432 rows from 33 satellites, with 7,955 assessed
phase/Doppler comparisons and 1,402 rows lacking carrier phase. It reports
24 slip indications from source loss-of-lock flags and no common-clock
candidate. These indications are not independent confirmations of integer
slips. Fourteen independent analyzer tests passed. The existing binding smoke
passed four tests and skipped eight dataset-dependent tests; a separate raw
PPC binding check verified 64 corrected rows, native clock groups and carrier
frequencies between 1 and 2 GHz.

Artifacts are under `output/observation-analysis-tokyo1-integrated-20261006/` in the
development worktree: `observations.csv`, `summary.json` and 33 satellite PNGs.
The summary records input, binding, analysis-script and output hashes. G05 and
C11 plots were visually reviewed; absent values and gaps break plotted lines.
This is a bounded example on existing development data, not a full-run or
held-out slip-detection benchmark.

The recorded input/binding hashes, satellite summaries and artifact manifest
are retained in [the verification record](ppc_python_observation_verification.json).
