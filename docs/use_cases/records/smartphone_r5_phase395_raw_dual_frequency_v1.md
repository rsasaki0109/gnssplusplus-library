# Phase395 — raw simultaneous-frequency carrier increments

Added `scripts/audit_raw_dual_frequency_carrier.py`. Reads only raw GPS/Galileo
ADR, state, frequency, satellite, epoch and clock columns, never enriched
positions/pseudoranges, MAT or truth. No inference outputs are reused.
Raw SHA256 matches Phase37 pins for H and LAX-T.

For simultaneous L1/L5 rows of a satellite, compute the temporal increment
of (ADR_L1 - ADR_L5), divided by UTC dt. Require valid ADR with reset/slip
bits clear at both endpoints, equal TimeNanos/TimeOffset/clock-count across
bands, unchanged hardware clock count across epochs, and dt in (0,1.5] s.
Reset pair history at every missing/invalid adjacent epoch. Duplicate
satellite/band keys fail. This is not the native P/D/L admission replica:
it does not apply C/N0, multipath or Doppler-consistency masks and does not
test half-cycle-resolution bits beyond the native-style reset/slip guard.

| Raw dual-frequency increment | LAX-T GPS | H GPS | LAX-T Galileo | H Galileo |
| --- | ---: | ---: | ---: | ---: |
| Count | 2503 | 12878 | 3229 | 7795 |
| Absolute P50, m/s | .005665675 | .005572969 | .008179100 | .006876184 |
| Absolute P95, m/s | .031357613 | .024630685 | .029218433 | .024194190 |
| Absolute max, m/s | .175145320 | .139336422 | 1.183351845 | .077331313 |
| Count above .1 m/s | 17 | 49 | 4 | 0 |
| Count above 1 m/s | 0 | 0 | 1 | 0 |

Both runs have zero detected cross-band time-offset/clock mismatches in
accepted pairs. Threshold counts are descriptive only, not new factor gates.
Five synthetic tests passed: common-mode cancellation, differential changes,
slip reset/no bridging, clock/offset rejection, duplicate rejection.

Frequency differences suppress ideal shared geometry/receiver-clock changes,
but are not pure ionosphere estimates: tracking noise, multipath, hardware
interfrequency effects and unflagged slips remain. No orbital/atmospheric
model or truth is used. The LAX-T Galileo maximum is of similar magnitude
to the E5a postfit tail, but no epoch identity match was performed, so do
not claim it is the same event or already identify an outlier gate.

The raw evidence supports investigating frequency-dependent measurement
error rather than attributing every TDCP discrepancy to a common clock.
Before implementing a frequency-dependent state, inspect existing residual
ionosphere/clock-state parameterization to avoid double counting and test
observability. Do not derive an ionosphere correction by fitting truth or
reuse these aggregate diagnostics as inference inputs. Goal remains unmet.
