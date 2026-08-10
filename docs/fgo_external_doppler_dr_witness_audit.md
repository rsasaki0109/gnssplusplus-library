# FGO external Doppler dead-reckoning witness audit

## Purpose

Determine whether a Doppler-only dead-reckoning (DR) track can safely promote
otherwise-FLOAT LAMBDA position candidates.  This is an offline, monitor-only
audit.  It does not add single-difference Doppler factors to the FGO graph,
change an ambiguity decision, seed a hold, or alter a reported position.

The originally proposed current-epoch SPP/LAMBDA agreement audit was not
repeated.  The completed multi-epoch audit already included a fresh SPP
witness within 5 m in its best zero-wrong development rule.  That combination
recovered only six correct epochs over all 36,346 Tokyo epochs (0.0165
percentage points) and was rejected.

## Research and implementation basis

- Bahrami and Ziebart use Doppler-smoothed code to improve instantaneous RTK
  ambiguity resolution: <https://doi.org/10.1109/PLANS.2010.5507202>.
- Jiang, Ding, and Gao use an external INS prediction and solution-separation
  statistic to improve partial ambiguity reliability:
  <https://doi.org/10.1016/j.asr.2024.09.048>.
- Wang et al. formulate ambiguity-resolution integrity monitoring with
  multiple-hypothesis solution separation:
  <https://doi.org/10.33012/2023.18607>.
- Teunissen et al. show why ambiguity-resolved model testing must account for
  both ambiguity residuals and observation-model inconsistency rather than
  relying on a conventional ratio alone:
  <https://doi.org/10.3390/app15073531>.

The repository already contains a dormant external Doppler-DR validator.  It
solves receiver-clock-drift-free single-difference Doppler velocity by
weighted least squares, removes rows beyond four sigma once, propagates a 3-D
position and covariance, and compares an integer candidate through a 3-D
Mahalanobis statistic.  Previously this was evaluated only on relaxed-ratio
activation paths.  `--external-dr-shadow` extends the telemetry to every
provisional LAMBDA candidate while leaving every decision path disabled.

The DR track resets only from a normally accepted candidate with ratio at
least 20 in shadow mode.  The reset epoch itself is not evaluated; candidate
testing begins after at least one Doppler propagation epoch.  The frozen
availability limit is 30 epochs and the frozen acceptance threshold is
chi-square 11.345 (99% for three degrees of freedom).

## Frozen development gate

Tokyo run1 is the only development run.  A runtime rescue experiment and
run2/run3 inspection are permitted only if the fixed shadow rule produces:

- at least 100 evaluated FLOAT candidates;
- at least 60 accepted correct FLOAT candidates (about +0.5 percentage points
  over 11,905 run1 epochs); and
- zero accepted wrong FLOAT candidates.

A candidate is correct only when its own ECEF position, transformed into the
CSV local ENU frame, is within 0.5 m in 3-D.  The reported FLOAT position is
not substituted for the candidate.  Reference truth is read only by the
offline scorer after the solve.

If run1 fails any requirement, the experiment stops: do not inspect run2 or
run3, do not enable the existing runtime validator, and retain the shadow only
as diagnostic evidence.  If it passes, freeze every threshold and require
zero accepted wrong candidates independently on run2 and run3 before any
default-off runtime A/B.

## Reproduction

Append `--external-dr-shadow` to the documented shipping FGO preset and dump
the epoch CSV.  The monitor deliberately does not require `--sd-doppler`:
single-difference Doppler rows are materialized for the private DR estimator
but are not inserted into the graph.

```powershell
python scripts/analysis/analyze_fgo_external_dr_witness.py `
  --epoch-csv build-ffrt-msvc/validation/tokyo1_external_dr_shadow.csv `
  --json build-ffrt-msvc/validation/tokyo1_external_dr_shadow.json `
  --markdown build-ffrt-msvc/validation/tokyo1_external_dr_shadow.md
```

The scorer exits successfully only when the frozen development gate passes.

## Tokyo run1 result

The 50-epoch authority-neutrality replay matched all status, ECEF position,
ratio, fixed-count, and AR-outcome fields exactly.  Only the nine intended
external-DR diagnostic columns differed.

The full 11,905-epoch run produced 8,216 candidate epochs.  The independent
track evaluated 252 otherwise-FLOAT candidates:

| Population | Count |
|---|---:|
| Correct / wrong evaluated FLOAT candidates | 115 / 137 |
| Accepted correct FLOAT candidates | 112 |
| Rejected correct FLOAT candidates | 3 |
| Accepted wrong FLOAT candidates | **112** |
| Rejected wrong FLOAT candidates | 25 |

**Verdict: FAIL.** The support and correct-yield requirements passed, but the
zero-wrong requirement failed by 112 candidates.  Doppler-only DR followed
the same wrong position basin too often to be an independent ambiguity
witness on this run.  Per the frozen protocol, run2/run3 were not inspected,
the runtime validator was not enabled, and no FIX-rate gain is claimed.  The
default-off shadow remains useful negative evidence and diagnostic telemetry.
