# Sub-meter fleet tracking without a local base station

Use this route when the application needs better-than-standalone accuracy but
a private RTK base is not an option: fleet telematics, insurance telematics,
agricultural guidance previews, or asset tracking over wide areas. QZSS CLAS
(or MADOCA via L6E/L6D) delivers satellite-delivered corrections, so one
L6-capable rover receiver replaces the base-station link.

This page is the application framing. The decode smoke test and full option
surface live in [QZSS L6, CLAS, and MADOCA](qzss_l6.md); read that route
first if you have never decoded a capture here.

## What you need

| Item | Notes |
|---|---|
| Rover observations | RINEX from any multi-frequency receiver; raw UBX/SBF converts via `gnss convert` |
| Navigation file | Matching broadcast RINEX |
| L6 capture | Raw 250-byte frames logged by an L6-capable receiver; Japan-only coverage for CLAS |
| Reference (optional) | A surveyed point or independent solution to score against |

Without a reference the route still produces artifacts, but "sub-meter"
remains a claim you cannot check. Plan at least one scored run.

## One scored run

The frozen, public PPC Tokyo run1 decision route downloads the corresponding
QZSS archive hours, decodes Compact SSR, runs the complete interval, scores
against the independent PPC Applanix trajectory, and emits one manifest:

```bash
python3 apps/gnss.py clas-application-decision \
  --dataset-root data/PPC-Dataset \
  --output-dir output/use_cases/clas_decision/run1 \
  --l6-cache output/use_cases/clas_decision/l6_cache
```

The two hourly inputs are pinned as `2024205D.l6` and `2024205E.l6` (900,000
bytes each) with SHA-256 values in the manifest. The archive path is
`https://sys.qzss.go.jp/archives/l6/{year}/{YYYYDOY}{slot}.l6`; the files are
materialized from the QZSS public archive rather than copied into this
repository. `summary.json` separates FIX RMS2D, whole-route RMS2D, solution
availability, FIX rate, correction-time coverage, fallback epochs, first
correction, and first valid position. The manifest also hashes the POS,
scorecard, decode inventory/logs, KML, plot, and summary; missing or
non-finite products fail closed. Correction coverage is the union of
30-second Compact SSR intervals clipped to the solution POS time window, so a
valid-looking correction file outside the scored run is not counted. Missing,
entirely out-of-window, or explicitly lost corrections produce a non-zero exit and
`unusable` (or `degraded` when some in-window corrections remain), never a
silent success.

The frozen Tokyo run1 full replay on 2026-08-24 found 100% correction-time
coverage and 99.28% solution availability. FIX RMS2D was 0.352 m, but only
10.704% (1,270/11,865) of scored epochs were FIX, 2,884 epochs used the
standalone fallback state, and whole-route RMS2D was 41.862 m. The application
decision is therefore `degraded` and the continuous-sub-meter gate fails.
“Sub-meter” applies only to the explicitly counted FIX population, not the
whole route. With correction availability injected to zero, the same result
becomes `unusable` with `correction_unavailable`; fallback never masks the
loss.

For the fail-closed loss check, a test-only switch removes correction
availability from the decision (it never turns fallback into success):

```bash
python3 apps/gnss.py clas-application-decision \
  --dataset-root data/PPC-Dataset \
  --output-dir output/use_cases/clas_decision/run1 \
  --l6-cache output/use_cases/clas_decision/l6_cache \
  --reuse --inject-correction-loss
```

This intentionally exits non-zero with `state=unusable`. Do it only in a
copy of a result directory because it rewrites the decision summary.

```bash
mkdir -p output/use_cases/clas_submeter
python3 apps/gnss.py clas-ppp \
  --profile clas \
  --obs <rover.obs> \
  --nav <navigation.nav> \
  --qzss-l6 <qzss-l6-capture.bin> \
  --qzss-gps-week <week-of-capture> \
  --out output/use_cases/clas_submeter/clas_solution.pos \
  --summary-json output/use_cases/clas_submeter/clas_summary.json
```

The summary keys to archive are `epochs`, `ppp_fixed_epochs`,
`ppp_float_epochs`, `fallback_epochs`, and `ppp_solution_rate_pct`. Score
against your reference with the same matching conventions used by the
repository's moving-data gate (see `docs/ppc_clas_validation.md`) so results
stay comparable across runs.

For MADOCA L6E/L6D inputs use the native path instead:

```bash
python3 apps/gnss.py ppp \
  --obs <rover.obs> \
  --nav <navigation.nav> \
  --madoca-l6 <madoca-l6e-file> \
  --ar-method per-freq \
  --out output/use_cases/clas_submeter/madoca_solution.pos \
  --summary-json output/use_cases/clas_submeter/madoca_summary.json
```

CLAS, MADOCA L6E/L6D, and RTCM SSR are separate variants in the decision
table. Only CLAS is truth-scored by the frozen capture above. MADOCA depends
on its service/receiver signal and uses `--profile madoca`; RTCM SSR depends
on an IP caster and uses `--ssr-rtcm`. A CLAS score is never presented as
evidence for either alternative.

## Interpreting for an application decision

Published moving-data evidence for this correction path is in the README
"Moving CLAS PPP vs MRTKLIB" section: six urban runs with FIX RMS2D of
0.19-0.63 m on FIX epochs, but FIX rates of 9-38% — corrections help
precision when they hold, not availability everywhere. Translate that into
your application honestly:

- Guidance/telematics that tolerate float gaps: usually viable.
- Continuous cm-level control (machine guidance, lane keeping): not this
  route alone; combine with [IMU fusion](urban_rtk_fgo.md) or add RTK.
- Outside Japan or without L6 reception: no CLAS; SSR over IP (`--ssr`,
  RTCM SSR) is the repository's alternative input.

## First artifacts and exit criteria

```bash
test -s output/use_cases/clas_submeter/clas_solution.pos
grep -q '^% LibGNSS++ Position Solution' output/use_cases/clas_submeter/clas_solution.pos
python3 -c "import json;d=json.load(open('output/use_cases/clas_submeter/clas_summary.json'));print(d['epochs'],d.get('ppp_solution_rate_pct'))"
```

## Boundary and next step

Coverage, facility selection, and message-subtype support are bounded as
documented in [QZSS L6, CLAS, and MADOCA](qzss_l6.md). This route adds no
new decoder capability; it packages the existing one into an application
decision workflow.

Next step: log one day of paired rover + L6 captures on a vehicle that
matches your deployment, score them against the best available reference,
and only then commit to a correction service contract.
