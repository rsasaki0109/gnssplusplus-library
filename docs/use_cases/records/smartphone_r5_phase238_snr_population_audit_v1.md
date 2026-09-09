# Phase238: SNR masking population audit

Scope: source-code inspection only. No MAT, truth, candidate coordinates,
or solver execution. Phase234/235 remains the best H development reference.

## Finding

The hypothesis that source observation masks remove SNR samples from the
percentile population is not supported by the inspected code:

- `output/reproducibility-cache/gsdc2023/fgo_gnss_imu.m:65` calls
  `exobs`, then residual construction and `exobs_residuals`, then
  `obserrmodel` at line 95.
- `functions/exobs.m` copies the observation object and writes NaN into
  D, P and L only. Its status, multipath, SNR and carrier-jump masks do
  not modify S.
- `functions/exobs_residuals.m` likewise writes NaN into D, P and L,
  including elevation and adjacent Doppler/code/carrier consistency masks,
  without changing S.
- `output/reproducibility-cache/MatRTKLIB/+gt/Gobs.m:1116` residuals
  copies observations and adds residual fields; it does not mask S.
- `functions/obserrmodel.m` computes the per-band percentile over S
  using `prctile(..., "all")`, not the surviving carrier-factor subset.
- Native `collectOfficialTdcpSnrPercentiles` in
  `include/libgnss++/algorithms/observable_upstream_preprocessing.hpp:167`
  collects finite positive SNR from input epochs separately by band.
  `src/algorithms/fgo_problems.cpp:342` invokes it before factor admission.

Therefore do not introduce a post-factor-admission SNR percentile as a
source-parity correction. The temporal placement of native collection is
consistent with S surviving the inspected source masks.

## Limits and next action

This does not establish equality of source and native S populations.
Raw CSV ingestion, epoch/satellite/band mapping, duplicate-row selection,
unsupported signals, and zero/missing SNR handling can still differ.
Trace these construction steps next using source and native code, with
synthetic or raw-only diagnostics if needed. Do not infer population
equality merely from matching weighting formulas.

The source script's saved-result loads were inspected as text only and
must not be ported. Native initialization remains same-run in-memory.
Source atmospheric-residual parity also remains a separate open question.
