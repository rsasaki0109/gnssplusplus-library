# Phase313 unavailable correction attribution

Frozen at 5d1c4edb before reads. Verified source/binary/base/nav/phone hashes.
One hash scan plus one native pass per real input; same-process correction
generation/query only. No truth, candidate, MAT, station table, saved state,
positioning, network or Kaggle/token input; no score calculation.

All Phase311 support, signal-breakdown and base aggregates match exactly.
1819 unavailable corrections comprise 1343 queries before the individual
stream's first sample, 476 after its last sample and zero in-domain failures.
This establishes temporal-domain failure for these raw-parser queries, not
necessarily whole-route endpoint loss. No extrapolation/hold is justified.

The missing-stream 20172 observations remain separate (primarily absent
BeiDou base observations). Do not merge them with numerical failure or invent
zero corrections. Next integrate the existing finite-correction miss mask
with paired rover satellite/code-bias preparation, using actual FGO epoch
times and retained observations. Raw counts are not final factor counts.

Diagnostic compiled successfully. Previous 33 focused tests not rerun here;
no full CTest, solver or accuracy claim. Goal remains incomplete.
