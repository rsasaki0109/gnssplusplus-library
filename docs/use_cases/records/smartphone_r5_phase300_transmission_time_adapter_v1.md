# Phase300 selected-ephemeris transmission time adapter

Starting HEAD e77533e6; root-only. Source transmission time now consumes an
Ephemeris and a positive selected pseudorange. Keplerian systems use toc;
GLONASS uses toe with -taun/gamn; SBAS uses toe and the pinned seph2clk
recurrence. The SBAS plus af1*t iteration is deliberately retained, not
silently substituted with the Keplerian polynomial iteration.

Reviewed geph2clk/seph2clk in the pinned MALIB ephemeris.c via in-memory
HTTPS (same commit as Phase291). Final satellite measurement clock remains
separate. The new adapter does not select ephemerides or call propagation,
and is not wired into base or rover FGO. Selection-time/health policy and
final state parity remain required integration work.

Synthetic tests exercise GPS transmission time across a week boundary,
GLONASS sign, the literal SBAS recurrence, invalid ephemeris and nonpositive
range rejection. Positive range is a stricter propagation admission than
the Phase292 selector's preservation of finite negative values. No accuracy
or full source-state parity is inferred from these tests.

No real raw, truth, candidate, MAT, station table or Kaggle/token input.
Only algorithm source was fetched. No positioning solver or score run.

Validation: gnss_run_tests target built successfully; all 27 base-compensation
tests passed. Synthetic fixture I/O only; no full CTest or raw parity run.
