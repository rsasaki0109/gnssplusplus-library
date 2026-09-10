# Phase364 — H raw carrier jump support

Primary-agent raw-only diagnostic `scripts/native_carrier_jump_audit.cpp`.
Compiled against existing native libraries and executed successfully on the
H device_gnss.csv pinned in the Phase361 manifest. Explicit Pixel5 parser
configuration, enriched pseudorange verification disabled. No navigation,
truth, MAT or saved positioning input; no solver or accuracy evaluation.

- Raw epochs: 3140.
- Finite carrier pairs at consecutive epochs after parser quality admission: 71680.
- Current endpoints exceeding 20000 cycles: **0**.
- Nonadjacent satellite/signal identity pairs within 1.5 seconds: **0**.
- Native shared L-D masked endpoints: 0.

An initial diagnostic invocation omitted the explicit device_model; the
source was corrected to Pixel5 and rebuilt/re-executed. Aggregates were
identical. Neither invocation read truth or computed positioning scores.

These are parser-level counts, not final FGO factor counts or full source
tracking-slot parity. Since the two suspected conditions never occur in
this admitted H population, neither justifies another H accuracy run or a
production mask change. The historical differences may matter on another
dataset, but they do not explain the current H target gap. Close this H
hypothesis without threshold tuning. Retain the operational baseline.
