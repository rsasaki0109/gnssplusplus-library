# Phase554 — clock-jump hypothesis is inactive on current three routes

Default C7 TDCP uses only the base-clock difference, as does the locally
cached reference TDCPFactor_XXCC.h (hc(0)=1). Ordinary C0/D insertion constrains
the six other C differences with zero sigmas on eligible intervals. Clock
jumps omit that factor; TDCP admission receives both adjacent clock-jump
flags and records ClockDiscontinuity rejection through tdcp_contract.

Inspected existing native summaries: Phase535 H and Phase538 U/LAX all report
android clock_discontinuities=0, clock_c0d_clock_jump_skips=0 and TDCP
rejected_clock_discontinuity=0. Thus a jump-specific policy change cannot
explain the current three-route results through these recorded events.
This does not establish that every possible raw hardware anomaly is detected.

Do not run a new jump-policy accuracy experiment on these routes. Code-phase
jump rejections are a different condition (H2229/U617/LAX951); they are not
hardware-clock jumps. Prior code-gate ablation and readmission experiments
remain relevant and should not be repeated as if this were a new hypothesis.
No source mutation, raw/native rerun, truth read or accuracy evaluation here.
