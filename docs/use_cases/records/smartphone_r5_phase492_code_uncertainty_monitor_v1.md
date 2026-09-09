# Phase492 — retained-code uncertainty reach diagnostic

Confirmed current Android loader converts ReceivedSvTimeUncertaintyNanos to
metres and carries positive finite availability into Observation. Builder
copies it into each P factor before the later row selection, independently
of whether the optional floor is enabled. Therefore the retained P-factor
collection can quantify a prospective floor without rereading or rematching
raw CSV rows.

Added main-stage CLI stderr diagnostic after final row selection and before
optimizeProblem, only for Phase171 with the floor disabled. It reports retained
rows, positive finite uncertainty availability, how many uncertainty values
exceed current sigma, invalid sigma count and maximum floor/sigma ratio.
It changes neither weights nor seeds and does not emit individual observations.
When the floor is already on, this counterfactual diagnostic is suppressed
because comparing against the already-floored sigma would be misleading.

Main executable rebuild started; not yet a real-data result. Next run must
use floor-only numerical baseline settings (NHC off), pin current binary and
sources, and verify output equality against Phase479. No accuracy claim,
truth read, candidate promotion or submission in this step.
