# Phase104 raw execution result

Status: `NO-GO`, fail-closed before native application startup.

The independently authorized matrix attempted exactly one MTV-A invocation
followed by exactly one LAX-T invocation. Both children returned `127` from
the dynamic loader because `libmetis-gtsam.so` was unavailable. The native
application, GNSS-first solve, main solve, and Phase104 sidecar were never
entered.

No raw bytes were read by the wrapper, no truth file was opened, no accuracy
calculation was performed, and no rerun or fallback occurred. Logs and partial
run metadata are preserved under the ignored Phase104 output root. The two
routes have no solution, stage sidecar, displacement statistics, or summary,
so truth-only attribution is not authorized or possible for this execution.

The loader environment must be corrected and a fresh independent authorization
would be required before any future run; this sealed result authorizes no
rerun.
