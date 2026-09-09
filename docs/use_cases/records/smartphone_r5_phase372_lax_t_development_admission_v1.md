# Phase372 — third development route admission evidence

Selected existing raw route `2022-04-01-18-22-us-ca-lax-t/pixel5` for a
fixed baseline / code-gate-OFF comparison. Selection is for available raw
inputs and a third route, not a new heldout claim. Phase146's recorded
LAX-T score proves prior evaluation (old recipe 0.6230592714676478 m);
that result is not a current raw-native baseline or evidence of target
completion and its positioning payload must not be reused.

The current native raw carrier audit completed on this route: 1466 epochs,
26098 adjacent finite carrier pairs, zero >20000-cycle events, zero
nonadjacent <=1.5 s comparisons. Raw CMC >10 m occurs in 952 pairs, all
952 have finite Doppler witnesses within 1.5 m; maximum 0.509052 m rounded.
These are raw proxy counts, not final FGO admission counts.

A read-only CLI preflight with Phase249's recipe, LAX-T dataset ID and all
payload paths deliberately nonexistent reached raw GNSS ingress (expected
failure to open). Thus the baseline is not blocked by an earlier route
guard. The new code-gate selector still explicitly permits only H/U and
requires a tested LAX-T extension before use. No solver ran in preflight.

Next extend only this experimental selector's route admission, preserve
all measurement rules, test baseline and variant CLI paths, and freeze
both LAX-T raw runs before either new score. Use native first-epoch inclusion
consistently for the comparison; verify coverage rather than assuming it.
No truth or saved positioning payload was read in this audit. Operational
defaults and the full unachieved goal remain unchanged.
