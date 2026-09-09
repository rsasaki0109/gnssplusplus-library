# Phase493 — uncertainty floor affects every retained H code row

Single raw baseline replay via scripts/run_phase493_code_monitor.py completed
exit 0, PID 3763954 now terminal, elapsed 333.93518474395387 s. NHC off,
uncertainty floor off, numerical lambda floor on. Manifest, source/binary/raw
pins, started/completed metadata and logs are under
output/smartphone-r5/phase493-h-code-monitor-v1/mtv-h/.

After main-stage final row selection: retained 101916, uncertainty available
101916, source uncertainty greater than current sigma 101916, invalid sigma
0, maximum uncertainty/current-sigma ratio 3.43392 (printed precision).
No weights changed. The literal floor would weaken all retained P likelihoods,
not just a sparse subset. Relative effects across rows remain unquantified.

Output SHA256 equals Phase479 byte-for-byte:
4a0c4ef8822c72aa9134368cd57fe769817895e63b5dbe925783a9c8d091fa8e
Thus the diagnostic and disabled NHC implementation preserve this H output.
Runtime is a single observation, not a controlled performance comparison.

No truth reads, scoring, MAT or precomputed positioning inference input.
Next candidate must specify stage scope: the generic builder floor changes
measurement sigmas before both solves, so enabling it is not automatically
a main-only experiment. Preserve frozen raw selection and avoid score-based
scaling. Current source sigma floor has no fitted multiplier, but its effect
on this newer graph is untested. No promotion; overall goal unmet.
