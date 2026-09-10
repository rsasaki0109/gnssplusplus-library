# Phase390 — raw replay with TDCP robust diagnostics

Completed one native invocation, exit 0, 230.29665233194828 seconds.
Frozen manifest SHA256:
`cf84811363172cb6709bcbb21e69b997b93898065201313731d4ea42c8872d96`.
`scripts/verify_phase390_tdcp_diagnostic.py` passed source pins, completion,
both-stage convergence, candidate identity and aggregate consistency checks.

The 1466-row candidate is byte-identical to Phase382, SHA256
`d2c619121951d1a322ab39e15fcf16dbf7a6d61d2c651d03a4f68ea357c61e7d`.
Only its opaque bytes were hashed; no saved coordinates were supplied to
inference. Raw GNSS/IMU/nav only, zero truth reads or accuracy evaluations.
No new accuracy improvement is claimed.

Of 24964 finite TDCP residuals, 4259 (17.06056721679218%) are beyond the
configured Huber threshold. Reconstructed Huber cost is 153446.7752896991,
versus quadratic cost 525907.3166542781, ratio 0.2917753193963875. No invalid
cost samples. The quadratic total agrees with N*normalized_RMS^2/2.

This confirms that robust loss is active and substantially reduces the tail's
quadratic penalty; a high normalized RMS does not establish that robustness
is absent. These are postfit reconstructions, not independently summed graph
factor errors or a proof that TDCP causes the 3.1 m development error. The
diagnostic uses the existing ordinary-TDCP threshold resolver; the tested
recipe uses nonlinear ordinary TDCP, source metre sigma, and Huber 4.

Build passed and current CLI tests passed 27/27. Historical Phase116 tests
passed 2/5, with three errors on frozen source/binary SHA mismatches; their
pins were not rewritten. Full CTest and PPC real-data gate remain unrun.

Next compare H's same-recipe robust-tail aggregates before attributing the
LAX-T transfer weakness to a unique tail problem. Do not select another
Huber threshold from these reused development scores. Goal remains unmet.
