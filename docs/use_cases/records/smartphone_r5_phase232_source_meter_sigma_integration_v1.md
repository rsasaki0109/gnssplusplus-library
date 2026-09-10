# Phase232: opt-in source metre sigma factor construction

Primary agent. Added default-off use_source_tdcp_meter_sigma and CLI
--native-source-tdcp-meter-sigma. CLI requires Pixel5 Phase171 ECEF-D with
explicit UTC fallback and rejects Phase117/118/120 mixtures. Builder and
optimizeProblem also reject those mixtures. New factor construction uses
the previous endpoint's SNR and the existing usable-SNR band percentile
collector, passing sourceTdcpSigmaMeters directly as sigma_m. Invalid sigma
rejects the pair without fixed-sigma fallback. Historical Phase117 and default
fixed sigma remain unchanged.

Summary adds requested selector and metre-unit text; fixed_sigma_m becomes
null when selected. These are configuration telemetry, not independent proof
of actual stage factor sigma. The legacy official_carrier_sigma_units field
still describes the historical helper and needs clearer conditional labeling
before freezing a new raw experiment. SNR population parity remains open.

Build session 38331 completed exit 0 for native app and gnss_run_tests.
Focused factor-construction/preprocessing/Phase171/mapping tests passed 21/21.
Fresh native CLI tests passed 19/19. New builder fixture verifies 4 unchanged
pair identities and delta-carrier values with 0.002 m sigma at reference SNR;
missing previous SNR rejects exactly one pair. Not full CTest.

Remaining before raw execution: add mixed-selector negative tests, strengthen
previous-endpoint discrimination with unequal endpoint SNR, and verify the
application's stage config inheritance and diagnostic wording. Current tests
prove builder behavior and CLI admission, not a new dynamic-sigma full staged
solve. No real-data run, truth/candidate payload read, MAT or evaluation in
this phase. H best remains Phase222/223 1.2680574850266653 m.
