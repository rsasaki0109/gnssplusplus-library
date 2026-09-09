# Phase346: GPS L1 correction temporal common component

Added explicit `dense-temporal` mode to the raw phone correction-support
diagnostic. It constructs the existing source-complete shared-state dense
151-epoch base model from raw H base/nav, then queries raw phone P times.
For adjacent phone epochs separated by 0.5..1.5 s, difference corrections
for the same GPS L1 satellite and divide by elapsed seconds. Retain an
interval common-component proxy only with >=4 common satellites; take the
median signed rate, then report absolute-rate percentiles. This proxy is
not an independent receiver-clock measurement; satellite/atmosphere trends
can contribute. No FGO solve, correction applied, MAT, saved positions or truth.

Successful standalone build and one raw pass exited 0. Model support:
3500 base epochs, 68697 satellite states, 112050 source-frequency rows,
38 streams; raw phone 3140 epochs and 86947 finite queries. All 3139 phone
intervals met the common-satellite criterion. Absolute common-rate summary:

- median: 0.001010901588323845 m/s
- linear p95: 0.0029730021498723586 m/s
- maximum: 0.0080039075094461391 m/s
- maximum individual-satellite rate: 0.56409693465894983 m/s

No large common per-second correction jump appears in this GPS L1 proxy.
This does not bound slowly accumulated clock displacement, other bands,
the final FGO subset or individual satellite effects. The isolated larger
individual rate must not be called a common receiver-clock error. No evidence
here warrants correcting raw Doppler with this derived proxy. Production
recipe and development accuracy unchanged.

Build uses native libraries and C++17/O1; executable:
`/dev/shm/gnss-test-build-recovery.BJvQt9/correction_temporal_audit`.
Raw inputs: same H base/nav/phone paths as Phase340. Exploratory aggregate
diagnostic, not a hash-frozen accuracy experiment.
