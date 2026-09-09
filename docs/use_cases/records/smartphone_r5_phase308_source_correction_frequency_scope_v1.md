# Phase308 source correction frequency scope

Starting HEAD 9a10109e; root-only. Phase307 failed before publication.

Source fgo_gnss_imu.m:36 fixes FTYPE to L1/L5 and lines 86 onward call base
correction only for those slots. Phase307 diagnostic instead preserved and
corrected all supported native bands. That is a scope mismatch, independently
of score. Galileo C7/C8 map to one native signal but distinct source slots;
both are outside source FTYPE. Per-stream attribution of the historical
failure remains unmeasured; do not claim that aggregate failure proves it.

Added default-off use_source_fgo_frequency_slots requiring source epoch
states. Satellite-state preparation still sees the whole selected epoch;
correction rows are then admitted only for source slots 0/2. Excluded rows
are counted. Duplicate stream rejection is unchanged. The existing native
code/SNR/epoch admission and baseline remain unchanged when flag is off.

Extended actual-model synthetic test: SBAS band 5 occupies source slot 1,
so it is counted/excluded while slot 0 correction remains usable. This
deliberately checks slot semantics rather than testing physical band digit 5.
No previous raw run is reclassified as successful. No raw/truth/MAT/candidate,
station-table/network/Kaggle input or score computation occurred here.

Next freeze a new raw model diagnostic with this source-backed scope; verify
counts and correction build before paired rover/FGO integration.

Validation: target build succeeded; all 33 base-compensation tests passed,
including extended positive-model/exclusion test. No full CTest/raw validation.
