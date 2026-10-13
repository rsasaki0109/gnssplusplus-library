# Interfaces

See [received-event RTK/IMU](online_rtk_imu.md) for the sequential native API
and stdin executable, including input availability and timestamp contracts.

See [runtime FIX recovery](fix_recovery_guard.md) for optional residual
containment, clean-candidate recovery and the off/on evaluation contract.

[Python satellite observation analysis](python_observation_analysis.md) provides
code residual, SNR and carrier-continuity diagnostics from the public bindings.

## CLI

The primary interface is the `gnss` dispatcher plus native command binaries.

Examples:

- `gnss doctor`
- `gnss ros2-doctor`
- `gnss ros2-bag-doctor`
- `gnss field-report`
- `gnss robotics-smoke`
- `gnss spp`
- `gnss solve` (`--pos-uncertainty` appends per-epoch sigma columns, see below)
- `gnss uncertainty-eval` (truth-based honesty check of those sigma columns)
- `gnss ppp` (`--nav`, `--sp3`, `--clk`, `--ionex`, `--dcb`, `--ssr`, `--ssr-rtcm`)
- `gnss clas-ppp`
- `gnss qzss-l6-info` (`--compact-flush-policy <lag-tolerant-union|orbit-or-clock-only|orbit-and-clock-only>`, `--compact-atmos-merge-policy <stec-coeff-carry|no-carry|network-locked-stec-coeff-carry>`, `--compact-atmos-subtype-merge-policy <union|gridded-priority|combined-priority>`, `--compact-phase-bias-merge-policy <latest-union|message-reset|selected-mask-prune>`, `--compact-phase-bias-source-policy <arrival-order|subtype5-priority|subtype6-priority>`, `--compact-code-bias-composition-policy <direct-values|base-plus-network|base-only-if-present>`, `--compact-code-bias-bank-policy <pending-epoch|same-30s-bank|close-30s-bank|latest-preceding-bank|delayed-15s-bank>`, `--compact-phase-bias-composition-policy <direct-values|base-plus-network|base-only-if-present>`, `--compact-phase-bias-bank-policy <pending-epoch|same-30s-bank|close-30s-bank|latest-preceding-bank>`, `--compact-bias-row-materialization <overlap-only|selected-satellite-base-extend|all-base-satellite-extend>`, `--compact-row-construction-policy <independent|coupled-code-phase|row-first-value-second|network-row-driven>`)
- `gnss fetch-products`
- `gnss ppp-products-signoff`
- `gnss ionex-info`
- `gnss dcb-info`
- `gnss visibility`
- `gnss visibility-plot`
- `gnss moving-base-plot`
- `gnss moving-base-signoff`
- `gnss moving-base-prepare`
- `gnss scorpion-moving-base-signoff`
- `gnss stream`
- `gnss convert`
- `gnss live`
- `gnss realtime-stress` (recorded-data real-time stress harness, see [Real-time stress](realtime_stress.md))
- `gnss rtcm-encode`
- `gnss station` (`check|start|status|stop|restart`, with a station TOML)
- `gnss rcv`
- `gnss web`

For longer workflows, `gnss web`, `gnss live-signoff`,
`gnss moving-base-signoff`, `gnss scorpion-moving-base-signoff`,
`gnss ppc-rtk-signoff`, `gnss ppc-coverage-matrix`, and
`gnss ppp-products-signoff`
also accept `--config-toml <file>`. Templates are indexed in
`configs/README.md` and grouped by purpose.

For CLAS-oriented PPP experiments, `gnss ppp` also exposes the boundary and
selector knobs used by the experiment lane:

- `--clas-epoch-policy <strict-osr|hybrid-standard-ppp>`
- `--clas-osr-application <full-osr|orbit-clock-bias|orbit-clock-only>`
- `--clas-phase-continuity <full-repair|sis-continuity-only|repair-only|raw-phase-bias|no-phase-bias>`
- `--clas-phase-bias-values <full|phase-bias-only|compensation-only>`
- `--clas-phase-bias-reference-time <phase-bias-reference|clock-reference|observation-epoch>`
- `--clas-ssr-timing <lag-tolerant|clock-bound-phase-bias|clock-bound-atmos-and-phase-bias>`
- `--clas-expanded-values <full-composed|residual-only|polynomial-only>`
- `--clas-subtype12-values <full|planar|offset-only>`
- `--clas-residual-sampling <indexed-or-mean|indexed-only|mean-only>`
- `--clas-atmos-selection <grid-first|grid-guarded|balanced|freshness-first>`
- `--clas-atmos-stale-after-seconds <seconds>`

### Multi-constellation RTCM epoch assembly (`--no-rtcm-epoch-assembly`)

A receiver sends one MSM message per constellation with the same time tag and
sets the RTCM "multiple message" bit (legacy 1003/1004: synchronous GNSS flag)
on every message of the epoch except the last. `gnss live` (rover and base RTCM),
`gnss replay` (base RTCM) and `gnss convert` (RTCM to RINEX) merge those parts
into one epoch with one shared implementation
(`io::ObservationEpochAssembler`, fed by `RTCMStreamDecoder::Event::
observation_more_follows` or `RTCMProcessor::lastObservationMoreFollows()`):

- a part whose time differs from the pending epoch by more than 1 ms emits the
  pending epoch first (a lost last message cannot merge two epochs);
- otherwise it is merged; a (satellite, signal) already present is kept once;
- bit 0 emits the epoch immediately, so no latency is added;
- the pending epoch is flushed at end of stream.

A stream whose messages all carry bit 0 (every GPS-only stream) is unchanged.
It is on by default; `--no-rtcm-epoch-assembly` (in all three tools) restores
one epoch per observation message. `gnss rtcm-encode` sets the bit on every
observation message of an epoch except the last one emitted
(`RTCMProcessor::encodeObservations(..., more_follows)` defaults to `false`).
`gnss stream` only prints one diagnostic line per message and does not build
epochs, so it is not affected.

Parts can only be merged if their full GPST agrees, so `RTCMStreamDecoder`
keeps one GPST reference (`setReferenceTime`, `referenceTime`):

- non-GLONASS MSM messages carry only a time of week; the decoder gives such a
  part (week 0, or BeiDou's week-1 wrap) the week nearest to the reference, and
  the reference follows every resolved observation time (so a week rollover is
  followed);
- while a reference exists it is passed to the processor before every message,
  so GLONASS MSM epochs and 1020 ephemerides are dated from the stream and not
  from the host clock;
- the reference is seeded explicitly, otherwise from the toe of the first
  decoded GPS/Galileo/BeiDou/QZSS ephemeris. A time derived from the host clock
  (GLONASS MSM or 1020 decoded without a reference) never seeds or advances it;
- without a reference TOW-only parts stay week 0 and GLONASS uses the host
  clock, as before.

`gnss live` seeds both RTCM decoders from `--nav-rinex` (latest GPS toe); when
one decoder obtains a reference from stream ephemerides and the other has none
(a rover stream normally carries none) the other is seeded with it, and a
week-0 epoch from a UBX rover is given the week nearest to the newest decoder
reference. Without `--nav-rinex` and without any ephemeris in either stream the
head of the stream is not resolved (week 0, host-clock GLONASS).
`gnss convert` keeps its own `--gps-week` resolution. `gnss replay --base-rtcm`
decodes the base with the same `RTCMStreamDecoder` (and the same assembler): it
is seeded from `--nav-rinex` (latest GPS toe) when given, otherwise from the
first in-stream GPS/Galileo/BeiDou/QZSS ephemeris, so the base epochs get the
same GPST week as a rover RINEX and the result does not depend on the host
clock. Base epochs decoded before an in-stream seed existed (no `--nav-rinex`,
no early ephemeris) are resolved against that first reference after the pass.
Only a stream with neither a `--nav-rinex` nor any such ephemeris stays in
week 0 and cannot be aligned to a rover RINEX.

### RTCM ephemeris messages (`gnss rtcm-encode --eph-systems`)

`RTCMProcessor` decodes and encodes broadcast ephemeris messages for five
constellations:

| message | constellation | notes |
|---|---|---|
| 1019 | GPS | |
| 1020 | GLONASS | time tag resolved against the stream reference, not the host clock |
| 1042 | BeiDou | layout and scaling as RTKLIB 2.4.3 b34 `decode_type1042` |
| 1044 | QZSS | as `decode_type1044` |
| 1045 / 1046 | Galileo F/NAV / I/NAV | as `decode_type1045/1046` |

The decoded `Ephemeris` is the record the RINEX navigation reader produces for
the same broadcast:

- BeiDou: `toe` and `toc` are GPST (BDT week + 1356, + 14 s); `week` is the BDT
  week and `toes` the raw BDT toe seconds, which the satellite-position code
  uses. `tgd` is TGD1 (B1/B3), `tgd_secondary` is TGD2 (B2/B3), `iode` is AODE
  and `iodc` is AODC. GEO satellites (PRN 1 to 5 and 59 to 63) need nothing
  extra: the position code selects the GEO orbit model from the PRN.
- QZSS: the library stores PRNs as printed in RINEX (J01 is 1), and the 4-bit
  satellite field of 1044 is that number. The 10-bit week is resolved like
  1019: against the stream reference when one exists.
- Galileo: 1045 sets `data_source_code` to the F/NAV word (258) and 1046 to the
  I/NAV word (513), as before.
- In all three, `toc` is placed in the week nearest `toe`.

`isEphemerisMessage()` is true for all six types, so `RTCMStreamDecoder`
(`gnss live`, `gnss replay --base-rtcm`), `gnss convert --nav-out` and
`gnss stream --decode-navigation` decode them. Before this, 1045/1046 were decoded
only by `mergeRTCMEphemerides` (used by `gnss ppp --ssr-rtcm`), which is
unchanged and still merges GPS 1019 and Galileo 1045/1046 only. A decoded
GPS, Galileo, BeiDou or QZSS ephemeris can seed the stream reference (see
above).

`RTCMProcessor::encodeEphemeris` writes the same six messages. The Galileo
message follows the record's `navigation_message_type`, else its RINEX
data-source word; a record whose type cannot be determined is not encoded. A
value that does not fit its field (for example a BeiDou `af0` beyond 24 bits)
gives an invalid message, as for 1019. Not carried: the L2 code and fit
interval bits of 1044, and the Galileo reserved bits.

`gnss rtcm-encode --eph-systems <G,R,E,J,C>` selects the constellations that get
ephemeris messages: G 1019, R 1020, E 1045/1046, J 1044, C 1042. The default
is `G,R`, which gives the same bytes as before the option existed. Only
systems that are also in `--systems` are broadcast, because a consumer infers
the week from the first ephemeris it sees. Round-robin (`--eph-interval`) and
the initial burst (`--no-eph-burst`) cover the added systems. `gnss
realtime-stress prepare --eph-systems` passes the list to the base encode
only; without it the command line is unchanged.

Verification on the navigation file of the UrbanNav Tokyo runs (DLR BCEmerge `base.nav`): every Galileo (4645),
BeiDou (696) and QZSS (95, J01, J02, J03, J07) record was encoded, decoded and compared with the library's own
satellite position and clock function at nine times from toe - 2 h to toe + 2 h. All 5436 were encoded, and the worst
difference was 0.053 mm in position and 5e-16 s in clock. The same records, decoded by RTKLIB 2.4.3 b34 `rtcm3.c` and
re-read from its RINEX output (0.63 mm and 5e-15 s at worst), and encoded by RTKLIB `gen_rtcm3` and decoded by the
library (the same 0.053 mm), agreed within the tolerance of 0.01 m and 1e-11 s. 693 of the 696 BeiDou 1042 frames are
bit-identical to RTKLIB's. RTKLIB differs from the encoder in the Galileo SISA index (it truncates where the encoder
takes the nearest index), and in the 1044 L2 code and fit interval bits. No 1044 stream from a real receiver or caster
was available: the real QZSS records are covered by those round trips and by the bit-level comparison with RTKLIB, not
by a live broadcast.

Ephemerides in the stream are not a drop-in for `--nav-rinex` when the file has an `IONOSPHERIC CORR` header (the PPC
navigation files do): `gnss live --nav-rinex` reads those parameters and applies a Klobuchar model in its SPP seed, and a
stream carries none, so without a navigation RINEX `nav.ionosphere_model.valid` stays false and no model is applied.
With a file that has no such header (the UrbanNav BCEmerge file) the two give the same status on 99.4 to 100 % of the
epochs and the same position to the print precision at p50
([measured](realtime_stress.md#ephemerides-in-the-base-stream-eph-systems)).

### Broadcast ephemerides from a u-blox rover (`--ubx-nav`, `--nav-rinex-out`)

A u-blox receiver logs the navigation subframes of the satellites it tracks
(`UBX-RXM-SFRBX`). `UBXNavDecoder` (`ubx_nav_decode.hpp`) turns them into
`Ephemeris` records and the GPS Klobuchar set, so a u-blox-only stream no
longer needs a navigation RINEX or an RTCM base that sends ephemerides.

| System | Signal (`sigId`) | Message | Record complete on | Checks |
|---|---|---|---|---|
| GPS, QZSS | L1 C/A (0) | LNAV subframes 1 to 3 | subframe 3 | IODE of subframes 2 and 3 equals the low byte of IODC |
| GPS | L1 C/A (0) | subframe 4, page 18 (SV id 56) | Klobuchar alpha/beta, no ephemeris | none |
| Galileo | E1-B (1; 0 before protocol 27) | I/NAV word types 1 to 5 | word type 5 | CRC-24Q of every page pair, one IODnav in word types 1 to 4, word type 4 names the satellite |
| BeiDou MEO/IGSO | B1I, B2I, B3I D1 (0, 2, 4) | D1 subframes 1 to 3 | subframe 3 | consecutive subframe times, `toc == toe` |
| BeiDou GEO | B1I, B2I, B3I D2 (1, 3, 10; or `sigId` 0 on PRN 1 to 5, 59 to 63) | D2 subframe 1, pages 1 to 10 | page 10 | consecutive page times, `toc == toe` |
| GLONASS | L1OF, L2OF (0, 2) | strings 1 to 4 | string 4 | Hamming code of every string, string 4 names the slot |

Not decoded: GPS and QZSS CNAV (L2C, L5), QZSS L1S, Galileo F/NAV (E5a) and
E5b-I I/NAV, BeiDou B-CNAV (B1C, B2a), NavIC, SBAS. They are counted as skipped.

The layouts, scale factors, completion triggers and consistency checks follow
RTKLIB 2.4.3 b34 (`rcv/ublox.c`, `rcvraw.c`). GPS and BeiDou words are not
parity-checked, as in RTKLIB: the receiver forwards only subframes that passed
its own parity check. Signals of one message layout share a buffer, because a
receiver forwards only some of the strings or subframes of each signal (BeiDou
D1 on B1I and B2I, GLONASS L1OF and L2OF). Records are emitted in the form the
RINEX navigation reader produces for the same broadcast; Galileo records carry
the data-source word of RTCM 1046 (513: I/NAV E1-B, clock referenced to E5b/E1).

**Time reference.** The 10-bit GPS and QZSS week and the GLONASS day need the
receiver time, which is the newest `UBX-RXM-RAWX` epoch of the same stream.
Without one those records are dropped (`no_time_reference`); Galileo and BeiDou
carry full weeks. The GPS week is the one nearest to the reference, not the
latest one before the host clock as in RTKLIB, so old logs decode correctly.

**`gnss live --ubx-nav`** (default off; without it the `.pos`, the options and
the output are byte-identical to earlier releases). Decoded ephemerides are
merged with `addEphemerisIfNew`, like RTCM ephemerides: the first copy of a
record at once, taken back out if a later copy of the same issue differs
(accept, then confirm; below). The first decoded GPS
Klobuchar set is used when no ionosphere model is valid yet; a `--nav-rinex`
that has an `IONOSPHERIC CORR` header wins. With the flag the rover's first
RAWX epoch is read before the base and becomes the GPST reference of the base
decoder when it has none: a base stream without ephemerides would otherwise be
dated from the host clock and never align with the rover. `--ubx-nav` skips a
record whose health marks the satellite unusable and counts it: GPS and QZSS
LNAV `svh` other than 0, Galileo E1-B `HS` or `DVS` other than 0, BeiDou `SatH1`
other than 0, GLONASS `Bn` MSB set or string-3 `ln` set (`Ephemeris::glonass_ln`,
filled by this decoder only). The `summary:` line gains `ubx_nav_ephemerides=`
(records added to the database, a record that was retracted and merged again
counts twice), `ubx_nav_klobuchar=` (0/1), `ubx_nav_unhealthy_skipped=`,
`ubx_nav_unconfirmed=`, `ubx_nav_conflicts=` and `ubx_nav_retracted=`
(defined below). `--ubx-nav` needs `--rover-ubx`.

**Accept, then confirm (`gnss live --ubx-nav`).** Waiting for a second
transmission before using an ephemeris (confirm-twice, below) delays every record
by one repeat of its frame set: at a cold start the filter had ephemerides for
only a few satellites for the first minute, and on one development log it fixed
one L1 cycle off and held that fix. `--ubx-nav` therefore merges a record at once
and uses the second copy only to catch a corrupted one
(`UBXNavAcceptConfirm`, `ubx_nav_decode.hpp`). Per satellite, on every complete
frame set the decoder produces (also the repeats it does not emit):

1. A copy of an issue not seen for that satellite (the first record, or a new
   IODE/toe) is merged at once, after the health rule. The previous record stays.
   "Issue" is what the decoder itself uses to tell repeats: GPS/QZSS IODE, IODC,
   toe and toc; Galileo IODnav, toe and toc; BeiDou toe; GLONASS IODE.
2. A later copy of that issue that is equal in every broadcast field (as in
   confirm-twice) and assembled entirely from SFRBX messages received after the
   first copy's last message *confirms* it. An equal copy that reuses a message of
   the first one is the same transmission and changes nothing.
3. A later copy that differs is a *conflict*. The record that was merged is removed
   from the navigation data (`NavigationData::removeEphemeris`, which erases only a
   record equal in every field, so a record that came from `--nav-rinex` or RTCM is
   never touched), and that satellite falls back to confirm-twice: nothing is merged
   for it until two equal copies from different transmissions exist. The first copy
   stays a pending candidate next to the differing one, because either may be the
   corrupted one, so the next copy equal to the first confirms it (at most four
   candidates are kept). After a confirmation the satellite is back at rule 1 for its
   next record.
4. A copy that differs from a record that was already confirmed and is in the
   navigation data is ignored: two agreeing transmissions outweigh one differing copy,
   so a confirmed record is never retracted. If the confirmed record never got into the
   navigation data (the health rule refused it), a differing copy is a record the
   navigation data does not hold and follows rule 1; BeiDou satellites clear their health
   flag within one toe this way.

Counters on the `summary:` line: `ubx_nav_conflicts=` is the number of copies that
differed from an unconfirmed copy (a conflict at rule 3, and every later copy that
matched no pending candidate of a satellite in fallback); `ubx_nav_retracted=`
the records actually removed from the navigation data by rule 3;
`ubx_nav_unconfirmed=` the records merged and never confirmed (still unconfirmed at
the end, or forgotten unconfirmed), plus, for satellites in fallback, the
candidates that were never confirmed (replaced or left pending; the retracted record
is counted by `ubx_nav_retracted=` and not again here). A single transmission of a
record, such as a satellite that sets after one frame set, therefore shows up as
`ubx_nav_unconfirmed=`, not as a conflict.

The price of merging at once is the exposure: a copy corrupted in a way the
decoder's checks do not catch is in the navigation data from its first copy until a
second copy of the same issue arrives (one repeat of the frame set, 30 s at the
median) and is retracted. A corrupted record that its satellite never repeats is
never retracted and stays selectable for its validity window; it is the one thing
confirm-twice removes that this does not, and `gnss convert --nav-rinex-out`
therefore keeps confirm-twice. A corruption that lands in an issue field makes the
copy look like a new issue (rule 1), and is merged at once as well.

**Confirm-twice (`gnss convert --nav-rinex-out`).** The converter writes a record only
when the decoder has produced the same record twice from different transmissions
(`UBXNavConfirmTwice`); a file has no start-up latency to protect. (`--ubx-nav`
uses it only for a satellite in fallback, above.) "The same" is every broadcast field of the `Ephemeris`,
bit for bit; only `tof`, which is resolved against the receiver clock, is left
out. "Different transmissions" means the second copy is assembled entirely from
SFRBX messages received after the last message of the first, so a copy that
reuses a stored subframe of the first one does not count. A copy that differs
from the pending one replaces it (counted in `nav_rinex:` as `conflicts=`), a copy
equal to an already accepted record is a duplicate, and `unconfirmed=` counts the
pending ones that were never confirmed, replaced or left at the end. A record that was corrupted in one
transmission and passed the checks above (the GLONASS record of the measurement
below) is therefore never accepted, at the price of one more transmission before
every new ephemeris is written: 30 s at the median, 60 to 111 s at the 95th
percentile and 240 s at most per system on the 2.67 h log below. The decoder
(`UBXNavDecoder`, also behind `gnss ubx-info --decode-nav`) still reports every
record, so the counts under "Measured" are decoder output, before this step.

**`gnss convert --format ubx --nav-rinex-out <file>`** writes the decoded
ephemerides as RINEX 3.04 navigation, with the last decoded Klobuchar set as
`GPSA`/`GPSB` header lines, when the input ends, and prints a `nav_rinex:` line
(count per system, `klobuchar=0|1`, `unconfirmed=`, `conflicts=`). It writes every record the receiver sent,
unhealthy ones included, with their health word: a converter keeps what it was
given, and only once confirmed twice (unchanged by accept-then-confirm). It is separate from `--nav-out`, which keeps its own converter-side
decoders and writes no Klobuchar lines. `gnss ubx-info --decode-nav` prints a
`nav_decode:` line with the same counts (also unfiltered).

Limits. A receiver forwards the navigation data only of satellites it decodes,
and often only part of the subframes, so a satellite that is tracked but not
decoded gets no ephemeris and a new one takes between a few seconds and a few
minutes (see below). A satellite that broadcasts a default or unhealthy data set
(GPS and QZSS health 63 with URA index 15, BeiDou health 1) gives a record like any
other, which IGS BRDC files leave out; the decoder emits it, `--ubx-nav` skips it
and `--nav-rinex-out` writes it. `getEphemeris` itself does not look at health, so
a record loaded from a RINEX file or an RTCM stream is still selected on toe alone.
Each record is as old as its subframes: decoding does not check the ephemeris
against the time. GLONASS strings are protected by a Hamming code only, which is
accepted with the single-error allowance of RTKLIB and cannot reject every
multi-bit error: one record in the measurement below (R16, `tb` 12 where the other
strings of the frame say 15) is such a corruption, decoded identically by RTKLIB;
confirm-twice (`--nav-rinex-out`, and a satellite in fallback) does not accept it,
because the satellite never repeats it. `--ubx-nav` merges a first copy at once, so
a record like that one is merged and, with no second copy to differ from it, stays
(counted in `ubx_nav_unconfirmed=`) for its validity window.

`gnss live --rover-ubx` reads a UBX file in 4096-byte chunks and takes the first
RAWX epoch of each chunk, so a log with several epochs per chunk loses the others
(a 139 kB log of 201 epochs gave 34 aligned epochs). That predates `--ubx-nav`
and is unchanged; the 3DOM log has one epoch per 5 kB and lost none.

Measured. History: the first round, scored on two logs against a frozen plan,
failed several gates for causes outside the decoder arithmetic (forwarded default
data sets selected among real records, a completeness definition, and an RTKLIB
that cannot decode u-blox Galileo E1-B); the second round added the health rule
above, redefined those gates and was scored on logs not used before (develop
`0f4153dc`).

*Held-out log (Laura Bassi, 2025-10-15, L1/L5 receiver, 2.67 h, 2 Hz, IGS BKG BRDC
2025/288; Galileo E5a, BeiDou B2a and GPS L5 CNAV messages are counted and not
decoded).* A decoded record is compared with the BRDC record of the same
satellite, issue (IODE, IODnav, AODE or tb) and toe by the library's own
satellite position and clock at nine times from toe - 2 h to toe + 2 h (GLONASS
seven times, tb +- 15 min).

| | GPS | GLONASS | Galileo | BeiDou | QZSS |
|---|---|---|---|---|---|
| Records decoded (of which unhealthy) | 37 (0) | 69 (0) | 92 (0) | 44 (5) | 0 |
| Healthy records with a BRDC record of the same issue and toe | 37 of 37 | 68 of 69 | 89 of 92 | 30 of 39 | - |
| The same, BRDC 2025/287 and 288 together | 37 of 37 | 68 of 69 | 92 of 92 | 39 of 39 | - |
| Worst difference of the matched records | 0.23 mm, 5e-16 s | 0.07 mm, 5e-16 s | 0.22 mm, 1e-15 s | 0.27 mm, 5e-16 s | - |
| Same records as RTKLIB 2.4.3 b34 `convbin` (Galileo: with the patch below) | 37 of 37 | 68 of 68 | 92 of 92 | 44 of 44 | - |

The 13 healthy records without a BRDC match are 12 sets whose toe is on
2025-10-14 (the log starts at 23:58:27 UTC, and the daily BRDC file of the 15th
does not carry them; the file of the 14th does) and one GLONASS record (R16)
whose `tb` field was corrupted in transit in a way the Hamming code does not
reject: the other strings of that frame carry `tb` 15, and the record is up to 9
470 km off the neighbouring records within 15 min of its toe. RTKLIB decodes the
same record. The GPS Klobuchar sets (17 distinct ones) are identical to the
RTKLIB values in every printed digit, and the last one equals the BRDC header at
its print precision. The five unhealthy records are BeiDou satellites with
health 1; BRDC carries four of them.

Time from the first RAWX epoch of a satellite to its first ephemeris (median / 95th
percentile): GPS 32.5 s / 1952 s, GLONASS 36.5 s / 2037 s, Galileo 41.5 s / 241 s,
BeiDou 16.5 s / 1500 s. For a satellite tracked for at least 30 min with at least one
LNAV/I-NAV/D1/strings message in its first 15 min, a healthy ephemeris valid at 15
min had been decoded for GPS 11 of 12 (92 %), GLONASS 14 of 14, Galileo 11 of 11
and BeiDou 11 of 13; the misses are a rising GPS satellite for which the receiver
forwarded 3 messages in 15 min, a BeiDou satellite with 4, and BeiDou C49, whose
health is 1 throughout. One tracked BeiDou satellite had no message in its first
15 min and is not counted.

RTKLIB b34 decodes none of the Galileo records: it takes an E1-B message of 8
words (every protocol 27 and later receiver) for E5b-I and drops it, and wants a
word type 6 in the buffer. With one line changed (E5b-I told by `sigId`) it
returns exactly the records of this decoder. `convbin` also scans the file once
before converting and keeps the subframe buffers, so its first records can be
completed with subframes from the end of the file; the comparison above uses a
conversion pass with those buffers cleared. A differential run of the decoder
against RTKLIB's `input_raw` on 12 000 random but consistent records (QZSS and
BeiDou D2 included, 2000 per layout) gave identical records to 1e-12, and the
single-pass decoders agree on every record of this log to 1e-12 relative.

*End to end.* `--ubx-nav` against `--nav-rinex` is **not validated on a held-out
log**: no rover log with a same-day base that was not used before was available
to this evaluation (the 3DOM archive is a single 5.9 GB xz stream in random
order, and finding the day the plan called for exceeded the download budget). On
the log used while developing the rule (3DOM rover A, 2025-06-04, 5.25 h, base
3.6 km away, base encoded to RTCM without ephemeris messages, `--preset
low-cost`, BRDC 2025/155, 18 895 rover epochs), `--ubx-nav` gave 18 877 against
18 895 solutions, the same status on 99.88 % of the epochs, 18 873 against 18
891 FIXED epochs, a horizontal difference between the two FIXED positions of 0.0
mm at p50 and 12.6 mm at most, and no FIXED epoch more than 0.5 m from the
median FIXED position of the `--nav-rinex` run. The 15 unhealthy records of that
log (GPS and QZSS health 63, BeiDou health 1) were skipped. Without the health
rule the same run had 18 700 FIXED epochs and 975 FIXED epochs more than 0.5 m
from that position (up to 1.2 m from the `--nav-rinex` position of the same
epoch), because a default data set of G23 and G27 with the same toe as the real
one was selected. A rover with a clear sky and a base 3.6 km away is the easy
case: nothing here says how `--ubx-nav` behaves in a canyon, with a poor antenna
or on a moving rover.

### Causal output guard for `gnss live` (`--kinematic-post-filter`)

Opt-in (default off; without it `gnss live` output is byte-identical to
earlier releases) and adds no output delay. Once a FIX has been written, an
epoch whose height differs from the last *written* epoch by more than
`max(1 m, 4 m/s * dt)` (`dt <= 0` counts as 1 s) is not written, and a dropped
epoch does not move the anchor. This is the height-step rule of `gnss solve`'s
kinematic post-filter; both tools share one implementation
(`rtk_validation::KinematicPostFilter`). With the flag the `summary:` line
gains `post_filter_dropped_epochs=`; dropped epochs are missing rows in the
`.pos`. Measured effect and its limits: [Real-time stress
harness](realtime_stress.md#wrong-fix-on-the-real-time-path).

### Stall watchdog for `gnss live` (`--stall-timeout-ms`)

Opt-in (default `0` = off: reads block as before and the output, including the
`summary:` line, is unchanged). A `tcp://` source that stays connected but sends
nothing blocks `read()` forever; `--reconnect` only reacts to read errors and
EOF, so a dead peer that never sends a FIN hangs `gnss live`.

With `--stall-timeout-ms <ms>` (`> 0`) `gnss live` polls a `tcp://` or
`serial://`/device source for readability before each `read()`. If no byte
arrives for `<ms>` the source is *stalled*:

- with `--reconnect` (`tcp://` only): the socket is closed and reopened,
  following `--reconnect-delay-ms` and `--reconnect-attempts`; a stall counts as
  a failed read, so `n` consecutive stalls exhaust `--reconnect-attempts n`;
- without `--reconnect`, and for serial/device sources (which have no reconnect):
  the source ends like EOF.

The `summary:` line gains `rover_stalls=` and `base_stalls=` (only when the flag
is set), and `termination=rover_source_stalled` / `base_source_stalled` when a
stall is what ended the run. The timer measures silence on the descriptor, not
epoch rate, so it is a transport-level dead-peer detector; pick a value above the
longest gap the peer legitimately leaves (a 1 Hz base needs well above 1000).

What it applies to:

- `tcp://` rover/base: yes. `serial://` / device base or rover RTCM: yes.
- `--rover-ubx` from a serial device: yes, the same `poll()` before the device
  `read()`; a stall ends the rover source (no reconnect for serial). A UBX file: no.
- RTCM files: no effect.
- NTRIP (`ntrip://`, `http(s)://`): unchanged. `NTRIPClient` already waits with
  `select()` (1 s per read) and returns "no message" on a quiet but connected caster;
  it reconnects only when the socket is closed, not when the caster is silent, and
  `--stall-timeout-ms` does not change that or reuse its timeout.

Only the source being read can stall: while `gnss live` waits on the base, the
rover socket is not read (and vice versa), so a stall is detected on the source
that is blocked (unless [`--base-outage-fallback spp`](#base-outage-spp-fallback-for-gnss-live-base-outage-fallback-spp)
is given, which reads every live source in its own thread). See [Real-time stress harness](realtime_stress.md) for the
`base_blackhole` / `rover_blackhole` faults that exercise it.

### Base-outage SPP fallback for `gnss live` (`--base-outage-fallback spp`)

Opt-in (default off: the code path, the `.pos` and the `summary:` line are exactly
what they were). Without it a rover epoch that gets no base (no exact,
interpolated or held base) is skipped without any output, and while the base
source is silent `gnss live` blocks on the base read and does not read the rover
at all, so a base outage means no output for its whole length.

With `--base-outage-fallback spp`:

- A rover epoch that would be skipped for lack of a base is solved with the
  library's single-point solver (`SPPProcessor`, configured as the RTK
  processor configures its own SPP seed) and written with status `SPP` (`1` in
  the `.pos`). It is never `FLOAT` or `FIXED`. The RTK processor, its
  filter state, the rover seed and the `--kinematic-post-filter` are not touched
  by these epochs; RTK resumes on the next epoch that has a base, as it does
  today. The rows carry no differential age (`NaN` with `--pos-uncertainty`).
- **File sources**: the existing alignment decides. Every epoch it would skip
  gets an SPP row immediately, so base epochs missing for a span give SPP rows
  in the span and the RTK rows elsewhere are byte-identical to a run without the
  flag.
- **Live sources** (`tcp://`, `ntrip://`, `serial://`/device base or rover,
  including a serial `--rover-ubx`): each live source is read by its own thread
  into a bounded queue (4096 messages or UBX chunks), so a silent base no longer
  stops the rover from being read. The threads only do blocking I/O; decoding,
  epoch assembly, alignment and all solvers stay on the main thread. A rover epoch
  is declared baseless once `max(1 s, --base-hold-seconds + base interval)` of wall
  time has passed since the newest base epoch was taken from the source (the base
  interval is the spacing of the first two base epochs, then tracked within 2x, so
  an outage gap cannot stretch it) and no newer base epoch is queued. A rover epoch
  that the `--base-hold-seconds` window still covers is held (RTK) as before.
  The default timeout is 1.5 s (1 Hz base, 0.5 s hold).
- Without the flag no thread is created. With it, the threads stop on every exit
  path (EOF, `--stall-timeout-ms` termination, `--max-epochs`, an exception):
  blocking waits run in 50 ms `poll()` slices that check an abort flag
  (`RTCMReader::setAbortable()` / `requestAbort()`), and the threads are joined
  before the summary is printed. NTRIP keeps its own bounded `select()` wait.
- The `summary:` line gains `fallback_spp_epochs=` (only with the flag).
  `skipped_rover_epochs` still counts every epoch that got no RTK base; the SPP
  rows are counted in `fallback_spp_epochs` and in `written_solutions`.

Not covered: a base that never delivers a first epoch (the first aligned pair is
still awaited, because the week, the base position and the ephemerides come from
the base stream unless `--nav-rinex` and `--base-ecef` are given); a base source
that has *ended* (EOF, `--stall-timeout-ms` without `--reconnect`, exhausted
`--reconnect-attempts`) still ends the run as before; a silent rover is still
waited for. The fallback is a code-only position: its accuracy is that of
single-point positioning with the broadcast ephemerides. Measured: [Real-time
stress harness](realtime_stress.md#base-outage-spp-fallback).

### Per-epoch uncertainty in `.pos` (`--pos-uncertainty`)

`gnss solve`, `gnss replay` and `gnss live` accept `--pos-uncertainty` (default
off; with it off the `.pos` is byte-identical to earlier releases, and it
requires `--format pos`). It appends seven columns after
`RTKUpdateNISRejected` and declare them in the `% GPS_Week ...` header:

| Column | Meaning |
| --- | --- |
| `sdn(m) sde(m) sdu(m)` | 1-sigma north/east/up standard deviation in metres, from `PositionSolution::positionCovarianceEnu()` |
| `sdne(m) sdeu(m) sdun(m)` | covariance cross terms, RTKLIB signed square root `sign(c)*sqrt(abs(c))` |
| `DiffAge(s)` | rover epoch time minus the base observation time used (`0` for an exactly matched base epoch; for an interpolated or held base epoch, the age of the older sample) |

Unavailable values are written as `nan`. Existing column positions do not move,
so index-based readers keep working; `Solution::loadFromFile` also rebuilds the
ECEF covariance from these columns. The sigma is a model output, not a
certified bound: the default RTK reported covariance can be a constant (see
`--rtk-reported-covariance`). `gnss solve` has no NMEA output, so no GST
sentence is produced.

`gnss uncertainty-eval --pos run.pos --reference-csv reference.csv
[--out-json s.json --out-csv epochs.csv --out-stanford-csv bins.csv]` matches
the epochs to a PPC / UrbanNav `reference.csv` (`--match-tolerance-s`, default
0.25 s) and reports, for FIXED, FLOAT and all epochs: normalized errors
`e_H/sigma_H` (`sigma_H = sqrt(sdn^2+sde^2)`) and `|e_U|/sdu` (median, p95,
p99), exceedance fractions `P(|e| > k*sigma)` for k = 1, 2, 3, 5 beside the
Gaussian expectation, and Stanford-diagram bin counts of error against a
`K*sigma` bound (`--k-h`/`--k-v`, default 5.33) split into nominal, misleading
(error above the bound) and, with `--alert-limit-h/-v`, unavailable (bound above
the alert limit). `K*sigma` is a diagnostic proxy and is **not** a certified
protection level. A PNG is written with `--out-png` only when matplotlib is
installed.

### Float-fix distance in `.pos` (`--pos-fix-diagnostics`)

`gnss live --pos-fix-diagnostics`, `gnss solve --pos-fix-diagnostics` and
`gnss replay --pos-fix-diagnostics` (default off; with it off the `.pos` is
byte-identical to earlier releases, and it requires `--format pos`) append two
columns after every other column (after `DiffAge(s)` when `--pos-uncertainty`
is also on) and declare them in the `% GPS_Week ...` header:

| Column | Meaning |
| --- | --- |
| `FloatFixH(m)` | horizontal distance, in the local ENU frame at the FIXED position, between the epoch's FLOAT position (computed before ambiguity resolution) and the FIXED position |
| `FloatFixU(m)` | absolute vertical difference between the same two positions |

Both are `nan` on every row that is not `FIXED` (FLOAT, SPP and fallback rows).
The float position is recorded read-only next to the solution
(`PositionSolution::rtk_float_position_ecef`); no filter state, position,
status or covariance changes, and every other column is byte-identical to a run
without the flag. Existing column positions do not move, so index-based
readers and `Solution::loadFromFile` keep working.

#### `gnss solve`: only for the RTK epoch's own fixed position

`gnss solve` post-processes the RTK epochs before writing them, so the value is
defined against the **written** row: the distance between the epoch's FLOAT
position and the position written in that row, reported only when the written
status is `FIXED` **and** the written position is still the RTK epoch's own
fixed position. Otherwise both columns are `nan`. `gnss solve` records the
position of every epoch that `RTKProcessor` returned as `FIXED` (read-only,
before any post-processing) and compares the written row with it bit for bit
at write time, so the rule holds for every step below without per-step
bookkeeping. Every non-`FIXED` row is `nan`. Post-processing that changes what
is written for an RTK `FIXED` epoch:

| Step | What it does to a `FIXED` epoch | Written |
| --- | --- | --- |
| `--demote-fixed-status-*`, `--fix-recovery`, the realtime integrity gate | demotes the status to `FLOAT` | `FLOAT` row, `nan` |
| library fixed-quality gate (when enabled by a preset or option) | demotes to `FLOAT`, or can promote an epoch to `FIXED` at another candidate position | `FLOAT` row, or a `FIXED` row that is not the RTK epoch's own fixed position: `nan` |
| kinematic post-filter (on by default), height-step jump | drops the epoch, or replaces it with a propagated `FLOAT` continuity row | no row, or `FLOAT` row with `nan` |
| non-FIX drift, SPP height-step, FLOAT bridge-tail and FIXED bridge-burst guards | remove rows; they never edit a kept row | no row |
| `--safe-availability-fallback` re-insertion of removed epochs | re-inserts the epoch as `SPP` / `PROPAGATED` | `nan` |

The SPP / continuity / propagated fallbacks that replace a rejected non-`FIXED`
solution only ever produce non-`FIXED` rows, so they are `nan` too. A written
`FIXED` row whose position differs from the RTK epoch's own, or whose epoch
carried no float reference, is `nan` as well (the `fix diagnostics:` line
counts them as `nan_modified_fixed_rows` and `nan_no_float_reference_rows`);
neither occurs in the runs below.

Measured (flag on, `--mode kinematic --preset low-cost`, RINEX, develop
`36f05743` plus this option; `gnss solve` prints a `fix diagnostics:` summary
line with these counts):

| Run | RTK `FIXED` epochs | written `FIXED` rows | finite | `FIXED` rows with `nan` | `FIXED` epochs without a row |
| --- | ---: | ---: | ---: | ---: | ---: |
| PPC tokyo run1 | 8,432 | 8,271 | 8,271 | 0 | 161 |
| PPC tokyo run2 | 7,204 | 7,141 | 7,141 | 0 | 63 |
| PPC tokyo run3 | 11,115 | 10,960 | 10,960 | 0 | 155 |
| PPC nagoya run1 | 4,853 | 4,829 | 4,829 | 0 | 24 |
| PPC nagoya run2 | 4,580 | 4,544 | 4,544 | 0 | 36 |
| PPC nagoya run3 | 2,343 | 2,209 | 2,209 | 0 | 134 |

On tokyo run1 all 161 missing epochs are removed by the kinematic post-filter's
height-step drop (with `--no-kinematic-post-filter` all 8,432 are written, each
with finite columns). With the demotion options (`--demote-fixed-status-nis-per-obs 2
--demote-fixed-status-max-ratio 4`) demoted epochs are written as `FLOAT` with
`nan`; in a 5,000-epoch run every written `FIXED` row is still finite. So with these settings the
`nan` on a `FIXED` epoch always means "the row is not `FIXED` any more" and a
written `FIXED` row always has a value. The float position is the epoch's own
float solution, so the same caveats as for `gnss live` apply.

`gnss replay` writes the RTK epoch result directly (no post-processing), so it
uses the `gnss live` definition: finite exactly on `FIXED` rows.

Measured separation with `gnss solve` (descriptive, seen data; nothing was
tuned on it and no threshold is recommended): AUROC of `FloatFixH` alone for a
wrong FIX (FIXED with 3D error > 0.5 m against `reference.csv`, nearest epoch
within 0.25 s) against a correct FIX, `gnss solve --mode kinematic --preset
low-cost`, RINEX inputs, written `FIXED` rows (all finite):

| Run | scored `FIXED` rows | wrong FIX | AUROC `FloatFixH` |
| --- | ---: | ---: | ---: |
| PPC tokyo run1 | 8,271 | 494 | 0.76 |
| PPC tokyo run2 | 7,141 | 67 | 0.88 |
| PPC tokyo run3 | 10,960 | 191 | 0.84 |
| PPC nagoya run1 | 4,829 | 245 | 0.84 |
| PPC nagoya run2 | 4,544 | 64 | 0.98 |
| PPC nagoya run3 | 2,209 | 219 | 0.78 |

Pooled 0.82 (36,674 correct, 1,280 wrong). These are the same six PPC runs the
option was examined on, so the numbers are not a held-out result, and they do
not by themselves transfer to other receivers or environments. The `gnss live`
table above uses RTCM replays and a different fix set, so the two are not
directly comparable.

This is a **diagnostic, not an integrity bound**: it is a number to threshold or
log, not a certified protection level, and it carries no unit of probability. A
large value means the integer solution moved the position far from what the
float filter believed, which is more common on wrong fixes, but a small value
does not make a fix correct.

Measured separation (descriptive, seen data; nothing was tuned on it and no
threshold is recommended): AUROC of `FloatFixH` alone for a wrong FIX (FIXED
with 3D error > 0.5 m against `reference.csv`, nearest epoch within 0.25 s)
against a correct FIX, `gnss live --preset low-cost`, multi-GNSS RTCM replay,
one run each:

| Run | FIXED epochs | wrong FIX | AUROC `FloatFixH` |
| --- | ---: | ---: | ---: |
| UrbanNav Odaiba (Trimble) | 7687 | 78 | 0.81 |
| UrbanNav Odaiba (u-blox) | 2580 | 574 | 0.74 |
| UrbanNav Shinjuku (Trimble) | 13963 | 1687 | 0.73 |
| UrbanNav Shinjuku (u-blox) | 5043 | 276 | 0.75 |
| PPC tokyo run1 | 7884 | 247 | 0.80 |
| PPC tokyo run2 | 7142 | 141 | 0.90 |
| PPC tokyo run3 | 9897 | 274 | 0.84 |
| PPC nagoya run1 | 5252 | 137 | 0.77 |
| PPC nagoya run2 | 4321 | 52 | 0.95 |
| PPC nagoya run3 | 1964 | 72 | 0.78 |

Pooled: UrbanNav Tokyo 0.78, PPC 0.83. The AUROC is a ranking statistic over this data, with no threshold and no
false-alarm rate. Per run the median `FloatFixH` is 2 to 8 mm for a correct FIX
and 1 to 59 cm for a wrong FIX, and the two distributions overlap. Provenance:
develop `e35583e1` plus this option; the four UrbanNav Tokyo runs (Odaiba and
Shinjuku, Trimble and u-blox, Applanix `reference.csv`, base ECEF
`-3961904.9530 3348993.7578 3698211.7553`) and the six PPC runs (Tokyo and
Nagoya runs 1 to 3, PPC `reference.csv`, `base.nav`), all replayed as
multi-GNSS RTCM MSM streams re-encoded from the RINEX observations. These are
the same data the option was examined on, so they are not a held-out result and
they do not by themselves transfer to other receivers or environments.

### Broadcast ephemeris selection among same-toe copies

Merged RINEX navigation files and multi-stream RTCM carry more than one copy of
a broadcast record with the same toe: for Galileo an I/NAV and an F/NAV copy
(their `af0` differs by decimetres, because the clocks reference E5b/E1 and
E5a/E1). `NavigationData::getEphemeris()` takes the first record of the
smallest age, so the copy used depended on the order in which the records
were added (RINEX load, `gnss live` merge, RTCM decode, copy). Each satellite's
records are now kept in a fixed total order: toe, then, for Galileo only, I/NAV
(data-source bit 9) before records without it, then `data_source_code`, `iode`,
`iodc`, `toc`, `af0`, `af1`, `af2` and the remaining fields. A same-toe Galileo
I/NAV record is therefore always selected over its F/NAV copy; a lone F/NAV
record is still used (nothing is filtered), and non-Galileo ties are resolved by
the same order instead of by insertion order. The `.pos` of `gnss solve`,
`gnss live` and `gnss ppp` no longer changes when the navigation records are
permuted. `GNSS_PPP_GAL_INAV` and `setGalileoEphemerisSource` are unchanged.

### Satellite health rule

`gnss spp`, `gnss ppp` and the FGO tools exclude a satellite whose broadcast
ephemeris is flagged unhealthy. They share one rule,
`libgnss::ephemerisHealthy()` (`core/navigation.hpp`): the full health word must
be zero, except that QZSS masks bit 0 (the L6 health bit, which does not affect
L1/L2/L5), as RTKLIB `satexclude()` does. There is no GLONASS special case and no
URA/variance rule. `Ephemeris::health` is `uint16_t` and every reader (RINEX
2/3/4, RTCM 1019/1020/1042/1044/1045/1046, `gnss convert`) stores the whole word.
It used to be `uint8_t`, which wrapped a Galileo E5b health word with only bit 8
set (256) to 0 and made that satellite look healthy.

RTK (`gnss solve`, `gnss live`, `gnss replay`) does **not** apply this rule; it
uses a satellite whatever its health flag. Applying it was checked under a
contract frozen before the run (2026-10-13) and was No-Go. On the UrbanNav Tokyo
merged `base.nav`, the BDS-3 satellites C18, C23 and C24 are flagged health=1
all day (they were in test at the time), and one of them was in the double
differences on 44 to 76 % of the epochs of the four Shinjuku and Odaiba runs.
Excluding them, pooled over the four runs with `--preset low-cost`, moved
FIXED epochs with a 3D error above 0.5 m from 7.47 % to 7.58 % of FIXED in
`gnss solve` and from 6.35 % to 9.50 % in multi-GNSS `gnss live` (limit +1.0
point), so the rule is not applied. On the six PPC runs and HK Medium-Urban-1 no
unhealthy satellite is observed by both receivers and the rule would change
nothing. If your navigation data carries ephemerides that are unhealthy because
they are wrong (rather than flagged during a test campaign), RTK will still use
them.

## Docker

The repo also ships a multi-stage `Dockerfile`. The runtime image installs:

- the `gnss` dispatcher on `PATH`
- native CLI binaries under `/opt/libgnsspp/bin`
- the `libgnsspp` Python package via `PYTHONPATH`

The default entrypoint is `gnss`, so container invocations look like:

```bash
docker run --rm -it -v "$PWD:/workspace" libgnsspp:latest --help
docker run --rm -it -v "$PWD:/workspace" libgnsspp:latest ppp --help
docker run --rm -it -p 8085:8085 -v "$PWD:/workspace" libgnsspp:latest web --host 0.0.0.0 --port 8085 --root /workspace
docker compose up gnss-web
```

## Python

Python bindings expose:

- RINEX header and epoch inspection
- `.pos` loading and solution statistics
- coordinate conversion helpers
- file-based `SPP`, `PPP`, and `RTK` solve helpers
- read-only artifact helpers under `libgnsspp.artifacts`:
  `load_summary()`, `summary_schema()`, `load_pos()`, and `pos_stats()`

## ROS2

The ROS2 playback node publishes `.pos` files as:

- `sensor_msgs/NavSatFix`
- `geometry_msgs/PoseStamped`
- `nav_msgs/Path`
- solution status and satellite-count telemetry

## Local web UI

`gnss web` is a local visualization layer for existing artifacts, including field report summaries and Markdown previews, live, moving-base, PPC, and PPP-product sign-off summaries, artifact bundles, moving-base history charts, commercial receiver side-by-side metrics, next debug commands, and direct links to summary, plot, product, comparison CSV/PNG, match CSV, and provenance files.

It shows:

- Odaiba metrics
- PPC summaries
- live sign-off summaries
- moving-base plots
- commercial receiver comparisons for PPC and moving-base sign-offs
- 2D trajectories
- visibility summaries and a polar visibility view
- `gnss rcv` receiver status

## Experiment Interface

`PPP-AR` experiments now use a minimal shared interface under
`scripts/experiments/ppp_ar/`.

The current experiment surface is:

- `scripts/experiments/ppp_ar/strategies.toml`
- `scripts/experiments/ppp_ar/input.example.toml`
- `scripts/experiments/ppp_ar/suite.example.toml`
- `scripts/experiments/ppp_ar/run_experiments.py`

The current CLAS PPP experiment family includes:

- `osr_float_pipeline`
- `osr_float_strict_pipeline`
- `osr_float_hybrid_pipeline`
- `osr_float_orbit_clock_bias_pipeline`
- `osr_float_orbit_clock_only_pipeline`
- `osr_float_phase_bias_only_value_pipeline`
- `osr_float_compensation_only_value_pipeline`
- `osr_float_clock_reference_time_pipeline`
- `osr_float_observation_reference_time_pipeline`
- `osr_float_sis_continuity_only_pipeline`
- `osr_float_repair_only_pipeline`
- `osr_float_raw_phase_bias_pipeline`
- `osr_float_no_phase_bias_pipeline`
- `osr_float_clock_bound_phase_bias_pipeline`
- `osr_float_clock_bound_atmos_phase_bias_pipeline`
- `osr_float_orbit_or_clock_source_pipeline`
- `osr_float_orbit_and_clock_source_pipeline`
- `osr_float_no_carry_atmos_merge_pipeline`
- `osr_float_network_locked_atmos_merge_pipeline`
- `osr_float_gridded_priority_subtype_merge_pipeline`
- `osr_float_combined_priority_subtype_merge_pipeline`
- `osr_float_message_reset_phase_bias_merge_pipeline`
- `osr_float_subtype5_priority_phase_bias_source_pipeline`
- `osr_float_subtype6_priority_phase_bias_source_pipeline`
- `osr_float_base_plus_network_code_bias_composition_pipeline`
- `osr_float_base_only_code_bias_composition_pipeline`
- `osr_float_same_30s_code_bias_bank_pipeline`
- `osr_float_close_30s_code_bias_bank_pipeline`
- `osr_float_latest_preceding_code_bias_bank_pipeline`
- `osr_float_selected_satellite_base_extend_pipeline`
- `osr_float_all_base_satellite_extend_pipeline`
- `osr_float_base_plus_network_phase_bias_composition_pipeline`
- `osr_float_base_only_phase_bias_composition_pipeline`
- `osr_float_same_30s_phase_bias_bank_pipeline`
- `osr_float_close_30s_phase_bias_bank_pipeline`
- `osr_float_latest_preceding_phase_bias_bank_pipeline`
- `osr_float_residual_only_value_pipeline`
- `osr_float_planar_subtype12_value_pipeline`
- `osr_float_offset_only_subtype12_value_pipeline`
- `osr_float_indexed_only_residual_sampling_pipeline`
- `osr_float_mean_only_residual_sampling_pipeline`
- `osr_float_selected_mask_prune_phase_bias_merge_pipeline`
- `osr_float_polynomial_only_value_pipeline`
- `osr_float_guarded_pipeline`
- `osr_float_balanced_pipeline`
- `osr_float_freshness_pipeline`
- `osr_ar_pipeline`
- `osr_ar_strict_pipeline`
- `osr_ar_hybrid_pipeline`
- `osr_ar_orbit_clock_bias_pipeline`
- `osr_ar_orbit_clock_only_pipeline`
- `osr_ar_phase_bias_only_value_pipeline`
- `osr_ar_compensation_only_value_pipeline`
- `osr_ar_clock_reference_time_pipeline`
- `osr_ar_observation_reference_time_pipeline`
- `osr_ar_sis_continuity_only_pipeline`
- `osr_ar_repair_only_pipeline`
- `osr_ar_raw_phase_bias_pipeline`
- `osr_ar_no_phase_bias_pipeline`
- `osr_ar_clock_bound_phase_bias_pipeline`
- `osr_ar_clock_bound_atmos_phase_bias_pipeline`
- `osr_ar_orbit_or_clock_source_pipeline`
- `osr_ar_orbit_and_clock_source_pipeline`
- `osr_ar_no_carry_atmos_merge_pipeline`
- `osr_ar_network_locked_atmos_merge_pipeline`
- `osr_ar_gridded_priority_subtype_merge_pipeline`
- `osr_ar_combined_priority_subtype_merge_pipeline`
- `osr_ar_message_reset_phase_bias_merge_pipeline`
- `osr_ar_subtype5_priority_phase_bias_source_pipeline`
- `osr_ar_subtype6_priority_phase_bias_source_pipeline`
- `osr_ar_base_plus_network_code_bias_composition_pipeline`
- `osr_ar_base_only_code_bias_composition_pipeline`
- `osr_ar_same_30s_code_bias_bank_pipeline`
- `osr_ar_close_30s_code_bias_bank_pipeline`
- `osr_ar_latest_preceding_code_bias_bank_pipeline`
- `osr_ar_selected_satellite_base_extend_pipeline`
- `osr_ar_all_base_satellite_extend_pipeline`
- `osr_ar_base_plus_network_phase_bias_composition_pipeline`
- `osr_ar_base_only_phase_bias_composition_pipeline`
- `osr_ar_same_30s_phase_bias_bank_pipeline`
- `osr_ar_close_30s_phase_bias_bank_pipeline`
- `osr_ar_latest_preceding_phase_bias_bank_pipeline`
- `osr_ar_residual_only_value_pipeline`
- `osr_ar_planar_subtype12_value_pipeline`
- `osr_ar_offset_only_subtype12_value_pipeline`
- `osr_ar_indexed_only_residual_sampling_pipeline`
- `osr_ar_mean_only_residual_sampling_pipeline`
- `osr_ar_selected_mask_prune_phase_bias_merge_pipeline`
- `osr_ar_polynomial_only_value_pipeline`
- `osr_ar_guarded_pipeline`
- `osr_ar_balanced_pipeline`
- `osr_ar_freshness_pipeline`

The stable rule is that every strategy arm must share:

- one `obs`
- one `nav`
- one CLAS correction source (`qzss_l6` or pre-expanded `ssr_csv`)
- one `reference_ecef`
- one `max_epochs`
- one output schema

When promotion is under evaluation, the runner can also take a suite config and
compare the same strategy set across multiple CLAS cases before changing the
default solver path.

If `reference_ecef` is omitted for a case, the runner falls back to the RINEX
header `APPROX POSITION XYZ`. That keeps new cases easy to add while still
using one shared metric schema.

The current output schema for comparisons is:

- `wall_time_s`
- `ppp_solution_rate_pct`
- `ppp_fixed_solutions`
- `fallback_solutions`
- `clas_hybrid_fallback_epochs`
- `mean_3d_error_m`
- `median_3d_error_m`
- `p95_3d_error_m`
- `max_3d_error_m`
- `readability_score`
- `extensibility_score`

This keeps the stable core separate from the experiment lane while still making
results directly comparable.

For compact-SSR ingestion experiments, the shared decode boundary also exposes:

- `gnss qzss-l6-info --compact-flush-policy <lag-tolerant-union|orbit-or-clock-only|orbit-and-clock-only>`
- `gnss qzss-l6-info --compact-atmos-merge-policy <stec-coeff-carry|no-carry|network-locked-stec-coeff-carry>`
- `gnss qzss-l6-info --compact-atmos-subtype-merge-policy <union|gridded-priority|combined-priority>`
- `gnss qzss-l6-info --compact-phase-bias-merge-policy <latest-union|message-reset|selected-mask-prune>`
- `gnss qzss-l6-info --compact-phase-bias-source-policy <arrival-order|subtype5-priority|subtype6-priority>`
- `gnss qzss-l6-info --compact-code-bias-composition-policy <direct-values|base-plus-network|base-only-if-present>`
- `gnss qzss-l6-info --compact-code-bias-bank-policy <pending-epoch|same-30s-bank|close-30s-bank|latest-preceding-bank|delayed-15s-bank>`
- `gnss qzss-l6-info --compact-bias-row-materialization <overlap-only|selected-satellite-base-extend|all-base-satellite-extend>`
- `gnss qzss-l6-info --compact-phase-bias-composition-policy <direct-values|base-plus-network|base-only-if-present>`
- `gnss qzss-l6-info --compact-phase-bias-bank-policy <pending-epoch|same-30s-bank|close-30s-bank|latest-preceding-bank>`
- `gnss ppp --receiver-antenna-type <type>`
- `gnss clas-ppp --compact-atmos-merge-policy <stec-coeff-carry|no-carry|network-locked-stec-coeff-carry>`
- `gnss clas-ppp --compact-atmos-subtype-merge-policy <union|gridded-priority|combined-priority>`
- `gnss clas-ppp --compact-phase-bias-merge-policy <latest-union|message-reset|selected-mask-prune>`
- `gnss clas-ppp --compact-phase-bias-source-policy <arrival-order|subtype5-priority|subtype6-priority>`
- `gnss clas-ppp --compact-code-bias-composition-policy <direct-values|base-plus-network|base-only-if-present>`
- `gnss clas-ppp --compact-code-bias-bank-policy <pending-epoch|same-30s-bank|close-30s-bank|latest-preceding-bank|delayed-15s-bank>`
- `gnss clas-ppp --antex <antennas.atx>`
- `gnss clas-ppp --receiver-antenna-type <type>`
- `gnss clas-ppp --clas-atmos-selection <grid-first|grid-guarded|balanced|freshness-first>`
- `gnss clas-ppp --clas-atmos-stale-after-seconds <seconds>`
- `gnss clas-ppp --compact-bias-row-materialization <overlap-only|selected-satellite-base-extend|all-base-satellite-extend>`
- `gnss clas-ppp --compact-phase-bias-composition-policy <direct-values|base-plus-network|base-only-if-present>`
- `gnss clas-ppp --compact-phase-bias-bank-policy <pending-epoch|same-30s-bank|close-30s-bank|latest-preceding-bank>`

When a suite config is used, the output tree is partitioned by case:

- `output/.../<case_key>/compact_ssr.csv`
- `output/.../<case_key>/expanded_ssr.csv`
- `output/.../<case_key>/<strategy>/<strategy>.pos`
- `output/.../<case_key>/<strategy>/<strategy>_summary.json`

## Moving-base

`gnss solve`, `gnss replay`, and `gnss live` accept `--mode moving-base`.

`gnss moving-base-prepare` extracts rover/base UBX streams plus a per-epoch reference CSV from a ROS2 moving-base bag or Zenodo zip. It can also export the rover receiver `NAV-PVT` solution as a normalized commercial receiver CSV. `gnss moving-base-signoff` is the validation entrypoint for replay/live runs against those references. `gnss scorpion-moving-base-signoff` wraps the public SCORPION bag flow by chaining prepare, BRDC nav fetch, replay validation, and receiver side-by-side matching. Together they cover:

- ROS2 bag or zip ingestion with u-blox `NAV-PVT` / `RXM-RAWX` / `NAV-RELPOSNED`
- replay inputs via `--rover-ubx` and `--base-ubx`
- public receiver side-by-side summaries via `--commercial-csv` / `--commercial-pos`
- fix rate
- baseline error percentiles
- heading error percentiles
- realtime factor
- decoder error counts
