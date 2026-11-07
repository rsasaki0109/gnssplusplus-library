# BeiDou PPP-B2b Support

Status of BeiDou PPP-B2b support in libgnss++. PPP-B2b is the precise point
positioning service the BDS-3 GEO satellites (C59-C62) broadcast on B2b-I for
China and the surrounding Asia-Pacific region, which is outside the Galileo HAS
service area (BDS-SIS-ICD-PPP-B2b, Version 1.0, 2020).

| Capability | Status |
|---|---|
| B-CNAV1 / B-CNAV2 / B-CNAV3 and GPS / QZSS CNAV ephemerides (RINEX 4 CNV1-3, CNAV, CNV2) | **Supported**: parsed, kept apart from the LNAV / D1 records, orbit with the ADOT / delta-n0-dot terms; see [RINEX 4 plan](rinex4_plan.md), Phase 5b |
| PPP-B2b message decoder (`io::BdsB2bDecoder`) | **Supported**: cssrlib `wn tow prn type len hex` logs, CRC-24Q, message types 1 (mask), 2 (orbit + URA), 3 (code bias), 4 (clock), the IODP / IODSSR / IOD Corr bookkeeping |
| Decoder parity with cssrlib | **Exact**: every mask, orbit, clock and code-bias entry of C59, C60 and C62 on the two public Kamakura hours (about 1500 orbit, 35000 clock and 6500 code-bias entries per satellite and hour) equals cssrlib's `cssr_bds` decoder |
| Float PPP with B2b corrections | **Supported** (`gnss_ppp --b2b <log>`): GPS (LNAV) and BDS-3 (B-CNAV1) |
| Static float PPP on a known coordinate | **Validated** on the Kamakura hours below (0.03-0.15 m horizontal after one hour) |
| PPP-AR | **Not available**: PPP-B2b carries no phase biases |

## Using PPP-B2b corrections

```bash
gnss_ppp --obs 233h_rnx.obs --nav 233h_rnx.nav --b2b 233h_bdsb2b.txt \
  --antex igs20.atx --elevation-mask 10 --static --out b2b_static.pos
```

- `--nav` must hold **B-CNAV1 records (RINEX 4 `> EPH C.. CNV1`)**: the BDS
  orbit corrections refer to them by IODE. A RINEX 3 file has none and
  `gnss_ppp` stops with an error. A receiver's own RINEX 4 NAV of the same
  hour works; only the satellites whose referenced IODE it holds are
  corrected (9 of the corrected BDS-3 satellites on the 2025-02-15 hour).
- `--b2b-prn <prn>` picks the GEO whose stream is used (default: the first in
  the log). Each GEO broadcasts its own IODP / IODSSR sequence, so the
  streams are never mixed.
- `--antex` is recommended: the B2b BDS orbits and clocks refer to the B3I
  antenna phase centre, and the B1I / B3I combination PPP uses needs the
  difference of its satellite PCO from the B3I one (0.8-1.1 m on BDS-3 MEO in
  igs20.atx), as well as the receiver antenna model.
- Satellites without a B2b correction are not used: B2b covers GPS and
  BDS-3; Galileo, GLONASS, QZSS and BDS-2 are dropped.

What `gnss_ppp --b2b` does:

- **Ephemerides.** BDS uses the B-CNAV1 record whose IODE equals the
  correction's IODN & 0xff (`NavigationData::BeiDouEphemerisSource::BCnav1`);
  GPS uses the LNAV record of that IODE.
- **Orbit.** The radial / along-track / cross-track correction is in the B2b
  frame (radial = position direction, ICD 7.2) and subtracted from the
  broadcast position.
- **Clock.** C0 is subtracted from the broadcast clock (t = t_brdc - C0 / c),
  paired with the orbit by IOD Corr; a clock that has just moved to a new IOD
  Corr keeps serving an orbit on the previous one, as in cssrlib.
- **Holding.** Each orbit / clock pair holds until the next one, at most 96 s
  (the orbit validity of the ICD). An invalid value or a broken IOD Corr pair
  ends it: the satellite is unused until the next valid pair.
- **Signals and biases.** BDS-3 uses B1I / B3I (B3I is the clock reference;
  the B1I code bias of message type 3 is applied) with its own receiver
  clock. GPS uses the observation's L1 / L2 codes; the B2b stream of the
  samples carries no GPS code biases.

## Validation

Public data: [hirokawa/cssrlib-data](https://github.com/hirokawa/cssrlib-data)
(commit `ec1dfa2c`, no stated license, so nothing is redistributed) has two
one-hour static Kamakura recordings (Septentrio mosaic-X5, JAVRINGANT_DM),
each with the B2b messages of C59, C60 and C62 and a RINEX 4 NAV with
B-CNAV1, and the coordinate -3962108.6836, 3381309.5672, 3668678.6720.

| Hour | Horizontal / up after 1 h | Horizontal < 0.2 m from | cssrlib `test_pppbds` (B1C / B2a) | cssrlib with B1I / B3I |
|---|---|---|---|---|
| 2025-02-15 17h (`046r`) | 0.15 m / -0.10 m | 40 min | 0.10 m | 0.14 m |
| 2025-08-21 07h (`233h`) | 0.03 m / -0.09 m | 41 min | 0.03 m | 0.05 m |

`gnss_ppp --b2b <log> --b2b-prn 59 --antex igs20.atx --elevation-mask 10
--static`, 3599 epochs, about 7 s per hour. cssrlib (26734271) runs its
`samples/test_pppbds.py` on the same files (C59). The one-hour results are
within the spread of cssrlib's; cssrlib's uncombined filter converges faster
(horizontal below 0.2 m after about 15-20 minutes).

Correction checks on the 2025-02-15 hour:

- **cssrlib parity.** At the emission times PPP uses, the corrected satellite
  positions equal cssrlib's `satposs` within 0.3 mm and the clocks within
  the update-time difference (centimetres).
- **Against the ESA final MGEX orbits and clocks** (ESA0MGNFIN), after a
  constant per satellite and a common term per epoch: the BDS-3 signal-in-space
  range error varies by 2-8 mm (std), the GPS one by 3-6 cm (G14 12 cm, G20
  18 cm).

## Known limitations

- Float PPP only; no phase biases are broadcast.
- Message types 5 (URA), 6 and 7 (combined) are not decoded; none occurs in
  the samples.
- Logs in the cssrlib text format only (no Septentrio SBF or Unicore
  messages yet).
- The receiver NAV of a short recording may lack the B-CNAV1 issues the
  corrections reference; those BDS satellites are skipped.
