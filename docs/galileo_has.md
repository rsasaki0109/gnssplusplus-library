# Galileo HAS Support

Status of Galileo High Accuracy Service (HAS) support in libgnss++.

| Capability | Status |
|---|---|
| Float PPP with HAS corrections from the HAS Internet Data Distribution (IDD, RTCM 3 SSR) | **Supported** (`gnss_ppp --ssr-rtcm <idd.rtc> --ssr-rtcm-profile has-idd`) |
| Static float PPP validated against a known coordinate | **Validated** on the public OBE4 / 2023-08-17 IDD sample, lane `gnss reproduce has-idd-ppp` |
| Kinematic float PPP | **Validated** on the same sample (white-noise position, no motion prior): 0.12 m horizontal / +0.06 m up after one hour (hour RMS 0.09 m / 0.10 m), gated by the lane |
| HAS signal-in-space (Galileo E6-B C/NAV page) decoder | **Supported** (`gnss_ppp --has-pages <log>`, `gnss has-info`): u-blox RXM-SFRBX, Septentrio SBF GALRawCNAV and cssrlib page text; CRC-24Q, Reed-Solomon (HPVRS) page recovery, MT1 mask / orbit / clock full-set / clock subset / code bias / phase bias decoding |
| SIS decoder parity with cssrlib | **Exact**: every decoded orbit, clock and code-bias value of three public recordings (1 x 9 min u-blox X20, 2 x 1 h Septentrio mosaic-X5; 95 k values) equals cssrlib's `cssr_has` decoder |
| Static float PPP with SIS corrections | **Validated, indicative** on the Kamakura 2025-02-15 hour (Japan, outside the HAS service area), lane `gnss reproduce has-sis-ppp`: 0.07 m horizontal / +0.11 m up after one hour |
| HAS phase biases and PPP-AR | **Not available**: phase biases are decoded (`gnss has-info`) but not applied; the HAS Service Level 1 streams used here carry none, so ambiguities stay float |

## Using HAS IDD corrections

The HAS IDD service distributes the HAS corrections as an RTCM 3 SSR stream
(GPS 1060 / 1059, Galileo 1243 / 1242) together with the GPS 1019 and Galileo
1046 broadcast ephemerides the corrections refer to.

```bash
gnss_ppp --obs OBE42023229c.obs --nav OBE42023229c.nav \
  --ssr-rtcm idd2023229c.rtc --ssr-rtcm-profile has-idd \
  --static --out has_static.pos
```

`--ssr-rtcm-profile has-idd` switches the RTCM SSR ingestion to RTCM 10403.3
semantics, which the `legacy` default (kept byte-identical for existing users)
does not apply:

- **IODE-matched broadcast orbits.** Every correction sample carries the SSR
  IODE / IODnav and PPP evaluates exactly that broadcast record. A satellite
  whose referenced ephemeris is missing is skipped instead of being corrected
  against a different record.
- **Ephemerides from the stream.** GPS 1019 and Galileo 1045 / 1046 messages
  in the SSR file are merged into the navigation data. A RINEX navigation file
  converted from the same stream usually keeps only the last record per
  satellite; on the OBE4 sample, for example, the HAS corrections for G11 and
  G12 reference IODEs that are not in the RINEX file.
- **Galileo I/NAV only.** HAS clocks refer to the I/NAV clock, so F/NAV
  records with the same IODnav are never selected
  (`NavigationData::setGalileoEphemerisSource(INavOnly)`).
- **Updates held until the next one.** Each orbit / clock update is applied
  with its rates and clock polynomial from its epoch until the next update of
  the same satellite (at most 90 s), and never interpolated across an IODE
  change.
- **RTCM code biases.** Bias signal IDs follow the RTCM SSR tables (GPS 0 =
  L1 C/A, 10 = L2 P, 8 = L2C(L); Galileo 2 = E1-C, 6 = E5a-Q, 9 = E5b-Q,
  16 = E6-C) and are added to the pseudorange. HAS code-bias messages carry
  a fixed, stale epoch, so the latest bias set per satellite is held by
  stream order. The HAS biases replace broadcast TGD / BGD. GPS L2 biases
  follow the tracked RINEX code, as on the SIS path: C2W / C2P / C2Y use the
  HAS L2 P bias, C2L / C2S / C2X the L2 CL bias.
- **No broadcast fallback.** HAS covers GPS and Galileo only; a satellite
  without a HAS orbit / clock sample at the epoch (other constellations,
  satellites outside the HAS mask such as E18 / E33 on the OBE4 sample, or a
  correction older than 90 s) is excluded instead of being processed on its
  broadcast orbit.

Orbit corrections are applied with the RTCM convention
(`x = x_brdc - R_rac->ecef * dRAC`, clock `dt = dt_brdc + dC / c`) and refer
to the antenna phase centre of the broadcast orbit, so no satellite antenna
offset is applied.

## Using HAS signal-in-space pages

HAS corrections are broadcast on the Galileo E6-B signal as 448-bit HAS pages
inside the C/NAV pages. `gnss_ppp --has-pages` decodes a receiver log of those
pages and feeds the corrections to float PPP with the same conventions as the
`has-idd` profile (IODE-matched broadcast orbits, Galileo I/NAV only, held
updates, code biases instead of TGD / BGD):

```bash
gnss_ppp --obs 046r_rnx.obs --nav BRDC00WRD_S_20250460000_01D_MN.rnx \
  --has-pages 046r_gale6.txt --static --out has_sis_static.pos
```

| Page log | `--has-pages-format` | Notes |
|---|---|---|
| u-blox UBX (`.ubx`) | `ubx` | RXM-SFRBX with gnssId 2 / sigId 8 (E6-B, 16 words), e.g. X20 / F9 E6 firmware; page time = latest RXM-RAWX epoch |
| Septentrio SBF (`.sbf`, `*.yy_`) | `sbf` | GALRawCNAV (block 4024); pages the receiver flags as CRC-failed are dropped |
| cssrlib page text | `cssrlib` (default for other extensions) | `wn tow prn type len hex` lines as in [cssrlib-data](https://github.com/hirokawa/cssrlib-data) `*_gale6.txt` |

`gnss has-info --input <log>` prints the page / message statistics, the masks
(satellites and signals), the flag patterns and validity intervals, and dumps
`--messages-csv`, `--corrections-csv` (every decoded orbit / clock / bias
value with its HAS sign) and `--updates-csv` (the held per-satellite updates
PPP uses).

Decoding follows the HAS SIS ICD Issue 1.0:

- **Pages.** CRC-24Q over the 462 reserved + HAS page bits; dummy pages
  (`0xAF3BC3`) are skipped. HASS 0 (test) and 1 (operational) pages are used,
  HASS 3 ("don't use") discards every received message and truncates the held
  corrections at that time.
- **High Parity Vertical Reed-Solomon.** Pages are collected per message ID;
  any MS distinct page IDs recover the message by inverting the k x k
  sub-matrix of the RS(255,32) generator matrix over GF(256) (primitive
  polynomial 0x11D). The generator matrix of ICD Annex B is derived from the
  generator polynomial (checked against the Annex B file and the Annex C
  decoding example in the unit tests). Later pages of a decoded message are
  checked by re-encoding, so a reused message ID starts a new collection.
- **MT1.** Mask, orbit, clock full-set, clock subset, code-bias and phase-bias
  blocks, with Mask ID / IOD Set ID caches: a clock-only message is paired with
  the orbit block of the same Mask ID and IOD Set ID. DCC "not available" and
  "do not use" values end the satellite's held correction.
- **Time and validity.** TOH is resolved to GST (= GPST seconds of week) from
  the page reception time (ICD Eq. 28 / 29). Each orbit + clock pair becomes a
  held update from the later of the two reference times until the next update
  of the satellite, at most until the end of the shorter validity interval
  (the recorded streams use 300 s for orbits and biases, 60 s for clocks).
- **Conventions.** HAS orbit corrections are added to the broadcast position
  (ICD Eq. 22), so they are stored negated in the RTCM-convention SSR
  container; the clock correction is DCC x DCM added to the broadcast clock
  (Eq. 23); code biases are added to the pseudoranges (Eq. 25) and are mapped
  from the HAS signal index (ICD Table 20) to the RTCM SSR signal IDs. GPS L2
  biases follow the tracked RINEX code: C2W / C2P / C2Y use the HAS L2 P bias,
  C2L / C2S / C2X the L2 CL bias.
- **Coverage.** HAS corrects GPS and Galileo only; with `--has-pages` any
  satellite without a valid HAS orbit / clock at the epoch (other
  constellations, expired validity, missing IODref ephemeris) is excluded
  instead of falling back to its broadcast orbit.

## Validation (HAS SIS, Kamakura, 2025)

Data: the public HAS SIS samples of
[hirokawa/cssrlib-data](https://github.com/hirokawa/cssrlib-data)
(`data/doy2025-046`: 2025-02-15 17:00-18:00 GPST, `data/doy2025-233`:
2025-08-21 07:00-08:00 GPST; Septentrio mosaic-X5 with a JAVRINGANT_DM, E6-B
pages `*_gale6.txt`, RINEX observations) and the IGS merged broadcast
navigation files of those days (`BRDC00WRD_S_2025{046,233}0000_01D_MN.rnx`
from `https://igs.bkg.bund.de/root_ftp/IGS/BRDC/2025/`). The reference
coordinate is the cssrlib sample value `(-3962108.6836, 3381309.5672,
3668678.6720)` m. Kamakura is **outside the HAS service area** (the HAS SDD
excludes 60S-60N / 90E-180E), so these numbers are indicative only.

```bash
python3 apps/gnss.py reproduce has-sis-ppp --has-sis-data-root /data/cssrlib-data/data --check
```

Decoder parity with cssrlib (`cssr_has`, hirokawa/cssrlib main, 2026-09):
every message of the three recordings was decoded by both, and all values
agree exactly.

| Recording | Pages (dummy) | MT1 messages | Orbit / clock / code-bias values compared | Max difference |
|---|---:|---:|---:|---:|
| u-blox X20, Boulder CO, 2025-07-08 19:34-19:43 GPST (rtklibexplorer/GNSS_IMU `drive_0708`) | 3214 (1772) | 67 (66 usable; the first clock message precedes any mask) | 2288 / 2564 / 1949 | 0 |
| mosaic-X5, Kamakura, 2025-08-21 07h | 34026 (14151) | 432 | 14896 / 18620 / 12721 | 0 |
| mosaic-X5, Kamakura, 2025-02-15 17h | 33451 (17839) | 432 | 13596 / 16995 / 11669 | 0 |

The HAS-corrected satellite positions and clocks at 2025-08-21 07:30:00 also
agree with cssrlib's `satposs()` to 0.1 mm in clock and to about 2 cm in position
(transmission-time differences). The MRTKLIB mosaic-G5 SBF sample
(`tests/data/has/has_testdata.tar.gz`, Tokyo 2026-06-10, 15 min) decodes to
109 MT1 messages (3 receiver-flagged CRC failures); it carries no RINEX
observations, so it is used for decoding only.

Static and kinematic float PPP (`--has-pages`, IGS BRDC navigation, default
15 degree elevation mask; 2026-10-01, after the solid-earth-tide frame fix
described below and the one-measurement-update-per-epoch fix of the static
filter), H / U error at the given time after the first epoch; convergence as
in the IDD table below:

| Run | 10 min | 20 min | 30 min | 60 min | Converged H < 0.20 m | Converged \|U\| < 0.40 m |
|---|---|---|---|---|---:|---:|
| 2025-02-15 17h, libgnss++ SIS, static | 0.473 / +0.257 m | 0.102 / +0.075 m | **0.031 / +0.126 m** | **0.184 / -0.123 m** | 58.7 min | **4.0 min** |
| 2025-02-15 17h, libgnss++ SIS, kinematic | 0.521 / +0.290 m | 0.117 / +0.123 m | 0.052 / +0.250 m | 0.184 / -0.256 m | 59.4 min | 5.2 min |
| 2025-02-15 17h, cssrlib SIS, static (10 deg, E29 excluded, igs20.atx) | 0.531 / +0.450 m | 0.220 / -0.033 m | 0.077 / +0.193 m | 0.153 / -0.065 m | 48.4 min | 45.1 min |
| 2025-02-15 17h, libgnss++ broadcast only, GPS + Galileo | 0.454 / -0.145 m | 0.122 / +0.214 m | 0.192 / +0.086 m | 0.319 / +0.123 m | never | 9.3 min |
| 2025-08-21 07h, libgnss++ SIS, static | 0.849 / -0.317 m | 0.641 / +0.014 m | 0.668 / -0.066 m | 0.520 / -0.400 m | never | never |
| 2025-08-21 07h, libgnss++ SIS, kinematic | 0.404 / +0.314 m | 0.368 / +0.504 m | 0.385 / +0.322 m | 0.372 / -0.540 m | never | never |
| 2025-08-21 07h, cssrlib SIS, static (10 deg, L1 C/A + L2 CL) | 0.303 / -0.663 m | 0.112 / -0.501 m | 0.079 / -0.186 m | 0.079 / +0.212 m | 18.1 min | 22.2 min |
| 2025-08-21 07h, cssrlib SIS, static (15 deg, L1 C/A + L2 W) | 0.381 / -1.214 m | 0.117 / -0.629 m | 0.139 / -0.607 m | 0.084 / +0.563 m | 18.4 min | never |
| 2025-08-21 07h, libgnss++ JPL GDGPS RTCM SSR, GPS + Galileo | 1.050 / +0.886 m | 0.719 / -0.007 m | 0.640 / +0.092 m | 0.524 / -0.113 m | never | 17.8 min |
| 2025-08-21 07h, libgnss++ broadcast only, GPS + Galileo | 0.874 / +0.774 m | 0.710 / +0.069 m | 0.699 / +0.067 m | 0.651 / -0.246 m | never | 17.8 min |
| For reference: OBE4 (Germany) HAS IDD, static (table below) | 0.158 / -0.081 m | 0.130 / -0.029 m | 0.027 / +0.028 m | 0.174 / +0.069 m | 6.4 min | 4.7 min |

The lane gates the 2025-02-15 static run (H <= 0.20 m and |U| <= 0.40 m at
30 and 60 min) and the decoder (all 432 MT1 messages of each hour, no CRC
failure), and reports the other rows. The JPL and broadcast-only rows use the
observation file limited to GPS and Galileo.

**Static filter: one measurement update per epoch (2026-10-01).** The static
rows above used to re-apply each epoch's rows eight times with the geometry
frozen at the prior position (see the
[igs-final-ppp lane](reproduce.md#igs-final-ppp-precise-product-ppp-2026-10-01)).
With one update per epoch the static runs follow the kinematic ones more
closely: the 2025-02-15 SIS static run ends at 0.184 / -0.123 m instead of
0.010 / +0.041 m (the kinematic run ends at 0.184 / -0.256 m; H at 60 min is
still inside the 0.20 m gate), and on 2025-08-21 every static source ends
higher (SIS -0.732 -> -0.400 m, JPL -0.809 -> -0.113 m, broadcast -0.655 ->
-0.246 m) and 0.06-0.22 m worse horizontally.

**2025-08-21 vertical offset: solid-earth tide.** Before the fix libgnss++
ended this hour about 1 m low whatever the correction source (HAS SIS static
-1.165 m, kinematic -0.959 m, JPL GDGPS -1.235 m, broadcast only -1.081 m at
60 min), while cssrlib ends it at +0.21 / +0.56 m. The main cause was the IERS
2010 (Dehant) solid-earth tide, the default `--use-iers-solid-tide` model: it
was fed the SOFA Sun and Moon in the ICRS, while the routine needs them in the
station's Earth-fixed frame, so the Earth's rotation dropped out of the
station-body geometry. The modelled radial tide was a nearly constant +0.28 m
over that hour instead of -0.10 to -0.14 m (cssrlib `tidedisp` /
`tidedispIERS2010`, ports of RTKLIB `tide_solid()`, and the libgnss++ Step-1
model `--no-iers-solid-tide` agree on the latter), a 0.4 m
error in up; on 2025-02-15 17h the true tide was +0.10 to +0.14 m and the
error smaller. The Sun and Moon are now rotated to ITRS (`icrsToItrs`, IAU
2006/2000A, with EOP when `--eop-c04` is given). Every 2025-08-21 row moves up
by 0.42-0.44 m at 60 min, and the 2025-02-15 static run ends at +0.04 m. The
CLAS lane (own tide model) and the per-frequency MADOCA profiles (Step-1 model)
do not use this path; the static ionosphere-free MADOCA `ppp` profile does.

Before the static one-update fix the hour still ended 0.5-0.7 m low; after it
the static runs end 0.11-0.40 m low. What was checked (before that fix):

- *Reference coordinate.* RTKLIB demo5 b34k PPP-static with IGS final orbits
  and clocks (GPS only, `igs20.atx`) ends the hour at -0.21 m (10 degree mask)
  and -0.28 m (15 degree), the 2025-02-15 hour at -0.02 / -0.20 m.
- *Troposphere.* The zenith delay RTKLIB estimates with the position held at
  the reference is 2.59 m (2025-02-15: 2.41 m). The libgnss++ a priori
  (UNB3m-style climatology) is 2.395 m on 2025-08-21, 0.2 m short for this
  humid afternoon; the filter estimate reaches 2.67 m after 45 min. With the
  zenith delay held at 2.59 m the static run still ended at -0.75 m (before the
  tide fix), so the troposphere is not the main driver.
- *Model or estimator.* A batch least-squares float solution on the libgnss++
  corrected observables (clocks per epoch and system, one ambiguity per arc)
  gives the same up offset as the filter (-1.05 to -1.19 m before the tide
  fix), so the remaining offset is in the observables of that hour, not in the
  filter. Code residuals at the reference differ by satellite by up to
  +/-0.4 m (G28 -0.9 m) with 11-13 satellites in view; GPS-only and
  Galileo-only runs are both about 1 m low (before the tide fix).
- *Satellite set.* The IGS merged BRDC file used by the lane lacks the G12
  ephemeris with IODE 8 (uploaded at 07:59:44) that the HAS corrections refer
  to, so libgnss++ excludes G12 (51-64 degrees elevation) for the whole hour;
  cssrlib takes it from the receiver's RINEX navigation file. A navigation file
  that carries it (BRDC00IGS_R) did not help (-1.67 m before the tide fix).
- *Other terms.* Receiver ANTEX (`--antex igs20.atx`, -0.02 m), a 10 degree
  mask (-1.09 m before and -0.65 m after the fix) and a Galileo clock offset
  state (horizontal 0.52 -> 0.13 m, up worse) do not explain it;
  the HAS code biases of each satellite nearly cancel in the
  ionosphere-free combination (flipping their sign moves the result by 1 cm).

Smoke test on the u-blox X20 drive (rtklibexplorer/GNSS_IMU `drive_0708`,
Boulder CO, inside the service area, 9 min, RXM-RAWX converted to RINEX,
IGS BRDC navigation): kinematic float PPP runs on all 549 1-Hz epochs with
the 2850 HAS updates decoded from the same UBX file. The RTK track shipped
with the data set is relative to a base of unknown absolute coordinate
(constant offset E +6.0 / N +4.7 / U -23.0 m for both HAS and broadcast PPP),
so it cannot validate absolute accuracy; after removing that offset the HAS
kinematic track scatters 0.46 m (H RMS) / 1.25 m (U RMS) after 5 min against
0.31 m / 1.33 m broadcast only, as expected for an unconverged 9-minute run.

## Validation (OBE4, 2023-08-17 02:00-03:00 GPST)

Data: the public HAS IDD sample in
[hirokawa/cssrlib-data](https://github.com/hirokawa/cssrlib-data)
`data/doy2023-229/` (`OBE42023229c.obs`, `OBE42023229c.nav`,
`idd2023229c.rtc`). Its license is not stated, so the files are not
redistributed with libgnss++; download them and point
`--has-data-root` / `GNSSPP_HAS_DATA_ROOT` at that directory. The reference
coordinate is the cssrlib sample value
`(4186704.2262, 834903.7677, 4723664.9337)` m (OBE4, Septentrio AsteRx4 with
a SEPCHOKE_B3E6 antenna).

```bash
python3 apps/gnss.py reproduce has-idd-ppp --has-data-root /data/cssrlib-data/data/doy2023-229 --check
```

Lane result (2026-10-01, after GPS L2 code biases started following the
tracked code and satellites without HAS corrections were excluded, MSVC
Release, elapsed time from the first epoch 01:59:12 GPST; the historical rows
keep the values of their time):

| Run | H / U at 10 min | 20 min | 30 min | 60 min | Converged H < 0.20 m | Converged \|U\| < 0.40 m |
|---|---|---|---|---|---:|---:|
| `has-idd`, static | 0.158 / -0.081 m | 0.130 / -0.029 m | 0.027 / +0.028 m | **0.174 / +0.069 m** | **6.4 min** | **4.7 min** |
| `has-idd`, static, L2 bias by coarse signal, broadcast fallback | 0.056 / -0.147 m | 0.067 / -0.170 m | 0.092 / +0.049 m | 0.014 / +0.050 m | 0.8 min | 5.8 min |
| `has-idd`, static, before the one-update fix | 0.018 / -0.503 m | 0.057 / -0.432 m | 0.072 / -0.090 m | 0.097 / -0.136 m | 4.2 min | 21.5 min |
| `legacy` conversion of the same stream, static | 0.517 / -0.120 m | 0.207 / +0.072 m | 0.273 / +0.551 m | 0.105 / +0.284 m | 57.8 min | 58.7 min |
| `has-idd`, kinematic | 0.185 / +0.064 m | 0.143 / +0.184 m | 0.116 / +0.393 m | **0.147 / +0.612 m** | 58.7 min | never |
| `has-idd`, kinematic, L2 bias by coarse signal, broadcast fallback | 0.033 / -0.201 m | 0.064 / -0.128 m | 0.057 / +0.018 m | 0.114 / +0.077 m | 59.6 min | 6.1 min |
| `has-idd`, kinematic, before the residual screening | 0.035 / -0.195 m | 0.064 / -0.127 m | 0.136 / +0.058 m | 0.018 / -0.390 m | 45.0 min | never (-0.44 m at the last epoch) |
| `has-idd`, kinematic, before the one-update fix | 3.03 / -5.95 m | 2.47 / -5.13 m | 2.20 / -5.71 m | 1.87 / -5.61 m | never | never |

The lane gates the static run (H at 60 min <= 0.20 m, |U| at 60 min <= 0.40 m,
horizontal convergence <= 10 min, vertical convergence <= 30 min) and the
kinematic run (H <= 0.30 m and |U| <= 0.60 m at 30 and 60 min), and reports
the other rows. With the kinematic post-fit residual screening (see
[Broadcast kinematic PPP](broadcast_kinematic_ppp.md)) the kinematic run
stayed within 0.21 m horizontally and 0.24 m vertically after the first 10
minutes (RMS 0.086 m / 0.099 m) while E18 and E33, which have no HAS
corrections in this stream, were still processed on their broadcast orbits.
Excluding them (and applying the HAS L2 P bias to C2W) leaves 12 satellites
for most of the hour: the static run improves vertically (RMS after 10 min
U 0.117 m against 0.249 m, H 0.118 m against 0.125 m), while the white-noise
kinematic run drifts up to +0.6..+0.8 m after 40 min (RMS after 10 min H
0.176 m, U 0.503 m) and its 60-minute |U| gate (0.60 m) reads 0.612 m.

Comparison with [cssrlib](https://github.com/hirokawa/cssrlib) (main,
`samples/test_ppprtcm.py` case 1, which processes the same files from
02:00:00 GPST with `igs20.atx`). libgnss++ was run on the observation file cut
to start at 02:00:00 as well (2026-09-29, before the solid-earth-tide frame
fix, which moves the lane rows above by at most 2 cm):

| Solver (static float PPP, from 02:00:00) | 10 min | 20 min | 30 min | 60 min | Converged H < 0.20 m | Converged \|U\| < 0.40 m |
|---|---|---|---|---|---:|---:|
| libgnss++ `has-idd` | 0.036 / -0.460 m | 0.076 / -0.407 m | 0.081 / -0.061 m | 0.105 / -0.137 m | 3.8 min | 20.3 min |
| libgnss++ `has-idd` + `--antex igs20.atx` | 0.037 / -0.568 m | 0.077 / -0.515 m | 0.081 / -0.169 m | 0.105 / -0.244 m | 3.8 min | 27.9 min |
| cssrlib (static) | 0.214 / -0.487 m | 0.162 / -0.215 m | 0.140 / -0.100 m | 0.152 / -0.300 m | 11.0 min | 12.9 min |
| libgnss++ `legacy` (develop behaviour) | 0.306 / -0.606 m | 0.225 / -0.248 m | 0.262 / +0.253 m | 0.148 / +0.060 m | 55.1 min | 15.4 min |

Both solvers end the hour at 0.10-0.15 m horizontal and 0.14-0.30 m vertical
error. cssrlib's kinematic mode in that sample uses a 0.01 m/sqrt(s) position
random walk and gives the same numbers as its static mode. The libgnss++
kinematic filter re-seeds the position from SPP every epoch (white-noise
position, no motion prior); from 02:00:00 it gives 0.051 / -0.183, 0.072 /
-0.143, 0.064 / +0.009 and 0.099 / +0.114 m at 10 / 20 / 30 / 60 min (with
`--antex igs20.atx`: 0.052 / -0.290, 0.072 / -0.251, 0.063 / -0.098 and
0.099 / +0.007 m).

H is the horizontal error and U the up error at the given time after the
first epoch; convergence is the first time after which H stays below 0.20 m
(resp. |U| below 0.40 m) until the end of the hour. No receiver ANTEX is
applied; with `--antex igs20.atx` the up error shifts by about -0.1 m.

## Known limitations

- **SIS: service area.** Both public SIS hours are from Japan, outside the HAS
  service area; the in-area X20 recording is 9 minutes of driving with an
  RTK track of unknown absolute datum. There is no in-area static SIS
  validation yet.
- **SIS: navigation data.** The RINEX 4 navigation file shipped with the
  2025-02-15 cssrlib-data hour is not usable by the libgnss++ RINEX reader
  (every satellite is rejected with ~100 km residuals, also without HAS), and
  the RINEX navigation files of both hours hold only the ephemerides received
  during the hour, so earlier IODrefs cannot be matched. The lane uses the IGS
  merged BRDC files instead.
- **SIS: phase biases** are decoded and dumped but not applied (float PPP).
- **SIS: SBF observations.** SBF input is used for the HAS pages only; the
  observations must be converted to RINEX separately.
- **IDD kinematic geometry.** With satellites outside the HAS mask excluded
  (as on the SIS path), the OBE4 kinematic hour runs on 12 satellites and its
  vertical error drifts to +0.6..+0.8 m in the second half hour.

- **Kinematic PPP (fixed).** Until the one-update fix, `--kinematic` settled
  about 2 m horizontal / 5.6 m vertical off on this hour with HAS, legacy SSR
  and broadcast-only input alike. The PPP filter re-applied the same epoch's
  measurement update up to eight times while the observation geometry stayed
  at the prior (SPP-seeded) position, so every extra pass pushed the position
  again by the innovation it had already absorbed and the troposphere and
  float ambiguities soaked up the difference. Kinematic PPP now commits one
  update per epoch, as RTKLIB / MADOCALIB do, and since 2026-10-01 so do
  static and `--low-dynamics` runs (only the coherent MADOCA static
  ionosphere-free profile keeps the historical passes); at start-up the
  repeated push had moved static solutions tens to hundreds of metres. See
  the [igs-final-ppp lane](reproduce.md#igs-final-ppp-precise-product-ppp-2026-10-01).
  Kinematic PPP also screens its post-fit residuals (w-test, 4 sigma); see
  [Broadcast kinematic PPP](broadcast_kinematic_ppp.md).
- **Galileo inter-system bias.** Galileo shares the GPS receiver clock unless
  `GNSS_PPP_ESTIMATE_ISB=gal` is set (as on the other non-MADOCA PPP paths).
  On this sample Galileo code residuals sit about 2 m below GPS; estimating
  the ISB did not improve the one-hour static result.
- **Satellite antenna frequency dependency.** Corrections are applied at the
  broadcast antenna phase centre without the per-signal satellite PCO / PCV
  differences cssrlib applies (centimetre level).
- **One-hour, one-station validation.** The public IDD sample is a single hour
  at one station; there is no multi-day validation yet.
