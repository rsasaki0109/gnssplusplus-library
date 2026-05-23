# RINEX 4.02 Plan

This plan scopes the RINEX 4 work before implementation.  The intent for
iter1 is to record the parser state, isolate the RINEX 4 surface area, and add
only a compile-time skeleton.  Functional parsing changes should start in
iter2.

## Reference Sources

- IGS RINEX 4.02 specification:
  https://files.igs.org/pub/data/format/rinex_4.02.pdf
- IGS release note for RINEX 4.02:
  https://igs.org/news/rinex-4-02/
- IGS formats page:
  https://igs.org/formats-and-standards/
- GSI RNXCMP / CompactRINEX note:
  https://terras.gsi.go.jp/ja/crx2rnx.html

## Current Parser State

The active reader/writer lives in:

- `include/libgnss++/io/rinex.hpp`
- `src/io/rinex.cpp`

Current behavior is mostly RINEX 2.x and 3.x:

- `RINEXReader::readHeader()` reads fixed 80-column header lines until
  `END OF HEADER`.
- `parseHeaderLine()` detects the file version with `std::stod(line.substr(0, 9))`.
- File type is inferred from `line[20]`, with observation, navigation,
  meteorological, and clock mapped to `FileType`.
- Observation headers support RINEX 2 `# / TYPES OF OBSERV` and RINEX 3
  `SYS / # / OBS TYPES`.
- The RINEX 3 observation type parser currently reads only the first line for a
  system, up to 13 observation types.  Continuation lines are not accumulated.
- `readObservationEpoch()` dispatches `version < 3.0` to the RINEX 2 epoch
  parser and every `version >= 3.0` to the RINEX 3 epoch parser.
- `parseObservationEpochV3()` assumes fixed epoch fields:
  year at columns 2-5, seconds in `substr(18, 13)`, flag at columns 31-32,
  and satellite count at columns 33-35.
- RINEX 3 satellite observation rows are parsed as one line per satellite, with
  16-character observation fields starting at column 4.
- `readNavigationData()` treats every `version >= 3.0` navigation file as a
  RINEX 3 navigation file.
- Navigation record detection expects the first character of an ephemeris data
  line to be a constellation identifier.
- `supportsBroadcastNavigationSystem()` accepts GPS, GLONASS, Galileo, BeiDou,
  and QZSS for broadcast navigation parsing.  SBAS and NavIC are not accepted.
- `parseNavigationMessage()` handles RINEX 2 GPS-style records and RINEX 3
  GPS/Galileo/BeiDou/QZSS eight-line Kepler records, plus four-line GLONASS
  FDMA records.
- Header-level RINEX 3 `IONOSPHERIC CORR` parsing is limited to GPSA/GPSB.
- `RINEXWriter` can write simple RINEX 3-style headers and broadcast
  navigation records, but it has no RINEX 4 data-record header support.

The important conclusion is that RINEX 4 observation files may look close to
RINEX 3, but RINEX 4 navigation files do not fit the current navigation parser.
The current `version >= 3.0` branch is not a sufficient RINEX 4 compatibility
story.

## RINEX 4.02 Deltas

The RINEX 4 line starts at RINEX 4.00:

- Navigation files use a new data record header line beginning with `>`.
- Navigation record types include `EPH`, `STO`, `EOP`, and `ION`.
- The header line carries a data source and a navigation message type, for
  example `LNAV`, `FDMA`, `FNAV`, `INAV`, `CNAV`, `CNV1`, `CNV2`, `CNV3`,
  `L1NV`, `L1OC`, and `L3OC`.
- System records such as `STO`, `EOP`, and `ION` are first-class data records,
  not just legacy header fields.
- RINEX 4.00 made system-dependent observation code lists required for
  observation files.

The RINEX 4.02 update adds or clarifies:

- Observation epoch time tagging may carry extra second digits up to
  picosecond resolution.
- The observation epoch record can include an optional receiver clock offset
  estimate.
- NavIC L1 `L1NV` navigation messages are included.
- GLONASS L1OC and L3OC CDMA navigation messages are included.
- Navigation message subtypes are present in the navigation data record header.
- ION subtypes are defined for QZSS and NavIC dual ionosphere models:
  QZSS `CNVX WIDE`, QZSS `CNVX JAPN`, NavIC `L1NV KLOB`, and
  NavIC `L1NV NEQN`.

CompactRINEX / CRINEX is separate from the RINEX syntax itself:

- Hatanaka CompactRINEX is a compressed representation for RINEX observation
  files.
- GSI RNXCMP advertises conversion for RINEX 2.xx, 3.xx, and 4.xx observation
  files.
- This repo currently has no native CRINEX reader.  Iter2 should treat CRINEX
  as an input-preparation concern unless a native decompressor is explicitly
  chosen.

## Impact On Current Code

Observation file impact:

- Current RINEX 3 epoch parsing will not safely parse picosecond-width seconds,
  because flag and satellite-count columns are fixed after a 13-character
  seconds field.
- The parser needs token-based epoch parsing after the `>` marker while still
  preserving optional receiver clock offset parsing.
- System observation type continuation lines must be supported before claiming
  robust RINEX 4 observation support.
- Time precision should be stored as `double` for now because `GNSSTime::tow`
  is a `double`; picosecond text may be parsed but not represented losslessly.

Navigation file impact:

- Current RINEX 4 navigation files will begin records with `> EPH ...`,
  `> STO ...`, `> EOP ...`, or `> ION ...`; the current parser will not treat
  those lines as ephemeris starts.
- Record metadata must be parsed before record-body dispatch.
- Existing ephemeris parsing can be reused for selected `EPH` bodies after the
  RINEX 4 header line is consumed, but the selected message type must be kept
  in the resulting object or at least respected for line count and time-system
  interpretation.
- STO/EOP/ION require new data containers or an explicit skip policy.  Silent
  misparse is not acceptable.
- NavIC and GLONASS CDMA navigation messages need either new `Ephemeris`
  fields or a generic navigation-message container.

Writer impact:

- RINEX 4 navigation writing must emit data record header lines.
- Observation writing must preserve the expanded epoch second precision and the
  optional receiver clock offset field.
- Writer support can lag reader support; it should not block initial reader
  acceptance.

## Proposed Skeleton Ownership

The new files introduced in iter1 are:

- `include/libgnss++/io/rinex4.hpp`
- `src/io/rinex4.cpp`

They should remain small and separate until the compatibility boundary is
clear.  The existing `RINEXReader` should remain the production path for
RINEX 2/3.  RINEX 4 additions can later be folded into `RINEXReader` once the
tests prove that no RINEX 2/3 behavior regressed.

## Roadmap

Phase 0, iter1 foundation:

- Add empty RINEX 4 skeleton files and compile them.
- Document current parser assumptions.
- Do not change behavior in `src/io/rinex.cpp`.

Phase 1, reader detection and safe dispatch:

- Add explicit `isRinex4()` checks rather than relying on `version >= 3.0`.
- Add tests with minimal RINEX 4.02 observation and navigation fixtures.
- For RINEX 4 navigation records, parse the `>` data record header and route
  unsupported record types to a deliberate skip path.
- Keep RINEX 2/3 tests unchanged.

Phase 2, observation support:

- Replace fixed-column epoch parsing with token-based parsing for RINEX 4 epoch
  lines.
- Preserve optional receiver clock offset estimate when present.
- Handle expanded second precision text without shifting flag/count parsing.
- Accumulate `SYS / # / OBS TYPES` continuation lines.
- Add CRINEX policy tests that verify `.crx`/`.crx.gz` is rejected with a clear
  message or preprocessed through an external converter.

Phase 3, `EPH` navigation support:

- Parse RINEX 4 `EPH` data record headers.
- Reuse current GPS/Galileo/BeiDou/QZSS/GLONASS FDMA bodies where compatible.
- Add explicit skip/error handling for unsupported `L1NV`, `L1OC`, and `L3OC`
  before implementing them.
- Preserve message type metadata for Galileo `FNAV` vs `INAV`.

Phase 4, system data records:

- Add internal containers for `STO`, `EOP`, and `ION`.
- Start with ION because QZSS/NavIC ION subtypes matter for MADOCA/RINEX 4
  convergence.
- Decide whether these containers live in `NavigationData` or a RINEX-specific
  sidecar.

Phase 5, new navigation messages:

- Implement NavIC L1 `L1NV` parsing.
- Implement GLONASS `L1OC` and `L3OC` parsing.
- Add conformance fixtures from public RINEX 4.02 files before enabling these
  records in production flows.

Phase 6, writer support:

- Emit RINEX 4 navigation data record headers.
- Add writer round-trip tests only after reader behavior is stable.

## Acceptance Gates

- Existing RINEX 2/3 tests continue to pass.
- RINEX 4 observation fixture parses header, epoch time, satellite count, and
  at least GPS/QZSS observation rows.
- RINEX 4 navigation fixture with `> EPH G.. LNAV` parses at parity with the
  equivalent RINEX 3 record.
- Unsupported RINEX 4 records fail or skip deterministically with diagnostics.
- CRINEX input policy is explicit and tested.

## Non-Goals For Iter1

- No RINEX 4 parser behavior changes.
- No native CRINEX decompressor.
- No NavIC or GLONASS CDMA message implementation.
- No writer behavior changes.
