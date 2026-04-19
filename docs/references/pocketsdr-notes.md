# PocketSDR Notes

This page tracks what is worth borrowing from `PocketSDR` as a future
integration reference, without turning libgnss++ into an SDR application.

Primary upstream:

- [tomojitakasu/PocketSDR](https://github.com/tomojitakasu/PocketSDR)
- [PocketSDR README](https://github.com/tomojitakasu/PocketSDR/blob/main/README.md)

## Scope

`PocketSDR` matters here because it is a strong public reference for:

- GNSS SDR frontend handling,
- IF capture and replay workflows,
- `convbin` support for PocketSDR logs,
- `L6D` / `L6E` signal handling from a real SDR ingest path,
- sample IF data and replay-oriented command flow.

For libgnss++, the useful parts are:

- a future offline or recorded-input path feeding `gnss convert`, `gnss live`,
  or sign-off commands,
- possible `PocketSDR -> RINEX / nav / L6` interoperability,
- a cleaner story for testing raw `QZSS L6` and other GNSS signals from SDR
  captures,
- replay-oriented validation around real RF/IF datasets.

The GUI and SDR receiver application itself are **not** the target.

## What PocketSDR appears to cover

Based on the public repository README:

- RF frontend devices with `2/4/8` channels,
- IF capture utilities such as `pocket_dump`,
- SDR utilities/APs such as acquisition, tracking, and snapshot positioning,
- `convbin` support for PocketSDR outputs,
- broad signal coverage including `QZSS L6D` and `L6E`,
- sample IF data and replay-friendly utilities.

## Why this matters to libgnss++

PocketSDR is useful as a **frontend and replay reference**, especially for the
part of libgnss++ that already cares about:

- `QZSS L6`,
- live/replay sign-off,
- raw decoder coverage,
- conversion pipelines,
- web-visible operational summaries.

The most relevant questions for libgnss++ are:

- should we support a first-class `PocketSDR` import or conversion path,
- should recorded IF / SDR-derived logs become part of replay validation,
- can `PocketSDR` become the cleanest way to widen real-data `L6D/L6E`
  coverage without implementing our own SDR stack,
- which PocketSDR artifacts are better treated as external preprocessing
  rather than native libgnss++ responsibilities.

## Current libgnss++ position

libgnss++ already has:

- `UBX / RTCM / NMEA / NovAtel / SBF / SBP / Trimble / SkyTraq / QZSS L6`
  inspection and conversion paths,
- `QZSS L6` direct decode for several Compact SSR subtypes,
- live/replay sign-off commands,
- browser-visible local web summaries,
- PPC and Odaiba benchmark/sign-off flows.

What it does **not** yet have as first-class features:

- a `PocketSDR`-specific importer,
- an IF replay path,
- `PocketSDR convbin` interoperability helpers,
- a packaged SDR-oriented replay/sign-off workflow.

## Immediate work items derived from this note

1. Decide whether `PocketSDR` should be treated as:
   - an external preprocessing tool only, or
   - a supported conversion/replay input for libgnss++.
2. Evaluate whether a small `PocketSDR -> RINEX/QZSS L6` interoperability
   helper is worth a dedicated command or doc recipe.
3. Decide whether future `QZSS L6` sign-off should include a recorded
   PocketSDR-derived dataset.
4. Keep PocketSDR as a future operations/reference lane, not as a demand to
   embed SDR tracking inside libgnss++.

## Exit condition for this note

This page is considered "done enough" when:

- the repo has a clear decision on `PocketSDR` as preprocessing-only versus
  supported interoperability,
- and any planned replay or conversion work is captured as a small PR family
  rather than a broad SDR rewrite.
