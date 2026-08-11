# QZSS L6, CLAS, and MADOCA

Use this route when the input is a QZSS L6 capture or a correction file. The
small offline smoke below exercises the existing raw-frame decoder without a
receiver, credentials, or network access. The later commands show the
repository's CLAS/PPP and native MADOCA paths with your own local files.

## First output: decode a small L6 capture

The repository already contains the deterministic frame builder used by the
CLI tests. Create five valid 250-byte frames from it:

```bash
mkdir -p output/use_cases/qzss_l6
python3 - <<'PY'
from pathlib import Path
from tests.cli_cases._support import build_qzss_l6_frame

capture = b"".join(
    build_qzss_l6_frame(
        prn=199,
        facility_id=0,
        subframe_start=(index == 0),
        alert=False,
        data_part=b"CLAS-L6-SMOKE",
    )
    for index in range(5)
)
Path("output/use_cases/qzss_l6/smoke_l6.bin").write_bytes(capture)
PY
```

Run the exact repository decoder and retain its text summary and CSV files:

```bash
python3 apps/gnss.py qzss-l6-info \
  --input output/use_cases/qzss_l6/smoke_l6.bin \
  --limit 100 \
  --extract-data-parts output/use_cases/qzss_l6/data_parts.csv \
  --extract-subframes output/use_cases/qzss_l6/subframes.csv \
  | tee output/use_cases/qzss_l6/summary.txt
```

The first line is a `summary:` record with `frames`, `valid`, `subframes`,
`prns`, and related counts. `data_parts.csv` begins with
`frame_index,prn,vendor_id,facility_id,reserved_bits,subframe_start,alert`;
`subframes.csv` begins with
`subframe_index,prn,vendor_id,facility_id,frame_count,alert_frames`. A
non-zero process exit, a missing `summary:` line, or an empty CSV is a failed
smoke. This fixture is a decoder wiring check, not a CLAS correction stream.

For a real capture, add the existing extraction switches as needed:
`--extract-compact-messages`, `--extract-compact-corrections`,
`--extract-service-info`, and `--show-preview`. The full option surface is in
the [interfaces guide](../interfaces.md).

## CLAS PPP path

With a rover observation file, matching navigation file, and a raw L6 capture
that contains the corrections required by the selected epochs, run the
repository's no-credential local path:

```bash
python3 apps/gnss.py clas-ppp \
  --profile clas \
  --obs <rover.obs> \
  --nav <navigation.nav> \
  --qzss-l6 <qzss-l6-capture.bin> \
  --qzss-gps-week 2200 \
  --out output/use_cases/qzss_l6/clas_solution.pos \
  --summary-json output/use_cases/qzss_l6/clas_summary.json
```

The summary JSON exposes the route's measurable artifacts, including
`correction_profile`, `ssr_transport`, `correction_encoding`, `qzss_l6`,
`epochs`, `ppp_float_epochs`, `ppp_fixed_epochs`, `fallback_epochs`,
`ppp_solution_rate_pct`, `atmos_messages`, `atmos_rows`,
`ppp_atmospheric_trop_corrections`, and
`ppp_atmospheric_ionosphere_corrections`. Exit criteria are a zero process
return, a non-empty `.pos` beginning with `% LibGNSS++ Position Solution`, and
a summary whose `qzss_l6` and epoch counts describe the input rather than an
empty fallback run. The [CLAS validated-datasets notes](../clas_validated_datasets.md)
show the same command family and its validation expectations.

## Native MADOCA L6E/L6D path

`apps/gnss.py ppp` dispatches to the native `gnss_ppp` executable. Build it
before asking the dispatcher for help. Use the existing [Robotics quick
start](../robotics_quickstart.md) for the full native build, or run this
small prerequisite from the repository root:

```bash
cmake -S . -B build -DCMAKE_BUILD_TYPE=Release
cmake --build build --target gnss_ppp --parallel 2
test -x build/apps/gnss_ppp
```

For native MADOCA Compact SSR materialization, inspect the current CLI and
then provide an L6E file plus its matching navigation data:

```bash
python3 apps/gnss.py ppp --help
python3 apps/gnss.py ppp \
  --obs <rover.obs> \
  --nav <navigation.nav> \
  --madoca-l6 <madoca-l6e-file> \
  --madoca-materialization-dump \
    output/use_cases/qzss_l6/madoca_l6e_materialization.csv \
  --summary-json output/use_cases/qzss_l6/madoca_materialization_summary.json \
  --madoca-materialization-dump-only
```

The materialization CSV is the first output for this diagnostic path. Its
header starts with `schema_version,sat,system,prn,week,tow,orbit_frame` and
continues with orbit/clock validity, reference epochs, IODs, RAC orbit
components, `clock_m`, `code_biases_m`, and `phase_biases_m` fields. A
non-empty file with at least one row and a zero return is the materialization
exit condition. For PPP application, retain the L6E input, add the L6D file,
and select the required per-frequency ambiguity mode:

```bash
python3 apps/gnss.py ppp \
  --obs <rover.obs> \
  --nav <navigation.nav> \
  --madoca-l6 <madoca-l6e-file> \
  --madoca-l6d <madoca-l6d-file> \
  --ar-method per-freq \
  --out output/use_cases/qzss_l6/madoca_solution.pos \
  --summary-json output/use_cases/qzss_l6/madoca_summary.json
```

`--madoca-l6d` is an opt-in per-frequency application path and requires an
L6E input. Use `--madoca-l6d-shadow <file>` when only the diagnostic shadow
comparison is wanted. A successful application has a non-empty libgnss++
`.pos` and summary JSON; check the summary's `processed_epochs`,
`valid_solutions`, `ppp_float_solutions`, `ppp_fixed_solutions`,
`fallback_solutions`, and MADOCA input/materialization fields before
comparing it.

## Supported boundary and next step

The current boundaries are intentionally specific:

- `qzss-l6-info` decodes the raw CSSR type 4073 stream. Its Python decoder
  has explicit branches for the direct CLAS vendor path at subtypes
  1/2/3/4/5/6/7/8/9/10/11/12.
- The native `libgnss++` QZSS L6 decoder exposes handlers for subtypes
  1/2/3/4/5/6/7/8/9/11; subtypes 10 and 12 stop at the native switch's
  unknown-subtype boundary. This is a different interface boundary from the
  inspection CLI.
- Native MADOCA L6E handles Compact SSR subtypes 1/2/3/4/5/7, while the
  L6D decoder covers the implemented MT1/MT2 ionosphere-region corrections.
- `clas-ppp` adapts raw L6 into the repository's compact-correction PPP path;
  native `ppp` consumes the MADOCA L6E/L6D options described above.

These are implemented slices, not a claim of full L6D, L6E, CLAS, MADOCA,
vendor, facility, or message coverage, nor a claim of PPP accuracy for every
capture. Use the [validation guide](../validation.md), [benchmark notes](../benchmarks.md),
and [interfaces](../interfaces.md) to define a measured next step. The
[CLAS decoder gap note](../references/claslib-gap.md) records known boundary
details.

Next step: retain the raw capture, input file hashes, GPS-week/time window,
materialization or summary keys, and a reference solution before tuning or
claiming correction performance.
