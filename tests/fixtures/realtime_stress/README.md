# Recorded RTCM3 fixture for `gnss realtime-stress` and gnss_live tests

A 40 s excerpt (GPS only, 5 Hz rover, 1 Hz base) of the public Precise Positioning Challenge
dataset, Tokyo run1 (`data/PPC-Dataset/tokyo/run1`, MIT licence, (c) 2025 Taro Suzuki), encoded
to RTCM3 MSM7 + 1005 + 1019 with `gnss_rtcm_encode`:

```bash
gnss realtime-stress prepare --run-dir data/PPC-Dataset/tokyo/run1 --out-dir /tmp/fx \
    --duration 40 --eph-interval 20
```

* `rover.rtcm` / `base.rtcm`: RTCM3 byte streams; `*.idx`: one line per epoch
  (`tow week offset length obs_messages aux_messages`).
* `reference.csv`: the matching rows of the dataset's 5 Hz reference trajectory.
* `manifest.json`: loadable by `gnss realtime-stress run --prepared tests/fixtures/realtime_stress`.

No receiver is involved; this is recorded data only.
