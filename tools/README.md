# Standalone tools

These are small command-line tools that can be used independently of the
`apps/gnss.py` dispatcher and are installed under the package's `tools/`
directory.

- `rtk_stats.py`: text-only RTK solution statistics.
- `compare_rtklib.py`: LibGNSS++/RTKLIB comparison statistics and plots.
- `plot_rtk.py`: RTK status and trajectory plots.
- `plot_trajectory.py`: status-colored 2D trajectory comparison.
- `rtk_geometry.py`: shared WGS84/ECEF/ENU implementation used by the RTK
  tools above; it is a library helper, not a user-facing command.
- `ubx_reader.cpp` and `gnss_data_generator.cpp`: native receiver-data tools.

Generated plots, `.pos` files, and logs should be written below `output/`.
Keep reusable code here; keep experiment-specific orchestration in `scripts/`
and user-facing commands in `apps/`.
