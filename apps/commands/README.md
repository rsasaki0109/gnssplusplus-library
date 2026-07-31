# Python commands

Source-tree Python commands are grouped by responsibility:

- `diagnostics/`: environment, ROS, comparison, and residual diagnostics.
- `receivers/`: receiver protocols, stream control, and correction decoding.
- `products/`: GNSS product fetching, conversion, and inspection.
- `positioning/`: operational positioning and sign-off workflows.
- `benchmarks/`: reproducibility, dataset evaluation, and parity harnesses.
- `visualization/`: reports, plots, manifests, and the local web UI.
- `support/`: shared importable implementation modules.

Use `python3 apps/gnss.py <command>` from the source tree. CMake installs the
command files into one flat `bin/` directory, so public executable names do
not depend on this source layout.
