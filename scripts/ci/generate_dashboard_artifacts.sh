#!/usr/bin/env bash
set -euo pipefail

repo_root="$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)"
cd "${repo_root}"

python_bin="${PYTHON:-python3}"
output_dir="${OUTPUT_DIR:-output}"
visibility_obs="${VISIBILITY_OBS:-data/rover_static.obs}"
visibility_nav="${VISIBILITY_NAV:-data/navigation_static.nav}"

mkdir -p "${output_dir}"
if [[ -f "${visibility_obs}" && -f "${visibility_nav}" ]]; then
    "${python_bin}" apps/gnss.py visibility \
        --obs "${visibility_obs}" \
        --nav "${visibility_nav}" \
        --csv "${output_dir}/visibility_static.csv" \
        --summary-json "${output_dir}/visibility_static_summary.json" \
        --max-epochs 5 \
        --quiet
    "${python_bin}" apps/gnss.py visibility-plot \
        "${output_dir}/visibility_static.csv" \
        "${output_dir}/visibility_static.png"
else
    echo "Skipping visibility artifact generation: sample data is unavailable." >&2
fi
"${python_bin}" apps/gnss.py artifact-manifest \
    --root . \
    --output "${output_dir}/artifact_manifest.json"
