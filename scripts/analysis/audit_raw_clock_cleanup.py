"""Independent reconstruction of source clock-drift cleanup from raw CSV values."""
from bisect import bisect_left
import math


def audit_raw_clock_cleanup(keys, originals, provenance, seeds):
    n = len(keys)
    if (len(originals) != n or len(provenance.get("epochs", [])) != n or
            len(seeds.get("seeds", [])) != n or
            seeds.get("clock_rate_source") != "source-raw-clock-drift-jump-mask-fill" or
            provenance.get("truth_used") is not False):
        raise ValueError("raw clock cleanup provenance scope mismatch")
    finite = lambda v: type(v) in (int, float) and math.isfinite(v)
    if any(not finite(seed.get("selected_clock_rate_mps")) for seed in seeds["seeds"]):
        raise ValueError("raw clock cleanup numeric seed export missing or nonfinite")
    masks = set()
    for i in range(1, n):
        if finite(originals[i-1]) and finite(originals[i]) and abs(originals[i]-originals[i-1]) > 50:
            masks.update((i-1, i))
    known = [i for i, v in enumerate(originals) if finite(v) and i not in masks]
    if not known:
        raise ValueError("raw clock cleanup has no finite support")
    filled = 0
    close = lambda a, b: finite(a) and math.isclose(a, b, rel_tol=1e-12, abs_tol=1e-12)
    for i, (row, seed) in enumerate(zip(provenance["epochs"], seeds["seeds"])):
        if row.get("utc") != keys[i] or row.get("raw_source_index") != i or seed.get("raw_source_index") != i:
            raise ValueError("raw clock cleanup epoch identity mismatch")
        original = row.get("original_drift_mps")
        if (finite(originals[i]) and not close(original, originals[i])) or (not finite(originals[i]) and original is not None):
            raise ValueError("raw clock cleanup original measurement mismatch")
        left = right = i
        weight = 0.0
        if i not in known:
            filled += 1
            upper = bisect_left(known, i)
            if masks and len(known) >= 2:
                left, right = (known[:2] if upper == 0 else known[-2:] if upper == len(known)
                               else (known[upper-1], known[upper]))
                weight = (i-left)/(right-left)
            else:
                left = right = (known[0] if upper == 0 else known[-1] if upper == len(known)
                                else known[upper-1] if i-known[upper-1] < known[upper]-i else known[upper])
        expected = originals[left] + weight * (originals[right] - originals[left])
        if (row.get("left_epoch_index") != left or row.get("right_epoch_index") != right or
                not close(row.get("right_weight"), weight) or
                not close(row.get("selected_drift_mps"), expected) or
                not close(seed.get("selected_clock_rate_mps"), expected)):
            raise ValueError("raw clock cleanup donor or selected value mismatch")
    if provenance.get("jump_masks") != len(masks) or provenance.get("filled_values") != filled:
        raise ValueError("raw clock cleanup count mismatch")
    return {"epochs": n, "jump_masks": len(masks), "filled_values": filled,
            "verified_against_original_raw_csv": True, "output_coordinate_interpolation": False}
