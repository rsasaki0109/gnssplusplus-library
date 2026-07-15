"""Shared Compact SSR bias-bank operations for the QZSS L6 decoder.

The decoder keeps code and phase biases in the same nested shape.  This module
owns the time-bucket and lookup mechanics so the two correction families cannot
silently drift apart.  Policy names remain defined by ``gnss_qzss_l6_info``;
callers pass them explicitly to keep this module independent of the CLI.
"""

from __future__ import annotations

from collections.abc import Iterable
from collections.abc import Callable


BiasRows = dict[int, float]
SatelliteBiasBank = dict[str, BiasRows]
BiasBanks = dict[int, SatelliteBiasBank]


def bank_anchor_tow(tow: int, bucket_seconds: int) -> int:
    """Return the inclusive time anchor for a bias bank."""
    if bucket_seconds <= 0:
        raise ValueError("bias-bank bucket must be positive")
    return tow - (tow % bucket_seconds)


def prune_bias_banks(
    banks: BiasBanks | None,
    current_tow: int,
    *,
    bucket_seconds: int,
    retention_seconds: int,
) -> BiasBanks | None:
    """Drop banks older than the configured retention window."""
    if banks is None:
        return None
    min_anchor = (
        bank_anchor_tow(current_tow, bucket_seconds) - retention_seconds
    )
    return {
        anchor: bank
        for anchor, bank in banks.items()
        if anchor >= min_anchor
    }


def store_bias_bank_entry(
    banks: BiasBanks | None,
    tow: int,
    satellite_token: str,
    signal_id: int,
    bias_m: float,
    *,
    bucket_seconds: int,
    retention_seconds: int,
) -> BiasBanks:
    """Store one bias value and return the possibly-created bank mapping."""
    retained = prune_bias_banks(
        banks,
        tow,
        bucket_seconds=bucket_seconds,
        retention_seconds=retention_seconds,
    )
    if retained is None:
        retained = {}
    anchor = bank_anchor_tow(tow, bucket_seconds)
    bank = retained.setdefault(anchor, {})
    bank.setdefault(satellite_token, {})[signal_id] = bias_m
    return retained


def candidate_bank_anchors(
    banks: BiasBanks,
    tow: int,
    bank_policy: str,
    *,
    pending_policy: str,
    same_bucket_policy: str,
    close_bucket_policy: str,
    latest_preceding_policy: str,
    bucket_seconds: int,
    delayed_policy: str | None = None,
    effective_delay_seconds: int = 0,
) -> list[int]:
    """Return lookup anchors in precedence order for a validated policy."""
    policies = {
        pending_policy,
        same_bucket_policy,
        close_bucket_policy,
        latest_preceding_policy,
    }
    if delayed_policy is not None:
        policies.add(delayed_policy)
    if bank_policy not in policies:
        raise ValueError(f"unsupported Compact SSR bias-bank policy: {bank_policy}")
    if bank_policy == pending_policy or not banks:
        return []
    current_anchor = bank_anchor_tow(tow, bucket_seconds)
    if bank_policy == same_bucket_policy:
        return [current_anchor]
    if bank_policy == close_bucket_policy:
        anchors = [
            anchor
            for anchor in banks
            if anchor <= tow and (current_anchor - anchor) <= bucket_seconds
        ]
    elif delayed_policy is not None and bank_policy == delayed_policy:
        effective_tow = tow - effective_delay_seconds
        anchors = [anchor for anchor in banks if anchor <= effective_tow]
    else:
        anchors = [anchor for anchor in banks if anchor <= tow]
    anchors.sort(reverse=True)
    return anchors


def lookup_bias_bank_entry(
    banks: BiasBanks | None,
    anchors: Iterable[int],
    satellite_token: str,
    signal_id: int,
) -> float | None:
    """Find one signal value in the first eligible bank that contains it."""
    if not banks:
        return None
    for anchor in anchors:
        bias_m = banks.get(anchor, {}).get(satellite_token, {}).get(signal_id)
        if bias_m is not None:
            return bias_m
    return None


def lookup_bias_bank_rows(
    banks: BiasBanks | None,
    anchors: Iterable[int],
    satellite_token: str,
) -> BiasRows:
    """Find one satellite's rows in the first eligible bank that contains it."""
    if not banks:
        return {}
    for anchor in anchors:
        rows = banks.get(anchor, {}).get(satellite_token)
        if rows:
            return dict(rows)
    return {}


def resolve_bias_rows(
    bank_rows: BiasRows,
    pending_base_rows: dict[str, BiasRows] | None,
    satellite_token: str,
) -> BiasRows:
    """Overlay message-local base rows on rows restored from a bank."""
    rows = dict(bank_rows)
    if pending_base_rows is not None:
        rows.update(pending_base_rows.get(satellite_token, {}))
    return rows


def materialize_missing_bias_rows(
    pending_rows: dict[str, BiasRows],
    *,
    target_satellites: Iterable[str],
    resolve_rows: Callable[[str], BiasRows],
    pending_sources: dict[str, dict[int, int]] | None = None,
    source_subtype: int | None = None,
) -> None:
    """Extend a correction row set with base-bank rows without overwriting data."""
    if pending_sources is not None and source_subtype is None:
        raise ValueError("source subtype is required when materializing source rows")
    for satellite_token in sorted(target_satellites):
        base_rows = resolve_rows(satellite_token)
        if not base_rows:
            continue
        satellite_biases = pending_rows.setdefault(satellite_token, {})
        satellite_sources = (
            pending_sources.setdefault(satellite_token, {})
            if pending_sources is not None
            else None
        )
        for signal_id, bias_m in base_rows.items():
            if signal_id in satellite_biases:
                continue
            satellite_biases[signal_id] = bias_m
            if satellite_sources is not None:
                satellite_sources[signal_id] = source_subtype  # type: ignore[assignment]


def compose_bias_value(
    network_bias_m: float,
    base_bias_m: float | None,
    composition_policy: str,
    *,
    direct_policy: str,
    base_plus_network_policy: str,
    base_only_if_present_policy: str,
) -> float:
    """Apply the shared code/phase base-bank composition contract."""
    if composition_policy not in {
        direct_policy,
        base_plus_network_policy,
        base_only_if_present_policy,
    }:
        raise ValueError(
            "unsupported Compact SSR bias composition policy: "
            f"{composition_policy}"
        )
    if composition_policy == direct_policy or base_bias_m is None:
        return network_bias_m
    if composition_policy == base_plus_network_policy:
        return base_bias_m + network_bias_m
    return base_bias_m
