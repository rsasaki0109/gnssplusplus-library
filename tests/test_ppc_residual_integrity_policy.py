from __future__ import annotations

import sys
from pathlib import Path


ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "scripts"))

import analyze_ppc_wrong_fix_residuals as audit  # noqa: E402
import evaluate_ppc_residual_integrity_policy as policy_audit  # noqa: E402


def epoch(
    tow: float,
    *,
    status: int = 4,
    nsat: int = 15,
    ratio: float = 10.0,
    outliers: int = 12,
    prefit: float = 20.0,
    x: float = 0.0,
) -> audit.SolutionEpoch:
    return audit.SolutionEpoch(
        week=2300,
        tow_s=tow,
        ecef=(x, 0.0, 0.0),
        status=status,
        nsat=nsat,
        ratio=ratio,
        baseline_m=100.0,
        outliers=outliers,
        prefit_rms_m=prefit,
        prefit_max_m=prefit,
        post_rms_m=0.5,
        post_max_m=1.0,
        nis_per_obs=0.2,
        observations=20,
        telemetry_complete=True,
    )


def test_buffered_streak_selects_prefix_only_after_confirmation() -> None:
    epochs = [epoch(float(index), prefit=45.0) for index in range(8)]
    streak, spike = policy_audit.selected_indices(epochs, policy_audit.Policy())
    assert streak == set(range(8))
    assert not spike


def test_nonfixed_epoch_breaks_buffered_streak() -> None:
    epochs = [epoch(float(index), prefit=45.0) for index in range(7)]
    epochs.append(epoch(7.0, status=3, prefit=45.0))
    epochs.extend(epoch(float(index), prefit=45.0) for index in range(8, 15))
    streak, _ = policy_audit.selected_indices(epochs, policy_audit.Policy())
    assert not streak


def test_spike_gate_and_offline_labels_are_separate() -> None:
    epochs = [epoch(1.0, nsat=14, prefit=40.0, x=1.0)]
    result = policy_audit.evaluate_run(
        epochs,
        {(2300, 1.0): (0.0, 0.0, 0.0)},
        policy_audit.Policy(),
    )
    assert result["spike_selected"] == 1
    assert result["wrong_caught"] == 1
    assert result["correct_harmed"] == 0


def test_base_low_satellite_gate_is_part_of_staged_policy() -> None:
    result = policy_audit.evaluate_run(
        [epoch(1.0, nsat=11, ratio=10.0, prefit=1.0, x=1.0)],
        {(2300, 1.0): (0.0, 0.0, 0.0)},
        policy_audit.Policy(),
    )
    assert result["base_selected"] == 1
    assert result["selected_epochs"] == 1
    assert result["wrong_caught"] == 1


def test_base_gate_keeps_legacy_short_rows_fail_open() -> None:
    sample = epoch(1.0, nsat=11, ratio=10.0, prefit=1.0, x=1.0)
    sample = audit.SolutionEpoch(**{**sample.__dict__, "telemetry_complete": False})
    result = policy_audit.evaluate_run(
        [sample],
        {(2300, 1.0): (0.0, 0.0, 0.0)},
        policy_audit.Policy(),
    )
    assert result["base_selected"] == 0


def test_reference_matching_accepts_urbannav_half_epoch_offset() -> None:
    epochs = [epoch(1.0, nsat=14, prefit=40.0, x=1.0)]
    result = policy_audit.evaluate_run(
        epochs,
        {(2300, 1.1): (0.0, 0.0, 0.0)},
        policy_audit.Policy(),
        match_tolerance_s=0.25,
    )
    assert result["fixed_before"] == 1
    assert result["wrong_caught"] == 1


def test_reference_matching_rejects_epoch_outside_tolerance() -> None:
    result = policy_audit.evaluate_run(
        [epoch(1.0, nsat=14, prefit=40.0, x=1.0)],
        {(2300, 1.3): (0.0, 0.0, 0.0)},
        policy_audit.Policy(),
        match_tolerance_s=0.25,
    )
    assert result["fixed_before"] == 0
    assert result["selected_epochs"] == 1


def test_reference_loader_accepts_comma_space_urbannav_header(tmp_path: Path) -> None:
    reference = tmp_path / "reference.csv"
    reference.write_text(
        "GPS TOW (s), GPS Week, ECEF X (m), ECEF Y (m), ECEF Z (m)\n"
        "1.10, 2300, 1.0, 2.0, 3.0\n",
        encoding="utf-8",
    )
    assert audit.load_reference(reference) == {(2300, 1.1): (1.0, 2.0, 3.0)}
