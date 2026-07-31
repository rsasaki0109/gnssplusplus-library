from __future__ import annotations

import argparse
from pathlib import Path
import sys

import numpy as np


ROOT_DIR = Path(__file__).resolve().parents[1]
SCRIPTS_DIR = ROOT_DIR / "scripts"
if str(SCRIPTS_DIR) not in sys.path:
    sys.path.insert(0, str(SCRIPTS_DIR))

import apply_ppc_integrity_consensus as consensus  # noqa: E402
import generate_driving_comparison as comparison  # noqa: E402


def args() -> argparse.Namespace:
    return argparse.Namespace(
        agreement_aperture_m=10.0,
        suspect_streak=2,
        quarantine_streak=3,
        recovery_streak=5,
        shadow_max_gdop=3.0,
        shadow_max_ddpr_rms_m=40.0,
        shadow_min_satellites=8,
        primary_max_prefit_rms_m=10.0,
        primary_min_outliers=35,
        primary_max_covariance_trace_m2=0.01,
        allow_recovery_fixed_output=False,
        recovery_min_ratio=3.0,
        recovery_max_separation_m=6.0,
        recovery_max_prefit_rms_m=5.0,
        recovery_max_outliers=20,
        allow_soft_suspect_without_shadow=False,
        require_primary_suspect_for_disagreement=False,
    )


def primary(tow: float, x_m: float = 0.0, *, prefit: float = 1.0, outliers: int = 0):
    return comparison.SolutionEpoch(
        week=2325,
        tow=tow,
        lat_deg=0.0,
        lon_deg=0.0,
        height_m=0.0,
        ecef=np.array([x_m, 0.0, 0.0]),
        status=4,
        num_satellites=20,
        ratio=5.0,
        rtk_update_prefit_residual_rms_m=prefit,
        rtk_update_suppressed_outliers=outliers,
    )


def shadow(tow: float, x_m: float = 0.0):
    return consensus.ShadowEpoch(
        tow=tow,
        ecef=np.array([x_m, 0.0, 0.0]),
        status="FLOAT",
        gdop=2.0,
        ddpr_rms_m=5.0,
        nsat=20,
    )


def test_disagreement_quarantines_then_consecutive_agreement_recovers() -> None:
    epochs = [primary(float(tow), 100.0 if 1 <= tow <= 3 else 0.0) for tow in range(9)]
    shadows = {float(tow): shadow(float(tow)) for tow in range(9)}

    output, summary, ledger = consensus.apply_consensus(epochs, shadows, {}, args())

    assert output[1].status == 4  # one-epoch detection latency
    assert [epoch.status for epoch in output[2:8]] == [3] * 6
    assert output[8].status == 4
    assert ledger[3]["state"] == consensus.QUARANTINE
    assert ledger[8]["promote_joint_anchor"] is True
    assert summary["positions_replaced"] == 0
    assert summary["fixed_status_demotions"] == 6


def test_missing_shadow_cannot_recover_from_quarantine() -> None:
    epochs = [primary(float(tow), 100.0) for tow in range(5)]
    shadows = {0.0: shadow(0.0), 1.0: shadow(1.0), 2.0: shadow(2.0)}

    output, summary, ledger = consensus.apply_consensus(epochs, shadows, {}, args())

    assert ledger[-1]["state"] == consensus.QUARANTINE
    assert output[-1].status == 3
    assert summary["final_state"] == consensus.QUARANTINE


def test_missing_shadow_soft_suspect_does_not_create_unrecoverable_quarantine() -> None:
    epochs = [
        primary(float(tow), prefit=12.0, outliers=40) for tow in range(5)
    ]

    output, summary, ledger = consensus.apply_consensus(epochs, {}, {}, args())

    assert all(epoch.status == 4 for epoch in output)
    assert all(not row["effective_primary_suspect"] for row in ledger)
    assert summary["final_state"] == consensus.NORMAL


def test_joint_evidence_mode_ignores_clean_primary_disagreement() -> None:
    replay_args = args()
    replay_args.require_primary_suspect_for_disagreement = True
    epochs = [primary(float(tow), 100.0) for tow in range(5)]
    shadows = {float(tow): shadow(float(tow)) for tow in range(5)}

    output, summary, _ = consensus.apply_consensus(
        epochs, shadows, {}, replay_args
    )

    assert all(epoch.status == 4 for epoch in output)
    assert summary["final_state"] == consensus.NORMAL


def test_overconfidence_debug_signal_enters_quarantine_immediately() -> None:
    epoch = primary(1.0, prefit=12.0, outliers=40)
    debug = {
        (2325, 1.0): {
            "float_position_covariance_trace_m2": "0.001",
        }
    }

    output, _, ledger = consensus.apply_consensus(
        [epoch], {1.0: shadow(1.0)}, debug, args()
    )

    assert output[0].status == 3
    assert ledger[0]["hard_primary_suspect"] is True
    assert ledger[0]["request_primary_reset"] is True


def test_shadow_agreement_exonerates_soft_primary_suspect() -> None:
    epochs = [primary(float(tow), prefit=12.0, outliers=40) for tow in range(4)]
    shadows = {float(tow): shadow(float(tow)) for tow in range(4)}

    output, summary, ledger = consensus.apply_consensus(epochs, shadows, {}, args())

    assert all(epoch.status == 4 for epoch in output)
    assert all(row["primary_suspect"] for row in ledger)
    assert not any(row["effective_primary_suspect"] for row in ledger)
    assert summary["final_state"] == consensus.NORMAL


def test_healthy_recovery_candidate_is_provisional_fixed() -> None:
    replay_args = args()
    replay_args.allow_recovery_fixed_output = True
    epochs = [
        primary(0.0, prefit=12.0, outliers=40),
        primary(1.0, prefit=1.0, outliers=2),
    ]
    debug = {
        (2325, 0.0): {"float_position_covariance_trace_m2": "0.001"},
    }

    output, _, ledger = consensus.apply_consensus(
        epochs, {0.0: shadow(0.0), 1.0: shadow(1.0)}, debug, replay_args
    )

    assert output[0].status == 3
    assert output[1].status == 4
    assert ledger[1]["state"] == consensus.RECOVERY
    assert ledger[1]["provisional_recovery_fixed"] is True
    assert ledger[1]["promote_joint_anchor"] is False
