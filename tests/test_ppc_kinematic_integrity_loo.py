from __future__ import annotations

from pathlib import Path
import sys

import numpy as np


ROOT_DIR = Path(__file__).resolve().parents[1]
SCRIPTS_DIR = ROOT_DIR / "scripts"
if str(SCRIPTS_DIR) not in sys.path:
    sys.path.insert(0, str(SCRIPTS_DIR))

import evaluate_ppc_kinematic_integrity_loo as loo  # noqa: E402


def test_candidate_mask_holds_current_and_bounded_following_epochs() -> None:
    rows = np.asarray(
        [
            (1, 0, 0, 0.1),
            (1, 13, 300, 20.0),
            (1, 0, 0, 20.0),
            (1, 0, 0, 0.1),
        ],
        dtype=float,
    )

    mask = loo.candidate_mask(rows, 12.0, 200.0, 2)

    assert mask.tolist() == [False, True, True, False]
    assert loo.score_mask(rows, mask) == {
        "demoted": 2,
        "correct_fix_harmed": 0,
        "wrong_fix_caught": 2,
        "wrong_fix_gt5m_caught": 2,
        "wrong_fix_gt10m_caught": 2,
    }


def test_advanced_mask_extends_plateau_and_uses_secondary_telemetry() -> None:
    # base, jump, acceleration, truth error, prefit, ratio, outliers, satellites
    rows = np.asarray(
        [
            (1, 0, 0, 0.1, 0, 20, 0, 20),
            (1, 13, 300, 20, 0, 20, 0, 20),
            (1, 0.05, 0, 20, 0, 20, 0, 20),
            (1, 0.05, 0, 20, 0, 20, 0, 20),
            (1, 0.05, 0, 20, 0, 20, 0, 20),
            (1, 1, 0, 0.1, 0, 20, 0, 20),
            (1, 6, 110, 20, 6, 9, 10, 13),
            (1, 1, 0, 0.1, 0, 20, 0, 20),
        ],
        dtype=float,
    )

    mask = loo.advanced_candidate_mask(rows, 0.1, 5, (5.0, 10.0, 10, 13))

    assert mask.tolist() == [False, True, True, True, True, False, True, True]
