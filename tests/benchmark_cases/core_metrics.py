"""PPC core metric and status benchmark test cases."""

from ._support import *  # noqa: F401,F403

__all__ = [
    "PPCGoalScorecardTest",
    "PPCStatusTrajectoryTest",
    "StatusPositionMergeTest",
    "TcFgoNpzConverterTest",
    "PPCMetricsTest",
]


class PPCGoalScorecardTest(unittest.TestCase):
    def test_build_payload_keeps_public_profiles_and_gici_separate(self) -> None:
        metric_values = {
            "fix_rate_pct": 80.0,
            "wrong_fix_rate_pct": 2.0,
            "correct_fix_ref_pct": 70.0,
            "ppc_score_3d_50cm_ref_pct": 75.0,
            "ppc_official_score_pct": 76.0,
            "p95_h_m": 1.0,
            "p95_abs_up_m": 2.0,
        }
        with tempfile.TemporaryDirectory(prefix="gnss_ppc_goal_scorecard_") as td:
            root = Path(td)
            gici_dir = root / "gici"
            gici_dir.mkdir()
            matrix = {
                "runs": [
                    {"label": key, **metric_values}
                    for key, _, _ in ppc_goal_scorecard.RUNS
                ],
                "macro_mean": metric_values,
                "weighted_official_score_pct": 78.75,
            }
            matrix_path = root / "matrix.json"
            matrix_path.write_text(json.dumps(matrix), encoding="utf-8")
            gici_metrics = {key: value - 10.0 for key, value in metric_values.items()}
            for _, _, filename in ppc_goal_scorecard.RUNS:
                (gici_dir / filename).write_text(
                    json.dumps({"solutions": [gici_metrics]}), encoding="utf-8"
                )
            nagoya_path = root / "nagoya.json"
            nagoya_path.write_text(
                json.dumps(
                    {
                        "fix_rate_pct": 85.11,
                        "wrong_fix_rate_pct": 0.9,
                        "p95_h_m": 1.4,
                    }
                ),
                encoding="utf-8",
            )
            consensus_path = root / "consensus.json"
            consensus_path.write_text(
                json.dumps(
                    {
                        "runtime_truth_used": False,
                        "positions_replaced": 0,
                        "agreement_aperture_m": 5.0,
                        "shadow_max_gdop": 4.0,
                        "final_state": "NORMAL",
                    }
                ),
                encoding="utf-8",
            )
            integrity_path = root / "integrity.json"
            integrity_path.write_text(
                json.dumps(
                    {
                        "policy": {"streak_prefit_rms_m": 40.0},
                        "reference_truth_used_by_runtime_policy": False,
                        "bounded_output_latency_epochs": 7,
                        "external_validation_status": "safe_no_false_demotions",
                        "external_policy_active": True,
                    }
                ),
                encoding="utf-8",
            )
            args = argparse.Namespace(
                lib_matrix=matrix_path,
                gici_matrix=None,
                gici_results_dir=gici_dir,
                nagoya1_public_summary=nagoya_path,
                summary_json=root / "summary.json",
                comparison_png=root / "comparison.png",
                targets_png=root / "targets.png",
                gici_commit="deadbeef",
                status_demotion_min_satellites=9,
                status_demotion_low_satellite_ceiling=11,
                status_demotion_low_satellite_max_ratio=15.0,
                online_consensus_summary=[consensus_path],
                staged_integrity_audit=integrity_path,
            )

            payload = ppc_goal_scorecard.build_payload(args)

            self.assertEqual(payload["weighted_official_score_pct"], 78.75)
            self.assertEqual(payload["macro_mean"]["libgnss"]["fix_rate_pct"], 80.0)
            self.assertEqual(payload["macro_mean"]["gici"]["fix_rate_pct"], 70.0)
            self.assertEqual(payload["targets"][1]["achieved_pct"], 85.11)
            self.assertIn("separate", payload["targets"][1]["profile"])
            self.assertFalse(payload["evaluation"]["reference_used_by_runtime_selector"])
            self.assertEqual(
                payload["evaluation"]["status_demotion"]["min_satellites"], 9
            )
            self.assertEqual(
                payload["evaluation"]["status_demotion"]["low_satellite_ceiling"], 11
            )
            self.assertFalse(
                payload["evaluation"]["status_demotion"]["reference_used"]
            )
            consensus = payload["evaluation"]["online_consensus_replays"][0]
            self.assertFalse(consensus["runtime_truth_used"])
            self.assertEqual(consensus["positions_replaced"], 0)
            self.assertEqual(consensus["final_state"], "NORMAL")
            staged = payload["evaluation"]["staged_integrity_policy"]
            self.assertFalse(staged["runtime_truth_used"])
            self.assertTrue(staged["external_policy_active"])
            self.assertEqual(
                staged["external_validation_status"], "safe_no_false_demotions"
            )


class PPCStatusTrajectoryTest(unittest.TestCase):
    def test_loads_selected_pos_paths_from_goal_metrics(self) -> None:
        with tempfile.TemporaryDirectory(prefix="gnss_ppc_status_paths_") as td:
            metrics = Path(td) / "metrics.json"
            metrics.write_text(
                json.dumps(
                    {
                        "runs": [
                            {
                                "key": key,
                                "libgnss": {"pos": f"output/{key}.pos"},
                            }
                            for key, _, _ in ppc_status_trajectories.RUNS
                        ]
                    }
                ),
                encoding="utf-8",
            )

            paths = ppc_status_trajectories.load_solution_paths(None, metrics)

        self.assertEqual(set(paths), {key for key, _, _ in ppc_status_trajectories.RUNS})
        self.assertEqual(
            paths["tokyo_run1"],
            ppc_status_trajectories.ROOT_DIR / "output/tokyo_run1.pos",
        )


class StatusPositionMergeTest(unittest.TestCase):
    @staticmethod
    def epoch(tow: float, x_m: float, status: int) -> comparison.SolutionEpoch:
        return comparison.SolutionEpoch(
            2300, tow, 0.0, 0.0, 0.0, np.asarray([x_m, 0.0, 0.0]), status, 12
        )

    def test_replaces_position_only_on_requested_status_transition(self) -> None:
        baseline = [self.epoch(0.0, 0.0, 3), self.epoch(1.0, 1.0, 4)]
        candidate = [self.epoch(0.0, 10.0, 4), self.epoch(1.0, 11.0, 4)]

        merged, summary = status_position_merge.merge_positions(baseline, candidate, 3, 4)

        np.testing.assert_allclose(merged[0].ecef, candidate[0].ecef)
        np.testing.assert_allclose(merged[1].ecef, baseline[1].ecef)
        self.assertEqual([epoch.status for epoch in merged], [3, 4])
        self.assertEqual(summary["replaced_positions"], 1)
        self.assertFalse(summary["reference_truth_used"])

    def test_displacement_gate_rejects_far_candidate(self) -> None:
        baseline = [self.epoch(0.0, 0.0, 3)]
        candidate = [self.epoch(0.0, 10.0, 4)]

        merged, summary = status_position_merge.merge_positions(
            baseline, candidate, 3, 4, max_displacement_m=5.0
        )

        np.testing.assert_allclose(merged[0].ecef, baseline[0].ecef)
        self.assertEqual(summary["replaced_positions"], 0)
        self.assertEqual(summary["displacement_rejected_positions"], 1)

    def test_optional_satellite_gate_works_across_baseline_statuses(self) -> None:
        baseline = [self.epoch(0.0, 0.0, 3), self.epoch(1.0, 1.0, 4)]
        baseline[1] = dataclasses.replace(baseline[1], num_satellites=17)
        candidate = [self.epoch(0.0, 10.0, 4), self.epoch(1.0, 11.0, 4)]

        merged, summary = status_position_merge.merge_positions(
            baseline,
            candidate,
            None,
            4,
            max_baseline_satellites=16,
        )

        np.testing.assert_allclose(merged[0].ecef, candidate[0].ecef)
        np.testing.assert_allclose(merged[1].ecef, baseline[1].ecef)
        self.assertEqual([epoch.status for epoch in merged], [3, 4])
        self.assertEqual(summary["replaced_positions"], 1)
        self.assertEqual(summary["telemetry_rejected_positions"], 1)

    def test_preserve_existing_up_applies_only_horizontal_correction(self) -> None:
        baseline = [self.epoch(0.0, 0.0, 3)]
        candidate = [self.epoch(0.0, 10.0, 4)]
        candidate[0] = dataclasses.replace(
            candidate[0], ecef=np.asarray([10.0, 5.0, 0.0])
        )

        merged, summary = status_position_merge.merge_positions(
            baseline, candidate, 3, 4, preserve_existing_up=True
        )

        np.testing.assert_allclose(merged[0].ecef, [0.0, 5.0, 0.0], atol=1e-9)
        self.assertTrue(summary["preserve_existing_up"])

    def test_can_promote_candidate_status_with_ratio_gate(self) -> None:
        baseline = [self.epoch(0.0, 0.0, 3), self.epoch(1.0, 1.0, 3)]
        candidate = [
            dataclasses.replace(self.epoch(0.0, 0.05, 4), ratio=4.0),
            dataclasses.replace(self.epoch(1.0, 1.05, 4), ratio=2.0),
        ]

        merged, summary = status_position_merge.merge_positions(
            baseline,
            candidate,
            3,
            4,
            max_displacement_m=0.075,
            min_candidate_ratio=3.0,
            promote_candidate_status=True,
        )

        self.assertEqual([epoch.status for epoch in merged], [4, 3])
        self.assertEqual(merged[0].ratio, 4.0)
        self.assertEqual(summary["replaced_positions"], 1)
        self.assertEqual(summary["ratio_rejected_positions"], 1)
        self.assertFalse(summary["preserved_status_labels"])
        self.assertFalse(summary["preserved_baseline_telemetry"])


class TcFgoNpzConverterTest(unittest.TestCase):
    def test_converts_estimator_fields_and_ignores_truth(self) -> None:
        with tempfile.TemporaryDirectory(prefix="gnsspp_tcfgo_convert_") as temp_dir:
            root = Path(temp_dir)
            source = root / "oracle.npz"
            output = root / "oracle.pos"
            np.savez(
                source,
                week=np.asarray([2323.0, 2323.0]),
                tow=np.asarray([555720.0, 555720.2]),
                sol_xyz=np.asarray(
                    [[-3817680.0, 3562840.0, 3650159.0],
                     [-3817680.1, 3562840.1, 3650159.1]]
                ),
                smode=np.asarray([4, 5]),
                nb=np.asarray([8, 0]),
                truth_xyz=np.full((2, 3), 123456789.0),
            )

            self.assertEqual(tcfgo_converter.convert(source, output), 2)
            rows = [
                line.split()
                for line in output.read_text(encoding="ascii").splitlines()
                if line and not line.startswith("%")
            ]
            self.assertEqual([row[8] for row in rows], ["4", "3"])
            self.assertEqual([row[0] for row in rows], ["2323", "2323"])
            self.assertAlmostEqual(float(rows[1][1]), 555720.2, places=3)

    def test_requires_timestamps(self) -> None:
        with tempfile.TemporaryDirectory(prefix="gnsspp_tcfgo_convert_") as temp_dir:
            root = Path(temp_dir)
            source = root / "oracle.npz"
            np.savez(source, sol_xyz=np.zeros((1, 3)), smode=np.asarray([4]))
            with self.assertRaisesRegex(ValueError, "week"):
                tcfgo_converter.convert(source, root / "oracle.pos")

class PPCMetricsTest(unittest.TestCase):
    def test_fgo_parity_csv_uses_common_ppc_epoch_metrics(self) -> None:
        origin = comparison.llh_to_ecef(35.0, 139.0, 50.0)
        reference = [
            comparison.ReferenceEpoch(2300, 1.0, 35.0, 139.0, 50.0, origin),
            comparison.ReferenceEpoch(2300, 2.0, 35.0, 139.0, 50.0, origin),
        ]
        with tempfile.TemporaryDirectory(prefix="fgo_ppc_csv_") as temp_dir:
            csv_path = Path(temp_dir) / "fgo.csv"
            csv_path.write_text(
                "tow,status,e_pos_m,n_pos_m,u_err_m,nsat,ratio\n"
                "1.000,FIXED,0.000,0.000,0.100,12,4.5\n"
                "2.000,FLOAT,0.300,0.000,0.000,10,0.0\n",
                encoding="ascii",
            )

            epochs = ppc_metrics.load_fgo_parity_csv(csv_path, reference)
            payload = ppc_metrics.summarize_solution_epochs(
                reference,
                epochs,
                fixed_status=4,
                label="FGO test",
                match_tolerance_s=0.11,
                solver_wall_time_s=None,
            )

        self.assertEqual([epoch.status for epoch in epochs], [4, 3])
        self.assertEqual(epochs[0].num_satellites, 12)
        self.assertAlmostEqual(epochs[0].ratio, 4.5)
        self.assertEqual(payload["fix_rate_pct"], 50.0)
        self.assertEqual(payload["wrong_fix_epochs"], 0)
        self.assertEqual(payload["ppc_score_3d_50cm_ref_pct"], 100.0)

    def test_fgo_parity_csv_prefers_self_contained_up_position(self) -> None:
        origin = comparison.llh_to_ecef(35.0, 139.0, 50.0)
        reference = [
            comparison.ReferenceEpoch(2300, 1.0, 35.0, 139.0, 50.0, origin),
        ]
        with tempfile.TemporaryDirectory(prefix="fgo_ppc_up_position_") as temp_dir:
            csv_path = Path(temp_dir) / "fgo.csv"
            csv_path.write_text(
                "tow,status,e_pos_m,n_pos_m,u_pos_m,u_err_m,nsat,ratio\n"
                "1.000,FLOAT,0.000,0.000,2.000,99.000,10,0.0\n",
                encoding="ascii",
            )

            epochs = ppc_metrics.load_fgo_parity_csv(csv_path, reference)
            error_enu = comparison.ecef_to_enu(
                epochs[0].ecef - origin,
                reference[0].lat_deg,
                reference[0].lon_deg,
            )

        self.assertAlmostEqual(error_enu[2], 2.0, places=5)

    def test_fgo_parity_csv_prefers_absolute_ecef_position(self) -> None:
        origin = comparison.llh_to_ecef(35.0, 139.0, 50.0)
        expected = origin + np.asarray([1.0, 2.0, 3.0])
        reference = [
            comparison.ReferenceEpoch(2300, 1.0, 35.0, 139.0, 50.0, origin),
        ]
        with tempfile.TemporaryDirectory(prefix="fgo_ppc_ecef_position_") as temp_dir:
            csv_path = Path(temp_dir) / "fgo.csv"
            csv_path.write_text(
                "tow,status,e_pos_m,n_pos_m,u_pos_m,u_err_m,"
                "x_ecef_m,y_ecef_m,z_ecef_m,nsat,ratio\n"
                f"1.000,FLOAT,99,99,99,99,{expected[0]},{expected[1]},"
                f"{expected[2]},10,0.0\n",
                encoding="ascii",
            )

            epochs = ppc_metrics.load_fgo_parity_csv(csv_path, reference)

        np.testing.assert_allclose(epochs[0].ecef, expected, atol=1e-6)
