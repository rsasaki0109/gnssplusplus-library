"""Executable argument checks; every payload path is deliberately absent."""
import json
import os
from pathlib import Path
import subprocess
import unittest

ROOT = Path(__file__).resolve().parents[1]


class PairedEpochStatesCliTest(unittest.TestCase):
    def args(self):
        manifest = ROOT / 'docs/use_cases/records/smartphone_r5_phase234_h_native_phase233_meter_sigma_manifest_v1.json'
        args = json.loads(manifest.read_text())['argv'].copy()
        for flag in ('--android-gnss', '--android-imu', '--nav', '--out', '--summary-json'):
            args[args.index(flag) + 1] = '/nonexistent/gnss-paired-preflight/' + flag[2:]
        return args + [
            '--native-paired-epoch-states', '--native-base-pseudorange-compensation',
            '--native-base-pseudorange-source-miss-mask', '--native-base-rinex',
            '/nonexistent/gnss-paired-preflight/base.obs',
            '--native-base-rinex-sha256', '0' * 64]

    def run_cli(self, args):
        env = os.environ.copy()
        env['LD_LIBRARY_PATH'] = '/home/sasaki/.local/lib:' + env.get('LD_LIBRARY_PATH', '')
        result = subprocess.run(args, cwd=ROOT, env=env, capture_output=True, text=True, timeout=15)
        self.assertNotEqual(result.returncode, 0)
        self.assertEqual(result.stdout, '')
        return result.stderr

    def test_complete_recipe_reaches_raw_ingress(self):
        self.assertIn('failed to open raw Android GNSS CSV', self.run_cli(self.args()))

    def test_frequency_state_baselines_reach_raw_ingress(self):
        args = self.args()
        args = args[:args.index('--native-paired-epoch-states')]
        for route in ('2021-08-24-20-32-us-ca-mtv-h/pixel5',
                      '2022-04-01-18-22-us-ca-lax-t/pixel5'):
            variant = args.copy()
            variant[variant.index('--dataset-id') + 1] = route
            self.assertIn('failed to open raw Android GNSS CSV', self.run_cli(
                variant + ['--native-tdcp-frequency-residual-states']))

    def test_frequency_state_rejects_ablation_mix(self):
        args = self.args()
        base_off = args[:args.index('--native-paired-epoch-states')]
        for variant in (args, base_off + ['--native-tdcp-no-code-jump-gate'],
                        base_off + ['--native-stationary-gyro-initializer']):
            self.assertIn('requires raw H/LAX-T baseline frequency experiment recipe',
                          self.run_cli(variant + ['--native-tdcp-frequency-residual-states']))

    def test_frequency_state_requires_source_sigma_and_scoped_route(self):
        args = self.args()
        args = args[:args.index('--native-paired-epoch-states')]
        missing_sigma = args.copy()
        missing_sigma.remove('--native-source-tdcp-meter-sigma')
        wrong_route = args.copy()
        wrong_route[wrong_route.index('--dataset-id') + 1] = 'unlisted/pixel5'
        for variant in (missing_sigma, wrong_route):
            self.assertIn('requires raw H/LAX-T baseline frequency experiment recipe',
                          self.run_cli(variant + ['--native-tdcp-frequency-residual-states']))

    def test_main_cauchy_base_off_reaches_raw_ingress(self):
        args=self.args();args=args[:args.index('--native-paired-epoch-states')]
        self.assertIn('failed to open raw Android GNSS CSV', self.run_cli(args+['--native-main-p-cauchy']))

    def test_stationary_gyro_reaches_raw_ingress(self):
        args = self.args()
        args = args[:args.index('--native-paired-epoch-states')]
        self.assertIn('failed to open raw Android GNSS CSV', self.run_cli(
            args + ['--native-stationary-gyro-initializer']))

    def test_tdcp_code_gate_development_routes_reach_raw_ingress(self):
        args = self.args()
        args = args[:args.index('--native-paired-epoch-states')]
        for route in ('2021-08-24-20-32-us-ca-mtv-h/pixel5',
                      '2023-03-08-21-34-us-ca-mtv-u/pixel5',
                      '2022-04-01-18-22-us-ca-lax-t/pixel5'):
            variant = args.copy()
            variant[variant.index('--dataset-id') + 1] = route
            self.assertIn('failed to open raw Android GNSS CSV', self.run_cli(
                variant + ['--native-tdcp-no-code-jump-gate']))

    def test_tdcp_code_gate_rejects_base_and_gyro_mix(self):
        args = self.args()
        base_off = args[:args.index('--native-paired-epoch-states')]
        for variant in (args, base_off + ['--native-stationary-gyro-initializer']):
            self.assertIn('requires raw H/U/LAX-T all-epoch Phase171 base-off recipe', self.run_cli(
                variant + ['--native-tdcp-no-code-jump-gate']))

    def test_sparse_p_lax_t_baseline_and_variant_reach_ingress(self):
        args = self.args()
        args = args[:args.index('--native-paired-epoch-states')]
        args[args.index('--dataset-id') + 1] = '2022-04-01-18-22-us-ca-lax-t/pixel5'
        for extra in ([], ['--native-tdcp-no-code-jump-gate']):
            self.assertIn('failed to open raw Android GNSS CSV', self.run_cli(
                args + ['--native-sparse-p-staging'] + extra))

    def test_sparse_p_rejects_h(self):
        args = self.args()
        args = args[:args.index('--native-paired-epoch-states')]
        self.assertIn('requires raw LAX-T all-epoch Phase171 base-off recipe', self.run_cli(
            args + ['--native-sparse-p-staging']))

    def test_tdcp_code_gate_rejects_unlisted_route(self):
        args = self.args()
        args = args[:args.index('--native-paired-epoch-states')]
        args[args.index('--dataset-id') + 1] = 'unlisted/pixel5'
        self.assertNotIn('failed to open raw Android GNSS CSV', self.run_cli(
            args + ['--native-tdcp-no-code-jump-gate']))

    def test_stationary_gyro_rejects_base_mix(self):
        self.assertIn('requires raw H all-epoch Phase171 base-off Phase197 recipe', self.run_cli(
            self.args() + ['--native-stationary-gyro-initializer']))

    def test_stationary_gyro_requires_offset_and_rejects_cauchy(self):
        args = self.args()
        args = args[:args.index('--native-paired-epoch-states')]
        for variant in (
            [a for a in args if a != '--native-phase197-source-utc-fallback-imu-offset'],
            args + ['--native-main-p-cauchy'],
        ):
            self.assertIn('requires raw H all-epoch Phase171 base-off Phase197 recipe',
                          self.run_cli(variant + ['--native-stationary-gyro-initializer']))

    def test_main_cauchy_rejects_base_mix(self):
        self.assertIn('requires raw H all-epoch Phase171 base-off recipe',self.run_cli(
            self.args()+['--native-main-p-cauchy']))

    def test_c7_recipe_rejects_legacy_signal_bias_family(self):
        self.assertIn('requires the frozen Phase93 recipe without other optional candidate switches', self.run_cli(
            self.args()+['--native-signal-bias-states']))

    def test_gps_values_recipe_reaches_raw_ingress(self):
        self.assertIn('failed to open raw Android GNSS CSV', self.run_cli(
            self.args() + ['--native-dense-base-smoothing', '--native-base-gps-values-only-ablation']))

    def test_gps_values_requires_dense(self):
        self.assertIn('requires paired/dense', self.run_cli(
            self.args() + ['--native-base-gps-values-only-ablation']))

    def test_gps_center_requires_gps_values_mode(self):
        self.assertIn('requires --native-base-gps-values-only-ablation', self.run_cli(
            self.args()+['--native-dense-base-smoothing','--native-base-gps-center-ablation']))

    def test_gps_center_reaches_raw_ingress(self):
        self.assertIn('failed to open raw Android GNSS CSV', self.run_cli(self.args()+[
            '--native-dense-base-smoothing','--native-base-gps-values-only-ablation',
            '--native-base-gps-center-ablation']))

    def test_gps_values_rejects_mask_only(self):
        self.assertIn('rejects mask-only', self.run_cli(self.args() + [
            '--native-dense-base-smoothing', '--native-base-gps-values-only-ablation',
            '--native-base-mask-only-ablation']))

    def test_missing_mask_rejected_before_ingress(self):
        args = self.args()
        args.remove('--native-base-pseudorange-source-miss-mask')
        self.assertIn('--native-paired-epoch-states requires raw H', self.run_cli(args))

    def test_other_route_rejected_before_ingress(self):
        args = self.args()
        args[args.index('--dataset-id') + 1] = '2021-03-16-18-59-us-ca-mtv-a/pixel5'
        self.assertIn('--native-paired-epoch-states requires raw H', self.run_cli(args))

    def test_legacy_phase126_not_silently_reinterpreted(self):
        self.assertIn('--native-paired-epoch-states requires raw H', self.run_cli(
            self.args() + ['--native-phase126-raw-base-source-complete']))

    def test_mat_base_rejected_before_ingress(self):
        args = self.args()
        args[args.index('--native-base-rinex') + 1] = '/nonexistent/base.mat'
        self.assertIn('MATLAB .mat paths are forbidden', self.run_cli(args))

    def test_rover_only_ablation_reaches_raw_ingress(self):
        args = self.args()
        args = args[:args.index('--native-paired-epoch-states')]
        args.append('--native-rover-epoch-states')
        self.assertIn('failed to open raw Android GNSS CSV', self.run_cli(args))

    def test_rover_only_rejects_paired_base_mix(self):
        self.assertIn('--native-rover-epoch-states requires raw H', self.run_cli(
            self.args() + ['--native-rover-epoch-states']))

    def test_dense_paired_recipe_reaches_raw_ingress(self):
        self.assertIn('failed to open raw Android GNSS CSV', self.run_cli(
            self.args() + ['--native-dense-base-smoothing']))

    def test_dense_requires_paired_recipe(self):
        args = self.args()
        args.remove('--native-paired-epoch-states')
        self.assertIn('--native-dense-base-smoothing requires', self.run_cli(
            args + ['--native-dense-base-smoothing']))

    def test_mask_only_requires_dense_pair(self):
        self.assertIn('--native-base-mask-only-ablation requires', self.run_cli(
            self.args() + ['--native-base-mask-only-ablation']))

    def test_mask_only_dense_pair_reaches_raw_ingress(self):
        self.assertIn('failed to open raw Android GNSS CSV', self.run_cli(
            self.args() + ['--native-dense-base-smoothing', '--native-base-mask-only-ablation']))


if __name__ == '__main__':
    unittest.main()
