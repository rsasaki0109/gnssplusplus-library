"""One-shot, frozen H development evaluation; never invoke a native solver."""
import argparse
import hashlib
import importlib.util
import json
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
RECORDS = ROOT / 'docs/use_cases/records'
MANIFEST = RECORDS / 'smartphone_r5_phase277_h_accuracy_manifest_v1.json'
AUTH = RECORDS / 'smartphone_r5_phase277_h_accuracy_authorization_v1.json'
RESULT = RECORDS / 'smartphone_r5_phase277_h_accuracy_result_v1.json'
ROUTE = '2021-08-24-20-32-us-ca-mtv-h/pixel5'
BASELINE = 1.0769392017393964


def require(condition, message):
    if not condition:
        raise ValueError(message)


def static_bytes(path):
    require(path.suffix in {'.json', '.py'}, 'non-metadata access forbidden')
    return path.read_bytes()


def digest(data):
    return hashlib.sha256(data).hexdigest()


def read(path):
    def unique(pairs):
        result = {}
        for key, value in pairs:
            require(key not in result, 'duplicate JSON key')
            result[key] = value
        return result
    return json.loads(static_bytes(path), object_pairs_hook=unique)


def verify():
    m = read(MANIFEST)
    require(m['phase'] == 277 and m['route'] == ROUTE, 'wrong experiment')
    require(m['baseline_m'] == BASELINE, 'baseline changed')
    require(m['role'] == 'development/train; not heldout or leaderboard proof', 'role changed')
    for name, pin in m['authority'].items():
        path = Path(pin['path'])
        require(not path.is_absolute() and '..' not in path.parts, 'unsafe authority path')
        require(digest(static_bytes(ROOT / path)) == pin['sha256'], f'pin mismatch: {name}')
    a = m['authority']
    require(a['evaluator']['path'] == str(Path(__file__).resolve().relative_to(ROOT)), 'wrong evaluator')
    old = read(ROOT / a['phase203_manifest']['path'])
    structural = read(ROOT / a['phase276_result']['path'])
    require(structural['execution']['return_code'] == 0, 'native run failed')
    require(structural['accuracy_and_payload_policy']['truth_used'] is False, 'native truth boundary')
    require(structural['graph']['converged'] is True, 'main did not converge')
    for stage in ('main', 'gnss_first'):
        termination = structural['termination'][stage]
        require(all(termination[k] is True for k in
                    ('costs_finite', 'no_fallback', 'termination_trace_complete',
                     'configuration_valid')), 'invalid termination evidence')
    handoff = structural['recipe_and_handoff']
    require(handoff['converged'] is True and
            handoff['epoch_identity_alignment_valid'] is True and
            handoff['handoff_mode'] == 'gnss-first-in-memory-meter-clock-state',
            'invalid same-run handoff')
    keys = structural['raw_utc_key_contract']
    require(keys['exact_solution_epochs'] == 3139 and
            keys['target_epochs'] == 3139 and
            all(keys[k] == 0 for k in
                ('interpolated_epochs', 'edge_hold_epochs', 'unresolved_epochs')),
            'non-exact output alignment')
    require(structural['graph']['phase217_main_motion'] == {'requested': True, 'enabled': True, 'factors': 3139, 'gap_skips': 0, 'sigma_m': 0.05, 'gap_threshold_s': 1.5}, 'main motion counts changed')
    require(structural['graph']['phase213_main_doppler'] == {'requested': True, 'enabled': True, 'factors': 66685}, 'main Doppler counts changed')
    require(all(structural['comparison_vs_phase234'][k] for k in ('gnss_first', 'epochs', 'imu_initialization', 'imu_measurement_noise', 'imu_utc_fallback_offset', 'output_contract', 'raw_utc_key_contract')), 'baseline composition changed')
    require(structural['tdcp_contract']['source_tdcp_meter_sigma_requested'] and structural['tdcp_contract']['official_huber_k'] == 4 and structural['tdcp_contract']['fixed_sigma_m'] is None, 'source TDCP contract changed')
    require(structural['tdcp_contract']['source_tdcp_resl_observable_requested'] is False, 'source resL changed')
    tdcp = structural['tdcp_contract']
    require(tdcp['tdcp_only_affine_geometry_requested'] is False, 'affine geometry unexpectedly enabled')
    require(tdcp['tdcp_only_affine_factors_inserted'] == 0 and
            tdcp['gnss_first_tdcp_only_affine_factors_inserted'] == 0,
            'affine insertion coverage changed')
    require(tdcp['epoch_heading_attitude_seeds_requested'] is False and
            tdcp['epoch_heading_attitude_seeds_inserted'] == 0,
            'heading insertion coverage changed')
    require(tdcp['omit_first_imu_bias_prior_requested'] is False and
            tdcp['first_imu_bias_priors_inserted'] == 1 and
            tdcp['first_imu_bias_priors_omitted'] == 0, 'bias prior omission changed')
    require(tdcp['omit_first_imu_velocity_prior_requested'] is False and
            tdcp['first_imu_velocity_priors_inserted'] == 1 and
            tdcp['first_imu_velocity_priors_omitted'] == 0, 'velocity prior omission changed')
    require(tdcp['relative_height_pairs_requested'] is True and
            tdcp['relative_height_pairs_selected'] == 7266 and
            tdcp['relative_height_factors_inserted'] == 7266, 'relative height coverage changed')
    require(tdcp['relative_height_pair_convention'] == 'source-cumulative-speed-samples' and
            tdcp['relative_height_sigma_m'] == 0.1 and tdcp['relative_height_huber_k'] == 0.5 and
            tdcp['relative_height_proximity_m'] == 15 and
            tdcp['relative_height_speed_sample_sum_threshold'] == 100 and
            tdcp['relative_height_no_external_reference'] is True, 'relative height recipe changed')
    require(structural['graph']['factors'] == 259650, 'graph factor count changed')
    require(m['truth'] == old['truth'], 'truth metadata changed')
    require(m['metric_contract'] == old['metric_contract'], 'metric changed')
    candidate = structural['candidate_identity']
    require(m['candidate'] == {k: candidate[k] for k in ('path', 'sha256', 'bytes')} | {'rows': 3139}, 'candidate changed')
    require(candidate['published_rows'] == 3139, 'candidate coverage changed')
    for name in ('evaluator', 'phase199_evaluator', 'phase196_thin_wrapper',
                 'phase189_kernel', 'phase76_parser', 'phase74_metric_helper'):
        target = 'phase203_evaluator' if name == 'evaluator' else name
        require(a[target]['path'] == old['authority'][name]['path'] and
                a[target]['sha256'] == old['authority'][name]['sha256'], 'metric dependency changed')
    return m


def evaluate():
    m = verify()
    auth = read(AUTH)
    require(auth == {'phase': 277, 'allow_truth_read': True,
                     'manifest_sha256': digest(static_bytes(MANIFEST)),
                     'candidate_sha256': m['candidate']['sha256'],
                     'truth_sha256': m['truth']['sha256']}, 'authorization mismatch')
    require(not RESULT.exists(), 'refusing result overwrite')
    # Exclusive claim also prevents a second score after a failed attempt.
    claim = RESULT.with_suffix('.attempt.json')
    with claim.open('x') as f:
        json.dump(auth, f, indent=2)
    path = ROOT / m['authority']['phase203_evaluator']['path']
    spec = importlib.util.spec_from_file_location('phase203_for_277', path)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    score = module.score_payloads(ROOT / m['candidate']['path'], m['candidate'],
                                  ROOT / m['truth']['path'], m['truth'],
                                  route=ROUTE, result_path=RESULT)
    value = score['metric']['route_score_m']
    result = {'phase': 277, 'execution_label': 'primary agent',
              'manifest_sha256': auth['manifest_sha256'], 'role': m['role'],
              'baseline_m': BASELINE, 'delta_m': value - BASELINE,
              'improved_on_h': value < BASELINE, **score}
    with RESULT.open('x') as f:
        json.dump(result, f, indent=2, allow_nan=False)
    return result


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--evaluate', action='store_true')
    args = parser.parse_args()
    print(json.dumps(evaluate() if args.evaluate else verify(), indent=2))
