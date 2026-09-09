"""Evaluation-only position response to NHC; no truth or inference input."""
import json
import math
from compare_lambda_floor_outputs import ROOT, read_output, separation, digest

def main():
    specs = [
        ('phase479-h-robust-diagnostic-v1', '4a0c4ef8822c72aa9134368cd57fe769817895e63b5dbe925783a9c8d091fa8e'),
        ('phase488-h-batch-nhc-v1', '0bbc0d1b4bdc18ee29972e74ca9b5d9666b766b361b9cf8c600b762e584098eb')]
    outputs = []
    for name, sha in specs:
        path = ROOT / 'output/smartphone-r5' / name / 'mtv-h/opaque_solution_output.csv'
        assert digest(path) == sha
        outputs.append(read_output(path))
    a, b = outputs
    assert a.keys() == b.keys() and len(a) == 3139
    distances = sorted(separation(a[key], b[key]) for key in a)
    def percentile(p):
        index = (len(distances) - 1) * p
        lo, hi = math.floor(index), math.ceil(index)
        return distances[lo] + (distances[hi] - distances[lo]) * (index - lo)
    result = dict(phase=490, rows=len(a), exact_keys=True,
        separation_m=dict(mean=sum(distances)/len(distances), p50=percentile(.5),
                          p95=percentile(.95), maximum=distances[-1]),
        truth_reads=0, accuracy_evaluations=0, inference_input=False)
    path = ROOT / 'docs/use_cases/records/smartphone_r5_phase490_nhc_output_response_v1.json'
    with path.open('x') as handle:
        json.dump(result, handle, indent=2)
    print(json.dumps(result, indent=2))

if __name__ == '__main__':
    main()
