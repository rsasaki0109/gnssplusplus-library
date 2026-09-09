"""Structural checks for frozen main-only readmission; never read truth."""
import json
import re
from run_phase486_nhc_monitor import ROOT, digest
from verify_phase479_lambda_floor import check_stage

def main():
    directory=ROOT/'output/smartphone-r5/phase507-h-code-monitor-v1/mtv-h'
    manifest=json.loads((directory/'manifest.json').read_text())
    done=json.loads((directory/'completed.json').read_text())
    assert done['return_code']==0
    for name, sha in manifest['source_pins'].items():
        assert digest(ROOT/name)==sha, name
    assert digest(ROOT/manifest['argv'][0])==manifest['binary_sha256']
    assert digest(directory/'opaque_solution_output.csv')==done['output_sha256']
    s=json.loads((directory/'native_summary.json').read_text())
    baseline=json.loads((ROOT/'output/smartphone-r5/phase505-h-code-monitor-v1/mtv-h/native_summary.json').read_text())
    assert s['gnss_first']==baseline['gnss_first']
    assert s['graph']['converged']
    assert s['graph']['factors']==baseline['graph']['factors']+243
    for key,value in baseline['graph'].items():
        if key not in ('factors','iterations','initial_cost','final_cost'):
            assert s['graph'][key]==value, key
    for stage in ('main','gnss_first'):
        check_stage(s['phase143_termination'][stage],stage)
    assert done['monitor']==['[native-main-code-edge-readmission] added=243 gnss_first_changed=0 median_changed=0']
    logs=(directory/'stderr.log').read_text()
    assert re.findall(r'^\[native-code-edge-shadow\].*$',logs,re.M)==[
        '[native-code-edge-shadow] geometry_quality_pass=420 residual_pass=243 retained_epoch_pass=243 factors_added=0']
    print(json.dumps(dict(verified=True, extra_factors=243, gnss_first_unchanged=True,
                         output_sha256=done['output_sha256'],
                         iterations=s['graph']['iterations'], truth_reads=0,
                         accuracy_evaluations=0),indent=2))

if __name__=='__main__': main()
