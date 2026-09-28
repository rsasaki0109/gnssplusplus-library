"""Read-only ADR/Doppler consistency diagnostic; does not select solver rows."""
import argparse
import csv
import hashlib
import json
import math
from pathlib import Path


def audit(path):
    previous = {}
    epochs = {}
    pairs = []
    with path.open(newline='', encoding='utf-8-sig') as stream:
        for row in csv.DictReader(stream):
            utc = int(row['utcTimeMillis'])
            epochs.setdefault(utc, len(epochs))
            key = (row['ConstellationType'], row['Svid'], row['SignalType'])
            old = previous.get(key)
            previous[key] = row
            if old is None:
                continue
            dt = (utc - int(old['utcTimeMillis'])) / 1000
            first = int(old['AccumulatedDeltaRangeState'])
            last = int(row['AccumulatedDeltaRangeState'])
            if not (0 < dt < 1.5 and first & 1 and last & 1) or (first | last) & 6:
                continue
            adr = float(row['AccumulatedDeltaRangeMeters']) - float(old['AccumulatedDeltaRangeMeters'])
            integrated_doppler = dt * 0.5 * (
                float(row['PseudorangeRateMetersPerSecond']) +
                float(old['PseudorangeRateMetersPerSecond']))
            error = adr - integrated_doppler
            if math.isfinite(error):
                pairs.append(dict(epoch=epochs[utc], utc_ms=utc,
                                  constellation=key[0], svid=key[1], signal=key[2],
                                  adr_minus_integrated_doppler_m=error))
    pairs.sort(key=lambda item: abs(item['adr_minus_integrated_doppler_m']), reverse=True)
    with path.open('rb') as stream:
        digest = hashlib.file_digest(stream, 'sha256').hexdigest()
    return dict(input_sha256=digest, raw_epochs=len(epochs), valid_pairs=len(pairs),
                selection='same constellation/SVID/signal; 0 < dt < 1.5 s; both ADR valid, neither reset nor cycle-slip',
                scope='Raw consistency only; no native quality, base, navigation or factor-admission masks. No truth input.',
                inference_changes=False, cause_established=False, largest_pairs=pairs[:10])


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('input', type=Path)
    parser.add_argument('--out', type=Path, required=True)
    args = parser.parse_args()
    result = audit(args.input)
    args.out.write_text(json.dumps(result, indent=2) + '\n', encoding='utf-8')
    print(json.dumps({key: result[key] for key in ['raw_epochs', 'valid_pairs', 'largest_pairs']}))


if __name__ == '__main__':
    main()
