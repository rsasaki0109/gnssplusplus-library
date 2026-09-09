"""Fail-closed raw timing preflight for the experimental paired TDCP lane.

No coordinates or enriched measurements are inspected. This is validation,
not an inference input. Intended to run after input hash verification.
"""
import argparse
import csv
from decimal import Decimal, InvalidOperation
import json
from pathlib import Path


def validate_rows(rows):
    clocks = {}
    count = 0
    for row in rows:
        if row['MessageType'] != 'Raw' or row['ConstellationType'] not in ('1', '6'):
            continue
        count += 1
        text = row['TimeOffsetNanos'].strip()
        try:
            offset = Decimal(text)
            bias = Decimal(row['BiasNanos'].strip() or '0')
        except InvalidOperation as error:
            raise ValueError('Missing or invalid raw timing value') from error
        if not offset.is_finite() or offset != 0 or not bias.is_finite():
            raise ValueError('Paired TDCP experiment requires zero finite raw time offsets')
        identity = (int(row['utcTimeMillis']), int(row['ConstellationType']), int(row['Svid']))
        clock = (int(row['TimeNanos']), int(row['FullBiasNanos']), bias,
                 int(row['HardwareClockDiscontinuityCount']))
        if identity in clocks and clocks[identity] != clock:
            raise ValueError('Cross-signal raw clock mismatch')
        clocks[identity] = clock
    if count == 0:
        raise ValueError('No in-scope raw GPS/Galileo rows')
    return {'validated_rows': count, 'satellite_epoch_keys': len(clocks),
            'all_offsets_zero': True, 'cross_signal_clocks_consistent': True}


def validate(path):
    with path.open(newline='') as stream:
        return validate_rows(csv.DictReader(stream))


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('raw_gnss', type=Path)
    print(json.dumps(validate(parser.parse_args().raw_gnss), sort_keys=True))
