"""decimate_rinex.py in.obs out.obs : keep RINEX 3 epochs on integer seconds.

LLI bit 0 seen on a dropped epoch is carried into the next kept epoch for that
satellite/observation slot, and a slot that was missing on any dropped epoch
since the last kept one is flagged too, so the decimated file cannot hide a
cycle slip.
"""
import sys

src, dst = sys.argv[1], sys.argv[2]
with open(src) as f:
    lines = f.read().split('\n')
i = 0
out = []
while i < len(lines):
    out.append(lines[i])
    if 'END OF HEADER' in lines[i]:
        i += 1
        break
    i += 1

pending = {}  # (sat, slot) -> flag
last_kept = {}  # sat -> set of slots present at the last kept epoch
kept = dropped = 0
while i < len(lines):
    line = lines[i]
    if not line.startswith('>'):
        i += 1
        continue
    n = int(line[32:35])
    flag = int(line[31])
    block = lines[i + 1:i + 1 + n]
    i += 1 + n
    if flag > 1:  # event records pass through unchanged
        out.append(line)
        out.extend(block)
        continue
    sec = float(line[19:29])
    keep = abs(sec - round(sec)) < 1e-6
    present = {}
    for b in block:
        sat = b[:3]
        slots = set()
        for k in range((len(b) - 3 + 15) // 16):
            field = b[3 + 16 * k:3 + 16 * (k + 1)]
            if field[:14].strip():
                slots.add(k)
        present[sat] = (b, slots)
    if not keep:
        dropped += 1
        for sat, (b, slots) in present.items():
            for k in slots:
                field = b[3 + 16 * k:3 + 16 * (k + 1)]
                lli = field[14:15].strip()
                if lli and int(lli) & 1:
                    pending[(sat, k)] = True
            for k in last_kept.get(sat, set()) - slots:
                pending[(sat, k)] = True
        for sat in set(last_kept) - set(present):
            for k in last_kept[sat]:
                pending[(sat, k)] = True
        continue
    kept += 1
    out.append(line)
    for sat, (b, slots) in present.items():
        b = b.ljust(3 + 16 * max(slots, default=-1) + 16)
        chars = list(b)
        for k in slots:
            if pending.pop((sat, k), False):
                pos = 3 + 16 * k + 14
                cur = chars[pos].strip()
                chars[pos] = str((int(cur) if cur else 0) | 1)
        out.append(''.join(chars).rstrip())
    last_kept = {sat: slots for sat, (b, slots) in present.items()}
with open(dst, 'w') as f:
    f.write('\n'.join(out))
print('kept', kept, 'dropped', dropped)
