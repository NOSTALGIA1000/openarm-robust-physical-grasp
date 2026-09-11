#!/usr/bin/env python3
"""Same clearance, three object sizes, across batches.

stage47_pairs compared 30mm against 50mm at matched clearance and found three
pairs of four indistinguishable. stage52_cube20 adds a third size. Pooling them
tests the clearance hypothesis over a 30mm span of edge length rather than 20.

The batches ran on different days but the same code and the same derived-value
rules, and the trial records carry a config snapshot, so the comparison is
checked rather than assumed: any cell whose lift command, bias or offsets
differ from its partner is reported and excluded.

Usage: analyse_crosssize.py <batch dir> [<batch dir> ...]
"""
import collections
import glob
import json
import math
import os
import re
import sys

ID = re.compile(r"(?:cl|ob)_c(\d+)_o(\d+)_y(\d+)_(\d+)$")
KEYS = ("lift_command_m", "rel_offset_x", "rel_offset_y",
        "open_position", "close_position", "detector_z_bias")


def wilson(k, n, z=1.96):
    if n == 0:
        return 0.0, 1.0
    p = k / n
    d = 1.0 + z * z / n
    c = (p + z * z / (2 * n)) / d
    h = z * math.sqrt(p * (1 - p) / n + z * z / (4 * n * n)) / d
    return max(0.0, c - h), min(1.0, c + h)


def newcombe(k1, n1, k2, n2, z=1.96):
    p1, p2 = k1 / n1, k2 / n2
    l1, u1 = wilson(k1, n1, z)
    l2, u2 = wilson(k2, n2, z)
    d = p1 - p2
    return (d,
            d - math.sqrt((p1 - l1) ** 2 + (u2 - p2) ** 2),
            d + math.sqrt((u1 - p1) ** 2 + (p2 - l2) ** 2))


def main():
    if len(sys.argv) < 2:
        sys.exit(__doc__)
    cells = collections.defaultdict(list)
    cfgs = {}
    for root in sys.argv[1:]:
        for path in sorted(glob.glob(os.path.join(root, "*", "trial.json"))):
            rec = json.loads(open(path).read())
            m = ID.match(rec["trial_id"])
            if not m:
                continue
            side = int(m.group(1))
            opening = float("0." + m.group(2))
            bias = int(m.group(3))
            clearance = round((2 * opening * 1000 - side) / 2.0, 1)
            key = (clearance, bias, side)
            cells[key].append(rec["category"])
            c = rec.get("config") or {}
            cfgs.setdefault(key, {k: c.get(k) for k in KEYS})

    print("%9s %6s %6s %6s %20s  %s" % (
        "clear mm", "bias", "cube", "n", "success", "other"))
    stats = {}
    for key in sorted(cells):
        clearance, bias, side = key
        out = cells[key]
        k = sum(1 for c in out if c == "success")
        lo, hi = wilson(k, len(out))
        stats[key] = (k, len(out))
        other = ", ".join("%s:%d" % kv for kv in
                          collections.Counter(c for c in out
                                              if c != "success").items())
        print("%9.1f %5dmm %5dmm %6d %6s [%4.0f%%,%4.0f%%]  %s" % (
            clearance, bias, side, len(out), "%d/%d" % (k, len(out)),
            lo * 100, hi * 100, other))

    print("\n同余隙、同偏置、不同边长的配对：")
    groups = collections.defaultdict(list)
    for (clearance, bias, side) in stats:
        groups[(clearance, bias)].append(side)
    found = False
    for (clearance, bias), sides in sorted(groups.items()):
        if len(sides) < 2:
            continue
        found = True
        print("  余隙 %.1fmm, 偏置 %dmm:" % (clearance, bias))
        for a, b in [(sides[i], sides[j])
                     for i in range(len(sides)) for j in range(i + 1, len(sides))]:
            ka, na = stats[(clearance, bias, a)]
            kb, nb = stats[(clearance, bias, b)]
            ca, cb = cfgs[(clearance, bias, a)], cfgs[(clearance, bias, b)]
            differs = [k for k in ("lift_command_m", "rel_offset_x", "rel_offset_y")
                       if ca.get(k) != cb.get(k)]
            if differs:
                print("    %2dmm vs %2dmm -> 跳过：配置不同 (%s)"
                      % (a, b, ", ".join(differs)))
                continue
            d, lo, hi = newcombe(ka, na, kb, nb)
            verdict = ("区间跨 0：不可分辨，与「余隙主导」一致"
                       if lo <= 0 <= hi else "区间不跨 0：边长另有影响")
            print("    %2dmm (%d/%d) vs %2dmm (%d/%d): %+6.1f%% CI [%+6.1f%%, %+6.1f%%]  %s"
                  % (a, ka, na, b, kb, nb, d * 100, lo * 100, hi * 100, verdict))
    if not found:
        print("  （没有可配对的格点）")


if __name__ == "__main__":
    main()
