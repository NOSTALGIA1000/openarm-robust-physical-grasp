#!/usr/bin/env python3
"""Does dropping rel_offset cost anything, and from which clearance?

The ablation found it costs nothing at 25mm of clearance. The hypothesis is
that self-centering absorbs the constant while there is room for it, which
means the cost should appear as clearance shrinks. Each clearance is a paired
comparison of the same two arms, so the only thing differing is the constant.

Usage: analyse_offsetneed.py <batch dir>
"""
import collections
import glob
import json
import math
import os
import re
import sys

ID = re.compile(r"ro_c(\d+)_(on|off)_y(\d+)_(\d+)$")


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
    if len(sys.argv) != 2:
        sys.exit(__doc__)
    cells = collections.defaultdict(list)
    for path in sorted(glob.glob(os.path.join(sys.argv[1], "*", "trial.json"))):
        rec = json.loads(open(path).read())
        m = ID.match(rec["trial_id"])
        if m:
            cells[(int(m.group(1)), m.group(2))].append(rec["category"])

    print("%8s %6s %5s %20s  %s" % ("余隙", "offset", "n", "success", "other"))
    stats = {}
    for key in sorted(cells):
        clr, arm = key
        out = cells[key]
        k = sum(1 for c in out if c == "success")
        lo, hi = wilson(k, len(out))
        stats[key] = (k, len(out))
        other = ", ".join("%s:%d" % kv for kv in
                          collections.Counter(c for c in out
                                              if c != "success").items())
        print("%6dmm %6s %5d %6s [%4.0f%%,%4.0f%%]  %s" % (
            clr, arm, len(out), "%d/%d" % (k, len(out)),
            lo * 100, hi * 100, other))

    print("\n每档余隙下「去掉 offset」的代价（负号 = 去掉后更差）：")
    for clr in sorted({c for c, _ in cells}):
        if (clr, "on") not in stats or (clr, "off") not in stats:
            continue
        k1, n1 = stats[(clr, "off")]
        k0, n0 = stats[(clr, "on")]
        d, lo, hi = newcombe(k1, n1, k0, n0)
        verdict = ("offset **必需**" if hi < 0 else
                   "去掉后更好（意外，需查）" if lo > 0 else
                   "代价不可与 0 分辨——该余隙下 offset 未被测出必要性")
        print("  余隙 %2dmm: %+6.1f%%  CI [%+6.1f%%, %+6.1f%%]  %s"
              % (clr, d * 100, lo * 100, hi * 100, verdict))


if __name__ == "__main__":
    main()
