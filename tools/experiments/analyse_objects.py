#!/usr/bin/env python3
"""Success by object width and bias, with Wilson intervals.

Tests two predictions at once. Clearance falls as width grows, so success at
small bias should fall with it; and the 283mm field-of-view wall should be
untouched by object geometry, since it is the gripper leaving the frame.

Usage: analyse_objects.py <batch dir>
"""
import collections
import glob
import json
import math
import os
import sys


def wilson(k, n, z=1.96):
    if n == 0:
        return 0.0, 1.0
    p = k / n
    d = 1.0 + z * z / n
    c = (p + z * z / (2 * n)) / d
    h = z * math.sqrt(p * (1 - p) / n + z * z / (4 * n * n)) / d
    return max(0.0, c - h), min(1.0, c + h)


def main():
    if len(sys.argv) != 2:
        sys.exit(__doc__)
    cells = collections.defaultdict(list)
    for path in sorted(glob.glob(os.path.join(sys.argv[1], "*", "trial.json"))):
        rec = json.loads(open(path).read())
        parts = rec["trial_id"].split("_")      # ob_w<width>_y<bias>_<rep>
        cells[(int(parts[1][1:]), int(parts[2][1:]))].append(rec["category"])

    widths = sorted({w for w, _ in cells})
    biases = sorted({b for _, b in cells})
    print("%8s %9s" % ("", ""), end="")
    for b in biases:
        print("%22s" % ("bias %dmm" % b), end="")
    print()
    for w in widths:
        print("%5dmm %6dmm/side" % (w, (60 - w) // 2), end="")
        for b in biases:
            out = cells.get((w, b), [])
            if not out:
                print("%22s" % "-", end="")
                continue
            k = sum(1 for c in out if c == "success")
            lo, hi = wilson(k, len(out))
            print("%9s [%4.0f%%,%4.0f%%]" % (
                "%d/%d" % (k, len(out)), lo * 100, hi * 100), end="")
        print()

    print("\n失败形态：")
    for w in widths:
        for b in biases:
            out = cells.get((w, b), [])
            other = collections.Counter(c for c in out if c != "success")
            if other:
                print("  w%dmm y%dmm: %s" % (
                    w, b, ", ".join("%s:%d" % kv for kv in other.items())))


if __name__ == "__main__":
    main()
