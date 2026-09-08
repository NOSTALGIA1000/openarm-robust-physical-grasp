#!/usr/bin/env python3
"""Success rates with Wilson intervals for the paired ceiling batch.

The question is whether the radial correction moves the cliff or only the
precision, so this reports outcomes rather than residuals - and it reports
intervals, because every ROS2-phase result so far has quoted bare fractions
on cells of two to four trials. See docs/statistics.md: at n=12 an all-success
cell only supports "at least 74%".

Usage: analyse_ceiling.py <batch dir>
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
    centre = (p + z * z / (2 * n)) / d
    half = z * math.sqrt(p * (1 - p) / n + z * z / (4 * n * n)) / d
    return max(0.0, centre - half), min(1.0, centre + half)


def main():
    if len(sys.argv) != 2:
        sys.exit(__doc__)
    cells = collections.defaultdict(list)
    resid = collections.defaultdict(list)
    for path in sorted(glob.glob(os.path.join(sys.argv[1], "*", "trial.json"))):
        rec = json.loads(open(path).read())
        parts = rec["trial_id"].split("_")     # cl_<arm>_y<bias>_<rep>
        key = (parts[1], int(parts[2][1:]))
        cells[key].append(rec["category"])
        m = rec.get("measurements") or {}
        if m.get("grip_cam_z", 0.0) >= 0.66:
            resid[key].append((m["residual_x"], m["residual_y"]))

    for arm in sorted({a for a, _ in cells}):
        print("\n=== %s ===" % arm)
        print("%6s %4s %8s %20s %9s %9s  %s" % (
            "bias", "n", "success", "Wilson 95%", "res_x mm", "res_y mm",
            "other outcomes"))
        for bias in sorted(b for a, b in cells if a == arm):
            outcomes = cells[(arm, bias)]
            n = len(outcomes)
            k = sum(1 for c in outcomes if c == "success")
            lo, hi = wilson(k, n)
            r = resid[(arm, bias)]
            rx = sum(v[0] for v in r) / len(r) * 1000.0 if r else float("nan")
            ry = sum(v[1] for v in r) / len(r) * 1000.0 if r else float("nan")
            other = ", ".join(
                "%s:%d" % (c, v)
                for c, v in collections.Counter(outcomes).items()
                if c != "success")
            print("%6d %4d %3d/%-4d [%5.1f%%, %5.1f%%] %9.2f %9.2f  %s" % (
                bias, n, k, n, lo * 100, hi * 100, rx, ry, other))


if __name__ == "__main__":
    main()
