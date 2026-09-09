#!/usr/bin/env python3
"""Success against clearance, with the matched pairs compared directly.

Clearance is (2*open_position - side)/2, so the same value is reachable with
different objects. The point of the batch is those matched pairs: if clearance
governs, a 50mm cube at open 0.040 should behave like a 30mm cube at 0.030
despite the objects differing by 20mm.

Usage: analyse_clearance.py <batch dir>
"""
import collections
import glob
import json
import math
import os
import re
import sys

ID = re.compile(r"cl_c(\d+)_o(\d+)_y(\d+)_(\d+)$")

# A trial can fail before the jaws ever reach the object, and those failures
# say nothing about clearance. Leaving them in the denominator let one planning
# failure and one detector timeout drag a cell below its neighbour for reasons
# unrelated to the hypothesis, so both rates are reported: overall, and among
# trials that actually got to attempt a grasp.
PIPELINE = {"planning_failure", "detector_timeout", "gripper_command_failure"}


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
        m = ID.match(rec["trial_id"])
        if not m:
            continue
        side = int(m.group(1))
        opening = float("0." + m.group(2))
        bias = int(m.group(3))
        clearance = (2 * opening * 1000 - side) / 2.0
        cells[(clearance, side, opening, bias)].append(rec["category"])

    print("%9s %6s %7s %6s %6s %19s %19s  %s" % (
        "clear mm", "cube", "open", "bias", "n", "success all",
        "success reached", "other"))
    stats = {}
    for key in sorted(cells):
        clearance, side, opening, bias = key
        out = cells[key]
        k = sum(1 for c in out if c == "success")
        reached = [c for c in out if c not in PIPELINE]
        kr = sum(1 for c in reached if c == "success")
        lo, hi = wilson(k, len(out))
        rlo, rhi = wilson(kr, len(reached))
        stats[key] = (kr, len(reached), rlo, rhi)
        other = ", ".join("%s:%d" % kv for kv in
                          collections.Counter(c for c in out if c != "success").items())
        print("%9.1f %5dmm %7.3f %5dmm %6d %6s [%3.0f%%,%3.0f%%] %6s [%3.0f%%,%3.0f%%]  %s" % (
            clearance, side, opening, bias, len(out),
            "%d/%d" % (k, len(out)), lo * 100, hi * 100,
            "%d/%d" % (kr, len(reached)), rlo * 100, rhi * 100, other))

    print("\n同余隙、不同物体的配对：")
    by_pair = collections.defaultdict(list)
    for (clearance, side, opening, bias) in stats:
        by_pair[(clearance, bias)].append((side, opening))
    found = False
    for (clearance, bias), members in sorted(by_pair.items()):
        if len(members) < 2:
            continue
        found = True
        print("  余隙 %.1fmm, 偏置 %dmm:" % (clearance, bias))
        rates = []
        for side, opening in sorted(members):
            k, n, lo, hi = stats[(clearance, side, opening, bias)]
            if n == 0:
                print("    cube %2dmm open %.3f -> 无有效试验（全部流水线故障）"
                      % (side, opening))
                continue
            rates.append((k / n, lo, hi))
            print("    cube %2dmm open %.3f -> %d/%d [%4.0f%%,%4.0f%%]" % (
                side, opening, k, n, lo * 100, hi * 100))
        if len(rates) < 2:
            print("    -> 可比格点不足，无法判断")
            continue
        overlap = max(r[1] for r in rates) <= min(r[2] for r in rates)
        print("    -> %s" % (
            "区间重叠：与「余隙主导」一致" if overlap else
            "区间不重叠：余隙不足以解释，物体尺寸另有影响"))
    if not found:
        print("  （本批次没有可配对的格点）")


if __name__ == "__main__":
    main()
