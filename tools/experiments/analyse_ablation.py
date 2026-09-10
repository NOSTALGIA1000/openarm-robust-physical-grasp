#!/usr/bin/env python3
"""Each ablation against the full pipeline, as a difference with an interval.

The repository has one ablation on record and it is n=2. This asks, for each
component dropped, whether success changes by an amount distinguishable from
zero - not whether two intervals happen to overlap, which at these sample
sizes almost nothing fails.

Usage: analyse_ablation.py <batch dir>
"""
import collections
import glob
import json
import math
import os
import re
import sys

ID = re.compile(r"ab_([a-z]+)_y(\d+)_(\d+)$")
LABEL = {
    "full": "完整流水线（对照）",
    "norehover": "去 re-hover（修正后直接下降）",
    "nooffset": "去 rel_offset（不减表面-原点项）",
    "fixedroi": "固定 30px 窗口（不用级联）",
}


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
    arms = collections.defaultdict(list)
    for path in sorted(glob.glob(os.path.join(sys.argv[1], "*", "trial.json"))):
        rec = json.loads(open(path).read())
        m = ID.match(rec["trial_id"])
        if m:
            arms[m.group(1)].append(rec["category"])

    print("%-34s %5s %20s  %s" % ("arm", "n", "success", "other"))
    stats = {}
    for arm in ("full", "norehover", "nooffset", "fixedroi"):
        out = arms.get(arm)
        if not out:
            continue
        k = sum(1 for c in out if c == "success")
        lo, hi = wilson(k, len(out))
        stats[arm] = (k, len(out))
        other = ", ".join("%s:%d" % kv for kv in
                          collections.Counter(c for c in out
                                              if c != "success").items())
        print("%-34s %5d %6s [%4.0f%%,%4.0f%%]  %s" % (
            LABEL.get(arm, arm), len(out), "%d/%d" % (k, len(out)),
            lo * 100, hi * 100, other))

    if "full" not in stats:
        print("\n无对照臂，无法比较")
        return
    k0, n0 = stats["full"]
    print("\n相对完整流水线的差值（负号 = 去掉该部件后更差）：")
    for arm in ("norehover", "nooffset", "fixedroi"):
        if arm not in stats:
            continue
        k, n = stats[arm]
        d, lo, hi = newcombe(k, n, k0, n0)
        verdict = ("该部件**有**可测的贡献" if hi < 0 else
                   "去掉后**更好**，该部件有害" if lo > 0 else
                   "差值不可与 0 分辨——本条件下该部件的贡献未被测出")
        print("  %-32s %+6.1f%%  CI [%+6.1f%%, %+6.1f%%]  %s"
              % (LABEL.get(arm, arm), d * 100, lo * 100, hi * 100, verdict))


if __name__ == "__main__":
    main()
