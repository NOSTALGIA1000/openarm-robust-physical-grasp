#!/usr/bin/env python3
"""Success against injected depth noise, with the difference from the noiseless
control given as an interval.

The question is not whether noise hurts - it is whether it hurts by an amount
that matters at the 2-3mm a real depth camera delivers. Each level is compared
against the zero-noise cell at the same bias, so the comparison is paired on
everything except the noise.

The residual column is reported unfiltered, and with its own n.

The first version of this script filtered readings by grip_cam_z >= 0.66 and
printed no count for them. The 2026-09-10 document recorded that the threshold
was tuned for a 44mm cuboid and left cells of n=0..8 on the 50mm batch - but
the script was never changed, so the 20mm batch hit it again: 2 of 11 readings
survived in one cell. Worse, grip_cam_z IS the noisy depth reading, so a fixed
threshold on it selects a noise-dependent subsample - selection on the
dependent variable. Filtered, the residual looked like a smooth monotone
decline with noise; unfiltered it is a step at the first non-zero level and
flat after, which is the dithering signature and a different claim.

The shallow readings the threshold was meant to remove are the ones where the
detector caught the forearm instead of the gripper. That contamination is
real, so the filtered view is still printed - as a diagnostic, with its n, and
never as the headline.

Usage: analyse_depthnoise.py <batch dir>
"""
import collections
import glob
import json
import math
import os
import re
import statistics
import sys

ID = re.compile(r"dn_n(\d+)_y(\d+)_(\d+)$")
PIPELINE = {"planning_failure", "detector_timeout", "gripper_command_failure"}


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


# Readings shallower than this caught the forearm rather than the gripper; see
# the module docstring for why this is a diagnostic and not a filter.
SHALLOW_CUT_M = 0.66


def fmt_resid(values):
    """Mean +- sd with the count, because a figure from n=2 must say so."""
    if not values:
        return "n=0  -"
    if len(values) == 1:
        return "n=1  %.2f" % values[0]
    return "n=%d  %.2f±%.2f" % (len(values), statistics.mean(values),
                                statistics.stdev(values))


def main():
    if len(sys.argv) != 2:
        sys.exit(__doc__)
    cells = collections.defaultdict(list)
    resid = collections.defaultdict(list)
    deep = collections.defaultdict(list)
    for path in sorted(glob.glob(os.path.join(sys.argv[1], "*", "trial.json"))):
        rec = json.loads(open(path).read())
        m = ID.match(rec["trial_id"])
        if not m:
            continue
        noise_mm = int(m.group(1))          # 000 -> 0mm, 001 -> 1mm, 005 -> 5mm
        bias = int(m.group(2))
        cells[(noise_mm, bias)].append(rec["category"])
        meas = rec.get("measurements") or {}
        if "residual_x" in meas:
            r = math.hypot(meas["residual_x"], meas["residual_y"]) * 1000
            resid[(noise_mm, bias)].append(r)
            if meas.get("grip_cam_z", 0.0) >= SHALLOW_CUT_M:
                deep[(noise_mm, bias)].append(r)

    print("%7s %6s %5s %20s %18s %18s  %s" % (
        "noise", "bias", "n", "success", "|残差| mm (全部)",
        "(仅深读数, 诊断)", "other"))
    for key in sorted(cells):
        noise_mm, bias = key
        out = cells[key]
        k = sum(1 for c in out if c == "success")
        lo, hi = wilson(k, len(out))
        other = ", ".join("%s:%d" % kv for kv in
                          collections.Counter(c for c in out
                                              if c != "success").items())
        print("%5dmm %5dmm %5d %6s [%4.0f%%,%4.0f%%] %18s %18s  %s" % (
            noise_mm, bias, len(out), "%d/%d" % (k, len(out)),
            lo * 100, hi * 100,
            fmt_resid(resid.get(key, [])), fmt_resid(deep.get(key, [])),
            other))

    print("\n相对无噪对照的差值（同偏置配对）：")
    for bias in sorted({b for _, b in cells}):
        base = cells.get((0, bias))
        if not base:
            continue
        k0 = sum(1 for c in base if c == "success")
        print("  偏置 %dmm，对照 %d/%d：" % (bias, k0, len(base)))
        for noise_mm in sorted({n for n, b in cells if b == bias and n != 0}):
            out = cells[(noise_mm, bias)]
            k = sum(1 for c in out if c == "success")
            d, dlo, dhi = newcombe(k, len(out), k0, len(base))
            flag = ("与对照无可分辨差异" if dlo <= 0 <= dhi
                    else "**与对照有差异**")
            print("    %dmm 噪声 -> %d/%d  差值 %+.1f%% CI [%+.1f%%, %+.1f%%]  %s"
                  % (noise_mm, k, len(out), d * 100, dlo * 100, dhi * 100, flag))


if __name__ == "__main__":
    main()
