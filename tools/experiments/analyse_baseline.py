#!/usr/bin/env python3
"""One-shot correction against the ECL servo baseline.

Reports what each arm achieves and what it costs. Accuracy alone would miss
the point: the claim under test is that a single measurement added to forward
kinematics reaches the same place as an iterative servo without estimating a
Jacobian, so the number of planned motions matters as much as the residual.

The one-shot arm spends one re-hover. The servo arm spends two probe motions
plus however many iterations it needs.

Usage: analyse_baseline.py <batch dir>
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


def mean(values):
    return sum(values) / len(values) if values else float("nan")


def main():
    if len(sys.argv) != 2:
        sys.exit(__doc__)
    cells = collections.defaultdict(list)
    for path in sorted(glob.glob(os.path.join(sys.argv[1], "*", "trial.json"))):
        rec = json.loads(open(path).read())
        parts = rec["trial_id"].split("_")        # bl_<arm>_y<bias>_<rep>
        cells[(parts[1], int(parts[2][1:]))].append(rec)

    for arm in sorted({a for a, _ in cells}):
        print("\n=== %s ===" % arm)
        print("%6s %4s %9s %20s %10s %10s %8s %s" % (
            "bias", "n", "success", "Wilson 95%", "res_x mm", "res_y mm",
            "moves", "notes"))
        for bias in sorted(b for a, b in cells if a == arm):
            recs = cells[(arm, bias)]
            n = len(recs)
            k = sum(1 for r in recs if r["category"] == "success")
            lo, hi = wilson(k, n)
            ms = [r.get("measurements") or {} for r in recs]
            rx = [m["residual_x"] * 1000 for m in ms if "residual_x" in m]
            ry = [m["residual_y"] * 1000 for m in ms if "residual_y" in m]
            # One-shot spends a single re-hover; the servo spends its probes
            # and its iterations.
            moves = [m["servo_probes"] + m["servo_iters"]
                     for m in ms if "servo_probes" in m]
            conv = [m["servo_converged"] for m in ms if "servo_converged" in m]
            note = ", ".join(
                "%s:%d" % (c, v)
                for c, v in collections.Counter(
                    r["category"] for r in recs).items() if c != "success")
            if conv:
                note = ("converged %d/%d" % (int(sum(conv)), len(conv))
                        + (("; " + note) if note else ""))
            print("%6d %4d %3d/%-5d [%5.1f%%, %5.1f%%] %10.2f %10.2f %8.1f %s" % (
                bias, n, k, n, lo * 100, hi * 100,
                mean(rx), mean(ry), mean(moves) if moves else 1.0, note))


if __name__ == "__main__":
    main()
