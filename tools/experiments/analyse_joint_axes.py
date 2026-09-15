"""Are the two offset budgets independent, or is there one joint budget?

Single-axis open loop, n=20 per cell, where the bias reaches the achieved
offset directly:

    bias      8mm    16mm    24mm    32mm
    y       18/20   17/20    5/19       -
    x       20/20   16/20    0/20    0/20

Both budgets sit between 16 and 24mm. This asks what happens when both axes
are off at once, x = y = d, which separates three shapes for the joint region:

  rectangular  separate budgets, failures multiply: P = P_y(ey) * P_x(ex)
  radial       one budget on hypot(ex, ey)
  L1           one budget on ex + ey

Predictions are computed from the approach offset, not the commanded bias
------------------------------------------------------------------------
Each cell's offset is measured rather than assumed, with the per-axis
geometric baseline subtracted: dx carries +7.67mm and dy +1.50mm at zero bias,
from this batch's own zero-bias cells where the baseline holds to about 3mm.

It is measured against the cube's ORIGINAL position, not its position when the
jaws close - see achieved() for why. At large y bias the gripper shoves the
cube on the way down, so the closing-time offset is partly an effect of the
grasp it is meant to predict. The first version of this analysis used the
closing-time offset and put ol_y024 at (10.3, 18.1)mm; against the cube's
original place it is (2.0, 23.0), and the 24mm command asked for 23.

The single-axis response P(offset) is interpolated between measured cells and
clamped outside them. The x and y curves differ past 16mm, so radial and L1 -
which need one curve - are evaluated against both and reported as a range. If
that range is wide the two models cannot be told apart, which is a fact about
the evidence rather than a result, and the densified 18 and 21mm cells exist
to narrow it.

Usage:
    python tools/experiments/analyse_joint_axes.py <batch dir>
"""

import collections
import csv
import glob
import json
import math
import os
import re
import sys

# ol_xy008_1 as well as ol_x024_7 and ol_y016_3. The narrower ol_([xy])(\d+)
# silently dropped every joint trial: it matched the x and then met a y where
# it wanted a digit.
TRIAL = re.compile(r"ol_(xy|x|y)(\d+)_\d+$")

# gripper minus object at zero bias, measured from this batch's own ol_y000
# cell. The gripper link origin is not at the pad centre, so a perfect grasp
# does not read as zero.
BASELINE_DX_MM = 7.67
BASELINE_DY_MM = 1.50


def wilson(k, n, z=1.96):
    if n == 0:
        return (0.0, 1.0)
    p = k / n
    d = 1.0 + z * z / n
    c = p + z * z / (2 * n)
    h = z * math.sqrt(p * (1 - p) / n + z * z / (4 * n * n))
    return (max(0.0, (c - h) / d), min(1.0, (c + h) / d))


def newcombe(k1, n1, k2, n2, z=1.96):
    p1, p2 = k1 / n1, k2 / n2
    l1, u1 = wilson(k1, n1, z)
    l2, u2 = wilson(k2, n2, z)
    d = p1 - p2
    return (d,
            d - math.sqrt((p1 - l1) ** 2 + (u2 - p2) ** 2),
            d + math.sqrt((u1 - p1) ** 2 + (p2 - l2) ** 2))


def achieved(trial_dir):
    """(ex, ey, moved) in mm: the offset the arm approached with.

    Gripper position on the first close frame against the object's position on
    the FIRST frame of the trial, not its position at closing time. At large y
    bias the gripper brushes the cube on the way down and shoves it - 6.8mm in
    x and 5.4mm in y for ol_y024, 4.0mm in y for ol_xy020, against 0.02mm
    everywhere else. Measured at closing time the cube has already been pushed
    toward the gripper, so the offset reads smaller than the one the arm
    actually approached with: ol_y024 reads (10.3, 18.1) that way and
    (2.0, 23.0) against the cube's original place. The 10.3mm of x is entirely
    the cube being shoved sideways, and 23.0 is what the 24mm command asked
    for.

    That makes the closing-time offset endogenous - it is partly an effect of
    the grasp it is supposed to predict. `moved` reports how far the cube
    travelled so a caller can see which cells are affected.
    """
    try:
        with open(os.path.join(trial_dir, "samples.csv"), newline="",
                  encoding="utf-8") as handle:
            rows = list(csv.DictReader(handle))
    except OSError:
        return None
    if not rows:
        return None
    close = next((r for r in rows if r.get("stage") == "close"), None)
    if close is None:
        return None
    try:
        first = rows[0]
        dx = (float(close["gripper_x"]) - float(first["object_x"])) * 1000.0
        dy = (float(close["gripper_y"]) - float(first["object_y"])) * 1000.0
        moved = math.hypot(
            float(close["object_x"]) - float(first["object_x"]),
            float(close["object_y"]) - float(first["object_y"])) * 1000.0
    except (KeyError, ValueError):
        return None
    return (abs(dx - BASELINE_DX_MM), abs(dy - BASELINE_DY_MM), moved)


def response(curve, offset):
    """Interpolate a measured single-axis curve, clamped at both ends.

    Clamped rather than extrapolated: past the last measured cell every model
    would otherwise be evaluated off the end of the evidence, which is how a
    prediction turns into a guess wearing a number.
    """
    if offset <= curve[0][0]:
        return curve[0][1]
    if offset >= curve[-1][0]:
        return curve[-1][1]
    for (a, ra), (b, rb) in zip(curve, curve[1:]):
        if a <= offset <= b:
            return ra + (rb - ra) * (offset - a) / (b - a)
    return curve[-1][1]


def median(values):
    ordered = sorted(values)
    return ordered[len(ordered) // 2]


def cell_offsets(out):
    offs = [o for _, o in out if o]
    return (median([o[0] for o in offs]), median([o[1] for o in offs]),
            median([o[2] for o in offs]))


def main():
    if len(sys.argv) != 2:
        sys.exit(__doc__)
    cells = collections.defaultdict(list)
    for path in sorted(glob.glob(os.path.join(sys.argv[1], "*", "trial.json"))):
        rec = json.loads(open(path, encoding="utf-8").read())
        m = TRIAL.match(rec.get("trial_id", ""))
        if not m:
            continue
        cells[(m.group(1), int(m.group(2)))].append(
            (rec.get("category", "?"), achieved(os.path.dirname(path))))

    curves = {}
    for axis in ("y", "x"):
        pts = []
        for key in sorted(k for k in cells if k[0] == axis):
            out = cells[key]
            if not any(o for _, o in out):
                continue
            ex, ey, _ = cell_offsets(out)
            k = sum(1 for c, _ in out if c == "success")
            pts.append((ey if axis == "y" else ex, k / len(out)))
        curves[axis] = sorted(pts)
        print("single-axis %s response, by achieved offset:" % axis)
        for off, rate in curves[axis]:
            print("    %6.2f mm -> %5.1f%%" % (off, 100 * rate))

    joint = sorted(k for k in cells if k[0] == "xy")
    if not joint:
        print("no joint cells yet")
        return 0

    print("")
    print("%-8s %-4s %-15s %-9s %-12s %-12s %s"
          % ("cell", "n", "achieved ex,ey", "observed", "rectangular",
             "radial", "L1"))
    for key in joint:
        out = cells[key]
        n = len(out)
        k = sum(1 for c, _ in out if c == "success")
        ex, ey, moved = cell_offsets(out)
        rect = response(curves["y"], ey) * response(curves["x"], ex)
        rad = [response(curves[a], math.hypot(ex, ey)) for a in ("y", "x")]
        l1 = [response(curves[a], ex + ey) for a in ("y", "x")]
        print("%-8s %-4d %6.1f,%6.1f %7.1f%% %10.1f%% %6.0f-%3.0f%% %6.0f-%3.0f%%"
              % ("d=%dmm" % key[1], n, ex, ey, 100.0 * k / n, 100 * rect,
                 100 * min(rad), 100 * max(rad),
                 100 * min(l1), 100 * max(l1)))

    print("")
    print("observed against each model, Newcombe on the same n:")
    for key in joint:
        out = cells[key]
        n = len(out)
        k = sum(1 for c, _ in out if c == "success")
        ex, ey, moved = cell_offsets(out)
        preds = [("rectangular",
                  response(curves["y"], ey) * response(curves["x"], ex))]
        for name, val in (("radial", math.hypot(ex, ey)), ("L1", ex + ey)):
            for axis in ("y", "x"):
                preds.append(("%s via %s" % (name, axis),
                              response(curves[axis], val)))
        print("  d=%dmm observed %d/%d = %.1f%%" % (key[1], k, n, 100.0 * k / n))
        for name, p in preds:
            pk = int(round(p * n))
            diff, lo, hi = newcombe(k, n, pk, n)
            print("    %-16s predicts %5.1f%%  diff %+.3f [%+.3f, %+.3f]  %s"
                  % (name, 100 * p, diff, lo, hi,
                     "REJECTED" if (lo > 0 or hi < 0) else "consistent"))
    return 0


if __name__ == "__main__":
    sys.exit(main())
