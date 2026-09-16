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
Each cell's offset is measured rather than assumed, with the geometric
baseline subtracted - and that baseline is itself measured, from the batch's
own zero-bias successes. A batch without a zero-bias cell is refused rather
than given a borrowed constant.

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
#
# rel_ is the corrected arm (sweep_rel_jaw.sh), read by the same code on
# purpose: measuring both pipelines with one ruler is the whole point of the
# jaw-centre telemetry. Note that for rel_ the number in the trial id is the
# CAMERA bias, which the correction is supposed to remove - it is not the
# achieved offset and must not be read as one. Only the measured offset below
# is comparable between the two.
TRIAL = re.compile(r"(?:ol|rel)_(xy|x|y)(\d+)_\d+$")

# No hardcoded baseline. It is measured per batch by measure_baseline() from
# that batch's own zero-bias successes, because it differs by reference and by
# pipeline: +7.66mm in x from the link origin on stage54_openloop, +10.71mm
# from the jaw centre on stage56_jaw, and +21.45mm from the link origin on the
# corrected pipeline. A constant written here would be right for one batch.


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
    # jaw_x/jaw_y when the record has them (2026-09-16 onward): the midpoint of
    # the two finger geoms, which is where the jaws meet. Falling back to
    # gripper_x/y means the link origin, whose offset from the jaws depends on
    # the approach pose - at zero bias it sits +7.65mm out in x for the
    # open-loop pipeline and +21.45mm for the corrected one, so offsets taken
    # from it are not comparable across pipelines and the baseline below has to
    # be re-measured per batch.
    def pick(axis):
        key = "jaw_" + axis
        if key in close:
            value = close[key]
            if value not in ("", "nan", "NaN"):
                return float(value), True
        return float(close["gripper_" + axis]), False

    try:
        first = rows[0]
        gx, from_jaw = pick("x")
        gy, _ = pick("y")
        dx = (gx - float(first["object_x"])) * 1000.0
        dy = (gy - float(first["object_y"])) * 1000.0
        moved = math.hypot(
            float(close["object_x"]) - float(first["object_x"]),
            float(close["object_y"]) - float(first["object_y"])) * 1000.0
    except (KeyError, ValueError):
        return None
    return (dx, dy, moved, from_jaw)


def measure_baseline(trial_dirs, successes_only=True):
    """The geometric term, measured from a batch's own zero-bias successes.

    A perfect grasp does not read as zero on either reference. The link origin
    is not the jaw centre, so gripper_* carries a pose-dependent offset. And
    the jaw centre is the midpoint of the two fingers while they are still
    OPEN on the first close frame, so it is not yet the contact point either:
    measured on stage56_jaw's zero-bias cell it sits +10.56mm out in x and
    +1.13mm in y.

    The first version of this function subtracted nothing on the jaw path, on
    the reasoning that measuring from the jaws leaves nothing to subtract. That
    was wrong, and it put ol_y024 at ex 15.01mm when the honest figure is about
    4.5mm - thirteen millimetres of x error that came from the arithmetic
    rather than the arm. What the jaw centre actually fixes is that its term
    does NOT drift with the approach pose: 10.56mm at zero bias and 10.46mm at
    16mm of y, against 7.65 and 21.45 for the link origin between two
    pipelines. A constant can be subtracted; drift cannot.

    So it is measured, never assumed, and measured per batch.
    """
    values = []
    for trial_dir in trial_dirs:
        offset = achieved(trial_dir)
        if offset is None:
            continue
        values.append(offset)
    if len(values) < 5:
        return None
    return (median([v[0] for v in values]), median([v[1] for v in values]))


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
    raw = collections.defaultdict(list)
    zero_dirs = []
    for path in sorted(glob.glob(os.path.join(sys.argv[1], "*", "trial.json"))):
        rec = json.loads(open(path, encoding="utf-8").read())
        m = TRIAL.match(rec.get("trial_id", ""))
        if not m:
            continue
        trial_dir = os.path.dirname(path)
        key = (m.group(1), int(m.group(2)))
        raw[key].append((rec.get("category", "?"), achieved(trial_dir)))
        if key[1] == 0 and rec.get("success"):
            zero_dirs.append(trial_dir)

    # Measured from this batch's own zero-bias successes, never assumed and
    # never carried over from another batch. Without a zero-bias cell there is
    # nothing to measure it from, and guessing is how the last two x results
    # went wrong.
    base = measure_baseline(zero_dirs)
    if base is None:
        print("no zero-bias cell with at least 5 successes in %s - the "
              "geometric baseline cannot be measured and offsets would be "
              "wrong by an unknown constant. Refusing." % sys.argv[1])
        return 1
    from_jaw = any(o[3] for out in raw.values() for _, o in out if o)
    print("geometric baseline measured from %d zero-bias successes: "
          "dx %+.2fmm, dy %+.2fmm  (reference: %s)"
          % (len(zero_dirs), base[0], base[1],
             "jaw centre" if from_jaw else "gripper link origin"))
    print("")

    cells = {key: [(c, None if o is None
                    else (abs(o[0] - base[0]), abs(o[1] - base[1]), o[2]))
                   for c, o in out]
             for key, out in raw.items()}

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
