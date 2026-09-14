"""Are the two offset budgets independent, or is there one joint budget?

The single-axis curves (stage54_openloop, n=20 per cell, open loop so the bias
reaches the achieved offset directly) are:

    bias      8mm    16mm    24mm
    y       18/20   17/20    5/19
    x       20/20   16/20    0/20

Both budgets sit between 16 and 24mm. This asks what happens when both axes
are off at once, x = y = d, which separates two models:

  rectangular  the budgets are separate and the failures multiply:
               P(d) = P_y(d) * P_x(d)
  radial       there is one budget on hypot(d, d) = d * 1.414:
               P(d) = P_single(d * 1.414)

At d=16 the rectangular prediction is exact - both singles were measured at
16mm - and comes to 0.85 * 0.80 = 68%. The radial prediction needs the single
rate at 22.6mm, which was not measured; it lies between the 16mm rates
(80-85%) and the 24mm rates (0-26%), so interpolation puts it near 10-40%.
That gap is what n=20 can resolve. Observing 68% supports separate budgets;
observing under 40% supports a joint one.

d=8 is the control: both models predict about 90%, so a low rate there means
something other than the budget geometry is wrong.

d=20 is exploratory. Neither prediction is grounded: the singles were not
measured at 20mm, so the rectangular product rests on interpolation, and the
radial one needs a single rate at 28.3mm that does not exist either.

Usage:
    python tools/experiments/analyse_joint_axes.py <batch dir>
"""

import collections
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

# Measured singles, for the rectangular prediction. Successes over trials.
SINGLES = {
    "y": {0: (18, 20), 8: (18, 20), 16: (17, 20), 24: (5, 19)},
    "x": {8: (20, 20), 16: (16, 20), 24: (0, 20)},
}


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


def rectangular(d):
    """P_y(d) * P_x(d), or None when either single was not measured at d."""
    cells = [SINGLES[axis].get(d) for axis in ("y", "x")]
    if any(c is None for c in cells):
        return None
    return (cells[0][0] / cells[0][1]) * (cells[1][0] / cells[1][1])


def main():
    if len(sys.argv) != 2:
        sys.exit(__doc__)
    cells = collections.defaultdict(list)
    for path in sorted(glob.glob(os.path.join(sys.argv[1], "*", "trial.json"))):
        rec = json.loads(open(path, encoding="utf-8").read())
        m = TRIAL.match(rec.get("trial_id", ""))
        if m:
            cells[(m.group(1), int(m.group(2)))].append(rec.get("category", "?"))

    for axis in ("y", "x", "xy"):
        keys = sorted(k for k in cells if k[0] == axis)
        if not keys:
            continue
        print("\n=== %s ===" % ("joint, x = y = d" if axis == "xy"
                                else "single axis %s" % axis))
        for key in keys:
            out = cells[key]
            k, n = sum(1 for c in out if c == "success"), len(out)
            lo, hi = wilson(k, n)
            forms = collections.Counter(c for c in out if c != "success")
            line = ("  %2dmm  %2d/%-2d %6.1f%% [%3.0f%%,%3.0f%%]  %s"
                    % (key[1], k, n, 100.0 * k / n, lo * 100, hi * 100,
                       ", ".join("%s x%d" % c for c in forms.most_common(3))
                       or "-"))
            if axis == "xy":
                pred = rectangular(key[1])
                if pred is None:
                    line += "   [rectangular: singles not measured at %dmm]" % key[1]
                else:
                    pk, pn = round(pred * n), n
                    diff, dlo, dhi = newcombe(k, n, pk, pn)
                    line += ("   rectangular %.0f%%, observed minus predicted "
                             "%+.3f [%+.3f, %+.3f] %s"
                             % (100 * pred, diff, dlo, dhi,
                                "differs" if (dlo > 0 or dhi < 0)
                                else "consistent"))
            print(line)
    return 0


if __name__ == "__main__":
    sys.exit(main())
