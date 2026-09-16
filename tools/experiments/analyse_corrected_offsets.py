"""Where do the corrected arm's residuals fall inside the open-loop region?

The tolerance region of 2026-09-15 says how far the jaws may be from the cube
when they close. The corrected arm's claim is that it keeps that distance small
whatever the hand-eye error. Putting the two on one axis needs both measured
the same way, which is what this does: the same achieved() as
analyse_joint_axes.py, the same per-batch baseline from the batch's own
zero-bias cell, the same reference frame.

What it will NOT do is turn an (ex, ey) into a predicted success rate. No
candidate region shape survived 2026-09-15 - rectangular died on the diagonal,
every symmetric metric died off it - so there is no function to evaluate. What
can honestly be said is where a residual sits relative to the cells that were
actually measured, and that is what is printed: the median, the spread, and the
fraction of trials past each axis's half-failure point.

The spread is the part that matters and the part a median hides. An open-loop
residual is a clean translation, so every trial in a cell sits at nearly the
same offset. A corrected residual carries the relative measurement's own error,
so a cell with a median of 8mm can hold trials at 20mm, and those trials fail at
the rate the open-loop curve gives for 20mm rather than for 8. A comparison of
medians alone would miss that entirely.

Caveat, and the reason stage57_reljaw exists
--------------------------------------------
Batches recorded before 2026-09-16 have no jaw_centre, so this falls back to
the gripper link origin, whose offset from the jaws depends on the approach
pose. Subtracting a batch's own zero-bias baseline is only sound if the
corrected arm returns to a similar pose at every bias - which is what
correction means, but is an assumption here rather than a measurement. Where
the reference is the link origin, the printout says so.

Usage:
    python tools/experiments/analyse_corrected_offsets.py <batch dir> <zero cell prefix>

    python tools/experiments/analyse_corrected_offsets.py \
        docs/results/physics_grasp/stage52_cube20 cl_c020_o025_y000
"""

import collections
import glob
import json
import os
import re
import sys

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from analyse_joint_axes import achieved, measure_baseline, median, wilson

# The measured open-loop half-failure points (2026-09-15): y about 22mm, x
# about 18mm. Not a model of the region - just the two numbers each axis was
# measured to break at, used to say which side of them a residual sits on.
HALF_FAIL = {"x": 18.0, "y": 22.0}

CELL = re.compile(r"^(.*)_(\d+)$")


def cell_of(trial_id):
    m = CELL.match(trial_id)
    return m.group(1) if m else None


def quantile(values, q):
    ordered = sorted(values)
    if not ordered:
        return float("nan")
    idx = min(len(ordered) - 1, int(q * (len(ordered) - 1) + 0.5))
    return ordered[idx]


def main():
    if len(sys.argv) != 3:
        sys.exit(__doc__)
    batch, zero_cell = sys.argv[1], sys.argv[2]

    cells = collections.defaultdict(list)
    zero_dirs = []
    for path in sorted(glob.glob(os.path.join(batch, "*", "trial.json"))):
        rec = json.loads(open(path, encoding="utf-8").read())
        trial_id = rec.get("trial_id", "")
        cell = cell_of(trial_id)
        if cell is None:
            continue
        trial_dir = os.path.dirname(path)
        cells[cell].append((rec.get("category", "?"), achieved(trial_dir)))
        if cell == zero_cell and rec.get("success"):
            zero_dirs.append(trial_dir)

    if zero_cell not in cells:
        print("no cell named %s in %s. Cells present: %s"
              % (zero_cell, batch, ", ".join(sorted(cells))))
        return 1

    base = measure_baseline(zero_dirs)
    if base is None:
        print("cell %s has fewer than 5 zero-bias successes - the geometric "
              "baseline cannot be measured and every offset below would be "
              "wrong by an unknown constant. Refusing." % zero_cell)
        return 1
    from_jaw = any(o[3] for out in cells.values() for _, o in out if o)
    reference = "jaw centre" if from_jaw else "gripper link origin"
    print("batch %s" % batch)
    print("baseline from %d successes in %s: dx %+.2fmm, dy %+.2fmm  "
          "(reference: %s)" % (len(zero_dirs), zero_cell, base[0], base[1],
                               reference))
    if not from_jaw:
        print("  NOTE: link origin. Its offset from the jaws moves with the "
              "approach pose, so this baseline holds only as far as the "
              "corrected arm returns to the zero-bias pose.")
    print("")

    head = ("cell", "n", "succ", "|ex| med", "p90", "max", "|ey| med", "p90",
            "max", "past x18", "past y22")
    print("%-26s %-3s %-12s %8s %6s %6s %8s %6s %6s %8s %8s" % head)
    for cell in sorted(cells):
        out = cells[cell]
        offs = [o for _, o in out if o]
        if not offs:
            continue
        ex = [abs(o[0] - base[0]) for o in offs]
        ey = [abs(o[1] - base[1]) for o in offs]
        n = len(out)
        k = sum(1 for c, _ in out if c == "success")
        lo, hi = wilson(k, n)
        past_x = sum(1 for v in ex if v > HALF_FAIL["x"])
        past_y = sum(1 for v in ey if v > HALF_FAIL["y"])
        print("%-26s %-3d %2d/%-2d %3.0f%%  %8.2f %6.2f %6.2f %8.2f %6.2f "
              "%6.2f %6d/%-2d %5d/%-2d"
              % (cell, n, k, n, 100.0 * k / n,
                 median(ex), quantile(ex, 0.9), max(ex),
                 median(ey), quantile(ey, 0.9), max(ey),
                 past_x, len(ex), past_y, len(ey)))
        print("%-26s     Wilson [%.0f, %.0f]%%" % ("", 100 * lo, 100 * hi))
    print("")
    print("'past x18' / 'past y22' count trials whose residual exceeds that "
          "axis's measured half-failure point. They are a count of trials in "
          "the region where open loop fails half the time, not a prediction.")
    return 0


if __name__ == "__main__":
    sys.exit(main())
