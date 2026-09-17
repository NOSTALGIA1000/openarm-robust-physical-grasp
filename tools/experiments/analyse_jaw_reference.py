"""Split the jaw offset into the part that is the arm and the part that is aim.

measure_baseline() in analyse_joint_axes.py takes a batch's zero-bias median
and calls it geometry. Within one batch that is safe: everything it removes is
common to every cell, so the differences between cells survive. Across two
pipelines it is not, because a pipeline's zero-bias median contains its own
systematic aiming error as well as the geometry, and subtracting it defines
that error away. On the first 19 trials of stage57_reljaw that median is
+16.95mm in x against stage56_jaw's +10.71mm, and nothing in the number says
which pipeline is further off and which simply holds its fingers elsewhere.

The trial record carries the commanded target, so the sum can be split:

    jaw - object  =  (jaw - target)  +  (target - object)

The right-hand term is the aiming error - what the detector and whatever
correction is applied got wrong - and it is already recorded as residual_x/y.
The left-hand term is the arm: where the jaws end up relative to where they
were told to go, which is the finger geometry plus the controller's tracking
error and has nothing to do with the detector. That term is the candidate for a
constant that transfers between pipelines, and this script measures whether it
does.

If jaw - target agrees between the two batches, the 6.24mm gap between their
zero-bias medians is a real difference in aim and the open-loop curve can be
read at the corrected arm's residual. If it does not agree, the arms are
tracking differently and no single constant makes the two comparable - which
would be worth knowing before any more is built on top of the jaw centre.

Both are reported over successes and over all trials, because measure_baseline
uses successes only and selecting on the outcome can shrink an offset that is
being used to predict that outcome.

Usage:
    python tools/experiments/analyse_jaw_reference.py <batch dir> [<batch dir> ...]
"""

import csv
import glob
import json
import math
import os
import sys


def stats(values):
    if not values:
        return None
    ordered = sorted(values)
    n = len(ordered)

    def q(p):
        return ordered[min(n - 1, int(p * (n - 1) + 0.5))]

    return (n, q(0.5), q(0.1), q(0.9))


def read_trial(path):
    rec = json.loads(open(path, encoding="utf-8").read())
    meas = rec.get("measurements") or {}
    if "target_x" not in meas:
        return None
    trial_dir = os.path.dirname(path)
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

    def jaw(axis):
        key = "jaw_" + axis
        if key in close and close[key] not in ("", "nan", "NaN"):
            return float(close[key]), True
        return float(close["gripper_" + axis]), False

    try:
        jx, from_jaw = jaw("x")
        jy, _ = jaw("y")
        first = rows[0]
        ox, oy = float(first["object_x"]), float(first["object_y"])
    except (KeyError, ValueError):
        return None
    return {
        "success": bool(rec.get("success")),
        "from_jaw": from_jaw,
        # The arm: commanded target to where the jaws actually were.
        "tx": (jx - float(meas["target_x"])) * 1000.0,
        "ty": (jy - float(meas["target_y"])) * 1000.0,
        # The aim: commanded target to the truth. Recorded as residual_*; recomputed
        # here so a record with one field and not the other is caught.
        "rx": (float(meas["target_x"]) - float(meas["ground_truth_x"])) * 1000.0,
        "ry": (float(meas["target_y"]) - float(meas["ground_truth_y"])) * 1000.0,
        "resx": float(meas.get("residual_x", "nan")) * 1000.0,
        "resy": float(meas.get("residual_y", "nan")) * 1000.0,
        # The total, which is what analyse_joint_axes.py works in.
        "ox": (jx - ox) * 1000.0,
        "oy": (jy - oy) * 1000.0,
        # Does the record's ground truth agree with the first sample? If not,
        # the two halves of the split are measured against different objects.
        "gap": math.hypot(float(meas["ground_truth_x"]) - ox,
                          float(meas["ground_truth_y"]) - oy) * 1000.0,
    }


def report(batch, cell_filter):
    rows = []
    for path in sorted(glob.glob(os.path.join(batch, "*", "trial.json"))):
        trial_id = os.path.basename(os.path.dirname(path))
        if cell_filter and not trial_id.startswith(cell_filter):
            continue
        row = read_trial(path)
        if row:
            rows.append(row)
    if not rows:
        print("%s: nothing readable" % batch)
        return
    ref = "jaw centre" if all(r["from_jaw"] for r in rows) else "link origin"
    ok = [r for r in rows if r["success"]]
    print("%s  (%d trials, %d successes, reference %s)"
          % (batch, len(rows), len(ok), ref))
    gap = stats([r["gap"] for r in rows])
    print("    ground truth vs first sample: median %.2fmm, p90 %.2fmm"
          % (gap[1], gap[3]))
    for label, subset in (("successes", ok), ("all", rows)):
        if not subset:
            continue
        print("    over %-9s  %-22s %-22s %s"
              % (label, "jaw - target (arm)", "target - truth (aim)",
                 "jaw - object (total)"))
        for axis, ta, aa, oa in (("x", "tx", "rx", "ox"),
                                 ("y", "ty", "ry", "oy")):
            t = stats([r[ta] for r in subset])
            a = stats([r[aa] for r in subset])
            o = stats([r[oa] for r in subset])
            print("      %s  %+7.2f [%+6.2f,%+6.2f]  %+7.2f [%+6.2f,%+6.2f]  "
                  "%+7.2f [%+6.2f,%+6.2f]"
                  % (axis, t[1], t[2], t[3], a[1], a[2], a[3],
                     o[1], o[2], o[3]))
    print("      median [p10, p90], millimetres")
    print("")


def main():
    if len(sys.argv) < 2:
        sys.exit(__doc__)
    for arg in sys.argv[1:]:
        if ":" in arg:
            batch, cell = arg.rsplit(":", 1)
        else:
            batch, cell = arg, ""
        report(batch, cell)
    return 0


if __name__ == "__main__":
    sys.exit(main())
