"""Does the grasp succeed exactly when the jaw-axis residual fits the clearance?

Every tolerance number this project has produced is a table: this bias, this
cube, this opening, this success rate. This asks whether one computable
criterion accounts for them, by pooling batches that differ in object, opening
and date and splitting each trial on a quantity the trial record already holds:

    succeed  iff  |residual_y| < (2 * open_position * 1000 - width_mm) / 2

residual_y is the post-correction error in the direction the jaws close, and
the right-hand side is the per-side clearance. The claim is not that crossing
the clearance guarantees failure - it does not - but that staying inside it is
what success requires.

Where the object width comes from
---------------------------------
Preferably from environment.object_half_extents_m, which trial_record.py
records from 2026-09-13 by reading the scene that ran: the y half-extent times
two is the jaw-axis width.

Trials recorded before that carry the object only by geom NAME, and
detector_z_bias - the one numeric clue - is half the object's HEIGHT plus the
detector's 5.5mm surface offset. Height fixes the width only for a cube: the
30x30x44mm cuboid the scene carried before 2026-09-11 has a 22mm half-height
and a 30mm jaw-axis width. So for those, WIDTHS_MM below is an explicit lookup
and unknown values are refused rather than guessed - this project has
mis-applied a configuration-specific constant six times and guessing here
would make seven. The report says which source each trial used.

Usage:
    python tools/experiments/analyse_jaw_criterion.py <batch dir> [<batch dir> ...]
"""

import collections
import glob
import json
import math
import os
import sys

# round(detector_z_bias, 4) -> width along the jaw-closing axis, in mm.
#   0.0275 = 22.0 + 5.5  -> the 30x30x44mm cuboid, 30mm across the jaws
#   0.0155 = 10.0 + 5.5  -> the 20mm cube
#   0.0205 = 15.0 + 5.5  -> the 30mm cube
#   0.0305 = 25.0 + 5.5  -> the 50mm cube
WIDTHS_MM = {0.0275: 30.0, 0.0155: 20.0, 0.0205: 30.0, 0.0305: 50.0}

BINS_MM = [(0, 5), (5, 10), (10, 15), (15, 20), (20, 25), (25, 30), (30, 1000)]

# Failures that are not about where the jaws ended up. IK infeasibility is a
# different mechanism and would otherwise be charged against the criterion.
NON_GRASP_FAILURES = ("planning_failure", "detector_timeout")


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


def load(roots):
    rows, skipped = [], collections.Counter()
    for root in roots:
        batch = os.path.basename(os.path.normpath(root))
        for path in sorted(glob.glob(os.path.join(root, "*", "trial.json"))):
            rec = json.loads(open(path, encoding="utf-8").read())
            cfg = rec.get("config") or {}
            meas = rec.get("measurements") or {}
            if "residual_y" not in meas:
                skipped["no optical measurement recorded"] += 1
                continue
            opening = cfg.get("open_position")
            zbias = cfg.get("detector_z_bias")
            if opening is None or zbias is None:
                skipped["open_position or detector_z_bias missing"] += 1
                continue
            extents = (rec.get("environment") or {}).get(
                "object_half_extents_m")
            if extents and len(extents) >= 2:
                # y half-extent, doubled: the jaws close along world y and the
                # object is axis-aligned in these scenes.
                width, source = 2.0 * float(extents[1]) * 1000.0, "recorded"
            else:
                width, source = WIDTHS_MM.get(round(zbias, 4)), "z_bias lookup"
            if width is None:
                skipped["detector_z_bias %.4f not in WIDTHS_MM and no "
                        "object_half_extents_m recorded" % zbias] += 1
                continue
            rows.append({
                "batch": batch,
                "trial": rec.get("trial_id", ""),
                "clearance": round((2.0 * opening * 1000.0 - width) / 2.0, 1),
                "width_source": source,
                "res_y": abs(meas["residual_y"]) * 1000.0,
                "res_x": abs(meas.get("residual_x", 0.0)) * 1000.0,
                "success": bool(rec.get("success")),
                "category": rec.get("category", "?"),
            })
    return rows, skipped


def report_bins(rows, clearance):
    print("  %-11s %7s %7s %-15s  %s"
          % ("|res_y|", "succ", "rate", "Wilson 95%", "failure forms"))
    for lo, hi in BINS_MM:
        cell = [r for r in rows if lo <= r["res_y"] < hi]
        if not cell:
            continue
        k, n = sum(1 for r in cell if r["success"]), len(cell)
        wl, wh = wilson(k, n)
        forms = collections.Counter(r["category"] for r in cell
                                    if not r["success"])
        mark = " <" if lo < clearance <= hi else "  "
        label = "%d-%dmm" % (lo, hi) if hi < 1000 else ">=%dmm" % lo
        print("  %-11s %3d/%-3d %6.1f%% [%3.0f%%,%3.0f%%]%s %s"
              % (label, k, n, 100.0 * k / n, wl * 100, wh * 100, mark,
                 ", ".join("%s x%d" % c for c in forms.most_common(3)) or "-"))


def split(rows, clearance, label, drop_non_grasp):
    used = [r for r in rows
            if not (drop_non_grasp and not r["success"]
                    and r["category"] in NON_GRASP_FAILURES)]
    inside = [r for r in used if r["res_y"] < clearance]
    outside = [r for r in used if r["res_y"] >= clearance]
    if not inside or not outside:
        print("  %s: one side is empty (%d in, %d out) - no test"
              % (label, len(inside), len(outside)))
        return
    k1, n1 = sum(1 for r in inside if r["success"]), len(inside)
    k2, n2 = sum(1 for r in outside if r["success"]), len(outside)
    d, lo, hi = newcombe(k1, n1, k2, n2)
    print("  %s" % label)
    print("    inside  %3d/%-3d %6.1f%%   outside %3d/%-3d %6.1f%%"
          % (k1, n1, 100.0 * k1 / n1, k2, n2, 100.0 * k2 / n2))
    print("    difference %+.3f  Newcombe [%+.4f, %+.4f]  %s"
          % (d, lo, hi, "excludes 0" if (lo > 0 or hi < 0) else "straddles 0"))


def main():
    if len(sys.argv) < 2:
        sys.exit(__doc__)
    rows, skipped = load(sys.argv[1:])
    if not rows:
        print("no usable trials under %s" % ", ".join(sys.argv[1:]))
        for reason, n in skipped.most_common():
            print("  skipped %d: %s" % (n, reason))
        return 1

    batches = collections.Counter(r["batch"] for r in rows)
    print("%d trials from %d batch(es): %s"
          % (len(rows), len(batches),
             ", ".join("%s(%d)" % b for b in sorted(batches.items()))))
    srcs = collections.Counter(r["width_source"] for r in rows)
    print("object width from: %s"
          % ", ".join("%s x%d" % s for s in srcs.most_common()))
    for reason, n in skipped.most_common():
        print("  skipped %d: %s" % (n, reason))

    for clearance in sorted({r["clearance"] for r in rows}):
        cell = [r for r in rows if r["clearance"] == clearance]
        bs = sorted({r["batch"] for r in cell})
        print("\n=== clearance %.1f mm/side  (n=%d, batches: %s) ==="
              % (clearance, len(cell), ", ".join(bs)))
        report_bins(cell, clearance)
        split(cell, clearance, "split at the clearance, all trials", False)
        split(cell, clearance,
              "split at the clearance, non-grasp failures removed (%s)"
              % "/".join(NON_GRASP_FAILURES), True)

    # Pooled across clearances: the criterion is per-trial, so trials with
    # different clearances can be compared once each is judged against its own.
    print("\n=== pooled across clearances, each judged against its own ===")
    inside = [r for r in rows if r["res_y"] < r["clearance"]]
    outside = [r for r in rows if r["res_y"] >= r["clearance"]]
    for label, group in (("inside its clearance", inside),
                         ("outside its clearance", outside)):
        if not group:
            print("  %-24s empty" % label)
            continue
        k, n = sum(1 for r in group if r["success"]), len(group)
        wl, wh = wilson(k, n)
        forms = collections.Counter(r["category"] for r in group
                                    if not r["success"])
        print("  %-24s %3d/%-3d %6.1f%% [%3.0f%%,%3.0f%%]  %s"
              % (label, k, n, 100.0 * k / n, wl * 100, wh * 100,
                 ", ".join("%s x%d" % c for c in forms.most_common(4)) or "-"))
    if inside and outside:
        k1, n1 = sum(1 for r in inside if r["success"]), len(inside)
        k2, n2 = sum(1 for r in outside if r["success"]), len(outside)
        d, lo, hi = newcombe(k1, n1, k2, n2)
        print("  difference %+.3f  Newcombe [%+.4f, %+.4f]  %s"
              % (d, lo, hi, "excludes 0" if (lo > 0 or hi < 0)
                 else "straddles 0"))
    return 0


if __name__ == "__main__":
    sys.exit(main())
