"""Which offsets a grasp survives, measured where the jaws actually close.

Every tolerance number this project produced before 2026-09-14 was a table:
this bias, this cube, this opening, this success rate. This asks whether a
computable criterion accounts for them, by pooling batches that differ in
object, opening and date and splitting each trial on the offset between the
gripper and the object at the moment the jaws start closing.

    succeed  <==  |offset_y| < (2 * open_position * 1000 - width_mm) / 2
                  and |offset_x| < the pad span

The first bound is the per-side clearance. Staying inside it is sufficient,
not necessary - crossing it does not guarantee failure - and the two axes fail
differently: past the y bound one jaw still touches (unilateral_contact), past
the x bound neither does (no_contact).

Why the offset is read from samples.csv and not from residual_*
--------------------------------------------------------------
residual_* in trial.json is target minus ground truth: the error the picker
COMMANDED. What decides the grasp is the error the arm ACHIEVED. For y the two
agree - over 308 paired trials, Pearson r = 0.954 and a median difference of
-0.35mm - so the earlier residual-based criterion was measuring the right
thing on that axis.

For x they do not: r = 0.686 with the achieved offset running about +6.15mm
higher. Analyses built on residual_x were therefore wrong, and visibly so: five
cl_cal_y288 trials recorded residual_x near +25mm and mostly succeeded, which
supported a published claim that the pad width absorbs 25mm. Their achieved x
offset was about +8mm. The open-loop x sweep, where the bias goes straight into
the achieved offset, scores 0/20 at 24mm - the contradiction that exposed it.

gripper_x minus object_x also carries a fixed geometric term, because the
gripper link origin is not at the pad centre. It is measured here rather than
assumed: the median offset among trials that are square in y and succeeded.

Usage:
    python tools/experiments/analyse_jaw_criterion.py <batch dir> [<batch dir> ...]
"""

import collections
import csv
import glob
import statistics
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

# Trials this square in y are used to measure the fixed x term. They must be
# successes, so the jaws demonstrably closed on the cube.
BASELINE_EY_MM = 5.0

# How much the fixed x term may vary across the baseline trials before it stops
# deserving the name. Within one batch of one configuration it holds to about
# 6mm; pooled across batches it spans 22mm.
BASELINE_SPREAD_MAX_MM = 10.0

# Provisional. The open-loop x sweep ran 24/32/40/48mm and every level scored
# 0/20, so the knee is somewhere below 24mm and this bound is an upper limit,
# not a measurement. The 8 and 16mm levels are what will pin it.
X_BOUND_MM = 15.0

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


def achieved_offset(trial_dir):
    """gripper minus object, in mm, on the first frame of the close stage.

    The first frame, because once the jaws move the cube the offset stops
    describing where they met it.
    """
    path = os.path.join(trial_dir, "samples.csv")
    try:
        with open(path, newline="", encoding="utf-8") as handle:
            for row in csv.DictReader(handle):
                if row.get("stage") != "close":
                    continue
                return ((float(row["gripper_x"]) - float(row["object_x"])) * 1000.0,
                        (float(row["gripper_y"]) - float(row["object_y"])) * 1000.0)
    except (OSError, KeyError, ValueError):
        return None
    return None


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
            achieved = achieved_offset(os.path.dirname(path))
            if achieved is None:
                skipped["no close-stage frame in samples.csv"] += 1
                continue
            rows.append({
                "batch": batch,
                "trial": rec.get("trial_id", ""),
                "clearance": round((2.0 * opening * 1000.0 - width) / 2.0, 1),
                "width_source": source,
                "raw_dx": achieved[0],
                "ey": abs(achieved[1]),
                "res_y": abs(meas["residual_y"]) * 1000.0,
                "res_x": abs(meas.get("residual_x", 0.0)) * 1000.0,
                "success": bool(rec.get("success")),
                "category": rec.get("category", "?"),
            })

    # The fixed x term, measured per clearance group rather than assumed: the
    # gripper link origin is not at the pad centre, so an offset of zero error
    # does not read as zero. Refuse to guess when there is nothing to measure
    # it from.
    for clearance in {r["clearance"] for r in rows}:
        group = [r for r in rows if r["clearance"] == clearance]
        square = [r["raw_dx"] for r in group
                  if r["ey"] < BASELINE_EY_MM and r["success"]]
        if len(square) < 5:
            for r in group:
                r["ex"] = None
            skipped["clearance %.1fmm: only %d square successes, too few to "
                    "measure the fixed x term" % (clearance, len(square))] += 1
            continue
        # The x term is only fixed within one batch. Measured per batch it
        # ranges from -0.66mm (stage40_ceiling) to +21.58mm (stage52_cube20),
        # and inside a single batch it spans as much as 33mm - because
        # gripper_x is the link origin, not the pad centre, and where that
        # origin sits relative to the pads depends on the whole arm pose, not
        # just the commanded yaw. Subtracting a pooled median would absorb real
        # error into the baseline, so refuse unless the spread is small enough
        # that the term really is constant.
        spread = max(square) - min(square)
        if spread > BASELINE_SPREAD_MAX_MM:
            for r in group:
                r["ex"] = None
            skipped["clearance %.1fmm: x baseline spans %.1fmm across %d square "
                    "successes, so it is not a fixed term - |offset_x| refused"
                    % (clearance, spread, len(square))] += 1
            continue
        base = statistics.median(square)
        for r in group:
            r["ex"] = abs(r["raw_dx"] - base)
            r["x_base"] = base
    return rows, skipped


def report_bins(rows, clearance):
    print("  %-11s %7s %7s %-15s  %s"
          % ("|offset_y|", "succ", "rate", "Wilson 95%", "failure forms"))
    for lo, hi in BINS_MM:
        cell = [r for r in rows if lo <= r["ey"] < hi]
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


def report_x_budget(rows, clearance):
    """The second budget: how far off the pad axis the grasp survives.

    Restricted to trials that are inside the y clearance, because past that
    bound the y failure dominates and an x split would just be measuring it
    again. The failure forms separate the two: no_contact means neither jaw
    reached the cube, which is what running out of pad looks like.
    """
    inside_y = [r for r in rows if r["ey"] < clearance]
    if len(inside_y) < 20:
        print("  x budget: only %d trials inside the y clearance - no test"
              % len(inside_y))
        return
    print("  within |offset_y| < %.1fmm, by |offset_x| (n=%d):"
          % (clearance, len(inside_y)))
    for lo, hi in BINS_MM:
        cell = [r for r in inside_y if lo <= r["ex"] < hi]
        if not cell:
            continue
        k, n = sum(1 for r in cell if r["success"]), len(cell)
        wl, wh = wilson(k, n)
        forms = collections.Counter(r["category"] for r in cell
                                    if not r["success"])
        label = "%d-%dmm" % (lo, hi) if hi < 1000 else ">=%dmm" % lo
        print("    %-11s %3d/%-3d %6.1f%% [%3.0f%%,%3.0f%%]  %s"
              % (label, k, n, 100.0 * k / n, wl * 100, wh * 100,
                 ", ".join("%s x%d" % c for c in forms.most_common(3)) or "-"))


def split(rows, clearance, label, drop_non_grasp):
    """Test the y bound on trials that are inside the x bound.

    Without that condition the test is not about y at all: the open-loop x
    sweep puts 80 trials at a large x offset and a y offset near zero, so they
    land in the innermost y bin and fail there, and the split comes out
    backwards - 32.5% inside against 57.1% outside. Each bound has to be
    tested where the other one is satisfied.
    """
    used = [r for r in rows
            if not (drop_non_grasp and not r["success"]
                    and r["category"] in NON_GRASP_FAILURES)]
    if any(r.get("ex") is not None for r in used):
        before = len(used)
        used = [r for r in used
                if r.get("ex") is not None and r["ex"] < X_BOUND_MM]
        label += " (on the %d of %d inside |offset_x| < %gmm)" % (
            len(used), before, X_BOUND_MM)
    if not used:
        print("  %s: nothing left after conditioning - no test" % label)
        return
    inside = [r for r in used if r["ey"] < clearance]
    outside = [r for r in used if r["ey"] >= clearance]
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
        with_x = [r for r in cell if r.get("ex") is not None]
        if with_x:
            print("  fixed x term measured at %+.2f mm from %d square successes"
                  % (with_x[0]["x_base"],
                     sum(1 for r in cell
                         if r["ey"] < BASELINE_EY_MM and r["success"])))
        report_bins(cell, clearance)
        report_x_budget(with_x, clearance)
        split(cell, clearance, "split at the clearance, all trials", False)
        split(cell, clearance,
              "split at the clearance, non-grasp failures removed (%s)"
              % "/".join(NON_GRASP_FAILURES), True)

    # Pooled across clearances: the criterion is per-trial, so trials with
    # different clearances can be compared once each is judged against its own.
    print("\n=== pooled across clearances, each judged against its own ===")
    pool = rows
    if any(r.get("ex") is not None for r in rows):
        pool = [r for r in rows
                if r.get("ex") is not None and r["ex"] < X_BOUND_MM]
        print("  conditioned on |offset_x| < %gmm: %d of %d trials"
              % (X_BOUND_MM, len(pool), len(rows)))
    else:
        # No trustworthy x measure, so the y test cannot be run where the
        # x bound is satisfied. If any batch here varied x, its trials land
        # in the innermost y bin and fail there, and the pooled y result is
        # not about y. Say so rather than printing a contaminated number.
        misses = sum(1 for r in rows if r["category"] == "no_contact")
        print("  NOT conditioned on x: the baseline spread guard refused")
        print("  an |offset_x| for every clearance group here, so this")
        print("  pools y-varied and x-varied batches together. %d trials"
              % misses)
        print("  failed with no_contact, the x failure form - treat what")
        print("  follows as contaminated and run one batch at a time.")
    inside = [r for r in pool if r["ey"] < r["clearance"]]
    outside = [r for r in pool if r["ey"] >= r["clearance"]]
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
