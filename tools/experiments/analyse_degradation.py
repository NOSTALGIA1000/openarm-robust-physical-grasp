"""Is the failure gradual or a cliff?

docs/related-work.md section 3 lists "failure is gradual, not the sudden
collapse of a mis-fitted model" as a claim that must be proven from data or
struck. This tests it against the whole corpus rather than the one batch that
was in mind when the claim was written, because the field-of-view ceiling
(12/12 at 240mm, 9/12 at 288mm, 0/12 at 320mm) looks like a counterexample.

Gradual means: the success rate declines over several levels, and the failure
categories pass through intermediate forms before total miss. A cliff means one
level is fine, the next is zero, with no level and no failure form in between.

Where the bias comes from
-------------------------
This project injects the hand-eye error two different ways, and only one of
them was ever recorded:

  config.calib_bias_x/y   a picker parameter, a synthetic offset added to the
                          target. Recorded. Used by the old run_s2*.sh sweeps.
  CAM_BIAS_X/Y            an environment variable the launch file folds into
                          the world->camera_base_link static transform, which
                          is where hand-eye error physically lives. Used by
                          run_rel.sh and every modern sweep. Recorded in
                          environment.cam_bias_x/y only from 2026-09-12; before
                          that, nowhere.

So for every batch that produced the headline numbers, config.calib_bias_y is
0.0 and the independent variable survives only in the directory name. This
script prefers the recorded environment field, falls back to parsing the trial
id, and prints which source it used - a first run that silently grouped 240mm
and 320mm together as "0mm" is how this was found.

Usage:
    python tools/experiments/analyse_degradation.py [results_root]
    MIN_N=8 python tools/experiments/analyse_degradation.py
"""

import collections
import glob
import json
import math
import os
import re
import sys

TRIAL_ID_BIAS = re.compile(r"_([xy])(\d+)_\d+$")

# Pooling every batch at one bias level is wrong here, and the corpus makes it
# obvious: the 128mm cell holds 446 trials because it is the working point of
# the clearance, noise and ablation sweeps at once - including the arm with
# re-hover removed, which scored 0/20 by design. Pooled, 128mm then reads
# WORSE than 240mm, which is an artefact of the mixture, not degradation. So
# trials are grouped by configuration first and only compared within a group.
#
# These are the fields that were deliberately varied in some sweep. Fields the
# picker does not see - the detector ROI cascade and the depth noise, which are
# detector parameters - are not in config at all, so the arm tag from the trial
# id is part of the signature too: it is how each sweep script labelled the arm
# it was varying.
SIG_KEYS = ("relative_correction", "visual_correction",
            "rehover_after_correction", "pregrasp_redetect",
            "rel_offset_x", "rel_offset_y",
            "view_gain_gripper", "view_gain_object", "open_position",
            "close_position", "detector_z_bias", "object_geom",
            "lift_command_m", "lift_height_m", "vision_noise_std",
            "servo_baseline", "use_detector_target")

# Trial ids were named by several generations of sweep script, and the arm tag
# has to come out of them the same way regardless of generation. Two mistakes
# already made here:
#
#   * Leaving visual_correction out of SIG_KEYS put nine visually-corrected
#     trials in an open-loop group, where the label came from whichever record
#     happened to be first.
#   * A single pattern turned px008, px016 ... px048 into six separate "arms",
#     when they are six bias levels of one sweep with the magnitude written
#     into the tag. Each became a one-level group and the whole open-loop x
#     curve vanished from the report.
#
# So: strip the repetition suffix, then strip a trailing segment that encodes
# the bias rather than the arm.
REP_SUFFIX = re.compile(r"_r?\d+[a-z]?$")
BIAS_DIGITS = re.compile(r"\d+$")
# _x, _y and _b said which axis the magnitude was on and every such sweep ran
# the positive direction, so they carry no arm information. px/nx do carry it -
# positive and negative x are genuinely different arms - so they survive.
BARE_AXIS = re.compile(r"_[xyb]$")


def wilson(k, n, z=1.96):
    if n == 0:
        return (0.0, 1.0)
    p = k / n
    d = 1.0 + z * z / n
    c = p + z * z / (2 * n)
    h = z * math.sqrt(p * (1 - p) / n + z * z / (4 * n * n))
    return ((c - h) / d, (c + h) / d)


def newcombe(k1, n1, k2, n2, z=1.96):
    """Hybrid score interval for a difference of proportions.

    Verified against Newcombe 1998's worked example, 56/70 vs 48/80 ->
    [0.0524, 0.3339]; see docs/statistics.md.
    """
    p1, p2 = k1 / n1, k2 / n2
    l1, u1 = wilson(k1, n1, z)
    l2, u2 = wilson(k2, n2, z)
    d = p1 - p2
    return (d,
            d - math.sqrt((p1 - l1) ** 2 + (u2 - p2) ** 2),
            d + math.sqrt((u1 - p1) ** 2 + (p2 - l2) ** 2))


def separated(k1, n1, k2, n2):
    _, lo, hi = newcombe(k1, n1, k2, n2)
    return lo > 0.0 or hi < 0.0


Z_ALPHA = 1.959963985   # two-sided alpha = 0.05
Z_BETA = 0.8416212336   # power = 0.80


def n_for_power(p1, p2):
    """Trials per cell to have an 80% chance of detecting p1 vs p2.

    The same two-proportion formula behind the table in docs/statistics.md,
    which it reproduces exactly: 100/75 -> 27, 100/50 -> 11, 90/60 -> 32,
    95/70 -> 36, 80/50 -> 39.

    A first version answered a different question - "if I observe exactly
    these rates at n per cell, does the Newcombe interval exclude zero" - and
    returned 14 for 100 vs 75 against the repo's own documented 27, because
    round(0.75*14) is 10 under banker's rounding, so it was really testing
    100% against 71.4%. How many more trials to run is a design question, and
    a design question needs power, not a post-hoc interval.
    """
    d = abs(p1 - p2)
    if d == 0.0:
        return None
    pooled = (p1 + p2) / 2.0
    a = Z_ALPHA * math.sqrt(2.0 * pooled * (1.0 - pooled))
    b = Z_BETA * math.sqrt(p1 * (1.0 - p1) + p2 * (1.0 - p2))
    return int(math.ceil((a + b) ** 2 / (d * d)))


def bias_of(record):
    """Returns (bx_m, by_m, source). Prefers what was recorded."""
    cfg = record.get("config") or {}
    env = record.get("environment") or {}
    if "cam_bias_x" in env or "cam_bias_y" in env:
        return (float(env.get("cam_bias_x", 0.0)),
                float(env.get("cam_bias_y", 0.0)), "environment")
    cbx = float(cfg.get("calib_bias_x", 0.0))
    cby = float(cfg.get("calib_bias_y", 0.0))
    if cbx or cby:
        return (cbx, cby, "config")
    m = TRIAL_ID_BIAS.search(record.get("trial_id", ""))
    if m:
        v = int(m.group(2)) / 1000.0
        if m.group(1) == "x":
            return (v, 0.0, "trial_id")
        return (0.0, v, "trial_id")
    return (cbx, cby, "config")


def arm_tag_of(trial_id):
    """cl_cal_y288_3 -> cl_cal, px048_r2 -> px, xg_px080_2b -> xg_px."""
    tag = REP_SUFFIX.sub("", trial_id, count=1) or trial_id
    tag = BIAS_DIGITS.sub("", tag, count=1) or tag
    return BARE_AXIS.sub("", tag, count=1) or tag or "?"


def load(root):
    out = []
    for f in sorted(glob.glob(os.path.join(root, "*", "*", "trial.json"))):
        try:
            d = json.load(open(f, encoding="utf-8"))
        except Exception:
            continue
        cfg = d.get("config") or {}
        if "calib_bias_x" not in cfg:
            continue
        bx, by, src = bias_of(d)
        out.append({
            "batch": f.replace("\\", "/").split("/")[-3],
            "trial": d.get("trial_id", ""),
            "category": d.get("category", "?"),
            "success": bool(d.get("success")),
            "bx": bx,
            "by": by,
            "source": src,
            "corrected": bool(cfg.get("relative_correction")
                              or cfg.get("visual_correction")),
            "servo": bool(cfg.get("servo_baseline")),
            "arm": arm_tag_of(d.get("trial_id", "")),
            "sig": tuple(cfg.get(k) for k in SIG_KEYS),
        })
    return out


def axis_of(t):
    bx, by = abs(t["bx"]), abs(t["by"])
    if bx < 1e-9 and by < 1e-9:
        return "zero"
    if by < 1e-9:
        return "x"
    if bx < 1e-9:
        return "y"
    return "xy"


def verdict(levels, min_n):
    """levels: [(bias_mm, k, n)] ascending. Judges over EVERY level.

    Two earlier versions of this function were wrong in the two ways this
    project keeps repeating:

      1. It filtered to n>=min_n before looking for a zero level, which
         discarded exactly the thin top-end cells where the collapse happens,
         and then printed "never reaches zero" above a table showing 0%.
      2. It called a level intermediate on the point estimate alone. For
         cl_cal that named 288mm (9/12) intermediate between 240mm (12/12) and
         320mm (0/12) - but the Newcombe interval for 240 vs 288 is
         [-0.041, +0.532], straddling zero, so at n=12 those two levels are
         not distinguishable at all. Gradualness is a claim about rates and so
         needs an interval, the same lesson as the earlier "do the two Wilson
         intervals overlap" verdict that was replaced.

    A level is intermediate only if it separates from BOTH neighbours.
    """
    if len(levels) < 3:
        return "only %d level(s) - cannot judge a shape" % len(levels)
    thin = [b for b, _, n in levels if n < min_n]
    note = ""
    if thin:
        note = ("  [thin cells (n<%d) at %s mm]"
                % (min_n, ",".join("%g" % b for b in thin)))
    dead = [b for b, k, _ in levels if k == 0]
    if not dead:
        return ("no level reached zero in %g-%g mm - the ceiling is outside "
                "this range%s" % (levels[0][0], levels[-1][0], note))
    first_dead = min(dead)
    alive = [(b, k, n) for b, k, n in levels if b < first_dead]
    if not alive:
        return "zero at the lowest level tested - cannot judge%s" % note
    last_b, last_k, last_n = alive[-1]
    step = first_dead - last_b

    # Candidate intermediates: below the first dead level, not at the top rate.
    established, weak = [], []
    for i, (b, k, n) in enumerate(levels):
        if b >= first_dead or i == 0:
            continue
        lo_b, lo_k, lo_n = levels[i - 1]
        hi_b, hi_k, hi_n = levels[i + 1]
        if separated(lo_k, lo_n, k, n) and separated(k, n, hi_k, hi_n):
            established.append(b)
        elif 0 < k < n:
            weak.append((b, lo_b, n_for_power(lo_k / lo_n, k / n)))

    if established:
        return ("GRADUAL, established: %s mm separates from both neighbours; "
                "%d/%d at %g mm -> 0/%d at %g mm (step %g mm)%s"
                % (",".join("%g" % b for b in established), last_k, last_n,
                   last_b, levels[-1][2], first_dead, step, note))
    if weak:
        detail = "; ".join(
            "%g mm does not separate from %g mm (n=%s per cell for 80%% power)"
            % (b, lo_b, need if need else "?")
            for b, lo_b, need in weak)
        return ("GRADUAL in point estimates only, NOT established: %s%s"
                % (detail, note))
    return ("CLIFF: no partial level at all; %d/%d at %g mm -> 0/%d at %g mm "
            "(step %g mm)%s"
            % (last_k, last_n, last_b, levels[-1][2], first_dead, step, note))


def report(name, trials, min_n):
    if not trials:
        return
    by_mag = collections.defaultdict(list)
    for t in trials:
        by_mag[round(1000.0 * math.hypot(t["bx"], t["by"]), 1)].append(t)
    srcs = collections.Counter(t["source"] for t in trials)
    print("\n=== %s  (n=%d, %d level(s), bias from %s) ==="
          % (name, len(trials), len(by_mag),
             ", ".join("%s x%d" % s for s in srcs.most_common())))
    print("  bias mm    n   succ   rate   Wilson 95%    dominant failure forms")
    levels = []
    for mag in sorted(by_mag):
        ts = by_mag[mag]
        k = sum(1 for t in ts if t["success"])
        lo, hi = wilson(k, len(ts))
        cats = collections.Counter(t["category"] for t in ts if not t["success"])
        forms = ", ".join("%s x%d" % c for c in cats.most_common(3)) or "-"
        print("  %7.1f  %3d  %4d  %5.1f%%  [%.2f, %.2f]  %s"
              % (mag, len(ts), k, 100.0 * k / len(ts), lo, hi, forms))
        levels.append((mag, k, len(ts)))
    print("  -> %s" % verdict(levels, min_n))

    # The rate question and the mechanism question are separate, and the
    # second can be settled where the first cannot: whether the detector still
    # finds the gripper is a per-trial binary that does not depend on how the
    # grasp then went. For cl_cal this is 0/12 timeouts at 288mm against 12/12
    # at 320mm - decisive - while the success rates at 240 and 288 are not
    # separable at all. So report it on its own line rather than folding it
    # into the shape verdict.
    dead = [b for b, k, _ in levels if k == 0]
    if not dead:
        return
    first_dead = min(dead)
    alive = [b for b, _, _ in levels if b < first_dead]
    if not alive:
        return
    last_b = alive[-1]

    def timeouts(mag):
        ts = by_mag[mag]
        return (sum(1 for t in ts if t["category"] == "detector_timeout"),
                len(ts))

    a_k, a_n = timeouts(last_b)
    d_k, d_n = timeouts(first_dead)
    mark = "decisive" if separated(a_k, a_n, d_k, d_n) else "not separable"
    print("     detector lost the gripper: %d/%d at %g mm vs %d/%d at %g mm "
          "(%s)" % (a_k, a_n, last_b, d_k, d_n, first_dead, mark))


def main():
    root = sys.argv[1] if len(sys.argv) > 1 else os.path.join(
        "docs", "results", "physics_grasp")
    min_n = int(os.environ.get("MIN_N", "8"))
    trials = load(root)
    if not trials:
        print("no trials carrying a calib_bias field under %s" % root)
        return 1
    srcs = collections.Counter(t["source"] for t in trials)
    print("loaded %d trials from %s" % (len(trials), root))
    print("bias source: %s"
          % ", ".join("%s x%d" % s for s in srcs.most_common()))
    if srcs["trial_id"]:
        print("NOTE: %d trials carry their bias only in the directory name, "
              "because CAM_BIAS_X/Y was not recorded before 2026-09-12."
              % srcs["trial_id"])

    # One curve per (arm, configuration, axis). A curve that spans fewer than
    # three bias levels cannot show a shape, so it is counted and skipped
    # rather than printed - and the count is printed, because "we only looked
    # at the arms that span the cliff" is a selection the reader must see.
    groups = collections.defaultdict(list)
    for t in trials:
        axis = axis_of(t)
        if axis == "zero":
            continue
        groups[(t["arm"], t["sig"], axis)].append(t)

    shown = skipped = skipped_trials = 0
    for key in sorted(groups, key=lambda k: (-len(groups[k]), str(k[0]))):
        arm, _sig, axis = key
        ts = groups[key]
        levels = len({round(1000.0 * math.hypot(t["bx"], t["by"]), 1)
                      for t in ts})
        if levels < 3:
            skipped += 1
            skipped_trials += len(ts)
            continue
        shown += 1
        # Every trial in a group shares the signature, so corrected/servo are
        # constant within it; assert rather than trust the first record.
        assert len({(t["corrected"], t["servo"]) for t in ts}) == 1, \
            "group %s mixes corrected/servo - SIG_KEYS is incomplete" % (key,)
        label = "OPEN LOOP" if not ts[0]["corrected"] else (
            "ECL SERVO" if ts[0]["servo"] else "CORRECTED")
        report("%s | arm %s | axis %s" % (label, arm, axis), ts, min_n)

    print("\n%d curve(s) shown; %d group(s) covering %d trial(s) span fewer "
          "than 3 bias levels and cannot show a shape."
          % (shown, skipped, skipped_trials))
    print("%d trial(s) at zero bias are excluded from every curve."
          % sum(1 for t in trials if axis_of(t) == "zero"))
    return 0


if __name__ == "__main__":
    sys.exit(main())
