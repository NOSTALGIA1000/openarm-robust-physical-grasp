"""Put the corrected arm's residuals on the open-loop tolerance region.

This is what the whole stage54-57 line was for. The open-loop batches measure
how often a grasp succeeds at a given approach offset; the corrected arm
produces offsets it does not choose. The question is whether its success rate
is the one the geometry predicts at the offsets it actually reaches.

No region model is used, because none survived 2026-09-15 - rectangular died on
the diagonal, every symmetric metric died off it. Instead each corrected trial
is matched against the open-loop trials that landed near it, and the local
success rate among those neighbours is the prediction. That assumes only that
success depends on the offset, which is the hypothesis under test, and not on
any particular shape for how.

A trial with too few neighbours is reported as uncovered rather than given a
prediction from further away. The open-loop grid runs along the two axes and
the diagonal; the corrected arm scatters between them, so some of it falls in
gaps. Saying so is the point - an extrapolated prediction here would be a guess
wearing a number, which is the failure this project has made repeatedly.

Trials that never produced a target are excluded from the comparison and
counted separately. detector_timeout is a real failure of the corrected
pipeline and is reported in the batch's own results, but it has no offset, so
it cannot be placed on a region and must not be silently scored as a
geometric failure.

Both references are pooled. stage54 reads the gripper link origin and stage56
the jaw centre, but each has its own arm-term baseline subtracted, and on the
six cells they share the offsets agree to 1.4mm in x and 0.7mm in y with no
separable difference in success rate on any of them.

Usage:
    python tools/experiments/analyse_placement.py <corrected batch> <open-loop batch> [...]

    python tools/experiments/analyse_placement.py \
        docs/results/physics_grasp/stage57_reljaw \
        docs/results/physics_grasp/stage54_openloop \
        docs/results/physics_grasp/stage56_jaw
"""

import collections
import glob
import json
import os
import sys

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from analyse_joint_axes import achieved, measure_baseline, newcombe, wilson

# Half-width of the matching box, per axis, in millimetres, and the smallest
# neighbourhood that gets a prediction.
#
# 4mm is a compromise and it is worth being explicit about which way it errs.
# On the plateau the curve is flat and 4mm costs nothing. At the cliff it falls
# about 25 points per millimetre, so a 4mm box there averages over a range in
# which the true rate moves from most to none. Predictions for trials near the
# cliff are therefore soft, and the script prints the neighbourhood's own
# spread of offsets so a wide one is visible rather than implied.
BOX_MM = 4.0
MIN_NEIGHBOURS = 10


def offsets(batch):
    """[(ex, ey, success)] in mm for one batch, on its own arm-term baseline."""
    trials = []
    all_dirs = []
    for path in sorted(glob.glob(os.path.join(batch, "*", "trial.json"))):
        rec = json.loads(open(path, encoding="utf-8").read())
        trial_dir = os.path.dirname(path)
        all_dirs.append(trial_dir)
        trials.append((rec, trial_dir))
    base = measure_baseline(all_dirs, [])
    if base is None:
        return None, None
    out = []
    missing = 0
    for rec, trial_dir in trials:
        got = achieved(trial_dir)
        if got is None:
            missing += 1
            continue
        out.append((abs(got[0] - base[0]), abs(got[1] - base[1]),
                    bool(rec.get("success")), rec.get("category", "?")))
    return out, (base, missing)


def main():
    if len(sys.argv) < 3:
        sys.exit(__doc__)
    corrected, reference_batches = sys.argv[1], sys.argv[2:]

    pool = []
    for batch in reference_batches:
        rows, info = offsets(batch)
        if rows is None:
            print("%s: no measurable baseline, skipped" % batch)
            continue
        base, missing = info
        print("reference %s: %d trials, baseline dx %+.2f dy %+.2f (%s)"
              % (batch, len(rows), base[0], base[1], base[2]))
        pool.extend(rows)
    if not pool:
        print("no reference data")
        return 1

    rows, info = offsets(corrected)
    if rows is None:
        print("%s: no measurable baseline" % corrected)
        return 1
    base, missing = info
    print("corrected %s: %d trials with an offset, %d without (no target "
          "reached), baseline dx %+.2f dy %+.2f"
          % (corrected, len(rows), missing, base[0], base[1]))
    print("")

    covered = []
    uncovered = []
    for ex, ey, success, category in rows:
        near = [r for r in pool
                if abs(r[0] - ex) <= BOX_MM and abs(r[1] - ey) <= BOX_MM]
        if len(near) < MIN_NEIGHBOURS:
            uncovered.append((ex, ey, success, category, len(near)))
            continue
        rate = sum(1 for r in near if r[2]) / len(near)
        spread = (max(r[0] for r in near) - min(r[0] for r in near),
                  max(r[1] for r in near) - min(r[1] for r in near))
        covered.append((ex, ey, success, rate, len(near), spread))

    print("%d of %d placed, %d in gaps of the open-loop grid"
          % (len(covered), len(rows), len(uncovered)))
    if uncovered:
        print("  gaps, by offset:")
        for ex, ey, success, category, n in sorted(uncovered):
            print("    (%5.2f, %5.2f)  %-20s %d neighbours"
                  % (ex, ey, category if not success else "success", n))
    if not covered:
        print("nothing could be placed")
        return 0

    k = sum(1 for c in covered if c[2])
    n = len(covered)
    expected = sum(c[3] for c in covered)
    lo, hi = wilson(k, n)
    print("")
    print("observed  %d/%d = %.1f%%  Wilson [%.0f, %.0f]%%"
          % (k, n, 100.0 * k / n, 100 * lo, 100 * hi))
    print("predicted %.1f/%d = %.1f%%  from the open-loop neighbourhoods"
          % (expected, n, 100.0 * expected / n))
    diff, dlo, dhi = newcombe(k, n, int(round(expected)), n)
    print("difference %+.3f [%+.3f, %+.3f]  %s"
          % (diff, dlo, dhi,
             "SEPARABLE - the offset does not account for the corrected arm's "
             "outcome" if (dlo > 0 or dhi < 0)
             else "not separable - the corrected arm performs as its offsets "
                  "predict"))
    print("")
    print("%-16s %-8s %-9s %-6s %s"
          % ("corrected trial", "outcome", "predicted", "n", "neighbourhood spread"))
    for ex, ey, success, rate, count, spread in sorted(covered):
        print("(%5.2f, %5.2f)   %-8s %8.0f%% %5d   %.1f x %.1fmm"
              % (ex, ey, "success" if success else "fail", 100 * rate, count,
                 spread[0], spread[1]))
    return 0


if __name__ == "__main__":
    sys.exit(main())
