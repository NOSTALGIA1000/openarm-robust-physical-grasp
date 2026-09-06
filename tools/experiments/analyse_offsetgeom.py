#!/usr/bin/env python3
"""Decide whether rel_offset tracks the viewing angle or the calibration bias.

Reads the trial records written by sweep_offsetgeom.sh. Those run with zero
calibration offsets, so each residual IS the inherent surface-versus-origin
offset for that trial, and view_tan is the viewing direction taken from
grip_cam, which never passes through the extrinsic.

If the geometric account holds, residual / sin(theta) is constant across bias.
If instead the residual itself is constant, or neither is, the account fails
and the calibration-free motivation has to be reconsidered.

Only readings at the expected gripper depth are used. The residual bimodality
spans +/-15mm while the effect under test is about 7mm, so the shallow group
has no resolving power; the depth criterion is from the 2026-09-06 root-cause
document.

Usage: analyse_offsetgeom.py <batch dir> [--depth-min 0.66]
"""
import argparse
import json
import math
import pathlib
import statistics
import sys


def load(batch_dir, depth_min):
    kept, dropped = {}, {}
    for path in sorted(pathlib.Path(batch_dir).glob("*/trial.json")):
        rec = json.loads(path.read_text())
        m = rec.get("measurements") or {}
        if not m:
            print("no measurements in %s - was it run before "
                  "record_measurement existed?" % rec.get("trial_id"),
                  file=sys.stderr)
            continue
        parts = rec["trial_id"].split("_")          # og_<scheme>_x<bias>_<rep>
        key = (parts[1], int(parts[2][1:]))
        bucket = kept if m.get("grip_cam_z", 0.0) >= depth_min else dropped
        bucket.setdefault(key, []).append(m)
    return kept, dropped


def summarise(rows):
    resid = [r["residual_x"] for r in rows]
    tans = [r["view_tan"] for r in rows]
    sins = [t / math.sqrt(1.0 + t * t) for t in tans]
    ratios = [r / s for r, s in zip(resid, sins) if s]
    sd = lambda v: statistics.stdev(v) if len(v) > 1 else 0.0
    return {
        "n": len(rows),
        "resid_mm": statistics.mean(resid) * 1000.0,
        "resid_sd_mm": sd(resid) * 1000.0,
        "sin": statistics.mean(sins),
        "ratio_mm": statistics.mean(ratios) * 1000.0,
        "ratio_sd_mm": sd(ratios) * 1000.0,
        "ratio_se_mm": sd(ratios) * 1000.0 / math.sqrt(len(ratios)) if ratios else 0.0,
    }


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("batch_dir")
    ap.add_argument("--depth-min", type=float, default=0.66,
                    help="keep readings at or below this gripper depth "
                         "(default 0.66: good group ~0.682, bad ~0.640)")
    args = ap.parse_args()

    kept, dropped = load(args.batch_dir, args.depth_min)
    if not kept:
        sys.exit("no usable readings")

    for scheme in sorted({k[0] for k in kept}):
        print("\n=== %s ===" % scheme)
        print("%6s %4s %4s %12s %10s %14s" %
              ("bias", "n", "drop", "residual mm", "sin(th)",
               "residual/sin mm"))
        ratios, resids, ses = [], [], []
        for bias in sorted(b for s, b in kept if s == scheme):
            s = summarise(kept[(scheme, bias)])
            ratios.append(s["ratio_mm"])
            resids.append(s["resid_mm"])
            ses.append(s["ratio_se_mm"])
            print("%6d %4d %4d %7.2f+-%-4.2f %10.4f %8.2f+-%-5.2f" % (
                bias, s["n"], len(dropped.get((scheme, bias), [])),
                s["resid_mm"], s["resid_sd_mm"], s["sin"],
                s["ratio_mm"], s["ratio_sd_mm"]))
        if len(ratios) > 1:
            # Judge the spread against the sampling error rather than a fixed
            # threshold: with a dozen trials per point the means wander by a
            # millimetre or two on their own, and a constant residual - the
            # competing account - shows up as a ratio that falls as sin(theta)
            # grows, so both are reported.
            spread = max(ratios) - min(ratios)
            tol = 2.0 * max(ses) if max(ses) else 0.0
            r_spread = max(resids) - min(resids)
            print("  ratio  spread %.2f mm vs 2*SE %.2f mm -> %s" % (
                spread, tol,
                "constant: consistent with rel_offset = w*sin(theta)"
                if spread <= tol else
                "NOT constant: the geometric account fails for this scheme"))
            print("  residual spread %.2f mm -> %s" % (
                r_spread,
                "residual itself is constant; rel_offset does NOT track the "
                "viewing angle" if r_spread <= tol else
                "residual moves with bias, as the account requires"))


if __name__ == "__main__":
    main()
