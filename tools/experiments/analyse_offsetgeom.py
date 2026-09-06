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
        # Not "is the dict empty": the seed is recorded early, so a trial that
        # fails before the correction still carries a measurements block. Ask
        # for the reading itself.
        if "grip_cam_z" not in m:
            print("no optical reading in %s (%s)" % (
                rec.get("trial_id"), rec.get("category")), file=sys.stderr)
            continue
        parts = rec["trial_id"].split("_")   # og_<scheme>_<axis><bias>_<rep>
        key = (parts[1], int(parts[2][1:]))
        bucket = kept if m.get("grip_cam_z", 0.0) >= depth_min else dropped
        bucket.setdefault(key, []).append(m)
    return kept, dropped


def summarise(rows, axis):
    key = "residual_%s" % axis
    resid = [r[key] for r in rows]
    depth = [r["grip_cam_z"] for r in rows]
    tans = [r["view_tan"] for r in rows]
    sins = [t / math.sqrt(1.0 + t * t) for t in tans]
    ratios = [r / s for r, s in zip(resid, sins) if s]
    sd = lambda v: statistics.stdev(v) if len(v) > 1 else 0.0
    return {
        "n": len(rows),
        "resid_mm": statistics.mean(resid) * 1000.0,
        "resid_sd_mm": sd(resid) * 1000.0,
        "resid_se_mm": sd(resid) * 1000.0 / math.sqrt(len(resid)),
        "depth": statistics.mean(depth),
        "sin": statistics.mean(sins),
        "ratio_mm": statistics.mean(ratios) * 1000.0,
    }


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("batch_dir")
    ap.add_argument("--axis", default="y", choices=("x", "y"),
                    help="which residual component to report (default y)")
    ap.add_argument("--depth-min", type=float, default=0.66,
                    help="keep readings at or below this gripper depth "
                         "(default 0.66: good group ~0.682, bad ~0.640)")
    args = ap.parse_args()

    kept, dropped = load(args.batch_dir, args.depth_min)
    if not kept:
        sys.exit("no usable readings")

    for scheme in sorted({k[0] for k in kept}):
        print("")
        print("=== %s (axis %s) ===" % (scheme, args.axis))
        print("%6s %4s %5s %14s %9s %9s" % (
            "bias", "n", "drop", "residual mm", "grip_z", "resid/sin"))
        resids, ses, depths = [], [], []
        for bias in sorted(b for s_, b in kept if s_ == scheme):
            st = summarise(kept[(scheme, bias)], args.axis)
            resids.append(st["resid_mm"])
            ses.append(st["resid_se_mm"])
            depths.append(st["depth"])
            print("%6d %4d %5d %8.2f+-%-5.2f %9.4f %9.2f" % (
                bias, st["n"], len(dropped.get((scheme, bias), [])),
                st["resid_mm"], st["resid_sd_mm"], st["depth"],
                st["ratio_mm"]))
        if len(resids) > 1:
            # The question is no longer whether residual/sin(theta) is
            # constant - that model came from a geometric account the zero-bias
            # data refuted. What matters now is whether the residual stays put
            # as bias grows, and whether the reading drifts shallow while it
            # moves, which is the signature of the window sliding off the
            # gripper onto the forearm.
            spread = max(resids) - min(resids)
            tol = 2.0 * max(ses)
            print("  residual spread %.2f mm vs 2*SE %.2f mm -> %s" % (
                spread, tol,
                "flat: the offset does not track the bias"
                if spread <= tol else
                "grows with bias: something in the pipeline still follows it"))
            print("  depth %.4f -> %.4f (%+.4f) -> %s" % (
                depths[0], depths[-1], depths[-1] - depths[0],
                "reading creeps shallow, consistent with forearm entering the "
                "window" if depths[-1] < depths[0] - 0.005 else
                "depth holds"))


if __name__ == "__main__":
    main()
