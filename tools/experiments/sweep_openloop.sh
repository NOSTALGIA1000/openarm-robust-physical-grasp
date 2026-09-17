#!/usr/bin/env bash
# The open-loop tolerance curve, through the modern path, at a usable n.
#
# This is the baseline the whole comparison rests on and it is the thinnest
# thing in the repository: 113 open-loop trials exist, all from stage2/2b/2c,
# all at n=3 per cell, and all through run_s2*.sh, which injects the bias as a
# synthetic offset added to the picker's target rather than into the
# world->camera extrinsic where hand-eye error actually lives. The corrected
# arm has been re-measured many times since on the modern path; the baseline
# it is compared against has not been measured on that path even once.
#
# Falsifiable prediction, written before the run.
#
# The 2026-09-13 jaw-axis criterion says a grasp succeeds when the residual in
# the jaw-closing direction stays inside the per-side clearance, which is 15mm
# for the 20mm cube at open_position 0.025. In open loop with
# use_detector_target the target is the detector's world-frame object position,
# so residual_y is the camera bias plus the detector's own error - there is no
# correction to remove it. The criterion therefore predicts:
#
#   8mm   inside the clearance          -> succeeds at about the zero-bias rate
#   16mm  just outside                  -> falls sharply
#   24mm  well outside                  -> mostly fails, unilateral_contact
#
# The old curve disagrees: it put the knee at 24-32mm, with 3/3 succeeding at
# 24mm. But 3/3 has a Wilson interval of [43.8%, 100%], which contains the
# 45.2% the criterion gives for trials outside the budget, so the old data
# never contradicted the criterion - it simply had no power. n=20 here does.
#
# If 24mm still succeeds at 20 trials, the criterion is incomplete for the
# open-loop case and the reason has to be found: the corrected arm's residual
# carries the relative measurement's spread (about 10mm) while an open-loop
# residual is a near-pure translation, and a clean offset may be absorbed
# differently from a noisy one. That would be worth knowing.
EXPDIR="${EXPDIR:-$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)}"
source "$EXPDIR/env.sh"

REPEATS="${REPEATS:-20}"
FIRST="${FIRST:-1}"
# Which axis the bias goes on. y is the jaw-closing direction, where the
# budget is the per-side clearance; x is the pad-width direction, where the
# budget is whatever the pads span and has never been measured.
AXIS="${AXIS:-y}"
case "$AXIS" in
  # Bracketing the 15mm clearance: control, inside, just outside, well out.
  y) BIASES="${BIASES:-0 8 16 24}" ;;
  # The 2026-09-13 three-regime finding says res_x does not matter while
  # res_y is inside the clearance - but it cannot not matter without limit,
  # because at some x offset the jaws miss the cube entirely. That limit is
  # the second budget and completes the two-dimensional picture. The old
  # n=2 data puts it between 32 and 40mm (32mm 2/2, 40mm 0/2 no_contact),
  # which these levels bracket. All are well inside the x kinematic limit
  # of 112-128mm, so planning_failure should not appear; if it does, the
  # run hit IK rather than the pads and the levels need lowering.
  x) BIASES="${BIASES:-0 24 32 40}" ;;
  # Both axes at once, x = y = d, to ask whether the two budgets are
  # independent. Measured singles: y 90/90/85/26% and x -/100/80/0% at
  # 0/8/16/24mm. Two models, and the levels are chosen where they part:
  #
  #   rectangular  the budgets are separate and the failures multiply, so
  #                P(d) = P_y(d) * P_x(d)
  #   radial       there is one joint budget on hypot(d,d) = d*sqrt(2), so
  #                P(d) = P_single(d*1.414)
  #
  #   d= 8  hypot 11.3   rectangular 90%   radial ~88%   control, agree
  #   d=16  hypot 22.6   rectangular 68%   radial 10-40%
  #   d=20  hypot 28.3   rectangular ~22%  radial ~0%
  #
  # d=12 is deliberately absent: hypot 17.0 puts both models near 80% and
  # n=20 cannot separate them, so it would cost an hour and settle nothing.
  #
  # Both components positive. Opposite signs are not tested and the result
  # does not speak to them.
  xy) BIASES="${BIASES:-8 16 20}" ;;
  # The same y levels on the other side of zero. Every open-loop cell run so
  # far sits at y >= 0, which was listed as a limitation and has now become a
  # blocker: stage57_reljaw put 25 of the corrected arm's 46 measurable trials
  # at y < 0, with cell medians of -5.91mm at 128mm of camera bias and
  # -10.67mm at 240mm. The residual the correction leaves behind lives in the
  # half-plane the tolerance region was never measured in.
  #
  # Sign symmetry in y is more plausible than the x-y symmetry that died on
  # 2026-09-15 - the two fingers are symmetric about the tool axis, which the
  # closing and width directions are not - but plausible is what that one was
  # too. It cost +0.800 [+0.531, +0.919] to find out.
  #
  # Two levels, and they are the two stage56_jaw already ran on the positive
  # side: 16mm on the plateau and 24mm past the cliff. Run into the same
  # OUTDIR they are in, so the pair shares one batch, one baseline and one
  # reference frame - a cross-batch comparison here would mix the jaw centre
  # with the link origin, which differ by about 3.5mm, on a question whose
  # whole content is a few millimetres. 21mm is left out: stage54 has it on
  # the positive side but under the other reference, so pairing it would cost
  # an extra hour and a half to compare two different rulers.
  #
  # Prediction: 90% at -16mm and 5% at -24mm, matching the positive side
  # within sampling error. If either departs, the region is not sign-symmetric
  # and every corrected-arm comparison has to be made against the y < 0 curve
  # rather than the one already published.
  #
  # Outcome, 2026-09-17: -16 gave 20/20 and -24 gave 9/20, against 18/20 and
  # 1/20 on the positive side. Read off the COMMAND that is a flat contradiction
  # of the prediction, +0.400 [+0.133, +0.612]. Read off the ACHIEVED offset it
  # is not a contradiction at all, because the gain from command to achieved
  # offset differs by sign: +16 and +24 land at +17.26 and +24.83, while -16 and
  # -24 land at -14.61 and -22.71. The negative cliff cell sits 2.1mm short of
  # the positive one on a stretch where the curve falls from 75% to 15% in 3mm.
  # Matched on achieved offset the two sides agree, +0.150 [-0.141, +0.409]
  # against one interpolation of the positive curve and -0.150 [-0.414, +0.148]
  # against the other.
  #
  # So the gap is narrowed, not closed: the interval is about eighty points
  # wide because nothing was measured at a matched offset. -26mm is what closes
  # it. The gain measured on the two negative cells is 0.913 and 0.946, so -26
  # should land near -24.6 and can be compared with +24.83 directly instead of
  # through an interpolation of a curve that is nearly vertical there.
  #
  # Prediction for -26: between 5% and 26%, the two positive readings at that
  # offset. If it comes back at 45% again, the sides really do differ and the
  # difference is not the offset.
  yneg) BIASES="${BIASES:-16 24}" ;;
  # Off the diagonal, to test whether the joint region is symmetric at
  # all. Every single-metric model - hypot, L1, anything on one radius -
  # requires (ex, ey) and (ey, ex) to score the same. Rectangular does
  # not: with the measured curves it puts x=8,y=20 at P_x(8)*P_y(20) =
  # 1.00*0.75 = 75%, and x=20,y=8 at P_x(20)*P_y(8) = 0.18*0.90 = 16%.
  #
  # The two cells agreeing kills rectangular again; the two differing
  # kills every symmetric metric at once. PAIRS is "bx:by" in mm.
  pair) PAIRS="${PAIRS:-8:20 20:8}" ;;
  *) echo "AXIS must be x, y, yneg, xy or pair, got '$AXIS'" >&2; exit 2 ;;
esac
OUTDIR="${OUTDIR:-stage54_openloop}"
SIDE="${SIDE:-20}"
export OPEN_POSITION="${OPEN_POSITION:-0.025}"
export LIFT_COMMAND_M="${LIFT_COMMAND_M:-0.050}"
export REHOVER=true
export DETECTOR_ROI_CASCADE=90,55,30
# The point of the batch. run_rel.sh defaults this to true.
export RELATIVE_CORRECTION=false

python3 "$EXPDIR/make_object_variants.py" --open "$OPEN_POSITION" \
  "$OPENARM_MODEL/table_scene_physics.xml" "$OPENARM_MODEL" "$SIDE" || exit 6
scene=$(printf "table_scene_c%03d.xml" "$SIDE")
# The rebuild of 2026-09-16 made colcon symlink each installed model file
# back to src, so this copy became a file onto itself and cp refused,
# taking the sweep down with it. Skip when they already resolve to the
# same file; still copy when the install holds a real separate copy.
if [ "$(readlink -f "$OPENARM_MODEL/$scene")" != "$(readlink -f "$OPENARM_SHARE/model/$scene" 2>/dev/null)" ]; then
  cp "$OPENARM_MODEL/$scene" "$OPENARM_SHARE/model/$scene" || exit 6
fi
export PHYSICS_SCENE="$scene"
# Derived values come from the generator's sidecar, so the rules that produce
# them live in one file.
set -a; . "$OPENARM_MODEL/${scene%.xml}.env"; set +a

# rel_offset is inert with relative_correction off, but pass the fitted values
# anyway so the config snapshot matches the corrected batches field for field
# and analyse_crosssize.py's guard will pair them.
OFFX="${OFFX:--0.0209}"
OFFY="${OFFY:--0.0007}"

if [ "$AXIS" = "pair" ]; then
  for p in $PAIRS; do
    mx="${p%%:*}"; my="${p##*:}"
    bx=$(mm_to_m "$mx"); by=$(mm_to_m "$my")
    echo "=== open loop, pair x=${mx}mm y=${my}mm (bx=$bx by=$by) ==="
    for r in $(seq "$FIRST" "$REPEATS"); do
      bash "$EXPDIR/run_rel.sh" \
        "$(printf 'ol_p%02dx%02d_%s' "$mx" "$my" "$r")" \
        "$bx" "$by" "$OUTDIR" "$OFFX" "$OFFY" 2>&1 \
        | grep -E 'INSTABILITY|trial .* ->' | tail -2
    done
  done
else
for m in $BIASES; do
  v=$(mm_to_m "$m")
  tag="$AXIS"
  case "$AXIS" in
    x)  bx="$v";      by="0.0000" ;;
    y)  bx="0.0000";  by="$v" ;;
    xy) bx="$v";      by="$v" ;;
    # mm_to_m carries the sign through awk, so this is the one place the
    # leading-zero trap of bc does not bite. The trial id spells the sign as
    # 'n' because a minus in a directory name does not survive the analysis
    # regex, and a cell silently dropped is worse than one named oddly.
    yneg) bx="0.0000"; by=$(mm_to_m "-$m"); tag="yn" ;;
  esac
  echo "=== open loop, ${AXIS} bias ${m}mm (bx=$bx by=$by) ==="
  for r in $(seq "$FIRST" "$REPEATS"); do
    bash "$EXPDIR/run_rel.sh" "$(printf 'ol_%s%03d_%s' "$tag" "$m" "$r")" \
      "$bx" "$by" "$OUTDIR" "$OFFX" "$OFFY" 2>&1 \
      | grep -E 'INSTABILITY|trial .* ->' | tail -2
  done
done
fi
unset PHYSICS_SCENE REHOVER DETECTOR_ROI_CASCADE RELATIVE_CORRECTION
echo OPENLOOP_DONE
