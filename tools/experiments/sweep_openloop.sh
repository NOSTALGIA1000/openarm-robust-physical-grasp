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
  # Off the diagonal, to test whether the joint region is symmetric at
  # all. Every single-metric model - hypot, L1, anything on one radius -
  # requires (ex, ey) and (ey, ex) to score the same. Rectangular does
  # not: with the measured curves it puts x=8,y=20 at P_x(8)*P_y(20) =
  # 1.00*0.75 = 75%, and x=20,y=8 at P_x(20)*P_y(8) = 0.18*0.90 = 16%.
  #
  # The two cells agreeing kills rectangular again; the two differing
  # kills every symmetric metric at once. PAIRS is "bx:by" in mm.
  pair) PAIRS="${PAIRS:-8:20 20:8}" ;;
  *) echo "AXIS must be x, y, xy or pair, got '$AXIS'" >&2; exit 2 ;;
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
cp "$OPENARM_MODEL/$scene" "$OPENARM_SHARE/model/$scene" || exit 6
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
  case "$AXIS" in
    x)  bx="$v";      by="0.0000" ;;
    y)  bx="0.0000";  by="$v" ;;
    xy) bx="$v";      by="$v" ;;
  esac
  echo "=== open loop, ${AXIS} bias ${m}mm (bx=$bx by=$by) ==="
  for r in $(seq "$FIRST" "$REPEATS"); do
    bash "$EXPDIR/run_rel.sh" "$(printf 'ol_%s%03d_%s' "$AXIS" "$m" "$r")" \
      "$bx" "$by" "$OUTDIR" "$OFFX" "$OFFY" 2>&1 \
      | grep -E 'INSTABILITY|trial .* ->' | tail -2
  done
done
fi
unset PHYSICS_SCENE REHOVER DETECTOR_ROI_CASCADE RELATIVE_CORRECTION
echo OPENLOOP_DONE
