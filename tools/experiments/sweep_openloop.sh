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
# Bracketing the 15mm clearance: one control, one inside, one just outside,
# one well outside.
BIASES="${BIASES:-0 8 16 24}"
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

for m in $BIASES; do
  v=$(mm_to_m "$m")
  echo "=== open loop, bias ${m}mm (clearance 15mm/side) ==="
  for r in $(seq "$FIRST" "$REPEATS"); do
    bash "$EXPDIR/run_rel.sh" "$(printf 'ol_y%03d_%s' "$m" "$r")" \
      0.0 "$v" "$OUTDIR" "$OFFX" "$OFFY" 2>&1 \
      | grep -E 'INSTABILITY|trial .* ->' | tail -2
  done
done
unset PHYSICS_SCENE REHOVER DETECTOR_ROI_CASCADE RELATIVE_CORRECTION
echo OPENLOOP_DONE
