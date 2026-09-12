#!/usr/bin/env bash
# How wrong can the correction's own constant be, in the direction that matters?
#
# Zeroing rel_offset costs nothing (stage49_ablation, stage50_offsetneed at
# three clearances). The reason turned out to be geometric rather than
# interesting: the fitted constant is (-20.9, -0.7)mm, so almost all of it
# lies in x - the pad-width direction, where the pads are wide and the
# open-loop tolerance is 32-40mm - while the jaws close along y, where
# tolerance is 24-32mm. Zeroing it moves residual_x by 25.0mm and residual_y
# by 2.5mm, and the grasp does not care about the first.
#
# That answers "can the constant be dropped" but not "how accurate does it
# have to be", which is the question a calibration-free claim actually rests
# on. This puts a deliberate error into the component the grasp is sensitive
# to and finds where it starts to cost.
#
# rel_offset_x stays at its fitted value throughout; only y varies.
#
# Falsifiable prediction, written before the run. The 2026-09-12 ceiling
# re-analysis showed that a y residual larger than the per-side clearance
# forces one-sided contact: the vg arm reached res_y = 40.4mm against 15mm
# of clearance and failed 12/12, every one of them unilateral_contact, with
# no detection loss. The default configuration here is the 20mm cube at
# open_position 0.025, which is also 15mm of clearance per side. So:
#
#   0mm and 10mm of injected error should succeed at the rate the 128mm
#   bias cell already shows (about 10/11 to 11/11);
#   20mm and 30mm should fail, and fail specifically as unilateral_contact
#   rather than no_contact or detector_timeout.
#
# If 20mm still succeeds, the clearance argument is wrong and self-centring
# absorbs more than the geometry says. If failures come back as no_contact,
# the mechanism is not the one claimed. Either outcome is informative; a
# boundary between 10 and 20mm would then be worth resolving at 15mm.
#
# Note the clearance changed with the cube: the header figures above
# (residual_x 25.0mm, residual_y 2.5mm) were measured on the 50mm cube at
# 25mm of clearance. At 15mm this test is stricter, which is the point.
EXPDIR="${EXPDIR:-$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)}"
source "$EXPDIR/env.sh"

REPEATS="${REPEATS:-20}"
BIAS="${BIAS:-128}"
OUTDIR="${OUTDIR:-stage51_offseterror}"
# The lab cube is 20mm (corrected 2026-09-11); 50mm was the earlier wrong
# assumption. Both remain runnable, but the default is the real one.
SIDE="${SIDE:-20}"
OFFX="${OFFX:--0.0209}"
# Error injected into rel_offset_y, in millimetres. 0 is the fitted value.
ERRORS="${ERRORS:-0 10 20 30}"
export LIFT_COMMAND_M="${LIFT_COMMAND_M:-0.050}"
export REHOVER=true
export DETECTOR_ROI_CASCADE=90,55,30
# Was hardcoded at 0.050 with export, so passing OPEN_POSITION=0.025 was
# silently ignored and a 20mm run would really have had 40mm of clearance
# per side instead of 15mm. 0.025 is the configuration chosen for the robot.
export OPEN_POSITION="${OPEN_POSITION:-0.025}"

python3 "$EXPDIR/make_object_variants.py" \
  "$OPENARM_MODEL/table_scene_physics.xml" "$OPENARM_MODEL" "$SIDE" || exit 6
scene=$(printf "table_scene_c%03d.xml" "$SIDE")
cp "$OPENARM_MODEL/$scene" "$OPENARM_SHARE/model/$scene" || exit 6
export PHYSICS_SCENE="$scene"
# Derived values come from the generator's sidecar, so the rules that
# produce them live in one file. Restating them here is how three
# constants went wrong before.
set -a; . "$OPENARM_MODEL/${scene%.xml}.env"; set +a

v=$(mm_to_m "$BIAS")
# In millimetres, so the injected error below is plain subtraction.
FITTED_Y_MM=-0.7

for e in $ERRORS; do
  # Fitted value plus the injected error, in metres.
  # mm_to_m, not bc: bc yields -.0107, which the ROS CLI resolves as a
  # STRING (PyYAML rejects a signed leading dot), so every level of this
  # sweep would have run on the default rel_offset_y with nothing in the
  # log to say so. See the note on mm_to_m in env.sh.
  offy=$(mm_to_m "$(awk -v f="$FITTED_Y_MM" -v e="$e" 'BEGIN{print f - e}')")
  echo "=== rel_offset_y 误差 ${e}mm (offy=$offy, offx=$OFFX) ==="
  for r in $(seq 1 "$REPEATS"); do
    bash "$EXPDIR/run_rel.sh" "$(printf 'oe_e%02d_y%s_%s' "$e" "$BIAS" "$r")" \
      0.0 "$v" "$OUTDIR" "$OFFX" "$offy" 2>&1 \
      | grep -E 'INSTABILITY|trial .* ->' | tail -2
  done
done
unset PHYSICS_SCENE REHOVER DETECTOR_ROI_CASCADE
echo OFFSETERROR_DONE
