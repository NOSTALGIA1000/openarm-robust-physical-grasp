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
EXPDIR="${EXPDIR:-$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)}"
source "$EXPDIR/env.sh"

REPEATS="${REPEATS:-20}"
BIAS="${BIAS:-128}"
OUTDIR="${OUTDIR:-stage51_offseterror}"
SIDE="${SIDE:-50}"
OFFX="${OFFX:--0.0209}"
# Error injected into rel_offset_y, in millimetres. 0 is the fitted value.
ERRORS="${ERRORS:-0 10 20 30}"
export LIFT_COMMAND_M="${LIFT_COMMAND_M:-0.050}"
export REHOVER=true
export DETECTOR_ROI_CASCADE=90,55,30
export OPEN_POSITION=0.050

python3 "$EXPDIR/make_object_variants.py" \
  "$OPENARM_MODEL/table_scene_physics.xml" "$OPENARM_MODEL" "$SIDE" || exit 6
scene=$(printf "table_scene_c%03d.xml" "$SIDE")
cp "$OPENARM_MODEL/$scene" "$OPENARM_SHARE/model/$scene" || exit 6
export PHYSICS_SCENE="$scene"
# Derived values come from the generator's sidecar, so the rules that
# produce them live in one file. Restating them here is how three
# constants went wrong before.
set -a; . "$OPENARM_MODEL/${scene%.xml}.env"; set +a

v=$(echo "scale=4; $BIAS/1000" | bc)
FITTED_Y=-0.0007

for e in $ERRORS; do
  # Fitted value plus the injected error, in metres.
  offy=$(echo "scale=4; $FITTED_Y - $e/1000" | bc)
  echo "=== rel_offset_y 误差 ${e}mm (offy=$offy, offx=$OFFX) ==="
  for r in $(seq 1 "$REPEATS"); do
    bash "$EXPDIR/run_rel.sh" "$(printf 'oe_e%02d_y%s_%s' "$e" "$BIAS" "$r")" \
      0.0 "0$v" "$OUTDIR" "$OFFX" "$offy" 2>&1 \
      | grep -E 'INSTABILITY|trial .* ->' | tail -2
  done
done
unset PHYSICS_SCENE REHOVER DETECTOR_ROI_CASCADE
echo OFFSETERROR_DONE
