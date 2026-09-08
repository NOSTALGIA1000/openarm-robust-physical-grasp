#!/usr/bin/env bash
# Does the result generalise past the one object every trial so far has used?
#
# Every recorded trial used a 30x30x44mm cuboid. The lab's real objects are
# cubes, so this sweeps cubes - the shape the hardware will actually present.
#
# Side length moves three things together, which is why the variants are
# generated rather than hand-edited:
#   clearance  (60 - s)/2 per side, the budget that absorbs residual error
#   z bias     s/2 + 5.5mm, since the detector measures the top face
#   placement  origin half a side above the table top
#
# Two predictions, and they fail differently:
#   1. Success at small bias should fall as the cube grows, because clearance
#      is what absorbs residual.
#   2. The 283mm wall should NOT move. It is the gripper leaving the frame
#      (cy*z/fy), which has nothing to do with the object. Biases straddle it
#      at 256 and 320mm.
#
# The 30mm cube is not a control for the historical data - that was 44mm tall.
# It is a control for the swap machinery only.
EXPDIR="${EXPDIR:-$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)}"
source "$EXPDIR/env.sh"

REPEATS="${REPEATS:-12}"
SIDES="${SIDES:-20 30 40}"
BIASES="${BIASES:-000 256 320}"
OUTDIR="${OUTDIR:-stage45_cubes}"
OFFX="${OFFX:--0.0209}"
OFFY="${OFFY:--0.0007}"

python3 "$EXPDIR/make_object_variants.py" \
  "$OPENARM_MODEL/table_scene_physics.xml" "$OPENARM_MODEL" $SIDES || exit 6

for s in $SIDES; do
  scene=$(printf "table_scene_c%03d.xml" "$s")
  cp "$OPENARM_MODEL/$scene" "$OPENARM_SHARE/model/$scene" || exit 6
  export PHYSICS_SCENE="$scene"
  # s/2 + 5.5mm, the detector's own top-surface offset measured on the cuboid.
  export DETECTOR_Z_BIAS=$(echo "scale=4; $s/2000 + 0.0055" | bc)
  echo "=== cube ${s}mm (clearance $(( (60 - s) / 2 ))mm/side, z_bias=$DETECTOR_Z_BIAS) ==="
  for m in $BIASES; do
    v=$(echo "scale=4; $m/1000" | bc)
    for r in $(seq 1 "$REPEATS"); do
      bash "$EXPDIR/run_rel.sh" "$(printf 'ob_c%03d_y%s_%s' "$s" "$m" "$r")" \
        0.0 "0$v" "$OUTDIR" "$OFFX" "$OFFY" 2>&1 \
        | grep -E 'INSTABILITY|trial .* ->' | tail -2
    done
  done
done
unset PHYSICS_SCENE DETECTOR_Z_BIAS
echo OBJECTS_DONE
