#!/usr/bin/env bash
# Clearance, reached two independent ways.
#
# The lab's cubes are 50mm. The pre-grasp gap is twice open_position, so at the
# 0.030 the simulation has always used, a 50mm cube leaves 5mm per side - not
# the 15mm every tolerance result so far has rested on. That matters: the
# finding that residual inside +/-15mm gets absorbed by self-centering is a
# statement about clearance, and on the real objects that budget is a third of
# what was measured.
#
# Clearance = (2*open_position - side) / 2, so it moves with either the object
# or the opening. That makes this a test of the mechanism rather than a table:
#
#   cube 50mm, open 0.040  ->  15mm/side  \  same clearance,
#   cube 30mm, open 0.030  ->  15mm/side  /  different object
#   cube 50mm, open 0.050  ->  25mm/side  \  same clearance,
#   cube 30mm, open 0.040  ->  25mm/side  /  different object
#
# If clearance governs, the matched pairs behave alike despite the objects
# differing by 20mm. If they do not, clearance is not the whole story and the
# self-centering account needs revising.
#
# close_position scales as (side - 0.018)/2 to hold the squeeze at 18mm.
# Leaving it at 0.006 would squeeze a 50mm cube by 38mm and confound grip force
# with clearance; the formula reproduces 0.006 at 30mm.
#
# Finger travel caps at 0.060 per finger, so a 50mm cube can reach 35mm/side.
EXPDIR="${EXPDIR:-$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)}"
source "$EXPDIR/env.sh"

REPEATS="${REPEATS:-12}"
# "<side_mm>:<open_position>" - the pairs above plus the as-is 50mm case.
CELLS="${CELLS:-50:0.030 50:0.040 50:0.050 30:0.030 30:0.040}"
BIASES="${BIASES:-000 128}"
OUTDIR="${OUTDIR:-stage46_clearance}"
OFFX="${OFFX:--0.0209}"
OFFY="${OFFY:--0.0007}"
# Commanded gripper rise, separate from the 20mm acceptance threshold. At the
# old 0.030 the object slipped 8.5-11.5mm in the jaws and a 30mm cube landed at
# 18.4mm mean - every failure just under the line, while a 50mm cube reached
# 25.5mm. The threshold was not size-neutral, which voided the paired test.
# 0.050 puts both sizes clear of it: the 30mm cube now lifts 39.1mm.
export LIFT_COMMAND_M="${LIFT_COMMAND_M:-0.030}"

sides=$(for c in $CELLS; do echo "${c%%:*}"; done | sort -u | tr '\n' ' ')
python3 "$EXPDIR/make_object_variants.py" \
  "$OPENARM_MODEL/table_scene_physics.xml" "$OPENARM_MODEL" $sides || exit 6

for cell in $CELLS; do
  s="${cell%%:*}"; op="${cell##*:}"
  scene=$(printf "table_scene_c%03d.xml" "$s")
  cp "$OPENARM_MODEL/$scene" "$OPENARM_SHARE/model/$scene" || exit 6
  export PHYSICS_SCENE="$scene"
  # Derived values come from the generator's sidecar, so the rules that
  # produce them live in one file. Restating them here is how three
  # constants went wrong before.
  set -a; . "$OPENARM_MODEL/${scene%.xml}.env"; set +a
    export OPEN_POSITION="$op"
    clear_mm=$(echo "scale=1; ($op * 2000 - $s) / 2" | bc)
  if [ "$(echo "$clear_mm <= 0" | bc)" = "1" ]; then
    echo "=== SKIP cube ${s}mm open ${op}: clearance ${clear_mm}mm, jaws do not fit ==="
    continue
  fi
  echo "=== cube ${s}mm, open ${op} (clearance ${clear_mm}mm/side, close=$CLOSE_POSITION, z_bias=$DETECTOR_Z_BIAS, band_hi=$DETECTOR_OBJECT_DEPTH_HI) ==="
  for m in $BIASES; do
    v=$(echo "scale=4; $m/1000" | bc)
    for r in $(seq 1 "$REPEATS"); do
      bash "$EXPDIR/run_rel.sh" \
        "$(printf 'cl_c%03d_o%s_y%s_%s' "$s" "${op#0.}" "$m" "$r")" \
        0.0 "0$v" "$OUTDIR" "$OFFX" "$OFFY" 2>&1 \
        | grep -E 'INSTABILITY|trial .* ->' | tail -2
    done
  done
done
unset PHYSICS_SCENE DETECTOR_Z_BIAS OPEN_POSITION CLOSE_POSITION DETECTOR_OBJECT_DEPTH_HI
echo CLEARANCE_DONE
