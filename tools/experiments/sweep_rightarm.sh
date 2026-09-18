#!/usr/bin/env bash
# The tolerance region has been measured on exactly one arm.
#
# Every cell of stage54 and stage56 drives the left arm. That is enough to say
# how the geometry behaves, and nothing at all about whether what was measured
# is a property of the gripper or of the particular arm carrying it. The
# distinction matters for anything written down: a budget that belongs to the
# jaws transfers to any arm holding them, and a budget that belongs to one
# arm's kinematics does not transfer even to the identical arm on the other
# side of the same torso.
#
# The two arms are NOT mirror images and this batch must not be read as if they
# were. Their base poses are mirrored - Rx(-90) at y=+0.031 against Rx(+90) at
# y=-0.031 - but every link below the base is identical, so the right arm is a
# left arm mounted on a mirrored base. The same joint angles that put the left
# hand at (0.3, 0, 0.447) put the right hand at (0.3, 0, 0.949), in the air.
#
# Falsifiable predictions, written before the run.
#
# 0. Reachability, which the zero-bias cell settles on its own and which
#    nothing so far establishes. If planning_failure dominates there, the right
#    arm does not reach (0.3, 0, 0.447) with a downward tool axis and the
#    object has to move before any of the rest means anything. Run this cell at
#    REPEATS=3 first; three planning failures is an answer and costs 20
#    minutes.
#
# 1. The arm term. jaw - target is +9.15 to +10.21mm in x and -0.09 to -1.05 in
#    y across all six left-arm cells of stage56_jaw and +9.42 / +0.12 on the
#    corrected pipeline, which is why it was adopted as the baseline. If that
#    constant is the fingers, the right arm reads the same. If it is the left
#    arm's approach pose, it does not, and the baseline is a per-arm quantity
#    rather than a geometric one - which would not invalidate any published
#    number, since each batch measures its own, but would remove the claim that
#    it transfers.
#
# 2. The rates. If the region is the gripper's, y024 lands near stage56_jaw's
#    1/20 and x016 near its 15/20. A departure at matched ACHIEVED offset - not
#    at matched command, which is the trap that produced a false y asymmetry on
#    2026-09-17 - says the arm matters.
#
# Three cells rather than the full nine: the zero cell for reachability and the
# arm term, y024 at the cliff and x016 on the x slope, which are the two places
# the left-arm curves are steep enough for a difference to show. The plateau
# cells would cost three hours to confirm that 100% is still 100%.
EXPDIR="${EXPDIR:-$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)}"
source "$EXPDIR/env.sh"

export ARM="${ARM:-right}"
# Where the object sits. The smoke test of 2026-09-18 found the right arm
# sweeping a LINK through the cube on the way in: three trials, all
# no_contact, the cube leaving at 11 m/s during descend with both finger
# contacts false and zero force, once with the tracked hand parked 0.10m
# above it. Contact classification only watches the two finger geoms, so a
# link passing through the object is invisible in the record and shows up
# only as an object that is no longer there. Empty keeps the scene own y,
# which is what every left-arm batch used.
OBJECT_Y="${OBJECT_Y:-}"
REPEATS="${REPEATS:-20}"
FIRST="${FIRST:-1}"
# "axis:mm" pairs, so the three cells can be named without a second loop.
CELLS="${CELLS:-y:0 y:24 x:16}"
OUTDIR="${OUTDIR:-stage58_rightarm}"
SIDE="${SIDE:-20}"
export OPEN_POSITION="${OPEN_POSITION:-0.025}"
export LIFT_COMMAND_M="${LIFT_COMMAND_M:-0.050}"
export REHOVER=true
export DETECTOR_ROI_CASCADE=90,55,30
export RELATIVE_CORRECTION=false

if [ -n "$OBJECT_Y" ]; then
  python3 "$EXPDIR/make_object_variants.py" --open "$OPEN_POSITION" \
    --object-y "$OBJECT_Y" \
    "$OPENARM_MODEL/table_scene_physics.xml" "$OPENARM_MODEL" "$SIDE" || exit 6
  # Matches the generator's own naming, sign spelled out.
  ymm=$(awk -v v="$OBJECT_Y" 'BEGIN{ printf "%s%03d", (v<0 ? "m" : "p"), (v<0 ? -v : v) * 1000 + 0.5 }')
  scene=$(printf "table_scene_c%03d_y%s.xml" "$SIDE" "$ymm")
else
  python3 "$EXPDIR/make_object_variants.py" --open "$OPEN_POSITION" \
    "$OPENARM_MODEL/table_scene_physics.xml" "$OPENARM_MODEL" "$SIDE" || exit 6
  scene=$(printf "table_scene_c%03d.xml" "$SIDE")
fi
[ -f "$OPENARM_MODEL/$scene" ] || { echo "generator did not produce $scene" >&2; exit 6; }
if [ "$(readlink -f "$OPENARM_MODEL/$scene")" != "$(readlink -f "$OPENARM_SHARE/model/$scene" 2>/dev/null)" ]; then
  cp "$OPENARM_MODEL/$scene" "$OPENARM_SHARE/model/$scene" || exit 6
fi
export PHYSICS_SCENE="$scene"
set -a; . "$OPENARM_MODEL/${scene%.xml}.env"; set +a

OFFX="${OFFX:--0.0209}"
OFFY="${OFFY:--0.0007}"

for cell in $CELLS; do
  axis="${cell%%:*}"; m="${cell##*:}"
  v=$(mm_to_m "$m")
  case "$axis" in
    x) bx="$v";     by="0.0000" ;;
    y) bx="0.0000"; by="$v" ;;
    *) echo "cell axis must be x or y, got '$axis'" >&2; exit 2 ;;
  esac
  echo "=== ${ARM} arm, ${axis} bias ${m}mm (bx=$bx by=$by), object_y=${OBJECT_Y:-scene default}, scene=$scene ==="
  for r in $(seq "$FIRST" "$REPEATS"); do
    bash "$EXPDIR/run_rel.sh" "$(printf 'ol_%s%03d_%s' "$axis" "$m" "$r")" \
      "$bx" "$by" "$OUTDIR" "$OFFX" "$OFFY" 2>&1 \
      | grep -E 'INSTABILITY|trial .* ->' | tail -2
  done
done
unset PHYSICS_SCENE REHOVER DETECTOR_ROI_CASCADE RELATIVE_CORRECTION
echo RIGHTARM_DONE
