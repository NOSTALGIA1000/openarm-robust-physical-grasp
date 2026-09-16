#!/usr/bin/env bash
# Where does rel_offset start to matter?
#
# The ablation found zeroing it changes nothing at the hardware configuration:
# 18/20 either way, +0.0% [-21.4%, +21.4%]. The likely reason is that the
# constant is about 21mm and that configuration has 25mm of clearance, so
# self-centering absorbs it. If that is right the calibration-free claim does
# not need the constant computed - it needs enough clearance not to need it.
#
# That has to have a boundary. stage47_pairs already shows a 50mm cube slipping
# at 15mm of clearance, so the absorption is not unlimited. This sweeps three
# clearances against the same two arms and looks for the level at which
# dropping the constant starts to cost something.
#
#   5mm  clearance  open 0.030
#   15mm clearance  open 0.040
#   25mm clearance  open 0.050   (the ablation's condition, as the anchor)
#
# Bias fixed at 128mm so the correction is doing work, n=20 per cell.
EXPDIR="${EXPDIR:-$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)}"
source "$EXPDIR/env.sh"

REPEATS="${REPEATS:-20}"
BIAS="${BIAS:-128}"
OUTDIR="${OUTDIR:-stage50_offsetneed}"
SIDE="${SIDE:-50}"
OPENINGS="${OPENINGS:-0.030 0.040 0.050}"
export LIFT_COMMAND_M="${LIFT_COMMAND_M:-0.050}"
export REHOVER=true
export DETECTOR_ROI_CASCADE=90,55,30

python3 "$EXPDIR/make_object_variants.py" \
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
# Derived values come from the generator's sidecar, so the rules that
# produce them live in one file. Restating them here is how three
# constants went wrong before.
set -a; . "$OPENARM_MODEL/${scene%.xml}.env"; set +a

v=$(echo "scale=4; $BIAS/1000" | bc)

for op in $OPENINGS; do
  export OPEN_POSITION="$op"
  clr=$(echo "scale=0; ($op * 2000 - $SIDE) / 2" | bc)
  for arm in on off; do
    if [ "$arm" = "on" ]; then offx=-0.0209; offy=-0.0007; else offx=0.0; offy=0.0; fi
    echo "=== 余隙 ${clr}mm, offset ${arm} (open=$op, offx=$offx offy=$offy) ==="
    for r in $(seq 1 "$REPEATS"); do
      bash "$EXPDIR/run_rel.sh" \
        "$(printf 'ro_c%02d_%s_y%s_%s' "$clr" "$arm" "$BIAS" "$r")" \
        0.0 "0$v" "$OUTDIR" "$offx" "$offy" 2>&1 \
        | grep -E 'INSTABILITY|trial .* ->' | tail -2
    done
  done
done
unset PHYSICS_SCENE REHOVER DETECTOR_ROI_CASCADE
echo OFFSETNEED_DONE
