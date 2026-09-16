#!/usr/bin/env bash
# What does each part of the pipeline actually contribute?
#
# The repository has one ablation, at n=2 - the re-hover comparison in the
# step-4 document. Everything else is asserted. A reviewer asking "does the
# cascade earn its place" or "how much does re-hover matter" currently has no
# answer, and neither do we.
#
# Four arms at one condition, 128mm of bias, where the correction is doing
# work. Each drops exactly one component from the working configuration:
#
#   full      the configuration chosen for hardware
#   norehover descend straight from the corrected target instead of returning
#             to hover first - step 4 saw 0/2 against 2/2 here
#   nooffset  rel_offset zeroed, so the surface-versus-origin term is not
#             subtracted
#   fixedroi  a single 30px window instead of the converging cascade, which is
#             what the detector used before the cascade existed
#
# n=20 per arm, from docs/statistics.md: at that size an all-success cell
# supports a lower bound of 83.9%, against 51.0% at n=4.
EXPDIR="${EXPDIR:-$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)}"
source "$EXPDIR/env.sh"

REPEATS="${REPEATS:-20}"
BIAS="${BIAS:-128}"
OUTDIR="${OUTDIR:-stage49_ablation}"
SIDE="${SIDE:-50}"
export LIFT_COMMAND_M="${LIFT_COMMAND_M:-0.050}"

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
export OPEN_POSITION=0.050

v=$(echo "scale=4; $BIAS/1000" | bc)

run_arm() {
  local tag="$1" offx="$2" offy="$3" r
  echo "=== $tag (rehover=$REHOVER, offset=$offx,$offy, roi=$DETECTOR_ROI_CASCADE) ==="
  for r in $(seq 1 "$REPEATS"); do
    bash "$EXPDIR/run_rel.sh" "ab_${tag}_y${BIAS}_${r}" 0.0 "0$v" \
      "$OUTDIR" "$offx" "$offy" 2>&1 \
      | grep -E 'INSTABILITY|trial .* ->' | tail -2
  done
}

export REHOVER=true DETECTOR_ROI_CASCADE=90,55,30
run_arm full -0.0209 -0.0007

export REHOVER=false DETECTOR_ROI_CASCADE=90,55,30
run_arm norehover -0.0209 -0.0007

export REHOVER=true DETECTOR_ROI_CASCADE=90,55,30
run_arm nooffset 0.0 0.0

export REHOVER=true DETECTOR_ROI_CASCADE=30
run_arm fixedroi -0.0209 -0.0007

unset PHYSICS_SCENE REHOVER DETECTOR_ROI_CASCADE
echo ABLATION_DONE
