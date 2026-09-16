#!/usr/bin/env bash
# Does any of this survive a real depth sensor?
#
# MuJoCo renders exact depth. Every threshold in the detector - the object
# band, the gripper band, the deep percentile, the blob area floor - was tuned
# against exact depth, and a RealSense D435 at 0.7m carries roughly 2-3mm RMS.
# If the pipeline degrades sharply between 0 and 3mm of noise, the simulation
# results do not predict hardware and the real-robot session will find that out
# the expensive way.
#
# Runs the configuration chosen for the hardware - 50mm cube, open 0.050, 25mm
# of clearance, which measured 12/12 at both biases without noise - and varies
# only the noise. Anything that changes is the noise.
#
# Usage: sweep_depthnoise.sh
#   NOISES="0.000 0.001 0.002 0.005"   REPEATS=12   BIASES="000 128"
EXPDIR="${EXPDIR:-$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)}"
source "$EXPDIR/env.sh"

REPEATS="${REPEATS:-12}"
NOISES="${NOISES:-0.000 0.001 0.002 0.005}"
BIASES="${BIASES:-000 128}"
OUTDIR="${OUTDIR:-stage48_depthnoise}"
SIDE="${SIDE:-50}"
OPEN="${OPEN:-0.050}"
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
export OPEN_POSITION="$OPEN"

for sd in $NOISES; do
  export DETECTOR_DEPTH_NOISE_STD="$sd"
  tag=$(echo "$sd" | sed 's/0\.//')
  echo "=== depth noise ${sd} m (cube ${SIDE}mm, open ${OPEN}) ==="
  for m in $BIASES; do
    v=$(echo "scale=4; $m/1000" | bc)
    for r in $(seq 1 "$REPEATS"); do
      # A different seed per trial, so the batch samples the noise process
      # rather than repeating one draw of it.
      export DETECTOR_DEPTH_NOISE_SEED="$r"
      bash "$EXPDIR/run_rel.sh" "dn_n${tag}_y${m}_${r}" 0.0 "0$v" \
        "$OUTDIR" -0.0209 -0.0007 2>&1 \
        | grep -E 'INSTABILITY|trial .* ->' | tail -2
    done
  done
done
unset PHYSICS_SCENE DETECTOR_DEPTH_NOISE_STD DETECTOR_DEPTH_NOISE_SEED
echo DEPTHNOISE_DONE
