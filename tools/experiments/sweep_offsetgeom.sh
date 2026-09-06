#!/usr/bin/env bash
# F1: does rel_offset depend on the calibration condition, or on the viewing
# angle it happens to imply?
#
# The workspace document measured rel_offset_x at -14.2mm with no bias and
# -20.9mm at 64mm, and noted that a real dependence would break "calibrate
# once, use everywhere". The geometric account (2026-09-06-openarm-rel-offset-
# geometry.md) says the dependence is real but computable: bias moves the
# camera, which changes the viewing angle theta, and the offset goes as
# w*sin(theta). What decides it is whether rel_offset / sin(theta) holds
# constant across bias, with theta taken from grip_cam - a reading that never
# passes through the extrinsic.
#
# Runs with zero calibration offsets, so the residual is the inherent offset
# itself. Each trial now records obj_cam, grip_cam, gripper_kin, view_tan and
# the residual into trial.json, so the analysis reads the data rather than the
# logs.
#
# Two window schedules, because a fixed window is placed by projecting through
# the biased extrinsic and so moves with the bias, which would masquerade as a
# dependence. The cascade re-centres on its own optical measurement and should
# not.
#
# Usage: sweep_offsetgeom.sh [fixed|cascade]   (default: both, ~6h)
EXPDIR="${EXPDIR:-$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)}"
source "$EXPDIR/env.sh"

REPEATS="${REPEATS:-12}"
run_scheme() {
  local tag="$1" cascade="$2"
  export DETECTOR_ROI_CASCADE="$cascade"
  echo "=== $tag (roi_cascade=[$cascade]) ==="
  for m in 000 064 128; do
    v=$(echo "scale=4; $m/1000" | bc)
    for r in $(seq 1 "$REPEATS"); do
      bash "$EXPDIR/run_rel.sh" "og_${tag}_x${m}_${r}" "0$v" 0.0 \
        stage35_offsetgeom 0.0 0.0 2>&1 | tail -1
    done
  done
}

case "${1:-both}" in
  fixed)   run_scheme fix 30 ;;
  cascade) run_scheme cas 90,55,30 ;;
  both)    run_scheme fix 30; run_scheme cas 90,55,30 ;;
  *) echo "usage: $0 [fixed|cascade|both]"; exit 2 ;;
esac
echo OFFSETGEOM_DONE
