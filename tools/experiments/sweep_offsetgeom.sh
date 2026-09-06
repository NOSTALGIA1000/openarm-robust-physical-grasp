#!/usr/bin/env bash
# F1: does rel_offset depend on the calibration condition, or on the viewing
# angle it happens to imply?
#
# The workspace document measured rel_offset_x at -14.2mm with no bias and
# -20.9mm at 64mm, and noted that a real dependence would break "calibrate
# once, use everywhere" - which is what calibration-free deployment needs.
#
# Runs with zero calibration offsets, so each residual is the inherent offset
# itself. Every trial records obj_cam, grip_cam, gripper_kin, view_tan and the
# residual into trial.json, so the analysis reads data rather than logs.
#
# Two window schedules, because a fixed window is placed by projecting through
# the biased extrinsic and so moves with the bias, which would masquerade as a
# dependence. The cascade re-centres on its own optical measurement.
#
# Axis matters, and the first run got it wrong. On x, 64mm left eight of twelve
# trials with no gripper reading at all and 128mm gave eleven planning failures
# out of twelve, because x meets the 112mm reach limit recorded in the step-10
# result. The method's range is on y, which reaches 240mm, so that is the axis
# on which the offset can actually be tracked against bias.
#
# Usage: sweep_offsetgeom.sh [fixed|cascade|both]
#   AXIS=y (default) | x     BIASES="000 064 128 192"     REPEATS=12
EXPDIR="${EXPDIR:-$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)}"
source "$EXPDIR/env.sh"

REPEATS="${REPEATS:-12}"
AXIS="${AXIS:-y}"
BIASES="${BIASES:-000 064 128 192}"
OUTDIR="${OUTDIR:-stage36_offsetgeom_y}"

run_scheme() {
  local tag="$1" cascade="$2" m v bx by r
  export DETECTOR_ROI_CASCADE="$cascade"
  echo "=== $tag (roi_cascade=[$cascade], axis=$AXIS) ==="
  for m in $BIASES; do
    v=$(echo "scale=4; $m/1000" | bc)
    if [ "$AXIS" = "x" ]; then bx="0$v"; by="0.0"; else bx="0.0"; by="0$v"; fi
    for r in $(seq 1 "$REPEATS"); do
      # Keep the divergence count as well as the verdict: piping to tail -1
      # left only a detector line, discarding the stability check that every
      # batch in this project is supposed to report.
      bash "$EXPDIR/run_rel.sh" "og_${tag}_${AXIS}${m}_${r}" "$bx" "$by" \
        "$OUTDIR" 0.0 0.0 2>&1 \
        | grep -E 'INSTABILITY|trial .* ->' | tail -2
    done
  done
}

case "${1:-both}" in
  fixed)   run_scheme fix 30 ;;
  cascade) run_scheme cas 90,55,30 ;;
  both)    run_scheme fix 30; run_scheme cas 90,55,30 ;;
  # A window schedule of your own, e.g. WINDOWS=240,120,55,30 for a first pass
  # wide enough that it need not be aimed by the extrinsic at all - the point
  # being that the standard cascade still anchors its first pass on the biased
  # projection and so inherits the error it is meant to survive.
  custom)  run_scheme "${TAG:-cus}" "${WINDOWS:?set WINDOWS, e.g. 240,120,55,30}" ;;
  *) echo "usage: $0 [fixed|cascade|both|custom]"; exit 2 ;;
esac
echo OFFSETGEOM_DONE
