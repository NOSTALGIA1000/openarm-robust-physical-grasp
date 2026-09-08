#!/usr/bin/env bash
# Does the viewing-angle correction move the tolerance ceiling, or only the
# precision? The retrospective puts the established scheme at 12/12 through
# 240mm, 4/7 at 256mm and 0/4 at 288mm. If a six-fold reduction in residual
# buys nothing at the cliff, the improvement is real but inconsequential, and
# that is worth knowing before it is written up as an advance.
#
# Paired, because the detector has changed repeatedly since stage19 measured
# that curve - ROI 55 to 30, band edge 0.09 to 0.06, and now the radial
# correction. Running both arms in one batch removes every configuration
# difference except the one under test.
#
#   cal  the established scheme: no radial correction, rel_offset calibrated
#        at -20.9/-0.7mm
#   vg   the calibration-free scheme: radial correction on both readings,
#        rel_offset zero
#
# The two must not be combined: rel_offset was fitted under the uncorrected
# pipeline at 64mm of bias and already contains the angle term, so applying
# both would count it twice - the mistake this project has made twice before.
EXPDIR="${EXPDIR:-$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)}"
source "$EXPDIR/env.sh"

REPEATS="${REPEATS:-12}"
BIASES="${BIASES:-240 288 320}"
OUTDIR="${OUTDIR:-stage40_ceiling}"
GAIN="${GAIN:-0.0478}"
# The window schedule. Every pass is clipped to the frame, so a first radius of
# 400 covers the whole 640x480 image from any centre - which is how a search
# that does not depend on the extrinsic at all can be run without touching the
# detector. At 320mm of bias the projected centre is about 280px from the
# gripper, past even a 240px window, so the standard 90px first pass cannot
# contain it and every trial there timed out.
WINDOWS="${WINDOWS:-90,55,30}"
ARMTAG="${ARMTAG:-}"

run_arm() {
  local tag="$1" gain="$2" offx="$3" offy="$4" m v r
  export VIEW_GAIN_GRIPPER="$gain" VIEW_GAIN_OBJECT="$gain"
  export DETECTOR_ROI_CASCADE="$WINDOWS"
  tag="${ARMTAG:-$tag}"
  echo "=== $tag (gain=$gain, rel_offset=$offx,$offy, windows=$WINDOWS) ==="
  for m in $BIASES; do
    v=$(echo "scale=4; $m/1000" | bc)
    for r in $(seq 1 "$REPEATS"); do
      bash "$EXPDIR/run_rel.sh" "cl_${tag}_y${m}_${r}" 0.0 "0$v" \
        "$OUTDIR" "$offx" "$offy" 2>&1 \
        | grep -E 'INSTABILITY|trial .* ->' | tail -2
    done
  done
}

case "${1:-both}" in
  cal)  run_arm cal 0.0 -0.0209 -0.0007 ;;
  vg)   run_arm vg "$GAIN" 0.0 0.0 ;;
  both) run_arm cal 0.0 -0.0209 -0.0007; run_arm vg "$GAIN" 0.0 0.0 ;;
  *) echo "usage: $0 [cal|vg|both]"; exit 2 ;;
esac
echo CEILING_DONE
