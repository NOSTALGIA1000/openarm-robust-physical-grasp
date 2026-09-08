#!/usr/bin/env bash
# One-shot relative correction against an endpoint closed-loop servo baseline.
#
# The literature says the mechanism this project relies on is ECL - a camera
# that sees both target and end-effector is insensitive to calibration
# (Hutchinson, Hager & Corke 1996), applied to grasping by alignment in one
# uncalibrated image (Horaud, Dornaika & Espiau 1998). What has never been
# tested here is the claim that follows: that a single measurement added to
# forward kinematics reaches the same place as an iterative servo, without
# estimating an image Jacobian and without the moves that estimating it costs.
#
# Both arms read the same two optical measurements through the same detector
# and the same gripper hint, and neither touches the extrinsic. What differs is
# only the control law, which is what the comparison is about.
#
#   rel     one measurement, added to kinematics, one re-hover
#   servo   two probe motions to identify the Jacobian, then Broyden-updated
#           iterations until the image-space error falls under tolerance
#
# The servo arm gets ten iterations rather than the default six: cutting it off
# early would hand the comparison to the other arm for the wrong reason.
EXPDIR="${EXPDIR:-$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)}"
source "$EXPDIR/env.sh"

REPEATS="${REPEATS:-12}"
BIASES="${BIASES:-000 128 256}"
OUTDIR="${OUTDIR:-stage44_baseline}"

run_arm() {
  # Export explicitly rather than prefixing the call: assignments in front of a
  # shell function are not reliably exported to the processes it spawns, and
  # run_rel.sh reads these from its own environment.
  local tag="$1" offx="$2" offy="$3" m v r
  case "$tag" in
    rel)   export RELATIVE_CORRECTION=true  SERVO_BASELINE=false ;;
    servo) export RELATIVE_CORRECTION=false SERVO_BASELINE=true
           export SERVO_MAX_ITERS="${SERVO_MAX_ITERS:-10}" ;;
  esac
  echo "=== $tag (relative=$RELATIVE_CORRECTION servo=$SERVO_BASELINE off=$offx,$offy) ==="
  for m in $BIASES; do
    v=$(echo "scale=4; $m/1000" | bc)
    for r in $(seq 1 "$REPEATS"); do
      bash "$EXPDIR/run_rel.sh" "bl_${tag}_y${m}_${r}" 0.0 "0$v"         "$OUTDIR" "$offx" "$offy" 2>&1         | grep -E 'INSTABILITY|trial .* ->|servo baseline ->' | tail -3
    done
  done
}

case "${1:-both}" in
  rel)   run_arm rel -0.0209 -0.0007 ;;
  servo) run_arm servo 0.0 0.0 ;;
  both)  run_arm rel -0.0209 -0.0007; run_arm servo 0.0 0.0 ;;
  *) echo "usage: $0 [rel|servo|both]"; exit 2 ;;
esac
echo BASELINE_DONE
