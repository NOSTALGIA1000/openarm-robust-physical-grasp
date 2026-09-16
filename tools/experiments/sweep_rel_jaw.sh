#!/usr/bin/env bash
# The corrected arm, measured with the same ruler as the open-loop curve.
#
# The tolerance region of 2026-09-15 is a statement about geometry: how far the
# jaws may be from the cube when they close. The corrected arm's whole claim is
# that it keeps that distance small whatever the hand-eye error. The two have
# never been put on one axis, because they were never measured against the same
# thing - the open-loop curve came off the gripper link origin, whose offset
# from the jaws is +7.65mm on this pipeline and +21.45mm on the corrected one.
# Thirteen millimetres of the difference between the two was the reference.
#
# The jaw-centre telemetry added on 2026-09-16 exists to remove that. Its term
# does not drift with the approach pose: 10.56mm at zero bias, 10.46mm at 16mm
# of y. This batch is what tests whether it also does not drift between
# PIPELINES, which is the property the comparison actually needs.
#
# Every field matches sweep_openloop.sh: 20mm cube, open_position 0.025,
# lift 0.050, rehover on, cascade 90,55,30, rel_offset -20.9/-0.7mm. The one
# difference is RELATIVE_CORRECTION, which is the point.
#
# Falsifiable predictions, written before the run.
#
# 1. The geometric baseline. Measured from the jaw centre on the zero-bias
#    cell, the corrected arm should read about +10.7mm in x and +1.1mm in y -
#    the same constant stage56_jaw measured, because it is a property of the
#    fingers on the first close frame and not of the pipeline that drove them
#    there. If it differs by more than about 3mm, the jaw centre is no more
#    portable than the link origin and no cross-mode comparison can be built on
#    it. That is the sharpest test here and it costs one cell.
#
# 2. The achieved offsets. stage40_ceiling put the cal arm at 12/12 through
#    240mm of bias. If the correction works as claimed, the achieved jaw offset
#    at 128 and 240mm should stay inside the open-loop budget - under 16mm in
#    y, where the open-loop curve still reads 85%.
#
# 3. The transfer. If the offsets land inside the budget but the success rate
#    does not match what the open-loop curve gives at that offset, the curve
#    does not transfer, and the reason is worth more than the agreement would
#    have been: an open-loop residual is a clean translation, a corrected one
#    carries the relative measurement's spread of about 10mm. A cell whose
#    MEDIAN offset is 8mm can hold trials at 20mm. Per-trial offsets are
#    recorded, so this is checkable rather than speculative.
#
# The bias levels are 0 / 128 / 240mm because that is where the corrected arm
# has been measured before, on the old ruler, and 240mm is the last level where
# it was still at 12/12. Beyond it lies the detector's FOV ceiling at 283mm,
# which is a different failure and not what this batch is about.
EXPDIR="${EXPDIR:-$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)}"
source "$EXPDIR/env.sh"

REPEATS="${REPEATS:-20}"
FIRST="${FIRST:-1}"
BIASES="${BIASES:-0 128 240}"
OUTDIR="${OUTDIR:-stage57_reljaw}"
SIDE="${SIDE:-20}"
export OPEN_POSITION="${OPEN_POSITION:-0.025}"
export LIFT_COMMAND_M="${LIFT_COMMAND_M:-0.050}"
export REHOVER=true
export DETECTOR_ROI_CASCADE=90,55,30
# The one field that differs from sweep_openloop.sh.
export RELATIVE_CORRECTION=true

python3 "$EXPDIR/make_object_variants.py" --open "$OPEN_POSITION" \
  "$OPENARM_MODEL/table_scene_physics.xml" "$OPENARM_MODEL" "$SIDE" || exit 6
scene=$(printf "table_scene_c%03d.xml" "$SIDE")
if [ "$(readlink -f "$OPENARM_MODEL/$scene")" != "$(readlink -f "$OPENARM_SHARE/model/$scene" 2>/dev/null)" ]; then
  cp "$OPENARM_MODEL/$scene" "$OPENARM_SHARE/model/$scene" || exit 6
fi
export PHYSICS_SCENE="$scene"
set -a; . "$OPENARM_MODEL/${scene%.xml}.env"; set +a

# The established scheme's fitted offsets. Live here, unlike in the open-loop
# batch where they were passed only so the config snapshots would match.
OFFX="${OFFX:--0.0209}"
OFFY="${OFFY:--0.0007}"

for m in $BIASES; do
  v=$(mm_to_m "$m")
  echo "=== corrected arm, y bias ${m}mm (by=$v) ==="
  for r in $(seq "$FIRST" "$REPEATS"); do
    bash "$EXPDIR/run_rel.sh" "$(printf 'rel_y%03d_%s' "$m" "$r")" \
      "0.0000" "$v" "$OUTDIR" "$OFFX" "$OFFY" 2>&1 \
      | grep -E 'INSTABILITY|trial .* ->' | tail -2
  done
done
unset PHYSICS_SCENE REHOVER DETECTOR_ROI_CASCADE RELATIVE_CORRECTION
echo RELJAW_DONE
