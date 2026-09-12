#!/usr/bin/env bash
# Direction study: is the 24 mm tolerance direction-dependent?
EXPDIR="${EXPDIR:-$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)}"
source "$EXPDIR/env.sh"
for m in 008 016 024; do
  # mm_to_m, not bc: the negative arm below needs a leading zero, which
  # bc does not emit and the ROS CLI will not accept. See env.sh.
  v=$(mm_to_m "$m")
  nv=$(mm_to_m "-$m")
  for r in 1 2; do
    bash "$EXPDIR/run_s2b.sh" "px${m}_r${r}"  "$v"   0.000 false stage2b 2>&1 | tail -1
    bash "$EXPDIR/run_s2b.sh" "nx${m}_r${r}" "$nv"  0.000 false stage2b 2>&1 | tail -1
  done
done
echo AXIS_DONE
