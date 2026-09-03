#!/usr/bin/env bash
# Randomised initial object pose: displacement and yaw sampled per trial, so the
# batch measures generalisation rather than repeating one condition N times.
# Yaw matters independently: a 30 mm box at 45 deg presents a 42 mm diagonal to
# a jaw that only closes to 24 mm.
set --   "r01 4 -3 0" "r02 -5 2 0" "r03 2 6 0" "r04 -3 -6 0"   "r05 3 4 10" "r06 -4 -2 -10" "r07 5 -5 20" "r08 -2 3 -20"   "r09 0 0 30" "r10 3 -4 -30" "r11 -6 5 45" "r12 4 2 -45"
for spec in "$@"; do
  set -- $spec
  bash /tmp/run_rand.sh "$1" "$2" "$3" "$4" stage2c 2>&1 | tail -1
done
echo RAND_DONE
