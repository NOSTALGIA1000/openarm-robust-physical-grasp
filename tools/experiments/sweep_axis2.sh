#!/usr/bin/env bash
EXPDIR="${EXPDIR:-$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)}"
source "$EXPDIR/env.sh"
for m in 032 040 048; do
  v=$(echo "scale=3; $m/1000" | bc)
  for r in 1 2; do
    bash "$EXPDIR/run_s2b.sh" "px${m}_r${r}" "$v" 0.000 false stage2b 2>&1 | tail -1
  done
done
echo AXIS2_DONE
