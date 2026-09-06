#!/usr/bin/env bash
# Densify the y tolerance curve between 256mm and 288mm, where success falls
# from 4/7 to 0/4 with nothing measured in between (2026-09-06 retrospective).
#
# 240 and 256 are anchors, not new points. The detector has changed since
# stage19 measured that curve - ROI 55 -> 30 px (stage24), band lower edge
# 0.09 -> 0.06 (stage30) - so a batch run today is not automatically
# comparable with it. If the anchors reproduce (240 at 4/4, 256 near 4/7) the
# new points extend the same curve; if they do not, the configuration
# difference is what the batch has found, and that must be reported instead.
#
# measure_samples stays at its default of 1: best-of-N is proven useless and
# would otherwise silently differ from every earlier batch.
EXPDIR="${EXPDIR:-$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)}"
source "$EXPDIR/env.sh"
for m in 240 256 264 272 280; do
  v=$(echo "scale=4; $m/1000" | bc)
  for r in 1 2 3 4; do
    bash "$EXPDIR/run_rel.sh" "yd${m}_${r}" 0.0 "0$v" stage34_ydense -0.0209 -0.0007 2>&1 | tail -1
  done
done
echo YDENSE_DONE
