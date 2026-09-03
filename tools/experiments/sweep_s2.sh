#!/usr/bin/env bash
# Stage 2 sweep: open-loop tolerance curve, plus corrected trials where the
# open-loop grasp has broken down.
REPS="${1:-3}"
for b in 0.000 0.012 0.020 0.024 0.028 0.032; do
  tag=$(echo "$b" | sed 's/0\.0*//')
  for r in $(seq 1 "$REPS"); do
    bash /tmp/run_s2.sh "ol_b${tag}_r${r}" "$b" false stage2 2>&1 | grep -E 'trial .* ->' | tail -1
  done
done
for b in 0.024 0.028 0.032; do
  tag=$(echo "$b" | sed 's/0\.0*//')
  for r in $(seq 1 "$REPS"); do
    bash /tmp/run_s2.sh "vc_b${tag}_r${r}" "$b" true stage2 2>&1 | grep -E 'trial .* ->' | tail -1
  done
done
echo SWEEP_DONE
