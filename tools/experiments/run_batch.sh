#!/usr/bin/env bash
# Stage 1 grasp-formation batch. Each trial gets a fresh launch (the scene has
# no reset service, so a displaced object contaminates every later trial in the
# same instance) and is validated before it is counted.
EXPDIR="${EXPDIR:-$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)}"
source "$EXPDIR/env.sh"
source "$ROS_SETUP"
source "$OPENARM_INSTALL/setup.bash"

N="${1:-5}"
OUT="$OPENARM_RESULTS/stage3_acceptance"
mkdir -p "$OUT"

for i in $(seq 1 "$N"); do
  TRIAL=$(printf "s%02d" "$i")
  LOG="/tmp/physics_launch_${TRIAL}.log"
  echo "=== $TRIAL ==="
  bash "$EXPDIR/preflight.sh" 1024 >/dev/null || { echo "$TRIAL preflight_abort"; continue; }

  setsid nohup ros2 launch openarm_moveit_config run_physics_grasp.launch.py \
    viewer_render_period:=1.0 > "$LOG" 2>&1 < /dev/null &
  for k in $(seq 1 120); do grep -q 'Successfully loaded controller' "$LOG" 2>/dev/null && break; sleep 3; done
  for k in $(seq 1 60);  do ros2 action list 2>/dev/null | grep -q '/move_action' && break; sleep 2; done
  sleep 8

  strip() { sed 's/\x1b\[[0-9;]*m//g'; }
  is_active() { ros2 control list_controllers 2>/dev/null | strip | awk -v c="$1" '$1==c && $3=="active"{f=1} END{exit !f}'; }
  NEED="joint_state_broadcaster left_arm_controller left_hand_controller"
  for attempt in 1 2 3 4 5; do
    OK=1
    for c in $NEED; do
      if ! is_active "$c"; then
        OK=0
        if ! ros2 control list_controllers 2>/dev/null | strip | awk -v c="$c" '$1==c{f=1} END{exit !f}'; then
          ros2 control load_controller --set-state active "$c" >/dev/null 2>&1
        else
          ros2 control set_controller_state "$c" inactive >/dev/null 2>&1; sleep 1
          ros2 control set_controller_state "$c" active   >/dev/null 2>&1
        fi
      fi
    done
    [ "$OK" = "1" ] && break
    sleep 3
  done
  SKIP=0
  for c in $NEED; do is_active "$c" || { echo "$TRIAL controller_abort:$c"; SKIP=1; }; done
  [ "$SKIP" = "1" ] && continue

  timeout 600 ros2 run openarm_grasp_physics physics_picker --ros-args \
    -p use_sim_time:=true -p trial_id:="$TRIAL" -p output_dir:="$OUT" > "/tmp/picker_${TRIAL}.log" 2>&1
  grep -E 'trial .* ->' "/tmp/picker_${TRIAL}.log" | tail -1
  echo "$TRIAL instability=$(grep -c 'unstable\|QACC' "$LOG")" 
done

python3 - "$OUT" <<'PY'
import csv, json, os, sys
out=sys.argv[1]; rows=[]
for name in sorted(os.listdir(out)):
    d=os.path.join(out,name)
    tj=os.path.join(d,'trial.json'); sc=os.path.join(d,'samples.csv')
    if not (os.path.isfile(tj) and os.path.isfile(sc)): continue
    p=json.load(open(tj)); o=p['observations']
    t=[float(r['sim_time']) for r in csv.DictReader(open(sc))]
    monotonic = all(t[i]>=t[i-1] for i in range(1,len(t)))
    rows.append(dict(trial=p['trial_id'], category=p['category'],
                     bilateral=o['bilateral_confirmed'], hold=o['hold_bilateral_ratio'],
                     elev=o['max_elevation_m'], samples=len(t), valid=monotonic))
print()
print('%-6s %-24s %-10s %-6s %-9s %-7s %s' % ('trial','category','bilateral','hold','elev(m)','samples','valid'))
for r in rows:
    print('%-6s %-24s %-10s %-6.2f %-9.4f %-7d %s' % (r['trial'],r['category'],r['bilateral'],r['hold'],r['elev'],r['samples'],r['valid']))
valid=[r for r in rows if r['valid']]
formed=[r for r in valid if r['bilateral']]
held=[r for r in valid if r['hold']>=0.80]
print()
succ=[r for r in valid if r['category']=='success']
print('valid trials              : %d/%d' % (len(valid),len(rows)))
print('SUCCESS (all criteria)    : %d/%d' % (len(succ),len(valid)))
print('bilateral grasp formed    : %d/%d' % (len(formed),len(valid)))
print('hold bilateral ratio >=0.8: %d/%d' % (len(held),len(valid)))
print('met 20mm lift             : %d/%d' % (sum(1 for r in valid if r['elev']>=0.020),len(valid)))
json.dump(rows, open(os.path.join(out,'summary.json'),'w'), indent=2)
PY
