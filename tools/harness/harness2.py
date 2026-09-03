"""Representative offline harness: real grasp joint configuration, object
resting on the table (as in the pipeline) rather than parked in free space."""
import mujoco, numpy as np, json, sys, os

os.chdir('/home/nostalgia/ros2_ws_course_repro/ros2_ws/src/openarm_moveit_config/model')
trace = json.load(open('/tmp/joint_trace.json'))
picker = open('/tmp/picker_jrec.log').read()

# Stage transition wall-clock stamps from the picker log.
import re
stamps = {}
for line in picker.splitlines():
    m = re.search(r'\[INFO\] \[(\d+\.\d+)\].*stage -> (\w+)', line)
    if m: stamps.setdefault(m.group(2), float(m.group(1)))
print('stages:', {k: round(v,1) for k,v in stamps.items()})

fin_key='openarm_left_finger_joint1'
# The finger starts at 0 (initial state), opens to ~0.030, then closes onto the
# object. Only the closure AFTER the open counts, so skip until it has opened.
opened=False; q_close=None
for r in trace:
    v=r['q'].get(fin_key, 0.0)
    if v > 0.020: opened=True
    elif opened and v < 0.015:
        q_close=r['q']; break
assert q_close is not None, 'no post-open closure found in trace'
print('grasp-pose sample selected, finger=%.5f' % q_close[fin_key])
for k in sorted(q_close): print('   %-32s % .5f' % (k, q_close[k]))

m = mujoco.MjModel.from_xml_path('table_scene_physics.xml')
d = mujoco.MjData(m)
ARM=['openarm_left_joint%d'%i for i in range(1,8)]
adr=[m.jnt_qposadr[mujoco.mj_name2id(m,mujoco.mjtObj.mjOBJ_JOINT,n)] for n in ARM]
vadr=[m.jnt_dofadr[mujoco.mj_name2id(m,mujoco.mjtObj.mjOBJ_JOINT,n)] for n in ARM]
j1=mujoco.mj_name2id(m,mujoco.mjtObj.mjOBJ_JOINT,'openarm_left_finger_joint1')
j2=mujoco.mj_name2id(m,mujoco.mjtObj.mjOBJ_JOINT,'openarm_left_finger_joint2')
a1,a2=m.jnt_qposadr[j1],m.jnt_qposadr[j2]; v1=m.jnt_dofadr[j1]
gL=mujoco.mj_name2id(m,mujoco.mjtObj.mjOBJ_GEOM,'openarm_left_left_finger_collision')
gR=mujoco.mj_name2id(m,mujoco.mjtObj.mjOBJ_GEOM,'openarm_left_right_finger_collision')
og=mujoco.mj_name2id(m,mujoco.mjtObj.mjOBJ_GEOM,'physics_object_geom')
tt=mujoco.mj_name2id(m,mujoco.mjtObj.mjOBJ_GEOM,'table_top')
oj=m.body_jntadr[mujoco.mj_name2id(m,mujoco.mjtObj.mjOBJ_BODY,'physics_object')]
oq=m.jnt_qposadr[oj]
hand=mujoco.mj_name2id(m,mujoco.mjtObj.mjOBJ_BODY,'openarm_left_hand')

# Put the arm at the real grasp configuration; leave the object where the scene
# spawns it (on the table), matching the pipeline.
for n,a in zip(ARM,adr):
    d.qpos[a]=q_close[n]
d.qpos[a1]=q_close.get('openarm_left_finger_joint1',0.006)
d.qpos[a2]=d.qpos[a1]
mujoco.mj_forward(m,d)
print('\nobject rest z=%.4f  hand z=%.4f  jaw centre z=%.4f' %
      (d.qpos[oq+2], d.xpos[hand][2], (d.geom_xpos[gL][2]+d.geom_xpos[gR][2])/2))
print('object y=%+.5f  jaw centre y=%+.5f  (dy=%+.5f)' %
      (d.qpos[oq+1], (d.geom_xpos[gL][1]+d.geom_xpos[gR][1])/2,
       d.qpos[oq+1]-(d.geom_xpos[gL][1]+d.geom_xpos[gR][1])/2))

def contacts(m,d):
    fin=0.0; tab=0.0
    for i in range(d.ncon):
        c=d.contact[i]
        if og in (c.geom[0],c.geom[1]):
            f=np.zeros(6); mujoco.mj_contactForce(m,d,i,f)
            other = c.geom[1] if c.geom[0]==og else c.geom[0]
            if other in (gL,gR): fin+=abs(f[0])
            elif other==tt: tab+=abs(f[0])
    return fin,tab

grip=15.0
for _ in range(1500):
    d.qfrc_applied[v1]=-grip; mujoco.mj_step(m,d)
fin,tab=contacts(m,d)
print('after close: fingerN=%.2f tableN=%.2f obj_z=%.4f' % (fin,tab,d.qpos[oq+2]))

z0=d.qpos[oq+2]; h0=d.xpos[hand][2]

# Which joint direction actually raises the hand at THIS pose? Probe each joint
# with a small perturbation on a scratch copy rather than assuming.
best=None
for k in range(7):
    for sign in (+1.0,-1.0):
        d2=mujoco.MjData(m)
        d2.qpos[:]=d.qpos; d2.qvel[:]=0
        d2.qpos[adr[k]] += sign*0.01
        mujoco.mj_forward(m,d2)
        dz=d2.xpos[hand][2]-h0
        if best is None or dz>best[0]: best=(dz,k,sign)
dz,k_lift,sgn = best
print('lift axis: joint%d sign %+.0f  (%.4f m per 0.01 rad)' % (k_lift+1, sgn, dz))

tgt=[d.qpos[x] for x in adr]
step = sgn*0.0006
for cyc in range(400):
    tgt[k_lift] += step
    for a,v,q in zip(adr,vadr,tgt): d.qpos[a]=q; d.qvel[v]=0.0
    for _ in range(5):
        d.qfrc_applied[v1]=-grip; mujoco.mj_step(m,d)
fin,tab=contacts(m,d)
dh=d.xpos[hand][2]-h0; do=d.qpos[oq+2]-z0
print('after raise: hand %+.4f obj %+.4f carry %.2f | fingerN=%.2f tableN=%.2f' % (dh,do,do/max(1e-9,abs(dh))*(1 if dh>0 else -1),fin,tab))
