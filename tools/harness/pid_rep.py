"""Stage 3 proper: actuator-style PID on the arm, tested on the REPRESENTATIVE
harness (real grasp pose, object resting on the table)."""
import mujoco, numpy as np, json, os
os.chdir('/home/nostalgia/ros2_ws_course_repro/ros2_ws/src/openarm_moveit_config/model')
trace=json.load(open('/tmp/joint_trace.json'))
fk='openarm_left_finger_joint1'
opened=False; Q=None
for r in trace:
    v=r['q'].get(fk,0.0)
    if v>0.020: opened=True
    elif opened and v<0.015: Q=r['q']; break
ARM=['openarm_left_joint%d'%i for i in range(1,8)]
LIM=[40,40,27,27,7,7,7]

def trial(kp,kd,armature,gravcomp=True,grip=15.0,label=''):
    m=mujoco.MjModel.from_xml_path('table_scene_physics.xml'); d=mujoco.MjData(m)
    adr=[m.jnt_qposadr[mujoco.mj_name2id(m,mujoco.mjtObj.mjOBJ_JOINT,n)] for n in ARM]
    vadr=[m.jnt_dofadr[mujoco.mj_name2id(m,mujoco.mjtObj.mjOBJ_JOINT,n)] for n in ARM]
    for v in vadr: m.dof_armature[v]=armature
    j1=mujoco.mj_name2id(m,mujoco.mjtObj.mjOBJ_JOINT,'openarm_left_finger_joint1')
    j2=mujoco.mj_name2id(m,mujoco.mjtObj.mjOBJ_JOINT,'openarm_left_finger_joint2')
    a1,a2=m.jnt_qposadr[j1],m.jnt_qposadr[j2]; v1=m.jnt_dofadr[j1]
    gL=mujoco.mj_name2id(m,mujoco.mjtObj.mjOBJ_GEOM,'openarm_left_left_finger_collision')
    gR=mujoco.mj_name2id(m,mujoco.mjtObj.mjOBJ_GEOM,'openarm_left_right_finger_collision')
    og=mujoco.mj_name2id(m,mujoco.mjtObj.mjOBJ_GEOM,'physics_object_geom')
    tt=mujoco.mj_name2id(m,mujoco.mjtObj.mjOBJ_GEOM,'table_top')
    oq=m.jnt_qposadr[m.body_jntadr[mujoco.mj_name2id(m,mujoco.mjtObj.mjOBJ_BODY,'physics_object')]]
    hand=mujoco.mj_name2id(m,mujoco.mjtObj.mjOBJ_BODY,'openarm_left_hand')
    for n,a in zip(ARM,adr): d.qpos[a]=Q[n]
    d.qpos[a1]=Q[fk]; d.qpos[a2]=Q[fk]
    mujoco.mj_forward(m,d)
    tgt=[d.qpos[a] for a in adr]
    def ctrl():
        for k,(a,v) in enumerate(zip(adr,vadr)):
            u=kp*(tgt[k]-d.qpos[a]) - kd*d.qvel[v]
            if gravcomp: u+=d.qfrc_bias[v]
            d.qfrc_applied[v]=float(np.clip(u,-LIM[k],LIM[k]))
    for _ in range(1500):
        ctrl(); d.qfrc_applied[v1]=-grip; mujoco.mj_step(m,d)
    def cf():
        fin=tab=0.0
        for i in range(d.ncon):
            c=d.contact[i]
            if og in (c.geom[0],c.geom[1]):
                f=np.zeros(6); mujoco.mj_contactForce(m,d,i,f)
                o=c.geom[1] if c.geom[0]==og else c.geom[0]
                if o in (gL,gR): fin+=abs(f[0])
                elif o==tt: tab+=abs(f[0])
        return fin,tab
    f0,_=cf(); z0=d.qpos[oq+2]; h0=d.xpos[hand][2]
    best=None
    for k in range(7):
        for sg in (1.0,-1.0):
            d2=mujoco.MjData(m); d2.qpos[:]=d.qpos; mujoco.mj_forward(m,d2)
            d2.qpos[adr[k]]+=sg*0.01; mujoco.mj_forward(m,d2)
            dz=d2.xpos[hand][2]-h0
            if best is None or dz>best[0]: best=(dz,k,sg)
    _,kl,sg=best
    div=False
    for c in range(400):
        tgt[kl]+=sg*0.0006
        for _ in range(5):
            ctrl(); d.qfrc_applied[v1]=-grip; mujoco.mj_step(m,d)
            if not np.all(np.isfinite(d.qpos)): div=True; break
        if div: break
    f1,t1=cf()
    dh=d.xpos[hand][2]-h0; do=d.qpos[oq+2]-z0
    trk=abs(d.qpos[adr[kl]]-tgt[kl])
    print('%-40s hand %+.4f obj %+.4f carry %5.2f | gripN %5.1f->%5.1f tableN %4.2f trk %.4f %s' %
          (label,dh,do,do/max(1e-9,dh),f0,f1,t1,trk,'DIVERGED' if div else ''))

for kp,kd,arm,gc in [(2000,60,0.10,True),(2000,60,0.10,False),
                     (5000,150,0.10,False),(2000,60,0.30,False),
                     (5000,150,0.30,False),(10000,200,0.30,False)]:
    try: trial(kp,kd,arm,gravcomp=gc,label='kp=%-6d kd=%-4d arm=%.2f gravcomp=%s'%(kp,kd,arm,gc))
    except Exception as e: print('ERROR',e)
