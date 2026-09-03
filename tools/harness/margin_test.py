import mujoco, numpy as np, json, re, os
os.chdir('/home/nostalgia/ros2_ws_course_repro/ros2_ws/src/openarm_moveit_config/model')
trace=json.load(open('/tmp/joint_trace.json'))
fk='openarm_left_finger_joint1'
opened=False; Q=None
for r in trace:
    v=r['q'].get(fk,0.0)
    if v>0.020: opened=True
    elif opened and v<0.015: Q=r['q']; break
ARM=['openarm_left_joint%d'%i for i in range(1,8)]
BASE=open('table_scene_physics.xml').read(); MJ=open('openarm_bimanual_physics.xml').read()
NAME=r'(name="openarm_(?:left|right)_(?:left|right)_finger_collision"(?:[^>]*?))'

def build(margin, grip_close=None):
    s=re.sub(NAME+r'margin="[^"]*"', lambda m: m.group(1)+'margin="%s"'%margin, MJ)
    open('_mtf.xml','w').write(s)
    open('_mts.xml','w').write(BASE.replace('openarm_bimanual_physics.xml','_mtf.xml'))
    return mujoco.MjModel.from_xml_path('_mts.xml')

def trial(m,grip=15.0,close_extra=0.0):
    d=mujoco.MjData(m)
    adr=[m.jnt_qposadr[mujoco.mj_name2id(m,mujoco.mjtObj.mjOBJ_JOINT,n)] for n in ARM]
    vadr=[m.jnt_dofadr[mujoco.mj_name2id(m,mujoco.mjtObj.mjOBJ_JOINT,n)] for n in ARM]
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
    d.qpos[a1]=Q[fk]-close_extra; d.qpos[a2]=d.qpos[a1]
    mujoco.mj_forward(m,d)
    for _ in range(1500): d.qfrc_applied[v1]=-grip; mujoco.mj_step(m,d)
    def cf():
        fin=tab=0.0; dmin=9.9
        for i in range(d.ncon):
            c=d.contact[i]
            if og in (c.geom[0],c.geom[1]):
                f=np.zeros(6); mujoco.mj_contactForce(m,d,i,f)
                o=c.geom[1] if c.geom[0]==og else c.geom[0]
                if o in (gL,gR): fin+=abs(f[0]); dmin=min(dmin,c.dist)
                elif o==tt: tab+=abs(f[0])
        return fin,tab,dmin
    f0,_,d0=cf(); z0=d.qpos[oq+2]; h0=d.xpos[hand][2]
    best=None
    for k in range(7):
        for sg in (1.0,-1.0):
            d2=mujoco.MjData(m); d2.qpos[:]=d.qpos; mujoco.mj_forward(m,d2)
            d2.qpos[adr[k]]+=sg*0.01; mujoco.mj_forward(m,d2)
            dz=d2.xpos[hand][2]-h0
            if best is None or dz>best[0]: best=(dz,k,sg)
    _,kl,sg=best
    tgt=[d.qpos[x] for x in adr]
    for c in range(400):
        tgt[kl]+=sg*0.0006
        # Match the patched plugin: hold joints at target, but give the moving
        # joint a velocity consistent with its commanded step. Zeroing every
        # qvel (as an earlier version of this harness did) reproduces the
        # pre-fix plugin, not the current one.
        for a,v,q in zip(adr,vadr,tgt):
            d.qvel[v]=(q-d.qpos[a])/0.01
            d.qpos[a]=q
        for _ in range(5): d.qfrc_applied[v1]=-grip; mujoco.mj_step(m,d)
    f1,t1,_=cf()
    dh=d.xpos[hand][2]-h0; do=d.qpos[oq+2]-z0
    return dh,do,do/max(1e-9,dh),f0,d0,f1,t1

for margin,extra,g,label in [('0.001',0.0,15.0,'current margin 0.001'),
                             ('0',    0.0,15.0,'margin 0'),
                             ('0',    0.0,30.0,'margin 0, grip 30N'),
                             ('0',    0.003,15.0,'margin 0, close 3mm deeper'),
                             ('0.0001',0.0,15.0,'margin 0.0001')]:
    try:
        m=build(margin)
        dh,do,c,f0,d0,f1,t1=trial(m,grip=g,close_extra=extra)
        print('%-28s hand %+.4f obj %+.4f carry %6.2f | gripN %5.1f->%5.1f dist %+.5f tableN %4.2f' %
              (label,dh,do,c,f0,f1,d0,t1))
    except Exception as e:
        print('%-28s ERROR %s' % (label,str(e)[:60]))
for f in ('_mtf.xml','_mts.xml'):
    if os.path.exists(f): os.remove(f)
