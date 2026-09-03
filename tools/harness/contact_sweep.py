import mujoco, numpy as np, re, os
BASE=open('table_scene_physics.xml').read()
MJ=open('openarm_bimanual_physics.xml').read()
ARM=['openarm_left_joint%d'%i for i in range(1,8)]
NAME=r'(name="openarm_(?:left|right)_(?:left|right)_finger_collision"(?:[^>]*?))'

def build(condim,fric,solimp):
    s=MJ
    s=re.sub(NAME+r'condim="\d"', lambda mm: mm.group(1)+'condim="%d"'%condim, s)
    s=s.replace('friction="2.0 1.0 0.01"','friction="%s"'%fric)
    if solimp:
        s=re.sub(NAME+r'solimp="[^"]*"', lambda mm: mm.group(1)+'solimp="%s"'%solimp, s)
    open('_sw_f.xml','w').write(s)
    open('_sw_s.xml','w').write(BASE.replace('openarm_bimanual_physics.xml','_sw_f.xml'))
    return mujoco.MjModel.from_xml_path('_sw_s.xml')

def carry(m,grip=15.0):
    d=mujoco.MjData(m)
    adr=[m.jnt_qposadr[mujoco.mj_name2id(m,mujoco.mjtObj.mjOBJ_JOINT,n)] for n in ARM]
    vadr=[m.jnt_dofadr[mujoco.mj_name2id(m,mujoco.mjtObj.mjOBJ_JOINT,n)] for n in ARM]
    j1=mujoco.mj_name2id(m,mujoco.mjtObj.mjOBJ_JOINT,'openarm_left_finger_joint1')
    j2=mujoco.mj_name2id(m,mujoco.mjtObj.mjOBJ_JOINT,'openarm_left_finger_joint2')
    a1,a2=m.jnt_qposadr[j1],m.jnt_qposadr[j2]; v1=m.jnt_dofadr[j1]
    gL=mujoco.mj_name2id(m,mujoco.mjtObj.mjOBJ_GEOM,'openarm_left_left_finger_collision')
    gR=mujoco.mj_name2id(m,mujoco.mjtObj.mjOBJ_GEOM,'openarm_left_right_finger_collision')
    og=mujoco.mj_name2id(m,mujoco.mjtObj.mjOBJ_GEOM,'physics_object_geom')
    oj=m.body_jntadr[mujoco.mj_name2id(m,mujoco.mjtObj.mjOBJ_BODY,'physics_object')]
    oq=m.jnt_qposadr[oj]; ov=m.jnt_dofadr[oj]
    hand=mujoco.mj_name2id(m,mujoco.mjtObj.mjOBJ_BODY,'openarm_left_hand')
    sav=m.opt.gravity.copy(); m.opt.gravity[:]=0
    d.qpos[a1]=0.030; d.qpos[a2]=0.030; mujoco.mj_forward(m,d)
    d.qpos[oq:oq+3]=(d.geom_xpos[gL]+d.geom_xpos[gR])/2.0; d.qpos[oq+3:oq+7]=[1,0,0,0]
    mujoco.mj_forward(m,d)
    for _ in range(1500): d.qfrc_applied[v1]=-grip; mujoco.mj_step(m,d)
    m.opt.gravity[:]=sav; d.qvel[ov:ov+6]=0
    tgt=[d.qpos[a] for a in adr]; z0=d.qpos[oq+2]; h0=d.xpos[hand][2]
    for cyc in range(300):
        tgt[1]-=0.0006
        for a,v,q in zip(adr,vadr,tgt): d.qpos[a]=q; d.qvel[v]=0.0
        for _ in range(5): d.qfrc_applied[v1]=-grip; mujoco.mj_step(m,d)
    f=0.0
    for i in range(d.ncon):
        c=d.contact[i]
        if og in (c.geom[0],c.geom[1]) and (gL in (c.geom[0],c.geom[1]) or gR in (c.geom[0],c.geom[1])):
            ff=np.zeros(6); mujoco.mj_contactForce(m,d,i,ff); f+=abs(ff[0])
    dh=d.xpos[hand][2]-h0; do=d.qpos[oq+2]-z0
    return dh,do,do/max(1e-9,dh),f

cases=[(6,'2.0 1.0 0.01',None,'baseline (current)'),
       (6,'2.0 1.0 0.01','0.95 0.99 0.001','solimp 0.95/0.99'),
       (6,'2.0 1.0 0.01','0.97 0.995 0.001','solimp 0.97/0.995'),
       (6,'2.0 1.0 0.01','0.98 0.997 0.001','solimp 0.98/0.997'),
       (6,'2.0 1.0 0.01','0.99 0.999 0.001','solimp 0.99/0.999'),
       (6,'2.0 1.0 0.01','0.99 0.999 0.005','solimp 0.99/0.999 w.005'),
       (3,'2.0 1.0 0.01','0.98 0.997 0.001','condim3 solimp 0.98/0.997')]
for condim,fric,solimp,label in cases:
    try:
        m=build(condim,fric,solimp)
        g=mujoco.mj_name2id(m,mujoco.mjtObj.mjOBJ_GEOM,'openarm_left_left_finger_collision')
        dh,do,c,f=carry(m)
        print('%-34s condim=%d fric=%s -> hand %+.4f obj %+.4f carry %5.2f gripN %5.1f' %
              (label,m.geom_condim[g],np.round(m.geom_friction[g],3),dh,do,c,f))
    except Exception as e:
        print('%-34s ERROR %s' % (label,str(e)[:70]))
for f in ('_sw_f.xml','_sw_s.xml'):
    if os.path.exists(f): os.remove(f)
