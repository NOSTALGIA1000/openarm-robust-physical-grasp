"""Offline Stage 3 probe: can actuator-style force control carry a gripped object,
where qpos-teleport could not? Runs in seconds, no ROS."""
import mujoco, numpy as np, sys

MODEL='table_scene_physics.xml'
ARM=['openarm_left_joint%d'%i for i in range(1,8)]

def setup(m,d,grip_force):
    j1=mujoco.mj_name2id(m,mujoco.mjtObj.mjOBJ_JOINT,'openarm_left_finger_joint1')
    j2=mujoco.mj_name2id(m,mujoco.mjtObj.mjOBJ_JOINT,'openarm_left_finger_joint2')
    a1,a2=m.jnt_qposadr[j1],m.jnt_qposadr[j2]; v1=m.jnt_dofadr[j1]
    gL=mujoco.mj_name2id(m,mujoco.mjtObj.mjOBJ_GEOM,'openarm_left_left_finger_collision')
    gR=mujoco.mj_name2id(m,mujoco.mjtObj.mjOBJ_GEOM,'openarm_left_right_finger_collision')
    oj=m.body_jntadr[mujoco.mj_name2id(m,mujoco.mjtObj.mjOBJ_BODY,'physics_object')]
    oq=m.jnt_qposadr[oj]; ov=m.jnt_dofadr[oj]
    sav=m.opt.gravity.copy(); m.opt.gravity[:]=0
    d.qpos[a1]=0.030; d.qpos[a2]=0.030
    mujoco.mj_forward(m,d)
    d.qpos[oq:oq+3]=(d.geom_xpos[gL]+d.geom_xpos[gR])/2.0
    d.qpos[oq+3:oq+7]=[1,0,0,0]
    mujoco.mj_forward(m,d)
    for _ in range(1500):
        d.qfrc_applied[v1]=-grip_force; mujoco.mj_step(m,d)
    m.opt.gravity[:]=sav; d.qvel[ov:ov+6]=0
    return v1,oq,ov

def gripN(m,d,og,gL,gR):
    tot=0.0
    for i in range(d.ncon):
        c=d.contact[i]
        if og in (c.geom[0],c.geom[1]) and (gL in (c.geom[0],c.geom[1]) or gR in (c.geom[0],c.geom[1])):
            f=np.zeros(6); mujoco.mj_contactForce(m,d,i,f); tot+=abs(f[0])
    return tot

def run(mode, kp=0.0, kd=0.0, gravcomp=False, armature=None, grip=15.0, label=''):
    m=mujoco.MjModel.from_xml_path(MODEL); d=mujoco.MjData(m)
    adr=[m.jnt_qposadr[mujoco.mj_name2id(m,mujoco.mjtObj.mjOBJ_JOINT,n)] for n in ARM]
    vadr=[m.jnt_dofadr[mujoco.mj_name2id(m,mujoco.mjtObj.mjOBJ_JOINT,n)] for n in ARM]
    if armature is not None:
        for v in vadr: m.dof_armature[v]=armature
    v1,oq,ov=setup(m,d,grip)
    gL=mujoco.mj_name2id(m,mujoco.mjtObj.mjOBJ_GEOM,'openarm_left_left_finger_collision')
    gR=mujoco.mj_name2id(m,mujoco.mjtObj.mjOBJ_GEOM,'openarm_left_right_finger_collision')
    og=mujoco.mj_name2id(m,mujoco.mjtObj.mjOBJ_GEOM,'physics_object_geom')
    hand=mujoco.mj_name2id(m,mujoco.mjtObj.mjOBJ_BODY,'openarm_left_hand')
    lim=[40,40,27,27,7,7,7]
    tgt=[d.qpos[a] for a in adr]
    z0=d.qpos[oq+2]; h0=d.xpos[hand][2]
    maxtrq=0.0; diverged=False
    for cyc in range(300):
        tgt[1]-=0.0006
        for _ in range(5):
            if mode=='teleport':
                for a,v,q in zip(adr,vadr,tgt): d.qpos[a]=q; d.qvel[v]=0.0
            else:
                for k,(a,v) in enumerate(zip(adr,vadr)):
                    u=kp*(tgt[k]-d.qpos[a]) - kd*d.qvel[v]
                    if gravcomp: u+=d.qfrc_bias[v]
                    u=float(np.clip(u,-lim[k],lim[k]))
                    d.qfrc_applied[v]=u
                    maxtrq=max(maxtrq,abs(u))
            d.qfrc_applied[v1]=-grip
            mujoco.mj_step(m,d)
            if not np.all(np.isfinite(d.qpos)): diverged=True; break
        if diverged: break
    dh=d.xpos[hand][2]-h0; do=d.qpos[oq+2]-z0
    trk=abs(d.qpos[adr[1]]-tgt[1])
    print('%-34s hand %+.4f obj %+.4f carry %5.2f  track_err %.4f rad  maxTrq %5.1f  gripN %5.1f %s' %
          (label, dh, do, do/max(1e-9,dh), trk, maxtrq, gripN(m,d,og,gL,gR), 'DIVERGED' if diverged else ''))

run('teleport', label='baseline: qpos teleport')
for kp,kd in [(50,5),(200,10),(500,20),(1000,40)]:
    run('pid', kp=kp, kd=kd, label='PID kp=%-5d kd=%-3d no gravcomp'%(kp,kd))
for kp,kd in [(200,10),(500,20)]:
    run('pid', kp=kp, kd=kd, gravcomp=True, label='PID kp=%-5d kd=%-3d +gravcomp'%(kp,kd))
for kp,kd in [(500,20),(1000,40)]:
    run('pid', kp=kp, kd=kd, gravcomp=True, armature=0.01, label='PID kp=%-5d kd=%-3d +grav +armature'%(kp,kd))
