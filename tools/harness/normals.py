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
m=mujoco.MjModel.from_xml_path('table_scene_physics.xml'); d=mujoco.MjData(m)
adr=[m.jnt_qposadr[mujoco.mj_name2id(m,mujoco.mjtObj.mjOBJ_JOINT,n)] for n in ARM]
j1=mujoco.mj_name2id(m,mujoco.mjtObj.mjOBJ_JOINT,'openarm_left_finger_joint1')
j2=mujoco.mj_name2id(m,mujoco.mjtObj.mjOBJ_JOINT,'openarm_left_finger_joint2')
a1,a2=m.jnt_qposadr[j1],m.jnt_qposadr[j2]; v1=m.jnt_dofadr[j1]
gL=mujoco.mj_name2id(m,mujoco.mjtObj.mjOBJ_GEOM,'openarm_left_left_finger_collision')
gR=mujoco.mj_name2id(m,mujoco.mjtObj.mjOBJ_GEOM,'openarm_left_right_finger_collision')
og=mujoco.mj_name2id(m,mujoco.mjtObj.mjOBJ_GEOM,'physics_object_geom')
oq=m.jnt_qposadr[m.body_jntadr[mujoco.mj_name2id(m,mujoco.mjtObj.mjOBJ_BODY,'physics_object')]]
for n,a in zip(ARM,adr): d.qpos[a]=Q[n]
d.qpos[a1]=Q[fk]; d.qpos[a2]=Q[fk]
mujoco.mj_forward(m,d)
for _ in range(1500): d.qfrc_applied[v1]=-15.0; mujoco.mj_step(m,d)
print('object centre: %s' % np.round(d.qpos[oq:oq+3],4))
print('pad L centre : %s' % np.round(d.geom_xpos[gL],4))
print('pad R centre : %s' % np.round(d.geom_xpos[gR],4))
print()
print('finger-object contacts:')
for i in range(d.ncon):
    c=d.contact[i]
    if og in (c.geom[0],c.geom[1]):
        o=c.geom[1] if c.geom[0]==og else c.geom[0]
        nm=mujoco.mj_id2name(m,mujoco.mjtObj.mjOBJ_GEOM,o)
        if o not in (gL,gR): continue
        f=np.zeros(6); mujoco.mj_contactForce(m,d,i,f)
        n=c.frame[0:3]
        print('  %-38s pos=%s  normal=%s  |Fn|=%6.2f  dist=%+.5f' %
              (nm, np.round(c.pos,4), np.round(n,3), abs(f[0]), c.dist))
