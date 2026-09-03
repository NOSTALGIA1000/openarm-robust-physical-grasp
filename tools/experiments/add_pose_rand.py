"""Per-trial randomised object pose: writes a scene variant with the object
displaced and yawed, so a batch samples a distribution of initial conditions
instead of repeating one condition N times."""
import re, sys, math, random, os

MODEL="/home/nostalgia/ros2_ws_course_repro/ros2_ws/src/openarm_moveit_config/model/"
base=open(MODEL+"table_scene_physics.xml").read()

seed=int(sys.argv[1]); dx_mm=float(sys.argv[2]); dy_mm=float(sys.argv[3]); yaw_deg=float(sys.argv[4])
out=sys.argv[5]

m=re.search(r'<body name="physics_object" pos="([-\d.]+) ([-\d.]+) ([-\d.]+)"', base)
x,y,z=(float(g) for g in m.groups())
nx, ny = x + dx_mm/1000.0, y + dy_mm/1000.0
half=math.radians(yaw_deg)/2.0
quat="%.6f 0 0 %.6f" % (math.cos(half), math.sin(half))
s=base.replace(m.group(0),
    '<body name="physics_object" pos="%.5f %.5f %.5f" quat="%s"' % (nx,ny,z,quat))
open(MODEL+out,"w").write(s)
print("%s: pos=(%.4f, %.4f) yaw=%.1f deg" % (out,nx,ny,yaw_deg))
