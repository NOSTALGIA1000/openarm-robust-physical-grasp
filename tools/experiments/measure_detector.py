"""Measure the real detector error: compare /apple_location against the
simulator's ground-truth object pose across a set of object placements.
This is what relates an injected calibration bias to actual perception error."""
import rclpy, time, json, math
from rclpy.node import Node
from geometry_msgs.msg import Point
from openarm_grasp_msgs.msg import GraspRawState
from openarm_grasp_msgs.srv import ResetObject

POSES = [(0.300, 0.000, 0), (0.320, 0.020, 0), (0.280, -0.020, 0),
         (0.310, -0.015, 20), (0.290, 0.015, -20), (0.300, 0.000, 45),
         (0.330, 0.000, 0), (0.270, 0.000, 0), (0.300, 0.030, 0),
         (0.300, -0.030, 0)]

class M(Node):
    def __init__(self):
        super().__init__('detector_accuracy')
        self.det=None; self.truth=None
        self.create_subscription(Point,'/apple_location',self.on_det,10)
        self.create_subscription(GraspRawState,'/grasp_raw_state',self.on_truth,10)
        self.cli=self.create_client(ResetObject,'/reset_object')
    def on_det(self,m): self.det=(m.x,m.y,m.z)
    def on_truth(self,m): self.truth=(m.object_pose.position.x,
                                      m.object_pose.position.y,
                                      m.object_pose.position.z)

rclpy.init()
n=M()
while not n.cli.wait_for_service(timeout_sec=2.0):
    n.get_logger().info('waiting for /reset_object')
rows=[]
for (x,y,yaw) in POSES:
    req=ResetObject.Request(); req.x=x; req.y=y; req.yaw_deg=float(yaw); req.use_spawn=False
    fut=n.cli.call_async(req)
    rclpy.spin_until_future_complete(n,fut,timeout_sec=10.0)
    n.det=None
    t0=time.time()
    while time.time()-t0 < 8.0:
        rclpy.spin_once(n,timeout_sec=0.2)
        if n.det is not None and n.truth is not None: break
    if n.det is None or n.truth is None:
        print('pose (%.3f,%.3f,%d): NO DETECTION' % (x,y,yaw), flush=True); continue
    dx=n.det[0]-n.truth[0]; dy=n.det[1]-n.truth[1]; dz=n.det[2]-n.truth[2]
    rows.append(dict(px=x,py=y,yaw=yaw,det=n.det,truth=n.truth,
                     ex=dx,ey=dy,ez=dz,exy=math.hypot(dx,dy)))
    print('pose (%.3f,%+.3f,%3d)  det=(%.4f,%+.4f,%.4f) truth=(%.4f,%+.4f,%.4f)  err x%+.4f y%+.4f z%+.4f  |xy|=%.4f'
          % (x,y,yaw,n.det[0],n.det[1],n.det[2],n.truth[0],n.truth[1],n.truth[2],dx,dy,dz,math.hypot(dx,dy)), flush=True)
json.dump(rows,open('/tmp/detector_accuracy.json','w'),indent=2)
if rows:
    import statistics as st
    print()
    print('n=%d' % len(rows))
    for k in ('ex','ey','ez','exy'):
        v=[r[k] for r in rows]
        print('  %-4s mean %+.4f  sd %.4f  min %+.4f  max %+.4f' % (k, st.mean(v), st.pstdev(v), min(v), max(v)))
