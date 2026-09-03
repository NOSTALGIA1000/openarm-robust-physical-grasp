import rclpy, json, atexit
from rclpy.node import Node
from sensor_msgs.msg import JointState
rows=[]
def dump():
    json.dump(rows, open('/tmp/joint_trace.json','w'))
    print('recorded %d joint samples' % len(rows), flush=True)
atexit.register(dump)
class R(Node):
    def __init__(self):
        super().__init__('joint_recorder')
        self.create_subscription(JointState,'/joint_states',self.cb,50)
    def cb(self,m):
        d=dict(zip(m.name,m.position))
        arm={k:d[k] for k in d if 'openarm_left_joint' in k or 'left_finger' in k}
        if arm:
            rows.append({'t': m.header.stamp.sec + m.header.stamp.nanosec*1e-9, 'q': arm})
rclpy.init()
try: rclpy.spin(R())
except BaseException: pass
