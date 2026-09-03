import rclpy, collections, atexit
from rclpy.node import Node
from openarm_grasp_msgs.msg import GraspRawState

state = {'seen': collections.Counter(), 'maxf': collections.defaultdict(float),
         'first_move': None, 'x0': None, 'pairs': collections.Counter()}

def report():
    print('=== geoms contacting the object ===', flush=True)
    for g, c in state['seen'].most_common():
        print('  %-45s frames=%5d  maxforce=%7.2f' % (g, c, state['maxf'][g]), flush=True)
    print('=== contact-set combinations ===', flush=True)
    for combo, c in state['pairs'].most_common(8):
        print('  %-60s frames=%5d' % (combo, c), flush=True)
    fm = state['first_move']
    print('first motion >1mm at sim_time=%s, contacts then: %s' %
          (fm[0] if fm else None, fm[1] if fm else None), flush=True)

atexit.register(report)

class Probe(Node):
    def __init__(self):
        super().__init__('contact_probe')
        self.create_subscription(GraspRawState, '/grasp_raw_state', self.cb, 50)

    def cb(self, m):
        x = m.object_pose.position.x
        if state['x0'] is None:
            state['x0'] = x
        if state['first_move'] is None and abs(x - state['x0']) > 0.001:
            state['first_move'] = (round(m.sim_time, 3), list(m.contact_geoms))
        names = sorted(m.contact_geoms)
        state['pairs']['+'.join(names) if names else '(none)'] += 1
        for g, f in zip(m.contact_geoms, m.contact_normal_forces):
            state['seen'][g] += 1
            state['maxf'][g] = max(state['maxf'][g], abs(f))

rclpy.init()
try:
    rclpy.spin(Probe())
except BaseException:
    pass
