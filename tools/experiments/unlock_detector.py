p="/home/nostalgia/ros2_ws_course_repro/ros2_ws/src/openarm_moveit_config/launch/apple_detector.py"
s=open(p).read()

# The detector latches its first detection and rebroadcasts it forever. That is
# fine for the demo (a stable target) but makes a pre-grasp visual correction
# impossible in principle: no second measurement is ever produced. Make the
# latch optional, defaulting to the existing behaviour so demo_mode is untouched.
a = """        self.fixed_world_point = None  # 用于存储锁定的坐标
        self.lock_time = None          # 用于记录锁定的起始时间
"""
assert s.count(a) == 1
b = """        self.fixed_world_point = None  # 用于存储锁定的坐标
        self.lock_time = None          # 用于记录锁定的起始时间

        # Latching the first detection is the demo's behaviour and is kept as the
        # default. physics_mode needs repeated measurements: a pre-grasp visual
        # correction requires a second observation, which a latched detector can
        # never provide.
        self.declare_parameter('lock_first_detection', True)
        self.lock_enabled = self.get_parameter('lock_first_detection').value
        if not self.lock_enabled:
            self.get_logger().info('=== detector running UNLATCHED (continuous re-detection) ===')
"""
s = s.replace(a, b)

a2 = """        if self.fixed_world_point is not None:
"""
assert s.count(a2) == 1
b2 = """        if self.lock_enabled and self.fixed_world_point is not None:
"""
s = s.replace(a2, b2)

# When unlatched, keep publishing fresh measurements instead of storing one.
a3 = """                            self.fixed_world_point = point_world.point # 永久保存这个坐标
                            self.lock_time = self.get_clock().now()    # 记录开始时间
"""
assert s.count(a3) == 1
b3 = """                            if self.lock_enabled:
                                self.fixed_world_point = point_world.point
                                self.lock_time = self.get_clock().now()
                            else:
                                self.fixed_world_point = point_world.point
"""
s = s.replace(a3, b3)

a4 = """                            self.get_logger().info(
                                f"🎯 目标初次锁定! World:({point_world.point.x:.2f}, {point_world.point.y:.2f}, {point_world.point.z:.2f})"
                            )
"""
assert s.count(a4) == 1
b4 = """                            if self.lock_enabled:
                                self.get_logger().info(
                                    f"🎯 目标初次锁定! World:({point_world.point.x:.2f}, {point_world.point.y:.2f}, {point_world.point.z:.2f})"
                                )
                            else:
                                self.get_logger().info(
                                    f"detected World:({point_world.point.x:.4f}, {point_world.point.y:.4f}, {point_world.point.z:.4f})",
                                    throttle_duration_sec=2.0
                                )
"""
s = s.replace(a4, b4)
open(p, "w").write(s)
print("detector: latch now optional, default unchanged")
