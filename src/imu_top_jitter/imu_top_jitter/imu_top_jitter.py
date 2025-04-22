import rclpy
from rclpy.node import Node
from novatel_oem7_msgs.msg import RAWIMU
from std_msgs.msg import Float64
import time
from collections import deque
import statistics

class Imu_top_jitter(Node):
    def __init__(self):
        super().__init__('imu_top_jitter')

        # Store (timestamp, rclpy_time) for last 1 second
        self.time_deltas = deque()
        self.last_msg_time = None

        self.subscription = self.create_subscription(
            RAWIMU,
            '/novatel_top/rawimu',
            self.imu_callback,
            10  # QoS
        )

        self.jitter_publisher = self.create_publisher(
            Float64,
            'imu_top/jitter',
            10
        )

    def imu_callback(self, msg: RAWIMU):
        now = self.get_clock().now().nanoseconds / 1e9  # current ROS time in seconds

        if self.last_msg_time is not None:
            delta_t = now - self.last_msg_time

            # Append the new delta and time
            self.time_deltas.append((now, delta_t))

            # Remove entries older than 1 second
            while self.time_deltas and self.time_deltas[0][0] < now - 1.0:
                self.time_deltas.popleft()

            # Compute jitter if we have at least 2 deltas
            if len(self.time_deltas) >= 2:
                jitter = statistics.variance([dt for _, dt in self.time_deltas])
                self.get_logger().info(f"Jitter: {jitter:.6f} s^2")

                msg_out = Float64()
                msg_out.data = jitter
                self.jitter_publisher.publish(msg_out)

        self.last_msg_time = now

def main(args=None):
    rclpy.init(args=args)
    node = Imu_top_jitter()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
