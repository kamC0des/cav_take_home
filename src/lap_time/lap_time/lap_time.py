import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64
from std_msgs.msg import Float32
import time

class LapTimer(Node):
    def __init__(self):
        super().__init__('lap_time')

        self.subscription = self.create_subscription(
            Float32,
            'curvilinear_distance',
            self.distance_callback,
            10
        )

        self.lap_publisher = self.create_publisher(
            Float64,
            'lap_time',
            10
        )

        self.prev_distance = None
        self.lap_started = False
        self.lap_start_time = None
        self.min_distance_threshold = 0.5   # meters (close enough to 0)
        self.lap_completion_threshold = 100.0  # must go at least this far before checking for lap end
        self.past_threshold = False

    def distance_callback(self, msg: Float64):
        current_time = self.get_clock().now().nanoseconds / 1e9
        distance = msg.data

        if not self.lap_started and distance < self.min_distance_threshold:
            # First time starting lap
            self.lap_start_time = current_time
            self.lap_started = True
            self.get_logger().info("Lap started!")

        elif self.lap_started and distance > self.lap_completion_threshold:
            # Lap is in progress
            self.prev_distance = distance
            self.past_threshold = True

        elif self.lap_started and distance < self.min_distance_threshold and self.past_threshold == True:
            # Lap completed (returned close to 0)
            lap_time = current_time - self.lap_start_time
            self.lap_started = False  # Reset for next lap
            self.past_threshold = False

            self.get_logger().info(f"Lap completed in {lap_time:.2f} seconds")

            msg_out = Float64()
            msg_out.data = lap_time
            self.lap_publisher.publish(msg_out)

def main(args=None):
    rclpy.init(args=args)
    node = LapTimer()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
