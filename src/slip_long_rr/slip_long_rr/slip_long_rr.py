import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64
from raptor_dbw_msgs.msg import WheelSpeedReport

class Slip_long_rr(Node):
    def __init__(self):
        super().__init__('slip_long_rr')

        self.vel_x_rr = None  # From /vel_x_rear_right (Float64)
        self.wheel_speed_rr = None  # From /raptor_dbw_interface/wheel_speed_report (WheelSpeedReport)

        # Subscriber to rear right velocity (from another node)
        self.create_subscription(
            Float64,
            '/vel_x_rear_right',
            self.vel_x_callback,
            10
        )

        # Subscriber to rear right wheel speed report
        self.create_subscription(
            WheelSpeedReport,
            '/raptor_dbw_interface/wheel_speed_report',
            self.wheel_speed_callback,
            10
        )

        # Publisher for processed slip
        self.publisher = self.create_publisher(
            Float64,
            '/slip/long/rr',
            10
        )

    def vel_x_callback(self, msg):
        self.vel_x_rr = msg.data
        self.get_logger().info(f"Received rear right velocity: {self.vel_x_rr}")
        self.try_publish()

    def wheel_speed_callback(self, msg):
        self.wheel_speed_rr = msg.rear_right
        self.get_logger().info(f"Received rear right wheel speed: {self.wheel_speed_rr}")
        self.try_publish()

    def try_publish(self):
        if self.vel_x_rr is not None and self.wheel_speed_rr is not None:
            # Example calculation (slip = 1 - (vehicle speed / wheel speed))
            if self.wheel_speed_rr != 0:
                #slip = 1.0 - (self.vel_x_rr / self.wheel_speed_rr)
                slip = (self.wheel_speed_rr - self.vel_x_rr) / self.vel_x_rr
        
            else:
                slip = 0.0  # Avoid division by zero

            msg = Float64()
            msg.data = slip

            self.publisher.publish(msg)
            self.get_logger().info(f"Published slip: {slip}")

def main(args=None):
    rclpy.init(args=args)
    node = Slip_long_rr()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
