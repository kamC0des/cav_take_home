import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32
from std_msgs.msg import Float64
from raptor_dbw_msgs.msg import SteeringExtendedReport
import math

class Vel_angle_x_front_left(Node):
    def __init__(self):
        super().__init__('vel_angle_x_front_left')

        self.primary_steering_angle = None  # From /raptor_dbw_interface/steering_extended_report  (Float32)
        self.vel_x_fl = None  # From
        self.vel_y_fl = None
        # Subscriber to rear right velocity (from another node)
        self.create_subscription(
            SteeringExtendedReport,
            '/raptor_dbw_interface/steering_extended_report',
            self.steering_ex_callback,
            10
        )

        # Subscriber to rear right wheel speed report
        self.create_subscription(
            Float64,
            '/vel_x_front_left',
            self.vel_x_fl_callback,
            10
        )
         # Subscriber to rear right wheel speed report
        self.create_subscription(
            Float64,
            '/vel_y_front_left',
            self.vel_y_fl_callback,
            10
        )


        # Publisher for processed slip
        self.publisher = self.create_publisher(
            Float64,
            'vel_angle_x_front_left',
            10
        )

    def steering_ex_callback(self, msg):
        self.primary_steering_angle = msg.primary_steering_angle_fbk / 15.0
        self.get_logger().info(f"Received primary steering angle: {self.primary_steering_angle}")
        self.try_publish()

    def vel_x_fl_callback(self, msg):
        self.vel_x_fl = msg.data
        self.get_logger().info(f"Received x velocity of front left wheel: {self.vel_x_fl}")
        self.try_publish()

    def vel_y_fl_callback(self, msg):
        self.vel_y_fl = msg.data
        self.get_logger().info(f"Received y velocity of front left wheel: {self.vel_y_fl}")
        self.try_publish()

    def try_publish(self):
        if self.vel_x_fl is not None and self.vel_y_fl is not None and self.primary_steering_angle is not None:
            vel_angle_x_fl = math.cos(self.primary_steering_angle) * self.vel_x_fl - math.sin(self.primary_steering_angle) * self.vel_y_fl

            msg = Float64()
            msg.data = vel_angle_x_fl

            self.publisher.publish(msg)
            self.get_logger().info(f"Published x velocity angle of front right wheel: {vel_angle_x_fl}")

def main(args=None):
    rclpy.init(args=args)
    node = Vel_angle_x_front_left()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
