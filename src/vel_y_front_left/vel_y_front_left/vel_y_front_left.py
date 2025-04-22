import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Vector3
from std_msgs.msg import Float64

class Vel_y_front_left(Node):
    def __init__(self):
        super().__init__('vel_y_front_left')

        self.subscription = self.create_subscription(
            Odometry,
            '/vehicle/uva_odometry',  # Replace with your topic if different
            self.odom_callback,
            10
        )

        self.publisher = self.create_publisher(
            Float64,
            '/vel_y_front_left',
            10
        )

    def odom_callback(self, msg):
        # Extract linear and angular velocity 
        linear = msg.twist.twist.linear
        angular = msg.twist.twist.angular
        self.get_logger().info(f"Received linear velocity: x={linear.x:.2f}, y={linear.y:.2f}, z={linear.z:.2f}")
        self.get_logger().info(f"Received angular velocity: x={angular.x:.2f}, y={angular.y:.2f}, z={angular.z:.2f}")

        # Perform some calculation (finding the x velocity of the rear right wheel)
        linear_vel_y = linear.y
        angular_yaw = angular.z
        long_dist = 1.7238
        vel_y_fl = linear_vel_y + angular_yaw * long_dist
        processed = Float64()
        processed.data = vel_y_fl
        

        # Publish processed velocity
        self.publisher.publish(processed)
        self.get_logger().info(f"Published processed y velocity of the front left wheel={processed.data}")

def main(args=None):
    rclpy.init(args=args)
    node = Vel_y_front_left()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
