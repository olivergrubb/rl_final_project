import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
import math

class ChaserNode(Node):
    def __init__(self):
        super().__init__('chaser_node')
        self.evader_pose = None
        self.chaser_pose = None

        self.evader_subscription = self.create_subscription(
            Odometry,
            '/evader0/odom',
            self.evader_odom_callback,
            10
        )

        self.chaser_subscription = self.create_subscription(
            Odometry,
            '/chaser0/odom',
            self.chaser_odom_callback,
            10
        )

        self.publisher = self.create_publisher(Twist, '/chaser0/cmd_vel', 10)

    def evader_odom_callback(self, msg):
        self.evader_pose = msg.pose.pose

    def chaser_odom_callback(self, msg):
        self.chaser_pose = msg.pose.pose
        if self.evader_pose is not None:
            self.calculate_and_publish_twist()

    def calculate_and_publish_twist(self):
        if self.evader_pose is not None:
            # Calculate the heading to intercept the evader
            dx = self.evader_pose.position.x - self.chaser_pose.position.x
            dy = self.evader_pose.position.y - self.chaser_pose.position.y
            desired_heading = math.atan2(dy, dx)

            # Calculate the difference between the current heading and the desired heading
            current_heading = math.atan2(
                2.0 * (self.chaser_pose.orientation.w * self.chaser_pose.orientation.z +
                    self.chaser_pose.orientation.x * self.chaser_pose.orientation.y),
                1.0 - 2.0 * (self.chaser_pose.orientation.y**2 + self.chaser_pose.orientation.z**2)
            )

            heading_difference = desired_heading - current_heading

            # Adjust the angular velocity based on the heading difference
            twist_msg = Twist()
            twist_msg.linear.x = 0.4  # Adjust this value as needed
            twist_msg.angular.z = heading_difference  # Adjust this value as needed

            self.publisher.publish(twist_msg)

def main(args=None):
    rclpy.init(args=args)
    chaser_node = ChaserNode()
    rclpy.spin(chaser_node)
    chaser_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
