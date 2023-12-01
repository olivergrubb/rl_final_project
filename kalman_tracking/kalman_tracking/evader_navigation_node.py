import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from math import atan2

class EvaderNavigationNode(Node):
    def __init__(self, goal_pose):
        super().__init__('evader_navigation_node')
        self.publisher = self.create_publisher(Twist, '/evader0/cmd_vel', 10)
        self.goal_pose = goal_pose

        # Subscribe to the current pose of the robot
        self.chaser_subscription = self.create_subscription(
            Odometry,
            '/chaser0/odom',
            self.chaser_odom_callback,
            10)
        self.subscription = self.create_subscription(
            Odometry,
            '/evader0/odom',
            self.odom_callback,
            10)
        self.current_pose = None
        self.chaser_pose = Odometry().pose.pose
        self.first_rotation = True
        self._start_time = self.get_clock().now()


    def odom_callback(self, msg):
        self.current_pose = msg.pose.pose
        if self.current_pose is not None:
            # Calculate the current angle between where the robot is currently facing and where it should face point at the goals x,y location
            robot_heading = atan2(
            2.0 * (self.current_pose.orientation.w * self.current_pose.orientation.z +
                self.current_pose.orientation.x * self.current_pose.orientation.y),
            1.0 - 2.0 * (self.current_pose.orientation.y**2 + self.current_pose.orientation.z**2)
            )
            goal_heading = atan2(self.goal_pose[1] - self.current_pose.position.y, self.goal_pose[0] - self.current_pose.position.x)
            angle = goal_heading - robot_heading
            
            # Rotate towards the goal pose
            if self.reached_goal(self.current_pose, self.goal_pose):
                self.get_logger().info(f'Goal reached.')
                # Write new line to file and time taken 
                time_taken = self.get_clock().now() - self._start_time
                time_taken = float(time_taken.nanoseconds / 1000000000)
                self.get_logger().info(f'Time taken: {time_taken}')
                with open("Results.txt", "a") as file:
                    file.write(f'GOAL REACHED {time_taken}\n')
                # Stop the robot
                request = Twist()
                request.linear.x = 0.0
                request.angular.z = 0.0
                self.publisher.publish(request)
                # Kill this node
                self.destroy_node()
            elif self.caught(self.current_pose):
                self.get_logger().info(f'Caught.')
                time_taken = self.get_clock().now() - self._start_time
                time_taken = float(time_taken.nanoseconds / 1000000000)
                self.get_logger().info(f'Time taken: {time_taken}')
                # Write new line to file
                with open("Results.txt", "a") as file:
                    file.write(f'CAUGHT {time_taken}\n')
                # Stop the robot
                request = Twist()
                request.linear.x = 0.0
                request.angular.z = 0.0
                self.publisher.publish(request)
                # Kill this node
                self.destroy_node()
            elif angle > 0.1:
                self.rotate(True)
            elif angle < -0.1:
                self.rotate(False)
            else:
                self.move_forward()

    def chaser_odom_callback(self, msg):
        self.chaser_pose = msg.pose.pose

    def rotate(self, clockwise=False):
        request = Twist()
        request.linear.x = 0.0
        if clockwise and not self.first_rotation:
            request.angular.z = 0.2
        else:
            request.angular.z = -0.2
        self.publisher.publish(request)

    def move_forward(self):
        request = Twist()
        request.linear.x = 0.3
        request.angular.z = 0.0
        self.publisher.publish(request)
        self.first_rotation = False

    def reached_goal(self, current_pose, goal_pose):
        distance = ((current_pose.position.x - goal_pose[0])**2 + (current_pose.position.y - goal_pose[1])**2)**0.5
        if distance < 0.5:
            return True
        else:
            return False
        
    def caught(self, current_pose):
        distance = ((current_pose.position.x - self.chaser_pose.position.x)**2 + (current_pose.position.y - self.chaser_pose.position.y)**2)**0.5
        if distance < 0.3:
            return True
        else:
            return False

