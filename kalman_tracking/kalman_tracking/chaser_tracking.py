
from kalman_filter import KalmanFilter
from sensor_msgs.msg import Image
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from rclpy.node import Node
import numpy as np
import rclpy
import cv2
from cv_bridge import CvBridge
import time

class KalmanTrackingNode(Node):
    def __init__(self):
        super().__init__('kalman_tracking_node')
        self.dt = 0.5
        self.br = CvBridge()
        initial_state = self._estimate_initial_state()
        initial_covariance = np.eye(4)
        self.kalman_filter = KalmanFilter(self.dt, initial_state, initial_covariance)
        self._log_results()
        self.last_recieved_message = None
        self.last_measured_position = None
        self.last_measured_velocity = np.array([0, 0])
        self.publisher = self.create_publisher(Twist, '/chaser0/cmd_vel', 10)
        self.lost_target = False
        self.lost_target_count = 0
        self._track()

    def _track(self):
        while True:
            
            # Position of the evader is estimated by the depth camera on the chaser
            self._get_last_message('/chaser0/camera/depth/image_raw', Image)
            depth_image = self.last_recieved_message
            self._get_last_message('/chaser0/camera/image_raw', Image)
            camera_image = self.last_recieved_message
            center_x, center_y = self._find_blue_center(camera_image)

            if center_x is None or center_y is None:
                self.get_logger().warning(f'Target not found - Searching for target.')
                self._search()
            else:
                self.lost_target = False
                depth = self._get_depth_at_point(depth_image, center_x, center_y)
                # Get the current state of the chaser
                self._get_last_message('/chaser0/odom', Odometry)
                chaser_odom = self.last_recieved_message.pose.pose

                # Angle from the chaser to the negative x axis
                angle = np.arctan2(
                    2.0 * (chaser_odom.orientation.w * chaser_odom.orientation.z +
                        chaser_odom.orientation.x * chaser_odom.orientation.y),
                    1.0 - 2.0 * (chaser_odom.orientation.y**2 + chaser_odom.orientation.z**2)
                )                

                # Assuming the chaser is facing the evader, calculate the estimated position of the evader
                estimated_evader_x = chaser_odom.position.x + depth * np.cos(angle)
                estimated_evader_y = chaser_odom.position.y + depth * np.sin(angle)
                
                measurement = np.array([estimated_evader_x, estimated_evader_y])
                # Calculate control signal acceleration
                control_input = np.array([0, 0])
                if self.last_measured_position is not None:
                    change_in_position = measurement - self.last_measured_position
                    change_in_velocity = change_in_position / self.dt
                    if self.last_measured_velocity is not None:
                        change_in_acceleration = (change_in_velocity - self.last_measured_velocity) / self.dt
                        control_input = np.array([change_in_acceleration[0], change_in_acceleration[1]])
                        self.last_measured_velocity = change_in_velocity
                self.last_measured_position = measurement
                
                # Predict the next state of the evader
                self.kalman_filter.predict(control_input)
                self._log_results(type='gui', reading_type='prediction')

                # Update the state of the evader
                self.kalman_filter.update(measurement)
                self._log_results(type='gui', reading_type='correction')
                
                self._log_results()

                # Use predicted state to move the chaser towards the evader
                self._move_chaser(center_x)
            # Sleep for 0.5 seconds
            time.sleep(self.dt)

    def _estimate_initial_state(self):
        # Get last image from camera and corresponding depth data
        self._get_last_message('/chaser0/camera/image_raw', Image)
        camera_image = self.last_recieved_message
        self._get_last_message('/chaser0/camera/depth/image_raw', Image)
        depth_image = self.last_recieved_message
        # Find the position of the evader in the image by looking for blue pixels. Location will be center of the blue pixels
        center_x, center_y = self._find_blue_center(camera_image)

        if center_x is None or center_y is None:
            self.get_logger().warning(f'No blue pixels found in the image. Cannot estimate initial state.')
            return None
        
        # Get the depth data at the position of the evader
        depth = self._get_depth_at_point(depth_image, center_x, center_y)

        # Using chaser odom and depth data, estimate the initial location of the evader
        self._get_last_message('/chaser0/odom', Odometry)
        chaser_odom = self.last_recieved_message.pose.pose

        # Initial state location is the location of the chaser plus the depth (assuming the chaser is facing the evader)        
        initial_state_x = chaser_odom.position.x - depth
        initial_state_y = chaser_odom.position.y

        # Calculate the unit vector of the velocity vector
        velocity_vector_to_goal = np.zeros(2)
        
        # Return the initial state
        return np.array([initial_state_x, initial_state_y, velocity_vector_to_goal[0], velocity_vector_to_goal[1]])

    def _move_chaser(self, center_x):

        # Calculate the desired position of the blue area in the image
        desired_center_x = 640 / 2

        # Calculate the difference between the current and desired positions
        delta_x = desired_center_x - center_x

        # Normalize the difference to be between -0.3 and 0.3
        if delta_x > 0:
            delta_x = min(delta_x, 320)
            delta_x = delta_x / 320 * 0.4
        else:
            delta_x = max(delta_x, -320)
            delta_x = delta_x / 320 * 0.4
        
        # Move the chaser robot based on the difference in positions
        request = Twist()
        request.linear.x = 0.5
        request.angular.z = delta_x  # Adjust the angular velocity based on the difference in x position
        self.publisher.publish(request)

    def _search(self):
        if not self.lost_target:
            # Write to file that the target has been lost
            with open('Results.txt', 'a') as f:
                f.write(f'TARGET LOST\n')
        self.lost_target = True
        # Rotate clockwise
        request = Twist()
        request.linear.x = 0.0
        request.angular.z = -0.4
        self.publisher.publish(request)

    def _get_last_message(self, topic, data_type):
        self.subscription = self.create_subscription(data_type, topic, self._singular_msg, 10)
        rclpy.spin_once(self)
        timeout_sec = 5.0
        start_time = self.get_clock().now()

        while not self.last_recieved_message:
            rclpy.spin_once(self, timeout_sec=1.0)

            if (self.get_clock().now().to_msg().sec - start_time.to_msg().sec) > timeout_sec:
                self.get_logger().warning(f'Timeout reached. No message received on topic {topic}.')
                break

        self.destroy_subscription(self.subscription)
            
    def _singular_msg(self, msg):
        #if hasattr(msg, 'data'):
        #    self.last_recieved_message = msg.data
        #else:
            self.last_recieved_message = msg

    def _find_blue_center(self, msg):
        # Read the image
        image = self.br.imgmsg_to_cv2(msg)

        # Convert the image to HSV color space
        hsv_image = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)

        # Define the lower and upper bounds for the blue color in HSV
        lower_blue = np.array([100, 50, 50])
        upper_blue = np.array([130, 255, 255])

        # Threshold the image to get only blue pixels
        blue_mask = cv2.inRange(hsv_image, lower_blue, upper_blue)

        # Find contours in the binary image
        contours, _ = cv2.findContours(blue_mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

        if len(contours) == 0:
            return None, None

        # Find the largest contour (assuming only one continuous region exists)
        largest_contour = max(contours, key=cv2.contourArea)

        # Calculate the center of the contour
        moments = cv2.moments(largest_contour)
        center_x = int(moments["m10"] / moments["m00"])
        center_y = int(moments["m01"] / moments["m00"])

        return center_x, center_y
    
    def _get_depth_at_point(self, depth_image, x, y):
        cv_image = self.br.imgmsg_to_cv2(depth_image, "32FC1")
        depth_val = cv_image[y, x]

        return depth_val

    def _log_results(self, type='results', reading_type = None):
        # Get the actual position of the evader
        self._get_last_message('/evader0/odom', Odometry)
        evader_odom = self.last_recieved_message.pose.pose
        actual_evader_x = evader_odom.position.x
        actual_evader_y = evader_odom.position.y

        if type == 'results':
            # Get the actual velocity of the evader
            evader_odom = self.last_recieved_message.twist.twist
            actual_evader_vx = evader_odom.linear.x
            actual_evader_vy = evader_odom.linear.y

            # Record reward in text file
            with open('Results.txt', 'a') as f:
                f.write(f'{self.kalman_filter.state[0]} {self.kalman_filter.state[1]} {actual_evader_x} {actual_evader_y} {self.kalman_filter.state[2]} {actual_evader_vx} {self.kalman_filter.state[3]} {actual_evader_vy}\n')
        elif type == 'gui':
            with open('gui.txt', 'a') as f:
                f.write(f'{reading_type} {self.kalman_filter.state[0]} {self.kalman_filter.state[1]} {actual_evader_x} {actual_evader_y}\n')
        


def main(args=None):
    rclpy.init(args=args)

    kalman_tracking_node = KalmanTrackingNode()

    rclpy.spin(kalman_tracking_node)

    kalman_tracking_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()