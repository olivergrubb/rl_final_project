# Subscribed to trainer
# When trainer publishes an action, this node publishes the action to the robot cmd_vel topic
# Distinguish between the different robots

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray
from geometry_msgs.msg import Twist

class ActionPublisher(Node):
    
        def __init__(self):
            super().__init__('action_publisher_node')
            self.declare_parameter('robot', 'default_value')
            robot = self.get_parameter('robot').get_parameter_value().string_value
            rclpy.logging.get_logger('rclpy.node').info('Robot: %s action publisher node initialised' % str(robot))

            robot_trainer_topic = f'/{robot}/trainer'
            # Create a subscriber for the trainer
            self.subscription = self.create_subscription(
                Float32MultiArray,
                robot_trainer_topic,
                self.action_callback,
                10  # QoS profile, adjust as needed
            )
            self.subscription  # prevent unused variable warning
    
            # Create a publisher for the cmd_vel topic
            topic = f'/{robot}/cmd_vel'
            self.publisher = self.create_publisher(
                Twist,
                topic,
                10  # QoS profile, adjust as needed
            )
    
        def action_callback(self, msg):
            try:
                msg = handle_action(msg)
                # Publish the action to the robot
                self.publisher.publish(msg)
    
            except Exception as e:
                self.get_logger().error('Error publishing action: %s' % str(e))


def handle_action(msg):
    action = msg.data
    vel_msg = Twist()
    vel_msg.linear.x = action[0]
    vel_msg.angular.z = action[1]

    return vel_msg

def main(args=None):
    rclpy.init(args=args)
    action_publisher_node = ActionPublisher()
    rclpy.spin(action_publisher_node)
    action_publisher_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()