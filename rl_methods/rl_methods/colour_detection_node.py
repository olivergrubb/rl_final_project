import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image

class ColourDetectionNode(Node):

    def __init__(self):
        super().__init__('image_subscriber_node')

        # Create a subscriber for the image_raw topic
        self.subscription = self.create_subscription(
            Image,
            'chaser0/camera/image_raw',
            self.image_callback,
            10  # QoS profile, adjust as needed
        )
        self.subscription  # prevent unused variable warning

    def image_callback(self, msg):
        try:
            rclpy.logging.get_logger('rclpy.node').info('Received image: %s' % str(msg))

        except Exception as e:
            self.get_logger().error('error in colour detection node: %s' % str(e))

def main(args=None):
    rclpy.init(args=args)
    image_subscriber_node = ColourDetectionNode()
    rclpy.spin(image_subscriber_node)
    image_subscriber_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
