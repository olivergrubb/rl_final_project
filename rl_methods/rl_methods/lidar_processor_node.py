import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Float32MultiArray


class ProcessedScanPublisher(Node):

    def __init__(self):
        super().__init__('lidar_processor_node')

        # distinguishing between the different robots:
        self.declare_parameter('robot', 'default_value')
        robot = self.get_parameter('robot').get_parameter_value().string_value
        rclpy.logging.get_logger('rclpy.node').info('robot:%s lidar processor node' % str(robot))
        self.subscriber = self.create_subscription(LaserScan, f'/{robot}/scan', self.laser_callback, 10)

        self.publisher = self.create_publisher(Float32MultiArray, f'/{robot}/cleaned_lidar', 10)


    
    def laser_callback(self, msg):
        # process the laser data:
        cleaned_lidar = []

        # Go through the 720 ranges for the original 360 degree scan and return one reading per 5 degrees
        for range in msg.ranges[0:720:5]:
            if str(range) == 'inf':
                cleaned_lidar.append(msg.range_max)
            else:
                cleaned_lidar.append(range)

        cleaned_lidar = Float32MultiArray(data=cleaned_lidar)
        
        self.publisher.publish(cleaned_lidar)


def main(args=None):
    rclpy.init(args=args)

    lidar_processor_node = ProcessedScanPublisher()

    rclpy.spin(lidar_processor_node)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    lidar_processor_node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()