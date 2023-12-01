#!/usr/bin/env python3

import rclpy
import cv2
import numpy as np
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
from std_msgs.msg import String


class ImageNode(Node):

    def __init__(self):
        super().__init__('image_node')
        #subscribe to the image topic

        #grab the robot name from the host node
        self.declare_parameter('robot', 'default_value')
        robot = self.get_parameter('robot').get_parameter_value().string_value
        rclpy.logging.get_logger('rclpy.node').info('Robot: %s colour detection node initialised' % str(robot))

        publish_topic = f'/{robot}/visible_color'
        subscribe_topic = f'/{robot}/camera/image_raw'

        self.image_subscriber = self.create_subscription(
            Image,
            subscribe_topic,
            self.image_callback,
            10
        )

        self.br = CvBridge()

                
        #publish to the topic /visible_color

        self.publisher = self.create_publisher(
                String,
                publish_topic,
                10 
            )


    def image_callback(self, msg: Image):
        
        #adjust the color detecction parameters here,
        #change the resize
        #change the 0,0,200 to the lower bound of the color
        #change the 100,100,255 to the upper bound of the color
        #change the inequality with the sum to change the threshold

        visible_color = String()

        image = self.br.imgmsg_to_cv2(msg)

        image = cv2.resize(image, (50, 50))

        lower_red = np.array([0, 0, 200], dtype = "uint8")

        upper_red= np.array([100, 100, 255], dtype = "uint8")

        detected_output_red = cv2.inRange(image, lower_red, upper_red)
    
        is_red_present = np.sum(detected_output_red) > 0

        lower_blue = np.array([150, 0, 0], dtype = "uint8")

        upper_blue= np.array([255, 100, 100], dtype = "uint8")

        detected_output_blue = cv2.inRange(image, lower_blue, upper_blue)

        is_blue_present = np.sum(detected_output_blue) > 0

        lower_green = np.array([0, 150, 0], dtype = "uint8")

        upper_green= np.array([100, 255, 100], dtype = "uint8")

        detected_output_green = cv2.inRange(image, lower_green, upper_green)

        is_green_present = np.sum(detected_output_green) > 0

        if is_red_present == True:
                is_red = "T"
        else:
            is_red = "F"

        if is_blue_present == True:
            is_blue = "T"
        else:
            is_blue = "F"

        if is_green_present == True:
            is_green = "T"
        else:
            is_green = "F"

        #outputs a string of the form "RGB" where each letter is either T or F eg (TTT)
        visible_color.data = str(str(is_red) + str(is_green) + str(is_blue))

        self.publisher.publish(visible_color)


def main(args=None):
    rclpy.init(args=args)
    image_node = ImageNode()
    rclpy.spin(image_node)
    rclpy.shutdown()

if __name__ == 'main':
    main()