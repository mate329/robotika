
import rclpy
from rclpy.node import Node

import os
import numpy as np
import cv2
from cv_bridge import CvBridge
from sensor_msgs.msg import Image
from rcl_interfaces.msg import SetParametersResult
from geometry_msgs.msg import Twist

CV2_THRESHOLD = 127

class TrackBB9(Node):
    def __init__(self):
        super().__init__('tracking_node')

        # Podaci koji su dostupni unutar zadatka
        self.f = 265.23
        self.u0 = 160.5

        self.angular_speed = 5
        self.linear_speed = 0.0
        self.is_robot_in_frame = False

        self.frame_pub = self.create_publisher(Image,
            '/camera_output',
            10)
        self.camera_sub = self.create_subscription(Image,
            '/robot/camera/image_raw', 
            self.frame_callback, 
            10)
        self.cmd_pub = self.create_publisher(Twist, 
            '/robot/cmd_vel', 
            10)

        self.add_on_set_parameters_callback(self.parameter_callback)

    def parameter_callback(self, parameter_list):
        for p in parameter_list: self.get_logger().info(f'\nParamater name {p.name} = {p.value}')
        return SetParametersResult(successful=True)

    def frame_callback(self, msg):
        frame = CvBridge().imgmsg_to_cv2(msg)
        gray_picture = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        	
        (thresh, black_and_white_picture) = cv2.threshold(gray_picture, CV2_THRESHOLD, 255, cv2.THRESH_BINARY_INV)

        cv2_moments = cv2.moments(black_and_white_picture)

        if cv2_moments["m00"] != 0:
            x = int(cv2_moments["m10"] / cv2_moments["m00"])
            y = int(cv2_moments["m01"] / cv2_moments["m00"])
            self.is_robot_in_frame = True
        else:
            x = 0
            y = 0

        if self.is_robot_in_frame: self.angular_speed = np.arctan((self.u0 - x) / self.f)

        cv2.circle(black_and_white_picture, (x, y), 3, (118, 208, 16), -1)

        image_message = CvBridge().cv2_to_imgmsg(black_and_white_picture, encoding="mono8")
        image_message.header = msg.header
        self.frame_pub.publish(image_message)

        cmdMsg = Twist() 
        cmdMsg.angular.z = float(self.angular_speed)

        self.cmd_pub.publish(cmdMsg)



def main(args=None):
    rclpy.init(args=args)
    robot = TrackBB9()

    rclpy.spin(robot)    
    rclpy.shutdown()

if __name__ == "__main__":
    main()