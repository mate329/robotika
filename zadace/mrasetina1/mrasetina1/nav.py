# Copyright 2016 Open Source Robotics Foundation, Inc.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.
# https://docs.ros.org/en/humble/Tutorials/Beginner-Client-Libraries/Writing-A-Simple-Py-Publisher-And-Subscriber.html

import rclpy
from rclpy.node import Node

import numpy as np
from math import atan2

from geometry_msgs.msg import PoseStamped, Twist
from nav_msgs.msg import Odometry

slow_down = 0.0

normal_distance = 0.1
normal_angle = 0.2
normal_angle_speed = 0.5
normal_linear_speed = 0.5

slow_down_distance = 0.02
slow_down_angle = 0.1
slow_down_angle_speed = 0.1
slow_down_linear_speed = 0.1

goal_x = 0.0
goal_y = 0.0

def euler_from_quaternion(x, y, z, w):
    t0 = +2.0 * (w * x + y * z)
    t1 = +1.0 - 2.0 * (x * x + y * y)
    roll_x = np.arctan2(t0, t1)

    t2 = +2.0 * (w * y - z * x)
    t2 = +1.0 if t2 > +1.0 else t2
    t2 = -1.0 if t2 < -1.0 else t2
    pitch_y = np.arcsin(t2)

    t3 = +2.0 * (w * z + x * y)
    t4 = +1.0 - 2.0 * (y * y + z * z)
    yaw_z = np.arctan2(t3, t4)

    return roll_x, pitch_y, yaw_z   

class Navigator(Node):
    def __init__(self):
        super().__init__('navigator')

        self.subscriberGoal = self.create_subscription(
            PoseStamped, 
            '/goal_pose', 
            self.listener_callback_goal_position, 
            10)

        self.subscriberOdom = self.create_subscription(
            Odometry, 
            '/odom', 
            self.listener_callback_odometry, 
            10)
        self.publisher = self.create_publisher(Twist, '/cmd_vel', 10)
    
        self.position_x = 0.0
        self.position_y = 0.0
        self.yaw = 0.0

    def listener_callback_goal_position(self, msg):
        self.get_logger().info(f'Need to go to {msg}')
        global goal_x, goal_y
        goal_x = msg.pose.position.x
        goal_y = msg.pose.position.y

    def listener_callback_odometry(self, msg):
        self.position_x = msg.pose.pose.position.x 
        self.position_y = msg.pose.pose.position.y

        self.rotations = msg.pose.pose.orientation
        (_, _, self.yaw) = euler_from_quaternion(self.rotations.x, self.rotations.y, self.rotations.z, self.rotations.w)

        self.goto_point() 

    def goto_point(self):
        distance_x = abs(self.position_x - goal_x)
        distance_y = abs(self.position_y - goal_y)

        goal_angle = atan2(goal_y - self.position_y, goal_x - self.position_x)
        angle = goal_angle - self.yaw

        speed = Twist()

        if (distance_x > slow_down_distance) or (distance_y > slow_down_distance):
            if angle > slow_down_angle:
                speed.linear.x = slow_down
                speed.angular.z = slow_down_angle_speed    
            elif angle < -slow_down_angle:
                speed.linear.x = slow_down
                speed.angular.z = -slow_down_angle_speed  
            else:
                speed.linear.x = slow_down_linear_speed
                speed.angular.z = slow_down
        elif (distance_x > normal_distance) and (distance_y > normal_distance):
            if angle > normal_angle:
                speed.linear.x = slow_down
                speed.angular.z = normal_angle_speed 
            elif angle < -normal_angle:
                speed.linear.x = slow_down
                speed.angular.z = -normal_angle_speed  
            else:
                speed.linear.x = normal_linear_speed
                speed.angular.z = slow_down
        else:
            speed.linear.x = slow_down
            speed.angular.z = slow_down

        self.publisher.publish(speed)

def main(args=None):
    rclpy.init(args=args)
    navigator = Navigator()
    rclpy.spin(navigator)
    rclpy.shutdown()

if __name__ == "__main__":
    main()