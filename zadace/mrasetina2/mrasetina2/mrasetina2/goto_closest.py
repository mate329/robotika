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
import math

import rclpy
from rclpy.node import Node

from std_srvs.srv import Empty

import tf2_ros

import np
from nav_msgs.msg import Odometry
from geometry_msgs.msg import PoseStamped, Twist, Vector3, Point, Quaternion
from sensor_msgs.msg import LaserScan
from visualization_msgs.msg import Marker
from std_msgs.msg import ColorRGBA

angle_thres = pi / 16
regulator = 0.5
speed = 0.5
angle_max = 6 * pi
distance_thres = 0.3


def euler(q):
    t0 = +2.0 * (q.w * q.x + q.y * q.z)
    t1 = +1.0 - 2.0 * (q.x * q.x + q.y * q.y)
    roll_x = np.arctan2(t0, t1)

    t2 = +2.0 * (q.w * q.y - q.z * q.x)
    t2 = +1.0 if t2 > +1.0 else t2
    t2 = -1.0 if t2 < -1.0 else t2
    pitch_y = np.arcsin(t2)

    t3 = +2.0 * (q.w * q.z + q.x * q.y)
    t4 = +1.0 - 2.0 * (q.y * q.y + q.z * q.z)
    yaw_z = np.arctan2(t3, t4)

    return Vector3(x=roll_x, y=pitch_y, z=yaw_z)


def point_diff(point1, point2):
    return Vector3(x=(point2.x - point1.x),
                   y=(point2.y - point1.y),
                   z=(point2.z - point1.z))


def point_transform(point, trans, rot):
    return Point(x=(point.x * np.cos(rot) - point.y * np.sin(rot) + trans.x),
                 y=(point.x * np.sin(rot) + point.y * np.cos(rot) + trans.y),
                 z=(point.z + trans.z))


def magnitude_2D(vector):
    return np.sqrt(vector.x ** 2 + vector.y ** 2)


def orientation(vector):
    return np.arctan2(vector.y, vector.x)


class GotoClosest(Node):

    def __init__(self):
        super().__init__('goto_closest')
        self.pose = None
        self.goal = None
        self.move = False

        self.tfBuffer = tf2_ros.Buffer()
        self.listener = tf2_ros.TransformListener(self.tfBuffer, self)

        self.srv = self.create_service(Empty, 'goto_closest', self.goto_closest_callback)

        self.odom_subscription = self.create_subscription(
            Odometry,
            'odom',
            self.odom_callback,
            10)

        self.laser_subscription = self.create_subscription(
            LaserScan,
            'scan',
            self.laser_callback,
            10)

        self.marker_publisher = self.create_publisher(Marker, 'visualization_marker', 10)
        self.cmd_vel_publisher = self.create_publisher(Twist, 'cmd_vel', 10)

    def goto_closest_callback(self, request, response):
        self.move = True
        return response

    def laser_callback(self, msg):
        if not msg.ranges:
            return

        closest = min(msg.ranges)

        if closest >= float('inf'):
            return

        closest_angle = msg.ranges.index(closest) * msg.angle_increment

        try:
            trans = self.tfBuffer.lookup_transform('odom',
                                                   'base_scan',
                                                   rclpy.time.Time())
        except (tf2_ros.LookupException,
                tf2_ros.ConnectivityException,
                tf2_ros.ExtrapolationException):
            return

        yaw = euler(trans.transform.rotation).z

        goal_x = closest * np.cos(closest_angle)
        goal_y = closest * np.sin(closest_angle)
        goal = Point(x=goal_x, y=goal_y, z=0.0)

        self.goal = point_transform(goal, trans.transform.translation, yaw)
        self.goal.z = 0.0
        self.marker_maker()

    def marker_maker(self):
        marker = Marker()
        marker.header.frame_id = 'odom'
        marker.type = 3
        marker.id = 0
        marker.action = Marker.ADD

        marker.pose.position = self.goal
        marker.pose.orientation = Quaternion(x=0.0, y=0.0, z=0.0, w=1.0)
        marker.scale = Vector3(x=0.25, y=0.25, z=0.25)
        marker.color = ColorRGBA(a=1.0, r=0.0, g=1.0, b=0.0)

        self.marker_publisher.publish(marker)

    def odom_callback(self, pose):
        self.pose = pose.pose

        if not (self.move and self.goal):
            return

        diff_vec = point_diff(self.pose.pose.position, self.goal)
        distance = magnitude_2D(diff_vec)

        robot_orientation = euler(self.pose.pose.orientation)
        angle_diff = orientation(diff_vec) - robot_orientation.z

        if angle_diff < -pi or angle_diff > pi:
            angle_diff = pi - angle_diff

        yaw_vel = min(angle_max, regulator * round(angle_diff, 10))
        lin_vel = speed

        if distance < lin_vel:
            lin_vel = min(0.5 * round(distance, 10), speed)
            yaw_vel = min(0.5 * regulator * round(angle_diff, 10), angle_max)
            if distance <= distance_thres:
                lin_vel = 0.0
                yaw_vel = 0.0
                self.move = False

        linear = Vector3(x=0.0, y=0.0, z=0.0)
        if abs(angle_diff) < angle_thres:
            linear = Vector3(x=lin_vel, y=0.0, z=0.0)

        angular = Vector3(x=0.0, y=0.0, z=yaw_vel)

        cmd_vel_msg = Twist(linear=linear, angular=angular)

        self.cmd_vel_publisher.publish(cmd_vel_msg)


def main(args=None):
    rclpy.init(args=args)

    goto_closest = GotoClosest()

    rclpy.spin(goto_closest)

    goto_closest.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
