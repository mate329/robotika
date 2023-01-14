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
# Matia Rasetina Mobilna robotika

import rclpy
from rclpy.node import Node

from geometry_msgs.msg import PoseStamped


class GoalPoseSubscriber(Node):

    def __init__(self):
        super().__init__('goal_pose_subscriber')
        self.goal = None
        self.subscription = self.create_subscription(
            PoseStamped,
            'goal_pose',
            self.listener_callback,
            10)
        self.subscription  # prevent unused variable warning

    def listener_callback(self, msg):
        self.get_logger().info(f'Gotta go to {msg}')
        self.goal = msg


def main(args=None):
    rclpy.init(args=args)

    goal_pose_subscriber = GoalPoseSubscriber()

    rclpy.spin(goal_pose_subscriber)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    goal_pose_subscriber.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()