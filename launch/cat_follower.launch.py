#!/usr/bin/env python3
#
# Copyright 2019 ROBOTIS CO., LTD.
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
#
# Authors: Joep Tool

import os

import launch_ros.actions
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    pkg_zadatak = get_package_share_directory('project')

    navigator = launch_ros.actions.Node(
        package='project',
        executable='cat_follower',
        name='cat_follower',
        parameters=[
            {'weights_file': os.path.join(pkg_zadatak, 'yolo', 'yolov3.weights')},
            {'cfg_file': os.path.join(pkg_zadatak, 'yolo', 'yolov3.cfg')},
            {'classes_file': os.path.join(pkg_zadatak, 'yolo', 'coco.names')},
        ],
        output='screen'
    )

    camera_turn_on = Node(
        name='ros2',
        package='image_tools',
        executable='cam2images',
        arguments=['--ros-args'],
        parameters=[{'p': "frequency:=10.0"},
                    {'p': 'history:=keep_last'}]
    ),

    ld = LaunchDescription()

    # Add the commands to the launch description
    ld.add_action(navigator)

    return ld
