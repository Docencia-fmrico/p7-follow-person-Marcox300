# Copyright 2025 Marcos
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

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    pkg_dir = get_package_share_directory('camera')
    param_file = os.path.join(pkg_dir, 'config', 'params.yaml')

    declare_target_param = DeclareLaunchArgument('target', default_value='person', description='El objetivo para el parámetro target')

    yolo_2d = Node(
            package='camera',
            executable='yolo_detection',
            name='darkent_detection_node',
            output='screen',
            parameters=[param_file],
            remappings=[
                ('input_detection', '/yolo/detections'),
                ('output_detection_2d', 'detection_2d'),
            ]
        )

    convert_2d_3d = Node(
            package='camera',
            executable='detection_2d_to_3d_pc2',
            name='detection_to_3d_from_pc2_node',
            output='screen',
            remappings=[
                ('input_detection_2d', 'detection_2d'),
                ('input_pointcloud', '/depth/points'),  # camera_points_topic
                ('output_detection_3d', 'detection_3d'),
                ('camera_info', '/color/camera_info'),  # camerainfo_topic
            ]
        )

    creator_tf = Node(
            package='follow_p7',
            executable='tf_creator_class',
            name='tf_creator_node',
            output='screen',
            parameters=[
                {'target': LaunchConfiguration('target')}
            ],
    )

    control_follow = Node(
            package='follow_p7',
            executable='lifecycle_follow_class',
            name='life_cycle_follow',
            output='screen',
        )

    ld = LaunchDescription()
    ld.add_action(declare_target_param)
    ld.add_action(yolo_2d)
    ld.add_action(convert_2d_3d)
    ld.add_action(creator_tf)
    ld.add_action(control_follow)

    return ld

# astra
# 'params_file': param_file,
# 'camera_image_topic': '/color/image_raw',
# 'camera_points_topic': '/depth/points',
# 'camerainfo_topic': '/color/camera_info'

# gazebo
# /rgbd_camera/image
# /rgbd_camera/points
# /rgbd_camera/camera_info

# xtion
# /rgb/image_raw
# /depth_registered/points
# /rgb/camera_info
