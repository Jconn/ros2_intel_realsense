# Copyright (c) 2018 Intel Corporation
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

"""Launch realsense_ros2_camera node without rviz2."""

from launch import LaunchDescription
import launch_ros.actions
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
def generate_launch_description():
    depth_fps = LaunchConfiguration('depth_fps', default='15')
    color_fps = LaunchConfiguration('color_fps', default='6')
    json_file_path = LaunchConfiguration('json_file_path', default='none')

    return LaunchDescription([
            # Realsense
            launch_ros.actions.Node
            (
                package='realsense_ros2_camera',
                node_name='realsense_ros2_camera', 
                node_executable='realsense_ros2_camera',
                output='screen',
                parameters=[{
                'enable_pointcloud': True, 
                'enable_aligned_pointcloud': True,
                'enable_depth': True,
                'enable_aligned_depth': True,
                'depth_fps': depth_fps,
                'color_fps': color_fps,
                'depth_width': 848,#640,#848,
                'depth_height': 480, 
                'json_file_path': json_file_path
                }]
            )   
    ])
