# Copyright 2025 Jackson Huang
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

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    # Declare launch arguments
    use_sim_time = LaunchConfiguration('use_sim_time')

    declare_use_sim_time_cmd = DeclareLaunchArgument(
        'use_sim_time',
        default_value='true',
        description='Use simulation (Gazebo) clock if true'
    )

    # Loam interface node
    loam_interface_node = Node(
        package='loam_interface',
        executable='loam_interface_node',
        name='loam_interface',
        output='screen',
        parameters=[{
            'use_sim_time': use_sim_time,
            'input_odom_topic': '/aft_mapped_to_init',
            'output_odom_topic': 'odom',
            'odom_frame': 'odom',
            'base_frame': 'base',
            'map_frame': 'map',
            'publish_tf': True
        }]
    )

    return LaunchDescription([
        declare_use_sim_time_cmd,
        loam_interface_node,
    ])
