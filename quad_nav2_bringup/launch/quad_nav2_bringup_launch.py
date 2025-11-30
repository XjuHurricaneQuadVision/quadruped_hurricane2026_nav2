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

import os
import xacro
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction, IncludeLaunchDescription, RegisterEventHandler
from launch.event_handlers import OnProcessExit
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch.launch_description_sources import PythonLaunchDescriptionSource


def process_xacro():
    """Process the robot URDF xacro file with Gazebo configuration."""
    pkg_path = os.path.join(get_package_share_directory('hurricane_description'))
    xacro_file = os.path.join(pkg_path, 'xacro', 'robot.xacro')
    robot_description_config = xacro.process_file(xacro_file, mappings={'GAZEBO': 'true'})
    return robot_description_config.toxml()


def generate_launch_description():
    # Package directories
    bringup_dir = get_package_share_directory('quad_nav2_bringup')
    description_dir = get_package_share_directory('hurricane_description')

    # Declare launch arguments
    use_sim_time = LaunchConfiguration('use_sim_time')
    world = LaunchConfiguration('world')

    declare_use_sim_time_cmd = DeclareLaunchArgument(
        'use_sim_time',
        default_value='true',
        description='Use simulation (Gazebo) clock if true'
    )

    declare_world_cmd = DeclareLaunchArgument(
        'world',
        default_value=os.path.join(bringup_dir, 'worlds', 'big.world'),
        description='Full path to world file to load'
    )

    # Process robot description
    robot_description = process_xacro()

    # Robot state publisher node
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[
            {
                'publish_frequency': 20.0,
                'use_tf_static': True,
                'robot_description': robot_description,
                'ignore_timestamp': True,
                'use_sim_time': use_sim_time
            }
        ],
    )

    # Gazebo Classic launch
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [PathJoinSubstitution([FindPackageShare('gazebo_ros'),
                                   'launch',
                                   'gazebo.launch.py'])]),
        launch_arguments={
            'world': world,
            'verbose': 'true',
            'pause': 'false'
        }.items()
    )

    # Spawn robot in Gazebo Classic
    spawn_entity = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        output='screen',
        arguments=[
            '-topic', 'robot_description',
            '-entity', 'hurricane_robot',
            '-z', '0.4'
        ],
    )

    # Joint state broadcaster spawner
    joint_state_broadcaster = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["joint_state_broadcaster",
                   "--controller-manager", "/controller_manager"],
        parameters=[{'use_sim_time': use_sim_time}]
    )

    # IMU sensor broadcaster spawner
    imu_sensor_broadcaster = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["imu_sensor_broadcaster",
                   "--controller-manager", "/controller_manager"],
        parameters=[{'use_sim_time': use_sim_time}]
    )

    # Leg PD controller spawner
    leg_pd_controller = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["leg_pd_controller",
                   "--controller-manager", "/controller_manager"],
        parameters=[{'use_sim_time': use_sim_time}]
    )

    # RL quadruped controller spawner
    rl_controller = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["rl_quadruped_controller",
                   "--controller-manager", "/controller_manager"],
        parameters=[
            {
                'config_folder': os.path.join(description_dir, 'config', 'legged_gym'),
                'use_sim_time': use_sim_time
            }
        ],
    )

    # Register event handler to spawn controllers after leg_pd_controller is ready
    spawn_controllers_event = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=leg_pd_controller,
            on_exit=[rl_controller, imu_sensor_broadcaster, joint_state_broadcaster],
        )
    )

    return LaunchDescription([
        # Declare launch arguments
        declare_use_sim_time_cmd,
        declare_world_cmd,

        # Launch Gazebo Classic
        gazebo,

        # Launch robot
        robot_state_publisher,
        spawn_entity,

        # Launch controllers
        leg_pd_controller,
        spawn_controllers_event,
    ])
