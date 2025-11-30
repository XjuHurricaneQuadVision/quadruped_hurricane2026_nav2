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
from launch.actions import DeclareLaunchArgument, OpaqueFunction, IncludeLaunchDescription, RegisterEventHandler, TimerAction
from launch.conditions import IfCondition
from launch.event_handlers import OnProcessExit
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch.launch_description_sources import PythonLaunchDescriptionSource


def process_xacro():
    """Process the robot URDF xacro file with Gazebo Classic configuration."""
    pkg_path = os.path.join(get_package_share_directory('hurricane_description'))
    xacro_file = os.path.join(pkg_path, 'xacro', 'robot.xacro')
    robot_description_config = xacro.process_file(xacro_file, mappings={'GAZEBO': 'true', 'CLASSIC': 'true'})
    return robot_description_config.toxml()


def generate_launch_description():
    # Package directories
    bringup_dir = get_package_share_directory('quad_nav2_bringup')
    description_dir = get_package_share_directory('hurricane_description')
    nav2_bringup_dir = get_package_share_directory('nav2_bringup')

    # Declare launch arguments
    use_sim_time = LaunchConfiguration('use_sim_time')
    world = LaunchConfiguration('world')
    params_file = LaunchConfiguration('params_file')
    map_yaml_file = LaunchConfiguration('map')
    autostart = LaunchConfiguration('autostart')
    use_composition = LaunchConfiguration('use_composition')
    slam = LaunchConfiguration('slam')
    use_rviz = LaunchConfiguration('use_rviz')

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

    declare_params_file_cmd = DeclareLaunchArgument(
        'params_file',
        default_value=os.path.join(bringup_dir, 'config', 'nav2_params.yaml'),
        description='Full path to the ROS2 parameters file to use for Nav2'
    )

    declare_map_yaml_cmd = DeclareLaunchArgument(
        'map',
        default_value='',
        description='Full path to map yaml file to load'
    )

    declare_autostart_cmd = DeclareLaunchArgument(
        'autostart',
        default_value='true',
        description='Automatically startup the nav2 stack'
    )

    declare_use_composition_cmd = DeclareLaunchArgument(
        'use_composition',
        default_value='True',
        description='Whether to use composed bringup'
    )

    declare_slam_cmd = DeclareLaunchArgument(
        'slam',
        default_value='False',
        description='Whether to run SLAM (True) or localization (False)'
    )

    declare_use_rviz_cmd = DeclareLaunchArgument(
        'use_rviz',
        default_value='True',
        description='Whether to launch RViz for visualization'
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
            '-z', '0.5',
            '-robot_namespace', ''
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

    # Delay spawn robot to allow robot_state_publisher to be ready
    spawn_robot_delayed = TimerAction(
        period=3.0,
        actions=[spawn_entity]
    )

    # Register event handler to spawn controllers after leg_pd_controller is ready
    spawn_controllers_event = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=leg_pd_controller,
            on_exit=[rl_controller, imu_sensor_broadcaster, joint_state_broadcaster],
        )
    )

    # PointCloud to LaserScan converter node
    pointcloud_to_laserscan = Node(
        package='pointcloud_to_laserscan',
        executable='pointcloud_to_laserscan_node',
        name='pointcloud_to_laserscan',
        output='screen',
        remappings=[
            ('cloud_in', '/livox'),
            ('scan', '/scan')
        ],
        parameters=[{
            'target_frame': 'livox_frame',
            'transform_tolerance': 0.01,
            'min_height': -0.5,
            'max_height': 2.0,
            'angle_min': -3.14159,  # -π
            'angle_max': 3.14159,   # π
            'angle_increment': 0.00872665,  # π/360
            'scan_time': 0.1,
            'range_min': 0.1,
            'range_max': 100.0,
            'use_inf': True,
            'inf_epsilon': 1.0,
            'use_sim_time': use_sim_time
        }],
    )

    # Point-LIO SLAM node for odometry estimation
    point_lio_config = os.path.join(
        get_package_share_directory('point_lio'),
        'config',
        'mid360.yaml'
    )

    point_lio = Node(
        package='point_lio',
        executable='pointlio_mapping',
        name='pointlio_mapping',
        output='screen',
        parameters=[
            point_lio_config,
            {'use_sim_time': use_sim_time}
        ],
        remappings=[
            ('/tf', 'tf'),
            ('/tf_static', 'tf_static'),
            ('livox/lidar', '/livox'),
            ('livox/imu', '/imu_plugin/out')
        ]
    )

    # Loam interface node for odometry conversion
    loam_interface = Node(
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

    # Nav2 bringup launch (includes localization + navigation)
    nav2_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(nav2_bringup_dir, 'launch', 'bringup_launch.py')
        ),
        launch_arguments={
            'slam': slam,
            'map': map_yaml_file,
            'use_sim_time': use_sim_time,
            'params_file': params_file,
            'autostart': autostart,
            'use_composition': use_composition,
        }.items()
    )

    # RViz for visualization
    point_lio_dir = get_package_share_directory('point_lio')
    rviz_config_file = os.path.join(point_lio_dir, 'rviz_cfg', 'loam_livox.rviz')

    rviz_node = Node(
        condition=IfCondition(use_rviz),
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', rviz_config_file],
        parameters=[{'use_sim_time': use_sim_time}],
        output='screen'
    )

    return LaunchDescription([
        # Declare launch arguments
        declare_use_sim_time_cmd,
        declare_world_cmd,
        declare_params_file_cmd,
        declare_map_yaml_cmd,
        declare_autostart_cmd,
        declare_use_composition_cmd,
        declare_slam_cmd,
        declare_use_rviz_cmd,

        # Launch Gazebo Classic
        gazebo,

        # Launch robot
        robot_state_publisher,
        spawn_robot_delayed,

        # Launch sensors
        pointcloud_to_laserscan,

        # Launch Point-LIO for odometry estimation
        point_lio,

        # Launch odometry interface
        loam_interface,

        # Launch controllers
        leg_pd_controller,
        spawn_controllers_event,

        # Launch Nav2
        nav2_launch,

        # Launch RViz
        rviz_node,
    ])
