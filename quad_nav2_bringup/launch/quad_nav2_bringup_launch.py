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
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument, RegisterEventHandler
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, Command, PathJoinSubstitution
from launch.event_handlers import OnProcessExit
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
import xacro

def generate_launch_description():
    # ========================================================================
    # 1. 配置变量与路径
    # ========================================================================
    # TODO: 请修改为你的包名
    package_name = 'robocon_nav' 
    
    pkg_share = get_package_share_directory(package_name)
    nav2_bringup_dir = get_package_share_directory('nav2_bringup')
    gazebo_ros_dir = get_package_share_directory('gazebo_ros')

    # 路径设定
    # 假设你的 xacro 文件叫 robot.xacro，放在 urdf 文件夹下
    xacro_file = os.path.join(pkg_share, 'urdf', 'robot.xacro')
    
    # 假设你的参数文件和地图位置
    params_file_path = os.path.join(pkg_share, 'config', 'nav2_params.yaml')
    map_file_path = os.path.join(pkg_share, 'maps', 'map.yaml')
    rviz_config_path = os.path.join(pkg_share, 'config', 'nav.rviz')
    
    # Gazebo 世界文件 (如果没有，就用空的)
    world_file_path = LaunchConfiguration('world')

    # ========================================================================
    # 2. 声明启动参数 (Arguments)
    # ========================================================================
    declare_use_sim_time = DeclareLaunchArgument(
        'use_sim_time', default_value='true',
        description='Use simulation (Gazebo) clock if true')

    declare_world = DeclareLaunchArgument(
        'world', default_value='', # 默认为空，即加载 Gazebo 空世界
        description='Full path to world file to load')

    declare_map = DeclareLaunchArgument(
        'map', default_value=map_file_path,
        description='Full path to map.yaml file to load')

    declare_params = DeclareLaunchArgument(
        'params_file', default_value=params_file_path,
        description='Full path to the ROS2 parameters file to use')

    # ========================================================================
    # 3. 处理机器人描述 (Xacro -> URDF)
    # ========================================================================
    # 使用 xacro 库解析文件
    doc = xacro.process_file(xacro_file)
    robot_description_config = doc.toxml()
    
    # ========================================================================
    # 4. 定义节点 (Nodes)
    # ========================================================================

    # A. 启动 Gazebo 服务器 (gzserver)
    start_gazebo_server = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(gazebo_ros_dir, 'launch', 'gzserver.launch.py')
        ),
        launch_arguments={'world': world_file_path}.items()
    )

    # B. 启动 Gazebo 客户端 (gzclient - 图形界面)
    start_gazebo_client = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(gazebo_ros_dir, 'launch', 'gzclient.launch.py')
        )
    )

    # C. Robot State Publisher (发布 TF)
    # 仿真中必须开启 use_sim_time
    node_robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[{
            'robot_description': robot_description_config,
            'use_sim_time': True
        }]
    )

    # D. Spawn Entity (把机器人放进 Gazebo)
    # -z 0.5 是为了让机器人稍微悬空，防止一开始就卡在地里
    spawn_robot = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        arguments=['-entity', 'my_quadruped', 
                   '-topic', 'robot_description',
                   '-x', '0.0', '-y', '0.0', '-z', '0.5'],
        output='screen'
    )

    # E. 启动 Nav2 Bringup (核心导航)
    # 使用官方的 bringup_launch，它会自动启动 AMCL, MapServer, Controller, Planner 等
    bringup_nav2 = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(nav2_bringup_dir, 'launch', 'bringup_launch.py')
        ),
        launch_arguments={
            'map': LaunchConfiguration('map'),
            'use_sim_time': 'true',
            'params_file': LaunchConfiguration('params_file'),
            'autostart': 'true'
        }.items()
    )

    # F. 启动 Rviz2
    start_rviz = Node(
        package='rviz2',
        executable='rviz2',
        arguments=['-d', rviz_config_path],
        parameters=[{'use_sim_time': True}],
        output='screen'
    )

    # ========================================================================
    # 5. 组装 LaunchDescription
    # ========================================================================
    return LaunchDescription([
        declare_use_sim_time,
        declare_world,
        declare_map,
        declare_params,

        start_gazebo_server,
        start_gazebo_client,
        node_robot_state_publisher,
        spawn_robot,
        
        # 这里的逻辑是：等机器人生成好了，再启动导航和Rviz，防止报错
        RegisterEventHandler(
            event_handler=OnProcessExit(
                target_action=spawn_robot,
                on_exit=[bringup_nav2, start_rviz]
            )
        )
    ])
