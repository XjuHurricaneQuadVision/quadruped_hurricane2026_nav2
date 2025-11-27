import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument, RegisterEventHandler, LogInfo
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
    bringup_pkg_dir = get_package_share_directory('quad_nav2_bringup')
    description_pkg_dir = get_package_share_directory('go2_description')

    nav2_bringup_dir = get_package_share_directory('nav2_bringup')
    gazebo_ros_dir = get_package_share_directory('gazebo_ros')

    # 路径设定
    xacro_file = os.path.join(description_pkg_dir, 'xacro', 'robot.xacro')

    # 配置文件路径
    params_file_path = os.path.join(bringup_pkg_dir, 'config', 'nav2_params.yaml')
    map_file_path = os.path.join(bringup_pkg_dir, 'maps', 'empty_map.yaml') 
    rviz_config_path = os.path.join(bringup_pkg_dir, 'config', 'nav2.rviz')
    
    # Gazebo 世界文件
    world_file_path = LaunchConfiguration('world')

    # ========================================================================
    # 2. 声明启动参数
    # ========================================================================
    declare_use_sim_time = DeclareLaunchArgument(
        'use_sim_time', default_value='true',
        description='Use simulation (Gazebo) clock if true')

    declare_world = DeclareLaunchArgument(
        'world', default_value='', 
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
    if not os.path.exists(xacro_file):
        return LaunchDescription([LogInfo(msg=f"Error: Xacro file not found at {xacro_file}")])
    doc = xacro.process_file(xacro_file)
    robot_description_config = doc.toxml()
    
    # ========================================================================
    # 4. 定义节点
    # ========================================================================
    start_gazebo_server = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(gazebo_ros_dir, 'launch', 'gzserver.launch.py')
        ),
        launch_arguments={'world': world_file_path}.items()
    )

    start_gazebo_client = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(gazebo_ros_dir, 'launch', 'gzclient.launch.py')
        )
    )

    node_robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[{
            'robot_description': robot_description_config,
            'use_sim_time': True
        }]
    )

    spawn_robot = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        arguments=['-entity', 'go2', 
                   '-topic', 'robot_description',
                   '-x', '0.0', '-y', '0.0', '-z', '0.35'], # 机器人生成在 Gazebo 原点上方 0.35m 处
        output='screen'
    )

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

    start_rviz = Node(
        package='rviz2',
        executable='rviz2',
        arguments=['-d', rviz_config_path],
        parameters=[{'use_sim_time': True}],
        output='screen'
    )

    return LaunchDescription([
        declare_use_sim_time,
        declare_world,
        declare_map,
        declare_params,

        start_gazebo_server,
        start_gazebo_client,
        node_robot_state_publisher,
        spawn_robot,
        
        # 确保机器人生成后再启动导航
        RegisterEventHandler(
            event_handler=OnProcessExit(
                target_action=spawn_robot,
                on_exit=[bringup_nav2, start_rviz]
            )
        )
    ])
