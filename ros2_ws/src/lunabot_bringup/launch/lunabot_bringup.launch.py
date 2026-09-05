#!/usr/bin/env python3
"""
==============================================================================
LUNABOT MASTER ROS 2 LAUNCH FILE (PHASE 4 SENSOR INTEGRATION)
Location: /home/dk05/Desktop/SMART_HORIZON/LUNA_PRO/ros2_ws/src/lunabot_bringup/launch/lunabot_bringup.launch.py
Launches:
 1. Gazebo Sim 8 Lunar Habitat Simulation (via run_environment.sh)
 2. ROS 2 <-> Gazebo Sim Bridge (ros_gz_bridge)
 3. Robot State Publisher (TF & URDF frame tree for all sensors)
 4. Zone Manager Node (Static Zone & RViz Markers)
 5. Environmental Sensor Module Node (Dust, Gas, Temp & Radiation telemetry)
==============================================================================
"""

import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, IncludeLaunchDescription, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    pkg_lunabot_bringup = get_package_share_directory('lunabot_bringup')
    
    # Resolve workspace root dynamically
    workspace_root = os.environ.get('LUNA_PRO_ROOT', None)
    if not workspace_root or not os.path.exists(workspace_root):
        candidates = [
            os.path.abspath(os.path.join(os.path.dirname(__file__), "../../../..")),
            "/home/dk05/Desktop/SMART_HORIZON/LUNA_PRO",
            os.getcwd()
        ]
        for c in candidates:
            if os.path.exists(os.path.join(c, "environment")):
                workspace_root = c
                break
    if not workspace_root:
        workspace_root = "/home/dk05/Desktop/SMART_HORIZON/LUNA_PRO"

    urdf_path = os.path.join(workspace_root, 'environment', 'models', 'lunabot', 'lunabot.urdf')
    bridge_config_path = os.path.join(pkg_lunabot_bringup, 'config', 'bridge_config.yaml')
    rviz_config_path = os.path.join(pkg_lunabot_bringup, 'config', 'lunabot.rviz')
    
    use_sim_time = DeclareLaunchArgument(
        'use_sim_time',
        default_value='true',
        description='Use simulation (Gazebo) clock if true'
    )

    launch_gazebo_arg = DeclareLaunchArgument(
        'launch_gazebo',
        default_value='true',
        description='Whether to launch Gazebo simulation instance (set false if already running)'
    )

    rviz_arg = DeclareLaunchArgument(
        'rviz',
        default_value='false',
        description='Whether to launch pre-configured RViz2 visualizer'
    )

    dashboard_arg = DeclareLaunchArgument(
        'dashboard',
        default_value='false',
        description='Whether to launch live visual HUD dashboard'
    )

    web_arg = DeclareLaunchArgument(
        'web',
        default_value='false',
        description='Whether to launch remote web dashboard for other laptops'
    )

    slam_arg = DeclareLaunchArgument(
        'slam',
        default_value='false',
        description='Whether to launch SLAM Toolbox and EKF'
    )

    nav2_arg = DeclareLaunchArgument(
        'nav2',
        default_value='false',
        description='Whether to launch Nav2 autonomous navigation stack'
    )
    
    # 1. Environment & Model Paths
    models_dir = os.path.join(workspace_root, 'environment', 'models')
    worlds_dir = os.path.join(workspace_root, 'environment', 'worlds')
    existing_gz_path = os.environ.get('GZ_SIM_RESOURCE_PATH', '')
    os.environ['GZ_SIM_RESOURCE_PATH'] = f"{models_dir}:{worlds_dir}:{existing_gz_path}".rstrip(':')
    
    # 2. Gazebo Sim Process (using run_environment.sh)
    run_env_script = os.path.join(workspace_root, 'environment', 'run_environment.sh')
    gz_env = dict(os.environ)
    gz_env['GZ_SIM_RESOURCE_PATH'] = f"{models_dir}:{worlds_dir}:{existing_gz_path}".rstrip(':')
    gz_env['IGN_GAZEBO_RESOURCE_PATH'] = gz_env['GZ_SIM_RESOURCE_PATH']
    gz_env['SDF_PATH'] = gz_env['GZ_SIM_RESOURCE_PATH']

    gazebo_process = ExecuteProcess(
        cmd=[run_env_script],
        env=gz_env,
        condition=IfCondition(LaunchConfiguration('launch_gazebo')),
        output='screen'
    )
    
    # 3. ROS-Gazebo Bridge Node
    ros_gz_bridge_node = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        parameters=[{
            'config_file': bridge_config_path,
            'use_sim_time': True,
        }],
        output='screen'
    )
    
    # 4. Robot State Publisher
    with open(urdf_path, 'r') as f:
        robot_desc = f.read()
        
    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[{
            'use_sim_time': True,
            'robot_description': robot_desc
        }]
    )
    
    # 5. Zone Manager Node
    zone_manager_node = Node(
        package='lunabot_bringup',
        executable='zone_manager_node',
        name='zone_manager_node',
        output='screen',
        parameters=[{
            'use_sim_time': True
        }]
    )
    
    # 6. Environmental Sensor Node
    # 6. Environmental Sensor Node
    environmental_sensor_node = Node(
        package='lunabot_bringup',
        executable='environmental_sensor_node',
        name='environmental_sensor_node',
        output='screen',
        parameters=[{
            'use_sim_time': True
        }]
    )

    # 6b. Hazard Safety & Anti-Flip Monitor Node
    hazard_safety_node = Node(
        package='lunabot_bringup',
        executable='hazard_safety_node',
        name='hazard_safety_node',
        output='screen',
        parameters=[{
            'use_sim_time': True
        }]
    )

    # 6c. 3D Stereo Depth & Hazard PointCloud Node (Phase 2)
    stereo_depth_node = Node(
        package='lunabot_bringup',
        executable='stereo_depth_node',
        name='stereo_depth_node',
        output='screen',
        parameters=[{
            'use_sim_time': True
        }]
    )

    # 6d. Terramechanics & Machine Learning Anomaly Detection Node (Phase 3)
    terramechanics_node = Node(
        package='lunabot_bringup',
        executable='terramechanics_ml_node',
        name='terramechanics_ml_node',
        output='screen',
        parameters=[{
            'use_sim_time': True
        }]
    )

    # 7. Static TF for Gazebo GPU LiDAR Frame Compatibility
    static_tf_lidar_node = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='static_tf_lidar',
        arguments=['0', '0', '0', '0', '0', '0', 'lidar_link', 'lunabot/lidar_link/gpu_lidar'],
        parameters=[{'use_sim_time': True}],
        output='screen'
    )
    
    # 8. RViz2 Visualizer Node
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', rviz_config_path],
        condition=IfCondition(LaunchConfiguration('rviz')),
        output='screen'
    )

    # 8. Live Visual Sensor Dashboard HUD Process
    dashboard_script = os.path.join(workspace_root, 'scripts', 'live_sensor_dashboard.py')
    dashboard_process = ExecuteProcess(
        cmd=['python3', dashboard_script],
        condition=IfCondition(LaunchConfiguration('dashboard')),
        output='screen'
    )

    # 9. Remote Web Streaming Dashboard Process
    web_script = os.path.join(workspace_root, 'tools', 'web_dashboard', 'app.py')
    web_process = ExecuteProcess(
        cmd=['python3', web_script, '--ros-args', '-p', 'use_sim_time:=true'],
        condition=IfCondition(LaunchConfiguration('web')),
        output='screen'
    )
    
    # 10. SLAM Toolbox & EKF Localization Process
    slam_launch_file = os.path.join(pkg_lunabot_bringup, 'launch', 'slam.launch.py')
    slam_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(slam_launch_file),
        condition=IfCondition(LaunchConfiguration('slam')),
        launch_arguments={'use_sim_time': 'true'}.items()
    )

    # 11. Nav2 Autonomous Navigation Process (Delayed by 8.0s to allow SLAM /map initialization)
    nav2_launch_file = os.path.join(pkg_lunabot_bringup, 'launch', 'nav2.launch.py')
    nav2_launch = TimerAction(
        period=8.0,
        actions=[
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(nav2_launch_file),
                condition=IfCondition(LaunchConfiguration('nav2')),
                launch_arguments={'use_sim_time': 'true'}.items()
            )
        ]
    )

    return LaunchDescription([
        use_sim_time,
        launch_gazebo_arg,
        rviz_arg,
        dashboard_arg,
        web_arg,
        slam_arg,
        nav2_arg,
        gazebo_process,
        ros_gz_bridge_node,
        robot_state_publisher_node,
        zone_manager_node,
        environmental_sensor_node,
        hazard_safety_node,
        stereo_depth_node,
        terramechanics_node,
        static_tf_lidar_node,
        rviz_node,
        dashboard_process,
        web_process,
        slam_launch,
        nav2_launch,
    ])
