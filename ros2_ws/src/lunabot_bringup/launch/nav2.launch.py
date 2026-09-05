#!/usr/bin/env python3
"""
==============================================================================
LUNABOT NAV2 AUTONOMOUS NAVIGATION LAUNCH FILE
Location: /home/dk05/Desktop/SMART_HORIZON/LUNA_PRO/ros2_ws/src/lunabot_bringup/launch/nav2.launch.py
Features:
  1. Planner Server (Navfn/A* Global Planner)
  2. Controller Server (Regulated Pure Pursuit Controller)
  3. Behavior Server (Recoveries: Spin, BackUp, Wait)
  4. Behavior Tree Navigator (NavigateToPose)
  5. Nav2 Lifecycle Manager (Auto-activates all navigation servers)
==============================================================================
"""

import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, GroupAction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    pkg_lunabot_bringup = get_package_share_directory('lunabot_bringup')
    default_nav2_params = os.path.join(pkg_lunabot_bringup, 'config', 'nav2_params.yaml')

    use_sim_time = LaunchConfiguration('use_sim_time', default='true')
    params_file = LaunchConfiguration('params_file', default=default_nav2_params)

    lifecycle_nodes = [
        'controller_server',
        'planner_server',
        'behavior_server',
        'bt_navigator'
    ]

    remappings = [
        ('/tf', 'tf'),
        ('/tf_static', 'tf_static'),
        ('/cmd_vel', '/cmd_vel_raw')
    ]

    # 1. Controller Server
    controller_node = Node(
        package='nav2_controller',
        executable='controller_server',
        output='screen',
        parameters=[params_file, {'use_sim_time': use_sim_time}],
        remappings=remappings
    )

    # 2. Planner Server
    planner_node = Node(
        package='nav2_planner',
        executable='planner_server',
        name='planner_server',
        output='screen',
        parameters=[params_file, {'use_sim_time': use_sim_time}],
        remappings=remappings
    )

    # 3. Behavior Server (Recoveries)
    behavior_node = Node(
        package='nav2_behaviors',
        executable='behavior_server',
        name='behavior_server',
        output='screen',
        parameters=[params_file, {'use_sim_time': use_sim_time}],
        remappings=remappings
    )

    # 4. BT Navigator
    bt_navigator_node = Node(
        package='nav2_bt_navigator',
        executable='bt_navigator',
        name='bt_navigator',
        output='screen',
        parameters=[params_file, {'use_sim_time': use_sim_time}],
        remappings=remappings
    )

    # 5. Lifecycle Manager
    lifecycle_manager_node = Node(
        package='nav2_lifecycle_manager',
        executable='lifecycle_manager',
        name='lifecycle_manager_navigation',
        output='screen',
        parameters=[
            {'use_sim_time': use_sim_time},
            {'autostart': True},
            {'bond_timeout': 15.0},
            {'node_names': lifecycle_nodes}
        ]
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='true',
            description='Use simulation (Gazebo) clock if true'
        ),
        DeclareLaunchArgument(
            'params_file',
            default_value=default_nav2_params,
            description='Full path to Nav2 parameters yaml file'
        ),
        controller_node,
        planner_node,
        behavior_node,
        bt_navigator_node,
        lifecycle_manager_node
    ])
