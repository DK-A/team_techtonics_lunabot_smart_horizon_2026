#!/usr/bin/env python3
import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    pkg_bringup = get_package_share_directory('lunabot_bringup')
    
    slam_params_file = os.path.join(pkg_bringup, 'config', 'slam_toolbox_params.yaml')
    ekf_params_file = os.path.join(pkg_bringup, 'config', 'ekf.yaml')

    use_sim_time = LaunchConfiguration('use_sim_time', default='true')

    declare_use_sim_time = DeclareLaunchArgument(
        'use_sim_time',
        default_value='true',
        description='Use simulation (Gazebo) clock if true'
    )

    # 1. Robot Localization EKF Node
    ekf_node = Node(
        package='robot_localization',
        executable='ekf_node',
        name='ekf_filter_node',
        output='screen',
        parameters=[ekf_params_file, {'use_sim_time': use_sim_time}],
        remappings=[('/odometry/filtered', '/odom/filtered')]
    )

    # 2. SLAM Toolbox Async Node
    slam_node = Node(
        package='slam_toolbox',
        executable='async_slam_toolbox_node',
        name='slam_toolbox',
        output='screen',
        parameters=[slam_params_file, {'use_sim_time': use_sim_time}]
    )

    return LaunchDescription([
        declare_use_sim_time,
        ekf_node,
        slam_node
    ])
