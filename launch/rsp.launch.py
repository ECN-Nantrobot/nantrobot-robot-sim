"""
@file rsp.launch.py
@brief Robot State Publisher launch file for ROS2.

This launch file sets up the robot_state_publisher node to broadcast the robot's
state and transforms. It processes the URDF/xacro file and publishes the robot
description with support for namespaces and ros2_control.

Usage example:
    ros2 launch nantrobot_robot_sim rsp.launch.py use_sim_time:=true use_namespace:=robot1

Launch arguments:
    - use_sim_time: Use simulation time if true (default: false)
    - use_ros2_control: Enable ros2_control if true (default: false) "true might not work anymore"
    - use_namespace: Namespace to launch the node into (default: '')

@note This file is heavily inspired by Articulated Robotics ROS2 tutorials
      https://articulatedrobotics.xyz/category/getting-ready-to-build-a-ros-robot

@version 1.0
@date 28/12/2025

@author Alexis MORICE
"""
import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.substitutions import LaunchConfiguration, Command
from launch.actions import DeclareLaunchArgument
from launch_ros.actions import Node

import xacro


def generate_launch_description():

    # Check if we're told to use sim time
    use_sim_time = LaunchConfiguration('use_sim_time')
    use_ros2_control = LaunchConfiguration('use_ros2_control')
    use_namespace = LaunchConfiguration('use_namespace')


    # Process the URDF file
    pkg_path = os.path.join(get_package_share_directory('nantrobot_robot_sim'))
    xacro_file = os.path.join(pkg_path,'description','robot.urdf.xacro')
    # robot_description_config = xacro.process_file(xacro_file).toxml()
    robot_description_config = Command(['xacro ', xacro_file, ' use_ros2_control:=', use_ros2_control, ' sim_mode:=', use_sim_time, ' namespace:=', use_namespace, '/'])
    
    # Create a robot_state_publisher node
    params = {
        'robot_description': robot_description_config, 
        'use_sim_time': use_sim_time,
        'frame_prefix': [use_namespace, '/'] # Appends '/' to the namespace for the prefix
    }
    node_robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[params],
        namespace=use_namespace
    )


    # Launch!
    return LaunchDescription([
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='false',
            description='Use sim time if true'),
        DeclareLaunchArgument(
            'use_ros2_control',
            default_value='false',
            description='Use ros2_control if true'),
        DeclareLaunchArgument(
            'use_namespace',
            default_value='',
            description='Namespace to launch the node into'),


        node_robot_state_publisher
    ])
