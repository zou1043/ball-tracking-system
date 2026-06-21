#!/usr/bin/env python3
# -*- coding: utf-8 -*-

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    package_share = FindPackageShare('ball_detection')

    team_color = LaunchConfiguration('team_color')
    camera_index = LaunchConfiguration('camera_index')
    serial_port = LaunchConfiguration('serial_port')
    model_path = LaunchConfiguration('model_path')
    log_level = LaunchConfiguration('log_level')

    detect_node = Node(
        package='ball_detection',
        executable='detect_node',
        name='detect_node',
        output='screen',
        emulate_tty=True,
        arguments=['--ros-args', '--log-level', log_level],
        parameters=[{
            'team_color': team_color,
            'camera_index': ParameterValue(camera_index, value_type=int),
            'model_path': model_path,
        }],
    )

    crtl_node = Node(
        package='ball_detection',
        executable='crtl_node',
        name='crtl_node',
        output='screen',
        emulate_tty=True,
        arguments=['--ros-args', '--log-level', log_level],
        parameters=[{
            'team_color': team_color,
        }],
    )

    stm_node = Node(
        package='ball_detection',
        executable='stm_node',
        name='stm_node',
        output='screen',
        emulate_tty=True,
        arguments=['--ros-args', '--log-level', log_level],
        parameters=[{
            'team_color': team_color,
            'port': serial_port,
        }],
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            'team_color',
            default_value='red',
            description='Team color: red or blue',
        ),
        DeclareLaunchArgument(
            'camera_index',
            default_value='0',
            description='OpenCV camera index',
        ),
        DeclareLaunchArgument(
            'serial_port',
            default_value='/dev/ttyUSB0',
            description='STM serial port',
        ),
        DeclareLaunchArgument(
            'model_path',
            default_value=PathJoinSubstitution([package_share, '88.rknn']),
            description='Absolute path of RKNN model file',
        ),
        DeclareLaunchArgument(
            'log_level',
            default_value='info',
            description='ROS log level',
        ),
        detect_node,
        crtl_node,
        stm_node,
    ])
