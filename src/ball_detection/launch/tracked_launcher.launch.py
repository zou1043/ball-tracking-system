#!/usr/bin/env python3
# -*- coding: utf-8 -*-

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution, PythonExpression
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    package_share = FindPackageShare('ball_detection')

    model_path = LaunchConfiguration('model_path')
    camera_index = LaunchConfiguration('camera_index')
    camera_width = LaunchConfiguration('camera_width')
    camera_height = LaunchConfiguration('camera_height')
    camera_fps = LaunchConfiguration('camera_fps')
    camera_auto_exposure = LaunchConfiguration('camera_auto_exposure')
    camera_exposure = LaunchConfiguration('camera_exposure')
    camera_brightness = LaunchConfiguration('camera_brightness')
    camera_contrast = LaunchConfiguration('camera_contrast')
    camera_gain = LaunchConfiguration('camera_gain')
    camera_saturation = LaunchConfiguration('camera_saturation')
    serial_port = LaunchConfiguration('serial_port')
    baudrate = LaunchConfiguration('baudrate')
    track_width = LaunchConfiguration('track_width')
    wheel_diameter = LaunchConfiguration('wheel_diameter')
    encoder_ppr = LaunchConfiguration('encoder_ppr')
    image_width = LaunchConfiguration('image_width')
    image_height = LaunchConfiguration('image_height')
    target_bottom_y = LaunchConfiguration('target_bottom_y')
    target_area = LaunchConfiguration('target_area')
    team_color = LaunchConfiguration('team_color')
    log_level = LaunchConfiguration('log_level')
    start_detect = LaunchConfiguration('start_detect')
    start_track_logic = LaunchConfiguration('start_track_logic')
    start_mission_logic = LaunchConfiguration('start_mission_logic')
    start_serial_bridge = LaunchConfiguration('start_serial_bridge')
    print_serial_feedback = LaunchConfiguration('print_serial_feedback')

    detect_node = Node(
        package='ball_detection',
        executable='detect_node',
        name='detect_node',
        output='screen',
        emulate_tty=True,
        condition=IfCondition(start_detect),
        arguments=['--ros-args', '--log-level', log_level],
        parameters=[{
            'model_path': model_path,
            'camera_index': ParameterValue(camera_index, value_type=int),
            'camera_width': ParameterValue(camera_width, value_type=int),
            'camera_height': ParameterValue(camera_height, value_type=int),
            'camera_fps': ParameterValue(camera_fps, value_type=float),
            'camera_auto_exposure': ParameterValue(camera_auto_exposure, value_type=bool),
            'camera_exposure': ParameterValue(camera_exposure, value_type=float),
            'camera_brightness': ParameterValue(camera_brightness, value_type=float),
            'camera_contrast': ParameterValue(camera_contrast, value_type=float),
            'camera_gain': ParameterValue(camera_gain, value_type=float),
            'camera_saturation': ParameterValue(camera_saturation, value_type=float),
            'team_color': team_color,
        }],
    )

    track_logic_node = Node(
        package='ball_detection',
        executable='ball_track_logic',
        name='ball_track_logic',
        output='screen',
        emulate_tty=True,
        condition=IfCondition(
            PythonExpression([
                "'",
                start_track_logic,
                "' == 'true' and '",
                start_mission_logic,
                "' != 'true'",
            ])
        ),
        arguments=['--ros-args', '--log-level', log_level],
        parameters=[{
            'image_width': ParameterValue(image_width, value_type=float),
            'image_height': ParameterValue(image_height, value_type=float),
            'target_bottom_y': ParameterValue(target_bottom_y, value_type=float),
            'target_area': ParameterValue(target_area, value_type=float),
        }],
    )

    mission_logic_node = Node(
        package='ball_detection',
        executable='ball_mission_logic',
        name='ball_mission_logic',
        output='screen',
        emulate_tty=True,
        condition=IfCondition(start_mission_logic),
        arguments=['--ros-args', '--log-level', log_level],
        parameters=[{
            'image_width': ParameterValue(image_width, value_type=float),
            'team_color': team_color,
        }],
    )

    serial_bridge_node = Node(
        package='ball_detection',
        executable='esp32_serial_node',
        name='esp32_serial_node',
        output='screen',
        emulate_tty=True,
        condition=IfCondition(start_serial_bridge),
        arguments=['--ros-args', '--log-level', log_level],
        parameters=[{
            'port': serial_port,
            'baudrate': ParameterValue(baudrate, value_type=int),
            'track_width': ParameterValue(track_width, value_type=float),
            'wheel_diameter': ParameterValue(wheel_diameter, value_type=float),
            'encoder_pulses_per_wheel_rev': ParameterValue(encoder_ppr, value_type=float),
            'print_received_serial': ParameterValue(print_serial_feedback, value_type=bool),
        }],
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            'model_path',
            default_value=PathJoinSubstitution([package_share, '88.rknn']),
            description='Absolute path of RKNN model file',
        ),
        DeclareLaunchArgument(
            'camera_index',
            default_value='1',
            description='OpenCV camera index',
        ),
        DeclareLaunchArgument(
            'camera_width',
            default_value='640',
            description='OpenCV camera width',
        ),
        DeclareLaunchArgument(
            'camera_height',
            default_value='480',
            description='OpenCV camera height',
        ),
        DeclareLaunchArgument(
            'camera_fps',
            default_value='30.0',
            description='Requested camera FPS',
        ),
        DeclareLaunchArgument(
            'camera_auto_exposure',
            default_value='false',
            description='Whether to keep camera auto exposure enabled',
        ),
        DeclareLaunchArgument(
            'camera_exposure',
            default_value='10.0',
            description='Manual camera exposure value, used when auto exposure is false',
        ),
        DeclareLaunchArgument(
            'camera_brightness',
            default_value='-1.0',
            description='Optional brightness override, negative keeps driver default',
        ),
        DeclareLaunchArgument(
            'camera_contrast',
            default_value='-1.0',
            description='Optional contrast override, negative keeps driver default',
        ),
        DeclareLaunchArgument(
            'camera_gain',
            default_value='0.0',
            description='Optional gain override, negative keeps driver default',
        ),
        DeclareLaunchArgument(
            'camera_saturation',
            default_value='-1.0',
            description='Optional saturation override, negative keeps driver default',
        ),
        DeclareLaunchArgument(
            'serial_port',
            default_value='/dev/ttyUSB0',
            description='ESP32 serial port',
        ),
        DeclareLaunchArgument(
            'baudrate',
            default_value='115200',
            description='ESP32 serial baudrate',
        ),
        DeclareLaunchArgument(
            'track_width',
            default_value='0.115',
            description='Tracked vehicle center distance in meter',
        ),
        DeclareLaunchArgument(
            'wheel_diameter',
            default_value='0.042',
            description='Drive wheel diameter in meter',
        ),
        DeclareLaunchArgument(
            'encoder_ppr',
            default_value='390.0',
            description='Encoder pulses per wheel revolution, must be calibrated on real robot',
        ),
        DeclareLaunchArgument(
            'image_width',
            default_value='640.0',
            description='Camera image width in pixels',
        ),
        DeclareLaunchArgument(
            'image_height',
            default_value='480.0',
            description='Camera image height in pixels',
        ),
        DeclareLaunchArgument(
            'target_bottom_y',
            default_value='340.0',
            description='Fallback distance target when BallInfo has no bbox area',
        ),
        DeclareLaunchArgument(
            'target_area',
            default_value='12000.0',
            description='Preferred target bbox area if detect_node fills Point.z',
        ),
        DeclareLaunchArgument(
            'team_color',
            default_value='red',
            description='Compatibility parameter passed to detect_node',
        ),
        DeclareLaunchArgument(
            'log_level',
            default_value='info',
            description='ROS log level, e.g. debug/info/warn/error',
        ),
        DeclareLaunchArgument(
            'start_detect',
            default_value='true',
            description='Whether to start detect_node',
        ),
        DeclareLaunchArgument(
            'start_track_logic',
            default_value='true',
            description='Whether to start ball_track_logic',
        ),
        DeclareLaunchArgument(
            'start_mission_logic',
            default_value='false',
            description='Whether to start ball_mission_logic',
        ),
        DeclareLaunchArgument(
            'start_serial_bridge',
            default_value='true',
            description='Whether to start esp32_serial_node',
        ),
        DeclareLaunchArgument(
            'print_serial_feedback',
            default_value='false',
            description='Whether to print serial feedback from ESP32',
        ),
        detect_node,
        track_logic_node,
        mission_logic_node,
        serial_bridge_node,
    ])
