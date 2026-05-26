#!/usr/bin/env python3
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='skin_sensor',
            executable='skin_node',
            name='skin_sensor',
            output='screen',
            parameters=[{
                'port': '/dev/ttyUSB1',
                'baud': 921600,
                'threshold': 50.0,
                'ema_alpha': 0.35,
                'spatial_mix': 0.18,
                'calibrate_on_start': True,
                'calibrate_frames': 20,
                'baseline_path': '/home/neurobot/.config/neurobot/skin_baseline.json',
                'publish_raw': False,
            }],
        ),
    ])