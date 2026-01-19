import os
from launch import LaunchDescription
from launch.actions import TimerAction
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        # =========================================
        # 1. 硬件驱动与执行层 (立即启动)
        # =========================================
        
        # 摄像头驱动 (视觉的输入源)
        Node(
            package='camera_ros',
            executable='camera_node',
            name='camera_driver',
            output='screen',
            parameters=[{'width': 640}, {'height': 480}]
        ),
        
        # IoT 控制器 (四肢: 蓝牙/电机)
        Node(
            package='iot_controller',
            executable='iot_controller',
            name='iot_controller',
            output='screen'
        ),

        # Face Bridge (脸面: 屏幕表情)
        Node(
            package='face_bridge',
            executable='face_bridge',
            name='face_node',
            output='screen'
        ),

        # =========================================
        # 2. 能力服务层 (延迟 3秒)
        # =========================================
        TimerAction(
            period=3.0, 
            actions=[
                # 视觉引擎 (识别二维码/人脸)
                Node(
                    package='vision_engine',
                    executable='qr_node',
                    name='vision_qr_node',
                    output='screen'
                ),
                # 语音引擎 (听/说)
                Node(
                    package='audio_engine', 
                    executable='audio_node', 
                    name='audio_engine',
                    output='screen'
                ),
                # LLM 引擎 (知识库/对话) - 新增
                Node(
                    package='llm_engine', 
                    executable='llm_node', # 假设你的入口点叫 llm_service
                    name='llm_engine',
                    output='screen'
                ),
            ]
        ),

        # =========================================
        # 3. 决策控制层 (延迟 6秒，确保服务就绪)
        # =========================================
        TimerAction(
            period=6.0, 
            actions=[
                # 核心大脑 (状态机/统筹)
                Node(
                    package='brain_core',
                    executable='brain_core',
                    name='brain_core',
                    output='screen'
                )
            ]
        )
    ])