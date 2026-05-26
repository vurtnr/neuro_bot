from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, TimerAction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument(
            'image_topic',
            default_value='/camera/image_raw',
        ),
        DeclareLaunchArgument(
            'repeat_suppression_frames',
            default_value='10',
        ),
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

        # 🟢 [新增] Web 视频服务器 (调试神器)
        # 启动后访问 http://<机器人IP>:8080 查看实时画面
        Node(
            package='web_video_server',
            executable='web_video_server',
            name='web_video_server',
            output='screen'
        ),
        
        # 🟢 [新增] 电子皮肤驱动 (触觉感知硬件接口)
        # 直接对接电阻阵列采集板，发布 /skin/events /skin/surfaces
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
                    output='screen',
                    parameters=[{
                        'image_topic': LaunchConfiguration('image_topic'),
                        'repeat_suppression_frames': ParameterValue(
                            LaunchConfiguration('repeat_suppression_frames'),
                            value_type=int,
                        ),
                        'require_active_inspection': True,
                    }],
                ),
                # 语音引擎 (听/说)
                Node(
                    package='audio_engine',
                    executable='audio_node',
                    name='audio_engine',
                    output='screen'
                ),
                # LLM 引擎 (知识库/对话)
                Node(
                    package='llm_engine',
                    executable='llm_node',
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
                ),
                Node(
                    package='inspection_bridge',
                    executable='web_server',
                    name='inspection_bridge',
                    output='screen'
                )
            ]
        )
    ])
