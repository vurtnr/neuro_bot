from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        # 1. 摄像头驱动节点 (源码编译版)
        Node(
            package='camera_ros',
            executable='camera_node',
            name='camera_driver',
            parameters=[{
                # 'camera': 1,         # 你的截图显示 0 号能被正确识别为 imx708
                'width': 640,
                'height': 480,
                'format': 'YUYV',    # <--- 重点：必须是大写！
            }]
        ),
        
        # 2. 网页视频服务器
        Node(
            package='web_video_server',
            executable='web_video_server',
            name='web_video_server',
            parameters=[{
                'port': 8080,
                'address': '0.0.0.0'
            }]
        )
    ])