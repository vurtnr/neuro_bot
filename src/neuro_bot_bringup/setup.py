import os
from glob import glob
from setuptools import setup

package_name = 'neuro_bot_bringup'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        
        # 🟢 [核心修复] 必须添加这一行！
        # 意思是：把 launch 文件夹里的所有 .py 文件，拷贝到安装目录的 share/.../launch 下
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
        
        # 如果您还有 config 文件夹，也要加类似的：
        # (os.path.join('share', package_name, 'config'), glob('config/*.yaml')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Your Name',
    maintainer_email='your@email.com',
    description='Bringup package for NeuroBot',
    license='TODO',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            # 这里是您的节点入口，如果只是纯 launch 包，这里可以为空
        ],
    },
)