from setuptools import setup

package_name = 'skin_sensor'

setup(
    name=package_name,
    version='0.1.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', ['launch/skin.launch.py']),
    ],
    install_requires=['setuptools', 'pyserial', 'numpy'],
    zip_safe=True,
    maintainer='NeuroBot Team',
    maintainer_email='dev@neuro.bot',
    description='Electronic skin tactile sensor driver for NeuroBot',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'skin_node = skin_sensor.skin_node:main',
        ],
    },
)