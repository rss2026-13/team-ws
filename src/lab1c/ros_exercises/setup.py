from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'ros_exercises'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*')) #just added this for yaml
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='racecar',
    maintainer_email='muktharamesh20@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'talker = ros_exercises.simple_publisher:main',
            'listener = ros_exercises.simple_subscriber:main',
            'fake_scan_publisher = ros_exercises.fake_scan_publisher:main',
            'open_space = ros_exercises.open_space_publisher:main',
            'dynamic = ros_exercises.dynamic_tf_cam_publisher:main',
            'static = ros_exercises.static_tf_cam_publisher:main',
            'baselink2 = ros_exercises.base_link_tf_pub:main'
            # 'simple_pubsub_tmux.yaml = tmuxp load /home/racecar/racecar_ws/src/lab1c/ros_exercises/launch/simple_pubsub_tmux.yaml',
        ],
    },
)
