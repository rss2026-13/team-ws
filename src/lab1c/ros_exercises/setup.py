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
        (os.path.join('share',package_name,'launch'),glob(os.path.join('launch','*launch.[pxy][yma]*'))),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='racecar',
    maintainer_email='tissanyc999@gmail.com',
    description='Publishes a random number between 0 and 10.',
    license='TODO: License declaration',
    extras_require={
        'test': [
            'pytest',
        ],
        },
    entry_points={
        'console_scripts': ['simple_publisher  = ros_exercises.simple_publisher:main',
            'simple_subscriber = ros_exercises.simple_subscriber:main',
            'fake_scan_publisher = ros_exercises.fake_scan_publisher:main',
            'open_space_publisher = ros_exercises.open_space_publisher:main',
            'dynamic_tf_cam_publisher = ros_exercises.dynamic_tf_cam_publisher:main',
            'static_tf_cam_publisher = ros_exercises.static_tf_cam_publisher:main',
            'base_link_tf_pub = ros_exercises.base_link_tf_pub:main',
        ],
        },
)
