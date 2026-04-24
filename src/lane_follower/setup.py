from setuptools import find_packages, setup

package_name = 'lane_follower'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='RSS Team',
    maintainer_email='todo@mit.edu',
    description='Lane detection node for Final Challenge Part A',
    license='TODO',
    entry_points={
        'console_scripts': [
            'lane_follower_node = lane_follower.lane_follower_node:main',
            'sim_lanes=lane_follower.sim_lanes:main',
            'lane_controller=lane_follower.lane_controller:main',
            'debug_lanes=lane_follower.debug_lanes:main',
        ],
    },
)
