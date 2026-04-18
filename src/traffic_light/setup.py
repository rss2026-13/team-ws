from setuptools import find_packages, setup

package_name = 'traffic_light'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='racecar',
    maintainer_email='tissanyc999@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'traffic_light_detection=traffic_light.traffic_light_detection:main',
            'traffic_light_node=traffic_light.traffic_light_node:main',
            'test_detection=traffic_light.test_detection:main',
            'test_stop_behavior=traffic_light.test_stop_behavior:main',
            
        ],
    },
)
