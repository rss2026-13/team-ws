from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'boating_school_state_machine'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'),
            glob('launch/*.launch.xml')),
        (os.path.join('share', package_name, 'boating_school_state_machine'),
            glob('boating_school_state_machine/params.yaml')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='racecar',
    maintainer_email='jeryllewiscs@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'boating_school_state_machine = boating_school_state_machine.boating_school_state_machine:main',
        ],
    },
)