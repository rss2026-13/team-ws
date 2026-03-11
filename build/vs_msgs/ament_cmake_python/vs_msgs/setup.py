from setuptools import find_packages
from setuptools import setup

setup(
    name='vs_msgs',
    version='0.0.0',
    packages=find_packages(
        include=('vs_msgs', 'vs_msgs.*')),
)
