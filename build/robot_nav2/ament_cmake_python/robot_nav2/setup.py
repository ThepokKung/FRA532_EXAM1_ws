from setuptools import find_packages
from setuptools import setup

setup(
    name='robot_nav2',
    version='0.0.0',
    packages=find_packages(
        include=('robot_nav2', 'robot_nav2.*')),
)
