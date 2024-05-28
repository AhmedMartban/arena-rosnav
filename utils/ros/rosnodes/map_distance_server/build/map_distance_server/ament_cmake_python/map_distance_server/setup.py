from setuptools import find_packages
from setuptools import setup

setup(
    name='map_distance_server',
    version='0.0.0',
    packages=find_packages(
        include=('map_distance_server', 'map_distance_server.*')),
)
