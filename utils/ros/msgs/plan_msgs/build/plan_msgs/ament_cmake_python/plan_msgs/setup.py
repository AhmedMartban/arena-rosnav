from setuptools import find_packages
from setuptools import setup

setup(
    name='plan_msgs',
    version='0.0.0',
    packages=find_packages(
        include=('plan_msgs', 'plan_msgs.*')),
)
