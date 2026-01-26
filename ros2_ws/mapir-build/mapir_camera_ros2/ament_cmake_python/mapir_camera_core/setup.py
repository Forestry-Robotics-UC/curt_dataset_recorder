from setuptools import find_packages
from setuptools import setup

setup(
    name='mapir_camera_core',
    version='0.0.1',
    packages=find_packages(
        include=('mapir_camera_core', 'mapir_camera_core.*')),
)
