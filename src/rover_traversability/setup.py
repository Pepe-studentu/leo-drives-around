import os
from glob import glob
from setuptools import find_packages, setup

package_name = 'rover_traversability'

setup(
    name=package_name,
    version='0.0.1',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
        (os.path.join('share', package_name, 'config'), glob('config/*.yaml')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Petru',
    maintainer_email='155471233+Pepe-studentu@users.noreply.github.com',
    description='Elevation and traversability mapping for Nav2',
    license='TODO: License declaration',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'traversability_node = rover_traversability.traversability_node:main',
            'calib_checker = rover_traversability.calib_checker:main',
            'nav_e2e_checker = rover_traversability.nav_e2e_checker:main',
            'height_map_viz_node = rover_traversability.height_map_viz_node:main',
            'height_map_evaluator = rover_traversability.height_map_evaluator:main',
        ],
    },
)
