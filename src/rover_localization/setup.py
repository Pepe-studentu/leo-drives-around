import os
from glob import glob
from setuptools import find_packages, setup

package_name = 'rover_localization'

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
    maintainer='root',
    maintainer_email='Daisa.Io.Petru@student.utcluj.ro',
    description='TODO: Package description',
    license='TODO: License declaration',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'ground_truth_node = rover_localization.ground_truth_node:main',
            'benchmark_loc = rover_localization.benchmark_loc:main',
            'satellite_oracle_node = rover_localization.satellite_oracle_node:main',
            'global_3d_corrector = rover_localization.global_3d_corrector:main',
            'rover_kinematic_ekf = rover_localization.rover_kinematic_ekf:main',
        ],
    },
)
