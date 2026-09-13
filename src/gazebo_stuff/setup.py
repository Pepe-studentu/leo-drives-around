import os
from setuptools import find_packages, setup
from glob import glob

package_name = 'gazebo_stuff'

setup(
    name=package_name,
    version='0.0.1',
    packages=find_packages(exclude=['test']),
    data_files=[
            ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
            ('share/' + package_name, ['package.xml']),
            (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
            (os.path.join('share', package_name, 'worlds'), glob('worlds/*.sdf')),
            (os.path.join('share', package_name, 'urdf'), glob('urdf/*.xacro')),
            (os.path.join('share', package_name, 'rviz'), glob('rviz/*.rviz')),
        ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Petru',
    maintainer_email='Daisa.Io.Petru@student.utcluj.ro',
    description='plug and play gazebo simulation for a rover.',
    license='Apache-2.0',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
        ],
    },
)
