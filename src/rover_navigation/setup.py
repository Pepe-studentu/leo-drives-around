import os                                                                                                                                                                  
from glob import glob                                                                                                                                                      
from setuptools import find_packages, setup                                                                                                                                
                                                                                                                                                                            
package_name = 'rover_navigation'                                                                                                                                          
                                                                                                                                                                            
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
    description='Autonomous rover navigation',
    license='TODO: License declaration',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'mission_runner = rover_navigation.mission_runner:main',
        ],
    },
)
