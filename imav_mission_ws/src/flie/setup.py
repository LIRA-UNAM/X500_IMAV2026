from setuptools import find_packages, setup
import os 
from glob import glob

package_name = 'flie'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        (
            'share/ament_index/resource_index/packages',
            ['resource/' + package_name],
        ),
        (
            'share/' + package_name,
            ['package.xml'],
        ),
        (
            os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py'),
        ),
        (os.path.join('share', package_name, 'rviz'), glob(os.path.join('rviz', '*.rviz'))),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='yehousa',
    maintainer_email='yesbalonori@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'cf_takeoff = flie.cf_takeoff:main',
            'cf_vel_translate = flie.cf_vel_translate:main',
            'cf_brain_node = flie.cf_brain_node:main',
            'tf_broadcaster_node = flie.tf_broadcaster_node:main',
            'waypoint_navigator = flie.waypoint_navigator:main',
            'stateMachine = flie.stateMachine:main',
            'newState = flie.newState:main',
            'cf_takeoff_2 = flie.cf_takeoff_2:main',
        ],
    },
)
