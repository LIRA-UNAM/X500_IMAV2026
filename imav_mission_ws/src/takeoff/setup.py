from setuptools import find_packages, setup
import os
from setuptools import setup
from glob import glob

package_name = 'takeoff'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'),
            glob('launch/*.launch.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='quique',
    maintainer_email='enriquemedranorabak@hotmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'takeoff_node = takeoff.takeoff_node:main',
            'simple_move = takeoff.simple_move:main',
            'test_node = takeoff.test_node:main'
        ],
    },
)
