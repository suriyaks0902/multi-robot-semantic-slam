from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'semantic_fleet'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.py')),
        (os.path.join('share', package_name, 'config'), glob('config/*.yaml')),
        (os.path.join('share', package_name, 'worlds'), glob('worlds/*.world')),
        (os.path.join('share', package_name, 'rviz'), glob('rviz/*.rviz')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Suriya',
    maintainer_email='k.s.suriya0902@gmail.com',
    description='Multi-Robot Semantic SLAM Fleet Management System',
    license='Apache-2.0',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'yolo_detector = semantic_fleet.yolo_detector:main',
            'object_localizer = semantic_fleet.object_localizer:main',
            'semantic_mapper = semantic_fleet.semantic_mapper:main',
        ],
    },
)
