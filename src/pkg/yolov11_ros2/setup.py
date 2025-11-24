from setuptools import setup
import os
from glob import glob

package_name = 'yolov11_ros2'

setup(
    name=package_name,
    version='0.1.0',
    packages=[package_name],
    data_files=[
        # File di risorsa base
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        
        # File package.xml
        ('share/' + package_name, ['package.xml']),
        
        # Launch files - SOLO nella sottocartella launch
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
        
        # Models
        (os.path.join('share', package_name, 'models'), glob('models/*.pt')), 
    ],
    install_requires=['setuptools'],
    zip_safe=False,
    maintainer='Andreas Hovaldt',
    maintainer_email='andreas.hovaldt@gmail.com',
    description='ROS2 Package for object segmentation using YOLO',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'yolov11_node = yolov11_ros2.yolov11_node:main',
        ],
    },
)