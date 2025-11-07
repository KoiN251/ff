import os
from glob import glob
from setuptools import setup, find_packages

package_name = 'uav_mqtt_bridge'

setup(
    name=package_name,
    version='0.0.1',
    packages=find_packages(
        include=[package_name, f"{package_name}.*"]
    ),
    data_files=[
        ('share/ament_index/resource_index/packages',
         [os.path.join('resource', package_name)]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', glob('launch/*.py')),
        ('share/' + package_name + '/config', glob('config/*.yaml')),
        ('share/' + package_name + '/config', glob('uav_mqtt_bridge/config/*.json')),
    ],
    install_requires=['setuptools', 'paho-mqtt'],
    zip_safe=True,
    maintainer='nhut',
    maintainer_email='',
    description='ROS2 PX4 MQTT bridge',
    license='MIT',
    entry_points={
        'console_scripts': [
            'gimbal = uav_mqtt_bridge.gimbal:main',
        ],
    },
)
