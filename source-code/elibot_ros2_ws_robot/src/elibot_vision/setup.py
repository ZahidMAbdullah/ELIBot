import os
from glob import glob

from setuptools import find_packages, setup

package_name = 'elibot_vision'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='elibot',
    maintainer_email='elibot@todo.todo',
    description='Onboard QR + traffic-light vision status node (runs on the Raspberry Pi) plus the LED indicator node that reacts to it.',
    license='MIT',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'qr_led_indicator = elibot_vision.qr_led_indicator:main',
            'vision_status_node = elibot_vision.vision_status_node:main',
        ],
    },
)
