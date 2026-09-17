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
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='nisar-lab-abd',
    maintainer_email='nisar-lab-abd@todo.todo',
    description='Remote-deployable QR + traffic-light vision status node (pairs with elibot_ros2_ws_robot).',
    license='MIT',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'vision_status = elibot_vision.vision_status:main',
        ],
    },
)
