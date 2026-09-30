from setuptools import find_packages, setup

package_name = 'elibot_camera'

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
    maintainer='elibot',
    maintainer_email='elibot@todo.todo',
    description='Publishes RGB frames from the onboard USB camera to /camera/image_raw for the vision pipeline.',
    license='MIT',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
	"camera_node_publisher = elibot_camera.camera_node_publisher:main",
        ],
    },
)
