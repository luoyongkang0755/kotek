from setuptools import find_packages, setup

package_name = 'kotek_isaac_bridge'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='kotek',
    maintainer_email='dev@example.com',
    description=(
        'Isaac Sim <-> MoveIt joint bridge and sensor pose publisher for the '
        'kotek Scout Mini + Piper pick demo.'
    ),
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'isaac_joint_bridge = kotek_isaac_bridge.isaac_joint_bridge:main',
            'sensor_pose_publisher = kotek_isaac_bridge.sensor_pose_publisher:main',
        ],
    },
)
