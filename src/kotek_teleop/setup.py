from setuptools import find_packages, setup

package_name = 'kotek_teleop'

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
        'SpaceMouse base teleop + real Piper leader-arm teleop, with simulated '
        'contact force rendered back onto the leader arm.'
    ),
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'spacemouse_teleop_node = kotek_teleop.spacemouse_teleop_node:main',
            'piper_leader_node = kotek_teleop.piper_leader_node:main',
        ],
    },
)
