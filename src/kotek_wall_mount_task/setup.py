from setuptools import find_packages, setup

package_name = 'kotek_wall_mount_task'

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
        'Orchestrates the wall-mount task: 4 pick+magnetic-mount cycles via '
        'direct GraspObject/PlaceObject action calls, no task_coordinator FSM.'
    ),
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'wall_mount_task = kotek_wall_mount_task.wall_mount_task:main',
        ],
    },
)
