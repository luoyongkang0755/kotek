from setuptools import find_packages, setup

package_name = 'kotek_isaac_stage'

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
    description='Offline USD stage authoring for the kotek demo scene (usd-core, no Isaac Sim needed).',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'build_demo_stage = kotek_isaac_stage.build_demo_stage:main',
            'inspect_stage = kotek_isaac_stage.inspect_stage:main',
        ],
    },
)
