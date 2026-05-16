from glob import glob
from setuptools import setup

package_name = 'auto_nav_part3'

setup(
    name=package_name,
    version='0.1.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', glob('launch/*.launch.py')),
        ('share/' + package_name + '/simulation/urdf', glob('urdf/*.urdf')),
        ('share/' + package_name + '/simulation/meshes', glob('meshes/*.dae', recursive=True)),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='AUTO4508 Team 18',
    maintainer_email='team18@example.com',
    description='Minimal ROS2 package for Part 3 mapping and discovery.',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'state_manager = auto_nav_part3.state_manager:main',
            'mapping_service = auto_nav_part3.mapping_service:main',
            'waypoint_service = auto_nav_part3.waypoint_service:main',
            'safety_monitor = auto_nav_part3.safety_monitor:main',
            'ui_status = auto_nav_part3.ui_status:main',
        ],
    },
)
