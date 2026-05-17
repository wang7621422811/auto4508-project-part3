import os
from glob import glob
from setuptools import setup

package_name = 'auto_nav_part3'

data_files = [
    ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
    ('share/' + package_name, ['package.xml']),
    ('share/' + package_name + '/launch', glob('launch/*.launch.py')),
    ('share/' + package_name + '/rviz', glob('rviz/*.rviz')),
    # M1.C1.1：安装 EKF 配置文件，让 launch 文件能用 get_package_share_directory 找到它
    ('share/' + package_name + '/config', glob('config/*.yaml')),
]

# Install the URDF in the expected package share path.
for path in glob(package_name + '/simulation/urdf/*.urdf'):
    data_files.append(('share/' + package_name + '/urdf', [path]))

# Preserve mesh subdirectories when installing simulation meshes.
for path in glob(package_name + '/simulation/meshes/**/*', recursive=True):
    if os.path.isfile(path):
        rel_path = os.path.relpath(path, package_name + '/simulation')
        dest_dir = os.path.join('share', package_name, 'simulation', os.path.dirname(rel_path))
        data_files.append((dest_dir, [path]))

# Install Gazebo world files.
for path in glob(package_name + '/simulation/worlds/*.sdf'):
    data_files.append(('share/' + package_name + '/simulation/worlds', [path]))

setup(
    name=package_name,
    version='0.1.0',
    packages=[package_name],
    data_files=data_files,
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='AUTO4508 Team 18',
    maintainer_email='team18@example.com',
    description='Minimal ROS2 package for Part 3 mapping and discovery.',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'teleop_keyboard = auto_nav_part3.teleop_keyboard:main',
            'state_manager = auto_nav_part3.state_manager:main',
            'mapping_service = auto_nav_part3.mapping_service:main',
            'waypoint_service = auto_nav_part3.waypoint_service:main',
            'safety_monitor = auto_nav_part3.safety_monitor:main',
            'ui_status = auto_nav_part3.ui_status:main',
            'camera_info_publisher = auto_nav_part3.camera_info_publisher:main',
        ],
    },
)
