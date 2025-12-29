from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'nantrobot_robot_sim'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'worlds'), glob(os.path.join('worlds', '*.world'))),
        (os.path.join('share', package_name, 'description'), glob(os.path.join('description', '*.*xacro*'))),
        (os.path.join('share', package_name, 'launch'), glob(os.path.join('launch', '*launch.[pxy][yma]*'))),
        (os.path.join('share', package_name, 'worlds'), glob(os.path.join('worlds', '*.world'))),
        (os.path.join('share', package_name, 'config'), glob(os.path.join('config', '*.yaml'))),
        (os.path.join('share', package_name, 'config'), glob(os.path.join('config', '*.rviz'))),
        (os.path.join('share', package_name, 'mesh'), glob(os.path.join('mesh', '*.*png*'))),
        (os.path.join('share', package_name, 'mesh'), glob(os.path.join('mesh', '*.*dae'))),
        (os.path.join('share', package_name, 'mesh'), glob(os.path.join('mesh', '*.*stl'))),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Alexis MORICE',
    maintainer_email='al3xism0rice@gmail.com',
    description='Ros package for simulating robots for the french robotics cup',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'odom_tf_broadcaster = nantrobot_robot_sim.odom_tf_broadcaster:main',
            'teleport_node = nantrobot_robot_sim.teleport_node:main',
            'table_mesh_publisher = nantrobot_robot_sim.table_mesh_publisher:main',
        ],
    },
)
