from setuptools import find_packages, setup
import os, glob

package_name = 'segway_sim'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        # ('share/ament_index/resource_index/packages',
        #    ['resource/' + "spawn_segway_launch.py"]), # Launch files
        # ('share/ament_index/resource_index/packages', 
        #  ['resource/' + "segway.urdf"]),  # URDF (Gazebo) robot description files
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='ashvin',
    maintainer_email='ashvinanilkumarsh@gmail.com',
    description='Self-balancing robot simulation with Gazebo and ROS 2',
    license='Apache-2.0',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            "balance_node = segway_sim.balance_controller:main"
        ],
    },
)
