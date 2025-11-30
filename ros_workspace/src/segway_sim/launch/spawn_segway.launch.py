from launch import LaunchDescription
from launch_ros.actions import Node
import os
from ament_index_python.packages import get_package_share_directory
from launch.actions import ExecuteProcess

def generate_launch_description():
    pkg_dir = get_package_share_directory('segway_sim')

    # Launch gazebo
    gz = ExecuteProcess(
        cmd=['gazebo', '--verbose', '-s', 'libgazebo_ros_factory.so', 'empty_world.sdf'],
        output='screen'
    )

    # spawn robot
    robot_file = os.path.join(pkg_dir, 'urdf', 'segway.urdf.xacro')
    spawn = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        arguments=['-file', robot_file, '-entity', 'segway'],
        output='screen'
    )

    # balance controller node
    balance = Node(
        package='segway_sim',
        executable='balance_controller.py',
        output='screen',
        parameters=[{
            'Kp': 6.0,
            'Ki': 0.0,
            'Kd': 0.6,
            'max_speed': 1.0
        }]
    )

    return LaunchDescription([
        gz,
        spawn,
        balance
    ])
