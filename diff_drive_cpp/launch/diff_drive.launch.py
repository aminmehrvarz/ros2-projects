from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='diff_drive_cpp',
            executable='diff_drive_node',
            name='diff_drive_kinematics',
            output='screen'
        ),
        Node(
            package='diff_drive_cpp',
            executable='diff_drive_odom_node',
            name='diff_drive_odom',
            output='screen'
        )
    ])