from launch import LaunchDescription
from launch.actions import SetEnvironmentVariable
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        SetEnvironmentVariable('TURTLEBOT3_MODEL', 'burger_cam'),
        Node(
            package='turtlebot3_teleop',
            executable='teleop_keyboard',
            name='teleop_keyboard',
            output='screen',
            prefix='xterm -e',
        ),
    ])
