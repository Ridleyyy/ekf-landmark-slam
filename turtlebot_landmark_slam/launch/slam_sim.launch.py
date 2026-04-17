import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, SetEnvironmentVariable
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    pkg = get_package_share_directory("turtlebot_landmark_slam")

    turtlebot3_gazebo_launch = os.path.join(
        get_package_share_directory("turtlebot3_gazebo"),
        "launch",
        "turtlebot3_dqn_stage2.launch.py",
    )

    rviz_config = os.path.join(pkg, "config", "slam_viz.rviz")

    return LaunchDescription(
        [
            DeclareLaunchArgument("is_real", default_value="false"),
            SetEnvironmentVariable(name="TURTLEBOT3_MODEL", value="burger"),

            # Gazebo simulation world
            IncludeLaunchDescription(PythonLaunchDescriptionSource(turtlebot3_gazebo_launch)),

            # Sim support nodes
            Node(
                package="turtlebot_landmark_slam",
                executable="odom_to_control_republisher.py",
                name="odom_to_control_republisher",
                output="screen",
                remappings=[("~/odom", "/odom")],
            ),
            Node(
                package="turtlebot_landmark_slam",
                executable="landmark_publisher_sim.py",
                name="landmark_publisher_sim",
                output="screen",
                parameters=[{"std_dev_landmark_x": 0.01, "std_dev_landmark_y": 0.01}],
                remappings=[
                    ("~/odom", "/odom"),
                    ("~/landmarks", "/landmarks"),
                ],
            ),

            # EKF pipeline
            Node(
                package="turtlebot_landmark_slam",
                executable="ekf_pipeline_node.py",
                name="ekf",
                output="screen",
                parameters=[{"is_real": False}],
                remappings=[
                    ("~/landmarks", "/landmarks"),
                    ("~/gt_odom", "/odom"),
                    ("~/control", "/odom_to_control_republisher/control"),
                ],
            ),

            # RViz2
            Node(
                package="rviz2",
                executable="rviz2",
                name="rviz2",
                arguments=["-d", rviz_config],
                output="screen",
            ),

            # Trajectory plotter (Ctrl+C to save plot)
            Node(
                package="turtlebot_landmark_slam",
                executable="plot_trajectory.py",
                name="trajectory_recorder",
                output="screen",
            ),
        ]
    )
