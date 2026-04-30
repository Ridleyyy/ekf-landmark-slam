import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, SetEnvironmentVariable
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node


def generate_launch_description():
    pkg = get_package_share_directory("turtlebot_landmark_slam")

    turtlebot3_gazebo_launch = os.path.join(
        get_package_share_directory("turtlebot3_gazebo"),
        "launch",
        "turtlebot3_dqn_stage5_ArucoTag.launch.py",
    )

    rviz_config = os.path.join(pkg, "config", "slam_viz.rviz")

    return LaunchDescription(
        [
            DeclareLaunchArgument("is_real", default_value="false"),
            SetEnvironmentVariable(name="TURTLEBOT3_MODEL", value="burger_cam"),

            # Gazebo simulation world
            IncludeLaunchDescription(PythonLaunchDescriptionSource(turtlebot3_gazebo_launch)),

            # Sim support nodes
            Node(
                package="turtlebot_landmark_slam",
                executable="odom_to_control_republisher.py",
                name="odom_to_control_republisher",
                output="screen",
                parameters=[{"use_sim_time": True}],
                remappings=[("~/odom", "/odom")],
            ),
            Node(
                package="turtlebot_landmark_slam",
                executable="landmark_publisher_sim.py",
                name="landmark_publisher_sim",
                output="screen",
                parameters=[{
                    "use_sim_time": True,
                    "std_dev_landmark_x": 0.01,
                    "std_dev_landmark_y": 0.01,
                }],
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
                parameters=[{"is_real": False, "use_sim_time": True}],
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
                parameters=[{"use_sim_time": True}],
            ),

            # Trajectory plotter (Ctrl+C to save plot)
            Node(
                package="turtlebot_landmark_slam",
                executable="plot_trajectory.py",
                name="trajectory_recorder",
                output="screen",
                parameters=[{"use_sim_time": True}],
            ),

            # Teleop in its own terminal window
            Node(
                package="turtlebot3_teleop",
                executable="teleop_keyboard",
                name="teleop_keyboard",
                output="screen",
                prefix="xterm -e",
            ),
        ]
    )
