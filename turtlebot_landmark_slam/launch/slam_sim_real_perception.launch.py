"""
Sim integration test: Gazebo + real-perception node feeding the EKF.

Same wiring as slam_sim.launch.py except landmark_publisher_sim is replaced
with landmark_publisher_real. Validates the full perception → EKF pipeline
end-to-end in sim before running on the real robot.

Usage:
  ros2 launch turtlebot_landmark_slam slam_sim_real_perception.launch.py
  ros2 run turtlebot3_teleop teleop_keyboard
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, SetEnvironmentVariable
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node


# ----------------------------------------------------------------------------
# EKF noise (matches Variant 1 in slam_sim.launch.py for direct comparison)
# ----------------------------------------------------------------------------
EKF_PARAMS = {
    "is_real":             False,
    "std_dev_linear_vel":  0.01,
    "std_dev_angular_vel": 0.08726646259971647,   # 5 deg in radians
    "std_dev_landmark_x":  0.10,
    "std_dev_landmark_y":  0.10,
    # Innovation gating: reject known-landmark measurements whose predicted-vs-
    # actual distance exceeds this many metres. Set to -1.0 to disable.
    "gating_threshold":    -1.0,
}

# ----------------------------------------------------------------------------
# Perception params tuned for sim cylinders (larger than real ones)
# ----------------------------------------------------------------------------
PERCEPTION_PARAMS = {
    "distance_threshold":    0.10,
    "max_mse":               1e-4,
    "max_radius":            0.30,
    "min_radius":            0.05,
    "min_points":            4,
    # Larger than the real-robot default (50) because the hardcoded calibration
    # is for the physical robot, not the sim's burger_cam mount.
    "aruco_pixel_threshold": 200.0,
}


def generate_launch_description():
    pkg = get_package_share_directory("turtlebot_landmark_slam")

    turtlebot3_gazebo_launch = os.path.join(
        get_package_share_directory("turtlebot3_gazebo"),
        "launch",
        "turutlebot3_dqn_stage5_LEKF_CYL.launch.py",
    )

    rviz_config = os.path.join(pkg, "config", "slam_viz.rviz")

    return LaunchDescription([
        SetEnvironmentVariable(name="TURTLEBOT3_MODEL", value="burger_cam"),

        # Gazebo simulation world (cylinders + ArUco tags)
        IncludeLaunchDescription(PythonLaunchDescriptionSource(turtlebot3_gazebo_launch)),

        # Sim helper: republish ground-truth /odom twist as /control for the EKF
        Node(
            package="turtlebot_landmark_slam",
            executable="odom_to_control_republisher.py",
            name="odom_to_control_republisher",
            output="screen",
            remappings=[("~/odom", "/odom")],
        ),

        # REAL perception node — replaces landmark_publisher_sim.py
        Node(
            package="turtlebot_landmark_slam",
            executable="landmark_publisher_real.py",
            name="landmark_publisher_real",
            output="screen",
            parameters=[PERCEPTION_PARAMS],
            remappings=[("~/landmarks", "/landmarks")],
        ),

        # EKF pipeline in sim mode, now consuming real-perception output on /landmarks
        Node(
            package="turtlebot_landmark_slam",
            executable="ekf_pipeline_node.py",
            name="ekf",
            output="screen",
            parameters=[EKF_PARAMS],
            remappings=[
                ("~/landmarks", "/landmarks"),
                ("~/gt_odom",   "/odom"),
                ("~/control",   "/odom_to_control_republisher/control"),
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

        # Trajectory plotter (Ctrl+C to save slam_trajectory.png)
        Node(
            package="turtlebot_landmark_slam",
            executable="plot_trajectory.py",
            name="trajectory_recorder",
            output="screen",
        ),
    ])
