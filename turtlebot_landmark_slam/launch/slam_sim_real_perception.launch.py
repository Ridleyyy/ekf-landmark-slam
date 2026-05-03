"""
Sim integration test: Gazebo + real-perception node feeding the EKF.

Same wiring as slam_sim.launch.py except landmark_publisher_sim is replaced
with landmark_publisher_real. Validates the full perception → EKF pipeline
end-to-end in sim before running on the real robot.

Usage:
  ros2 launch turtlebot_landmark_slam slam_sim_real_perception.launch.py \\
      std_dev_linear_vel:=0.10 std_dev_angular_vel:=0.20 \\
      std_dev_landmark_x:=0.05 std_dev_landmark_y:=0.05 \\
      gating_threshold:=-1.0
  ros2 run turtlebot3_teleop teleop_keyboard
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, SetEnvironmentVariable
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    pkg = get_package_share_directory("turtlebot_landmark_slam")

    turtlebot3_gazebo_launch = os.path.join(
        get_package_share_directory("turtlebot3_gazebo"),
        "launch",
        "turutlebot3_dqn_stage5_LEKF_CYL.launch.py",
    )

    rviz_config = os.path.join(pkg, "config", "slam_viz.rviz")

    std_dev_linear_vel  = LaunchConfiguration("std_dev_linear_vel")
    std_dev_angular_vel = LaunchConfiguration("std_dev_angular_vel")
    std_dev_landmark_x  = LaunchConfiguration("std_dev_landmark_x")
    std_dev_landmark_y  = LaunchConfiguration("std_dev_landmark_y")
    gating_threshold    = LaunchConfiguration("gating_threshold")
    aruco_px_thresh     = LaunchConfiguration("aruco_pixel_threshold")
    max_radius          = LaunchConfiguration("max_radius")
    distance_threshold  = LaunchConfiguration("distance_threshold")
    max_mse             = LaunchConfiguration("max_mse")

    ekf_params = [{
        "is_real":             False,
        "std_dev_linear_vel":  ParameterValue(std_dev_linear_vel,  value_type=float),
        "std_dev_angular_vel": ParameterValue(std_dev_angular_vel, value_type=float),
        "std_dev_landmark_x":  ParameterValue(std_dev_landmark_x,  value_type=float),
        "std_dev_landmark_y":  ParameterValue(std_dev_landmark_y,  value_type=float),
        "gating_threshold":    ParameterValue(gating_threshold,    value_type=float),
    }]

    perception_params = [{
        "aruco_pixel_threshold": ParameterValue(aruco_px_thresh,    value_type=float),
        "max_radius":            ParameterValue(max_radius,         value_type=float),
        "distance_threshold":    ParameterValue(distance_threshold, value_type=float),
        "max_mse":               ParameterValue(max_mse,            value_type=float),
    }]

    return LaunchDescription([
        # ----- Tunable launch args (sim-friendly defaults) -----
        DeclareLaunchArgument("std_dev_linear_vel",    default_value="0.01"),
        DeclareLaunchArgument("std_dev_angular_vel",   default_value="0.087266"),
        DeclareLaunchArgument("std_dev_landmark_x",    default_value="0.10"),
        DeclareLaunchArgument("std_dev_landmark_y",    default_value="0.10"),
        DeclareLaunchArgument("gating_threshold",      default_value="-1.0"),
        DeclareLaunchArgument("aruco_pixel_threshold", default_value="200.0"),
        DeclareLaunchArgument("max_radius",            default_value="0.30"),
        DeclareLaunchArgument("distance_threshold",    default_value="0.10"),
        DeclareLaunchArgument("max_mse",               default_value="0.0001"),

        SetEnvironmentVariable(name="TURTLEBOT3_MODEL", value="burger_cam"),

        IncludeLaunchDescription(PythonLaunchDescriptionSource(turtlebot3_gazebo_launch)),

        Node(
            package="turtlebot_landmark_slam",
            executable="odom_to_control_republisher.py",
            name="odom_to_control_republisher",
            output="screen",
            remappings=[("~/odom", "/odom")],
        ),

        Node(
            package="turtlebot_landmark_slam",
            executable="landmark_publisher_real.py",
            name="landmark_publisher_real",
            output="screen",
            parameters=perception_params,
            remappings=[("~/landmarks", "/landmarks")],
        ),

        Node(
            package="turtlebot_landmark_slam",
            executable="ekf_pipeline_node.py",
            name="ekf",
            output="screen",
            parameters=ekf_params,
            remappings=[
                ("~/landmarks", "/landmarks"),
                ("~/gt_odom",   "/odom"),
                ("~/control",   "/odom_to_control_republisher/control"),
            ],
        ),

        Node(
            package="rviz2",
            executable="rviz2",
            name="rviz2",
            arguments=["-d", rviz_config],
            output="screen",
        ),

        Node(
            package="turtlebot_landmark_slam",
            executable="plot_trajectory.py",
            name="trajectory_recorder",
            output="screen",
        ),
    ])
