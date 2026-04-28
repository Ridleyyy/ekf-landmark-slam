import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, SetEnvironmentVariable
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node


# ==============================================================================
# STD DEV VARIANTS
# Pick ONE active variant by uncommenting it. Keep the other two commented out.
#
# Parameter meanings:
#   std_dev_linear_vel  — scales motion noise proportional to forward velocity
#   std_dev_angular_vel — scales motion noise proportional to angular velocity
#   std_dev_landmark_x  — overrides detector covariance for landmark x measurement
#   std_dev_landmark_y  — overrides detector covariance for landmark y measurement
#
# Larger motion std devs  → EKF trusts odometry less, corrects more from landmarks
# Larger landmark std devs → EKF trusts sensor less, relies more on odometry
# ==============================================================================

# ------------------------------------------------------------------------------
# VARIANT 1 — Baseline: balanced trust between odometry and landmarks (ACTIVE)
# ------------------------------------------------------------------------------
# EKF_PARAMS = {
#     "is_real":              False,
#     "std_dev_linear_vel":   0.01,
#     "std_dev_angular_vel":  0.08726646259971647,   # 5 deg in radians
#     "std_dev_landmark_x":   0.10,
#     "std_dev_landmark_y":   0.10,
# }

# ------------------------------------------------------------------------------
# VARIANT 2 — Trust landmarks more: high motion noise, tight landmark noise
# Useful when odometry drifts badly (wheel slip, uneven terrain).
# Uncomment below and comment out Variant 1 above to activate.
# ------------------------------------------------------------------------------
# EKF_PARAMS = {
#     "is_real":              False,
#     "std_dev_linear_vel":   0.10,
#     "std_dev_angular_vel":  0.34906585039886590,   # 20 deg in radians
#     "std_dev_landmark_x":   0.05,
#     "std_dev_landmark_y":   0.05,
# }

# ------------------------------------------------------------------------------
# VARIANT 3 — Trust odometry more: tight motion noise, large landmark noise
# Useful when the lidar has noisy detections (occlusion, reflections).
# Uncomment below and comment out Variant 1 above to activate.
# ------------------------------------------------------------------------------
EKF_PARAMS = {
    "is_real":              False,
    "std_dev_linear_vel":   0.005,
    "std_dev_angular_vel":  0.01745329251994330,   # 1 deg in radians
    "std_dev_landmark_x":   0.30,
    "std_dev_landmark_y":   0.30,
}


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
            #IncludeLaunchDescription(PythonLaunchDescriptionSource(turtlebot3_gazebo_launch)),

            # Sim support nodes
            Node(
                package="turtlebot_landmark_slam",
                executable="landmark_publisher_sim.py",
                name="landmark_publisher_sim",
                output="screen",
                parameters=[{
                    "std_dev_landmark_x": EKF_PARAMS["std_dev_landmark_x"],
                    "std_dev_landmark_y": EKF_PARAMS["std_dev_landmark_y"],
                }],
                remappings=[
                    ("~/odom", "/odom"),
                    ("~/landmarks", "/landmarks"),
                ],
            ),

            # EKF pipeline — receives all four std dev params
            Node(
                package="turtlebot_landmark_slam",
                executable="ekf_pipeline_node.py",
                name="ekf",
                output="screen",
                parameters=[EKF_PARAMS],
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