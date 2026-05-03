from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition, UnlessCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    is_real              = LaunchConfiguration("is_real")
    std_dev_linear_vel   = LaunchConfiguration("std_dev_linear_vel")
    std_dev_angular_vel  = LaunchConfiguration("std_dev_angular_vel")
    std_dev_landmark_x   = LaunchConfiguration("std_dev_landmark_x")
    std_dev_landmark_y   = LaunchConfiguration("std_dev_landmark_y")
    gating_threshold     = LaunchConfiguration("gating_threshold")
    aruco_px_thresh      = LaunchConfiguration("aruco_pixel_threshold")

    ekf_params = [{
        "is_real":             ParameterValue(is_real,             value_type=bool),
        "std_dev_linear_vel":  ParameterValue(std_dev_linear_vel,  value_type=float),
        "std_dev_angular_vel": ParameterValue(std_dev_angular_vel, value_type=float),
        "std_dev_landmark_x":  ParameterValue(std_dev_landmark_x,  value_type=float),
        "std_dev_landmark_y":  ParameterValue(std_dev_landmark_y,  value_type=float),
        "gating_threshold":    ParameterValue(gating_threshold,    value_type=float),
    }]

    perception_params = [{
        "aruco_pixel_threshold": ParameterValue(aruco_px_thresh, value_type=float),
    }]

    return LaunchDescription(
        [
            DeclareLaunchArgument("is_real",               default_value="false"),
            DeclareLaunchArgument("std_dev_linear_vel",    default_value="0.01"),
            DeclareLaunchArgument("std_dev_angular_vel",   default_value="0.087266"),  # 5 deg
            DeclareLaunchArgument("std_dev_landmark_x",    default_value="-1.0"),       # use detector cov
            DeclareLaunchArgument("std_dev_landmark_y",    default_value="-1.0"),
            DeclareLaunchArgument("gating_threshold",      default_value="0.5"),
            DeclareLaunchArgument("aruco_pixel_threshold", default_value="50.0"),

            Node(
                package="turtlebot_landmark_slam",
                executable="odom_to_control_republisher.py",
                name="odom_to_control_republisher",
                output="screen",
                remappings=[("~/odom", "/odom")],
                condition=IfCondition(is_real),
            ),
            Node(
                package="turtlebot_landmark_slam",
                executable="ekf_pipeline_node.py",
                name="ekf",
                output="screen",
                parameters=ekf_params,
                remappings=[
                    ("~/landmarks", "/landmarks"),
                    ("~/control",   "/odom_to_control_republisher/control"),
                    ("~/gt_odom",   "/odom"),
                ],
                condition=IfCondition(is_real),
            ),
            Node(
                package="turtlebot_landmark_slam",
                executable="landmark_publisher_real.py",
                name="landmark_publisher_real",
                output="screen",
                parameters=perception_params,
                remappings=[
                    ("~/landmarks", "/landmarks"),
                ],
                condition=IfCondition(is_real),
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
                condition=UnlessCondition(is_real),
            ),
        ]
    )
