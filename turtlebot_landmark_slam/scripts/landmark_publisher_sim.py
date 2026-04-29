#!/usr/bin/env python3
"""
Republishes the four cylinder obstacles from the turtlebot3_dqn_stage2 world as
LandmarksMsg on /landmarks for the EKF update step (is_real=false).

Obstacle world-frame positions are read directly from the Gazebo SDF
(models/turtlebot3_dqn_world/obstacles/model.sdf).  Gz Sim Harmonic's
SceneBroadcaster only populates entity IDs (not names) in the pose/info stream,
so subscribing to that topic via ros_gz_bridge yields empty frame IDs.  Since
the obstacles are static ground-truth landmarks, reading their positions from
the SDF is both correct and reliable.

The robot pose is read from /odom (ground-truth odometry from Gazebo), and each
obstacle position is transformed into the robot body frame before publishing.

Topic remappings (set in simulation.launch.py):
  ~/odom      -> /odom
  ~/landmarks -> /landmarks
"""

import numpy as np
import rclpy
from rclpy.node import Node

from nav_msgs.msg import Odometry

from landmarks_msg.msg import LandmarkMsg, LandmarksMsg

# ---------------------------------------------------------------------------
# Ground-truth obstacle positions in the world/odom frame.
# Source: models/turtlebot3_dqn_world/obstacles/model.sdf
#   <model name='obstacles'> contains four links at (±1, ±1, 0.25).
# label -> (world_x, world_y)
# ---------------------------------------------------------------------------
OBSTACLE_WORLD_POSITIONS: dict[int, tuple[float, float]] = {
    1: (-1.0, -1.0),  # obstacle_1
    2: (-1.0,  1.0),  # obstacle_2
    3: ( 1.0, -1.0),  # obstacle_3
    4: ( 1.0,  1.0),  # obstacle_4
}

class LandmarkPublisherSim(Node):

    def __init__(self) -> None:
        super().__init__("landmark_publisher_sim")

        # Cached robot pose in the odom/world frame
        self._robot_x: float = 0.0
        self._robot_y: float = 0.0
        self._robot_yaw: float = 0.0
        self._robot_pose_received: bool = False

        # Measurement variance (m²) on the diagonal of the 2×2 covariance matrix.
        # types.py: covariance = diag(s_x, s_y), so 0.01 m² → ~0.1 m std dev.
        self.std_dev_landmark_x = float(
            self.declare_parameter("std_dev_landmark_x", 0.01).value
        )
        self.std_dev_landmark_y = float(
            self.declare_parameter("std_dev_landmark_y", 0.01).value
        )
        self.get_logger().info(
            f"[LandmarkPublisherSim] std_dev_landmark_x: {self.std_dev_landmark_x}  "
            f"std_dev_landmark_y: {self.std_dev_landmark_y}"
        )

        self.create_subscription(Odometry, "~/odom", self._odom_callback, 10)

        self._landmarks_pub = self.create_publisher(LandmarksMsg, "~/landmarks", 10)

        # Publish at 2 Hz
        self.create_timer(0.5, self._publish_landmarks)

        self.get_logger().info("landmark_publisher_sim started")

    # ------------------------------------------------------------------
    # Callbacks
    # ------------------------------------------------------------------

    def _odom_callback(self, msg: Odometry) -> None:
        """Cache the robot pose from ground-truth odometry."""
        self._robot_x = msg.pose.pose.position.x
        self._robot_y = msg.pose.pose.position.y
        self._robot_yaw = _yaw_from_quaternion(msg.pose.pose.orientation)
        self._robot_pose_received = True

    # ------------------------------------------------------------------
    # Timer: transform obstacles to robot frame and publish
    # ------------------------------------------------------------------

    def _publish_landmarks(self) -> None:
        if not self._robot_pose_received:
            return

        cos_yaw = np.cos(self._robot_yaw)
        sin_yaw = np.sin(self._robot_yaw)

        landmarks_msg = LandmarksMsg()
        for label, (wx, wy) in OBSTACLE_WORLD_POSITIONS.items():
            # Translate then rotate into robot (base_link) frame
            dx = wx - self._robot_x
            dy = wy - self._robot_y
            rx =  cos_yaw * dx + sin_yaw * dy
            ry = -sin_yaw * dx + cos_yaw * dy

            lm = LandmarkMsg()
            lm.label = label
            lm.x = float(rx)
            lm.y = float(ry)
            lm.s_x = float(self.std_dev_landmark_x)
            lm.s_y = float(self.std_dev_landmark_y)
            landmarks_msg.landmarks.append(lm)

        self._landmarks_pub.publish(landmarks_msg)


# ---------------------------------------------------------------------------
# Helpers
# ---------------------------------------------------------------------------

def _yaw_from_quaternion(q) -> float:
    """Extract yaw (rotation about Z) from a geometry_msgs Quaternion."""
    siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
    cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
    return float(np.arctan2(siny_cosp, cosy_cosp))


# ---------------------------------------------------------------------------
# Entry point
# ---------------------------------------------------------------------------

def main(args=None) -> None:
    rclpy.init(args=args)
    node = LandmarkPublisherSim()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
