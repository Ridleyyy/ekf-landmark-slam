#!/usr/bin/env python3
"""
Record /ekf/odom, /odom, and /ekf/map then plot estimated vs ground truth
trajectory and landmark positions on Ctrl+C.

Usage:
    ros2 run turtlebot_landmark_slam plot_trajectory.py

    # Save to file instead of showing interactively:
    ros2 run turtlebot_landmark_slam plot_trajectory.py --ros-args -p output_file:=slam_result.png
"""

import os
import sys
import signal
from datetime import datetime
import numpy as np
import matplotlib
matplotlib.use("Agg")

import matplotlib.pyplot as plt
import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from visualization_msgs.msg import MarkerArray

# Resolve the workspace symlink to find the sandbox directory at runtime.
# ~/ekf_ws/src/turtlebot_landmark_slam is a symlink → .../sandbox/turtlebot_landmark_slam
_ws_src_link = os.path.join(os.path.expanduser("~"), "ekf_ws", "src", "turtlebot_landmark_slam")
_sandbox_dir = os.path.dirname(os.path.realpath(_ws_src_link))
_DEFAULT_OUT_DIR = os.path.join(_sandbox_dir, "sim_outputs")


class TrajectoryRecorder(Node):

    def __init__(self):
        super().__init__("trajectory_recorder")
        self.declare_parameter("output_file", "")

        self._ekf_traj = []   # [(x, y), ...]
        self._gt_traj = []
        self._landmarks = {}  # label -> (x, y)

        self.create_subscription(Odometry, "/ekf/odom", self._ekf_cb, 10)
        self.create_subscription(Odometry, "/odom",     self._gt_cb,  10)
        self.create_subscription(MarkerArray, "/ekf/map", self._map_cb, 10)

        self.get_logger().info("Recording — press Ctrl+C to stop and plot.")

    def _ekf_cb(self, msg: Odometry):
        p = msg.pose.pose.position
        self._ekf_traj.append((p.x, p.y))

    def _gt_cb(self, msg: Odometry):
        p = msg.pose.pose.position
        self._gt_traj.append((p.x, p.y))

    def _map_cb(self, msg: MarkerArray):
        for marker in msg.markers:
            self._landmarks[marker.id] = (
                marker.pose.position.x,
                marker.pose.position.y,
            )

    def plot(self):
        output_file = self.get_parameter("output_file").get_parameter_value().string_value

        fig, ax = plt.subplots(figsize=(10, 8))
        ax.set_aspect("equal")
        ax.grid(True, alpha=0.3)
        ax.set_title("EKF Landmark SLAM — Trajectory and Map")
        ax.set_xlabel("x (m)")
        ax.set_ylabel("y (m)")

        if self._gt_traj:
            gt = np.array(self._gt_traj)
            ax.plot(gt[:, 0], gt[:, 1], color="green", linewidth=1.5,
                    label="Ground truth trajectory")
            ax.plot(*gt[0], "go", markersize=8, label="Start")

        if self._ekf_traj:
            ekf = np.array(self._ekf_traj)
            ax.plot(ekf[:, 0], ekf[:, 1], color="royalblue", linewidth=1.5,
                    linestyle="--", label="EKF estimated trajectory")

        if self._landmarks:
            lx = [v[0] for v in self._landmarks.values()]
            ly = [v[1] for v in self._landmarks.values()]
            ax.scatter(lx, ly, marker="^", s=120, color="red", zorder=5,
                       label="Estimated landmarks")
            for label, (x, y) in self._landmarks.items():
                ax.annotate(str(label), (x, y), textcoords="offset points",
                            xytext=(6, 4), fontsize=8, color="red")

        ax.legend(loc="upper left")
        plt.tight_layout()

        if not output_file:
            timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
            output_file = os.path.join(_DEFAULT_OUT_DIR, f"slam_trajectory_{timestamp}.png")

        os.makedirs(os.path.dirname(output_file), exist_ok=True)
        plt.savefig(output_file, dpi=150, bbox_inches="tight")
        plt.close(fig)
        self.get_logger().info(f"Saved plot to {output_file}")


def main():
    rclpy.init()
    node = TrajectoryRecorder()

    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, SystemExit):
        pass
    finally:
        node.get_logger().info("Stopping — generating plot...")
        node.plot()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
