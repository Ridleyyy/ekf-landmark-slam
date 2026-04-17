#!/usr/bin/env python3
"""
Record /ekf/odom, /odom, and /ekf/map then plot estimated vs ground truth
trajectory and landmark positions on Ctrl+C.

Usage:
    ros2 run turtlebot_landmark_slam plot_trajectory.py

    # Save to file instead of showing interactively:
    ros2 run turtlebot_landmark_slam plot_trajectory.py --ros-args -p output_file:=slam_result.png
"""

import sys
import signal
import numpy as np
import matplotlib
matplotlib.use("Agg")  # non-interactive backend; switched to TkAgg below if display available
try:
    import matplotlib.pyplot as plt
    matplotlib.use("TkAgg")
    import matplotlib.pyplot as plt
except Exception:
    import matplotlib.pyplot as plt

import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from visualization_msgs.msg import MarkerArray


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

        if output_file:
            plt.savefig(output_file, dpi=150)
            self.get_logger().info(f"Saved plot to {output_file}")
        else:
            plt.savefig("slam_trajectory.png", dpi=150)
            self.get_logger().info("Saved plot to slam_trajectory.png")
            try:
                plt.show()
            except Exception:
                pass


def main():
    rclpy.init()
    node = TrajectoryRecorder()

    def _shutdown(sig, frame):
        node.get_logger().info("Stopping — generating plot...")
        node.plot()
        node.destroy_node()
        rclpy.shutdown()
        sys.exit(0)

    signal.signal(signal.SIGINT, _shutdown)

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.plot()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
