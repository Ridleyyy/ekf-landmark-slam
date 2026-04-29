#!/usr/bin/env python3

import math
import numpy as np
import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
import matplotlib.pyplot as plt


class NEESAnalyzer(Node):
    def __init__(self):
        super().__init__("nees_analyzer")

        self.true_pose = None
        self.ekf_pose = None
        self.ekf_cov = None
        self.nees_values = []
        self.time_values = []
        self.start_time = self.get_clock().now()
        self.create_subscription(Odometry, "/odom", self.true_odom_callback, 10)
        self.create_subscription(Odometry, "/ekf/odom", self.ekf_odom_callback, 10)
        self.get_logger().info("NEES analyzer started")
        self.get_logger().info("listening to /odom and /ekf/odom")

    def true_odom_callback(self, msg):
        x = msg.pose.pose.position.x
        y = msg.pose.pose.position.y
        yaw = self.yaw_from_quaternion(msg.pose.pose.orientation)

        self.true_pose = np.array([[x], [y], [yaw]])

    def ekf_odom_callback(self, msg):
        x = msg.pose.pose.position.x
        y = msg.pose.pose.position.y
        yaw = self.yaw_from_quaternion(msg.pose.pose.orientation)
        self.ekf_pose = np.array([[x], [y], [yaw]])
        cov = msg.pose.covariance

        #extracting x, y, yaw covariance block from ROS odometry covariance
        #3 DOF
        P = np.array([
            [cov[0],  cov[1],  cov[5]],
            [cov[6],  cov[7],  cov[11]],
            [cov[30], cov[31], cov[35]],
        ])

        self.ekf_cov = P
        self.compute_nees()

    def compute_nees(self):
        if self.true_pose is None or self.ekf_pose is None or self.ekf_cov is None:
            return

        #computing NEES formula: NEES = pose error^T × inverse covariance × pose error
        error = self.true_pose - self.ekf_pose
        error[2, 0] = self.wrap_angle(error[2, 0])

        P = self.ekf_cov

        #regularisation to avoid singular matrix errors
        P = P + 1e-9 * np.eye(3)

        try:
            nees = float((error.T @ np.linalg.inv(P) @ error)[0, 0])
        except np.linalg.LinAlgError:
            self.get_logger().warn("covariance matrix is singular, skipping NEES value")
            return

        elapsed = (self.get_clock().now() - self.start_time).nanoseconds / 1e9

        self.nees_values.append(nees)
        self.time_values.append(elapsed)

    def save_plot(self):
        if len(self.nees_values) == 0:
            self.get_logger().warn("no NEES values recorded, no plot saved")
            return

        #confidence interval:
        r1 = 0.352 #chi-square 95% lower bound for 3 DOF
        r2 = 7.815 #chi-square 95% upper bound for 3 DOF

        #checking if NEES is within the bounds of the confidence intevals [r1,r2]
        within_bounds = [
            r1 <= nees <= r2 for nees in self.nees_values
        ]
        percentage_within_bounds = sum(within_bounds) / len(within_bounds) * 100.0

        plt.figure(figsize=(10, 5))
        plt.plot(self.time_values, self.nees_values, label="NEES")
        plt.axhline(r1, linestyle="--", color="red", label="Lower bound (95%)")
        plt.axhline(r2, linestyle="--", color="orange", label="Upper bound (95%)")

        plt.xlabel("Time (s)")
        plt.ylabel("Normalized Estimation Error Squared")
        plt.title("EKF pose consistency using NEES")
        plt.legend()
        plt.grid(True)
        plt.tight_layout()
        plt.savefig("nees_plot.png", dpi=300)

        self.get_logger().info("plot saved to nees_plot.png")
        self.get_logger().info(f"NEES within bounds: {percentage_within_bounds:.2f}%")

    @staticmethod
    def yaw_from_quaternion(q):
        siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
        return math.atan2(siny_cosp, cosy_cosp)

    @staticmethod
    def wrap_angle(angle):
        return math.atan2(math.sin(angle), math.cos(angle))


def main(args=None):
    rclpy.init(args=args)
    node = NEESAnalyzer()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.save_plot()
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
