import math

import numpy as np
import rclpy
from rclpy.node import Node
from threading import Lock

from nav_msgs.msg import Odometry
from visualization_msgs.msg import Marker, MarkerArray

from turtlebot_landmark_slam.ekf import ExtendedKalmanFilter
from turtlebot_landmark_slam.types import LandmarkMeasurement, ControlMeasurement
from turtlebot_landmark_slam.dataprovider import *  # lazy import
from turtlebot_landmark_slam.utils import Absolute2RelativeXY


class Pipeline(object):
    """Wires together the EKF with ROS2 data providers and publishers.

    Subscribes to control and landmark measurements, runs the EKF predict/update
    cycle, and publishes the estimated odometry and landmark map.
    """

    def __init__(self, node: Node, ekf: ExtendedKalmanFilter) -> None:
        self._node = node
        self._ekf = ekf
        self._last_odom_time = None
        self._lock = Lock()
        self._seen_landmarks = set()

        self.is_real = bool(self._node.declare_parameter("is_real", False).value)

        # Innovation gating: reject measurements that are more than this many
        # metres from the EKF's predicted measurement. Set to a negative value
        # to disable. Only applied to known landmarks; first observation of a
        # new label is always accepted.
        self._gating_threshold = float(
            self._node.declare_parameter("gating_threshold", 0.5).value
        )

        # Seed EKF starting pose from /odom in BOTH modes so /ekf/odom and the
        # ground-truth /odom share a frame and overlay correctly in RViz.
        self._node.get_logger().info(
            "Waiting for ~/gt_odom to initialise the EKF starting pose."
        )
        self._initialiseStartingPose()

        if self.is_real:
            self._node.get_logger().info(
                "is_real=true: using OnlineDataProvider (no noise added to control input)."
                " Control noise std deviation can be set in ekf_pipeline.launch."
            )
            self._data_provider = OnlineDataProvider(
                self._node, self.controlHandler, self.landmarkHandler
            )
        else:
            self._node.get_logger().info(
                "is_real=false: using SimulationDataProvider (noise added to control input)."
            )
            self._data_provider = SimulationDataProvider(
                self._node, self.controlHandler, self.landmarkHandler
            )

        self.odom_publisher = self._node.create_publisher(Odometry, "~/odom", 1)
        self.map_publisher = self._node.create_publisher(MarkerArray, "~/map", 5)
        self.publisher_timer = self._node.create_timer(0.3, self.publishTimerCallback)

    # ------------------------------------------------------------------
    # EKF callbacks
    # ------------------------------------------------------------------

    def controlHandler(self, control_measurement: ControlMeasurement):
        """Run the EKF predict step on each incoming control measurement."""
        with self._lock:
            self._last_odom_time = self._node.get_clock().now().to_msg()
            self._ekf.predict(control_measurement)
            trace_xy = float(np.trace(self._ekf.pose_covariance[0:2, 0:2]))
            self._node.get_logger().info(
                f"[PREDICT] pose_cov_trace_xy = {trace_xy:.5f} m^2", throttle_duration_sec=1.0
            )

    def landmarkHandler(self, landmark_measurement: LandmarkMeasurement):
        """Run the EKF update step on each incoming landmark measurement,
        gating known landmarks against innovation distance to reject false
        associations."""
        # Ensure publishing turns on as soon as observations arrive, even if
        # /cmd_vel hasn't been received yet (e.g. real robot stationary).
        if self._last_odom_time is None:
            self._last_odom_time = self._node.get_clock().now().to_msg()

        with self._lock:
            is_new = landmark_measurement.label not in self._seen_landmarks

            # Innovation gating for known landmarks
            if not is_new and self._gating_threshold > 0:
                pose = self._ekf.pose
                lmk_abs = self._ekf.extract_landmark_from_state(
                    landmark_measurement.label, self._ekf.state_mean
                )
                predicted, _, _ = Absolute2RelativeXY(pose, lmk_abs)
                measured = np.array([[landmark_measurement.x],
                                     [landmark_measurement.y]])
                innovation_dist = float(np.linalg.norm(measured - predicted))
                if innovation_dist > self._gating_threshold:
                    self._node.get_logger().warn(
                        f"[GATING] Rejecting label={landmark_measurement.label} "
                        f"innovation={innovation_dist:.3f}m > "
                        f"threshold={self._gating_threshold:.3f}m"
                    )
                    return

            if is_new:
                self._seen_landmarks.add(landmark_measurement.label)
            trace_before = float(np.trace(self._ekf.pose_covariance[0:2, 0:2]))
            self._ekf.update(landmark_measurement, is_new)
            trace_after = float(np.trace(self._ekf.pose_covariance[0:2, 0:2]))
            self._node.get_logger().info(
                f"[UPDATE  label={landmark_measurement.label}] "
                f"pose_cov_trace_xy: {trace_before:.5f} -> {trace_after:.5f} m^2"
            )

    # ------------------------------------------------------------------
    # Publishing
    # ------------------------------------------------------------------

    def publishTimerCallback(self):
        self.publishState()

    def publishState(self):
        """Publish the current EKF state as an Odometry message and a landmark MarkerArray."""
        if self._last_odom_time is None:
            return

        self._publishOdometry()
        self._publishLandmarkMap()

    def _publishOdometry(self):
        msg = Odometry()
        msg.header.stamp = self._last_odom_time
        msg.header.frame_id = "odom"
        msg.child_frame_id = "base_link"

        with self._lock:
            msg.pose.pose.position.x = float(self._ekf.x[0])
            msg.pose.pose.position.y = float(self._ekf.y[0])
            msg.pose.pose.position.z = 0.0

            quat = self._quaternion_from_yaw(self._ekf.yaw)
            msg.pose.pose.orientation.x = quat[0]
            msg.pose.pose.orientation.y = quat[1]
            msg.pose.pose.orientation.z = quat[2]
            msg.pose.pose.orientation.w = quat[3]

            # The ROS2 Odometry covariance is a 6x6 matrix (row-major, 36 elements)
            # for [x, y, z, roll, pitch, yaw]. Populate the [x, y, yaw] sub-block.
            pose_cov = self._ekf.pose_covariance
            cov = np.zeros(36, dtype=np.float64)
            cov[0] = pose_cov[0, 0]  # x-x
            cov[1] = pose_cov[0, 1]  # x-y
            cov[5] = pose_cov[0, 2]  # x-yaw
            cov[6] = pose_cov[1, 0]  # y-x
            cov[7] = pose_cov[1, 1]  # y-y
            cov[11] = pose_cov[1, 2]  # y-yaw
            cov[30] = pose_cov[2, 0]  # yaw-x
            cov[31] = pose_cov[2, 1]  # yaw-y
            cov[35] = pose_cov[2, 2]  # yaw-yaw
            msg.pose.covariance = cov

        self.odom_publisher.publish(msg)

    def _publishLandmarkMap(self):
        """Publish three markers per landmark: cylinder, 2-sigma covariance
        ellipse, and text label. Uses _landmark_index for correct label→state
        mapping (set ordering is not guaranteed)."""
        marker_array_msg = MarkerArray()

        with self._lock:
            state = self._ekf.state_mean
            cov   = self._ekf.state_covariance
            label_to_idx = dict(self._ekf._landmark_index)

        for label, idx in label_to_idx.items():
            x = float(state[idx, 0])
            y = float(state[idx + 1, 0])
            lmk_cov = cov[idx:idx + 2, idx:idx + 2]

            # 1) Solid cylinder at the estimated landmark position
            cyl = Marker()
            cyl.header.frame_id = "odom"
            cyl.ns = "landmarks"
            cyl.id = int(label)
            cyl.type = Marker.CYLINDER
            cyl.action = Marker.ADD
            cyl.pose.position.x = x
            cyl.pose.position.y = y
            cyl.pose.position.z = 0.0
            cyl.pose.orientation.w = 1.0
            cyl.scale.x = 0.10
            cyl.scale.y = 0.10
            cyl.scale.z = 0.30
            cyl.color.r = 1.0
            cyl.color.a = 1.0
            marker_array_msg.markers.append(cyl)

            # 2) 2-sigma covariance ellipse (flat cylinder, semi-transparent)
            eigvals, eigvecs = np.linalg.eigh(lmk_cov)
            eigvals = np.maximum(eigvals, 1e-9)
            angle = math.atan2(eigvecs[1, 1], eigvecs[0, 1])  # major axis dir
            half_yaw = 0.5 * angle

            cov_marker = Marker()
            cov_marker.header.frame_id = "odom"
            cov_marker.ns = "landmark_covariance"
            cov_marker.id = int(label)
            cov_marker.type = Marker.CYLINDER
            cov_marker.action = Marker.ADD
            cov_marker.pose.position.x = x
            cov_marker.pose.position.y = y
            cov_marker.pose.position.z = 0.01
            cov_marker.pose.orientation.z = math.sin(half_yaw)
            cov_marker.pose.orientation.w = math.cos(half_yaw)
            cov_marker.scale.x = 4.0 * math.sqrt(eigvals[1])  # 2-sigma major
            cov_marker.scale.y = 4.0 * math.sqrt(eigvals[0])  # 2-sigma minor
            cov_marker.scale.z = 0.02
            cov_marker.color.r = 1.0
            cov_marker.color.g = 1.0
            cov_marker.color.b = 0.0
            cov_marker.color.a = 0.4
            marker_array_msg.markers.append(cov_marker)

            # 3) Text label hovering above the cylinder
            text = Marker()
            text.header.frame_id = "odom"
            text.ns = "landmark_label"
            text.id = int(label)
            text.type = Marker.TEXT_VIEW_FACING
            text.action = Marker.ADD
            text.pose.position.x = x
            text.pose.position.y = y
            text.pose.position.z = 0.40
            text.pose.orientation.w = 1.0
            text.scale.z = 0.15
            text.color.r = 1.0
            text.color.g = 1.0
            text.color.b = 1.0
            text.color.a = 1.0
            text.text = str(int(label))
            marker_array_msg.markers.append(text)

        self.map_publisher.publish(marker_array_msg)

    # ------------------------------------------------------------------
    # Helpers
    # ------------------------------------------------------------------

    def _initialiseStartingPose(self):
        """Seed the EKF state from a single ground-truth odometry message."""
        odom = self._wait_for_message("~/gt_odom", Odometry, 5.0)

        if odom is None:
            self._node.get_logger().warn(
                "Timed out waiting for ~/gt_odom. EKF starting pose may not match"
                " the simulated robot's actual starting pose."
            )
            return

        self._node.get_logger().info("Initialising EKF starting pose from ~/gt_odom.")
        x = odom.pose.pose.position.x
        y = odom.pose.pose.position.y
        q = odom.pose.pose.orientation
        yaw = self._yaw_from_quaternion(q.x, q.y, q.z, q.w)
        self._ekf._state_vector = np.array([[x], [y], [yaw]])

    @staticmethod
    def _yaw_from_quaternion(x: float, y: float, z: float, w: float) -> float:
        siny_cosp = 2.0 * (w * z + x * y)
        cosy_cosp = 1.0 - 2.0 * (y * y + z * z)
        return math.atan2(siny_cosp, cosy_cosp)

    @staticmethod
    def _quaternion_from_yaw(yaw) -> tuple:
        """Convert a yaw angle (numpy array of shape (1,)) to a (x, y, z, w) quaternion."""
        half_yaw = yaw[0] * 0.5
        return (0.0, 0.0, math.sin(half_yaw), math.cos(half_yaw))

    def _wait_for_message(self, topic: str, msg_type, timeout_sec: float):
        """Spin until a single message arrives on `topic` or the timeout elapses."""
        result = {"msg": None}

        def _callback(msg):
            result["msg"] = msg

        subscription = self._node.create_subscription(msg_type, topic, _callback, 1)
        start_time = self._node.get_clock().now()
        while (
            result["msg"] is None
            and (self._node.get_clock().now() - start_time).nanoseconds / 1e9
            < timeout_sec
        ):
            rclpy.spin_once(self._node, timeout_sec=0.1)

        self._node.destroy_subscription(subscription)
        return result["msg"]
