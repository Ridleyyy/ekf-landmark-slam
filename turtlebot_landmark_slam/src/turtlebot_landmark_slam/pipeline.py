from turtlebot_landmark_slam.ekf import ExtendedKalmanFilter
from turtlebot_landmark_slam.types import LandmarkMeasurement, ControlMeasurement
from turtlebot_landmark_slam.dataprovider import *  # lazy import
import numpy as np
from rclpy.node import Node
import math

from nav_msgs.msg import Odometry

from visualization_msgs.msg import Marker, MarkerArray
from geometry_msgs.msg import PoseStamped

from threading import RLock, Lock


class Pipeline(object):

    def __init__(self, node: Node, ekf: ExtendedKalmanFilter) -> None:
        self._node = node
        self._ekf = ekf
        self._last_odom_time = None
        self._lock = Lock()

        self.is_real = bool(self._node.declare_parameter("is_real", False).value)
        self._seen_landmarks = set()

        self._data_provider = None
        if self.is_real:
            self._node.get_logger().info(
                "~is_real param set to true. The OnlineDataProvider will be used which does not add any noise"
                " to your control measurements. The std deviation of control input can be set via the relevant params in"
                " the ekf_pipeline.launch file."
            )
            self._data_provider = OnlineDataProvider(
                self._node, self.controlHandler, self.landmarkHandler
            )
        else:
            self._node.get_logger().info(
                "~is_real param set to false. The SimulationDataProvider will be used which adds noise to the control input "
                " according to the set std deviation. These params are in ekf_pipeline.launch file. \n"
                "Program will wait for ~gt_odom message to initalise the starting state of the ekf."
            )
            self._initaliseStartingPose()
            self._data_provider = SimulationDataProvider(
                self._node, self.controlHandler, self.landmarkHandler
            )

        self.odom_publisher = self._node.create_publisher(Odometry, "~/odom", 1)
        self.map_publisher = self._node.create_publisher(MarkerArray, "~/map", 5)
        self.publisher_timer = self._node.create_timer(0.3, self.publishTimerCallback)

    def controlHandler(self, control_measurement: ControlMeasurement):
        with self._lock:
            self._last_odom_time = self._node.get_clock().now().to_msg()
            self._ekf.predict(control_measurement)

    def landmarkHandler(self, landmark_measurement: LandmarkMeasurement):
        with self._lock:
            is_new_label = True

            if landmark_measurement.label in self._seen_landmarks:
                is_new_label = False
            else:
                self._seen_landmarks.add(landmark_measurement.label)

            self._ekf.update(landmark_measurement, is_new_label)

    def _initaliseStartingPose(self):
        odom = self._wait_for_message("~/gt_odom", Odometry, 5.0)

        if odom is None:
            self._node.get_logger().warn(
                "Initalising starting pose failed listening to ~/gt_odom."
                " Starting pose may not match the pose of the simulated turtlebot"
            )
        else:
            self._node.get_logger().info(
                "Initalising ekf starting state with odometry."
            )
            x = odom.pose.pose.position.x
            y = odom.pose.pose.position.y
            quaternion = odom.pose.pose.orientation
            yaw = self._yaw_from_quaternion(
                quaternion.x, quaternion.y, quaternion.z, quaternion.w
            )
            self._ekf._state_vector = np.array([[x], [y], [yaw]])

    def publishTimerCallback(self):
        self.publishState()

    def publishState(self):
        msg = Odometry()
        if self._last_odom_time is None:
            return

        msg.header.stamp = self._last_odom_time  # time stamp
        msg.header.frame_id = "odom"  # reference frame
        msg.child_frame_id = "base_link"

        with self._lock:
            # Construct content
            msg.pose.pose.position.x = self._ekf.x[0]
            msg.pose.pose.position.y = self._ekf.y[0]
            msg.pose.pose.position.z = 0.0
            quat = self._quaternion_from_yaw(self._ekf.yaw)
            msg.pose.pose.orientation.x = quat[0]
            msg.pose.pose.orientation.y = quat[1]
            msg.pose.pose.orientation.z = quat[2]
            msg.pose.pose.orientation.w = quat[3]

            current_covariance = self._ekf.pose_covariance
            cov = np.zeros((36, 1), dtype=np.float64)
            cov[0] = current_covariance[0, 0]
            cov[1] = current_covariance[0, 1]
            cov[5] = current_covariance[0, 2]
            cov[6] = current_covariance[1, 0]
            cov[7] = current_covariance[1, 1]
            cov[11] = current_covariance[1, 2]
            cov[30] = current_covariance[2, 0]
            cov[31] = current_covariance[2, 1]
            cov[35] = current_covariance[2, 2]
            msg.pose.covariance = np.ravel(cov)

        self.odom_publisher.publish(msg)

        landmark_poses = self._ekf.state_mean[3:]
        marker_array_msg = MarkerArray()

        seen_landmarks = list(self._seen_landmarks)

        for i in range(int(len(landmark_poses) / 2)):
            marker = Marker()
            marker.header.frame_id = "odom"
            marker.id = seen_landmarks[i]
            marker.type = 3
            marker.action = 0
            marker.pose.position.x = landmark_poses[2 * i]
            marker.pose.position.y = landmark_poses[2 * i + 1]
            marker.pose.position.z = 0.0
            marker.pose.orientation.x = 0.0
            marker.pose.orientation.y = 0.0
            marker.pose.orientation.z = 0.0
            marker.pose.orientation.w = 1.0
            marker.color.r = 1.0
            marker.color.g = 0.0
            marker.color.b = 0.0
            marker.color.a = 1.0
            marker.scale.x = 0.1
            marker.scale.y = 0.1
            marker.scale.z = 0.1
            marker.frame_locked = False
            marker.lifetime.sec = 0
            marker.lifetime.nanosec = 0
            marker_array_msg.markers.append(marker)

        self.map_publisher.publish(marker_array_msg)

    @staticmethod
    def _yaw_from_quaternion(x: float, y: float, z: float, w: float) -> float:
        siny_cosp = 2.0 * (w * z + x * y)
        cosy_cosp = 1.0 - 2.0 * (y * y + z * z)
        return math.atan2(siny_cosp, cosy_cosp)

    @staticmethod
    def _quaternion_from_yaw(yaw: float):
        half_yaw = yaw[0] * 0.5
        return (0.0, 0.0, math.sin(half_yaw), math.cos(half_yaw))

    def _wait_for_message(self, topic: str, msg_type, timeout_sec: float):
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
            import rclpy

            rclpy.spin_once(self._node, timeout_sec=0.1)

        self._node.destroy_subscription(subscription)
        return result["msg"]
