from abc import ABC, abstractmethod
from typing import Callable, Tuple
import numpy as np
from rclpy.node import Node

from turtlebot_landmark_slam.types import ControlMeasurement, LandmarkMeasurement

from landmarks_msg.msg import LandmarksMsg
from geometry_msgs.msg import Twist

ControlHandler = Callable[[ControlMeasurement], None]
LandmarkHandler = Callable[[LandmarkMeasurement], None]


class DataProviderBase(ABC):

    def __init__(
        self,
        node: Node,
        control_handler: ControlHandler,
        landmark_handler: LandmarkHandler,
        **kwargs,
    ) -> None:
        self._node = node
        self._control_handler = control_handler
        self._landmark_handler = landmark_handler

        self._last_control_msg_time = None
        self._last_landmark_msg_time = None
        self._last_small_motion_log_time = None

        # --- Motion noise std devs ---
        self.std_dev_linear_vel = float(
            self._node.declare_parameter("std_dev_linear_vel", 0.01).value
        )
        self.std_dev_angular_vel = float(
            self._node.declare_parameter("std_dev_angular_vel", (5 * np.pi) / 180).value
        )

        # --- Landmark measurement noise std devs ---
        # These override the covariance published by the detector.
        # Set to -1.0 (default) to use the detector's own covariance values.
        self.std_dev_landmark_x = float(
            self._node.declare_parameter("std_dev_landmark_x", -1.0).value
        )
        self.std_dev_landmark_y = float(
            self._node.declare_parameter("std_dev_landmark_y", -1.0).value
        )

        self._node.get_logger().info(
            f"[DataProvider] std_dev_linear_vel:  {self.std_dev_linear_vel:.6f}"
        )
        self._node.get_logger().info(
            f"[DataProvider] std_dev_angular_vel: {self.std_dev_angular_vel:.6f} rad"
        )
        if self.std_dev_landmark_x > 0 and self.std_dev_landmark_y > 0:
            self._node.get_logger().info(
                f"[DataProvider] std_dev_landmark_x:  {self.std_dev_landmark_x:.6f} m  (overriding detector)"
            )
            self._node.get_logger().info(
                f"[DataProvider] std_dev_landmark_y:  {self.std_dev_landmark_y:.6f} m  (overriding detector)"
            )
        else:
            self._node.get_logger().info(
                "[DataProvider] std_dev_landmark_x/y: using detector-provided covariance"
            )

        self._landmarks_subscription = self._node.create_subscription(
            LandmarksMsg, "~/landmarks", self.landmarkCallback, 1
        )
        self._control_subscription = self._node.create_subscription(
            Twist, "~/control", self.controlCallback, 1
        )

    def controlCallback(self, twist: Twist):
        now = self._node.get_clock().now()

        if self._last_control_msg_time is None:
            self._last_control_msg_time = now
            return

        dt = (now - self._last_control_msg_time).nanoseconds / 1e9
        self._last_control_msg_time = now

        linear_vel  = twist.linear.x
        angular_vel = twist.angular.z

        if abs(linear_vel) < 0.009 and abs(angular_vel) < 0.09:
            should_log = (
                self._last_small_motion_log_time is None
                or (now - self._last_small_motion_log_time).nanoseconds / 1e9 >= 10.0
            )
            if should_log:
                self._node.get_logger().info(
                    "Small linear or angular motion. Skipping predict step"
                )
                self._last_small_motion_log_time = now
            return

        motion_command, motion_covariance = self._constructMotionWithCovariance(
            linear_vel, angular_vel, self.std_dev_linear_vel, self.std_dev_angular_vel, dt
        )

        assert motion_command.shape    == (3, 1)
        assert motion_covariance.shape == (3, 3)

        dx     = motion_command[0][0]
        dy     = motion_command[1][0]
        dtheta = motion_command[2][0]

        self._control_handler(ControlMeasurement(dx, dy, dtheta, motion_covariance))

    def landmarkCallback(self, landmarks: LandmarksMsg):
        # Resolve whether to override the detector covariance
        use_override = self.std_dev_landmark_x > 0 and self.std_dev_landmark_y > 0
        lmk_std_x = self.std_dev_landmark_x if use_override else None
        lmk_std_y = self.std_dev_landmark_y if use_override else None

        for landmark_msg in landmarks.landmarks:
            landmark_measurement = LandmarkMeasurement.from_landmark_msg(
                landmark_msg,
                std_dev_landmark_x=lmk_std_x,
                std_dev_landmark_y=lmk_std_y,
            )
            self._landmark_handler(landmark_measurement)

    @abstractmethod
    def _constructMotionWithCovariance(
        self,
        linear_vel: float,
        angular_vel: float,
        std_dev_linear_vel: float,
        std_dev_angular_vel: float,
        dt: float,
    ) -> Tuple[np.array, np.array]:
        ...


class OnlineDataProvider(DataProviderBase):

    def __init__(self, node, control_handler, landmark_handler, **kwargs):
        super().__init__(node, control_handler, landmark_handler, **kwargs)

    def _constructMotionWithCovariance(self, linear_vel, angular_vel,
                                        std_dev_linear_vel, std_dev_angular_vel, dt):
        s_linear_vel_x = std_dev_linear_vel  * linear_vel  * dt
        s_linear_vel_y = 0.000000001
        s_angular_vel  = std_dev_angular_vel * angular_vel * dt

        dx     = linear_vel  * dt + s_linear_vel_x
        dy     = 0.0
        dtheta = angular_vel * dt + s_angular_vel

        motion_command    = np.array([[dx], [dy], [dtheta]])
        motion_covariance = np.array([[(s_linear_vel_x)**2, 0.0, 0.0],
                                      [0.0, (s_linear_vel_y)**2, 0.0],
                                      [0.0, 0.0, (s_angular_vel)**2]])
        return motion_command, motion_covariance


class SimulationDataProvider(DataProviderBase):

    def __init__(self, node, control_handler, landmark_handler, **kwargs):
        super().__init__(node, control_handler, landmark_handler, **kwargs)

    def _constructMotionWithCovariance(self, linear_vel, angular_vel,
                                        std_dev_linear_vel, std_dev_angular_vel, dt):
        s_linear_vel_x = std_dev_linear_vel  * linear_vel  * dt
        s_linear_vel_y = 0.000000001
        s_angular_vel  = std_dev_angular_vel * angular_vel * dt

        dx     = linear_vel  * dt + s_linear_vel_x * np.random.standard_normal()
        dy     = 0.0
        dtheta = angular_vel * dt + s_angular_vel  * np.random.standard_normal()

        motion_command    = np.array([[dx], [dy], [dtheta]])
        motion_covariance = np.array([[(s_linear_vel_x)**2, 0.0, 0.0],
                                      [0.0, (s_linear_vel_y)**2, 0.0],
                                      [0.0, 0.0, (s_angular_vel)**2]])
        return motion_command, motion_covariance