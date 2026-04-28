from dataclasses import dataclass
import numpy as np
from landmarks_msg.msg import LandmarkMsg

@dataclass
class ControlMeasurement:
    dx: float
    dy: float
    dtheta: float
    covariance: np.array # [3x3]

    @property
    def motion_vector(self):
        return np.array([[self.dx], [self.dy], [self.dtheta]], copy=True)

    def __str__(self):
        return f"sx: {self.dx} dy: {self.dy} dtheta: {self.dtheta} cov: {self.covariance}"

@dataclass
class LandmarkMeasurement:
    x: float
    y: float
    label: int
    covariance: np.array # [2x2]
 
    def __str__(self):
        return f"x: {self.x} y: {self.y} label: {self.label} cov: {self.covariance}"
 
    @classmethod
    def from_landmark_msg(
        cls,
        msg: LandmarkMsg,
        std_dev_landmark_x: float = None,
        std_dev_landmark_y: float = None,
    ):
        """Construct a LandmarkMeasurement from a ROS message.
 
        If std_dev_landmark_x / std_dev_landmark_y are provided they override
        the covariance values supplied by the detector (msg.s_x, msg.s_y).
        This lets you tune measurement noise independently of the detector.
        """
        if std_dev_landmark_x is not None and std_dev_landmark_y is not None:
            # Use our own std dev constants — squares give the variance
            s_x = std_dev_landmark_x ** 2
            s_y = std_dev_landmark_y ** 2
        else:
            s_x = msg.s_x
            s_y = msg.s_y
 
        covariance = np.array([[s_x, 0.0],
                               [0.0, s_y]])
 
        # Deal with measurement covariance close to zero
        if s_x < 10**(-4) and s_y < 10**(-4):
            print("Measurement covariance is close to zero. Clamping to 0.01.")
            covariance = np.array([[0.01, 0.0], [0.0, 0.01]])
 
        return cls(msg.x, msg.y, msg.label, covariance)
