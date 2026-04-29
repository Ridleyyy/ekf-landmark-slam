import numpy as np
from turtlebot_landmark_slam.types import LandmarkMeasurement, ControlMeasurement
import turtlebot_landmark_slam.utils as utils
from copy import deepcopy


class ExtendedKalmanFilter(object):
    """EKF-SLAM filter that jointly estimates robot pose and landmark positions.

    The state vector has shape (3 + 2*M, 1) where M is the number of observed
    landmarks:  [x, y, yaw, lmk0_x, lmk0_y, lmk1_x, lmk1_y, ...]^T
    """

    def __init__(self) -> None:
        # State vector — grows as new landmarks are discovered (shape N x 1)
        self._state_vector = np.array([[0.0], [0.0], [0.0]])

        sigma_position = np.sqrt(10 ** (-3))
        sigma_orientation = np.sqrt(10 ** (-3))

        # Initial 3x3 robot-pose covariance block (grows to N x N with landmarks)
        self._state_covariance = np.array(
            [
                [sigma_position**2, 0.0, 0.0],
                [0.0, sigma_position**2, 0.0],
                [0.0, 0.0, sigma_orientation**2],
            ]
        )

        # Maps landmark label -> starting row index in the state vector
        self._landmark_index = {}

    # ------------------------------------------------------------------
    # State accessors
    # ------------------------------------------------------------------

    @property
    def x(self):
        return deepcopy(self._state_vector[0])

    @property
    def y(self):
        return deepcopy(self._state_vector[1])

    @property
    def yaw(self):
        return deepcopy(self._state_vector[2])

    @property
    def pose(self):
        """Robot pose as a (3,) array [x, y, yaw]."""
        return np.array([self.x, self.y, self.yaw], copy=True)

    @property
    def pose_covariance(self):
        """3x3 covariance block for the robot pose."""
        return np.array(self._state_covariance[0:3, 0:3], copy=True)

    @property
    def state_mean(self):
        """Full state vector (robot pose + landmark positions), shape (N, 1)."""
        return np.array(self._state_vector, copy=True)

    @property
    def state_covariance(self):
        """Full N x N state covariance matrix."""
        return np.array(self._state_covariance, copy=True)

    # ------------------------------------------------------------------
    # EKF predict step
    # ------------------------------------------------------------------

    def predict(self, control: ControlMeasurement):
        """Propagate the state forward using the motion model.

        Only the robot-pose block [0:3] of the state and covariance is updated;
        landmark estimates are unaffected by the motion model.
        """
        motion_command = control.motion_vector   # (3,1)
        motion_covariance = control.covariance   # (3,3)
        pose = self._state_vector[0:3]           # (3,1)

        # Propagate pose and get Jacobians
        new_pose, F, W = utils.Relative2AbsolutePose(pose, motion_command)
        np.copyto(self._state_vector[0:3], new_pose)

        N = self._state_covariance.shape[0]
        P = self._state_covariance

        # Robot-robot covariance block: F·P_rr·Fᵀ + W·Q·Wᵀ
        new_P_rr = F @ P[0:3, 0:3] @ F.T + W @ motion_covariance @ W.T

        # Robot-landmark cross-covariance: F·P_rl
        if N > 3:
            new_P_rl = F @ P[0:3, 3:]
            self._state_covariance[0:3, 3:] = new_P_rl
            self._state_covariance[3:, 0:3] = new_P_rl.T

        self._state_covariance[0:3, 0:3] = new_P_rr

    # ------------------------------------------------------------------
    # EKF update step
    # ------------------------------------------------------------------

    def update(self, landmark_measurement: LandmarkMeasurement, is_new: bool):
        """Correct the state estimate using a landmark measurement.

        If `is_new` is True the landmark is appended to the state vector and
        the covariance matrix is augmented before the standard EKF update.
        """
        pose = self._state_vector[0:3]                                   # (3,1) — view
        R = landmark_measurement.covariance                               # (2,2)
        z = np.array([[landmark_measurement.x], [landmark_measurement.y]])  # (2,1)
        z_flat = np.array([landmark_measurement.x, landmark_measurement.y]) # (2,) for utils
        label = landmark_measurement.label

        if is_new:
            N = self._state_vector.shape[0]
            P = self._state_covariance

            # Map relative measurement to absolute landmark position
            landmark_abs, G1, G2 = utils.Relative2AbsoluteXY(pose, z_flat)  # (2,1), (2,3), (2,2)

            # Record state-vector row index before augmenting
            self._landmark_index[label] = N

            # Augment state vector: append new landmark position
            self._state_vector = np.vstack([self._state_vector, landmark_abs])

            # Augment covariance:
            #   Cov(existing_state, new_landmark) = P[:,0:3] @ G1ᵀ
            #   Var(new_landmark)                 = G1 @ P[0:3,0:3] @ G1ᵀ + G2 @ R @ G2ᵀ
            P_cross = P[0:N, 0:3] @ G1.T                              # (N,2)
            P_lm = G1 @ P[0:3, 0:3] @ G1.T + G2 @ R @ G2.T          # (2,2)
            self._state_covariance = np.block([[P, P_cross],
                                               [P_cross.T, P_lm]])

        # Standard EKF update for this landmark (new or existing)
        lm_idx = self._landmark_index[label]
        N = self._state_vector.shape[0]
        P = self._state_covariance

        # Expected measurement and observation Jacobians
        landmark_abs_flat = self._state_vector[lm_idx:lm_idx + 2, 0]      # (2,) for utils
        z_pred, H, J = utils.Absolute2RelativeXY(pose, landmark_abs_flat)  # (2,1), (2,3), (2,2)

        # Observation Jacobian C (2 × N): non-zero at robot-pose and landmark blocks
        C = np.zeros((2, N))
        C[0:2, 0:3] = H
        C[0:2, lm_idx:lm_idx + 2] = J

        # Innovation
        y = z - z_pred  # (2,1)

        # Innovation covariance
        S = C @ P @ C.T + R  # (2,2)
        if abs(np.linalg.det(S)) < 1e-6:
            S = S + 0.1 * np.eye(2)

        # Kalman gain and state/covariance update
        K = P @ C.T @ np.linalg.inv(S)                   # (N,2)
        self._state_vector = self._state_vector + K @ y  # (N,1)
        self._state_covariance = (np.eye(N) - K @ C) @ P # (N,N)

        # Keep yaw in [-π, π]
        self._state_vector[2, 0] = utils.pi2pi(float(self._state_vector[2, 0]))
