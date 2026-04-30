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
        """Robot pose as a (3,1) column vector — matches the shape assert in utils functions."""
        return self._state_vector[0:3].copy()

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
    # Helper methods
    # ------------------------------------------------------------------

    @staticmethod
    def regularise_matrix(S, eps=1e-6):
        return S + eps * np.eye(S.shape[0])

    def extract_landmark_from_state(self, label: int, state_vector: np.ndarray) -> np.ndarray:
        """Return the (2,) absolute position of a landmark from the state vector."""
        idx = self._landmark_index[label]
        return state_vector[idx:idx + 2].flatten()

    def _log_landmark(self, landmark_measurement: LandmarkMeasurement, is_new: bool):
        status = "NEW   " if is_new else "UPDATE"
        n_landmarks = (self._state_vector.shape[0] - 3) // 2
        if not is_new and landmark_measurement.label in self._landmark_index:
            abs_pos = self.extract_landmark_from_state(
                landmark_measurement.label, self._state_vector
            )
            abs_str = f"abs=({abs_pos[0]:+.3f}, {abs_pos[1]:+.3f})"
        else:
            abs_str = "abs=(not yet in state)"
        print(
            f"[EKF] {status} | label={landmark_measurement.label:3d} "
            f"| rel=({landmark_measurement.x:+.3f}, {landmark_measurement.y:+.3f}) m "
            f"| {abs_str} m | total landmarks={n_landmarks}"
        )

    # ------------------------------------------------------------------
    # EKF update step
    # ------------------------------------------------------------------

    def update(self, landmark_measurement: LandmarkMeasurement, is_new: bool):
        """Correct the state estimate using a landmark measurement.

        If `is_new` is True the landmark is appended to the state vector and
        the covariance matrix is augmented before the standard EKF update.
        """
        self._log_landmark(landmark_measurement, is_new)

        pose = self.pose                          # (3,1) copy — satisfies utils shape asserts
        state_covariance = self.state_covariance  # copy — immune to in-place mutation

        landmark_rel_measurement = np.array([[landmark_measurement.x],
                                             [landmark_measurement.y]])  # (2,1)
        landmark_rel_flat = landmark_rel_measurement.flatten()            # (2,) for utils

        if is_new:
            # Record index before extending state
            lmk_idx = self._state_vector.shape[0]
            self._landmark_index[landmark_measurement.label] = lmk_idx

            # Compute absolute landmark position and Jacobians
            landmark_position, G1, G2 = utils.Relative2AbsoluteXY(pose, landmark_rel_flat)
            self._state_vector = np.vstack((self._state_vector, landmark_position.reshape(2, 1)))

            # Augment covariance
            P_rr = state_covariance[0:3, 0:3]
            P_rl = state_covariance[0:3, 3:]
            P_ll = state_covariance[3:, 3:]

            P_new_lmk = G1 @ P_rr @ G1.T + G2 @ landmark_measurement.covariance @ G2.T
            new_col_robot = (G1 @ P_rr).T                                          # (3,2)
            new_col_lmk = P_rl.T @ G1.T if P_rl.size > 0 else np.zeros((0, 2))    # (2*M,2)
            new_col = np.vstack((new_col_robot, new_col_lmk))

            existing = np.block([[P_rr, P_rl], [P_rl.T, P_ll]])
            self._state_covariance = np.block([
                [existing,  new_col],
                [new_col.T, P_new_lmk]
            ])

            # Run standard update immediately for this first measurement
            state_covariance = self.state_covariance
            landmark_position_abs = self.extract_landmark_from_state(
                landmark_measurement.label, self._state_vector
            )
            expected_measurement, H_pose, J_lmk = utils.Absolute2RelativeXY(
                pose, landmark_position_abs
            )
            innovation = landmark_rel_measurement - expected_measurement

            N = self._state_covariance.shape[0]
            H_full = np.zeros((2, N))
            H_full[:, 0:3] = H_pose
            H_full[:, lmk_idx:lmk_idx + 2] = J_lmk

            S = H_full @ state_covariance @ H_full.T + landmark_measurement.covariance
            K = state_covariance @ H_full.T @ np.linalg.inv(self.regularise_matrix(S))

            self._state_vector += K @ innovation
            self._state_covariance = (np.eye(N) - K @ H_full) @ state_covariance

        else:
            # Standard EKF update for a previously seen landmark
            lmk_idx = self._landmark_index[landmark_measurement.label]

            landmark_position_abs = self.extract_landmark_from_state(
                landmark_measurement.label, self._state_vector
            )
            expected_measurement, H_pose, J_lmk = utils.Absolute2RelativeXY(
                pose, landmark_position_abs
            )
            innovation = landmark_rel_measurement - expected_measurement

            N = self._state_covariance.shape[0]
            H_full = np.zeros((2, N))
            H_full[:, 0:3] = H_pose
            H_full[:, lmk_idx:lmk_idx + 2] = J_lmk

            S = H_full @ state_covariance @ H_full.T + landmark_measurement.covariance
            K = state_covariance @ H_full.T @ np.linalg.inv(self.regularise_matrix(S))

            self._state_vector += K @ innovation
            self._state_covariance = (np.eye(N) - K @ H_full) @ state_covariance
