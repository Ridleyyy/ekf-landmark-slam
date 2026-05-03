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
        """Robot pose as a (3,1) column vector [x, y, yaw].
        FIX: was returning a flat (3,) array which failed the (3,1) assert
        inside Relative2AbsolutePose and Absolute2RelativeXY.
        """
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
    # Helper methods
    # ------------------------------------------------------------------

    @staticmethod
    def block_diag(matrices):
        """Stack a list of 2D arrays into a block-diagonal matrix.
        FIX: was called but never defined.
        """
        # Handle empty matrices list or empty sub-matrices
        matrices = [m for m in matrices if m.size > 0]
        if not matrices:
            return np.zeros((0, 0))
        sizes = [m.shape[0] for m in matrices]
        total = sum(sizes)
        result = np.zeros((total, total))
        idx = 0
        for m in matrices:
            s = m.shape[0]
            result[idx:idx+s, idx:idx+s] = m
            idx += s
        return result

    @staticmethod
    def regularise_matrix(S, eps=1e-6):
        """Add a small diagonal term to ensure S is invertible.
        FIX: was called (as reguarlise_matrix) but never defined.
        """
        return S + eps * np.eye(S.shape[0])

    def extract_landmark_from_state(self, label: int, state_vector: np.ndarray) -> np.ndarray:
        """Return the (2,) absolute position of a landmark from the state vector.
        Returned flat so utils functions that index with [0],[1] get plain scalars.
        """
        idx = self._landmark_index[label]
        return state_vector[idx:idx+2].flatten()

    # ------------------------------------------------------------------
    # EKF predict step
    # ------------------------------------------------------------------

    def predict(self, control: ControlMeasurement):
        """Propagate the state forward using the motion model.

        Only the robot-pose block [0:3] of the state and covariance is updated;
        landmark estimates are unaffected by the motion model.
        """
        motion_command = control.motion_vector
        motion_covariance = control.covariance
        pose = self._state_vector[0:3]  # (3,1) — direct slice, correct shape

        # FIX: was calling utils.motion_model which does not exist.
        # Correct function is utils.Relative2AbsolutePose.
        predicted_robot_pose, F, W = utils.Relative2AbsolutePose(pose, motion_command)
        np.copyto(self._state_vector[0:3], predicted_robot_pose)

        # Update robot-robot covariance block
        P_rr = self._state_covariance[0:3, 0:3]
        self._state_covariance[0:3, 0:3] = F @ P_rr @ F.T + W @ motion_covariance @ W.T

        # Update robot-landmark cross-covariance blocks.
        # Bug fix: previously P_rl was a numpy VIEW of the slice, and the
        # first assignment overwrote the underlying memory before the second
        # line read it again — multiplying F twice and breaking symmetry.
        # Compute the new value once, then write both blocks.
        n = self._state_covariance.shape[0]
        if n > 3:
            new_P_rl = F @ self._state_covariance[0:3, 3:]
            self._state_covariance[0:3, 3:] = new_P_rl
            self._state_covariance[3:, 0:3] = new_P_rl.T

    # ------------------------------------------------------------------
    # EKF update step
    # ------------------------------------------------------------------

    def _log_landmark(self, landmark_measurement: LandmarkMeasurement, is_new: bool):
        """Print a one-line console summary of each landmark processed in update."""
        status = "NEW   " if is_new else "UPDATE"
        n_landmarks = (self._state_vector.shape[0] - 3) // 2
 
        # For known landmarks, also show their current estimated absolute position
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
            f"| {abs_str} m "
            f"| total landmarks={n_landmarks}"
        )


    def update(self, landmark_measurement: LandmarkMeasurement, is_new: bool):
        """Correct the state estimate using a landmark measurement.

        If `is_new` is True the landmark is appended to the state vector and
        the covariance matrix is augmented before the standard EKF update.
        """

        self._log_landmark(landmark_measurement, is_new)
        pose = self.pose  # (3,1) — FIX: pose property now returns correct shape
        state_covariance = self.state_covariance

        # landmark_rel_measurement is kept as (2,1) for matrix arithmetic (innovations etc.)
        landmark_rel_measurement = np.array([[landmark_measurement.x], [landmark_measurement.y]])

        # utils.Relative2AbsoluteXY and Absolute2RelativeXY index into the array with [0] and [1]
        # which returns (1,) sub-arrays when the input is (2,1), making np.array([[x2],[y2],[1]])
        # inhomogeneous and raising a ValueError.  Pass a flat (2,) view instead.
        landmark_rel_flat = landmark_rel_measurement.flatten()  # (2,) — safe for utils functions

        if is_new:
            # ----------------------------------------------------------
            # Augment state vector and covariance with the new landmark
            # ----------------------------------------------------------

            # record the landmark index BEFORE extending the state vector
            lmk_idx = self._state_vector.shape[0]
            self._landmark_index[landmark_measurement.label] = lmk_idx

            # Compute absolute landmark position and Jacobians
            landmark_position, G1, G2 = utils.Relative2AbsoluteXY(pose, landmark_rel_flat)
            self._state_vector = np.vstack((self._state_vector, landmark_position.reshape(2, 1)))

            # Augment covariance matrix
            # Partition existing covariance into robot and landmark blocks
            P_rr = state_covariance[0:3, 0:3]
            P_rl = state_covariance[0:3, 3:]   # (3, 2*existing_landmarks)
            P_ll = state_covariance[3:, 3:]    # (2*existing, 2*existing)

            # New landmark covariance block
            P_new_lmk = G1 @ P_rr @ G1.T + G2 @ landmark_measurement.covariance @ G2.T  # (2,2)

            # New off-diagonal block: covariance between robot pose and new landmark
            # P(robot, new_lmk) = G1 @ P_rr  →  shape (2,3), stored as (3,2) column
            new_col_robot = (G1 @ P_rr).T      # (3, 2)

            # New off-diagonal block: covariance between existing landmarks and new landmark
            # P(existing_lmk, new_lmk) = P_lr @ G1.T  →  shape (2*existing, 2)
            if P_rl.size > 0:
                new_col_lmk = P_rl.T @ G1.T    # (2*existing, 2)
            else:
                new_col_lmk = np.zeros((0, 2))

            # Assemble the new right column [robot↔new, existing_lmk↔new]
            new_col = np.vstack((new_col_robot, new_col_lmk))  # (3+2*existing, 2)

            # Assemble new full covariance:
            #   [ P_rr      P_rl      new_col_robot.T ]
            #   [ P_rl.T    P_ll      new_col_lmk     ]
            #   [ new_col.T           P_new_lmk       ]
            existing = np.block([[P_rr, P_rl], [P_rl.T, P_ll]])  # (N, N) before augment
            self._state_covariance = np.block([
                [existing,  new_col],
                [new_col.T, P_new_lmk]
            ])

            # After augmenting, also run the standard update for this first measurement
            # so the new landmark is immediately corrected (re-fetch updated covariance)
            state_covariance = self.state_covariance
            landmark_position_abs = self.extract_landmark_from_state(
                landmark_measurement.label, self._state_vector
            )
            expected_measurement, H_pose, J_lmk = utils.Absolute2RelativeXY(pose, landmark_position_abs)
            innovation = landmark_rel_measurement - expected_measurement

            N = self._state_covariance.shape[0]
            H_full = np.zeros((2, N))
            H_full[:, 0:3] = H_pose
            H_full[:, lmk_idx:lmk_idx+2] = J_lmk

            S = H_full @ state_covariance @ H_full.T + landmark_measurement.covariance
            K = state_covariance @ H_full.T @ np.linalg.inv(self.regularise_matrix(S))

            self._state_vector += K @ innovation
            # Keep yaw in [-pi, pi] so subsequent predict/update math stays sane
            self._state_vector[2, 0] = utils.pi2pi(self._state_vector[2, 0])
            self._state_covariance = (np.eye(N) - K @ H_full) @ state_covariance

        else:
            # ----------------------------------------------------------
            # Standard EKF update for a previously seen landmark
            # ----------------------------------------------------------

            lmk_idx = self._landmark_index[landmark_measurement.label]

            # We need to predict what the sensor should see, which means
            # converting the stored absolute landmark position into the robot frame.
            landmark_position_abs = self.extract_landmark_from_state(
                landmark_measurement.label, self._state_vector
            )
            expected_measurement, H_pose, J_lmk = utils.Absolute2RelativeXY(pose, landmark_position_abs)
            innovation = landmark_rel_measurement - expected_measurement

            # FIX: H_pose is (2,3) and J_lmk is (2,2) — only covering pose and one landmark.
            # We must build H_full of shape (2, N) with Jacobians placed at the correct columns
            # so that the Kalman gain K = P @ H_full.T @ inv(S) has the right dimensions.
            N = self._state_covariance.shape[0]
            H_full = np.zeros((2, N))
            H_full[:, 0:3] = H_pose
            H_full[:, lmk_idx:lmk_idx+2] = J_lmk

            S = H_full @ state_covariance @ H_full.T + landmark_measurement.covariance
            K = state_covariance @ H_full.T @ np.linalg.inv(self.regularise_matrix(S))

            self._state_vector += K @ innovation
            # Keep yaw in [-pi, pi] so subsequent predict/update math stays sane
            self._state_vector[2, 0] = utils.pi2pi(self._state_vector[2, 0])
            self._state_covariance = (np.eye(N) - K @ H_full) @ state_covariance