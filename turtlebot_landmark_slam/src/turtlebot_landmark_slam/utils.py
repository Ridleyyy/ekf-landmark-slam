"""
Created on Tue Aug 9 16:12:08 2016
Calculate the robot pose given the robot previous pose and motion of the robot
with respect to that pose
@author: admin-u5941570
Updated on 26-3-23
@maintainers: Viorela Ila, Max Revay, Jing Cheng, Stefan Williams,
Stephany Berrio Perez, Tejaswi Digumarti, Jesse Morris
"""

import numpy as np


def pi2pi(angle):
    """
    Maps angle to the range of [-pi, pi]
    :param angle: then angle that needs to be mapped to the range [-pi, pi]
    :return : angle in the range [-pi, pi]
    """

    # This function does not need any changes

    dp = 2 * np.pi
    if angle <= -dp or angle >= dp:
        angle = angle % dp
    if angle >= np.pi:
        angle = angle - dp
    if angle <= -np.pi:
        angle = angle + dp

    return angle


def Relative2AbsolutePose(robot_pose_abs, u_rel):
    """
    Calculates the new pose of the robot given its current pose in the
    absolute coordinate frame and a motion input.

    :param robot_pose_abs: current pose of the robot in the absolute reference
    frame [x, y, theta]
    :type robot_pose_abs: [type]
    :param u_rel: motion command in the robot's frame of reference
    [dx, dy, dtheta]
    :type u_rel: [type]
    :return: pose of the robot in the absolute reference frame after applying
    the motion and the Jacobians of the new pose wrt, the current pose
    and the motion command respectively
    :rtype: tuple
    """
    assert robot_pose_abs.shape == (3, 1)
    assert u_rel.shape == (3, 1)

    # TODO: implement the process model for EKF prediction.
    #       1) compute the predicted pose in global coordinates.
    #       2) compute the Jacobian F = df/dx of the motion model with
    #          respect to the robot pose.
    #       3) compute the Jacobian W = df/du of the motion model with
    #          respect to the control input.
    #       4) normalize the yaw angle to [-pi, pi].
    #       5) return the predicted pose and the Jacobians F and W.

    raise NotImplementedError("TODO: implement Relative2AbsolutePose")


def Absolute2RelativeXY(robot_pose_abs, landmark_position_abs):
    """Converts a landmark's position from the absolute frame of reference to
    the robot's frame of reference, i.e the position of the landmarks as
    measured by the robot.

    :param robot_pose_abs: pose of the robot in the absolute frame of
    reference [x, y, theta]. 3x1 array
    :type robot_pose_abs: np.array
    :param landmark_position_abs: position of the landmark in the absolute
    frame of reference [x, y]. 2x1 array
    :type landmark_position_abs: np.array
    :return: position of the landmark in the to the robot's frame of reference
    [x, y], and the Jacobians of the measurement model with respect to the robot
    pose and the landmark
    :rtype: tuple
    """
    assert robot_pose_abs.shape == (3, 1)
    assert landmark_position_abs.shape == (2, 1)

    # TODO: implement the measurement model for EKF update.
    #       1) compute the expected relative position of the landmark in the
    #          robot frame.
    #       2) compute the Jacobian H = dh/dx of the measurement model with
    #          respect to the robot pose.
    #       3) compute the Jacobian J = dh/dl of the measurement model with
    #          respect to the landmark position.
    #       4) return the expected measurement and the Jacobians H and J.

    raise NotImplementedError("TODO: implement Absolute2RelativeXY")


def Relative2AbsoluteXY(robot_pose_abs, landmark_position_rel):
    """
    Convert's a landmark's position from the robot's frame of reference to the absolute frame of reference
    :param robot_pose_abs: pose of the robot in the the absolute frame of reference [x, y, theta]
    :param landmark_position_rel: position of the landmark in the robot's frame of reference [x, y]
    :return : [position of the landmark in the absolute frame of reference [x, y], G1, G2]
    """
    assert robot_pose_abs.shape == (3, 1)
    assert landmark_position_rel.shape == (2, 1)

    # TODO: implement the inverse measurement model for landmark initialization.
    #       1) compute the absolute landmark position from the robot pose and
    #          relative landmark coordinates.
    #       2) compute the Jacobian G1 = dL/dx of the landmark position with
    #          respect to the robot pose.
    #       3) compute the Jacobian G2 = dL/dl of the landmark position with
    #          respect to the relative measurement.
    #       4) return the absolute landmark position and the Jacobians G1 and G2.

    raise NotImplementedError("TODO: implement Relative2AbsoluteXY")


def RelativeLandmarkPositions(landmark_position_abs, next_landmark_position_abs):
    """
    Given two input landmark positions in the absolute frame of reference, computes the relative position of the
    next landmark with respect to the current landmark
    :param landmark_position_abs: position of the current landmark in the absolute reference frame [x, y]
    :param next_landmark_position_abs: position of the next landmark in the absolute reference frame [x, y]
    :return : relative position of the next landmark with respect to the current landmark's position [dx, dy]
    """
    assert landmark_position_abs.shape == (2, 1)
    assert next_landmark_position_abs.shape == (2, 1)

    # This function does not need any changes

    # label is in position [0], hence use positions [1] and [2]
    x1 = float(landmark_position_abs[1])
    y1 = float(landmark_position_abs[2])
    x2 = float(next_landmark_position_abs[1])
    y2 = float(next_landmark_position_abs[2])

    # Calculate the difference of position in world frame
    diff = [x2 - x1, y2 - y1]

    return diff


def homogenous_transform(R: np.array, t: np.array):
    """
    Given a rotation matrix R and a translation vector t, compute the homogenous transformation matrix H
    that transforms points from the local frame to the global frame.
    """
    assert t.shape == (3, 1)
    assert R.shape == (3, 3)

    # This function does not need any changes

    H = np.eye(4)
    H[:3, :3] = R
    t = np.transpose(t)
    H[:3, 3] = t[:3]

    return H
