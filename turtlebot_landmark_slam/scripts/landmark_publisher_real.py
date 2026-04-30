#!/usr/bin/env python3
"""
Real-robot landmark publisher for EKF-SLAM.

Fuses 2D LIDAR circle detection with ArUco tag identification to produce
labelled LandmarksMsg on ~/landmarks (remapped to /landmarks by the launch file).

Pipeline per synchronised (scan, image) pair:
  1. LaserScan -> Nx2 Cartesian points
  2. extract_circular_objects() -> list of CircleFit (cx, cy in robot/LIDAR frame)
  3. Project each circle centre into camera image via known LIDAR-camera extrinsic
  4. Detect ArUco tags (DICT_4X4_50) in the camera image
  5. Match each projected circle to the nearest ArUco centroid within pixel threshold
  6. Publish only matched (labelled) landmarks — unmatched circles are dropped

Subscribes:
  /scan              (sensor_msgs/LaserScan)
  /camera/image_raw  (sensor_msgs/Image)

Publishes:
  ~/landmarks        (landmarks_msg/LandmarksMsg)  -> remapped to /landmarks
  ~/debug_image      (sensor_msgs/Image)            overlay for visual verification
"""

import time
import numpy as np
import cv2
import cv_bridge
import message_filters

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan, Image
from visualization_msgs.msg import Marker, MarkerArray

from landmarks_msg.msg import LandmarkMsg, LandmarksMsg
from turtlebot_landmark_slam.landmarks_circle_detector import extract_circular_objects


# ---------------------------------------------------------------------------
# Calibration constants
# Source: calibration/verify_extrinsic.py (originally from MTRX5700 Assignment 2)
# Redo with cam_lidar_2d_icp.py + cam_intrinsic.py if the robot/mount changes.
# ---------------------------------------------------------------------------

# Camera intrinsic matrix (focal lengths fx=fy≈490 px, principal point ≈(332,245))
_K = np.array([
    [490.5992175,   0.0,         331.76523829],
    [  0.0,         490.70353849, 245.05837429],
    [  0.0,           0.0,          1.0       ],
], dtype=np.float64)

# Lens distortion coefficients [k1, k2, p1, p2, k3]
_DIST = np.array(
    [[0.17352236, -0.28697994, -0.00061412, 0.00400239, 0.0244542]],
    dtype=np.float64,
)

# 2D ICP extrinsic: maps a LIDAR point (x, y) in robot frame to the camera
# origin's coordinate frame (also robot frame: x forward, y left).
# Rotation ~1.9 deg, translation (-117 mm, +6 mm).
_T_LIDAR_CAM = np.array([
    [ 0.99945471, -0.03301952, -0.11688005],
    [ 0.03301952,  0.99945471,  0.00576757],
    [ 0.0,          0.0,          1.0      ],
], dtype=np.float64)

# ArUco dictionary and detector parameters
_ARUCO_DICT = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_50)
try:
    _ARUCO_PARAMS = cv2.aruco.DetectorParameters_create()
except AttributeError:
    _ARUCO_PARAMS = cv2.aruco.DetectorParameters()


# ---------------------------------------------------------------------------
# Node
# ---------------------------------------------------------------------------

class LandmarkPublisherReal(Node):

    def __init__(self) -> None:
        super().__init__("landmark_publisher_real")

        # LIDAR detection params — tunable from launch file without code changes
        self._dist_thresh     = float(self.declare_parameter("distance_threshold",  0.05).value)
        self._max_mse         = float(self.declare_parameter("max_mse",             1e-5).value)
        self._max_radius      = float(self.declare_parameter("max_radius",          0.10).value)
        self._min_radius      = float(self.declare_parameter("min_radius",          0.05).value)
        self._min_points      = int(  self.declare_parameter("min_points",          4   ).value)
        self._aruco_px_thresh = float(self.declare_parameter("aruco_pixel_threshold", 50.0).value)

        self._bridge = cv_bridge.CvBridge()
        self._min_publish_interval = 0.5  # seconds → 2 Hz max
        self._last_publish_time = 0.0

        # Synchronised scan + image (50 ms slop, queue 5)
        scan_sub  = message_filters.Subscriber(self, LaserScan, "/scan")
        image_sub = message_filters.Subscriber(self, Image,     "/camera/image_raw")
        self._sync = message_filters.ApproximateTimeSynchronizer(
            [scan_sub, image_sub], queue_size=5, slop=0.05
        )
        self._sync.registerCallback(self._sync_cb)

        self._landmarks_pub      = self.create_publisher(LandmarksMsg, "~/landmarks",         10)
        self._debug_image_pub    = self.create_publisher(Image,         "~/debug_image",        1)
        self._debug_circles_pub  = self.create_publisher(MarkerArray,   "~/detected_circles",   1)

        self.get_logger().info(
            f"landmark_publisher_real started | "
            f"dist_thresh={self._dist_thresh} max_mse={self._max_mse} "
            f"max_radius={self._max_radius} aruco_px_thresh={self._aruco_px_thresh}"
        )

    # ------------------------------------------------------------------
    # Helpers
    # ------------------------------------------------------------------

    @staticmethod
    def _laserscan_to_points(msg: LaserScan) -> np.ndarray:
        """Convert a LaserScan message to an Nx2 array of (x, y) in robot frame."""
        angles = msg.angle_min + np.arange(len(msg.ranges)) * msg.angle_increment
        ranges = np.array(msg.ranges, dtype=np.float64)
        valid = np.isfinite(ranges) & (ranges >= msg.range_min) & (ranges <= msg.range_max)
        if not np.any(valid):
            return np.empty((0, 2))
        return np.column_stack([
            ranges[valid] * np.cos(angles[valid]),
            ranges[valid] * np.sin(angles[valid]),
        ])

    @staticmethod
    def _project_lidar_to_pixel(cx: float, cy: float):
        """Project a LIDAR point (cx, cy) in robot frame to image pixel (u, v).

        Returns (u, v) as floats, or None if the point is behind the camera.
        """
        # Apply 2D ICP extrinsic: LIDAR robot frame -> camera origin robot frame
        cam = _T_LIDAR_CAM @ np.array([cx, cy, 1.0])

        # Convert robot frame (x fwd, y left) -> OpenCV camera frame (x right, y down, z fwd)
        # y_cv = 0: LIDAR plane and camera assumed at the same height
        x_cv, y_cv, z_cv = -cam[1], 0.0, cam[0]

        if z_cv <= 0.01:
            return None

        pts, _ = cv2.projectPoints(
            np.array([[[x_cv, y_cv, z_cv]]], dtype=np.float64),
            np.zeros(3), np.zeros(3), _K, _DIST,
        )
        u, v = pts[0, 0]
        return float(u), float(v)

    @staticmethod
    def _aruco_centroids(img: np.ndarray) -> list:
        """Detect ArUco tags and return [(tag_id, u, v), ...] centroids in pixels."""
        gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
        corners, ids, _ = cv2.aruco.detectMarkers(gray, _ARUCO_DICT, parameters=_ARUCO_PARAMS)
        if ids is None:
            return []
        result = []
        for tag_corners, tag_id in zip(corners, ids.flatten()):
            pts = tag_corners[0]  # shape (4, 2)
            result.append((int(tag_id), float(np.mean(pts[:, 0])), float(np.mean(pts[:, 1]))))
        return result

    # ------------------------------------------------------------------
    # Main callback
    # ------------------------------------------------------------------

    def _sync_cb(self, scan_msg: LaserScan, img_msg: Image) -> None:
        now = time.monotonic()
        if now - self._last_publish_time < self._min_publish_interval:
            return
        self._last_publish_time = now

        scan_points = self._laserscan_to_points(scan_msg)
        if len(scan_points) < self._min_points:
            return

        circles = extract_circular_objects(
            scan_points,
            distance_threshold=self._dist_thresh,
            max_mse=self._max_mse,
            max_radius=self._max_radius,
            min_radius=self._min_radius,
            min_points=self._min_points,
        )

        # Visual debug: publish detected circles as markers in the LIDAR frame
        self._publish_circle_markers(scan_msg.header.frame_id, circles)

        if circles:
            radii_str = ", ".join(f"r={c.radius:.3f}m@({c.center[0]:+.2f},{c.center[1]:+.2f})"
                                   for c in circles)
            self.get_logger().info(f"detected {len(circles)} circle(s): {radii_str}")
        else:
            self.get_logger().info(f"detected 0 circles (scan_pts={len(scan_points)})")

        img_bgr = self._bridge.imgmsg_to_cv2(img_msg, desired_encoding="bgr8")
        img_h, img_w = img_bgr.shape[:2]

        # Project each detected circle centre into image pixel space.
        # Reject projections outside the image (with margin) — those points are
        # outside the camera FOV and the distortion model produces garbage there.
        projections = []  # [(fit, (u, v)), ...]
        margin = 100  # px — allows tags partially off the image edge
        for fit in circles:
            cx, cy = fit.center
            pixel = self._project_lidar_to_pixel(cx, cy)
            if pixel is None:
                continue
            u, v = pixel
            if -margin <= u <= img_w + margin and -margin <= v <= img_h + margin:
                projections.append((fit, pixel))

        tag_list = self._aruco_centroids(img_bgr)  # [(tag_id, u, v), ...]

        # Build candidate (circle, tag) matches within pixel threshold.
        # Horizontal distance only: the projected LIDAR centre's vertical (v)
        # position depends on where the LIDAR plane meets the image, while the
        # tag could be mounted anywhere on the cylinder's height. Only the
        # column (u) tells us if they're the same cylinder.
        candidates = []  # [(distance, circle_idx, tag_idx), ...]
        for ci, (_, (pu, _pv)) in enumerate(projections):
            for ti, (_, tu, _tv) in enumerate(tag_list):
                d = abs(pu - tu)
                if d < self._aruco_px_thresh:
                    candidates.append((d, ci, ti))

        # Greedy bipartite assignment: closest first, each circle/tag used at most once.
        # Prevents two circles from being labelled with the same tag (which would corrupt
        # the EKF state by stacking is_new=True followed by is_new=False on the same label).
        candidates.sort()
        used_circles, used_tags = set(), set()
        landmarks_msg = LandmarksMsg()
        matched_pairs = []  # [(fit, (pu, pv), tag_id), ...] for debug overlay

        for _, ci, ti in candidates:
            if ci in used_circles or ti in used_tags:
                continue
            used_circles.add(ci)
            used_tags.add(ti)

            fit, (pu, pv) = projections[ci]
            tag_id = tag_list[ti][0]
            cx, cy = fit.center

            s_x = float(fit.covariance[0, 0])
            s_y = float(fit.covariance[1, 1])
            # Guard against degenerate covariance (singular fit or near-zero)
            if not np.isfinite(s_x) or s_x < 1e-6:
                s_x = 0.01
            if not np.isfinite(s_y) or s_y < 1e-6:
                s_y = 0.01

            lm = LandmarkMsg()
            lm.label = tag_id
            lm.x = float(cx)
            lm.y = float(cy)
            lm.s_x = s_x
            lm.s_y = s_y
            landmarks_msg.landmarks.append(lm)
            matched_pairs.append((fit, (pu, pv), tag_id))

        self._publish_debug(img_bgr, projections, tag_list, matched_pairs)

        # Per-cycle diagnostic: horizontal-pixel distance from each unmatched
        # circle to the nearest tag (matching same metric used above).
        unmatched_info = []
        if tag_list and projections:
            for ci, (_, (pu, _pv)) in enumerate(projections):
                if ci in used_circles:
                    continue
                d_min = min(abs(pu - tu) for _, tu, _tv in tag_list)
                unmatched_info.append(f"{d_min:.0f}px")
        self.get_logger().info(
            f"projected={len(projections)} tags={len(tag_list)} "
            f"matched={len(landmarks_msg.landmarks)} "
            f"unmatched_dists=[{', '.join(unmatched_info)}]"
        )

        if not landmarks_msg.landmarks:
            return

        self._landmarks_pub.publish(landmarks_msg)

    # ------------------------------------------------------------------
    # Debug image
    # ------------------------------------------------------------------

    def _publish_debug(self, img_bgr, projections, tag_list, matched_pairs) -> None:
        if self._debug_image_pub.get_subscription_count() == 0:
            return

        dbg = img_bgr.copy()
        h, w = dbg.shape[:2]

        # Projected LIDAR circle centres — red hollow circles
        for _, (u, v) in projections:
            iu, iv = int(round(u)), int(round(v))
            if 0 <= iu < w and 0 <= iv < h:
                cv2.circle(dbg, (iu, iv), 10, (0, 0, 255), 2)

        # ArUco tag centroids — green filled dots + ID label
        for tag_id, tu, tv in tag_list:
            cv2.circle(dbg, (int(tu), int(tv)), 6, (0, 255, 0), -1)
            cv2.putText(dbg, str(tag_id), (int(tu) + 10, int(tv)),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)

        # Matched pairs — blue line from projected centre to tag centroid
        for _, (pu, pv), tag_id in matched_pairs:
            for tid, tu, tv in tag_list:
                if tid == tag_id:
                    cv2.line(dbg, (int(pu), int(pv)), (int(tu), int(tv)), (255, 0, 0), 1)
                    break

        self._debug_image_pub.publish(
            self._bridge.cv2_to_imgmsg(dbg, encoding="bgr8")
        )

    # ------------------------------------------------------------------
    # Detected circle markers (RViz)
    # ------------------------------------------------------------------

    def _publish_circle_markers(self, frame_id: str, circles: list) -> None:
        """Publish detected LIDAR circles as cylinder markers for RViz."""
        marker_array = MarkerArray()

        # Always include a DELETEALL so previous markers are cleared
        clear = Marker()
        clear.header.frame_id = frame_id
        clear.action = Marker.DELETEALL
        marker_array.markers.append(clear)

        for i, fit in enumerate(circles):
            cx, cy = fit.center
            m = Marker()
            m.header.frame_id = frame_id
            m.id = i
            m.type = Marker.CYLINDER
            m.action = Marker.ADD
            m.pose.position.x = float(cx)
            m.pose.position.y = float(cy)
            m.pose.position.z = 0.0
            m.pose.orientation.w = 1.0
            m.scale.x = 2.0 * float(fit.radius)
            m.scale.y = 2.0 * float(fit.radius)
            m.scale.z = 0.3
            m.color.r = 1.0
            m.color.g = 0.5
            m.color.b = 0.0
            m.color.a = 0.6
            marker_array.markers.append(m)

        self._debug_circles_pub.publish(marker_array)


# ---------------------------------------------------------------------------
# Entry point
# ---------------------------------------------------------------------------

def main(args=None) -> None:
    rclpy.init(args=args)
    node = LandmarkPublisherReal()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
