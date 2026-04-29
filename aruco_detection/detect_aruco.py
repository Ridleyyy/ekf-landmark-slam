"""
Offline ArUco detection pipeline.
Reads ROS2 bag(s), detects ArUco markers on cylinders, estimates distance
via LiDAR, and reports per-bag results.

Unlike sign_detection, the cylinders here are not necessarily red — detection
relies on ArUco corner finding, not HSV masking.

Usage:
    source /opt/ros/jazzy/setup.bash
    venv/bin/python3 aruco_detection/detect_aruco.py <bag_folder> [bag2 ...]
"""

import sys
import math
from pathlib import Path
from collections import Counter

import cv2
import numpy as np
from rclpy.serialization import deserialize_message
import rosbag2_py
from sensor_msgs.msg import Image, LaserScan

PROJECT_ROOT = Path(__file__).resolve().parent

# ── ArUco config ──────────────────────────────────────────────────────────────
ARUCO_DICT   = cv2.aruco.DICT_4X4_50
ARUCO_PARAMS = cv2.aruco.DetectorParameters()

# Minimum marker area (px²) — filters noise and very distant tags
MIN_MARKER_AREA = 200

# ── LiDAR cylinder detection ──────────────────────────────────────────────────
LIDAR_ANGLE_MIN   = -1.2    # rad — ±69° forward cone
LIDAR_ANGLE_MAX   =  1.2
LIDAR_RANGE_MIN   =  0.2    # m
LIDAR_RANGE_MAX   =  3.0    # m
LIDAR_CLUSTER_GAP =  0.04   # m — max inter-point gap within a cluster
CYLINDER_W_MIN    =  0.04   # m — min chord width
CYLINDER_W_MAX    =  0.25   # m — max chord width (slightly wider: no colour filter)
CYLINDER_MIN_PTS  =  5      # minimum points per cluster

# ── Camera–LiDAR extrinsic (from calibration_parameters.txt) ─────────────────
# cam_T_lidar (2D):
# [[0.99912,  0.04203, -0.131],
#  [-0.04203, 0.99912,  0.011],
#  [0,        0,        1   ]]
CAM_R00, CAM_R01 =  0.99911652,  0.04202605
CAM_R10, CAM_R11 = -0.04202605,  0.99911652
CAM_TX  = -0.13067443
CAM_TY  =  0.01125667
CAM_FX  = 483.04449161
CAM_CX  = 307.23210259

# Max pixel error for LiDAR–marker column match (fraction of frame width)
LIDAR_CAM_MAX_PX_ERR_FRAC = 0.20
LIDAR_MIN_DIST = 0.25   # m — ignore very close returns (marker out of frame)

SAVE_EVERY = 15


# ── ArUco detector ────────────────────────────────────────────────────────────

def build_detector():
    dictionary = cv2.aruco.getPredefinedDictionary(ARUCO_DICT)
    return cv2.aruco.ArucoDetector(dictionary, ARUCO_PARAMS)


def detect_markers(detector, frame):
    """
    Detect ArUco markers in frame.
    Returns list of dicts:
        { 'id': int, 'corners': np.ndarray (4,2), 'centre': (cx, cy),
          'area': float, 'bbox': (x, y, w, h) }
    """
    corners_list, ids, _ = detector.detectMarkers(frame)
    results = []
    if ids is None:
        return results
    for corners, marker_id in zip(corners_list, ids.flatten()):
        pts = corners[0]                          # shape (4, 2)
        cx  = float(pts[:, 0].mean())
        cy  = float(pts[:, 1].mean())
        x1, y1 = pts.min(axis=0)
        x2, y2 = pts.max(axis=0)
        area = float(cv2.contourArea(pts.astype(np.int32)))
        if area < MIN_MARKER_AREA:
            continue
        results.append({
            'id':      int(marker_id),
            'corners': pts,
            'centre':  (cx, cy),
            'area':    area,
            'bbox':    (int(x1), int(y1), int(x2 - x1), int(y2 - y1)),
        })
    return results


# ── LiDAR cylinder detection ──────────────────────────────────────────────────

def detect_lidar_cylinders(scan_msg):
    """Return list of (cx, cy, dist) for each cylinder-like cluster."""
    if scan_msg is None:
        return []
    ranges = np.array(scan_msg.ranges, dtype=np.float32)
    a_min  = scan_msg.angle_min
    a_inc  = scan_msg.angle_increment

    points = []
    for i, r in enumerate(ranges):
        angle = a_min + i * a_inc
        if not (LIDAR_ANGLE_MIN <= angle <= LIDAR_ANGLE_MAX):
            continue
        if not (LIDAR_RANGE_MIN <= r <= LIDAR_RANGE_MAX) or not math.isfinite(r):
            continue
        points.append((r * math.cos(angle), r * math.sin(angle)))

    if not points:
        return []

    clusters, current = [], [points[0]]
    for p in points[1:]:
        if math.hypot(p[0] - current[-1][0], p[1] - current[-1][1]) < LIDAR_CLUSTER_GAP:
            current.append(p)
        else:
            clusters.append(current)
            current = [p]
    clusters.append(current)

    cylinders = []
    for cluster in clusters:
        if len(cluster) < CYLINDER_MIN_PTS:
            continue
        xs    = [p[0] for p in cluster]
        ys    = [p[1] for p in cluster]
        width = math.hypot(max(xs) - min(xs), max(ys) - min(ys))
        if not (CYLINDER_W_MIN <= width <= CYLINDER_W_MAX):
            continue
        cx = sum(xs) / len(xs)
        cy = sum(ys) / len(ys)
        cylinders.append((cx, cy, math.hypot(cx, cy)))

    return cylinders


# ── Camera–LiDAR association ──────────────────────────────────────────────────

def lidar_confirms_marker(marker, frame_w, lidar_cyls):
    """
    Return (confirmed, dist).
    confirmed is True if a LiDAR cylinder projects within
    LIDAR_CAM_MAX_PX_ERR_FRAC * frame_w pixels of the marker centre column
    and is at least LIDAR_MIN_DIST away.
    """
    if not lidar_cyls:
        return False, None
    cx_px   = marker['centre'][0]
    max_err = LIDAR_CAM_MAX_PX_ERR_FRAC * frame_w
    best_err, best_dist = float('inf'), None
    for (lx, ly, d) in lidar_cyls:
        x_c = CAM_R00 * lx + CAM_R01 * ly + CAM_TX
        y_c = CAM_R10 * lx + CAM_R11 * ly + CAM_TY
        if x_c <= 0:
            continue
        u   = CAM_FX * (-y_c / x_c) + CAM_CX
        err = abs(u - cx_px)
        if err < best_err:
            best_err, best_dist = err, d
    if best_err <= max_err and best_dist is not None and best_dist >= LIDAR_MIN_DIST:
        return True, best_dist
    return False, None


# ── Bag I/O ───────────────────────────────────────────────────────────────────

def open_bag(bag_path):
    reader = rosbag2_py.SequentialReader()
    p      = Path(bag_path)
    nested = p / p.name
    uri    = str(nested) if (nested / 'metadata.yaml').exists() else str(p)
    reader.open(
        rosbag2_py.StorageOptions(uri=uri, storage_id='mcap'),
        rosbag2_py.ConverterOptions('', ''),
    )
    return reader


def decode_image(msg):
    arr = np.frombuffer(msg.data, dtype=np.uint8).reshape(msg.height, msg.width, -1)
    enc = msg.encoding.lower()
    if enc in ('rgb8', 'rgb'):
        return cv2.cvtColor(arr, cv2.COLOR_RGB2BGR).copy()
    if enc in ('mono8', '8uc1'):
        return cv2.cvtColor(arr, cv2.COLOR_GRAY2BGR).copy()
    return arr.copy()


# ── Per-bag processing ────────────────────────────────────────────────────────

def process_bag(bag_path, detector):
    reader = open_bag(bag_path)
    reader.set_filter(rosbag2_py.StorageFilter(topics=['/camera/image_raw', '/scan']))

    out_dir = Path(bag_path) / 'aruco_output'
    out_dir.mkdir(exist_ok=True)

    total       = 0
    detections  = []   # list of { 'id', 'dist', 'area' } per accepted detection
    latest_scan = None
    lidar_ready = False

    while reader.has_next():
        topic, data, _ = reader.read_next()

        if topic == '/scan':
            latest_scan = deserialize_message(data, LaserScan)
            lidar_ready = True
            continue

        if topic != '/camera/image_raw':
            continue

        frame   = decode_image(deserialize_message(data, Image))
        frame_h, frame_w = frame.shape[:2]
        total  += 1

        lidar_cyls = detect_lidar_cylinders(latest_scan)
        markers    = detect_markers(detector, frame)

        debug = frame.copy()
        cv2.aruco.drawDetectedMarkers(debug, [m['corners'][np.newaxis] for m in markers])

        for marker in markers:
            # ── LiDAR gate ────────────────────────────────────────────────────
            dist = None
            if lidar_ready:
                confirmed, dist = lidar_confirms_marker(marker, frame_w, lidar_cyls)
                if not confirmed:
                    continue

            detections.append({
                'id':   marker['id'],
                'dist': dist,
                'area': marker['area'],
            })

            # Annotate
            x, y, w, h = marker['bbox']
            label = f"ID {marker['id']}"
            if dist is not None:
                label += f"  {dist:.2f}m"
            cv2.rectangle(debug, (x, y), (x + w, y + h), (0, 255, 0), 2)
            cv2.putText(debug, label, (x, max(20, y - 8)),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.55, (0, 255, 0), 2)

        if total % SAVE_EVERY == 0:
            cv2.imwrite(str(out_dir / f'frame_{total:05d}.jpg'), debug)

    return total, detections


# ── Reporting ─────────────────────────────────────────────────────────────────

def report(bag_path, total, detections):
    print(f"\n{'='*55}")
    print(f"  Bag: {bag_path}")
    print(f"  Total frames:    {total}")
    print(f"  Marker detections: {len(detections)}  "
          f"({len(detections)/max(total, 1):.1%})")

    if not detections:
        return detections

    id_counts = Counter(d['id'] for d in detections)
    dists     = [d['dist'] for d in detections if d['dist'] is not None]

    print(f"  Unique IDs seen: {sorted(id_counts)}")
    for mid, count in id_counts.most_common():
        print(f"    ID {mid:3d} : {count:4d}x")

    if dists:
        arr = np.array(dists)
        print(f"  LiDAR distances ({len(arr)} readings):")
        print(f"    Mean {arr.mean():.3f} m  |  Std {arr.std():.3f} m  "
              f"|  Min {arr.min():.3f} m  |  Max {arr.max():.3f} m")

    return detections


# ── Entry point ───────────────────────────────────────────────────────────────

def main():
    if len(sys.argv) < 2:
        print(f"Usage: {sys.argv[0]} <bag_folder> [bag2 ...]")
        sys.exit(1)

    print(f"ArUco dictionary: DICT_4X4_50")
    detector = build_detector()

    all_detections = []

    for bag_path in sys.argv[1:]:
        total, detections = process_bag(bag_path, detector)
        all_detections.extend(report(bag_path, total, detections))

    if len(sys.argv) > 2 and all_detections:
        print(f"\n{'='*55}")
        print(f"  OVERALL across {len(sys.argv)-1} bags")
        print(f"  Total detections: {len(all_detections)}")
        id_counts = Counter(d['id'] for d in all_detections)
        print(f"  IDs: {dict(id_counts.most_common())}")
        print('='*55)


if __name__ == '__main__':
    main()
