#!/usr/bin/env python3
"""
Generate config/slam_viz.rviz from the parameters below.

Usage:
    python3 scripts/generate_rviz_config.py

Edit the parameters in the "User-configurable" block, then re-run to
regenerate the RViz config. Rebuild the package afterwards:
    colcon build --packages-select turtlebot_landmark_slam
"""

from pathlib import Path

# ── User-configurable parameters ──────────────────────────────────────────────

BACKGROUND_COLOR    = "48; 48; 48"       # dark grey
GRID_COLOR          = "160; 160; 164"
GRID_CELL_SIZE      = 1
GRID_COUNT          = 20

# EKF estimated pose trail (blue axes)
EKF_AXES_KEEP       = 5                  # number of recent poses to display
EKF_AXES_LENGTH     = 0.3
EKF_AXES_RADIUS     = 0.03
EKF_AXES_ALPHA      = 1.0
EKF_COV_COLOR       = "204; 51; 204"     # purple position covariance ellipse
EKF_COV_ALPHA       = 0.8
EKF_COV_ORI_COLOR   = "255; 255; 127"    # yellow orientation covariance

# Ground truth pose trail (green axes)
GT_AXES_KEEP        = 5
GT_AXES_LENGTH      = 0.2
GT_AXES_RADIUS      = 0.02
GT_AXES_ALPHA       = 0.7

# LiDAR scan points
LIDAR_COLOR         = "0; 255; 0"        # green
LIDAR_POINT_SIZE    = 0.03               # metres

# ──────────────────────────────────────────────────────────────────────────────

CONFIG_PATH = Path(__file__).resolve().parent.parent / "config" / "slam_viz.rviz"

YAML = f"""\
Panels:
  - Class: rviz_common/Displays
    Help Height: 78
    Name: Displays
    Property Tree Widget:
      Expanded:
        - /Global Options1
        - /EKF Pose1
        - /EKF Pose1/Covariance1
        - /Ground Truth Pose1
        - /Landmark Estimates1
      Splitter Ratio: 0.5
    Tree Height: 600
  - Class: rviz_common/Views
    Name: Views

Visualization Manager:
  Class: ""
  Displays:
    - Class: rviz_default_plugins/Grid
      Name: Grid
      Value: true
      Cell Size: {GRID_CELL_SIZE}
      Color: {GRID_COLOR}
      Plane: XY
      Plane Cell Count: {GRID_COUNT}

    - Class: rviz_default_plugins/Odometry
      Name: EKF Pose
      Topic:
        Depth: 5
        Durability Policy: Volatile
        Filter size: 10
        History Policy: Keep Last
        Reliability Policy: Reliable
        Value: /ekf/odom
      Value: true
      Shape:
        Alpha: {EKF_AXES_ALPHA}
        Axes Length: {EKF_AXES_LENGTH}
        Axes Radius: {EKF_AXES_RADIUS}
        Value: Axes
      Keep: {EKF_AXES_KEEP}
      Covariance:
        Value: true
        Orientation:
          Alpha: 0.5
          Color: {EKF_COV_ORI_COLOR}
          Color Style: Unique
          Value: true
        Position:
          Alpha: {EKF_COV_ALPHA}
          Color: {EKF_COV_COLOR}
          Value: true

    - Class: rviz_default_plugins/Odometry
      Name: Ground Truth Pose
      Topic:
        Depth: 5
        Durability Policy: Volatile
        Filter size: 10
        History Policy: Keep Last
        Reliability Policy: Reliable
        Value: /odom
      Value: true
      Shape:
        Alpha: {GT_AXES_ALPHA}
        Axes Length: {GT_AXES_LENGTH}
        Axes Radius: {GT_AXES_RADIUS}
        Value: Axes
      Keep: {GT_AXES_KEEP}
      Covariance:
        Value: false

    - Class: rviz_default_plugins/MarkerArray
      Name: Landmark Estimates
      Topic:
        Depth: 5
        Durability Policy: Volatile
        History Policy: Keep Last
        Reliability Policy: Reliable
        Value: /ekf/map
      Value: true

    - Class: rviz_default_plugins/RobotModel
      Name: Robot Model
      Value: true
      Alpha: 1

    - Class: rviz_default_plugins/TF
      Name: TF
      Value: false
      Marker Scale: 0.5
      Show Arrows: true
      Show Axes: true
      Show Names: false

    - Class: rviz_default_plugins/LaserScan
      Name: LiDAR Scan
      Topic:
        Depth: 5
        Durability Policy: Volatile
        History Policy: Keep Last
        Reliability Policy: Reliable
        Value: /scan
      Value: true
      Size (m): {LIDAR_POINT_SIZE}
      Color Transformer: FlatColor
      Color: {LIDAR_COLOR}

    - Class: rviz_default_plugins/Image
      Name: Camera
      Topic:
        Depth: 5
        Durability Policy: Volatile
        History Policy: Keep Last
        Reliability Policy: Reliable
        Value: /camera/image_raw
      Value: true

  Enabled: true
  Global Options:
    Background Color: {BACKGROUND_COLOR}
    Fixed Frame: odom
    Frame Rate: 30
  Name: root
  Tools:
    - Class: rviz_default_plugins/Interact
    - Class: rviz_default_plugins/MoveCamera
  Value: true
  Views:
    Current:
      Class: rviz_default_plugins/TopDownOrtho
      Name: Current View
      Near Clip Distance: 0.009999999776482582
      Scale: 100
      Target Frame: <Fixed Frame>
      Value: TopDownOrtho (rviz_default_plugins)
      X: 0
      Y: 0
"""

CONFIG_PATH.write_text(YAML)
print(f"Written to {CONFIG_PATH}")
