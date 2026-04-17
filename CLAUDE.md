# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

## Project Overview

Landmark-based EKF SLAM system for TurtleBot3, developed for MTRX5700 at the University of Sydney. The system jointly estimates robot pose and landmark positions by fusing motion control commands with cylindrical landmark observations from 2D LIDAR.

## Build & Run

**Build:**
```bash
colcon build
colcon build --packages-select turtlebot_landmark_slam  # single package
```

**Simulation — single launch file (recommended):**
```bash
# Terminal 1: Starts Gazebo, EKF pipeline, RViz2, and trajectory recorder
ros2 launch turtlebot_landmark_slam slam_sim.launch.py

# Terminal 2: Teleoperate robot
ros2 run turtlebot3_teleop teleop_keyboard
```
Ctrl+C on Terminal 1 shuts everything down and saves `slam_trajectory.png` in the current directory.

**Simulation — manual (individual terminals):**
```bash
ros2 launch turtlebot_landmark_slam simulation.launch.py        # Terminal 1
ros2 launch turtlebot_landmark_slam ekf_pipeline.launch.py is_real:=false  # Terminal 2
ros2 run turtlebot3_teleop teleop_keyboard                      # Terminal 3
```

**Real robot:**
```bash
ros2 launch turtlebot_landmark_slam ekf_pipeline.launch.py is_real:=true
```

**RViz2 config** (`config/slam_viz.rviz`): loads estimated trajectory (`/ekf/odom`), ground truth (`/odom`), and landmark markers (`/ekf/map`) in a top-down 2D view.

**Trajectory plotter** (`scripts/plot_trajectory.py`): subscribes to `/ekf/odom`, `/odom`, and `/ekf/map` while running; saves `slam_trajectory.png` on Ctrl+C. Can also be run standalone:
```bash
ros2 run turtlebot_landmark_slam plot_trajectory.py
```

**Save and evaluate map:**
```bash
ros2 run turtlebot_landmark_slam map_writer.py
python3 scripts/evaluate_map.py --solution map_slam.txt --gt <ground_truth_file>.txt
```

**Sourcing** (open a fresh terminal each session):
```bash
source /opt/ros/jazzy/setup.bash && source ~/ekf_ws/install/setup.bash
```

## Architecture

### Data Flow

**Simulation mode:**
```
/odom → odom_to_control_republisher → /control (Twist) → EKF predict()
/odom (pose) + SDF cylinder positions → landmark_publisher_sim → /landmarks → EKF update()
```

**Real robot mode:**
```
/cmd_vel (Twist) → EKF predict()
/scan (LaserScan) → landmarks_circle_detector → /landmarks → EKF update()
```

### Core Modules (`turtlebot_landmark_slam/src/turtlebot_landmark_slam/`)

| Module | Role |
|--------|------|
| `ekf.py` | **Primary student implementation file.** `ExtendedKalmanFilter` class with `predict()` and `update()` methods. State vector: `[x, y, θ, lmk0_x, lmk0_y, ...]` grows as landmarks are discovered. |
| `pipeline.py` | Bridges EKF to ROS 2. Handles control/landmark callbacks, runs predict/update cycles, publishes `/ekf/odom` and `/ekf/map`. Thread-safe with locks. |
| `dataprovider.py` | Converts raw ROS messages to typed `ControlMeasurement`/`LandmarkMeasurement`. `SimulationDataProvider` adds configurable Gaussian noise; skips negligible motion. |
| `landmarks_circle_detector.py` | Detects cylinders in LaserScan. Clusters points → algebraic least-squares circle fit → Levenberg-Marquardt refinement → uncertainty propagation. |
| `utils.py` | Geometry helpers returning poses/coordinates **and their Jacobians**: `Relative2AbsolutePose`, `Relative2AbsoluteXY`, `Absolute2RelativeXY`, `pi2pi`, `RelativeLandmarkPositions`. |
| `types.py` | `LandmarkMeasurement` (x, y, label, 2×2 covariance) and `ControlMeasurement` (dx, dy, dθ, 3×3 covariance). |

### Scripts (`turtlebot_landmark_slam/scripts/`)

| Script | Purpose |
|--------|---------|
| `ekf_pipeline_node.py` | Entry point; instantiates `EkfPipelineNode → Pipeline → ExtendedKalmanFilter`. |
| `odom_to_control_republisher.py` | Sim-only. Extracts twist from `/odom`, republishes as `/control` at ~50 Hz. |
| `landmark_publisher_sim.py` | Sim-only. Reads ground-truth cylinder positions from SDF, transforms to robot frame, publishes `LandmarksMsg` at 2 Hz. |
| `map_writer.py` | Subscribes to `/ekf/map` (MarkerArray), writes `map_slam.txt` as `POINT2D <id> <x> <y>`. |
| `evaluate_map.py` | Off-line evaluation against ground truth; reports relative landmark position error. |

### Custom Messages (`landmarks_msg/msg/`)

- `LandmarkMsg`: `label` (uint64), `x`, `y`, `s_x`, `s_y` (float32 covariances)
- `LandmarksMsg`: `LandmarkMsg[]` array published per scan

## EKF Implementation Notes

The EKF predict/update logic lives in `ekf.py`. The `utils.py` helpers are designed specifically for use in the EKF equations:

**Numpy mutable array warning:** The state vector and covariance matrix are numpy arrays passed by reference. Always use `np.copyto` (or explicit copies) when updating them to avoid unintended memory-sharing bugs.

- **Predict step**: Use `Relative2AbsolutePose()` — returns new pose and Jacobians F (w.r.t. state) and W (w.r.t. control noise).
- **Update step — new landmark**: Use `Relative2AbsoluteXY()` — returns landmark world position and Jacobians G1 (w.r.t. robot pose), G2 (w.r.t. measurement). Append to state and augment covariance.
- **Update step — known landmark**: Use `Absolute2RelativeXY()` — returns predicted measurement and Jacobians H (w.r.t. full state), J (w.r.t. measurement noise). Standard EKF update equations apply.

## Key Noise Parameters

Configured in `ekf_pipeline.launch.py` and `dataprovider.py`:

| Parameter | Default | Description |
|-----------|---------|-------------|
| `std_dev_linear_vel` | 0.01 | Control linear velocity noise (m/s) |
| `std_dev_angular_vel` | ~0.087 rad | Control angular velocity noise (5°) |
| `std_dev_landmark_x/y` | 0.01 m² | Landmark measurement noise |

## Notebooks

`notebooks/` contains Jupyter tutorials (`MTRX5700_2026_EKF_Landmark_SLAM_Tutorial.ipynb`, `MTRX5700_2026_Simple_Kalman_Filter.ipynb`). The directory has a `COLCON_IGNORE` file so colcon skips it during builds.
