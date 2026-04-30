---
name: Calibration redo reminder
description: Camera-LIDAR calibration is from a previous assignment and should be redone on this specific robot before/after demo if possible
type: project
---

The camera intrinsics and LIDAR-camera extrinsic in `calibration/` and hardcoded in `landmark_publisher_real.py` were originally produced for MTRX5700 Assignment 2, not this specific robot or mount.

**Why:** Physical camera/LIDAR mount positions may differ between robots. A stale extrinsic will cause LIDAR circle centres to project to the wrong pixel location, degrading ArUco–circle association.

**How to apply:** If lab time permits before or after the demo, re-run `calibration/cam_lidar_2d_icp.py` with fresh checkerboard images and LIDAR scans from this robot, then update the `T_LIDAR_CAM` constant in `landmark_publisher_real.py`. Re-run `calibration/cam_intrinsic.py` too if the camera has changed.
