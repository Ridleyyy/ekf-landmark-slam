---
name: Camera and LIDAR-camera extrinsic calibration values
description: Hardcoded camera intrinsics and 2D LIDAR-to-camera extrinsic used in landmark_publisher_real.py — copied from MTRX5700 Assignment 2 calibration
type: project
---

Source: `calibration/verify_extrinsic.py` (copied from MTRX5700 Assign 2). Used directly in `landmark_publisher_real.py`.

**Camera intrinsics:**
```
K    = [[490.5992175, 0,           331.76523829],
        [0,           490.70353849, 245.05837429],
        [0,           0,            1.0         ]]
dist = [0.17352236, -0.28697994, -0.00061412, 0.00400239, 0.0244542]
```

**2D LIDAR→camera extrinsic (3×3 ICP result):**
```
T = [[ 0.99945471, -0.03301952, -0.11688005],
     [ 0.03301952,  0.99945471,  0.00576757],
     [ 0.,          0.,           1.        ]]
```
Rotation ~1.9°, translation ~(−117 mm, +6 mm). Maps 2D LIDAR point (x,y) in robot frame → camera position in robot frame.

**Frame conversion for cv2.projectPoints** (from verify_extrinsic.py lines 33–36):
- Robot frame: x forward, y left
- x_cv = -y_robot, y_cv = 0 (height assumed equal), z_cv = x_robot
- Then: `cv2.projectPoints([[x_cv, y_cv, z_cv]], zeros, zeros, K, dist)`

**Why:** y_cv = 0 assumes LIDAR and camera are at same height — acceptable for identifying which image column a cylinder is in, but may cause vertical offset in row if heights differ significantly.
