
import argparse
import numpy as np
import cv2
from pathlib import Path


# Camera intrinsics
K = np.array([(490.5992175, 0.0, 331.76523829),
              (0.0, 490.70353849, 245.05837429),
              (0.0, 0.0, 1.0)])
dist = np.array([(0.17352236, -0.28697994, -0.00061412, 0.00400239, 0.0244542)])

# ICP extrinsic: maps 2D LiDAR points -> 2D camera points in robot frame (x fwd, y left)
icp_result = np.array([[ 0.99945471, -0.03301952, -0.11688005],
                        [ 0.03301952,  0.99945471,  0.00576757],
                        [ 0.,          0.,           1.        ]])

home = Path.home()


def project_and_show(image_path, laser_points_2d, index):
    img = cv2.imread(image_path)

    # Transform selected LiDAR wall points -> camera 2D robot frame using ICP extrinsic
    ones = np.ones((laser_points_2d.shape[0], 1))
    laser_h = np.hstack([laser_points_2d, ones])
    cam_2d = (icp_result @ laser_h.T).T

    x_robot = cam_2d[:, 0]
    y_robot = cam_2d[:, 1]

    # Convert robot frame (x fwd, y left, z up) -> OpenCV camera frame (x right, y down, z fwd)
    x_cv = -y_robot
    y_cv = np.zeros_like(x_cv)  # dz=0: LiDAR at same height as camera
    z_cv = x_robot

    valid = z_cv > 0.01
    x_cv, y_cv, z_cv = x_cv[valid], y_cv[valid], z_cv[valid]

    pts_cam = np.column_stack([x_cv, y_cv, z_cv]).reshape(-1, 1, 3).astype(np.float32)
    pts_img, _ = cv2.projectPoints(pts_cam, np.zeros(3), np.zeros(3), K, dist)
    pts_img = pts_img.reshape(-1, 2).astype(int)

    result = img.copy()
    h, w = result.shape[:2]
    for pt in pts_img:
        if 0 <= pt[0] < w and 0 <= pt[1] < h:
            cv2.circle(result, tuple(pt), 5, (0, 0, 255), -1)

    cv2.imshow(f'Scan {index} | LiDAR wall points (red) on image', result)
    cv2.waitKey(0)
    cv2.destroyAllWindows()


def main():
    parser = argparse.ArgumentParser(description="Verify extrinsic calibration by projecting selected LiDAR wall points onto each image.")
    parser.add_argument("--points", default=str(home / 'selected_lidar_wall_points.npy'),
                        help="Path to saved per-scan LiDAR wall points .npy file.")
    parser.add_argument("--images", default=str(home / 'calibration_image_paths.npy'),
                        help="Path to saved calibration image paths .npy file.")
    args = parser.parse_args()

    laser_points_per_scan = np.load(args.points, allow_pickle=True)
    image_paths = np.load(args.images, allow_pickle=True)

    assert len(laser_points_per_scan) == len(image_paths), "Mismatch between scans and images."
    print(f"Showing {len(image_paths)} image/scan pairs. Press any key to advance.")

    for i, (image_path, laser_points_2d) in enumerate(zip(image_paths, laser_points_per_scan)):
        print(f"Image {i}: {image_path} — {len(laser_points_2d)} wall points")
        project_and_show(image_path, laser_points_2d, i)


if __name__ == '__main__':
    main()
