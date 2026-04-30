
import os
import argparse
import numpy as np
import cv2


def load_images_from_folder(folder):
    """Load all valid images from a directory and return them with their filenames."""
    images = []
    filenames = []
    for filename in os.listdir(folder):
        img = cv2.imread(os.path.join(folder,filename))
        if img is not None:
            images.append(img)
            filenames.append(filename)
    return images, filenames


def main():
    parser = argparse.ArgumentParser(description="Calibrate image intrinsics from a collection of checkerboard images.")
    parser.add_argument("image_dir", help="Image directory.")
    args = parser.parse_args()

    image_dir = args.image_dir

    # termination criteria
    criteria = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 30, 0.001)

    # Object Points (objp) refers to the known points on an object, in this case a checkerboard
    # The known points are the checkerboard vertices (inside, excluding the out most corners)
    # Depending on your checkerboard layout, the parameters below willc change
    # Checkerboard: 11 cols × 8 rows of inner corners, 19 mm square size
    checkerboard_width = 11
    checkerboard_height = 8
    checkerboard_size = 0.019
    objp = np.zeros((checkerboard_width*checkerboard_height, 3), np.float32)
    objp[:,:2] = np.mgrid[0:checkerboard_height, 0:checkerboard_width].T.reshape(-1,2)*checkerboard_size

    # Arrays to store object points and image points from all the images.
    objpoints = [] # 3d point in real world space
    imgpoints = [] # 2d points in image plane.
    calibrated_imgs = [] # images where corners were found

    images, filenames = load_images_from_folder(image_dir)

    for img, filename in zip(images, filenames):
        gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
        # Find the chess board corners
        ret, corners = cv2.findChessboardCorners(gray, (checkerboard_height, checkerboard_width), None)
        # If found, add object points, image points (after refining them)
        if ret:
            print(f"PASS: {filename}")
            objpoints.append(objp)
            # Search in the grayscale image for the corners, refined sub pixel coorindates
            corners2 = cv2.cornerSubPix(gray, corners, (5, 5), (-1,-1), criteria)
            imgpoints.append(corners2)
            calibrated_imgs.append(img.copy())
            # Draw and display the corners for introspection - do the corners/vertices drawn back onto the image match with the checkerboard in the camera view?
            cv2.drawChessboardCorners(img, (checkerboard_height, checkerboard_width), corners2, ret)
            cv2.imshow('img', img)
            cv2.waitKey(500)
        else:
            print(f"FAIL: {filename}")
    cv2.destroyAllWindows()

    ret, mtx, dist, rvecs, tvecs = cv2.calibrateCamera(objpoints, imgpoints, gray.shape[::-1], None, None)

    print(mtx)
    print(dist)

    total_error = 0
    for i in range(len(objpoints)):
        imgpoints2, _ = cv2.projectPoints(objpoints[i], rvecs[i], tvecs[i], mtx, dist)
        error = cv2.norm(imgpoints[i], imgpoints2, cv2.NORM_L2) / len(imgpoints2)
        print(f"Image {i}: reprojection error = {error:.4f} px")
        total_error += error

        # Draw detected corners (green) and reprojected points (red)
        vis = calibrated_imgs[i].copy()
        for pt in imgpoints[i]:
            cv2.circle(vis, tuple(pt[0].astype(int)), 5, (0, 255, 0), 1)
        for pt in imgpoints2:
            cv2.circle(vis, tuple(pt[0].astype(int)), 3, (0, 0, 255), -1)
        cv2.imshow(f'Image {i} | green=detected  red=reprojected', vis)
        cv2.waitKey(0)

    cv2.destroyAllWindows()
    print(f"\nMean reprojection error: {total_error / len(objpoints):.4f} px")


if __name__ == '__main__':
    main()