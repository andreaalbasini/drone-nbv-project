#!/usr/bin/env python3
import os
import cv2
import numpy as np

# ==========================================
# PARAMETER CONFIGURATION (Insert your measurements)
# ==========================================
CHESSBOARD_SIZE = (7, 5)   # Number of inner corners (Columns, Rows)
SQUARE_SIZE_MM = 80.0     # Real side length of the square in millimetres
IMAGES_PATH = os.path.expanduser('~/ros2_ws_AA/calibration_data/')
# ==========================================

def run_offline_calibration():
    # Preparation of theoretical 3D coordinates (0,0,0), (25,0,0), (50,0,0)...
    objp = np.zeros((CHESSBOARD_SIZE[0] * CHESSBOARD_SIZE[1], 3), np.float32)
    objp[:, :2] = np.mgrid[0:CHESSBOARD_SIZE[0], 0:CHESSBOARD_SIZE[1]].T.reshape(-1, 2)
    objp *= SQUARE_SIZE_MM

    objpoints = []  # Points in real 3D space
    imgpoints = []  # Points on the 2D image plane

    # Retrieve all images saved in the folder
    if not os.path.exists(IMAGES_PATH):
        print(f"Error: folder {IMAGES_PATH} does not exist!")
        return

    images = [os.path.join(IMAGES_PATH, f) for f in os.listdir(IMAGES_PATH) if f.endswith('.jpg')]
    images.sort()

    if len(images) == 0:
        print(f"No .jpg images found in {IMAGES_PATH}")
        return

    print(f"Found {len(images)} images. Starting debug processing...")

    gray = None
    images_used = 0

    for fname in images:
        img = cv2.imread(fname)
        if img is None:
            print(f"Cannot read image: {fname}")
            continue

        gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
        # Search for inner corners
        ret, corners = cv2.findChessboardCorners(gray, CHESSBOARD_SIZE, None)

        if ret:
            objpoints.append(objp)
            
            # Subpixel refinement to increase millimetre precision
            criteria = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 30, 0.001)
            corners2 = cv2.cornerSubPix(gray, corners, (11, 11), (-1, -1), criteria)
            imgpoints.append(corners2)
            
            images_used += 1
            print(f"[OK] Corners found in: {os.path.basename(fname)}")

            # Show debug image to verify the algorithm picked the correct points
            cv2.drawChessboardCorners(img, CHESSBOARD_SIZE, corners2, ret)
            cv2.imshow("Calibration Debug (Press any key to continue)", cv2.resize(img, (800, 600)))
            cv2.waitKey(200) # Show for 200ms before moving to the next one
        else:
            print(f"[FAILED] Chessboard NOT found in: {os.path.basename(fname)} (will be discarded)")

    cv2.destroyAllWindows()

    if images_used < 10:
        print(f"\nWarning: Only {images_used} images were used. At least 10-15 valid photos are recommended.")
    
    if len(objpoints) > 0:
        print("\nComputing calibration matrix...")
        # Calibration
        ret, mtx, dist, rvecs, tvecs = cv2.calibrateCamera(
             objpoints, imgpoints, gray.shape[::-1],
             None, None,
             flags=cv2.CALIB_RATIONAL_MODEL
        )
        
        print("\n=============================================")
        print("          CALIBRATION RESULTS                ")
        print("=============================================")
        print(f"Total reprojection error: {ret:.4f} pixels (optimal if < 0.5)")
        print("\nCamera Intrinsic Matrix (K):")
        print(mtx)
        print("\nDistortion Coefficients (D):")
        print(dist.ravel())
        print("=============================================\n")
        
        # Save data to a human-readable text file
        output_file = os.path.join(IMAGES_PATH, "calibration_results.txt")
        with open(output_file, "w") as f:
            f.write(f"--- SIYI A8 Calibration Results ---\n")
            f.write(f"Reprojection Error: {ret}\n\n")
            f.write(f"Camera Matrix (K):\n{mtx}\n\n")
            f.write(f"Distortion Coefficients (D):\n{dist.ravel()}\n")
            
        print(f"Results saved successfully to:\n{output_file}")
    else:
        print("\nCritical error: None of the processed photos contained a valid chessboard.")

if __name__ == '__main__':
    run_offline_calibration()
