"""
Camera Calibration Utility
==========================
Captures checkerboard images from the ESP32-S3 camera and computes
the camera intrinsic matrix and distortion coefficients.

This gives far more accurate pose estimation than the FOV-based estimate
(typically reduces z-depth error from ~15% to ~1-2%).

Instructions:
  1. Print a checkerboard pattern (e.g., 9x6 inner corners, 25mm squares).
     Free PDF: https://calib.io/pages/camera-calibration-pattern-generator
     OR search "checkerboard calibration pattern 9x6 PDF" and print at 100% scale.
     Stick the printout flat on a hard surface (cardboard, clipboard) - no bends.
  2. Measure one square with a ruler and note the size in mm.
  3. Connect PC to ESP32S3-CAM WiFi, run this script.
  4. Move the board SLOWLY through the camera view - vary angles and distances.
     Aim for 15-25 good captures spread across the whole frame.
  5. Press SPACE to capture (only when green corners appear), then 'c' to calibrate.

Good calibration tips:
  - Cover all corners of the image frame, not just the center
  - Include tilted views (~30-45 degrees on X and Y axes)
  - Include close (~20cm) and far (~60cm) distances
  - RMS reprojection error should be < 1.0 px (ideally < 0.5 px)
  - If RMS > 2.0, discard and recapture with steadier hand

Usage:
  py -3.12 calibrate_camera.py --url http://192.168.4.1/capture
  py -3.12 calibrate_camera.py --url http://192.168.4.1/capture --cols 9 --rows 6 --square-size 25
"""

import argparse
import time

import cv2
import numpy as np
import requests


def fetch_frame(session: requests.Session, url: str, timeout: float = 3.0):
    """Fetch one JPEG frame from /capture endpoint."""
    try:
        r = session.get(url, timeout=timeout)
        if r.status_code == 200 and len(r.content) > 100:
            arr = np.frombuffer(r.content, dtype=np.uint8)
            return cv2.imdecode(arr, cv2.IMREAD_COLOR)
    except Exception:
        pass
    return None


def main():
    parser = argparse.ArgumentParser(description='Camera Calibration Tool')
    parser.add_argument('--url', type=str, default='http://192.168.4.1/capture',
                        help='Camera /capture URL (default: http://192.168.4.1/capture)')
    parser.add_argument('--cols', type=int, default=9,
                        help='Number of inner corners per row (default: 9)')
    parser.add_argument('--rows', type=int, default=6,
                        help='Number of inner corners per column (default: 6)')
    parser.add_argument('--square-size', type=float, default=25.0,
                        help='Checkerboard square size in mm (default: 25)')
    parser.add_argument('--output', type=str, default='calibration.npz',
                        help='Output calibration file (default: calibration.npz)')
    args = parser.parse_args()

    # Make sure we're hitting the /capture endpoint
    url = args.url.rstrip('/')
    if not url.endswith('/capture'):
        url = url.split('/stream')[0].rstrip('/') + '/capture'

    board_size = (args.cols, args.rows)
    square_size = args.square_size

    # Prepare 3D object points for the board (Z=0 plane)
    objp = np.zeros((board_size[0] * board_size[1], 3), np.float32)
    objp[:, :2] = np.mgrid[0:board_size[0], 0:board_size[1]].T.reshape(-1, 2)
    objp *= square_size

    obj_points = []   # 3D board points (same for every capture)
    img_points = []   # 2D detected corner points
    saved_frames = [] # gray frames for undistortion preview

    session = requests.Session()
    session.headers.update({'Connection': 'keep-alive'})

    # Verify connection
    print(f"Connecting to {url} ...")
    frame = fetch_frame(session, url)
    if frame is None:
        print("ERROR: Cannot connect to camera. Check WiFi and URL.")
        return

    print(f"\nCheckerboard: {args.cols}x{args.rows} inner corners, square={args.square_size}mm")
    print("\nControls:")
    print("  SPACE - Capture frame (only when board is detected and shown in green)")
    print("  c     - Compute calibration (needs 10+ captures)")
    print("  u     - Preview undistortion (only after calibration is computed)")
    print("  q     - Quit")
    print("\nTIPS: Slowly tilt the board in X and Y, cover all image corners.")
    print()

    criteria = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 30, 0.001)
    calibration_result = None   # store result for undistort preview

    while True:
        frame = fetch_frame(session, url)
        if frame is None:
            time.sleep(0.1)
            continue

        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)

        # Use faster flags for real-time feedback
        flags = (cv2.CALIB_CB_FAST_CHECK | cv2.CALIB_CB_ADAPTIVE_THRESH
                 | cv2.CALIB_CB_NORMALIZE_IMAGE)
        found, corners = cv2.findChessboardCorners(gray, board_size, flags)

        display = frame.copy()

        if found:
            corners_refined = cv2.cornerSubPix(
                gray, corners, (11, 11), (-1, -1), criteria)
            cv2.drawChessboardCorners(display, board_size, corners_refined, found)
            cv2.putText(display, f"FOUND - SPACE to capture  ({len(obj_points)} saved)",
                        (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2)
        else:
            cv2.putText(display, f"No board detected  ({len(obj_points)} saved)",
                        (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 100, 255), 2)

        # Show RMS if calibrated
        if calibration_result is not None:
            cv2.putText(display, f"RMS: {calibration_result:.4f} px",
                        (10, display.shape[0] - 10),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.55, (0, 255, 200), 2)

        cv2.imshow("Calibration - ESP32-S3 Cam", display)
        key = cv2.waitKey(1) & 0xFF

        if key == ord(' ') and found:
            obj_points.append(objp.copy())
            img_points.append(corners_refined)
            saved_frames.append(gray.copy())
            print(f"  Captured #{len(obj_points):2d}  "
                  f"(board at approx center {tuple(corners_refined.mean(axis=0).astype(int).ravel())})")

        elif key == ord('c'):
            n = len(obj_points)
            if n < 10:
                print(f"  Need at least 10 captures (have {n}). Keep going!")
                continue

            print(f"\n  Running calibration on {n} captures...")
            h, w = gray.shape
            rms, camera_matrix, dist_coeffs, rvecs, tvecs = cv2.calibrateCamera(
                obj_points, img_points, (w, h), None, None
            )
            calibration_result = rms

            print(f"\n  {'='*50}")
            print(f"  RMS reprojection error: {rms:.4f} px  ", end='')
            if rms < 0.5:
                print("(EXCELLENT)")
            elif rms < 1.0:
                print("(GOOD)")
            elif rms < 2.0:
                print("(ACCEPTABLE - consider more captures)")
            else:
                print("(POOR - too blurry/bent board, recapture)")

            print(f"\n  Camera matrix (fx, fy, cx, cy):")
            print(f"    fx={camera_matrix[0,0]:.2f}  fy={camera_matrix[1,1]:.2f}")
            print(f"    cx={camera_matrix[0,2]:.2f}  cy={camera_matrix[1,2]:.2f}")
            print(f"\n  Distortion: {dist_coeffs.ravel().round(6)}")

            np.savez(args.output,
                     camera_matrix=camera_matrix,
                     dist_coeffs=dist_coeffs,
                     rms_error=rms,
                     image_size=np.array([w, h]))
            print(f"\n  Saved to '{args.output}'")
            print(f"  Run vision app: py -3.12 vision_processor.py "
                  f"--url http://192.168.4.1 --calib {args.output} "
                  f"--box-width <W> --box-height <H>")

        elif key == ord('u') and calibration_result is not None and saved_frames:
            # Show undistortion on last captured frame
            h, w = saved_frames[-1].shape
            rms, camera_matrix, dist_coeffs, _, _ = cv2.calibrateCamera(
                obj_points, img_points, (w, h), None, None)
            undistorted = cv2.undistort(
                cv2.cvtColor(saved_frames[-1], cv2.COLOR_GRAY2BGR),
                camera_matrix, dist_coeffs)
            orig_color = cv2.cvtColor(saved_frames[-1], cv2.COLOR_GRAY2BGR)
            compare = np.hstack([orig_color, undistorted])
            cv2.putText(compare, "ORIGINAL", (10, 25),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 0, 255), 2)
            cv2.putText(compare, "UNDISTORTED", (w + 10, 25),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)
            cv2.imshow("Undistortion Preview (any key to close)", compare)
            cv2.waitKey(0)
            cv2.destroyWindow("Undistortion Preview (any key to close)")

        elif key == ord('q'):
            break

    session.close()
    cv2.destroyAllWindows()
    print("\nDone.")


if __name__ == '__main__':
    main()
