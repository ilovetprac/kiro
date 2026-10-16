#!/usr/bin/env python3
"""
qr_coordinates.py
-----------------
Continuously detect QR codes with an Intel RealSense D435 camera and print
their X, Y, Z coordinates (in meters) relative to the camera, along with the
decoded QR code ID (the text/data encoded in the QR code).

How it works:
  1. Stream aligned color + depth frames from the D435.
  2. Detect & decode QR codes in the color frame (OpenCV QRCodeDetector).
  3. For each QR code, take the center pixel, read its depth value, and use
     the camera intrinsics to deproject that pixel into a real 3D point
     (X, Y, Z) in meters, expressed in the camera's coordinate frame.
  4. Print the data continuously and (optionally) show an annotated preview.

Camera coordinate frame (RealSense convention, meters):
    +X : right
    +Y : down
    +Z : forward (away from the camera, into the scene)

Run:
    python qr_coordinates.py                 # with live preview window
    python qr_coordinates.py --no-display    # headless (print only)
"""

import argparse
import sys
import time

import numpy as np

try:
    import cv2
except ImportError:
    sys.exit(
        "OpenCV is not installed. Install it with:\n"
        "    pip install opencv-python"
    )

try:
    import pyrealsense2 as rs
except ImportError:
    sys.exit(
        "pyrealsense2 is not installed. Install it with:\n"
        "    pip install pyrealsense2\n"
        "(This also requires an Intel RealSense D435 camera connected via USB 3.0.)"
    )


def parse_args():
    parser = argparse.ArgumentParser(
        description="Print X,Y,Z of QR codes relative to a RealSense D435 camera."
    )
    parser.add_argument(
        "--width", type=int, default=640, help="Stream width in pixels (default: 640)."
    )
    parser.add_argument(
        "--height", type=int, default=480, help="Stream height in pixels (default: 480)."
    )
    parser.add_argument(
        "--fps", type=int, default=30, help="Stream frame rate (default: 30)."
    )
    parser.add_argument(
        "--no-display",
        action="store_true",
        help="Run headless (no OpenCV preview window), print only.",
    )
    return parser.parse_args()


def configure_pipeline(width, height, fps):
    """Create and start a RealSense pipeline with color + depth streams."""
    pipeline = rs.pipeline()
    config = rs.config()
    config.enable_stream(rs.stream.depth, width, height, rs.format.z16, fps)
    config.enable_stream(rs.stream.color, width, height, rs.format.bgr8, fps)
    profile = pipeline.start(config)

    # Align depth to color so pixel (x, y) in the color image maps to the
    # same pixel in the depth image.
    align = rs.align(rs.stream.color)
    return pipeline, align, profile


def main():
    args = parse_args()
    show_display = not args.no_display

    print("Starting Intel RealSense D435 pipeline...")
    try:
        pipeline, align, profile = configure_pipeline(args.width, args.height, args.fps)
    except RuntimeError as exc:
        sys.exit(
            f"Failed to start the RealSense camera: {exc}\n"
            "Make sure a D435 is connected to a USB 3.0 port and no other "
            "program is using it."
        )

    # Depth scale converts raw depth units to meters.
    depth_sensor = profile.get_device().first_depth_sensor()
    depth_scale = depth_sensor.get_depth_scale()
    print(f"Depth scale: {depth_scale} meters/unit")

    qr_detector = cv2.QRCodeDetector()

    print("\nReading frames. Point a QR code at the camera.")
    print("Press Ctrl+C in the terminal (or 'q' in the window) to quit.\n")

    try:
        while True:
            # Wait for a coherent pair of frames: depth + color
            frames = pipeline.wait_for_frames()
            aligned = align.process(frames)
            depth_frame = aligned.get_depth_frame()
            color_frame = aligned.get_color_frame()

            if not depth_frame or not color_frame:
                continue

            # Camera intrinsics of the (aligned) color stream, needed to
            # deproject a pixel + depth into a real 3D point.
            intrinsics = (
                color_frame.profile.as_video_stream_profile().intrinsics
            )

            color_image = np.asanyarray(color_frame.get_data())

            # Detect and decode every QR code in the frame.
            ok, decoded_info, points, _ = qr_detector.detectAndDecodeMulti(color_image)

            if ok and points is not None:
                for qr_id, corner_pts in zip(decoded_info, points):
                    corner_pts = corner_pts.astype(int)

                    # Center pixel of the QR code.
                    cx = int(np.mean(corner_pts[:, 0]))
                    cy = int(np.mean(corner_pts[:, 1]))

                    # Clamp to valid image bounds.
                    cx = max(0, min(cx, args.width - 1))
                    cy = max(0, min(cy, args.height - 1))

                    # Depth at the center pixel, in meters.
                    depth_m = depth_frame.get_distance(cx, cy)

                    label = qr_id if qr_id else "(unreadable)"

                    if depth_m <= 0:
                        # No valid depth at that pixel (too close/far/reflective).
                        print(
                            f"QR ID: {label!r:20} | X,Y,Z: no valid depth at "
                            f"pixel ({cx}, {cy})"
                        )
                        xyz = None
                    else:
                        # Deproject pixel -> 3D point (meters) in camera frame.
                        x, y, z = rs.rs2_deproject_pixel_to_point(
                            intrinsics, [cx, cy], depth_m
                        )
                        print(
                            f"QR ID: {label!r:20} | "
                            f"X: {x:+.3f} m  Y: {y:+.3f} m  Z: {z:+.3f} m"
                        )
                        xyz = (x, y, z)

                    # Draw annotations on the preview window.
                    if show_display:
                        cv2.polylines(
                            color_image, [corner_pts], True, (0, 255, 0), 2
                        )
                        cv2.circle(color_image, (cx, cy), 4, (0, 0, 255), -1)
                        text = f"ID:{label}"
                        if xyz is not None:
                            text += f" ({xyz[0]:.2f},{xyz[1]:.2f},{xyz[2]:.2f})m"
                        cv2.putText(
                            color_image,
                            text,
                            (corner_pts[0][0], max(0, corner_pts[0][1] - 10)),
                            cv2.FONT_HERSHEY_SIMPLEX,
                            0.5,
                            (0, 255, 0),
                            2,
                        )

            if show_display:
                cv2.imshow("D435 QR Coordinates", color_image)
                if cv2.waitKey(1) & 0xFF == ord("q"):
                    break

    except KeyboardInterrupt:
        print("\nStopping (Ctrl+C received).")
    finally:
        pipeline.stop()
        if show_display:
            cv2.destroyAllWindows()
        print("Camera stopped. Bye.")


if __name__ == "__main__":
    main()
