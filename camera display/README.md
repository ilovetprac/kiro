# Camera Display — QR Code 3D Coordinates (Intel RealSense D435)

Continuously detects QR codes with an Intel RealSense **D435** camera and prints
each QR code's **X, Y, Z position (in meters) relative to the camera**, along
with the **QR code ID** (the data encoded in the QR code).

## What it does

1. Streams aligned **color + depth** frames from the D435.
2. Detects and decodes QR codes in the color image.
3. Takes each QR code's center pixel, reads the depth there, and deprojects it
   into a real 3D point `(X, Y, Z)` in meters using the camera intrinsics.
4. Prints the result continuously, e.g.:

   ```
   QR ID: 'box-42'            | X: +0.051 m  Y: -0.012 m  Z: +0.734 m
   ```

### Coordinate frame (RealSense convention, meters)

| Axis | Direction          |
|------|--------------------|
| +X   | right              |
| +Y   | down               |
| +Z   | forward (away from camera, into the scene / depth) |

The origin `(0, 0, 0)` is the camera itself.

## Requirements

- An **Intel RealSense D435** connected to a **USB 3.0** port.
- Python 3.7+.
- Python packages (see `requirements.txt`):
  - `pyrealsense2` — the Intel RealSense SDK Python bindings
  - `opencv-python` — QR detection/decoding + preview window
  - `numpy`

> `pyrealsense2` only works on platforms where Intel ships wheels (commonly
> Linux x86_64 and Windows). On some systems you may need to install the
> [Intel RealSense SDK (librealsense)](https://github.com/IntelRealSense/librealsense)
> first.

## Setup

```bash
cd "camera display"
python -m venv .venv
# Linux/macOS:
source .venv/bin/activate
# Windows:
# .venv\Scripts\activate

pip install -r requirements.txt
```

## Run

```bash
# With a live annotated preview window:
python qr_coordinates.py

# Headless (terminal output only, no window):
python qr_coordinates.py --no-display

# Custom resolution / frame rate:
python qr_coordinates.py --width 1280 --height 720 --fps 30
```

Point a QR code at the camera. Coordinates print continuously.
Stop with **Ctrl+C** in the terminal, or press **q** in the preview window.

## Options

| Flag           | Default | Description                              |
|----------------|---------|------------------------------------------|
| `--width`      | 640     | Stream width in pixels                   |
| `--height`     | 480     | Stream height in pixels                  |
| `--fps`        | 30      | Stream frame rate                        |
| `--no-display` | off     | Run headless (print only, no window)     |

## Troubleshooting

- **"Failed to start the RealSense camera"** — Check the USB 3.0 connection and
  make sure no other app (e.g. RealSense Viewer) is using the camera.
- **"no valid depth at pixel ..."** — The QR code is too close, too far, highly
  reflective, or out of the depth sensor's range. Move it 0.3–3 m from the camera.
- **Multiple QR codes** are supported — each is printed on its own line every frame.
- The decoded **QR ID** is whatever text is encoded in the QR code (number, string,
  URL, etc.). If you need strictly numeric IDs, encode numbers in your QR codes.
