# Streaming the D435 feed: Pi → mini PC over WiFi (both ROS 2 Humble)

Publish the live D435 camera feed from the **Raspberry Pi** and subscribe +
**process it on the mini PC** over the `rachub2` WiFi network (no internet),
using the normal ROS 2 **publish/subscribe** model.

> **Setup assumption:** BOTH machines run **ROS 2 Humble on Ubuntu 22.04**.
> (The mini PC was reimaged from Jazzy/24.04 to match the Pi — this makes
> networking trivial: same distro = native, reliable pub/sub, no DDS tricks.)

Because images are heavy for WiFi, the Pi still re-publishes the feed as
**JPEG `CompressedImage`** and the mini PC decompresses locally — that keeps the
link light. That's the only "extra" piece; discovery itself is plain ROS 2.

---

## 0. One-time install (DO THIS WHILE YOU STILL HAVE INTERNET)

`rachub2` has no internet, so install everything first on a network that does.

**On BOTH machines (Pi and mini PC — both Humble):**
```bash
sudo apt update
sudo apt install -y \
  ros-humble-image-transport \
  ros-humble-compressed-image-transport \
  ros-humble-rqt-image-view          # optional, quick visual check
```
(The Pi already has the camera stack from earlier bring-up.)

---

## 1. Network: both machines on `rachub2`, same subnet

- Connect the Pi and the mini PC to `rachub2`.
- Confirm they can reach each other (no internet needed, just the LAN):
  ```bash
  ip addr show | grep "inet "      # find each machine's IP
  ping <other-machine-ip>          # must succeed both ways
  ```
  If ping fails, the AP is likely blocking device-to-device traffic
  ("AP/client isolation") — turn that OFF on `rachub2`. ROS 2 discovery cannot
  work without basic reachability.

---

## 2. Same ROS_DOMAIN_ID on both (that's it)

Add to **each machine's** `~/.bashrc` (then open a new terminal):
```bash
export ROS_DOMAIN_ID=7        # any 0..101, but IDENTICAL on both
```
Both machines are Humble, so they share the default DDS (Fast DDS) — no
`RMW_IMPLEMENTATION` needed, no CycloneDDS, no config files. Topics discover
each other automatically.

### If discovery is flaky over WiFi (optional)
WiFi multicast can be unreliable. If topics don't show up, try the Fast DDS
discovery server or simple unicast — but first just confirm ping works and the
domain IDs match; that fixes it the vast majority of the time.

---

## 3. Start the stream on the Pi

```bash
source /opt/ros/humble/setup.bash
source ~/ros2_ws/install/setup.bash
export ROS_DOMAIN_ID=7

ros2 launch qr_coordinates_ros2 camera_stream.launch.py
```

Options:
```bash
# tune JPEG quality for WiFi (lower = lighter):
ros2 launch qr_coordinates_ros2 camera_stream.launch.py jpeg_quality:=25

# stream color instead of IR (nicer to look at; enables the color sensor):
ros2 launch qr_coordinates_ros2 camera_stream.launch.py stream_source:=color

# camera already running? don't start a second driver:
ros2 launch qr_coordinates_ros2 camera_stream.launch.py start_camera:=false
```

Published (on the Pi):
| Topic | Type | Use |
|-------|------|-----|
| `/camera/camera/infra1/image_rect_raw` | `sensor_msgs/Image` | raw (local) |
| `/camera/camera/infra1/image_rect_raw/compressed` | `sensor_msgs/CompressedImage` | **stream this over WiFi** |

(For `stream_source:=color`, swap `infra1/image_rect_raw` → `color/image_raw`.)

---

## 4. Subscribe + PROCESS on the mini PC

```bash
source /opt/ros/humble/setup.bash
export ROS_DOMAIN_ID=7

# confirm the Pi's topics are visible:
ros2 topic list | grep camera
ros2 topic hz /camera/camera/infra1/image_rect_raw/compressed
```

**Quick visual check (optional):**
```bash
ros2 run rqt_image_view rqt_image_view     # pick the /compressed topic
```

**To PROCESS it as a normal image**, decompress locally (only the JPEG crosses
the WiFi; the raw image is reconstructed on the mini PC):
```bash
ros2 run image_transport republish compressed raw \
  --ros-args \
  -r in/compressed:=/camera/camera/infra1/image_rect_raw/compressed \
  -r out:=/d435/infra1
```
Now `/d435/infra1` is a standard `sensor_msgs/Image` on the mini PC — your
processing nodes subscribe to it exactly like a local camera.

Your own node can also subscribe to the `/compressed` topic directly via
`image_transport` (transport hint `compressed`) and decompress internally.

---

## Setting JPEG quality live (on the Pi)
```bash
ros2 param list /d435_stream_republish            # find the exact name
ros2 param set /d435_stream_republish out.compressed.jpeg_quality 25
```

## WiFi tuning tips
- Lower `jpeg_quality` (e.g. 20) if the link stalls; raise it for more detail.
- Subscribe with **BEST_EFFORT** QoS (sensor data uses it) — `rqt_image_view`
  and `image_transport` handle this automatically.
- Mono IR (640x480) is lighter than color — prefer `infra1` for processing.
- Keep resolution at 640x480; higher over WiFi rarely helps.

## Troubleshooting
- **Mini PC doesn't see the Pi's topics:** same `ROS_DOMAIN_ID` on both? ping
  works both ways? AP isolation off? Both sourced Humble? Those four fix almost
  everything now that the distros match.
- **Topics appear but `topic hz` is 0:** you're probably subscribing to the raw
  image over WiFi (too heavy) — subscribe to `/compressed` instead; or a QoS
  mismatch (use BEST_EFFORT).
