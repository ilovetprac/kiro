# Streaming the D435 feed: Pi (Humble) → mini PC (Jazzy) over WiFi

Publish the live D435 camera feed from the **Raspberry Pi (ROS 2 Humble,
Ubuntu 22.04)** and subscribe + **process it on the mini PC (ROS 2 Jazzy,
Ubuntu 24.04)** over the `rachub2` WiFi network (no internet).

It uses the normal ROS 2 **publish/subscribe** model — the Pi publishes the
camera topic, the mini PC subscribes — plus two things that make it actually
work in this specific setup:

1. **Compressed (JPEG) image transport** — raw images are too heavy for WiFi.
2. **CycloneDDS on BOTH machines + same `ROS_DOMAIN_ID`** — because Humble and
   Jazzy do not interoperate reliably with their different default DDS stacks.

---

## 0. One-time install (DO THIS WHILE YOU STILL HAVE INTERNET)

`rachub2` has no internet, so install everything first on a network that does.

**On the Pi (Humble):**
```bash
sudo apt update
sudo apt install -y \
  ros-humble-rmw-cyclonedds-cpp \
  ros-humble-image-transport \
  ros-humble-compressed-image-transport
```

**On the mini PC (Jazzy):**
```bash
sudo apt update
sudo apt install -y \
  ros-jazzy-rmw-cyclonedds-cpp \
  ros-jazzy-image-transport \
  ros-jazzy-compressed-image-transport \
  ros-jazzy-rqt-image-view          # optional, for a quick visual check
```

---

## 1. Network: put BOTH machines on `rachub2`, same subnet

- Connect the Pi and the mini PC to `rachub2`.
- Confirm they can see each other (no internet needed, just L2/L3 reachability):
  ```bash
  # on the Pi, find its IP:
  ip addr show | grep "inet "
  # from the mini PC, ping the Pi's IP:
  ping <pi-ip>
  ```
  If ping fails, it's a WiFi/AP-isolation problem — fix that first (DDS can't
  work without basic reachability). Some APs block client-to-client traffic
  ("AP/client isolation"); it must be OFF on `rachub2`.

---

## 2. Set the SAME DDS + domain on BOTH machines

Put these in **each machine's** shell (add to `~/.bashrc` so every terminal
gets them, then open a fresh terminal or `source ~/.bashrc`):

```bash
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
export ROS_DOMAIN_ID=7            # any number 0..101, but IDENTICAL on both
```

Both must match exactly. (`ROS_DOMAIN_ID` just has to be the same value on
both; 7 is arbitrary.)

### If discovery still doesn't work over WiFi (common)
Multicast discovery is often unreliable/blocked on WiFi. Force **unicast peer**
discovery by pointing CycloneDDS at the other machine's IP. On EACH machine,
create `~/cyclonedds.xml` with the OTHER machine's IP as a peer:

```xml
<?xml version="1.0" encoding="UTF-8"?>
<CycloneDDS xmlns="https://cdds.io/config">
  <Domain id="any">
    <Discovery>
      <Peers>
        <!-- put the OTHER machine's rachub2 IP here -->
        <Peer address="192.168.x.y"/>
      </Peers>
      <ParticipantIndex>auto</ParticipantIndex>
    </Discovery>
  </Domain>
</CycloneDDS>
```
Then point to it (add to `~/.bashrc` on both):
```bash
export CYCLONEDDS_URI=file://$HOME/cyclonedds.xml
```
(On the Pi, set the mini PC's IP as the peer; on the mini PC, set the Pi's IP.)

---

## 3. Start the stream on the Pi

```bash
# source ROS + workspace first
source /opt/ros/humble/setup.bash
source ~/ros2_ws/install/setup.bash
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
export ROS_DOMAIN_ID=7

ros2 launch qr_coordinates_ros2 camera_stream.launch.py
```

Options:
```bash
# stream the IR feed (default), tune JPEG quality for WiFi (lower = lighter):
ros2 launch qr_coordinates_ros2 camera_stream.launch.py jpeg_quality:=25

# stream color instead of IR (nicer to look at; enables the color sensor):
ros2 launch qr_coordinates_ros2 camera_stream.launch.py stream_source:=color

# camera already running elsewhere? don't start a second driver:
ros2 launch qr_coordinates_ros2 camera_stream.launch.py start_camera:=false
```

This publishes:
| Topic | Type | Use |
|-------|------|-----|
| `/camera/camera/infra1/image_rect_raw` | `sensor_msgs/Image` | raw (local) |
| `/camera/camera/infra1/image_rect_raw/compressed` | `sensor_msgs/CompressedImage` | **stream this over WiFi** |

(For `stream_source:=color`, swap `infra1/image_rect_raw` for
`color/image_raw`.)

---

## 4. Subscribe + PROCESS on the mini PC (Jazzy)

Open a terminal on the mini PC with the matching env:
```bash
source /opt/ros/jazzy/setup.bash
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
export ROS_DOMAIN_ID=7
export CYCLONEDDS_URI=file://$HOME/cyclonedds.xml   # if using unicast peers
```

**Check the Pi's topics are visible:**
```bash
ros2 topic list | grep camera
# you should see .../infra1/image_rect_raw/compressed
ros2 topic hz /camera/camera/infra1/image_rect_raw/compressed
```

**Quick visual check (optional):**
```bash
ros2 run rqt_image_view rqt_image_view
# pick the .../compressed topic
```

**To PROCESS it as a normal image**, decompress locally (keeps only the JPEG on
the wire; the raw image is reconstructed on the mini PC):
```bash
ros2 run image_transport republish compressed raw \
  --ros-args \
  -r in/compressed:=/camera/camera/infra1/image_rect_raw/compressed \
  -r out:=/d435/infra1
```
Now `/d435/infra1` is a standard `sensor_msgs/Image` on the mini PC that your
processing nodes can subscribe to exactly like a local camera.

Your own node can also subscribe to the `/compressed` topic directly via
`image_transport` (transport hint `compressed`) and get the decompressed image
without a separate republish — whichever you prefer.

---

## Setting JPEG quality reliably (live)

The `jpeg_quality:=` launch arg sets a default, but the most reliable way to
change it is live, on the compressed publisher's own parameter (on the Pi):
```bash
# list the republish node's params to see the exact name:
ros2 param list /d435_stream_republish
# set quality (1..100); lower = lighter over WiFi:
ros2 param set /d435_stream_republish out.compressed.jpeg_quality 25
```
(The exact parameter path can vary slightly by version — use `ros2 param list`
to confirm the name, then `ros2 param set` it.)

## WiFi tuning tips
- Lower `jpeg_quality` (e.g. 20) if the link stalls; raise it if you need detail.
- Image subscribers must use **BEST_EFFORT** QoS (realsense/sensor data uses
  it). `rqt_image_view` and `image_transport` handle this automatically.
- Mono IR (640x480) compresses smaller than color — prefer `infra1` if you just
  need the grayscale feed for processing.
- Keep the resolution at 640x480; going higher over WiFi rarely helps.

## Troubleshooting
- **`ros2 topic list` on the mini PC doesn't show the Pi's topics:** DDS
  discovery isn't crossing. Re-check: same `ROS_DOMAIN_ID`, `RMW_IMPLEMENTATION`
  = cyclonedds on BOTH, ping works, AP isolation OFF. Then add the unicast
  `cyclonedds.xml` peers (section 2).
- **Topics appear but `topic hz` is 0 / nothing arrives:** almost always QoS
  (use a BEST_EFFORT subscriber) or WiFi dropping the raw stream (make sure you
  are subscribing to `/compressed`, not the raw image, over the network).
- **Cross-distro type errors:** stick to standard `sensor_msgs` types (Image /
  CompressedImage) — custom messages are the most likely to fail Humble↔Jazzy.
