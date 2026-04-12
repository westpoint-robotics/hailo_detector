# hailo_detector

ROS 2 Jazzy package for real-time object detection using a Hailo accelerator on
Raspberry Pi.  A C++ node drives the **rpicam-apps / Hailo YOLOv6**
post-processing pipeline and publishes results as
`vision_msgs/Detection2DArray`.  Optionally it also publishes annotated camera
frames — raw (`sensor_msgs/Image`) and/or compressed (`sensor_msgs/CompressedImage`) —
with bounding boxes, centroids, and confidence labels drawn on each frame.
A Python **Tkinter GUI** node subscribes to the detection topic and displays
detections live.

```
┌──────────────────────────────────┐        ┌──────────────────────────────┐
│  detector_node  (C++)            │        │  detection_gui_node  (Python) │
│                                  │        │                               │
│  rpicam-apps + Hailo             │──────▶│  Tkinter GUI                  │
│  YOLOv6 pipeline                 │  det.  │  • Latest message panel       │
│                                  │  topic │  • Scrollable message log     │
│  Publishes:                      │        │  • Click-to-detail window     │
│  • vision_msgs/Detection2DArray  │        └──────────────────────────────┘
│  • sensor_msgs/Image        (opt)│
│  • sensor_msgs/CompressedImage   │
│                         (opt)    │
└──────────────────────────────────┘
```

---

## Requirements

| Requirement | Notes |
|---|---|
| ROS 2 Jazzy | `source /opt/ros/jazzy/setup.bash` |
| `vision_msgs` | `sudo apt install ros-jazzy-vision-msgs` |
| `sensor_msgs` | included with ROS 2 base |
| `cv_bridge` | `sudo apt install ros-jazzy-cv-bridge` |
| OpenCV | `sudo apt install libopencv-dev` |
| rpicam-apps | Headers at `/usr/local/include/rpicam-apps`, lib at `/usr/local/lib/aarch64-linux-gnu/librpicam_app.so` |
| libcamera | Detected via `pkg-config libcamera` |
| Hailo Runtime | Post-process JSON at `/usr/share/rpi-camera-assets/` |
| Python 3 + tkinter | `sudo apt install python3-tk` |

---

## Package layout

```
hailo_detector/
├── CMakeLists.txt
├── package.xml
├── config/
│   └── detector_params.yaml      # default parameter values
├── include/hailo_detector/
│   └── detector_node.hpp
├── launch/
│   └── detector.launch.py        # launches both nodes together
├── scripts/
│   └── detection_gui_node        # Python GUI subscriber (executable)
└── src/
    └── detector_node.cpp         # C++ publisher
```

---

## Building

```bash
# From the root of your ROS 2 workspace:
cd ~/ros2_ws
colcon build --packages-select hailo_detector
source install/setup.bash
```

---

## Running

### Launch both nodes together (recommended)

```bash
ros2 launch hailo_detector detector.launch.py \
    post_process_file:=/usr/share/rpi-camera-assets/hailo_yolov6_inference.json
```

To suppress the GUI (headless / logging only):

```bash
ros2 launch hailo_detector detector.launch.py gui:=false
```

### Annotated image output

Enable the raw annotated image topic:

```bash
ros2 launch hailo_detector detector.launch.py \
    publish_annotated_image:=true
```

Enable compressed output only (lower bandwidth):

```bash
ros2 launch hailo_detector detector.launch.py \
    publish_compressed_image:=true \
    jpeg_quality:=75
```

Enable both simultaneously:

```bash
ros2 launch hailo_detector detector.launch.py \
    publish_annotated_image:=true \
    publish_compressed_image:=true
```

View the annotated stream with:

```bash
rqt_image_view /annotated_image
# or for compressed:
rqt_image_view /annotated_image/compressed
```

### Run nodes individually

```bash
# Detector (C++)
ros2 run hailo_detector detector_node --ros-args \
    -p post_process_file:=/usr/share/rpi-camera-assets/hailo_yolov6_inference.json \
    -p confidence_threshold:=0.25

# GUI subscriber (Python) — on the same machine or any machine on the ROS network
ros2 run hailo_detector detection_gui_node
```

### Using the params file

```bash
ros2 run hailo_detector detector_node --ros-args \
    --params-file $(ros2 pkg prefix hailo_detector)/share/hailo_detector/config/detector_params.yaml
```

---

## Parameters

### `detector_node` (C++)

| Parameter | Type | Default | Description |
|---|---|---|---|
| `post_process_file` | string | `"/usr/share/rpi-camera-assets/hailo_yolov6_inference.json"` | Path to Hailo post-processing JSON. |
| `image_width` | int | `640` | Lores stream width fed to the inference engine. |
| `image_height` | int | `640` | Lores stream height fed to the inference engine. |
| `topic` | string | `"detections"` | Topic on which `Detection2DArray` is published. |
| `frame_id` | string | `"camera"` | TF coordinate frame written into message headers. |
| `confidence_threshold` | double | `0.0` | Detections below this confidence are dropped before publishing or drawing. |
| `enable_preview` | bool | `false` | Open an rpicam preview window. |
| `publish_annotated_image` | bool | `false` | Publish a `sensor_msgs/Image` with annotations drawn on every frame. |
| `annotated_image_topic` | string | `"annotated_image"` | Topic for the raw annotated image. |
| `publish_compressed_image` | bool | `false` | Publish a `sensor_msgs/CompressedImage` with annotations drawn on every frame. |
| `compressed_image_topic` | string | `"annotated_image/compressed"` | Topic for the compressed annotated image. |
| `compressed_format` | string | `"jpeg"` | Compression codec: `"jpeg"` or `"png"`. |
| `jpeg_quality` | int | `90` | JPEG quality (0–100). Ignored when `compressed_format` is `"png"`. |

### `detection_gui_node` (Python)

| Parameter | Type | Default | Description |
|---|---|---|---|
| `topic` | string | `"detections"` | Topic to subscribe to. Must match the detector. |

---

## Published topics

| Topic | Type | Description |
|---|---|---|
| `/detections` *(configurable)* | `vision_msgs/Detection2DArray` | One message per frame containing at least one detection above the confidence threshold. |
| `/annotated_image` *(configurable)* | `sensor_msgs/Image` | Every camera frame with bounding boxes, centroids, and confidence labels drawn. Published only when `publish_annotated_image` is `true`. |
| `/annotated_image/compressed` *(configurable)* | `sensor_msgs/CompressedImage` | Same annotated frame compressed as JPEG or PNG. Published only when `publish_compressed_image` is `true`. |

> When both image topics are enabled the annotated frame is built only once per
> camera tick and shared between publishers.

### Detection2DArray field mapping

| Field | Source |
|---|---|
| `header.stamp` | `rclcpp::Node::now()` at publish time |
| `header.frame_id` | `frame_id` parameter |
| `detections[i].bbox.center.x/y` | Centroid of the rpicam-apps bounding box (pixels) |
| `detections[i].bbox.size_x/y` | Bounding-box width / height (pixels) |
| `detections[i].results[0].hypothesis.class_id` | Class label string (e.g. `"person"`) |
| `detections[i].results[0].hypothesis.score` | Confidence in [0, 1] |

### Annotated image overlay

Each detection drawn on the frame consists of:

- **Green rectangle** — bounding box
- **Red filled circle** — centroid
- **Label** — `"<class> <confidence>%"` in white on a black background, placed above the box

Only detections that pass `confidence_threshold` are drawn.

---

## GUI overview

The Tkinter GUI window has three sections:

1. **Status bar** — shows the subscribed topic name and the timestamp of the
   last received message.
2. **Latest Message** — always reflects the most recently received
   `Detection2DArray`: message counter, timestamp, frame ID, and a treeview of
   all individual detections.
3. **Message Log** — scrollable table of the last 500 received messages.
   **Click any row** to open a floating detail window showing every detection
   in that message.

---

## Inspecting topics from the command line

```bash
# Detection messages
ros2 topic echo /detections
ros2 topic hz /detections

# Annotated image bandwidth
ros2 topic hz /annotated_image
ros2 topic hz /annotated_image/compressed

# One-shot type info
ros2 topic info /detections
```

---

## Relationship to detect_sender

This package is the ROS 2 successor to the `detect_sender` project.  The
original project used a raw TCP socket to stream newline-delimited JSON between
a sender process and a Python receiver GUI.  This package replaces that
transport with a standard ROS 2 topic, making the detections available to any
node on the ROS network without a custom protocol.

| `detect_sender` | `hailo_detector` |
|---|---|
| TCP socket transport | ROS 2 topic |
| Newline-delimited JSON | `vision_msgs/Detection2DArray` |
| Single receiver only | Any number of subscribers |
| Python receiver GUI | Python Tkinter GUI (`detection_gui_node`) |
| SQLite database backend | None |
