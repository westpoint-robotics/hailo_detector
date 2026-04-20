# hailo_detector

ROS 2 Jazzy package for real-time object detection using a Hailo accelerator on
Raspberry Pi.  A C++ node drives the **rpicam-apps / Hailo YOLOv6**
post-processing pipeline and publishes results as
`vision_msgs/Detection2DArray`.  Optionally it also publishes annotated camera
frames as raw or compressed images.  A Python **Tkinter GUI** node subscribes to
the detection topic and displays results live.

https://github.com/westpoint-robotics/hailo_detector.git
```
┌─────────────────────────────┐        ┌──────────────────────────────┐
│  detector_node  (C++)       │        │  detection_gui_node  (Python) │
│                             │        │                               │
│  rpicam-apps + Hailo        │        │  Tkinter GUI                  │
│  YOLOv6 pipeline            │──────▶│  • Latest message panel       │
│                             │  ROS 2 │  • Scrollable message log     │
│  Publishes:                 │  topic │  • Click-to-detail window     │
│  • Detection2DArray         │        │                               │
│  • Image (optional)         │        └──────────────────────────────┘
│  • CompressedImage (opt.)   │
└─────────────────────────────┘
```

---

## Requirements

| Requirement | Notes |
|---|---|
| ROS 2 Jazzy | `source /opt/ros/jazzy/setup.bash` |
| `vision_msgs` | `sudo apt install ros-jazzy-vision-msgs` |
| `sensor_msgs` | Included with ROS 2 Jazzy base install |
| OpenCV | Used for frame decoding, annotation, and compression |
| rpicam-apps | Headers at `/usr/local/include/rpicam-apps`, lib at `/usr/local/lib/aarch64-linux-gnu/librpicam_app.so` |
| libcamera | Detected via `pkg-config libcamera` |
| Hailo Runtime | Post-process JSON at `/usr/share/rpi-camera-assets/` |
| Python 3 + tkinter | `sudo apt install python3-tk` (for GUI node) |

> **Note:** A `COLCON_IGNORE` file is present at the package root. This prevents
> accidental builds on non-Raspberry Pi machines. Use `--packages-select` to
> build explicitly.

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

Enable the GUI:

```bash
ros2 launch hailo_detector detector.launch.py \
    post_process_file:=/usr/share/rpi-camera-assets/hailo_yolov6_inference.json \
    gui:=true
```

Enable annotated image streaming:

```bash
ros2 launch hailo_detector detector.launch.py \
    post_process_file:=/usr/share/rpi-camera-assets/hailo_yolov6_inference.json \
    publish_annotated_image:=true \
    publish_compressed_image:=true \
    compressed_format:=jpeg \
    jpeg_quality:=80
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
| `post_process_file` | string | `"/usr/share/rpi-camera-assets/hailo_yolov6_inference.json"` | **Required.** Path to Hailo post-processing JSON. Node exits fatally if empty. |
| `image_width` | int | `640` | Lores stream width fed to the inference engine. |
| `image_height` | int | `640` | Lores stream height fed to the inference engine. |
| `topic` | string | `"detections"` | Topic on which `Detection2DArray` is published. |
| `frame_id` | string | `"camera"` | TF coordinate frame written into message headers. |
| `confidence_threshold` | double | `0.0` | Detections below this confidence are dropped before publishing. |
| `enable_preview` | bool | `false` | Open an rpicam preview window. |
| `publish_annotated_image` | bool | `false` | Publish BGR8 raw annotated frames on `annotated_image_topic`. |
| `annotated_image_topic` | string | `"annotated_image"` | Topic for `sensor_msgs/Image` (BGR8). |
| `publish_compressed_image` | bool | `false` | Publish JPEG/PNG compressed annotated frames on `compressed_image_topic`. |
| `compressed_image_topic` | string | `"annotated_image/compressed"` | Topic for `sensor_msgs/CompressedImage`. |
| `compressed_format` | string | `"jpeg"` | Compression format: `"jpeg"` or `"png"`. |
| `jpeg_quality` | int | `90` | JPEG quality 0–100; ignored when `compressed_format` is `"png"`. |

### `detection_gui_node` (Python)

| Parameter | Type | Default | Description |
|---|---|---|---|
| `topic` | string | `"detections"` | Topic to subscribe to. Must match the detector's `topic` parameter. |

---

## Topics

| Topic | Type | Direction | Notes |
|---|---|---|---|
| `/detections` *(configurable)* | `vision_msgs/Detection2DArray` | Published | One message per frame with at least one detection above the confidence threshold. |
| `/annotated_image` *(configurable)* | `sensor_msgs/Image` | Published | BGR8 raw frame with bounding boxes drawn. Only when `publish_annotated_image:=true`. |
| `/annotated_image/compressed` *(configurable)* | `sensor_msgs/CompressedImage` | Published | JPEG or PNG compressed annotated frame. Only when `publish_compressed_image:=true`. |
| `/detections` *(configurable)* | `vision_msgs/Detection2DArray` | Subscribed (`detection_gui_node`) | Must match the detector's `topic` parameter. |

All publishers use a queue depth of 10.

### Detection2DArray field mapping

| `Detection2DArray` field | Source |
|---|---|
| `header.stamp` | `rclcpp::Node::now()` at publish time |
| `header.frame_id` | `frame_id` parameter |
| `detections[i].bbox.center.position.x/y` | Centroid of the bounding box in pixels (`box.x + width/2`, `box.y + height/2`) |
| `detections[i].bbox.size_x/y` | Bounding-box width / height in pixels |
| `detections[i].results[0].hypothesis.class_id` | Class label string (e.g. `"person"`) |
| `detections[i].results[0].hypothesis.score` | Confidence in [0, 1] |

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
# Print detection messages as they arrive
ros2 topic echo /detections

# Check publish rate
ros2 topic hz /detections

# View raw annotated frames (requires rqt or similar)
ros2 run rqt_image_view rqt_image_view /annotated_image

# One-shot topic info
ros2 topic info /detections
```

---

## Design notes

- **Threading model:** `main()` starts a dedicated `std::thread` for
  `rclcpp::spin()`, keeping ROS 2 parameter services alive while the blocking
  `run_camera_loop()` occupies the main thread.
- **Silent frames:** The detector only publishes a `Detection2DArray` when at
  least one detection passes the confidence threshold. Frames with no qualifying
  detections produce no topic traffic.
- **Annotated images are independent of detections:** The image publishing path
  fires on every frame and draws only the detections that pass the threshold.
  If no detections pass, a plain (unannotated) frame is still published.

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
