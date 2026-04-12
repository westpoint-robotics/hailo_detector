"""
detector.launch.py

Launches both the C++ detector node and the Python GUI subscriber together.

Arguments
---------
  post_process_file   Path to the Hailo post-processing JSON.
                      (required — no default; pass via --launch-file-args)
  image_width         Lores stream width  [default: 640]
  image_height        Lores stream height [default: 640]
  topic               Detection topic name [default: detections]
  frame_id            TF frame for message headers [default: camera]
  confidence_threshold  Minimum confidence to publish [default: 0.0]
  enable_preview      Show rpicam preview window [default: false]
  publish_annotated_image  Publish annotated sensor_msgs/Image [default: false]
  annotated_image_topic    Topic for the annotated image [default: annotated_image]
  publish_compressed_image Publish annotated sensor_msgs/CompressedImage [default: false]
  compressed_image_topic   Topic for the compressed image [default: annotated_image/compressed]
  compressed_format        Encoding format: "jpeg" or "png" [default: jpeg]
  jpeg_quality             JPEG quality 0-100 (ignored for png) [default: 90]
  gui                 Launch the Tkinter GUI subscriber [default: true]

Example
-------
  ros2 launch hailo_detector detector.launch.py \\
      post_process_file:=/usr/share/rpi-camera-assets/hailo_yolov6_inference.json
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([

        # ── Launch arguments ──────────────────────────────────────────────────
        DeclareLaunchArgument(
            "post_process_file", default_value="/usr/share/rpi-camera-assets/hailo_yolov6_inference.json",
            description="Path to the Hailo post-processing JSON (required)."),

        DeclareLaunchArgument(
            "image_width", default_value="640",
            description="Low-resolution stream width for inference."),

        DeclareLaunchArgument(
            "image_height", default_value="640",
            description="Low-resolution stream height for inference."),

        DeclareLaunchArgument(
            "topic", default_value="detections",
            description="ROS 2 topic name for Detection2DArray messages."),

        DeclareLaunchArgument(
            "frame_id", default_value="camera",
            description="TF coordinate frame written into message headers."),

        DeclareLaunchArgument(
            "confidence_threshold", default_value="0.0",
            description="Detections below this confidence are dropped."),

        DeclareLaunchArgument(
            "enable_preview", default_value="false",
            description="Open the rpicam preview window."),

        DeclareLaunchArgument(
            "publish_annotated_image", default_value="false",
            description="Publish a sensor_msgs/Image with boxes, centroids, and confidence drawn."),

        DeclareLaunchArgument(
            "annotated_image_topic", default_value="annotated_image",
            description="Topic name for the annotated raw image."),

        DeclareLaunchArgument(
            "publish_compressed_image", default_value="false",
            description="Publish a sensor_msgs/CompressedImage of the annotated frame."),

        DeclareLaunchArgument(
            "compressed_image_topic", default_value="annotated_image/compressed",
            description="Topic name for the compressed annotated image."),

        DeclareLaunchArgument(
            "compressed_format", default_value="jpeg",
            description="Compression format: 'jpeg' or 'png'."),

        DeclareLaunchArgument(
            "jpeg_quality", default_value="90",
            description="JPEG quality (0-100). Ignored when compressed_format is 'png'."),

        DeclareLaunchArgument(
            "gui", default_value="false",
            description="Launch the Tkinter GUI subscriber node."),

        # ── C++ detector node ─────────────────────────────────────────────────
        Node(
            package="hailo_detector",
            executable="detector_node",
            name="hailo_detector",
            output="screen",
            parameters=[{
                "post_process_file":    LaunchConfiguration("post_process_file"),
                "image_width":          LaunchConfiguration("image_width"),
                "image_height":         LaunchConfiguration("image_height"),
                "topic":                LaunchConfiguration("topic"),
                "frame_id":             LaunchConfiguration("frame_id"),
                "confidence_threshold":    LaunchConfiguration("confidence_threshold"),
                "enable_preview":          LaunchConfiguration("enable_preview"),
                "publish_annotated_image":  LaunchConfiguration("publish_annotated_image"),
                "annotated_image_topic":    LaunchConfiguration("annotated_image_topic"),
                "publish_compressed_image": LaunchConfiguration("publish_compressed_image"),
                "compressed_image_topic":   LaunchConfiguration("compressed_image_topic"),
                "compressed_format":        LaunchConfiguration("compressed_format"),
                "jpeg_quality":             LaunchConfiguration("jpeg_quality"),
            }],
        ),

        # ── Python GUI subscriber node ────────────────────────────────────────
        Node(
            package="hailo_detector",
            executable="detection_gui_node",
            name="detection_gui",
            output="screen",
            condition=IfCondition(LaunchConfiguration("gui")),
            parameters=[{
                "topic": LaunchConfiguration("topic"),
            }],
        ),
    ])
