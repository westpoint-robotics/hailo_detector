/* SPDX-License-Identifier: BSD-2-Clause */
/*
 * detector_node.cpp
 *
 * ROS 2 node that drives the rpicam-apps / Hailo YOLOv6 post-processing
 * pipeline and publishes each frame's detection results as a
 * vision_msgs/Detection2DArray message.
 *
 * ROS 2 Parameters
 * ────────────────
 *   post_process_file    (string)  Path to the Hailo post-processing JSON.
 *                                  Required — node will not start without it.
 *   image_width          (int)     Lores stream width  [default: 640]
 *   image_height         (int)     Lores stream height [default: 640]
 *   topic                (string)  Output topic name   [default: "detections"]
 *   frame_id             (string)  TF frame in headers [default: "camera"]
 *   confidence_threshold (double)  Detections below this score are dropped
 *                                  before publishing   [default: 0.0]
 *   enable_preview       (bool)    Open a camera preview window
 *                                                      [default: false]
 *
 * Message layout (vision_msgs/Detection2DArray)
 * ─────────────────────────────────────────────
 *   header.stamp            — rclcpp::Node::now() at publish time
 *   header.frame_id         — frame_id parameter
 *   detections[i].bbox.center.x/y — centroid in pixels
 *   detections[i].bbox.size_x/y   — bounding-box width / height in pixels
 *   detections[i].results[0].hypothesis.class_id — class label string
 *   detections[i].results[0].hypothesis.score    — confidence [0, 1]
 */

#include <cstring>
#include <thread>
#include <string>
#include <vector>

#include <opencv2/opencv.hpp>

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <vision_msgs/msg/detection2_d_array.hpp>
#include <vision_msgs/msg/detection2_d.hpp>
#include <vision_msgs/msg/object_hypothesis_with_pose.hpp>

#include <libcamera/formats.h>

#include "core/buffer_sync.hpp"
#include "core/options.hpp"
#include "core/rpicam_app.hpp"
#include "core/stream_info.hpp"
#include "post_processing_stages/object_detect.hpp"

#include "hailo_detector/detector_node.hpp"

// ─────────────────────────────────────────────────────────────────────────────
// Construction / parameter declaration
// ─────────────────────────────────────────────────────────────────────────────

DetectorNode::DetectorNode() : Node("hailo_detector")
{
    declare_parameter("post_process_file",    "/usr/share/rpi-camera-assets/hailo_yolov6_inference.json");
    declare_parameter("image_width",          640);
    declare_parameter("image_height",         640);
    declare_parameter("topic",                "detections");
    declare_parameter("frame_id",             "camera");
    declare_parameter("confidence_threshold",   0.0);
    declare_parameter("enable_preview",         false);
    declare_parameter("publish_annotated_image",  false);
    declare_parameter("annotated_image_topic",    "annotated_image");
    declare_parameter("publish_compressed_image", false);
    declare_parameter("compressed_image_topic",   "annotated_image/compressed");
    declare_parameter("compressed_format",        "jpeg");  // "jpeg" or "png"
    declare_parameter("jpeg_quality",             90);

    post_process_file_        = get_parameter("post_process_file").as_string();
    image_width_              = get_parameter("image_width").as_int();
    image_height_             = get_parameter("image_height").as_int();
    frame_id_                 = get_parameter("frame_id").as_string();
    confidence_threshold_     = get_parameter("confidence_threshold").as_double();
    enable_preview_           = get_parameter("enable_preview").as_bool();
    publish_annotated_image_  = get_parameter("publish_annotated_image").as_bool();
    publish_compressed_image_ = get_parameter("publish_compressed_image").as_bool();
    compressed_format_        = get_parameter("compressed_format").as_string();
    jpeg_quality_             = get_parameter("jpeg_quality").as_int();

    const auto topic = get_parameter("topic").as_string();
    pub_ = create_publisher<vision_msgs::msg::Detection2DArray>(topic, 10);

    if (publish_annotated_image_)
    {
        const auto img_topic = get_parameter("annotated_image_topic").as_string();
        image_pub_ = create_publisher<sensor_msgs::msg::Image>(img_topic, 10);
        RCLCPP_INFO(get_logger(), "Annotated image on '%s'", img_topic.c_str());
    }

    if (publish_compressed_image_)
    {
        const auto cmp_topic = get_parameter("compressed_image_topic").as_string();
        compressed_pub_ = create_publisher<sensor_msgs::msg::CompressedImage>(cmp_topic, 10);
        RCLCPP_INFO(get_logger(), "Compressed image (%s q=%d) on '%s'",
                    compressed_format_.c_str(), jpeg_quality_, cmp_topic.c_str());
    }

    RCLCPP_INFO(get_logger(), "Publishing on '%s'", topic.c_str());
    RCLCPP_INFO(get_logger(), "Resolution: %dx%d  confidence_threshold: %.2f",
                image_width_, image_height_, confidence_threshold_);
}

// ─────────────────────────────────────────────────────────────────────────────
// Message builder
// ─────────────────────────────────────────────────────────────────────────────

vision_msgs::msg::Detection2DArray DetectorNode::make_msg(
    const std::vector<Detection> &dets,
    uint32_t                      /*sequence*/) const
{
    vision_msgs::msg::Detection2DArray msg;
    msg.header.stamp    = now();
    msg.header.frame_id = frame_id_;

    for (const Detection &d : dets)
    {
        if (d.confidence < confidence_threshold_)
            continue;

        vision_msgs::msg::Detection2D det;
        det.header = msg.header;

        // rpicam-apps gives top-left origin + size; convert to centroid
        det.bbox.center.position.x = static_cast<double>(d.box.x) + d.box.width  * 0.5;
        det.bbox.center.position.y = static_cast<double>(d.box.y) + d.box.height * 0.5;
        det.bbox.size_x   = static_cast<double>(d.box.width);
        det.bbox.size_y   = static_cast<double>(d.box.height);

        vision_msgs::msg::ObjectHypothesisWithPose hyp;
        hyp.hypothesis.class_id = d.name;
        hyp.hypothesis.score    = static_cast<double>(d.confidence);
        det.results.push_back(hyp);

        msg.detections.push_back(det);
    }

    return msg;
}

// ─────────────────────────────────────────────────────────────────────────────
// Annotated image builder
// ─────────────────────────────────────────────────────────────────────────────

cv::Mat DetectorNode::build_annotated_frame(
    const std::vector<libcamera::Span<uint8_t>> &planes,
    const StreamInfo                             &info,
    const std::vector<Detection>                 &dets) const
{
    const int w      = image_width_;
    const int h      = image_height_;
    const int stride = static_cast<int>(info.stride);

    cv::Mat bgr;

    if (info.pixel_format == libcamera::formats::BGR888)
    {
        // Packed BGR — single plane, stride may include row padding
        bgr.create(h, w, CV_8UC3);
        for (int r = 0; r < h; ++r)
            std::memcpy(bgr.ptr(r), planes[0].data() + r * stride, w * 3);
    }
    else
    {
        // Assume YUV420 (I420) — three planes
        const int uv_stride = stride / 2;
        const int uv_h      = h / 2;
        const int uv_w      = w / 2;

        std::vector<uint8_t> yuv(w * h * 3 / 2);
        for (int r = 0; r < h; ++r)
            std::memcpy(&yuv[r * w], planes[0].data() + r * stride, w);
        for (int r = 0; r < uv_h; ++r)
            std::memcpy(&yuv[w * h + r * uv_w],                planes[1].data() + r * uv_stride, uv_w);
        for (int r = 0; r < uv_h; ++r)
            std::memcpy(&yuv[w * h + uv_h * uv_w + r * uv_w], planes[2].data() + r * uv_stride, uv_w);

        cv::Mat yuv_mat(h * 3 / 2, w, CV_8UC1, yuv.data());
        cv::cvtColor(yuv_mat, bgr, cv::COLOR_YUV2BGR_I420);
    }

    // ── Draw annotations ────────────────────────────────────────────────────
    constexpr double FONT_SCALE  = 0.55;
    constexpr int    THICKNESS   = 2;
    constexpr int    FONT        = cv::FONT_HERSHEY_SIMPLEX;
    const cv::Scalar BOX_COLOUR  { 0, 220,   0 };   // green
    const cv::Scalar DOT_COLOUR  { 0,   0, 220 };   // red
    const cv::Scalar TEXT_FG     { 255, 255, 255 }; // white
    const cv::Scalar TEXT_BG     { 0,   0,   0 };   // black

    for (const Detection &d : dets)
    {
        if (d.confidence < confidence_threshold_)
            continue;

        const int x = d.box.x, y = d.box.y;
        const int bw = d.box.width, bh = d.box.height;
        const int cx = x + bw / 2, cy = y + bh / 2;

        // Bounding box
        cv::rectangle(bgr, cv::Rect(x, y, bw, bh), BOX_COLOUR, THICKNESS);

        // Centroid
        cv::circle(bgr, {cx, cy}, 5, DOT_COLOUR, cv::FILLED);

        // Label: "classname 94%"
        char label[64];
        std::snprintf(label, sizeof(label), "%s %.0f%%",
                      d.name.c_str(), d.confidence * 100.0f);

        int baseline = 0;
        cv::Size ts = cv::getTextSize(label, FONT, FONT_SCALE, THICKNESS, &baseline);
        int lx = x;
        int ly = std::max(y - 4, ts.height + 4);  // keep label inside frame top

        // Dark background rectangle for readability
        cv::rectangle(bgr,
            cv::Point(lx, ly - ts.height - 4),
            cv::Point(lx + ts.width + 2, ly + baseline),
            TEXT_BG, cv::FILLED);
        cv::putText(bgr, label, {lx + 1, ly - 1},
                    FONT, FONT_SCALE, TEXT_FG, THICKNESS, cv::LINE_AA);
    }

    return bgr;
}

// ─────────────────────────────────────────────────────────────────────────────
// Camera loop
// ─────────────────────────────────────────────────────────────────────────────

void DetectorNode::run_camera_loop()
{
    if (post_process_file_.empty())
    {
        RCLCPP_FATAL(get_logger(),
            "Parameter 'post_process_file' must be set before starting.");
        return;
    }

    // Build a fake argv from ROS 2 parameters so rpicam-apps Options::Parse()
    // can be used without altering the ROS 2 command line.
    std::vector<std::string> arg_strings = {
        "detector_node",
        "--post-process-file", post_process_file_,
        "--lores-width",       std::to_string(image_width_),
        "--lores-height",      std::to_string(image_height_),
        "-t", "0",
    };
    if (!enable_preview_)
        arg_strings.emplace_back("--nopreview");

    std::vector<char *> argv;
    argv.reserve(arg_strings.size());
    for (auto &s : arg_strings)
        argv.push_back(s.data());
    int argc = static_cast<int>(argv.size());

    RPiCamApp app;
    Options  *opts = app.GetOptions();

    if (!opts->Parse(argc, argv.data()))
    {
        RCLCPP_FATAL(get_logger(), "Failed to parse rpicam-apps options.");
        return;
    }

    app.OpenCamera();
    app.ConfigureViewfinder();
    app.StartCamera();

    RCLCPP_INFO(get_logger(), "Camera started.");

    while (rclcpp::ok())
    {
        RPiCamApp::Msg msg = app.Wait();

        if (msg.type == RPiCamApp::MsgType::Quit)
            break;

        if (msg.type == RPiCamApp::MsgType::Timeout)
        {
            RCLCPP_WARN(get_logger(), "Camera timeout — restarting.");
            app.StopCamera();
            app.StartCamera();
            continue;
        }

        if (msg.type != RPiCamApp::MsgType::RequestComplete)
            continue;

        CompletedRequestPtr &req = std::get<CompletedRequestPtr>(msg.payload);

        std::vector<Detection> detections;
        req->post_process_metadata.Get("object_detect.results", detections);

        if (!detections.empty())
            pub_->publish(make_msg(detections, req->sequence));

        if (publish_annotated_image_ || publish_compressed_image_)
        {
            StreamInfo lores_info;
            RPiCamApp::Stream *lores = app.LoresStream(&lores_info);
            auto buf_it = req->buffers.find(lores);
            if (lores && buf_it != req->buffers.end())
            {
                BufferReadSync buf_sync(&app, buf_it->second);
                cv::Mat frame = build_annotated_frame(buf_sync.Get(), lores_info, detections);

                const auto stamp = now();

                if (publish_annotated_image_)
                {
                    sensor_msgs::msg::Image img_msg;
                    img_msg.header.stamp    = stamp;
                    img_msg.header.frame_id = frame_id_;
                    img_msg.height          = static_cast<uint32_t>(frame.rows);
                    img_msg.width           = static_cast<uint32_t>(frame.cols);
                    img_msg.encoding        = "bgr8";
                    img_msg.is_bigendian    = false;
                    img_msg.step            = static_cast<uint32_t>(frame.cols * 3);
                    img_msg.data.assign(frame.datastart, frame.dataend);
                    image_pub_->publish(img_msg);
                }

                if (publish_compressed_image_)
                {
                    const bool is_jpeg = (compressed_format_ != "png");
                    std::vector<int> params;
                    if (is_jpeg)
                        params = { cv::IMWRITE_JPEG_QUALITY, jpeg_quality_ };

                    std::vector<uint8_t> buf;
                    cv::imencode(is_jpeg ? ".jpg" : ".png", frame, buf, params);

                    sensor_msgs::msg::CompressedImage cmsg;
                    cmsg.header.stamp    = stamp;
                    cmsg.header.frame_id = frame_id_;
                    cmsg.format          = is_jpeg ? "jpeg" : "png";
                    cmsg.data            = std::move(buf);
                    compressed_pub_->publish(cmsg);
                }
            }
        }

        app.ShowPreview(req, app.ViewfinderStream());
    }

    app.StopCamera();
    RCLCPP_INFO(get_logger(), "Camera stopped.");
}

// ─────────────────────────────────────────────────────────────────────────────
// Entry point
// ─────────────────────────────────────────────────────────────────────────────

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<DetectorNode>();

    // Spin in a background thread so ROS 2 parameter services remain responsive
    // while the blocking camera loop runs on the main thread.
    std::thread spin_thread([&node]() { rclcpp::spin(node); });

    node->run_camera_loop();

    rclcpp::shutdown();
    spin_thread.join();
    return 0;
}
