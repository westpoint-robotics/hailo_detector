/* SPDX-License-Identifier: BSD-2-Clause */
#pragma once

#include <string>
#include <vector>

#include <opencv2/opencv.hpp>

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/compressed_image.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <vision_msgs/msg/detection2_d_array.hpp>

#include "core/stream_info.hpp"
#include "post_processing_stages/object_detect.hpp"

class DetectorNode : public rclcpp::Node
{
public:
    DetectorNode();

    /** Blocking camera loop — call from main(), spins until shutdown. */
    void run_camera_loop();

private:
    vision_msgs::msg::Detection2DArray make_msg(
        const std::vector<Detection> &dets,
        uint32_t                      sequence) const;

    /** Decode a lores buffer (BGR888 or YUV420) and draw detection annotations. */
    cv::Mat build_annotated_frame(
        const std::vector<libcamera::Span<uint8_t>> &planes,
        const StreamInfo                             &info,
        const std::vector<Detection>                 &dets) const;

    rclcpp::Publisher<vision_msgs::msg::Detection2DArray>::SharedPtr pub_;
    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr            image_pub_;
    rclcpp::Publisher<sensor_msgs::msg::CompressedImage>::SharedPtr  compressed_pub_;

    std::string post_process_file_;
    int         image_width_;
    int         image_height_;
    std::string frame_id_;
    double      confidence_threshold_;
    bool        enable_preview_;
    bool        publish_annotated_image_;
    bool        publish_compressed_image_;
    std::string compressed_format_;
    int         jpeg_quality_;
};
