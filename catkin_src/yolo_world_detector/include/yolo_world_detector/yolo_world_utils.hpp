#pragma once

#include <cstdint>
#include <string>
#include <vector>

#include <opencv2/core.hpp>

namespace yolo_world_detector
{

struct LetterboxResult
{
    cv::Mat image;
    float scale = 1.0F;
    int pad_left = 0;
    int pad_top = 0;
};

struct Detection
{
    int class_id = -1;
    float confidence = 0.0F;
    cv::Rect2f box;
};

struct TemporalHoldState
{
    std::vector<Detection> detections;
    double stamp_sec = -1.0;
};

std::vector<std::string> loadClassNames(const std::string& path);
LetterboxResult letterbox(const cv::Mat& source, int target_width, int target_height);
cv::Rect2f clipBox(const cv::Rect2f& box, int image_width, int image_height);

std::vector<Detection> applyTemporalHold(
    const std::vector<Detection>& current, std::size_t minimum_count,
    double hold_seconds, double now_seconds, TemporalHoldState* state,
    bool* used_hold = nullptr);

std::vector<Detection> decodeWorldOutput(
    const float* data, const std::vector<std::int64_t>& shape,
    std::size_t class_count, float confidence_threshold, float nms_threshold,
    const LetterboxResult& letterboxed, int image_width, int image_height);

}  // namespace yolo_world_detector
