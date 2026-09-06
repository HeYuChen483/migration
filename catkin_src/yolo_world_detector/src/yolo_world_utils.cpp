#include "yolo_world_detector/yolo_world_utils.hpp"

#include <algorithm>
#include <cmath>
#include <fstream>
#include <limits>
#include <stdexcept>
#include <unordered_map>
#include <unordered_set>

#include <opencv2/dnn/dnn.hpp>
#include <opencv2/imgproc.hpp>

namespace yolo_world_detector
{
namespace
{

std::string trim(const std::string& value)
{
    const std::string whitespace = " \t\r\n";
    const std::size_t first = value.find_first_not_of(whitespace);
    if (first == std::string::npos)
    {
        return {};
    }
    const std::size_t last = value.find_last_not_of(whitespace);
    return value.substr(first, last - first + 1U);
}

}  // namespace

std::vector<std::string> loadClassNames(const std::string& path)
{
    std::ifstream stream(path);
    if (!stream)
    {
        throw std::runtime_error("cannot open class names file: " + path);
    }

    std::vector<std::string> names;
    std::unordered_set<std::string> unique_names;
    std::string line;
    while (std::getline(stream, line))
    {
        const std::string name = trim(line);
        if (name.empty())
        {
            continue;
        }
        if (!unique_names.insert(name).second)
        {
            throw std::runtime_error("duplicate class name: " + name);
        }
        names.push_back(name);
    }
    if (names.empty())
    {
        throw std::runtime_error("class names file is empty: " + path);
    }
    return names;
}

LetterboxResult letterbox(const cv::Mat& source, int target_width, int target_height)
{
    if (source.empty())
    {
        throw std::invalid_argument("cannot letterbox an empty image");
    }
    if (target_width <= 0 || target_height <= 0)
    {
        throw std::invalid_argument("letterbox dimensions must be positive");
    }

    const float width_scale = static_cast<float>(target_width) / source.cols;
    const float height_scale = static_cast<float>(target_height) / source.rows;
    const float scale = std::min(width_scale, height_scale);
    const int resized_width = std::max(1, static_cast<int>(std::round(source.cols * scale)));
    const int resized_height = std::max(1, static_cast<int>(std::round(source.rows * scale)));

    cv::Mat resized;
    cv::resize(source, resized, cv::Size(resized_width, resized_height), 0.0, 0.0,
               scale < 1.0F ? cv::INTER_AREA : cv::INTER_LINEAR);

    const int horizontal_padding = target_width - resized_width;
    const int vertical_padding = target_height - resized_height;
    LetterboxResult result;
    result.scale = scale;
    result.pad_left = horizontal_padding / 2;
    result.pad_top = vertical_padding / 2;
    cv::copyMakeBorder(resized, result.image, result.pad_top,
                       vertical_padding - result.pad_top, result.pad_left,
                       horizontal_padding - result.pad_left, cv::BORDER_CONSTANT,
                       cv::Scalar(114, 114, 114));
    return result;
}

cv::Rect2f clipBox(const cv::Rect2f& box, int image_width, int image_height)
{
    const float left = std::max(0.0F, std::min(box.x, static_cast<float>(image_width)));
    const float top = std::max(0.0F, std::min(box.y, static_cast<float>(image_height)));
    const float right = std::max(left, std::min(box.x + box.width,
                                                static_cast<float>(image_width)));
    const float bottom = std::max(top, std::min(box.y + box.height,
                                                static_cast<float>(image_height)));
    return cv::Rect2f(left, top, right - left, bottom - top);
}

std::vector<Detection> applyTemporalHold(
    const std::vector<Detection>& current, std::size_t minimum_count,
    double hold_seconds, double now_seconds, TemporalHoldState* state,
    bool* used_hold)
{
    if (used_hold != nullptr)
    {
        *used_hold = false;
    }
    if (state == nullptr || minimum_count == 0U || hold_seconds <= 0.0 ||
        !std::isfinite(hold_seconds) || !std::isfinite(now_seconds))
    {
        return current;
    }

    if (current.size() >= minimum_count)
    {
        state->detections = current;
        state->stamp_sec = now_seconds;
        return current;
    }

    if (state->detections.size() >= minimum_count && state->stamp_sec >= 0.0 &&
        now_seconds - state->stamp_sec <= hold_seconds &&
        now_seconds >= state->stamp_sec)
    {
        if (used_hold != nullptr)
        {
            *used_hold = true;
        }
        return state->detections;
    }

    return current;
}

std::vector<Detection> decodeWorldOutput(
    const float* data, const std::vector<std::int64_t>& shape,
    std::size_t class_count, float confidence_threshold, float nms_threshold,
    const LetterboxResult& letterboxed, int image_width, int image_height)
{
    if (data == nullptr || class_count == 0U)
    {
        throw std::invalid_argument("YOLO output data and class names must not be empty");
    }
    if (shape.size() != 3U || shape[0] != 1)
    {
        throw std::runtime_error("expected YOLO output rank 3 with batch size 1");
    }

    const std::size_t attribute_count = 4U + class_count;
    bool attributes_first = false;
    std::size_t candidate_count = 0U;
    if (shape[1] == static_cast<std::int64_t>(attribute_count) && shape[2] > 0)
    {
        attributes_first = true;
        candidate_count = static_cast<std::size_t>(shape[2]);
    }
    else if (shape[2] == static_cast<std::int64_t>(attribute_count) && shape[1] > 0)
    {
        candidate_count = static_cast<std::size_t>(shape[1]);
    }
    else
    {
        throw std::runtime_error("YOLO output attributes do not match 4 + class count");
    }

    const auto value_at = [data, attributes_first, candidate_count, attribute_count](
                              std::size_t candidate, std::size_t attribute) {
        return attributes_first ? data[attribute * candidate_count + candidate]
                                : data[candidate * attribute_count + attribute];
    };

    std::vector<Detection> candidates;
    candidates.reserve(candidate_count / 8U + 1U);
    for (std::size_t candidate_index = 0; candidate_index < candidate_count;
         ++candidate_index)
    {
        int best_class = -1;
        float best_score = -std::numeric_limits<float>::infinity();
        for (std::size_t class_index = 0; class_index < class_count; ++class_index)
        {
            const float score = value_at(candidate_index, 4U + class_index);
            if (std::isfinite(score) && score > best_score)
            {
                best_score = score;
                best_class = static_cast<int>(class_index);
            }
        }
        if (best_class < 0 || best_score < confidence_threshold)
        {
            continue;
        }

        const float center_x = value_at(candidate_index, 0U);
        const float center_y = value_at(candidate_index, 1U);
        const float width = value_at(candidate_index, 2U);
        const float height = value_at(candidate_index, 3U);
        if (!std::isfinite(center_x) || !std::isfinite(center_y) ||
            !std::isfinite(width) || !std::isfinite(height) || width <= 0.0F ||
            height <= 0.0F)
        {
            continue;
        }

        const cv::Rect2f model_box(center_x - width * 0.5F,
                                   center_y - height * 0.5F, width, height);
        const cv::Rect2f original_box = clipBox(
            cv::Rect2f((model_box.x - letterboxed.pad_left) / letterboxed.scale,
                       (model_box.y - letterboxed.pad_top) / letterboxed.scale,
                       model_box.width / letterboxed.scale,
                       model_box.height / letterboxed.scale),
            image_width, image_height);
        if (original_box.width > 0.0F && original_box.height > 0.0F)
        {
            candidates.push_back({best_class, best_score, original_box});
        }
    }

    std::unordered_map<int, std::vector<std::size_t>> candidates_by_class;
    for (std::size_t index = 0; index < candidates.size(); ++index)
    {
        candidates_by_class[candidates[index].class_id].push_back(index);
    }

    std::vector<Detection> detections;
    detections.reserve(candidates.size());
    for (const auto& class_candidates : candidates_by_class)
    {
        std::vector<cv::Rect2d> boxes;
        std::vector<float> scores;
        boxes.reserve(class_candidates.second.size());
        scores.reserve(class_candidates.second.size());
        for (const std::size_t index : class_candidates.second)
        {
            const cv::Rect2f& box = candidates[index].box;
            boxes.emplace_back(box.x, box.y, box.width, box.height);
            scores.push_back(candidates[index].confidence);
        }

        std::vector<int> kept_indices;
        cv::dnn::NMSBoxes(boxes, scores, confidence_threshold, nms_threshold,
                          kept_indices);
        for (const int kept_index : kept_indices)
        {
            detections.push_back(candidates[class_candidates.second.at(
                static_cast<std::size_t>(kept_index))]);
        }
    }

    std::sort(detections.begin(), detections.end(),
              [](const Detection& left, const Detection& right) {
                  return left.confidence > right.confidence;
              });
    return detections;
}

}  // namespace yolo_world_detector
