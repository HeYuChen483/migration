#include <algorithm>
#include <array>
#include <atomic>
#include <chrono>
#include <cmath>
#include <condition_variable>
#include <cstddef>
#include <cstdint>
#include <iomanip>
#include <iostream>
#include <memory>
#include <mutex>
#include <sstream>
#include <stdexcept>
#include <string>
#include <thread>
#include <unordered_map>
#include <utility>
#include <vector>

#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.h>
#include <opencv2/dnn/dnn.hpp>
#include <opencv2/imgproc.hpp>
#include <ros/package.h>
#include <ros/ros.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/image_encodings.h>
#include <vision_msgs/DetectionArray.h>
#include <vision_msgs/DetectionResult.h>

#include <onnxruntime_cxx_api.h>

#include "yolo_world_detector/yolo_world_utils.hpp"

namespace
{

constexpr int kDefaultInputWidth = 640;
constexpr int kDefaultInputHeight = 640;
constexpr float kDefaultConfidenceThreshold = 0.25F;
constexpr float kDefaultNmsThreshold = 0.45F;
constexpr int kDefaultSkipFrames = 1;
constexpr int kDefaultIntraOpThreads = 1;

std::string shapeToString(const std::vector<int64_t>& shape)
{
    std::ostringstream stream;
    stream << '[';
    for (std::size_t index = 0; index < shape.size(); ++index)
    {
        if (index != 0U)
        {
            stream << ',';
        }
        stream << shape[index];
    }
    stream << ']';
    return stream.str();
}

cv::Scalar colorForClass(const std::string& class_name)
{
    if (class_name == "person")
    {
        return cv::Scalar(255, 170, 0);
    }
    if (class_name == "paper_ball")
    {
        return cv::Scalar(0, 215, 255);
    }
    if (class_name == "box")
    {
        return cv::Scalar(255, 0, 255);
    }
    return cv::Scalar(0, 255, 0);
}

cv::Scalar textColorForBackground(const cv::Scalar& background)
{
    const double luminance = 0.114 * background[0] + 0.587 * background[1] +
                             0.299 * background[2];
    return luminance > 140.0 ? cv::Scalar(0, 0, 0) : cv::Scalar(255, 255, 255);
}

cv::Rect clampRect(const vision_msgs::DetectionResult& detection, int width, int height)
{
    const int x1 = std::max(0, std::min(width, static_cast<int>(std::round(detection.x))));
    const int y1 = std::max(0, std::min(height, static_cast<int>(std::round(detection.y))));
    const int x2 = std::max(
        x1, std::min(width, static_cast<int>(std::round(detection.x + detection.width))));
    const int y2 = std::max(
        y1, std::min(height, static_cast<int>(std::round(detection.y + detection.height))));
    return cv::Rect(x1, y1, x2 - x1, y2 - y1);
}

void drawFilledBackground(cv::Mat* image, const cv::Rect& region, const cv::Scalar& color,
                          double alpha)
{
    if (image == nullptr || image->empty() || region.width <= 0 || region.height <= 0)
    {
        return;
    }
    cv::Mat overlay = image->clone();
    cv::rectangle(overlay, region, color, cv::FILLED);
    cv::addWeighted(overlay, alpha, *image, 1.0 - alpha, 0.0, *image);
}

void drawCornerBox(cv::Mat* image, const cv::Rect& box, const cv::Scalar& color,
                   int thickness)
{
    if (image == nullptr || image->empty())
    {
        return;
    }
    cv::rectangle(*image, box, color, thickness, cv::LINE_AA);

    const int corner = std::max(8, std::min(box.width, box.height) / 5);
    const std::array<cv::Point, 8> starts = {
        cv::Point(box.x, box.y), cv::Point(box.x, box.y),
        cv::Point(box.x + box.width, box.y), cv::Point(box.x + box.width, box.y),
        cv::Point(box.x, box.y + box.height), cv::Point(box.x, box.y + box.height),
        cv::Point(box.x + box.width, box.y + box.height),
        cv::Point(box.x + box.width, box.y + box.height)};
    const std::array<cv::Point, 8> ends = {
        cv::Point(box.x + corner, box.y), cv::Point(box.x, box.y + corner),
        cv::Point(box.x + box.width - corner, box.y),
        cv::Point(box.x + box.width, box.y + corner),
        cv::Point(box.x + corner, box.y + box.height),
        cv::Point(box.x, box.y + box.height - corner),
        cv::Point(box.x + box.width - corner, box.y + box.height),
        cv::Point(box.x + box.width, box.y + box.height - corner)};
    for (std::size_t index = 0; index < starts.size(); ++index)
    {
        cv::line(*image, starts[index], ends[index], color, thickness + 1, cv::LINE_AA);
    }
}

std::vector<yolo_world_detector::Detection> mergeDetections(
    const std::vector<yolo_world_detector::Detection>& primary,
    const std::vector<yolo_world_detector::Detection>& secondary,
    float nms_threshold)
{
    std::vector<yolo_world_detector::Detection> merged = primary;
    merged.insert(merged.end(), secondary.begin(), secondary.end());
    if (merged.empty())
    {
        return merged;
    }

    std::sort(merged.begin(), merged.end(),
              [](const yolo_world_detector::Detection& left,
                 const yolo_world_detector::Detection& right) {
                  return left.confidence > right.confidence;
              });

    std::vector<yolo_world_detector::Detection> filtered;
    filtered.reserve(merged.size());
    for (const auto& candidate : merged)
    {
        const float candidate_area = candidate.box.area();
        bool duplicate = false;
        for (auto& kept : filtered)
        {
            if (candidate.class_id != kept.class_id)
            {
                continue;
            }
            const cv::Rect2f intersection = candidate.box & kept.box;
            const float intersection_area = intersection.area();
            const float kept_area = kept.box.area();
            const float union_area = candidate_area + kept_area - intersection_area;
            const float iou = union_area > 0.0F ? intersection_area / union_area : 0.0F;
            const float candidate_contained = candidate_area > 0.0F
                                                ? intersection_area / candidate_area
                                                : 0.0F;
            const float kept_contained = kept_area > 0.0F
                                             ? intersection_area / kept_area
                                             : 0.0F;
            if (iou >= nms_threshold || candidate_contained >= 0.70F ||
                kept_contained >= 0.80F)
            {
                const bool candidate_is_bottle_shaped =
                    candidate.box.height > 0.0F &&
                    candidate.box.width <= candidate.box.height * 0.85F;
                duplicate = true;
                if (candidate_is_bottle_shaped && candidate_area > kept_area * 1.25F &&
                    kept_contained >= 0.80F)
                {
                    kept = candidate;
                }
                break;
            }
        }
        if (!duplicate)
        {
            filtered.push_back(candidate);
        }
    }
    return filtered;
}

}  // namespace

class YoloWorldDetectorNode
{
public:
    YoloWorldDetectorNode()
        : nh_(),
          private_nh_("~"),
          image_transport_(nh_),
          environment_(ORT_LOGGING_LEVEL_WARNING, "yolo_world_detector"),
          memory_info_(Ort::MemoryInfo::CreateCpu(OrtArenaAllocator, OrtMemTypeDefault)),
          stop_worker_(false),
          frame_count_(0)
    {
        loadParameters();
        initializeSession();

        result_publisher_ = nh_.advertise<vision_msgs::DetectionArray>(result_topic_, 2);
        if (publish_annotated_image_)
        {
            annotated_publisher_ = image_transport_.advertise(annotated_topic_, 1);
        }
        image_subscriber_ = image_transport_.subscribe(
            image_topic_, 1, &YoloWorldDetectorNode::imageCallback, this);
        worker_thread_ = std::thread(&YoloWorldDetectorNode::inferenceLoop, this);

        ROS_INFO("YOLO-World detector ready: model=%s classes=%zu input=%dx%d threads=%d",
                 model_path_.c_str(), class_names_.size(), input_width_, input_height_,
                 intra_op_threads_);
        ROS_INFO("YOLO-World topics: image=%s result=%s annotated=%s", image_topic_.c_str(),
                 result_topic_.c_str(),
                 publish_annotated_image_ ? annotated_topic_.c_str() : "disabled");
    }

    ~YoloWorldDetectorNode()
    {
        stop_worker_.store(true);
        frame_condition_.notify_all();
        if (worker_thread_.joinable())
        {
            worker_thread_.join();
        }
    }

private:
    void loadParameters()
    {
        const std::string package_path = ros::package::getPath("yolo_world_detector");
        private_nh_.param<std::string>("model_path", model_path_,
                                       "/home/hyc/robocup_vision/yolov8s-worldv2.onnx");
        private_nh_.param<std::string>("class_names_path", class_names_path_,
                                       package_path + "/config/classes.txt");
        private_nh_.param<std::string>("image_topic", image_topic_,
                                       "/kinect2/hd/image_color");
        private_nh_.param<std::string>("result_topic", result_topic_, "/yolo_world/detections");
        private_nh_.param<std::string>("annotated_topic", annotated_topic_,
                                       "/yolo_world/annotated_image");
        private_nh_.param("publish_annotated_image", publish_annotated_image_, true);
        private_nh_.param("confidence_threshold", confidence_threshold_,
                          kDefaultConfidenceThreshold);
        private_nh_.param("nms_threshold", nms_threshold_, kDefaultNmsThreshold);
        private_nh_.param("input_width", input_width_, kDefaultInputWidth);
        private_nh_.param("input_height", input_height_, kDefaultInputHeight);
        private_nh_.param("skip_frames", skip_frames_, kDefaultSkipFrames);
        private_nh_.param("intra_op_threads", intra_op_threads_, kDefaultIntraOpThreads);
        private_nh_.param<std::string>("execution_provider", execution_provider_, "cpu");
        private_nh_.param("cuda_device_id", cuda_device_id_, 0);
        private_nh_.param("temporal_hold_minimum_count", temporal_hold_minimum_count_, 0);
        private_nh_.param("temporal_hold_seconds", temporal_hold_seconds_, 0.0);
        private_nh_.param("fallback_center_crop_scale", fallback_center_crop_scale_, 1.0);
        private_nh_.param<std::string>("fallback_crop_mode", fallback_crop_mode_, "wide");

        if (model_path_.empty() || class_names_path_.empty())
        {
            throw std::invalid_argument("model_path and class_names_path must not be empty");
        }
        if (image_topic_.empty() || result_topic_.empty() ||
            (publish_annotated_image_ && annotated_topic_.empty()))
        {
            throw std::invalid_argument("configured ROS topics must not be empty");
        }
        if (confidence_threshold_ <= 0.0F || confidence_threshold_ > 1.0F ||
            nms_threshold_ <= 0.0F || nms_threshold_ > 1.0F)
        {
            throw std::invalid_argument("confidence and NMS thresholds must be in (0, 1]");
        }
        if (input_width_ != kDefaultInputWidth || input_height_ != kDefaultInputHeight)
        {
            throw std::invalid_argument(
                "this exported YOLO-World model requires input_width=640 and input_height=640");
        }
        if (skip_frames_ <= 0 || intra_op_threads_ <= 0)
        {
            throw std::invalid_argument("skip_frames and threads must be positive");
        }
        if (execution_provider_ != "cpu" && execution_provider_ != "cuda" &&
            execution_provider_ != "auto")
        {
            throw std::invalid_argument(
                "execution_provider must be one of: cpu, cuda, auto");
        }
        if (cuda_device_id_ < 0)
        {
            throw std::invalid_argument("cuda_device_id must be non-negative");
        }
        if (temporal_hold_minimum_count_ < 0 || temporal_hold_seconds_ < 0.0 ||
            fallback_center_crop_scale_ < 1.0)
        {
            throw std::invalid_argument(
                "temporal hold values must be non-negative and fallback crop scale must be at least 1.0");
        }
        if (fallback_crop_mode_ != "wide" && fallback_crop_mode_ != "center_bottom" &&
            fallback_crop_mode_ != "center" && fallback_crop_mode_ != "bottom")
        {
            throw std::invalid_argument(
                "fallback_crop_mode must be one of: wide, center_bottom, center, bottom");
        }

        class_names_ = yolo_world_detector::loadClassNames(class_names_path_);
        if (class_names_.empty())
        {
            throw std::runtime_error("YOLO-World configuration must contain at least one class");
        }
        input_shape_ = {1, 3, static_cast<int64_t>(input_height_),
                        static_cast<int64_t>(input_width_)};
    }

    void initializeSession()
    {
        Ort::SessionOptions options;
        options.SetIntraOpNumThreads(intra_op_threads_);
        options.SetGraphOptimizationLevel(GraphOptimizationLevel::ORT_ENABLE_ALL);
        configureExecutionProvider(options);
        session_.reset(new Ort::Session(environment_, model_path_.c_str(), options));

        if (session_->GetInputCount() != 1U || session_->GetOutputCount() != 1U)
        {
            throw std::runtime_error("expected exactly one model input and output");
        }

        const auto input_type_info = session_->GetInputTypeInfo(0);
        const auto output_type_info = session_->GetOutputTypeInfo(0);
        const auto input_info = input_type_info.GetTensorTypeAndShapeInfo();
        const auto output_info = output_type_info.GetTensorTypeAndShapeInfo();
        if (input_info.GetElementType() != ONNX_TENSOR_ELEMENT_DATA_TYPE_FLOAT ||
            output_info.GetElementType() != ONNX_TENSOR_ELEMENT_DATA_TYPE_FLOAT)
        {
            throw std::runtime_error("model input and output tensors must use float32");
        }

        const std::vector<int64_t> model_input_shape = input_info.GetShape();
        if (model_input_shape != std::vector<int64_t>({1, 3, 640, 640}))
        {
            throw std::runtime_error("model input shape " + shapeToString(model_input_shape) +
                                     " does not match required [1,3,640,640]");
        }

        const std::vector<int64_t> output_shape = output_info.GetShape();
        const std::vector<int64_t> required_static_output_shape = {
            1, static_cast<int64_t>(4U + class_names_.size()), 8400};
        const bool output_shape_compatible =
            output_shape.size() == required_static_output_shape.size() &&
            (output_shape[0] == required_static_output_shape[0] || output_shape[0] < 0) &&
            (output_shape[1] == required_static_output_shape[1] || output_shape[1] < 0) &&
            output_shape[2] == required_static_output_shape[2];
        if (!output_shape_compatible)
        {
            throw std::runtime_error("model output shape " + shapeToString(output_shape) +
                                     " does not match required [1," +
                                     std::to_string(required_static_output_shape[1]) +
                                     ",8400] (dynamic batch/channel dims are allowed)");
        }

        Ort::AllocatorWithDefaultOptions allocator;
        input_name_ = session_->GetInputNameAllocated(0, allocator).get();
        output_name_ = session_->GetOutputNameAllocated(0, allocator).get();
        if (input_name_ != "images" || output_name_ != "output0")
        {
            throw std::runtime_error("model tensor names must be images and output0; got " +
                                     input_name_ + " and " + output_name_);
        }
        input_names_[0] = input_name_.c_str();
        output_names_[0] = output_name_.c_str();
        ROS_INFO("ONNX model loaded: input=%s output=%s shape=%s provider=%s", input_name_.c_str(),
                 output_name_.c_str(), shapeToString(output_shape).c_str(),
                 active_execution_provider_.c_str());
    }

    void configureExecutionProvider(Ort::SessionOptions& options)
    {
        active_execution_provider_ = "cpu";
        if (execution_provider_ == "cpu")
        {
            ROS_INFO("Using ONNX Runtime CPUExecutionProvider");
            return;
        }

        OrtCUDAProviderOptions cuda_options;
        cuda_options.device_id = cuda_device_id_;
        try
        {
            options.AppendExecutionProvider_CUDA(cuda_options);
            active_execution_provider_ = "cuda";
            ROS_INFO("Using ONNX Runtime CUDAExecutionProvider device=%d", cuda_device_id_);
        }
        catch (const Ort::Exception& exception)
        {
            if (execution_provider_ == "cuda")
            {
                throw std::runtime_error(
                    std::string("requested CUDAExecutionProvider, but ONNX Runtime failed to enable it: ") +
                    exception.what());
            }
            ROS_WARN("CUDAExecutionProvider unavailable, falling back to CPUExecutionProvider: %s",
                     exception.what());
        }
    }

    void imageCallback(const sensor_msgs::ImageConstPtr& message)
    {
        if (!message)
        {
            ROS_ERROR_THROTTLE(2.0, "Received a null image message");
            return;
        }
        const std::size_t current_frame = ++frame_count_;
        if (current_frame % static_cast<std::size_t>(skip_frames_) != 0U)
        {
            return;
        }
        {
            std::lock_guard<std::mutex> lock(frame_mutex_);
            latest_frame_ = message;
        }
        frame_condition_.notify_one();
    }

    void inferenceLoop()
    {
        while (true)
        {
            sensor_msgs::ImageConstPtr message;
            {
                std::unique_lock<std::mutex> lock(frame_mutex_);
                frame_condition_.wait(lock, [this] {
                    return stop_worker_.load() || latest_frame_ != nullptr;
                });
                if (stop_worker_.load())
                {
                    return;
                }
                message = std::move(latest_frame_);
                latest_frame_.reset();
            }
            processImage(message);
        }
    }

    std::vector<yolo_world_detector::Detection> runModel(const cv::Mat& image,
                                                          cv::Mat& blob)
    {
        const auto letterboxed = yolo_world_detector::letterbox(
            image, input_width_, input_height_);
        cv::dnn::blobFromImage(letterboxed.image, blob, 1.0 / 255.0,
                               cv::Size(), cv::Scalar(), true, false, CV_32F);
        Ort::Value input_value = Ort::Value::CreateTensor<float>(
            memory_info_, blob.ptr<float>(), blob.total(), input_shape_.data(),
            input_shape_.size());
        std::vector<Ort::Value> outputs = session_->Run(
            Ort::RunOptions{nullptr}, input_names_.data(), &input_value, 1,
            output_names_.data(), 1);
        if (outputs.size() != 1U || !outputs.front().IsTensor())
        {
            throw std::runtime_error("model did not return one tensor output");
        }

        const auto info = outputs.front().GetTensorTypeAndShapeInfo();
        return yolo_world_detector::decodeWorldOutput(
            outputs.front().GetTensorData<float>(), info.GetShape(),
            class_names_.size(), confidence_threshold_, nms_threshold_, letterboxed,
            image.cols, image.rows);
    }

    std::vector<yolo_world_detector::Detection> detectWithOptionalFallback(
        const cv::Mat& image)
    {
        std::vector<yolo_world_detector::Detection> detections = runModel(
            image, input_blob_);
        const auto paper_ball_iterator = std::find(
            class_names_.begin(), class_names_.end(), "paper_ball");
        if (paper_ball_iterator == class_names_.end())
        {
            return detections;
        }
        const int paper_ball_class_id = static_cast<int>(
            std::distance(class_names_.begin(), paper_ball_iterator));
        const std::size_t paper_ball_count = static_cast<std::size_t>(
            std::count_if(detections.begin(), detections.end(),
                          [paper_ball_class_id](const auto& detection) {
                              return detection.class_id == paper_ball_class_id;
                          }));
        const std::size_t target_count = static_cast<std::size_t>(
            std::max(0, temporal_hold_minimum_count_));
        if (fallback_center_crop_scale_ <= 1.0 || target_count == 0U ||
            paper_ball_count >= target_count)
        {
            return detections;
        }

        const int crop_width = static_cast<int>(
            std::round(static_cast<double>(image.cols) / fallback_center_crop_scale_));
        const int crop_height = static_cast<int>(
            std::round(static_cast<double>(image.rows) / fallback_center_crop_scale_));
        if (crop_width <= 0 || crop_height <= 0 || crop_width > image.cols ||
            crop_height > image.rows)
        {
            return detections;
        }

        const int center_x = (image.cols - crop_width) / 2;
        const int center_y = (image.rows - crop_height) / 2;
        const int bottom_y = image.rows - crop_height;

        std::vector<cv::Point> crop_origins;
        if (fallback_crop_mode_ == "center")
        {
            crop_origins.emplace_back(center_x, center_y);
        }
        else if (fallback_crop_mode_ == "bottom")
        {
            crop_origins.emplace_back(center_x, bottom_y);
        }
        else if (fallback_crop_mode_ == "center_bottom")
        {
            crop_origins.emplace_back(center_x, center_y);
            crop_origins.emplace_back(center_x, bottom_y);
        }
        else
        {
            crop_origins.emplace_back(center_x, center_y);
            crop_origins.emplace_back(0, center_y);
            crop_origins.emplace_back(image.cols - crop_width, center_y);
            crop_origins.emplace_back(center_x, bottom_y);
            crop_origins.emplace_back(0, bottom_y);
            crop_origins.emplace_back(image.cols - crop_width, bottom_y);
        }

        std::vector<yolo_world_detector::Detection> remapped;
        for (const auto& crop_origin : crop_origins)
        {
            const cv::Rect crop_region(crop_origin.x, crop_origin.y, crop_width, crop_height);
            const std::vector<yolo_world_detector::Detection> crop_detections =
                runModel(image(crop_region), fallback_input_blob_);
            remapped.reserve(remapped.size() + crop_detections.size());
            for (auto detection : crop_detections)
            {
                detection.box.x += static_cast<float>(crop_origin.x);
                detection.box.y += static_cast<float>(crop_origin.y);
                remapped.push_back(detection);
            }
        }
        if (!remapped.empty())
        {
            ROS_WARN_THROTTLE(
                2.0,
                "YOLO-World fallback crops added %zu candidate detections",
                remapped.size());
        }
        return mergeDetections(detections, remapped, nms_threshold_);
    }

    std::vector<vision_msgs::DetectionResult> convertDetections(
        const std::vector<yolo_world_detector::Detection>& decoded)
    {
        std::vector<vision_msgs::DetectionResult> detections;
        detections.reserve(decoded.size());
        for (const auto& candidate : decoded)
        {
            vision_msgs::DetectionResult detection;
            detection.class_name = class_names_.at(
                static_cast<std::size_t>(candidate.class_id));
            detection.confidence = candidate.confidence;
            detection.x = candidate.box.x;
            detection.y = candidate.box.y;
            detection.width = candidate.box.width;
            detection.height = candidate.box.height;
            detection.has_bbox = true;
            detections.push_back(std::move(detection));
        }
        return detections;
    }

    std::vector<vision_msgs::DetectionResult> applyTemporalHoldAndConvert(
        const std::vector<yolo_world_detector::Detection>& raw_decoded,
        const ros::Time& stamp)
    {
        bool used_temporal_hold = false;
        const std::vector<yolo_world_detector::Detection> decoded =
            yolo_world_detector::applyTemporalHold(
                raw_decoded,
                static_cast<std::size_t>(std::max(0, temporal_hold_minimum_count_)),
                temporal_hold_seconds_, stamp.toSec(), &temporal_hold_state_,
                &used_temporal_hold);
        if (used_temporal_hold)
        {
            ROS_WARN_THROTTLE(
                2.0,
                "YOLO-World temporal hold reused %zu detections for a sparse frame",
                decoded.size());
        }
        return convertDetections(decoded);
    }

    void publishAnnotatedImage(const std_msgs::Header& header, const cv::Mat& source,
                               const std::vector<vision_msgs::DetectionResult>& detections)
    {
        if (!publish_annotated_image_)
        {
            return;
        }
        cv::Mat annotated = source.clone();
        for (const auto& detection : detections)
        {
            const cv::Rect box = clampRect(detection, annotated.cols, annotated.rows);
            if (box.width <= 0 || box.height <= 0)
            {
                continue;
            }

            const cv::Scalar color = colorForClass(detection.class_name);
            drawCornerBox(&annotated, box, color, 2);
            std::ostringstream label_stream;
            label_stream << detection.class_name << ' ' << std::fixed
                         << std::setprecision(2) << detection.confidence;
            const std::string label = label_stream.str();
            int baseline = 0;
            const cv::Size label_size = cv::getTextSize(
                label, cv::FONT_HERSHEY_SIMPLEX, 0.55, 1, &baseline);
            const int label_x = std::max(0, box.x);
            const int label_y = std::max(label_size.height + baseline, box.y);
            const cv::Rect background(label_x, label_y - label_size.height - baseline,
                                      std::min(label_size.width + 6,
                                               annotated.cols - label_x),
                                      label_size.height + baseline);
            if (background.width > 0 && background.height > 0)
            {
                drawFilledBackground(&annotated, background, color, 0.82);
            }
            cv::putText(annotated, label, cv::Point(label_x + 3, label_y - baseline),
                        cv::FONT_HERSHEY_SIMPLEX, 0.55, textColorForBackground(color),
                        1, cv::LINE_AA);
        }
        annotated_publisher_.publish(
            cv_bridge::CvImage(header, sensor_msgs::image_encodings::BGR8, annotated)
                .toImageMsg());
    }

    void processImage(const sensor_msgs::ImageConstPtr& message)
    {
        const auto started_at = std::chrono::steady_clock::now();
        try
        {
            const auto cv_ptr = cv_bridge::toCvCopy(
                message, sensor_msgs::image_encodings::BGR8);
            if (!cv_ptr || cv_ptr->image.empty())
            {
                ROS_ERROR_THROTTLE(2.0, "Received an empty image");
                return;
            }

            const std::vector<yolo_world_detector::Detection> raw_detections =
                detectWithOptionalFallback(cv_ptr->image);

            vision_msgs::DetectionArray result;
            result.header = message->header;
            result.backend = "yolo_world_onnxruntime";
            result.image_width = static_cast<std::uint32_t>(cv_ptr->image.cols);
            result.image_height = static_cast<std::uint32_t>(cv_ptr->image.rows);
            result.objects = applyTemporalHoldAndConvert(raw_detections,
                                                         message->header.stamp);
            result_publisher_.publish(result);
            publishAnnotatedImage(message->header, cv_ptr->image, result.objects);

            const std::chrono::duration<double, std::milli> elapsed =
                std::chrono::steady_clock::now() - started_at;
            ROS_INFO_THROTTLE(2.0,
                              "YOLO-World processing: %.1f ms (%.1f FPS), detections=%zu",
                              elapsed.count(), elapsed.count() > 0.0
                                                   ? 1000.0 / elapsed.count()
                                                   : 0.0,
                              result.objects.size());
        }
        catch (const cv_bridge::Exception& exception)
        {
            ROS_ERROR_THROTTLE(2.0, "cv_bridge error: %s", exception.what());
        }
        catch (const cv::Exception& exception)
        {
            ROS_ERROR_THROTTLE(2.0, "OpenCV error: %s", exception.what());
        }
        catch (const Ort::Exception& exception)
        {
            ROS_ERROR_THROTTLE(2.0, "ONNX Runtime error: %s", exception.what());
        }
        catch (const std::exception& exception)
        {
            ROS_ERROR_THROTTLE(2.0, "YOLO-World inference error: %s", exception.what());
        }
    }

    ros::NodeHandle nh_;
    ros::NodeHandle private_nh_;
    image_transport::ImageTransport image_transport_;
    image_transport::Subscriber image_subscriber_;
    image_transport::Publisher annotated_publisher_;
    ros::Publisher result_publisher_;

    Ort::Env environment_;
    Ort::MemoryInfo memory_info_;
    std::unique_ptr<Ort::Session> session_;
    std::string input_name_;
    std::string output_name_;
    std::array<const char*, 1> input_names_{{nullptr}};
    std::array<const char*, 1> output_names_{{nullptr}};
    std::array<int64_t, 4> input_shape_{{1, 3, 640, 640}};
    cv::Mat input_blob_;
    cv::Mat fallback_input_blob_;

    std::thread worker_thread_;
    std::mutex frame_mutex_;
    std::condition_variable frame_condition_;
    sensor_msgs::ImageConstPtr latest_frame_;
    std::atomic<bool> stop_worker_;
    std::atomic<std::size_t> frame_count_;

    std::string model_path_;
    std::string class_names_path_;
    std::vector<std::string> class_names_;
    std::string image_topic_;
    std::string result_topic_;
    std::string annotated_topic_;
    bool publish_annotated_image_ = true;
    float confidence_threshold_ = kDefaultConfidenceThreshold;
    float nms_threshold_ = kDefaultNmsThreshold;
    int input_width_ = kDefaultInputWidth;
    int input_height_ = kDefaultInputHeight;
    int skip_frames_ = kDefaultSkipFrames;
    int intra_op_threads_ = kDefaultIntraOpThreads;
    std::string execution_provider_ = "cpu";
    std::string active_execution_provider_ = "cpu";
    int cuda_device_id_ = 0;
    int temporal_hold_minimum_count_ = 0;
    double temporal_hold_seconds_ = 0.0;
    double fallback_center_crop_scale_ = 1.0;
    std::string fallback_crop_mode_ = "wide";
    yolo_world_detector::TemporalHoldState temporal_hold_state_;
};

int main(int argc, char** argv)
{
    ros::init(argc, argv, "yolo_world_detector_node");
    try
    {
        YoloWorldDetectorNode node;
        ros::spin();
    }
    catch (const Ort::Exception& exception)
    {
        ROS_FATAL("ONNX Runtime initialization failed: %s", exception.what());
        std::cerr << "ONNX Runtime initialization failed: " << exception.what()
                  << std::endl;
        return 1;
    }
    catch (const std::exception& exception)
    {
        ROS_FATAL("YOLO-World detector initialization failed: %s", exception.what());
        std::cerr << "YOLO-World detector initialization failed: " << exception.what()
                  << std::endl;
        return 1;
    }
    return 0;
}
