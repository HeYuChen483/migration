#include <algorithm>
#include <cmath>
#include <cstring>
#include <cstdint>
#include <deque>
#include <limits>
#include <stdexcept>
#include <string>
#include <vector>

#include <cv_bridge/cv_bridge.h>
#include <geometry_msgs/TransformStamped.h>
#include <opencv2/core.hpp>
#include <opencv2/imgproc.hpp>
#include <ros/ros.h>
#include <sensor_msgs/CameraInfo.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/image_encodings.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/PointField.h>
#include <tf2/LinearMath/Transform.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <vision_msgs/DetectionArray.h>
#include <vision_msgs/DetectionResult.h>

namespace
{

struct Point3
{
    double x{0.0};
    double y{0.0};
    double z{0.0};
};

double clampDouble(double value, double low, double high)
{
    return std::max(low, std::min(high, value));
}

bool validBox(const vision_msgs::DetectionResult& detection)
{
    return detection.has_bbox && detection.width > 0.0F && detection.height > 0.0F &&
           std::isfinite(detection.x) && std::isfinite(detection.y) &&
           std::isfinite(detection.width) && std::isfinite(detection.height);
}

float area(const vision_msgs::DetectionResult& detection)
{
    if (!validBox(detection))
    {
        return 0.0F;
    }
    return detection.width * detection.height;
}

float iou(const vision_msgs::DetectionResult& left,
          const vision_msgs::DetectionResult& right)
{
    if (!validBox(left) || !validBox(right))
    {
        return 0.0F;
    }

    const float left_x2 = left.x + left.width;
    const float left_y2 = left.y + left.height;
    const float right_x2 = right.x + right.width;
    const float right_y2 = right.y + right.height;

    const float ix1 = std::max(left.x, right.x);
    const float iy1 = std::max(left.y, right.y);
    const float ix2 = std::min(left_x2, right_x2);
    const float iy2 = std::min(left_y2, right_y2);
    const float iw = std::max(0.0F, ix2 - ix1);
    const float ih = std::max(0.0F, iy2 - iy1);
    const float intersection = iw * ih;
    const float union_area = area(left) + area(right) - intersection;
    return union_area > 0.0F ? intersection / union_area : 0.0F;
}

ros::Time effectiveStamp(const std_msgs::Header& header)
{
    return header.stamp.isZero() ? ros::Time::now() : header.stamp;
}

bool closeStamp(const ros::Time& left, const ros::Time& right,
                double max_delta_seconds)
{
    if (left.isZero() || right.isZero())
    {
        return true;
    }
    return std::abs((left - right).toSec()) <= max_delta_seconds;
}

double median(std::vector<double>* values)
{
    if (values == nullptr || values->empty())
    {
        return std::numeric_limits<double>::quiet_NaN();
    }
    const std::size_t mid = values->size() / 2U;
    std::nth_element(values->begin(), values->begin() + mid, values->end());
    double result = values->at(mid);
    if (values->size() % 2U == 0U)
    {
        std::nth_element(values->begin(), values->begin() + mid - 1U,
                         values->end());
        result = 0.5 * (result + values->at(mid - 1U));
    }
    return result;
}

double depthMetersAt(const cv::Mat& depth, int x, int y, double depth_unit)
{
    if (x < 0 || y < 0 || x >= depth.cols || y >= depth.rows)
    {
        return std::numeric_limits<double>::quiet_NaN();
    }
    if (depth.type() == CV_16UC1)
    {
        const auto raw = depth.at<std::uint16_t>(y, x);
        return raw == 0U ? std::numeric_limits<double>::quiet_NaN()
                         : static_cast<double>(raw) * depth_unit;
    }
    if (depth.type() == CV_32FC1)
    {
        return static_cast<double>(depth.at<float>(y, x));
    }
    if (depth.type() == CV_64FC1)
    {
        return depth.at<double>(y, x);
    }
    return std::numeric_limits<double>::quiet_NaN();
}

bool finitePoint(const Point3& point)
{
    return std::isfinite(point.x) && std::isfinite(point.y) && std::isfinite(point.z);
}

bool floatFieldOffset(const sensor_msgs::PointCloud2& cloud,
                      const std::string& field_name,
                      std::uint32_t* offset)
{
    if (offset == nullptr)
    {
        return false;
    }
    for (const auto& field : cloud.fields)
    {
        if (field.name == field_name &&
            field.datatype == sensor_msgs::PointField::FLOAT32 && field.count >= 1U)
        {
            *offset = field.offset;
            return true;
        }
    }
    return false;
}

bool cloudPointAt(const sensor_msgs::PointCloud2& cloud, int x, int y,
                  std::uint32_t x_offset, std::uint32_t y_offset,
                  std::uint32_t z_offset, Point3* point)
{
    if (point == nullptr || x < 0 || y < 0 ||
        x >= static_cast<int>(cloud.width) || y >= static_cast<int>(cloud.height) ||
        cloud.point_step == 0U || cloud.row_step == 0U)
    {
        return false;
    }
    const std::size_t base = static_cast<std::size_t>(y) * cloud.row_step +
                             static_cast<std::size_t>(x) * cloud.point_step;
    const std::size_t required = base +
        std::max({x_offset, y_offset, z_offset}) + sizeof(float);
    if (required > cloud.data.size())
    {
        return false;
    }

    float x_value = 0.0F;
    float y_value = 0.0F;
    float z_value = 0.0F;
    std::memcpy(&x_value, &cloud.data[base + x_offset], sizeof(float));
    std::memcpy(&y_value, &cloud.data[base + y_offset], sizeof(float));
    std::memcpy(&z_value, &cloud.data[base + z_offset], sizeof(float));
    point->x = static_cast<double>(x_value);
    point->y = static_cast<double>(y_value);
    point->z = static_cast<double>(z_value);
    return finitePoint(*point);
}

std::string appendBackend(const std::string& backend)
{
    if (backend.empty())
    {
        return "paper_ball_rgbd_vote";
    }
    return backend + "+paper_ball_rgbd_vote";
}

}  // namespace

class PaperBallRgbdVoteNode
{
public:
    PaperBallRgbdVoteNode()
        : nh_(), private_nh_("~"), tf_listener_(tf_buffer_)
    {
        loadParameters();

        detection_subscriber_ = nh_.subscribe(
            detections_topic_, 2, &PaperBallRgbdVoteNode::detectionsCallback, this);
        depth_subscriber_ = nh_.subscribe(
            depth_topic_, 1, &PaperBallRgbdVoteNode::depthCallback, this);
        point_cloud_subscriber_ = nh_.subscribe(
            point_cloud_topic_, 1, &PaperBallRgbdVoteNode::pointCloudCallback, this);
        camera_info_subscriber_ = nh_.subscribe(
            camera_info_topic_, 1, &PaperBallRgbdVoteNode::cameraInfoCallback, this);
        if (enable_hsv_ground_filter_)
        {
            image_subscriber_ = nh_.subscribe(
                image_topic_, 1, &PaperBallRgbdVoteNode::imageCallback, this);
        }
        publisher_ = nh_.advertise<vision_msgs::DetectionArray>(output_topic_, 2);
        if (!ground_mask_debug_topic_.empty())
        {
            ground_mask_publisher_ =
                nh_.advertise<sensor_msgs::Image>(ground_mask_debug_topic_, 1);
        }

        ROS_INFO("paper_ball RGB-D vote ready: detections=%s image=%s depth=%s cloud=%s camera_info=%s output=%s low=%.2f immediate=%.2f votes=%d window=%.2fs hold=%.2fs require_depth_for_vote=%s ground_z_filter=%s ground_z=[%.2f,%.2f] hsv_ground_filter=%s",
                 detections_topic_.c_str(), image_topic_.c_str(), depth_topic_.c_str(),
                 point_cloud_topic_.c_str(), camera_info_topic_.c_str(),
                 output_topic_.c_str(),
                 low_confidence_threshold_, immediate_confidence_threshold_,
                 minimum_votes_, vote_window_seconds_, hold_seconds_,
                 require_depth_for_vote_ ? "true" : "false",
                 enable_ground_z_filter_ ? "true" : "false",
                 min_ground_z_, max_ground_z_,
                 enable_hsv_ground_filter_ ? "true" : "false");
    }

private:
    struct Candidate
    {
        vision_msgs::DetectionResult detection;
        bool has_depth{false};
        bool depth_ok{false};
        double x_m{0.0};
        double y_m{0.0};
        double z_m{0.0};
        double physical_width_m{0.0};
        double physical_height_m{0.0};
        double center_x_px{0.0};
        double center_y_px{0.0};
        double depth_mad_m{0.0};
        int depth_samples{0};
        bool has_ground_mask{false};
        bool ground_mask_ok{true};
        double ground_contact_ratio{0.0};
        double ground_below_ratio{0.0};
        bool has_output_point{false};
        double output_x_m{0.0};
        double output_y_m{0.0};
        double output_z_m{0.0};
    };

    struct VoteTrack
    {
        std::uint32_t id{0U};
        ros::Time first_seen;
        ros::Time last_seen;
        ros::Time last_published;
        int hits{0};
        bool has_depth{false};
        double x_m{0.0};
        double y_m{0.0};
        double z_m{0.0};
        bool has_output_point{false};
        double output_x_m{0.0};
        double output_y_m{0.0};
        double output_z_m{0.0};
        double center_x_px{0.0};
        double center_y_px{0.0};
        vision_msgs::DetectionResult best_detection;
    };

    void loadParameters()
    {
        private_nh_.param<std::string>("detections_topic", detections_topic_,
                                       "/yolo_world/paper_box_secondary_detections");
        private_nh_.param<std::string>("image_topic", image_topic_,
                                       "/kinect2/hd/image_color");
        private_nh_.param<std::string>("depth_topic", depth_topic_,
                                       "/kinect2/hd/image_depth_rect");
        private_nh_.param<std::string>("point_cloud_topic", point_cloud_topic_,
                                       "/kinect2/hd/points");
        private_nh_.param<std::string>("camera_info_topic", camera_info_topic_,
                                       "/kinect2/hd/camera_info");
        private_nh_.param<std::string>("output_topic", output_topic_,
                                       "/yolo_world/paper_box_secondary_rgbd_voted_detections");
        private_nh_.param<std::string>("output_frame", output_frame_, "base_link");

        private_nh_.param("low_confidence_threshold", low_confidence_threshold_, 0.06F);
        private_nh_.param("immediate_confidence_threshold",
                          immediate_confidence_threshold_, 0.10F);
        private_nh_.param("publish_confidence_floor", publish_confidence_floor_, 0.10F);
        private_nh_.param("minimum_votes", minimum_votes_, 2);
        private_nh_.param("vote_window_seconds", vote_window_seconds_, 0.90);
        private_nh_.param("hold_seconds", hold_seconds_, 0.35);
        private_nh_.param("max_depth_time_delta", max_depth_time_delta_, 0.20);
        private_nh_.param("max_cloud_time_delta", max_cloud_time_delta_, 0.15);
        private_nh_.param("tf_timeout", tf_timeout_, 0.05);
        private_nh_.param("max_3d_match_distance", max_3d_match_distance_, 0.18);
        private_nh_.param("max_image_match_distance", max_image_match_distance_, 28.0);
        private_nh_.param("match_iou_threshold", match_iou_threshold_, 0.20F);
        private_nh_.param("depth_unit", depth_unit_, 0.001);
        private_nh_.param("min_depth", min_depth_, 0.35);
        private_nh_.param("max_depth", max_depth_, 6.00);
        private_nh_.param("min_depth_samples", min_depth_samples_, 5);
        private_nh_.param("depth_window_min_pixels", depth_window_min_pixels_, 7);
        private_nh_.param("depth_window_scale", depth_window_scale_, 1.8);
        private_nh_.param("max_depth_mad", max_depth_mad_, 0.25);
        private_nh_.param("min_physical_width", min_physical_width_, 0.006);
        private_nh_.param("max_physical_width", max_physical_width_, 0.22);
        private_nh_.param("min_physical_height", min_physical_height_, 0.006);
        private_nh_.param("max_physical_height", max_physical_height_, 0.22);
        private_nh_.param("min_center_y_ratio", min_center_y_ratio_, 0.20);
        private_nh_.param("max_aspect_ratio", max_aspect_ratio_, 2.60);
        private_nh_.param("min_ground_z", min_ground_z_, -0.60);
        private_nh_.param("max_ground_z", max_ground_z_, 0.25);
        private_nh_.param("prefer_point_cloud", prefer_point_cloud_, true);
        private_nh_.param("use_point_cloud", use_point_cloud_, true);
        private_nh_.param("enable_ground_z_filter", enable_ground_z_filter_, true);
        private_nh_.param("require_output_frame_for_vote",
                          require_output_frame_for_vote_, false);
        private_nh_.param("require_depth_for_vote", require_depth_for_vote_, true);
        private_nh_.param("filter_high_confidence_with_depth",
                          filter_high_confidence_with_depth_, true);
        private_nh_.param("allow_high_confidence_without_depth",
                          allow_high_confidence_without_depth_, true);
        private_nh_.param("enable_hsv_ground_filter", enable_hsv_ground_filter_, true);
        private_nh_.param("require_hsv_ground_filter", require_hsv_ground_filter_, true);
        private_nh_.param("max_hsv_image_time_delta", max_hsv_image_time_delta_, 0.50);
        private_nh_.param("hsv_ground_max_saturation", hsv_ground_max_saturation_, 70);
        private_nh_.param("hsv_ground_min_value", hsv_ground_min_value_, 20);
        private_nh_.param("hsv_ground_max_value", hsv_ground_max_value_, 255);
        private_nh_.param("enable_hsv_clahe", enable_hsv_clahe_, true);
        private_nh_.param("hsv_clahe_clip_limit", hsv_clahe_clip_limit_, 2.0);
        private_nh_.param("hsv_clahe_tile_grid_size", hsv_clahe_tile_grid_size_, 8);
        private_nh_.param("ground_mask_morph_kernel", ground_mask_morph_kernel_, 5);
        private_nh_.param("ground_mask_bottom_band_ratio", ground_mask_bottom_band_ratio_, 0.10);
        private_nh_.param("ground_mask_min_bottom_pixels", ground_mask_min_bottom_pixels_, 10);
        private_nh_.param("ground_contact_band_ratio", ground_contact_band_ratio_, 0.30);
        private_nh_.param("ground_contact_min_ratio", ground_contact_min_ratio_, 0.03);
        private_nh_.param("ground_below_band_ratio", ground_below_band_ratio_, 0.18);
        private_nh_.param("ground_below_width_ratio", ground_below_width_ratio_, 0.70);
        private_nh_.param("ground_below_min_ratio", ground_below_min_ratio_, 0.10);
        private_nh_.param<std::string>("ground_mask_debug_topic", ground_mask_debug_topic_,
                                       "/yolo_world/paper_ball_ground_mask");

        if (detections_topic_.empty() || depth_topic_.empty() || point_cloud_topic_.empty() ||
            camera_info_topic_.empty() || output_topic_.empty() || output_frame_.empty() ||
            (enable_hsv_ground_filter_ && image_topic_.empty()) ||
            low_confidence_threshold_ < 0.0F || low_confidence_threshold_ > 1.0F ||
            immediate_confidence_threshold_ < low_confidence_threshold_ ||
            immediate_confidence_threshold_ > 1.0F ||
            publish_confidence_floor_ < immediate_confidence_threshold_ ||
            publish_confidence_floor_ > 1.0F || minimum_votes_ < 1 ||
            vote_window_seconds_ <= 0.0 || hold_seconds_ < 0.0 ||
            max_depth_time_delta_ < 0.0 || max_cloud_time_delta_ < 0.0 ||
            tf_timeout_ < 0.0 || max_3d_match_distance_ <= 0.0 ||
            max_image_match_distance_ <= 0.0 || match_iou_threshold_ <= 0.0F ||
            match_iou_threshold_ > 1.0F || depth_unit_ <= 0.0 ||
            min_depth_ <= 0.0 || max_depth_ <= min_depth_ ||
            min_depth_samples_ < 1 || depth_window_min_pixels_ < 1 ||
            depth_window_scale_ < 1.0 || max_depth_mad_ < 0.0 ||
            min_physical_width_ < 0.0 || max_physical_width_ <= min_physical_width_ ||
            min_physical_height_ < 0.0 ||
            max_physical_height_ <= min_physical_height_ ||
            min_center_y_ratio_ < 0.0 || min_center_y_ratio_ > 1.0 ||
            max_aspect_ratio_ <= 0.0 || max_ground_z_ <= min_ground_z_ ||
            max_hsv_image_time_delta_ < 0.0 || hsv_ground_max_saturation_ < 0 ||
            hsv_ground_max_saturation_ > 255 || hsv_ground_min_value_ < 0 ||
            hsv_ground_min_value_ > 255 || hsv_ground_max_value_ < hsv_ground_min_value_ ||
            hsv_ground_max_value_ > 255 || ground_mask_morph_kernel_ < 1 ||
            hsv_clahe_clip_limit_ <= 0.0 || hsv_clahe_tile_grid_size_ < 1 ||
            ground_mask_bottom_band_ratio_ <= 0.0 || ground_mask_bottom_band_ratio_ > 1.0 ||
            ground_mask_min_bottom_pixels_ < 1 || ground_contact_band_ratio_ <= 0.0 ||
            ground_contact_band_ratio_ > 1.0 || ground_contact_min_ratio_ < 0.0 ||
            ground_contact_min_ratio_ > 1.0 || ground_below_band_ratio_ <= 0.0 ||
            ground_below_band_ratio_ > 1.0 || ground_below_width_ratio_ <= 0.0 ||
            ground_below_width_ratio_ > 1.0 || ground_below_min_ratio_ < 0.0 ||
            ground_below_min_ratio_ > 1.0)
        {
            throw std::invalid_argument("invalid paper_ball RGB-D vote parameters");
        }
    }

    cv::Mat buildGroundMask(const cv::Mat& bgr_image) const
    {
        if (bgr_image.empty())
        {
            return cv::Mat();
        }

        cv::Mat hsv;
        cv::cvtColor(bgr_image, hsv, cv::COLOR_BGR2HSV);
        if (enable_hsv_clahe_)
        {
            std::vector<cv::Mat> channels;
            cv::split(hsv, channels);
            cv::Ptr<cv::CLAHE> clahe = cv::createCLAHE(
                hsv_clahe_clip_limit_,
                cv::Size(hsv_clahe_tile_grid_size_, hsv_clahe_tile_grid_size_));
            clahe->apply(channels[2], channels[2]);
            cv::merge(channels, hsv);
        }

        cv::Mat low_saturation_mask;
        cv::inRange(hsv,
                    cv::Scalar(0, 0, hsv_ground_min_value_),
                    cv::Scalar(179, hsv_ground_max_saturation_, hsv_ground_max_value_),
                    low_saturation_mask);

        int kernel_size = ground_mask_morph_kernel_;
        if (kernel_size % 2 == 0)
        {
            ++kernel_size;
        }
        const cv::Mat kernel = cv::getStructuringElement(
            cv::MORPH_ELLIPSE, cv::Size(kernel_size, kernel_size));
        cv::morphologyEx(low_saturation_mask, low_saturation_mask,
                         cv::MORPH_OPEN, kernel);
        cv::morphologyEx(low_saturation_mask, low_saturation_mask,
                         cv::MORPH_CLOSE, kernel);
        return bottomConnectedGroundMask(low_saturation_mask);
    }

    cv::Mat bottomConnectedGroundMask(const cv::Mat& mask) const
    {
        if (mask.empty())
        {
            return cv::Mat();
        }

        cv::Mat labels;
        cv::Mat stats;
        cv::Mat centroids;
        const int label_count = cv::connectedComponentsWithStats(
            mask, labels, stats, centroids, 8, CV_32S);
        cv::Mat ground = cv::Mat::zeros(mask.size(), CV_8UC1);
        if (label_count <= 1)
        {
            return ground;
        }

        const int bottom_rows = std::max(
            1, std::min(mask.rows, static_cast<int>(std::round(
                                   mask.rows * ground_mask_bottom_band_ratio_))));
        const int bottom_start = std::max(0, mask.rows - bottom_rows);
        std::vector<int> bottom_pixels(static_cast<std::size_t>(label_count), 0);
        for (int y = bottom_start; y < mask.rows; ++y)
        {
            const auto* mask_row = mask.ptr<std::uint8_t>(y);
            const auto* label_row = labels.ptr<int>(y);
            for (int x = 0; x < mask.cols; ++x)
            {
                const int label = label_row[x];
                if (mask_row[x] > 0U && label > 0 && label < label_count)
                {
                    ++bottom_pixels[static_cast<std::size_t>(label)];
                }
            }
        }

        std::vector<bool> keep(static_cast<std::size_t>(label_count), false);
        for (int label = 1; label < label_count; ++label)
        {
            keep[static_cast<std::size_t>(label)] =
                bottom_pixels[static_cast<std::size_t>(label)] >= ground_mask_min_bottom_pixels_;
        }

        for (int y = 0; y < labels.rows; ++y)
        {
            const auto* label_row = labels.ptr<int>(y);
            auto* ground_row = ground.ptr<std::uint8_t>(y);
            for (int x = 0; x < labels.cols; ++x)
            {
                const int label = label_row[x];
                if (label > 0 && label < label_count && keep[static_cast<std::size_t>(label)])
                {
                    ground_row[x] = 255U;
                }
            }
        }
        return ground;
    }

    void imageCallback(const sensor_msgs::ImageConstPtr& message)
    {
        if (!message)
        {
            return;
        }
        try
        {
            const auto cv_image = cv_bridge::toCvCopy(
                message, sensor_msgs::image_encodings::BGR8);
            latest_ground_mask_ = buildGroundMask(cv_image->image);
            latest_ground_mask_stamp_ = message->header.stamp;
            if (ground_mask_publisher_ && !latest_ground_mask_.empty())
            {
                ground_mask_publisher_.publish(
                    cv_bridge::CvImage(message->header, sensor_msgs::image_encodings::MONO8,
                                       latest_ground_mask_).toImageMsg());
            }
        }
        catch (const cv_bridge::Exception& exception)
        {
            latest_ground_mask_.release();
            ROS_WARN_THROTTLE(2.0, "paper_ball RGB-D vote image cv_bridge error: %s",
                              exception.what());
        }
    }

    void depthCallback(const sensor_msgs::ImageConstPtr& message)
    {
        if (!message)
        {
            return;
        }
        try
        {
            latest_depth_ = cv_bridge::toCvShare(message);
        }
        catch (const cv_bridge::Exception& exception)
        {
            latest_depth_.reset();
            ROS_WARN_THROTTLE(2.0, "paper_ball RGB-D vote depth cv_bridge error: %s",
                              exception.what());
        }
    }

    void pointCloudCallback(const sensor_msgs::PointCloud2ConstPtr& message)
    {
        if (message && message->width > 0U && message->height > 0U &&
            !message->data.empty())
        {
            latest_cloud_ = message;
        }
    }

    void cameraInfoCallback(const sensor_msgs::CameraInfoConstPtr& message)
    {
        if (message && message->K[0] > 0.0 && message->K[4] > 0.0 &&
            message->width > 0U && message->height > 0U)
        {
            latest_camera_info_ = message;
        }
    }

    void detectionsCallback(const vision_msgs::DetectionArrayConstPtr& message)
    {
        if (!message)
        {
            return;
        }

        const ros::Time stamp = effectiveStamp(message->header);
        expireTracks(stamp);

        vision_msgs::DetectionArray output = *message;
        output.backend = appendBackend(message->backend);
        output.objects.clear();

        std::vector<std::uint32_t> published_tracks;
        for (const auto& detection : message->objects)
        {
            if (detection.class_name != "paper_ball" || !validBox(detection) ||
                detection.confidence < low_confidence_threshold_)
            {
                continue;
            }

            Candidate candidate = buildCandidate(detection, *message, stamp);
            const bool high_confidence =
                candidate.detection.confidence >= immediate_confidence_threshold_;
            if (enable_hsv_ground_filter_ && !candidate.ground_mask_ok)
            {
                ROS_DEBUG_THROTTLE(1.0,
                                   "paper_ball RGB-D vote rejected non-ground mask: contact=%.3f below=%.3f has_mask=%s",
                                   candidate.ground_contact_ratio,
                                   candidate.ground_below_ratio,
                                   candidate.has_ground_mask ? "true" : "false");
                continue;
            }
            const bool depth_rejects_ground_z =
                candidate.has_output_point && enable_ground_z_filter_ &&
                (candidate.output_z_m < min_ground_z_ ||
                 candidate.output_z_m > max_ground_z_);
            const bool hsv_grounded_high_confidence_fallback =
                high_confidence && allow_high_confidence_without_depth_ &&
                candidate.ground_mask_ok && !depth_rejects_ground_z;
            if (!candidate.depth_ok && require_depth_for_vote_ &&
                !hsv_grounded_high_confidence_fallback)
            {
                continue;
            }
            if (high_confidence && filter_high_confidence_with_depth_ &&
                candidate.has_depth && !candidate.depth_ok &&
                !hsv_grounded_high_confidence_fallback)
            {
                continue;
            }
            if (high_confidence && !candidate.has_depth &&
                !allow_high_confidence_without_depth_)
            {
                continue;
            }

            VoteTrack& track = updateTrack(candidate, stamp);
            const bool confirmed = high_confidence || track.hits >= minimum_votes_;
            if (!confirmed || containsTrack(published_tracks, track.id))
            {
                continue;
            }
            output.objects.push_back(publishedDetection(track, stamp));
            published_tracks.push_back(track.id);
        }

        appendHeldTracks(stamp, &output.objects, &published_tracks);
        std::sort(output.objects.begin(), output.objects.end(),
                  [](const vision_msgs::DetectionResult& left,
                     const vision_msgs::DetectionResult& right) {
                      return left.confidence > right.confidence;
                  });
        publisher_.publish(output);
    }

    Candidate buildCandidate(const vision_msgs::DetectionResult& detection,
                             const vision_msgs::DetectionArray& detections,
                             const ros::Time& stamp)
    {
        Candidate candidate;
        candidate.detection = detection;
        const double image_width =
            detections.image_width > 0U ? static_cast<double>(detections.image_width) : 0.0;
        const double image_height =
            detections.image_height > 0U ? static_cast<double>(detections.image_height) : 0.0;
        candidate.center_x_px = detection.x + 0.5 * detection.width;
        candidate.center_y_px = detection.y + 0.5 * detection.height;

        if (image_height > 0.0 && candidate.center_y_px / image_height < min_center_y_ratio_)
        {
            candidate.ground_mask_ok = false;
            return candidate;
        }
        const double aspect_ratio = std::max(
            static_cast<double>(detection.width) / static_cast<double>(detection.height),
            static_cast<double>(detection.height) / static_cast<double>(detection.width));
        if (aspect_ratio > max_aspect_ratio_)
        {
            candidate.ground_mask_ok = false;
            return candidate;
        }

        applyGroundMaskEvidence(detection, detections, stamp, &candidate);
        if (enable_hsv_ground_filter_ && !candidate.ground_mask_ok)
        {
            return candidate;
        }

        if (use_point_cloud_ && prefer_point_cloud_ &&
            applyPointCloudEvidence(detection, detections, stamp, &candidate))
        {
            return candidate;
        }

        if (!depthAvailable(stamp))
        {
            return candidate;
        }

        const cv::Mat& depth = latest_depth_->image;
        const double sx = image_width > 0.0 ? depth.cols / image_width : 1.0;
        const double sy = image_height > 0.0 ? depth.rows / image_height : 1.0;
        const int cx = static_cast<int>(std::round(candidate.center_x_px * sx));
        const int cy = static_cast<int>(std::round(candidate.center_y_px * sy));
        const int scaled_width = std::max(1, static_cast<int>(std::round(detection.width * sx)));
        const int scaled_height = std::max(1, static_cast<int>(std::round(detection.height * sy)));
        const int window_width = std::max(
            depth_window_min_pixels_,
            static_cast<int>(std::round(scaled_width * depth_window_scale_)));
        const int window_height = std::max(
            depth_window_min_pixels_,
            static_cast<int>(std::round(scaled_height * depth_window_scale_)));
        const int x1 = static_cast<int>(clampDouble(cx - window_width / 2.0, 0.0,
                                                    std::max(0, depth.cols - 1)));
        const int y1 = static_cast<int>(clampDouble(cy - window_height / 2.0, 0.0,
                                                    std::max(0, depth.rows - 1)));
        const int x2 = static_cast<int>(clampDouble(cx + window_width / 2.0, 0.0,
                                                    static_cast<double>(depth.cols - 1)));
        const int y2 = static_cast<int>(clampDouble(cy + window_height / 2.0, 0.0,
                                                    static_cast<double>(depth.rows - 1)));

        std::vector<double> depths;
        depths.reserve(static_cast<std::size_t>((x2 - x1 + 1) * (y2 - y1 + 1)));
        for (int y = y1; y <= y2; ++y)
        {
            for (int x = x1; x <= x2; ++x)
            {
                const double depth_m = depthMetersAt(depth, x, y, depth_unit_);
                if (std::isfinite(depth_m) && depth_m >= min_depth_ && depth_m <= max_depth_)
                {
                    depths.push_back(depth_m);
                }
            }
        }

        candidate.depth_samples = static_cast<int>(depths.size());
        if (candidate.depth_samples < min_depth_samples_)
        {
            return candidate;
        }
        std::vector<double> depths_for_median = depths;
        const double median_depth = median(&depths_for_median);
        if (!std::isfinite(median_depth))
        {
            return candidate;
        }
        std::vector<double> deviations;
        deviations.reserve(depths.size());
        for (const double depth_m : depths)
        {
            deviations.push_back(std::abs(depth_m - median_depth));
        }
        candidate.depth_mad_m = median(&deviations);
        candidate.has_depth = true;
        candidate.z_m = median_depth;

        const double fx = scaledFx(depth.cols);
        const double fy = scaledFy(depth.rows);
        const double px = scaledCx(depth.cols);
        const double py = scaledCy(depth.rows);
        if (fx <= 0.0 || fy <= 0.0)
        {
            return candidate;
        }
        candidate.x_m = (cx - px) * median_depth / fx;
        candidate.y_m = (cy - py) * median_depth / fy;
        candidate.physical_width_m = scaled_width * median_depth / fx;
        candidate.physical_height_m = scaled_height * median_depth / fy;

        bool ground_z_ok = true;
        Point3 output_point;
        if (transformPointToOutput(latest_depth_->header, candidate, &output_point))
        {
            candidate.has_output_point = true;
            candidate.output_x_m = output_point.x;
            candidate.output_y_m = output_point.y;
            candidate.output_z_m = output_point.z;
            ground_z_ok = !enable_ground_z_filter_ ||
                          (candidate.output_z_m >= min_ground_z_ &&
                           candidate.output_z_m <= max_ground_z_);
        }
        else if (enable_ground_z_filter_ && require_output_frame_for_vote_)
        {
            ground_z_ok = false;
        }

        candidate.depth_ok = candidate.depth_mad_m <= max_depth_mad_ &&
                             candidate.physical_width_m >= min_physical_width_ &&
                             candidate.physical_width_m <= max_physical_width_ &&
                             candidate.physical_height_m >= min_physical_height_ &&
                             candidate.physical_height_m <= max_physical_height_ &&
                             ground_z_ok && candidate.ground_mask_ok;
        if (use_point_cloud_ && !prefer_point_cloud_)
        {
            applyPointCloudEvidence(detection, detections, stamp, &candidate);
        }
        return candidate;
    }

    bool applyPointCloudEvidence(const vision_msgs::DetectionResult& detection,
                                 const vision_msgs::DetectionArray& detections,
                                 const ros::Time& stamp,
                                 Candidate* candidate)
    {
        if (candidate == nullptr || !pointCloudAvailable(stamp))
        {
            return false;
        }

        const sensor_msgs::PointCloud2& cloud = *latest_cloud_;
        std::uint32_t x_offset = 0U;
        std::uint32_t y_offset = 0U;
        std::uint32_t z_offset = 0U;
        if (!floatFieldOffset(cloud, "x", &x_offset) ||
            !floatFieldOffset(cloud, "y", &y_offset) ||
            !floatFieldOffset(cloud, "z", &z_offset))
        {
            ROS_WARN_THROTTLE(2.0, "paper_ball RGB-D vote point cloud has no float x/y/z fields");
            return false;
        }

        const double image_width = detections.image_width > 0U ?
            static_cast<double>(detections.image_width) : static_cast<double>(cloud.width);
        const double image_height = detections.image_height > 0U ?
            static_cast<double>(detections.image_height) : static_cast<double>(cloud.height);
        const double sx = image_width > 0.0 ? cloud.width / image_width : 1.0;
        const double sy = image_height > 0.0 ? cloud.height / image_height : 1.0;
        const int cx = static_cast<int>(std::round(candidate->center_x_px * sx));
        const int cy = static_cast<int>(std::round(candidate->center_y_px * sy));
        const int scaled_width = std::max(1, static_cast<int>(std::round(detection.width * sx)));
        const int scaled_height = std::max(1, static_cast<int>(std::round(detection.height * sy)));
        const int window_width = std::max(
            depth_window_min_pixels_,
            static_cast<int>(std::round(scaled_width * depth_window_scale_)));
        const int window_height = std::max(
            depth_window_min_pixels_,
            static_cast<int>(std::round(scaled_height * depth_window_scale_)));
        const int x1 = static_cast<int>(clampDouble(cx - window_width / 2.0, 0.0,
                                                    std::max(0, static_cast<int>(cloud.width) - 1)));
        const int y1 = static_cast<int>(clampDouble(cy - window_height / 2.0, 0.0,
                                                    std::max(0, static_cast<int>(cloud.height) - 1)));
        const int x2 = static_cast<int>(clampDouble(cx + window_width / 2.0, 0.0,
                                                    static_cast<double>(cloud.width - 1U)));
        const int y2 = static_cast<int>(clampDouble(cy + window_height / 2.0, 0.0,
                                                    static_cast<double>(cloud.height - 1U)));

        std::vector<double> xs;
        std::vector<double> ys;
        std::vector<double> zs;
        const std::size_t reserve_count = static_cast<std::size_t>(
            std::max(0, x2 - x1 + 1) * std::max(0, y2 - y1 + 1));
        xs.reserve(reserve_count);
        ys.reserve(reserve_count);
        zs.reserve(reserve_count);
        for (int y = y1; y <= y2; ++y)
        {
            for (int x = x1; x <= x2; ++x)
            {
                Point3 point;
                if (cloudPointAt(cloud, x, y, x_offset, y_offset, z_offset, &point) &&
                    point.z >= min_depth_ && point.z <= max_depth_)
                {
                    xs.push_back(point.x);
                    ys.push_back(point.y);
                    zs.push_back(point.z);
                }
            }
        }

        candidate->depth_samples = static_cast<int>(zs.size());
        if (candidate->depth_samples < min_depth_samples_)
        {
            return false;
        }

        std::vector<double> xs_for_median = xs;
        std::vector<double> ys_for_median = ys;
        std::vector<double> zs_for_median = zs;
        candidate->x_m = median(&xs_for_median);
        candidate->y_m = median(&ys_for_median);
        candidate->z_m = median(&zs_for_median);
        if (!std::isfinite(candidate->x_m) || !std::isfinite(candidate->y_m) ||
            !std::isfinite(candidate->z_m))
        {
            return false;
        }

        std::vector<double> deviations;
        deviations.reserve(zs.size());
        for (const double z : zs)
        {
            deviations.push_back(std::abs(z - candidate->z_m));
        }
        candidate->depth_mad_m = median(&deviations);

        const double fx = scaledFx(static_cast<int>(cloud.width));
        const double fy = scaledFy(static_cast<int>(cloud.height));
        if (fx <= 0.0 || fy <= 0.0)
        {
            return false;
        }
        candidate->physical_width_m = scaled_width * candidate->z_m / fx;
        candidate->physical_height_m = scaled_height * candidate->z_m / fy;
        candidate->has_depth = true;

        bool ground_ok = true;
        Point3 output_point;
        if (transformPointToOutput(cloud.header, *candidate, &output_point))
        {
            candidate->has_output_point = true;
            candidate->output_x_m = output_point.x;
            candidate->output_y_m = output_point.y;
            candidate->output_z_m = output_point.z;
            ground_ok = !enable_ground_z_filter_ ||
                        (candidate->output_z_m >= min_ground_z_ &&
                         candidate->output_z_m <= max_ground_z_);
        }
        else if (enable_ground_z_filter_ && require_output_frame_for_vote_)
        {
            ground_ok = false;
        }

        candidate->depth_ok = candidate->depth_mad_m <= max_depth_mad_ &&
                              candidate->physical_width_m >= min_physical_width_ &&
                              candidate->physical_width_m <= max_physical_width_ &&
                              candidate->physical_height_m >= min_physical_height_ &&
                              candidate->physical_height_m <= max_physical_height_ &&
                              ground_ok && candidate->ground_mask_ok;
        return true;
    }

    bool applyGroundMaskEvidence(const vision_msgs::DetectionResult& detection,
                                 const vision_msgs::DetectionArray& detections,
                                 const ros::Time& stamp,
                                 Candidate* candidate) const
    {
        if (candidate == nullptr || !enable_hsv_ground_filter_)
        {
            return true;
        }

        candidate->has_ground_mask = false;
        candidate->ground_mask_ok = !require_hsv_ground_filter_;
        candidate->ground_contact_ratio = 0.0;
        candidate->ground_below_ratio = 0.0;
        if (!groundMaskAvailable(stamp))
        {
            return candidate->ground_mask_ok;
        }

        const cv::Mat& mask = latest_ground_mask_;
        const double image_width = detections.image_width > 0U ?
            static_cast<double>(detections.image_width) : static_cast<double>(mask.cols);
        const double image_height = detections.image_height > 0U ?
            static_cast<double>(detections.image_height) : static_cast<double>(mask.rows);
        if (image_width <= 0.0 || image_height <= 0.0 || mask.empty())
        {
            return candidate->ground_mask_ok;
        }

        const double sx = mask.cols / image_width;
        const double sy = mask.rows / image_height;
        int x1 = static_cast<int>(std::floor(detection.x * sx));
        int y1 = static_cast<int>(std::floor(detection.y * sy));
        int x2 = static_cast<int>(std::ceil((detection.x + detection.width) * sx)) - 1;
        int y2 = static_cast<int>(std::ceil((detection.y + detection.height) * sy)) - 1;
        x1 = static_cast<int>(clampDouble(x1, 0.0, std::max(0, mask.cols - 1)));
        y1 = static_cast<int>(clampDouble(y1, 0.0, std::max(0, mask.rows - 1)));
        x2 = static_cast<int>(clampDouble(x2, 0.0, std::max(0, mask.cols - 1)));
        y2 = static_cast<int>(clampDouble(y2, 0.0, std::max(0, mask.rows - 1)));
        if (x2 < x1 || y2 < y1)
        {
            return candidate->ground_mask_ok;
        }

        candidate->has_ground_mask = true;
        const int box_width = x2 - x1 + 1;
        const int box_height = y2 - y1 + 1;
        const int contact_height = std::max(
            1, static_cast<int>(std::round(box_height * ground_contact_band_ratio_)));
        const int contact_y1 = std::max(y1, y2 - contact_height + 1);
        candidate->ground_contact_ratio = maskRatio(mask, x1, contact_y1, x2, y2);

        const int below_height = std::max(
            1, static_cast<int>(std::round(box_height * ground_below_band_ratio_)));
        const int below_width = std::max(
            1, static_cast<int>(std::round(box_width * ground_below_width_ratio_)));
        const int center_x = (x1 + x2) / 2;
        const int below_x1 = static_cast<int>(clampDouble(
            center_x - below_width / 2.0, 0.0, std::max(0, mask.cols - 1)));
        const int below_x2 = static_cast<int>(clampDouble(
            center_x + below_width / 2.0, 0.0, std::max(0, mask.cols - 1)));
        const int below_y1 = y2 + 1;
        if (below_y1 < mask.rows)
        {
            const int below_y2 = std::min(mask.rows - 1, y2 + below_height);
            candidate->ground_below_ratio =
                maskRatio(mask, below_x1, below_y1, below_x2, below_y2);
        }

        candidate->ground_mask_ok =
            candidate->ground_contact_ratio >= ground_contact_min_ratio_ ||
            candidate->ground_below_ratio >= ground_below_min_ratio_;
        return candidate->ground_mask_ok;
    }

    double maskRatio(const cv::Mat& mask, int x1, int y1, int x2, int y2) const
    {
        if (mask.empty())
        {
            return 0.0;
        }
        x1 = static_cast<int>(clampDouble(x1, 0.0, std::max(0, mask.cols - 1)));
        y1 = static_cast<int>(clampDouble(y1, 0.0, std::max(0, mask.rows - 1)));
        x2 = static_cast<int>(clampDouble(x2, 0.0, std::max(0, mask.cols - 1)));
        y2 = static_cast<int>(clampDouble(y2, 0.0, std::max(0, mask.rows - 1)));
        if (x2 < x1 || y2 < y1)
        {
            return 0.0;
        }
        const cv::Rect roi(x1, y1, x2 - x1 + 1, y2 - y1 + 1);
        const double total = static_cast<double>(roi.width * roi.height);
        return total > 0.0 ? cv::countNonZero(mask(roi)) / total : 0.0;
    }

    bool depthAvailable(const ros::Time& stamp) const
    {
        return latest_depth_ && latest_camera_info_ &&
               !latest_depth_->image.empty() &&
               closeStamp(stamp, latest_depth_->header.stamp, max_depth_time_delta_);
    }

    bool groundMaskAvailable(const ros::Time& stamp) const
    {
        return !latest_ground_mask_.empty() &&
               closeStamp(stamp, latest_ground_mask_stamp_, max_hsv_image_time_delta_);
    }

    bool pointCloudAvailable(const ros::Time& stamp) const
    {
        return latest_cloud_ && latest_camera_info_ && latest_cloud_->width > 0U &&
               latest_cloud_->height > 0U && !latest_cloud_->data.empty() &&
               closeStamp(stamp, latest_cloud_->header.stamp, max_cloud_time_delta_);
    }

    bool transformPointToOutput(const std_msgs::Header& header,
                                const Candidate& candidate,
                                Point3* output_point)
    {
        if (output_point == nullptr || header.frame_id.empty() || !candidate.has_depth)
        {
            return false;
        }
        const Point3 camera_point{candidate.x_m, candidate.y_m, candidate.z_m};
        if (!finitePoint(camera_point))
        {
            return false;
        }
        if (header.frame_id == output_frame_)
        {
            *output_point = camera_point;
            return true;
        }

        try
        {
            const geometry_msgs::TransformStamped transform_message =
                tf_buffer_.lookupTransform(output_frame_, header.frame_id, header.stamp,
                                           ros::Duration(tf_timeout_));
            tf2::Transform transform;
            tf2::fromMsg(transform_message.transform, transform);
            const tf2::Vector3 transformed =
                transform * tf2::Vector3(camera_point.x, camera_point.y, camera_point.z);
            output_point->x = transformed.x();
            output_point->y = transformed.y();
            output_point->z = transformed.z();
            return finitePoint(*output_point);
        }
        catch (const tf2::TransformException& exception)
        {
            ROS_WARN_THROTTLE(2.0, "paper_ball RGB-D vote TF unavailable: %s",
                              exception.what());
            return false;
        }
    }

    double scaledFx(int width) const
    {
        return latest_camera_info_ ?
            latest_camera_info_->K[0] * width / latest_camera_info_->width : 0.0;
    }

    double scaledFy(int height) const
    {
        return latest_camera_info_ ?
            latest_camera_info_->K[4] * height / latest_camera_info_->height : 0.0;
    }

    double scaledCx(int width) const
    {
        return latest_camera_info_ ?
            latest_camera_info_->K[2] * width / latest_camera_info_->width : 0.0;
    }

    double scaledCy(int height) const
    {
        return latest_camera_info_ ?
            latest_camera_info_->K[5] * height / latest_camera_info_->height : 0.0;
    }

    VoteTrack& updateTrack(const Candidate& candidate, const ros::Time& stamp)
    {
        const int index = matchingTrackIndex(candidate);
        if (index >= 0)
        {
            VoteTrack& track = tracks_.at(static_cast<std::size_t>(index));
            track.last_seen = stamp;
            ++track.hits;
            track.has_depth = candidate.has_depth;
            track.x_m = candidate.x_m;
            track.y_m = candidate.y_m;
            track.z_m = candidate.z_m;
            track.has_output_point = candidate.has_output_point;
            track.output_x_m = candidate.output_x_m;
            track.output_y_m = candidate.output_y_m;
            track.output_z_m = candidate.output_z_m;
            track.center_x_px = candidate.center_x_px;
            track.center_y_px = candidate.center_y_px;
            if (candidate.detection.confidence >= track.best_detection.confidence)
            {
                track.best_detection = candidate.detection;
            }
            else
            {
                track.best_detection.x = candidate.detection.x;
                track.best_detection.y = candidate.detection.y;
                track.best_detection.width = candidate.detection.width;
                track.best_detection.height = candidate.detection.height;
            }
            return track;
        }

        VoteTrack track;
        track.id = next_track_id_++;
        if (next_track_id_ == 0U)
        {
            next_track_id_ = 1U;
        }
        track.first_seen = stamp;
        track.last_seen = stamp;
        track.hits = 1;
        track.has_depth = candidate.has_depth;
        track.x_m = candidate.x_m;
        track.y_m = candidate.y_m;
        track.z_m = candidate.z_m;
        track.has_output_point = candidate.has_output_point;
        track.output_x_m = candidate.output_x_m;
        track.output_y_m = candidate.output_y_m;
        track.output_z_m = candidate.output_z_m;
        track.center_x_px = candidate.center_x_px;
        track.center_y_px = candidate.center_y_px;
        track.best_detection = candidate.detection;
        tracks_.push_back(track);
        return tracks_.back();
    }

    int matchingTrackIndex(const Candidate& candidate) const
    {
        int best_index = -1;
        double best_score = std::numeric_limits<double>::infinity();
        for (std::size_t index = 0; index < tracks_.size(); ++index)
        {
            const VoteTrack& track = tracks_[index];
            double score = std::numeric_limits<double>::infinity();
            if (candidate.has_output_point && track.has_output_point)
            {
                const double dx = candidate.output_x_m - track.output_x_m;
                const double dy = candidate.output_y_m - track.output_y_m;
                const double dz = candidate.output_z_m - track.output_z_m;
                const double distance = std::sqrt(dx * dx + dy * dy + dz * dz);
                if (distance <= max_3d_match_distance_)
                {
                    score = distance;
                }
            }
            else if (candidate.has_depth && track.has_depth)
            {
                const double dx = candidate.x_m - track.x_m;
                const double dy = candidate.y_m - track.y_m;
                const double dz = candidate.z_m - track.z_m;
                const double distance = std::sqrt(dx * dx + dy * dy + dz * dz);
                if (distance <= max_3d_match_distance_)
                {
                    score = distance;
                }
            }

            const double image_dx = candidate.center_x_px - track.center_x_px;
            const double image_dy = candidate.center_y_px - track.center_y_px;
            const double image_distance = std::sqrt(image_dx * image_dx + image_dy * image_dy);
            if (image_distance <= max_image_match_distance_ ||
                iou(candidate.detection, track.best_detection) >= match_iou_threshold_)
            {
                score = std::min(score, image_distance / std::max(1.0, max_image_match_distance_));
            }

            if (score < best_score)
            {
                best_score = score;
                best_index = static_cast<int>(index);
            }
        }
        return best_index;
    }

    vision_msgs::DetectionResult publishedDetection(VoteTrack& track,
                                                     const ros::Time& stamp) const
    {
        vision_msgs::DetectionResult detection = track.best_detection;
        detection.track_id = track.id;
        detection.confidence = std::max(detection.confidence, publish_confidence_floor_);
        track.last_published = stamp;
        return detection;
    }

    void appendHeldTracks(const ros::Time& stamp,
                          std::vector<vision_msgs::DetectionResult>* output,
                          std::vector<std::uint32_t>* published_tracks)
    {
        if (hold_seconds_ <= 0.0)
        {
            return;
        }
        for (auto& track : tracks_)
        {
            if (track.hits < minimum_votes_ || containsTrack(*published_tracks, track.id))
            {
                continue;
            }
            const double age = (stamp - track.last_seen).toSec();
            if (age < 0.0 || age > hold_seconds_)
            {
                continue;
            }
            const bool overlaps = std::any_of(
                output->begin(), output->end(),
                [&track, this](const vision_msgs::DetectionResult& detection) {
                    return detection.class_name == track.best_detection.class_name &&
                           iou(detection, track.best_detection) >= match_iou_threshold_;
                });
            if (overlaps)
            {
                continue;
            }
            vision_msgs::DetectionResult held = track.best_detection;
            held.track_id = track.id;
            held.confidence = std::max(publish_confidence_floor_, held.confidence * 0.90F);
            output->push_back(held);
            published_tracks->push_back(track.id);
        }
    }

    bool containsTrack(const std::vector<std::uint32_t>& tracks,
                       std::uint32_t track_id) const
    {
        return std::find(tracks.begin(), tracks.end(), track_id) != tracks.end();
    }

    void expireTracks(const ros::Time& stamp)
    {
        const double expire_seconds = vote_window_seconds_ + hold_seconds_;
        tracks_.erase(
            std::remove_if(tracks_.begin(), tracks_.end(),
                           [&stamp, expire_seconds](const VoteTrack& track) {
                               if (track.last_seen.isZero())
                               {
                                   return false;
                               }
                               const double age = (stamp - track.last_seen).toSec();
                               return age > expire_seconds;
                           }),
            tracks_.end());
    }

    ros::NodeHandle nh_;
    ros::NodeHandle private_nh_;
    ros::Subscriber detection_subscriber_;
    ros::Subscriber depth_subscriber_;
    ros::Subscriber point_cloud_subscriber_;
    ros::Subscriber camera_info_subscriber_;
    ros::Subscriber image_subscriber_;
    ros::Publisher publisher_;
    ros::Publisher ground_mask_publisher_;
    tf2_ros::Buffer tf_buffer_;
    tf2_ros::TransformListener tf_listener_;

    std::string detections_topic_;
    std::string image_topic_;
    std::string depth_topic_;
    std::string point_cloud_topic_;
    std::string camera_info_topic_;
    std::string output_topic_;
    std::string output_frame_;
    std::string ground_mask_debug_topic_;

    float low_confidence_threshold_{0.06F};
    float immediate_confidence_threshold_{0.10F};
    float publish_confidence_floor_{0.10F};
    int minimum_votes_{2};
    double vote_window_seconds_{0.90};
    double hold_seconds_{0.35};
    double max_depth_time_delta_{0.20};
    double max_cloud_time_delta_{0.15};
    double tf_timeout_{0.05};
    double max_3d_match_distance_{0.18};
    double max_image_match_distance_{28.0};
    float match_iou_threshold_{0.20F};
    double depth_unit_{0.001};
    double min_depth_{0.35};
    double max_depth_{6.00};
    int min_depth_samples_{5};
    int depth_window_min_pixels_{7};
    double depth_window_scale_{1.8};
    double max_depth_mad_{0.25};
    double min_physical_width_{0.006};
    double max_physical_width_{0.22};
    double min_physical_height_{0.006};
    double max_physical_height_{0.22};
    double min_center_y_ratio_{0.20};
    double max_aspect_ratio_{2.60};
    double min_ground_z_{-0.60};
    double max_ground_z_{0.25};
    double max_hsv_image_time_delta_{0.50};
    int hsv_ground_max_saturation_{70};
    int hsv_ground_min_value_{20};
    int hsv_ground_max_value_{255};
    double hsv_clahe_clip_limit_{2.0};
    int hsv_clahe_tile_grid_size_{8};
    int ground_mask_morph_kernel_{5};
    double ground_mask_bottom_band_ratio_{0.10};
    int ground_mask_min_bottom_pixels_{10};
    double ground_contact_band_ratio_{0.30};
    double ground_contact_min_ratio_{0.03};
    double ground_below_band_ratio_{0.18};
    double ground_below_width_ratio_{0.70};
    double ground_below_min_ratio_{0.10};
    bool prefer_point_cloud_{true};
    bool use_point_cloud_{true};
    bool enable_ground_z_filter_{true};
    bool require_output_frame_for_vote_{false};
    bool require_depth_for_vote_{true};
    bool filter_high_confidence_with_depth_{true};
    bool allow_high_confidence_without_depth_{true};
    bool enable_hsv_ground_filter_{true};
    bool require_hsv_ground_filter_{true};
    bool enable_hsv_clahe_{true};

    cv_bridge::CvImageConstPtr latest_depth_;
    sensor_msgs::PointCloud2ConstPtr latest_cloud_;
    sensor_msgs::CameraInfoConstPtr latest_camera_info_;
    cv::Mat latest_ground_mask_;
    ros::Time latest_ground_mask_stamp_;
    std::vector<VoteTrack> tracks_;
    std::uint32_t next_track_id_{1U};
};

int main(int argc, char** argv)
{
    ros::init(argc, argv, "paper_ball_rgbd_vote_node");
    try
    {
        PaperBallRgbdVoteNode node;
        ros::spin();
    }
    catch (const std::exception& exception)
    {
        ROS_FATAL("paper_ball RGB-D vote node failed: %s", exception.what());
        return 1;
    }
    return 0;
}
