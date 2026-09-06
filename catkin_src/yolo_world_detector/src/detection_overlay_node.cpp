#include <algorithm>
#include <array>
#include <cmath>
#include <cstdint>
#include <iomanip>
#include <sstream>
#include <string>

#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.h>
#include <opencv2/imgproc.hpp>
#include <ros/ros.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/image_encodings.h>
#include <vision_msgs/DetectionArray.h>
#include <vision_msgs/DetectionResult.h>

namespace
{

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
    const int x2 = std::max(x1, std::min(width, static_cast<int>(std::round(detection.x + detection.width))));
    const int y2 = std::max(y1, std::min(height, static_cast<int>(std::round(detection.y + detection.height))));
    return cv::Rect(x1, y1, x2 - x1, y2 - y1);
}

bool closeStamp(const ros::Time& left, const ros::Time& right, double max_delta_seconds)
{
    if (left.isZero() || right.isZero())
    {
        return true;
    }
    return std::abs((left - right).toSec()) <= max_delta_seconds;
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

}  // namespace

class DetectionOverlayNode
{
public:
    DetectionOverlayNode()
        : nh_(), private_nh_("~"), image_transport_(nh_)
    {
        private_nh_.param<std::string>("image_topic", image_topic_,
                                       "/kinect2/hd/image_color");
        private_nh_.param<std::string>("detections_topic", detections_topic_,
                                       "/yolo_world/enhanced_detections");
        private_nh_.param<std::string>("annotated_topic", annotated_topic_,
                                       "/yolo_world/enhanced_annotated_image");
        private_nh_.param("max_stamp_delta_seconds", max_stamp_delta_seconds_, 2.50);
        private_nh_.param("minimum_confidence", minimum_confidence_, 0.0F);
        private_nh_.param("box_thickness", box_thickness_, 2);
        private_nh_.param("label_font_scale", label_font_scale_, 0.55);
        private_nh_.param("show_status_bar", show_status_bar_, true);

        image_subscriber_ = image_transport_.subscribe(
            image_topic_, 1, &DetectionOverlayNode::imageCallback, this);
        detection_subscriber_ = nh_.subscribe(
            detections_topic_, 2, &DetectionOverlayNode::detectionCallback, this);
        annotated_publisher_ = image_transport_.advertise(annotated_topic_, 1);

        ROS_INFO("Detection overlay ready: image=%s detections=%s annotated=%s",
                 image_topic_.c_str(), detections_topic_.c_str(), annotated_topic_.c_str());
    }

private:
    void detectionCallback(const vision_msgs::DetectionArrayConstPtr& message)
    {
        if (message)
        {
            latest_detections_ = *message;
            have_detections_ = true;
        }
    }

    void imageCallback(const sensor_msgs::ImageConstPtr& message)
    {
        if (!message)
        {
            return;
        }

        try
        {
            const auto cv_ptr = cv_bridge::toCvCopy(
                message, sensor_msgs::image_encodings::BGR8);
            cv::Mat annotated = cv_ptr->image;
            int drawn_count = 0;
            const bool current_detections = have_detections_ &&
                                            closeStamp(message->header.stamp,
                                                       latest_detections_.header.stamp,
                                                       max_stamp_delta_seconds_);
            if (current_detections)
            {
                drawn_count = drawDetections(&annotated);
            }
            if (show_status_bar_)
            {
                drawStatusBar(&annotated, drawn_count, have_detections_ && !current_detections);
            }
            annotated_publisher_.publish(
                cv_bridge::CvImage(message->header, sensor_msgs::image_encodings::BGR8,
                                   annotated)
                    .toImageMsg());
        }
        catch (const cv_bridge::Exception& exception)
        {
            ROS_ERROR_THROTTLE(2.0, "cv_bridge error in detection overlay: %s",
                               exception.what());
        }
    }

    int drawDetections(cv::Mat* image) const
    {
        if (image == nullptr || image->empty())
        {
            return 0;
        }

        int drawn_count = 0;
        const int thickness = std::max(1, box_thickness_);
        for (const auto& detection : latest_detections_.objects)
        {
            if (!detection.has_bbox || detection.confidence < minimum_confidence_)
            {
                continue;
            }
            const cv::Rect box = clampRect(detection, image->cols, image->rows);
            if (box.width <= 0 || box.height <= 0)
            {
                continue;
            }

            const cv::Scalar color = colorForClass(detection.class_name);
            drawCornerBox(image, box, color, thickness);

            std::ostringstream label_stream;
            label_stream << detection.class_name << ' ' << std::fixed
                         << std::setprecision(2) << detection.confidence;
            if (detection.track_id != 0U)
            {
                label_stream << " #" << detection.track_id;
            }
            const std::string label = label_stream.str();

            int baseline = 0;
            const cv::Size label_size = cv::getTextSize(
                label, cv::FONT_HERSHEY_SIMPLEX, label_font_scale_, 1, &baseline);
            const int label_x = std::max(0, box.x);
            const int label_y = std::max(label_size.height + baseline, box.y);
            const cv::Rect background(
                label_x, label_y - label_size.height - baseline,
                std::min(label_size.width + 6, image->cols - label_x),
                label_size.height + baseline);
            if (background.width > 0 && background.height > 0)
            {
                drawFilledBackground(image, background, color, 0.82);
            }
            cv::putText(*image, label, cv::Point(label_x + 3, label_y - baseline),
                        cv::FONT_HERSHEY_SIMPLEX, label_font_scale_,
                        textColorForBackground(color), 1, cv::LINE_AA);
            ++drawn_count;
        }
        return drawn_count;
    }

    void drawStatusBar(cv::Mat* image, int drawn_count, bool stale_detections) const
    {
        if (image == nullptr || image->empty())
        {
            return;
        }

        std::ostringstream status_stream;
        status_stream << "WPB vision | detections: " << drawn_count;
        if (stale_detections)
        {
            status_stream << " | waiting for current frame";
        }
        const std::string status = status_stream.str();
        int baseline = 0;
        const cv::Size text_size = cv::getTextSize(status, cv::FONT_HERSHEY_SIMPLEX, 0.50,
                                                   1, &baseline);
        const int height = text_size.height + baseline + 10;
        const int bar_height = std::min(height, image->rows);
        const cv::Rect background(0, image->rows - bar_height, image->cols, bar_height);
        drawFilledBackground(image, background, cv::Scalar(24, 24, 24), 0.62);
        cv::putText(*image, status, cv::Point(8, image->rows - baseline - 5),
                    cv::FONT_HERSHEY_SIMPLEX, 0.50, cv::Scalar(255, 255, 255), 1,
                    cv::LINE_AA);
    }

    ros::NodeHandle nh_;
    ros::NodeHandle private_nh_;
    image_transport::ImageTransport image_transport_;
    image_transport::Subscriber image_subscriber_;
    image_transport::Publisher annotated_publisher_;
    ros::Subscriber detection_subscriber_;

    std::string image_topic_;
    std::string detections_topic_;
    std::string annotated_topic_;
    double max_stamp_delta_seconds_ = 2.50;
    float minimum_confidence_ = 0.0F;
    int box_thickness_ = 2;
    double label_font_scale_ = 0.55;
    bool show_status_bar_ = true;

    bool have_detections_ = false;
    vision_msgs::DetectionArray latest_detections_;
};

int main(int argc, char** argv)
{
    ros::init(argc, argv, "detection_overlay");
    DetectionOverlayNode node;
    ros::spin();
    return 0;
}
