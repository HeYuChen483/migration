#include <algorithm>
#include <cmath>
#include <string>
#include <unordered_set>
#include <vector>

#include <ros/ros.h>
#include <vision_msgs/DetectionArray.h>
#include <vision_msgs/DetectionResult.h>

namespace
{

float area(const vision_msgs::DetectionResult& detection)
{
    if (!detection.has_bbox || detection.width <= 0.0F || detection.height <= 0.0F)
    {
        return 0.0F;
    }
    return detection.width * detection.height;
}

float iou(const vision_msgs::DetectionResult& left,
          const vision_msgs::DetectionResult& right)
{
    if (!left.has_bbox || !right.has_bbox)
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

bool isPaperBallClass(const std::string& class_name)
{
    return class_name == "paper_ball";
}

bool isBoxClass(const std::string& class_name)
{
    return class_name == "box";
}

bool isPersonClass(const std::string& class_name)
{
    return class_name == "person";
}

bool isTrashBinClass(const std::string& class_name)
{
    return class_name == "trash_bin";
}

}  // namespace

class EnhancedDetectionMerger
{
public:
    EnhancedDetectionMerger()
        : nh_(), private_nh_("~")
    {
        private_nh_.param<std::string>("primary_topic", primary_topic_,
                                       "/yolo_world/detections");
        private_nh_.param<std::string>("supplemental_topic", supplemental_topic_,
                                       "/yolo_world/paper_box_detections");
        private_nh_.param<std::string>("supplemental2_topic", supplemental2_topic_,
                                       std::string(""));
        private_nh_.param<std::string>("output_topic", output_topic_,
                                       "/yolo_world/enhanced_detections");
        private_nh_.param<std::string>("trash_bin_topic", trash_bin_topic_, std::string(""));
        private_nh_.param("merge_iou_threshold", merge_iou_threshold_, 0.45F);
        private_nh_.param("supplemental_min_confidence", supplemental_min_confidence_,
                          0.25F);
        private_nh_.param("supplemental2_min_confidence", supplemental2_min_confidence_,
                          supplemental_min_confidence_);
        private_nh_.param("primary_box_min_confidence", primary_box_min_confidence_, 0.0F);
        private_nh_.param("supplemental_box_min_confidence",
                          supplemental_box_min_confidence_, supplemental_min_confidence_);
        private_nh_.param("box_merge_iou_threshold", box_merge_iou_threshold_,
                          merge_iou_threshold_);
        private_nh_.param("box_person_overlap_iou_threshold",
                          box_person_overlap_iou_threshold_, 1.0F);
        private_nh_.param("box_max_aspect_ratio", box_max_aspect_ratio_, 100.0F);
        private_nh_.param("trash_bin_min_confidence", trash_bin_min_confidence_, 0.25F);
        private_nh_.param("max_stamp_delta_seconds", max_stamp_delta_seconds_, 0.30);

        if (merge_iou_threshold_ <= 0.0F || merge_iou_threshold_ > 1.0F ||
            supplemental_min_confidence_ < 0.0F || supplemental_min_confidence_ > 1.0F ||
            supplemental2_min_confidence_ < 0.0F || supplemental2_min_confidence_ > 1.0F ||
            primary_box_min_confidence_ < 0.0F || primary_box_min_confidence_ > 1.0F ||
            supplemental_box_min_confidence_ < 0.0F ||
            supplemental_box_min_confidence_ > 1.0F ||
            box_merge_iou_threshold_ <= 0.0F || box_merge_iou_threshold_ > 1.0F ||
            box_person_overlap_iou_threshold_ < 0.0F ||
            box_person_overlap_iou_threshold_ > 1.0F ||
            box_max_aspect_ratio_ <= 0.0F ||
            trash_bin_min_confidence_ < 0.0F || trash_bin_min_confidence_ > 1.0F ||
            max_stamp_delta_seconds_ < 0.0)
        {
            throw std::invalid_argument("invalid enhanced merger thresholds");
        }

        publisher_ = nh_.advertise<vision_msgs::DetectionArray>(output_topic_, 2);
        primary_subscriber_ = nh_.subscribe(primary_topic_, 2,
                                            &EnhancedDetectionMerger::primaryCallback, this);
        supplemental_subscriber_ = nh_.subscribe(
            supplemental_topic_, 2, &EnhancedDetectionMerger::supplementalCallback, this);
        if (!supplemental2_topic_.empty())
        {
            supplemental2_subscriber_ = nh_.subscribe(
                supplemental2_topic_, 2, &EnhancedDetectionMerger::supplemental2Callback, this);
        }
        if (!trash_bin_topic_.empty())
        {
            trash_bin_subscriber_ = nh_.subscribe(
                trash_bin_topic_, 2, &EnhancedDetectionMerger::trashBinCallback, this);
        }

        ROS_INFO("Enhanced detection merger ready: primary=%s supplemental=%s supplemental2=%s trash_bin=%s output=%s",
                 primary_topic_.c_str(), supplemental_topic_.c_str(),
                 supplemental2_topic_.empty() ? "<disabled>" : supplemental2_topic_.c_str(),
                 trash_bin_topic_.empty() ? "<disabled>" : trash_bin_topic_.c_str(),
                 output_topic_.c_str());
    }

private:
    void supplementalCallback(const vision_msgs::DetectionArrayConstPtr& message)
    {
        if (message)
        {
            latest_supplemental_ = *message;
            have_supplemental_ = true;
        }
    }

    void supplemental2Callback(const vision_msgs::DetectionArrayConstPtr& message)
    {
        if (message)
        {
            latest_supplemental2_ = *message;
            have_supplemental2_ = true;
        }
    }

    void trashBinCallback(const vision_msgs::DetectionArrayConstPtr& message)
    {
        if (message)
        {
            latest_trash_bin_ = *message;
            have_trash_bin_ = true;
        }
    }

    void primaryCallback(const vision_msgs::DetectionArrayConstPtr& message)
    {
        if (!message)
        {
            return;
        }

        vision_msgs::DetectionArray merged = *message;
        merged.backend = message->backend;
        filterPrimaryBoxes(&merged.objects);

        if (have_supplemental_ && isCloseStamp(message->header.stamp,
                                               latest_supplemental_.header.stamp))
        {
            merged.backend += supplementalBackendSuffix(latest_supplemental_.backend);
            mergeDetections(latest_supplemental_, supplemental_min_confidence_,
                            isPaperBallClass, &merged.objects);
            mergeDetections(latest_supplemental_, supplemental_box_min_confidence_,
                            isBoxClass, &merged.objects);
        }
        if (!supplemental2_topic_.empty() && have_supplemental2_ &&
            isCloseStamp(message->header.stamp, latest_supplemental2_.header.stamp))
        {
            merged.backend += "+paper_box_secondary_supplement";
            mergeDetections(latest_supplemental2_, supplemental2_min_confidence_,
                            isPaperBallClass, &merged.objects);
        }
        if (have_trash_bin_ && isCloseStamp(message->header.stamp, latest_trash_bin_.header.stamp))
        {
            merged.backend += "+trash_bin_supplement";
            mergeDetections(latest_trash_bin_, trash_bin_min_confidence_,
                            isTrashBinClass, &merged.objects);
        }
        suppressPersonLikeBoxes(&merged.objects);

        std::sort(merged.objects.begin(), merged.objects.end(),
                  [](const vision_msgs::DetectionResult& left,
                     const vision_msgs::DetectionResult& right) {
                      return left.confidence > right.confidence;
                  });
        publisher_.publish(merged);
    }

    void filterPrimaryBoxes(std::vector<vision_msgs::DetectionResult>* detections) const
    {
        if (primary_box_min_confidence_ <= 0.0F)
        {
            return;
        }
        detections->erase(
            std::remove_if(
                detections->begin(), detections->end(),
                [this](const vision_msgs::DetectionResult& detection) {
                    return isBoxClass(detection.class_name) &&
                           detection.confidence < primary_box_min_confidence_;
                }),
            detections->end());
    }

    void suppressPersonLikeBoxes(std::vector<vision_msgs::DetectionResult>* detections) const
    {
        std::vector<vision_msgs::DetectionResult> persons;
        persons.reserve(detections->size());
        for (const auto& detection : *detections)
        {
            if (isPersonClass(detection.class_name) && detection.has_bbox &&
                detection.width > 0.0F && detection.height > 0.0F)
            {
                persons.push_back(detection);
            }
        }

        detections->erase(
            std::remove_if(
                detections->begin(), detections->end(),
                [this, &persons](const vision_msgs::DetectionResult& detection) {
                    if (!isBoxClass(detection.class_name) || !detection.has_bbox ||
                        detection.width <= 0.0F || detection.height <= 0.0F)
                    {
                        return false;
                    }
                    const float aspect_ratio =
                        detection.height / std::max(detection.width, 1.0e-6F);
                    if (aspect_ratio > box_max_aspect_ratio_)
                    {
                        return true;
                    }
                    return std::any_of(
                        persons.begin(), persons.end(),
                        [this, &detection](const vision_msgs::DetectionResult& person) {
                            return iou(detection, person) >= box_person_overlap_iou_threshold_;
                        });
                }),
            detections->end());
    }

    std::string supplementalBackendSuffix(const std::string& backend) const
    {
        if (backend.find("bytetrack") != std::string::npos)
        {
            return "+paper_box_bytetrack_supplement";
        }
        return "+paper_box_supplement";
    }

    bool isCloseStamp(const ros::Time& primary_stamp, const ros::Time& supplemental_stamp) const
    {
        if (primary_stamp.isZero() || supplemental_stamp.isZero())
        {
            return true;
        }
        return std::abs((primary_stamp - supplemental_stamp).toSec()) <=
               max_stamp_delta_seconds_;
    }

    void mergeDetections(const vision_msgs::DetectionArray& source, float minimum_confidence,
                         bool (*class_filter)(const std::string&),
                         std::vector<vision_msgs::DetectionResult>* detections) const
    {
        for (const auto& candidate : source.objects)
        {
            if (!class_filter(candidate.class_name) ||
                candidate.confidence < minimum_confidence)
            {
                continue;
            }
            addOrReplaceSupplemental(candidate, detections);
        }
    }

    void addOrReplaceSupplemental(const vision_msgs::DetectionResult& candidate,
                                  std::vector<vision_msgs::DetectionResult>* detections) const
    {
        for (auto& kept : *detections)
        {
            if (kept.class_name != candidate.class_name)
            {
                continue;
            }
            const float required_iou =
                isBoxClass(candidate.class_name) ? box_merge_iou_threshold_
                                                 : merge_iou_threshold_;
            if (iou(kept, candidate) < required_iou)
            {
                continue;
            }
            if (candidate.confidence > kept.confidence)
            {
                kept = candidate;
            }
            return;
        }
        detections->push_back(candidate);
    }

    ros::NodeHandle nh_;
    ros::NodeHandle private_nh_;
    ros::Subscriber primary_subscriber_;
    ros::Subscriber supplemental_subscriber_;
    ros::Subscriber supplemental2_subscriber_;
    ros::Subscriber trash_bin_subscriber_;
    ros::Publisher publisher_;

    std::string primary_topic_;
    std::string supplemental_topic_;
    std::string supplemental2_topic_;
    std::string output_topic_;
    std::string trash_bin_topic_;
    float merge_iou_threshold_ = 0.45F;
    float supplemental_min_confidence_ = 0.25F;
    float supplemental2_min_confidence_ = 0.25F;
    float primary_box_min_confidence_ = 0.0F;
    float supplemental_box_min_confidence_ = 0.25F;
    float box_merge_iou_threshold_ = 0.45F;
    float box_person_overlap_iou_threshold_ = 0.15F;
    float box_max_aspect_ratio_ = 1.80F;
    float trash_bin_min_confidence_ = 0.25F;
    double max_stamp_delta_seconds_ = 0.30;

    bool have_supplemental_ = false;
    bool have_supplemental2_ = false;
    bool have_trash_bin_ = false;
    vision_msgs::DetectionArray latest_supplemental_;
    vision_msgs::DetectionArray latest_supplemental2_;
    vision_msgs::DetectionArray latest_trash_bin_;
};

int main(int argc, char** argv)
{
    ros::init(argc, argv, "enhanced_detection_merger");
    try
    {
        EnhancedDetectionMerger node;
        ros::spin();
    }
    catch (const std::exception& exception)
    {
        ROS_FATAL("Enhanced detection merger failed: %s", exception.what());
        return 1;
    }
    return 0;
}
