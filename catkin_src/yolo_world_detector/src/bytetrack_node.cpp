#include <algorithm>
#include <cmath>
#include <cstdint>
#include <limits>
#include <stdexcept>
#include <sstream>
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

bool validBox(const vision_msgs::DetectionResult& detection)
{
    return detection.has_bbox && detection.width > 0.0F && detection.height > 0.0F &&
           std::isfinite(detection.x) && std::isfinite(detection.y) &&
           std::isfinite(detection.width) && std::isfinite(detection.height);
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
    const float intersection_width = std::max(0.0F, ix2 - ix1);
    const float intersection_height = std::max(0.0F, iy2 - iy1);
    const float intersection = intersection_width * intersection_height;
    const float union_area = area(left) + area(right) - intersection;
    return union_area > 0.0F ? intersection / union_area : 0.0F;
}

ros::Time effectiveStamp(const std_msgs::Header& header)
{
    return header.stamp.isZero() ? ros::Time::now() : header.stamp;
}

std::string appendBackend(const std::string& backend)
{
    if (backend.empty())
    {
        return "bytetrack";
    }
    return backend + "+bytetrack";
}

std::string trim(const std::string& value)
{
    const std::size_t first = value.find_first_not_of(" \t\r\n");
    if (first == std::string::npos)
    {
        return "";
    }
    const std::size_t last = value.find_last_not_of(" \t\r\n");
    return value.substr(first, last - first + 1U);
}

std::unordered_set<std::string> parseClassFilter(const std::string& csv)
{
    std::unordered_set<std::string> classes;
    std::stringstream stream(csv);
    std::string token;
    while (std::getline(stream, token, ','))
    {
        token = trim(token);
        if (!token.empty())
        {
            classes.insert(token);
        }
    }
    return classes;
}

std::string describeClassFilter(const std::unordered_set<std::string>& classes)
{
    if (classes.empty())
    {
        return "all";
    }

    std::vector<std::string> sorted(classes.begin(), classes.end());
    std::sort(sorted.begin(), sorted.end());
    std::ostringstream stream;
    for (std::size_t index = 0; index < sorted.size(); ++index)
    {
        if (index != 0U)
        {
            stream << ',';
        }
        stream << sorted[index];
    }
    return stream.str();
}

}  // namespace

class ByteTrackNode
{
public:
    ByteTrackNode()
        : nh_(), private_nh_("~")
    {
        private_nh_.param<std::string>("detections_topic", detections_topic_,
                                       "/yolo_world/enhanced_detections");
        private_nh_.param<std::string>("tracked_topic", tracked_topic_,
                                       "/yolo_world/tracked_detections");
        private_nh_.param("high_confidence_threshold", high_confidence_threshold_, 0.25F);
        private_nh_.param("low_confidence_threshold", low_confidence_threshold_, 0.10F);
        private_nh_.param("new_track_threshold", new_track_threshold_, 0.25F);
        private_nh_.param("match_iou_threshold", match_iou_threshold_, 0.30F);
        private_nh_.param("max_lost_seconds", max_lost_seconds_, 1.00);
        private_nh_.param("publish_lost_tracks", publish_lost_tracks_, false);
        private_nh_.param("lost_track_confidence_scale", lost_track_confidence_scale_, 0.75F);
        private_nh_.param("suppress_lost_tracks_when_class_seen",
                          suppress_lost_tracks_when_class_seen_, true);
        private_nh_.param("class_aware", class_aware_, true);
        std::string tracked_classes_csv;
        private_nh_.param<std::string>("tracked_classes", tracked_classes_csv,
                                       std::string(""));
        private_nh_.param("min_track_hits", min_track_hits_, 1);
        std::string delay_output_classes_csv;
        private_nh_.param<std::string>("delay_output_classes",
                                       delay_output_classes_csv,
                                       std::string(""));
        std::string suppress_lost_track_classes_csv;
        private_nh_.param<std::string>("suppress_lost_track_classes",
                                       suppress_lost_track_classes_csv,
                                       std::string(""));
        private_nh_.param("passthrough_untracked_classes",
                          passthrough_untracked_classes_, true);
        tracked_classes_ = parseClassFilter(tracked_classes_csv);
        delay_output_classes_ = parseClassFilter(delay_output_classes_csv);
        suppress_lost_track_classes_ = parseClassFilter(suppress_lost_track_classes_csv);

        if (detections_topic_.empty() || tracked_topic_.empty())
        {
            throw std::invalid_argument("tracking topics must not be empty");
        }
        if (low_confidence_threshold_ < 0.0F || low_confidence_threshold_ > 1.0F ||
            high_confidence_threshold_ < low_confidence_threshold_ ||
            high_confidence_threshold_ > 1.0F || new_track_threshold_ < 0.0F ||
            new_track_threshold_ > 1.0F || match_iou_threshold_ <= 0.0F ||
            match_iou_threshold_ > 1.0F || max_lost_seconds_ < 0.0 ||
            lost_track_confidence_scale_ < 0.0F || lost_track_confidence_scale_ > 1.0F ||
            min_track_hits_ <= 0)
        {
            throw std::invalid_argument("invalid ByteTrack thresholds");
        }

        publisher_ = nh_.advertise<vision_msgs::DetectionArray>(tracked_topic_, 2);
        subscriber_ = nh_.subscribe(detections_topic_, 2, &ByteTrackNode::callback, this);

        ROS_INFO("ByteTrack-style tracker ready: detections=%s tracked=%s high=%.2f low=%.2f new=%.2f iou=%.2f max_lost=%.2fs publish_lost=%s class_aware=%s classes=%s delay_hits=%d delay_classes=%s suppress_lost=%s passthrough_untracked=%s",
                 detections_topic_.c_str(), tracked_topic_.c_str(),
                 high_confidence_threshold_, low_confidence_threshold_,
                 new_track_threshold_, match_iou_threshold_, max_lost_seconds_,
                 publish_lost_tracks_ ? "true" : "false",
                 class_aware_ ? "true" : "false",
                 describeClassFilter(tracked_classes_).c_str(),
                 min_track_hits_,
                 describeClassFilter(delay_output_classes_).c_str(),
                 describeClassFilter(suppress_lost_track_classes_).c_str(),
                 passthrough_untracked_classes_ ? "true" : "false");
    }

private:
    struct Track
    {
        std::uint32_t id{0};
        std::string class_name;
        vision_msgs::DetectionResult detection;
        ros::Time last_seen;
        int hits{0};
    };

    struct MatchCandidate
    {
        float score{0.0F};
        std::size_t track_index{0U};
        std::size_t detection_index{0U};
    };

    struct MatchResult
    {
        std::size_t track_index{0U};
        std::size_t detection_index{0U};
    };

    void callback(const vision_msgs::DetectionArrayConstPtr& message)
    {
        if (!message)
        {
            return;
        }

        const ros::Time stamp = effectiveStamp(message->header);
        expireTracks(stamp);

        vision_msgs::DetectionArray tracked = *message;
        tracked.backend = appendBackend(message->backend);
        for (auto& detection : tracked.objects)
        {
            detection.track_id = 0U;
        }

        std::vector<std::size_t> high_detections;
        std::vector<std::size_t> low_detections;
        partitionDetections(tracked.objects, &high_detections, &low_detections);

        std::vector<std::size_t> track_indices = activeTrackIndices();
        std::vector<bool> matched_tracks(tracks_.size(), false);
        std::vector<bool> matched_detections(tracked.objects.size(), false);

        applyMatches(match(track_indices, high_detections, tracked.objects), stamp,
                     &tracked.objects, &matched_tracks, &matched_detections);

        std::vector<std::size_t> unmatched_tracks;
        for (const std::size_t track_index : track_indices)
        {
            if (!matched_tracks[track_index])
            {
                unmatched_tracks.push_back(track_index);
            }
        }
        applyMatches(match(unmatched_tracks, low_detections, tracked.objects), stamp,
                     &tracked.objects, &matched_tracks, &matched_detections);

        for (const std::size_t detection_index : high_detections)
        {
            if (!matched_detections[detection_index] &&
                tracked.objects[detection_index].confidence >= new_track_threshold_)
            {
                createTrack(tracked.objects[detection_index], stamp);
                tracked.objects[detection_index].track_id = tracks_.back().id;
                tracks_.back().detection.track_id = tracks_.back().id;
                matched_detections[detection_index] = true;
            }
        }

        tracked.objects = stableOutput(tracked.objects, matched_detections,
                                       matched_tracks, stamp);

        publisher_.publish(tracked);
    }

    void partitionDetections(const std::vector<vision_msgs::DetectionResult>& detections,
                             std::vector<std::size_t>* high_detections,
                             std::vector<std::size_t>* low_detections) const
    {
        for (std::size_t index = 0; index < detections.size(); ++index)
        {
            const auto& detection = detections[index];
            if (!shouldTrackClass(detection.class_name) || !validBox(detection) ||
                detection.confidence < low_confidence_threshold_)
            {
                continue;
            }
            if (detection.confidence >= high_confidence_threshold_)
            {
                high_detections->push_back(index);
            }
            else
            {
                low_detections->push_back(index);
            }
        }
    }

    std::vector<std::size_t> activeTrackIndices() const
    {
        std::vector<std::size_t> indices;
        indices.reserve(tracks_.size());
        for (std::size_t index = 0; index < tracks_.size(); ++index)
        {
            indices.push_back(index);
        }
        return indices;
    }

    std::vector<MatchResult> match(
        const std::vector<std::size_t>& track_indices,
        const std::vector<std::size_t>& detection_indices,
        const std::vector<vision_msgs::DetectionResult>& detections) const
    {
        std::vector<MatchCandidate> candidates;
        for (const std::size_t track_index : track_indices)
        {
            for (const std::size_t detection_index : detection_indices)
            {
                const auto& track = tracks_[track_index];
                const auto& detection = detections[detection_index];
                if (class_aware_ && track.class_name != detection.class_name)
                {
                    continue;
                }
                const float score = iou(track.detection, detection);
                if (score >= match_iou_threshold_)
                {
                    candidates.push_back({score, track_index, detection_index});
                }
            }
        }
        std::sort(candidates.begin(), candidates.end(),
                  [](const MatchCandidate& left, const MatchCandidate& right) {
                      return left.score > right.score;
                  });

        std::vector<bool> used_tracks(tracks_.size(), false);
        std::vector<bool> used_detections(detections.size(), false);
        std::vector<MatchResult> matches;
        for (const auto& candidate : candidates)
        {
            if (used_tracks[candidate.track_index] || used_detections[candidate.detection_index])
            {
                continue;
            }
            used_tracks[candidate.track_index] = true;
            used_detections[candidate.detection_index] = true;
            matches.push_back({candidate.track_index, candidate.detection_index});
        }
        return matches;
    }

    void applyMatches(const std::vector<MatchResult>& matches, const ros::Time& stamp,
                      std::vector<vision_msgs::DetectionResult>* detections,
                      std::vector<bool>* matched_tracks,
                      std::vector<bool>* matched_detections)
    {
        for (const auto& match_result : matches)
        {
            Track& track = tracks_[match_result.track_index];
            auto& detection = detections->at(match_result.detection_index);
            detection.track_id = track.id;
            track.detection = detection;
            track.class_name = detection.class_name;
            track.last_seen = stamp;
            ++track.hits;
            matched_tracks->at(match_result.track_index) = true;
            matched_detections->at(match_result.detection_index) = true;
        }
    }

    void createTrack(const vision_msgs::DetectionResult& detection, const ros::Time& stamp)
    {
        Track track;
        track.id = next_track_id_++;
        if (next_track_id_ == 0U)
        {
            next_track_id_ = 1U;
        }
        track.class_name = detection.class_name;
        track.detection = detection;
        track.last_seen = stamp;
        track.hits = 1;
        tracks_.push_back(track);
    }

    std::vector<vision_msgs::DetectionResult> stableOutput(
        const std::vector<vision_msgs::DetectionResult>& detections,
        const std::vector<bool>& matched_detections,
        const std::vector<bool>& matched_tracks,
        const ros::Time& stamp) const
    {
        std::vector<vision_msgs::DetectionResult> output;
        output.reserve(detections.size() + tracks_.size());
        for (std::size_t index = 0; index < detections.size(); ++index)
        {
            if ((matched_detections[index] &&
                 shouldPublishConfirmedDetection(detections[index])) ||
                shouldPassThroughDetection(detections[index]))
            {
                output.push_back(detections[index]);
            }
        }

        if (publish_lost_tracks_)
        {
            for (std::size_t track_index = 0; track_index < matched_tracks.size(); ++track_index)
            {
                if (matched_tracks[track_index])
                {
                    continue;
                }
                const Track& track = tracks_[track_index];
                if (!shouldPublishHeldTrack(track, output, stamp))
                {
                    continue;
                }
                vision_msgs::DetectionResult held = track.detection;
                held.track_id = track.id;
                held.confidence = std::max(low_confidence_threshold_,
                                           held.confidence * lost_track_confidence_scale_);
                output.push_back(held);
            }
        }

        std::sort(output.begin(), output.end(),
                  [](const vision_msgs::DetectionResult& left,
                     const vision_msgs::DetectionResult& right) {
                      return left.confidence > right.confidence;
                  });
        return output;
    }

    bool shouldPublishConfirmedDetection(
        const vision_msgs::DetectionResult& detection) const
    {
        if (min_track_hits_ <= 1 || delay_output_classes_.count(detection.class_name) == 0U ||
            detection.track_id == 0U)
        {
            return true;
        }
        const auto track = std::find_if(
            tracks_.begin(), tracks_.end(),
            [&detection](const Track& candidate) {
                return candidate.id == detection.track_id;
            });
        return track == tracks_.end() || track->hits >= min_track_hits_;
    }

    bool shouldPublishHeldTrack(
        const Track& track,
        const std::vector<vision_msgs::DetectionResult>& current_output,
        const ros::Time& stamp) const
    {
        if (!validBox(track.detection) || track.last_seen.isZero())
        {
            return false;
        }
        if (suppress_lost_track_classes_.count(track.class_name) != 0U)
        {
            return false;
        }
        const double age = (stamp - track.last_seen).toSec();
        if (age < 0.0 || age > max_lost_seconds_)
        {
            return false;
        }
        if (!suppress_lost_tracks_when_class_seen_)
        {
            return true;
        }
        return std::none_of(current_output.begin(), current_output.end(),
                            [this, &track](const vision_msgs::DetectionResult& detection) {
                                return detection.class_name == track.class_name &&
                                       iou(track.detection, detection) >=
                                           match_iou_threshold_;
                            });
    }

    bool shouldTrackClass(const std::string& class_name) const
    {
        return tracked_classes_.empty() || tracked_classes_.count(class_name) != 0U;
    }

    bool shouldPassThroughDetection(const vision_msgs::DetectionResult& detection) const
    {
        return passthrough_untracked_classes_ && !shouldTrackClass(detection.class_name);
    }

    void expireTracks(const ros::Time& stamp)
    {
        tracks_.erase(
            std::remove_if(tracks_.begin(), tracks_.end(),
                           [this, &stamp](const Track& track) {
                               if (track.last_seen.isZero())
                               {
                                   return false;
                               }
                               const double age = (stamp - track.last_seen).toSec();
                               return age > max_lost_seconds_;
                           }),
            tracks_.end());
    }

    ros::NodeHandle nh_;
    ros::NodeHandle private_nh_;
    ros::Subscriber subscriber_;
    ros::Publisher publisher_;

    std::string detections_topic_;
    std::string tracked_topic_;
    float high_confidence_threshold_{0.25F};
    float low_confidence_threshold_{0.10F};
    float new_track_threshold_{0.25F};
    float match_iou_threshold_{0.30F};
    double max_lost_seconds_{1.00};
    bool publish_lost_tracks_{false};
    float lost_track_confidence_scale_{0.75F};
    bool suppress_lost_tracks_when_class_seen_{true};
    bool class_aware_{true};
    bool passthrough_untracked_classes_{true};
    std::unordered_set<std::string> tracked_classes_;
    int min_track_hits_{1};
    std::unordered_set<std::string> delay_output_classes_;
    std::unordered_set<std::string> suppress_lost_track_classes_;

    std::vector<Track> tracks_;
    std::uint32_t next_track_id_{1U};
};

int main(int argc, char** argv)
{
    ros::init(argc, argv, "bytetrack_node");
    try
    {
        ByteTrackNode node;
        ros::spin();
    }
    catch (const std::exception& exception)
    {
        ROS_FATAL("ByteTrack-style tracker failed: %s", exception.what());
        return 1;
    }
    return 0;
}
