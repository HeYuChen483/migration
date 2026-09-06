#include <garbage_localization/GarbageObject.h>
#include <garbage_localization/GarbageObjectArray.h>
#include <garbage_localization/localization_utils.hpp>

#include <diagnostic_msgs/DiagnosticArray.h>
#include <diagnostic_msgs/DiagnosticStatus.h>
#include <diagnostic_msgs/KeyValue.h>
#include <geometry_msgs/TransformStamped.h>
#include <ros/ros.h>
#include <sensor_msgs/CameraInfo.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/point_cloud2_iterator.h>
#include <tf2/LinearMath/Transform.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <vision_msgs/DetectionArray.h>

#include <XmlRpcValue.h>

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <deque>
#include <limits>
#include <map>
#include <set>
#include <sstream>
#include <stdexcept>
#include <string>
#include <unordered_set>
#include <utility>
#include <vector>

namespace garbage_localization
{
namespace
{

std::string headerKey(const std_msgs::Header& header)
{
  std::ostringstream stream;
  stream << header.stamp.sec << ':' << header.stamp.nsec << ':' << header.seq
         << ':' << header.frame_id;
  return stream.str();
}

diagnostic_msgs::KeyValue keyValue(const std::string& key,
                                   const std::string& value)
{
  diagnostic_msgs::KeyValue result;
  result.key = key;
  result.value = value;
  return result;
}

template <typename T>
std::string toString(const T& value)
{
  std::ostringstream stream;
  stream << value;
  return stream.str();
}

}  // namespace

class GarbageLocalizationNode
{
public:
  GarbageLocalizationNode()
    : private_node_handle_("~"), tf_listener_(tf_buffer_)
  {
    loadParameters();

    object_publisher_ = node_handle_.advertise<GarbageObjectArray>(output_topic_, 10);
    diagnostics_publisher_ =
        node_handle_.advertise<diagnostic_msgs::DiagnosticArray>(diagnostics_topic_, 10);
    cloud_subscriber_ = node_handle_.subscribe(
        point_cloud_topic_, 10, &GarbageLocalizationNode::cloudCallback, this);
    camera_info_subscriber_ = node_handle_.subscribe(
        camera_info_topic_, 2, &GarbageLocalizationNode::cameraInfoCallback, this);
    detection_subscriber_ = node_handle_.subscribe(
        detection_topic_, 10, &GarbageLocalizationNode::detectionCallback, this);

    ROS_INFO_STREAM("garbage_localization: detections=" << detection_topic_
                    << ", cloud=" << point_cloud_topic_
                    << ", camera_info=" << camera_info_topic_
                    << ", output=" << output_topic_);
  }

private:
  struct DetectionCandidate
  {
    const vision_msgs::DetectionResult* detection{nullptr};
    BoundingBox box;
    std::vector<Point3> points;
  };

  void loadParameters()
  {
    private_node_handle_.param("detection_topic", detection_topic_,
                               std::string("/vision/fused_detections"));
    private_node_handle_.param("point_cloud_topic", point_cloud_topic_,
                               std::string("/kinect2/sd/points"));
    private_node_handle_.param("camera_info_topic", camera_info_topic_,
                               std::string("/kinect2/hd/camera_info"));
    private_node_handle_.param("output_topic", output_topic_,
                               std::string("/garbage_localization/objects"));
    private_node_handle_.param("diagnostics_topic", diagnostics_topic_,
                               std::string("/diagnostics"));
    private_node_handle_.param("output_frame", output_frame_,
                               std::string("base_link"));
    private_node_handle_.param("rgb_frame", rgb_frame_,
                               std::string("kinect2_rgb_optical_frame"));

    private_node_handle_.param("minimum_detection_confidence",
                               minimum_detection_confidence_, 0.25);
    private_node_handle_.param("process_first_header_only",
                               process_first_header_only_, true);
    private_node_handle_.param("duplicate_cache_size", duplicate_cache_size_, 300);
    private_node_handle_.param("cloud_cache_size", cloud_cache_size_, 40);
    private_node_handle_.param("cloud_cache_seconds", cloud_cache_seconds_, 3.0);
    private_node_handle_.param("max_cloud_time_delta", max_cloud_time_delta_, 0.08);
    private_node_handle_.param("tf_timeout", tf_timeout_, 0.10);
    private_node_handle_.param("min_depth", min_depth_, 0.5);
    private_node_handle_.param("max_depth", max_depth_, 6.0);
    private_node_handle_.param("roi_shrink_ratio", roi_shrink_ratio_, 0.10);
    private_node_handle_.param("minimum_bbox_size", minimum_bbox_size_, 4.0);

    int minimum_candidate_points = 20;
    int minimum_inlier_points = 12;
    private_node_handle_.param("minimum_candidate_points",
                               minimum_candidate_points, 20);
    private_node_handle_.param("minimum_inlier_points", minimum_inlier_points, 12);
    estimate_parameters_.minimum_candidate_points =
        static_cast<std::size_t>(std::max(1, minimum_candidate_points));
    estimate_parameters_.minimum_inlier_points =
        static_cast<std::size_t>(std::max(1, minimum_inlier_points));
    private_node_handle_.param("minimum_inlier_ratio",
                               estimate_parameters_.minimum_inlier_ratio, 0.25);
    private_node_handle_.param("mad_scale", estimate_parameters_.mad_scale, 2.5);
    private_node_handle_.param("minimum_depth_band",
                               estimate_parameters_.minimum_depth_band, 0.04);

    private_node_handle_.param("workspace_min_x", workspace_min_x_, 0.20);
    private_node_handle_.param("workspace_max_x", workspace_max_x_, 6.00);
    private_node_handle_.param("workspace_min_y", workspace_min_y_, -3.00);
    private_node_handle_.param("workspace_max_y", workspace_max_y_, 3.00);
    private_node_handle_.param("workspace_min_z", workspace_min_z_, -0.10);
    private_node_handle_.param("workspace_max_z", workspace_max_z_, 2.00);

    XmlRpc::XmlRpcValue classes;
    if (private_node_handle_.getParam("target_classes", classes) &&
        classes.getType() == XmlRpc::XmlRpcValue::TypeArray)
    {
      for (int index = 0; index < classes.size(); ++index)
      {
        if (classes[index].getType() == XmlRpc::XmlRpcValue::TypeString)
        {
          target_classes_.insert(static_cast<std::string>(classes[index]));
        }
      }
    }
    if (target_classes_.empty())
    {
      target_classes_.insert("bottle");
    }

    duplicate_cache_size_ = std::max(1, duplicate_cache_size_);
    cloud_cache_size_ = std::max(1, cloud_cache_size_);
  }

  void cloudCallback(const sensor_msgs::PointCloud2ConstPtr& cloud)
  {
    if (!cloud || cloud->header.stamp.isZero())
    {
      return;
    }

    cloud_cache_.push_back(cloud);
    last_cloud_time_ = ros::Time::now();
    while (cloud_cache_.size() > static_cast<std::size_t>(cloud_cache_size_))
    {
      cloud_cache_.pop_front();
    }
    while (!cloud_cache_.empty() &&
           (cloud->header.stamp - cloud_cache_.front()->header.stamp).toSec() >
               cloud_cache_seconds_)
    {
      cloud_cache_.pop_front();
    }
  }

  void cameraInfoCallback(const sensor_msgs::CameraInfoConstPtr& camera_info)
  {
    if (camera_info)
    {
      camera_info_ = camera_info;
      last_camera_info_time_ = ros::Time::now();
    }
  }

  sensor_msgs::PointCloud2ConstPtr nearestCloud(const ros::Time& stamp,
                                                double* time_delta) const
  {
    sensor_msgs::PointCloud2ConstPtr best;
    double best_delta = std::numeric_limits<double>::infinity();
    for (const sensor_msgs::PointCloud2ConstPtr& cloud : cloud_cache_)
    {
      const double delta = std::abs((cloud->header.stamp - stamp).toSec());
      if (delta < best_delta)
      {
        best = cloud;
        best_delta = delta;
      }
    }
    if (time_delta != nullptr)
    {
      *time_delta = best_delta;
    }
    return best;
  }

  bool rememberHeader(const std_msgs::Header& header)
  {
    if (!process_first_header_only_)
    {
      return true;
    }

    const std::string key = headerKey(header);
    if (seen_headers_.count(key) != 0)
    {
      increment("duplicate_header");
      return false;
    }
    seen_headers_.insert(key);
    header_order_.push_back(key);
    while (header_order_.size() > static_cast<std::size_t>(duplicate_cache_size_))
    {
      seen_headers_.erase(header_order_.front());
      header_order_.pop_front();
    }
    return true;
  }

  bool cameraIntrinsics(const vision_msgs::DetectionArray& detections,
                        CameraIntrinsics* intrinsics) const
  {
    if (!camera_info_ || intrinsics == nullptr || camera_info_->width == 0 ||
        camera_info_->height == 0 || camera_info_->K[0] <= 0.0 ||
        camera_info_->K[4] <= 0.0 || detections.image_width != camera_info_->width ||
        detections.image_height != camera_info_->height)
    {
      return false;
    }
    if (!detections.header.frame_id.empty() &&
        !camera_info_->header.frame_id.empty() &&
        detections.header.frame_id != camera_info_->header.frame_id)
    {
      return false;
    }

    intrinsics->fx = camera_info_->K[0];
    intrinsics->fy = camera_info_->K[4];
    intrinsics->cx = camera_info_->K[2];
    intrinsics->cy = camera_info_->K[5];
    intrinsics->width = static_cast<int>(camera_info_->width);
    intrinsics->height = static_cast<int>(camera_info_->height);
    return validIntrinsics(*intrinsics);
  }

  std::vector<DetectionCandidate> prepareCandidates(
      const vision_msgs::DetectionArray& detections,
      const CameraIntrinsics& intrinsics)
  {
    std::vector<DetectionCandidate> candidates;
    for (const vision_msgs::DetectionResult& detection : detections.objects)
    {
      if (!detection.has_bbox)
      {
        increment("invalid_bbox");
        continue;
      }
      if (target_classes_.count(detection.class_name) == 0 ||
          detection.confidence < minimum_detection_confidence_)
      {
        increment("class_filtered");
        continue;
      }

      BoundingBox clipped;
      const BoundingBox input{detection.x, detection.y, detection.width,
                              detection.height};
      if (!clipAndShrinkBoundingBox(input, intrinsics.width, intrinsics.height,
                                    roi_shrink_ratio_, minimum_bbox_size_,
                                    &clipped))
      {
        increment("invalid_bbox");
        continue;
      }

      DetectionCandidate candidate;
      candidate.detection = &detection;
      candidate.box = clipped;
      candidates.push_back(std::move(candidate));
    }
    return candidates;
  }

  bool lookupTransforms(const sensor_msgs::PointCloud2& cloud,
                        tf2::Transform* depth_to_rgb,
                        tf2::Transform* rgb_to_output)
  {
    if (cloud.header.frame_id.empty() || depth_to_rgb == nullptr ||
        rgb_to_output == nullptr)
    {
      return false;
    }

    try
    {
      const geometry_msgs::TransformStamped depth_transform =
          tf_buffer_.lookupTransform(rgb_frame_, cloud.header.frame_id,
                                     cloud.header.stamp,
                                     ros::Duration(tf_timeout_));
      const geometry_msgs::TransformStamped output_transform =
          tf_buffer_.lookupTransform(output_frame_, rgb_frame_,
                                     cloud.header.stamp,
                                     ros::Duration(tf_timeout_));
      tf2::fromMsg(depth_transform.transform, *depth_to_rgb);
      tf2::fromMsg(output_transform.transform, *rgb_to_output);
      return true;
    }
    catch (const tf2::TransformException& exception)
    {
      ROS_WARN_THROTTLE(2.0, "garbage_localization TF unavailable: %s",
                        exception.what());
      return false;
    }
  }

  bool collectProjectedPoints(const sensor_msgs::PointCloud2& cloud,
                              const CameraIntrinsics& intrinsics,
                              const tf2::Transform& depth_to_rgb,
                              std::vector<DetectionCandidate>* candidates)
  {
    if (candidates == nullptr)
    {
      return false;
    }

    bool has_x = false;
    bool has_y = false;
    bool has_z = false;
    for (const sensor_msgs::PointField& field : cloud.fields)
    {
      const bool valid_type = field.datatype == sensor_msgs::PointField::FLOAT32;
      has_x = has_x || (field.name == "x" && valid_type);
      has_y = has_y || (field.name == "y" && valid_type);
      has_z = has_z || (field.name == "z" && valid_type);
    }
    if (!has_x || !has_y || !has_z)
    {
      return false;
    }

    try
    {
      sensor_msgs::PointCloud2ConstIterator<float> x_iterator(cloud, "x");
      sensor_msgs::PointCloud2ConstIterator<float> y_iterator(cloud, "y");
      sensor_msgs::PointCloud2ConstIterator<float> z_iterator(cloud, "z");
      for (; x_iterator != x_iterator.end();
           ++x_iterator, ++y_iterator, ++z_iterator)
      {
        const Point3 depth_point{*x_iterator, *y_iterator, *z_iterator};
        if (!isFinite(depth_point) || depth_point.z < min_depth_ ||
            depth_point.z > max_depth_)
        {
          continue;
        }

        const tf2::Vector3 transformed =
            depth_to_rgb * tf2::Vector3(depth_point.x, depth_point.y,
                                        depth_point.z);
        const Point3 rgb_point{transformed.x(), transformed.y(), transformed.z()};
        double u = 0.0;
        double v = 0.0;
        if (!projectPoint(rgb_point, intrinsics, &u, &v))
        {
          continue;
        }
        for (DetectionCandidate& candidate : *candidates)
        {
          if (containsPixel(candidate.box, u, v))
          {
            candidate.points.push_back(rgb_point);
          }
        }
      }
    }
    catch (const std::runtime_error& exception)
    {
      ROS_ERROR_THROTTLE(2.0, "Invalid PointCloud2 layout: %s", exception.what());
      return false;
    }
    return true;
  }

  void detectionCallback(const vision_msgs::DetectionArrayConstPtr& detections)
  {
    if (!detections || detections->header.stamp.isZero())
    {
      increment("invalid_detection_header");
      publishDiagnostics(diagnostic_msgs::DiagnosticStatus::WARN,
                         "invalid detection header", 0, 0);
      return;
    }
    last_detection_time_ = ros::Time::now();
    if (!rememberHeader(detections->header))
    {
      publishDiagnostics(diagnostic_msgs::DiagnosticStatus::WARN,
                         "duplicate detection header", detections->objects.size(), 0);
      return;
    }

    CameraIntrinsics intrinsics;
    if (!cameraIntrinsics(*detections, &intrinsics))
    {
      increment("camera_info_invalid");
      publishEmpty(*detections, detections->header.stamp);
      publishDiagnostics(diagnostic_msgs::DiagnosticStatus::ERROR,
                         "camera info unavailable or incompatible",
                         detections->objects.size(), 0);
      return;
    }

    std::vector<DetectionCandidate> candidates =
        prepareCandidates(*detections, intrinsics);
    if (candidates.empty())
    {
      publishEmpty(*detections, detections->header.stamp);
      publishDiagnostics(diagnostic_msgs::DiagnosticStatus::WARN,
                         "no eligible target detections",
                         detections->objects.size(), 0);
      return;
    }

    double time_delta = std::numeric_limits<double>::infinity();
    const sensor_msgs::PointCloud2ConstPtr cloud =
        nearestCloud(detections->header.stamp, &time_delta);
    last_time_delta_ = time_delta;
    if (!cloud || time_delta > max_cloud_time_delta_)
    {
      increment("no_nearby_cloud");
      publishEmpty(*detections, detections->header.stamp);
      publishDiagnostics(diagnostic_msgs::DiagnosticStatus::WARN,
                         "no point cloud near detection timestamp",
                         detections->objects.size(), 0);
      return;
    }

    tf2::Transform depth_to_rgb;
    tf2::Transform rgb_to_output;
    if (!lookupTransforms(*cloud, &depth_to_rgb, &rgb_to_output))
    {
      increment("tf_unavailable");
      publishEmpty(*detections, cloud->header.stamp);
      publishDiagnostics(diagnostic_msgs::DiagnosticStatus::ERROR,
                         "timestamped TF unavailable",
                         detections->objects.size(), 0);
      return;
    }

    if (!collectProjectedPoints(*cloud, intrinsics, depth_to_rgb, &candidates))
    {
      increment("invalid_point_cloud");
      publishEmpty(*detections, cloud->header.stamp);
      publishDiagnostics(diagnostic_msgs::DiagnosticStatus::ERROR,
                         "point cloud fields are invalid",
                         detections->objects.size(), 0);
      return;
    }

    GarbageObjectArray output;
    output.header.stamp = cloud->header.stamp;
    output.header.frame_id = output_frame_;
    output.detection_header = detections->header;
    output.detector_backend = detections->backend;

    for (const DetectionCandidate& candidate : candidates)
    {
      RobustEstimate estimate;
      if (!robustEstimate(candidate.points, estimate_parameters_, &estimate))
      {
        increment(candidate.points.size() < estimate_parameters_.minimum_candidate_points
                      ? "not_enough_points"
                      : "outlier_rejection_empty");
        continue;
      }

      const tf2::Vector3 transformed = rgb_to_output *
          tf2::Vector3(estimate.point.x, estimate.point.y, estimate.point.z);
      const Point3 output_point{transformed.x(), transformed.y(), transformed.z()};
      if (!isInsideWorkspace(output_point, workspace_min_x_, workspace_max_x_,
                             workspace_min_y_, workspace_max_y_, workspace_min_z_,
                             workspace_max_z_))
      {
        increment("workspace_rejected");
        continue;
      }

      GarbageObject object;
      object.class_name = candidate.detection->class_name;
      object.confidence = candidate.detection->confidence;
      object.position.x = output_point.x;
      object.position.y = output_point.y;
      object.position.z = output_point.z;
      object.bbox_x = candidate.detection->x;
      object.bbox_y = candidate.detection->y;
      object.bbox_width = candidate.detection->width;
      object.bbox_height = candidate.detection->height;
      object.valid_point_count = static_cast<std::uint32_t>(estimate.inlier_count);
      object.median_depth = estimate.median_depth;
      output.objects.push_back(object);
    }

    object_publisher_.publish(output);
    publishDiagnostics(output.objects.empty() ? diagnostic_msgs::DiagnosticStatus::WARN
                                              : diagnostic_msgs::DiagnosticStatus::OK,
                       output.objects.empty() ? "no target localized" : "localization OK",
                       detections->objects.size(), output.objects.size());
  }

  void publishEmpty(const vision_msgs::DetectionArray& detections,
                    const ros::Time& stamp)
  {
    GarbageObjectArray output;
    output.header.stamp = stamp;
    output.header.frame_id = output_frame_;
    output.detection_header = detections.header;
    output.detector_backend = detections.backend;
    object_publisher_.publish(output);
  }

  void increment(const std::string& name)
  {
    ++failure_counts_[name];
  }

  void publishDiagnostics(const std::uint8_t level, const std::string& message,
                          const std::size_t input_count,
                          const std::size_t output_count)
  {
    diagnostic_msgs::DiagnosticArray array;
    array.header.stamp = ros::Time::now();
    diagnostic_msgs::DiagnosticStatus status;
    status.level = level;
    status.name = "garbage_localization/status";
    status.hardware_id = "rgbd_localizer";
    status.message = message;
    status.values.push_back(keyValue("input_detections", toString(input_count)));
    status.values.push_back(keyValue("localized_objects", toString(output_count)));
    status.values.push_back(keyValue("cloud_cache_size", toString(cloud_cache_.size())));
    status.values.push_back(keyValue("last_cloud_delta_sec", toString(last_time_delta_)));
    status.values.push_back(keyValue("output_frame", output_frame_));
    status.values.push_back(keyValue("rgb_frame", rgb_frame_));
    status.values.push_back(keyValue("last_detection_received", toString(last_detection_time_)));
    status.values.push_back(keyValue("last_cloud_received", toString(last_cloud_time_)));
    status.values.push_back(keyValue("last_camera_info_received", toString(last_camera_info_time_)));
    for (const auto& entry : failure_counts_)
    {
      status.values.push_back(keyValue(entry.first, toString(entry.second)));
    }
    array.status.push_back(status);
    diagnostics_publisher_.publish(array);
  }

  ros::NodeHandle node_handle_;
  ros::NodeHandle private_node_handle_;
  ros::Subscriber detection_subscriber_;
  ros::Subscriber cloud_subscriber_;
  ros::Subscriber camera_info_subscriber_;
  ros::Publisher object_publisher_;
  ros::Publisher diagnostics_publisher_;

  tf2_ros::Buffer tf_buffer_;
  tf2_ros::TransformListener tf_listener_;

  std::deque<sensor_msgs::PointCloud2ConstPtr> cloud_cache_;
  sensor_msgs::CameraInfoConstPtr camera_info_;
  std::set<std::string> target_classes_;
  std::unordered_set<std::string> seen_headers_;
  std::deque<std::string> header_order_;
  std::map<std::string, std::uint64_t> failure_counts_;

  std::string detection_topic_;
  std::string point_cloud_topic_;
  std::string camera_info_topic_;
  std::string output_topic_;
  std::string diagnostics_topic_;
  std::string output_frame_;
  std::string rgb_frame_;

  double minimum_detection_confidence_{0.25};
  bool process_first_header_only_{true};
  int duplicate_cache_size_{300};
  int cloud_cache_size_{40};
  double cloud_cache_seconds_{3.0};
  double max_cloud_time_delta_{0.08};
  double tf_timeout_{0.10};
  double min_depth_{0.5};
  double max_depth_{6.0};
  double roi_shrink_ratio_{0.10};
  double minimum_bbox_size_{4.0};
  RobustEstimateParameters estimate_parameters_;
  double workspace_min_x_{0.20};
  double workspace_max_x_{6.00};
  double workspace_min_y_{-3.00};
  double workspace_max_y_{3.00};
  double workspace_min_z_{-0.10};
  double workspace_max_z_{2.00};

  ros::Time last_detection_time_;
  ros::Time last_cloud_time_;
  ros::Time last_camera_info_time_;
  double last_time_delta_{std::numeric_limits<double>::infinity()};
};

}  // namespace garbage_localization

int main(int argc, char** argv)
{
  ros::init(argc, argv, "garbage_localization");
  garbage_localization::GarbageLocalizationNode node;
  ros::spin();
  return 0;
}
