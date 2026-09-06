#pragma once

#include <cstddef>
#include <string>
#include <vector>

namespace garbage_localization
{

struct Point3
{
  double x{0.0};
  double y{0.0};
  double z{0.0};
};

struct CameraIntrinsics
{
  double fx{0.0};
  double fy{0.0};
  double cx{0.0};
  double cy{0.0};
  int width{0};
  int height{0};
};

struct BoundingBox
{
  double x{0.0};
  double y{0.0};
  double width{0.0};
  double height{0.0};
};

struct RobustEstimateParameters
{
  std::size_t minimum_candidate_points{20};
  std::size_t minimum_inlier_points{12};
  double minimum_inlier_ratio{0.25};
  double mad_scale{2.5};
  double minimum_depth_band{0.04};
};

struct RobustEstimate
{
  Point3 point;
  std::size_t inlier_count{0};
  double median_depth{0.0};
};

bool isFinite(const Point3& point);
bool validIntrinsics(const CameraIntrinsics& intrinsics);
bool clipAndShrinkBoundingBox(const BoundingBox& input, int image_width,
                              int image_height, double shrink_ratio,
                              double minimum_size, BoundingBox* output);
bool projectPoint(const Point3& point, const CameraIntrinsics& intrinsics,
                  double* u, double* v);
bool containsPixel(const BoundingBox& box, double u, double v);
bool robustEstimate(const std::vector<Point3>& candidates,
                    const RobustEstimateParameters& parameters,
                    RobustEstimate* estimate);
bool isInsideWorkspace(const Point3& point, double min_x, double max_x,
                       double min_y, double max_y, double min_z,
                       double max_z);

}  // namespace garbage_localization
