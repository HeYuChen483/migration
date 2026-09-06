#include "garbage_localization/localization_utils.hpp"

#include <algorithm>
#include <cmath>
#include <limits>

namespace garbage_localization
{
namespace
{

double median(std::vector<double> values)
{
  if (values.empty())
  {
    return std::numeric_limits<double>::quiet_NaN();
  }

  const std::size_t middle = values.size() / 2;
  std::nth_element(values.begin(), values.begin() + middle, values.end());
  const double upper = values[middle];
  if (values.size() % 2 != 0)
  {
    return upper;
  }

  const double lower = *std::max_element(values.begin(), values.begin() + middle);
  return 0.5 * (lower + upper);
}

}  // namespace

bool isFinite(const Point3& point)
{
  return std::isfinite(point.x) && std::isfinite(point.y) &&
         std::isfinite(point.z);
}

bool validIntrinsics(const CameraIntrinsics& intrinsics)
{
  return intrinsics.width > 0 && intrinsics.height > 0 &&
         std::isfinite(intrinsics.fx) && std::isfinite(intrinsics.fy) &&
         std::isfinite(intrinsics.cx) && std::isfinite(intrinsics.cy) &&
         intrinsics.fx > 0.0 && intrinsics.fy > 0.0;
}

bool clipAndShrinkBoundingBox(const BoundingBox& input, const int image_width,
                              const int image_height, const double shrink_ratio,
                              const double minimum_size, BoundingBox* output)
{
  if (output == nullptr || image_width <= 0 || image_height <= 0 ||
      !std::isfinite(input.x) || !std::isfinite(input.y) ||
      !std::isfinite(input.width) || !std::isfinite(input.height) ||
      input.width <= 0.0 || input.height <= 0.0 ||
      !std::isfinite(shrink_ratio) || shrink_ratio < 0.0 ||
      shrink_ratio >= 0.5 || !std::isfinite(minimum_size) ||
      minimum_size <= 0.0)
  {
    return false;
  }

  const double left = std::max(0.0, input.x);
  const double top = std::max(0.0, input.y);
  const double right = std::min(static_cast<double>(image_width),
                                input.x + input.width);
  const double bottom = std::min(static_cast<double>(image_height),
                                 input.y + input.height);
  const double clipped_width = right - left;
  const double clipped_height = bottom - top;
  if (clipped_width < minimum_size || clipped_height < minimum_size)
  {
    return false;
  }

  const double margin_x = clipped_width * shrink_ratio;
  const double margin_y = clipped_height * shrink_ratio;
  output->x = left + margin_x;
  output->y = top + margin_y;
  output->width = clipped_width - 2.0 * margin_x;
  output->height = clipped_height - 2.0 * margin_y;
  return output->width >= minimum_size && output->height >= minimum_size;
}

bool projectPoint(const Point3& point, const CameraIntrinsics& intrinsics,
                  double* u, double* v)
{
  if (u == nullptr || v == nullptr || !isFinite(point) || point.z <= 0.0 ||
      !validIntrinsics(intrinsics))
  {
    return false;
  }

  *u = intrinsics.fx * point.x / point.z + intrinsics.cx;
  *v = intrinsics.fy * point.y / point.z + intrinsics.cy;
  return std::isfinite(*u) && std::isfinite(*v);
}

bool containsPixel(const BoundingBox& box, const double u, const double v)
{
  return std::isfinite(u) && std::isfinite(v) && u >= box.x && v >= box.y &&
         u < box.x + box.width && v < box.y + box.height;
}

bool robustEstimate(const std::vector<Point3>& candidates,
                    const RobustEstimateParameters& parameters,
                    RobustEstimate* estimate)
{
  if (estimate == nullptr ||
      candidates.size() < parameters.minimum_candidate_points ||
      parameters.minimum_inlier_points == 0 ||
      !std::isfinite(parameters.minimum_inlier_ratio) ||
      parameters.minimum_inlier_ratio < 0.0 ||
      parameters.minimum_inlier_ratio > 1.0 ||
      !std::isfinite(parameters.mad_scale) || parameters.mad_scale < 0.0 ||
      !std::isfinite(parameters.minimum_depth_band) ||
      parameters.minimum_depth_band < 0.0)
  {
    return false;
  }

  std::vector<Point3> finite_candidates;
  std::vector<double> depths;
  finite_candidates.reserve(candidates.size());
  depths.reserve(candidates.size());
  for (const Point3& point : candidates)
  {
    if (isFinite(point) && point.z > 0.0)
    {
      finite_candidates.push_back(point);
      depths.push_back(point.z);
    }
  }
  if (finite_candidates.size() < parameters.minimum_candidate_points)
  {
    return false;
  }

  const double median_depth = median(depths);
  std::vector<double> deviations;
  deviations.reserve(depths.size());
  for (const double depth : depths)
  {
    deviations.push_back(std::abs(depth - median_depth));
  }
  const double mad = median(deviations);
  const double depth_band =
      std::max(parameters.mad_scale * 1.4826 * mad,
               parameters.minimum_depth_band);

  std::vector<double> xs;
  std::vector<double> ys;
  std::vector<double> zs;
  xs.reserve(finite_candidates.size());
  ys.reserve(finite_candidates.size());
  zs.reserve(finite_candidates.size());
  for (const Point3& point : finite_candidates)
  {
    if (std::abs(point.z - median_depth) <= depth_band)
    {
      xs.push_back(point.x);
      ys.push_back(point.y);
      zs.push_back(point.z);
    }
  }

  if (zs.size() < parameters.minimum_inlier_points ||
      static_cast<double>(zs.size()) /
              static_cast<double>(finite_candidates.size()) <
          parameters.minimum_inlier_ratio)
  {
    return false;
  }

  estimate->point = {median(xs), median(ys), median(zs)};
  estimate->inlier_count = zs.size();
  estimate->median_depth = median_depth;
  return isFinite(estimate->point);
}

bool isInsideWorkspace(const Point3& point, const double min_x,
                       const double max_x, const double min_y,
                       const double max_y, const double min_z,
                       const double max_z)
{
  return isFinite(point) && min_x <= max_x && min_y <= max_y &&
         min_z <= max_z && point.x >= min_x && point.x <= max_x &&
         point.y >= min_y && point.y <= max_y && point.z >= min_z &&
         point.z <= max_z;
}

}  // namespace garbage_localization
