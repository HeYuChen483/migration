#include <garbage_localization/localization_utils.hpp>

#include <gtest/gtest.h>

#include <cmath>
#include <limits>
#include <vector>

namespace garbage_localization
{

TEST(BoundingBoxTest, ClipsAndShrinksAtImageBoundary)
{
  BoundingBox output;
  ASSERT_TRUE(clipAndShrinkBoundingBox({-10.0, 20.0, 110.0, 100.0},
                                      1920, 1080, 0.10, 4.0, &output));
  EXPECT_DOUBLE_EQ(10.0, output.x);
  EXPECT_DOUBLE_EQ(30.0, output.y);
  EXPECT_DOUBLE_EQ(80.0, output.width);
  EXPECT_DOUBLE_EQ(80.0, output.height);
}

TEST(BoundingBoxTest, RejectsInvalidOrTinyBoxes)
{
  BoundingBox output;
  EXPECT_FALSE(clipAndShrinkBoundingBox({0.0, 0.0, -1.0, 10.0},
                                       100, 100, 0.1, 4.0, &output));
  EXPECT_FALSE(clipAndShrinkBoundingBox(
      {0.0, 0.0, std::numeric_limits<double>::quiet_NaN(), 10.0},
      100, 100, 0.1, 4.0, &output));
  EXPECT_FALSE(clipAndShrinkBoundingBox({99.0, 99.0, 2.0, 2.0},
                                       100, 100, 0.1, 4.0, &output));
}

TEST(ProjectionTest, ProjectsUsingPinholeIntrinsics)
{
  CameraIntrinsics intrinsics{100.0, 120.0, 320.0, 240.0, 640, 480};
  double u = 0.0;
  double v = 0.0;
  ASSERT_TRUE(projectPoint({1.0, -0.5, 2.0}, intrinsics, &u, &v));
  EXPECT_DOUBLE_EQ(370.0, u);
  EXPECT_DOUBLE_EQ(210.0, v);
  EXPECT_TRUE(containsPixel({360.0, 200.0, 30.0, 30.0}, u, v));
  EXPECT_FALSE(projectPoint({1.0, 1.0, -1.0}, intrinsics, &u, &v));
}

TEST(RobustEstimateTest, RejectsDepthOutliers)
{
  std::vector<Point3> points;
  for (int index = 0; index < 30; ++index)
  {
    const double offset = static_cast<double>(index % 5) * 0.001;
    points.push_back({0.10 + offset, 0.20 - offset, 2.0 + offset});
  }
  points.push_back({5.0, 5.0, 5.0});
  points.push_back({-3.0, -2.0, 0.5});

  RobustEstimateParameters parameters;
  RobustEstimate estimate;
  ASSERT_TRUE(robustEstimate(points, parameters, &estimate));
  EXPECT_NEAR(0.102, estimate.point.x, 0.003);
  EXPECT_NEAR(0.198, estimate.point.y, 0.003);
  EXPECT_NEAR(2.002, estimate.point.z, 0.003);
  EXPECT_EQ(30u, estimate.inlier_count);
}

TEST(RobustEstimateTest, RejectsInsufficientFinitePoints)
{
  RobustEstimateParameters parameters;
  RobustEstimate estimate;
  std::vector<Point3> points(19, {0.0, 0.0, 1.0});
  points.push_back({0.0, 0.0, std::numeric_limits<double>::quiet_NaN()});
  EXPECT_FALSE(robustEstimate(points, parameters, &estimate));
}

TEST(WorkspaceTest, EnforcesAllAxesAndFiniteValues)
{
  EXPECT_TRUE(isInsideWorkspace({1.0, 0.0, 0.1}, 0.2, 6.0, -3.0, 3.0,
                                -0.1, 2.0));
  EXPECT_FALSE(isInsideWorkspace({0.0, 0.0, 0.1}, 0.2, 6.0, -3.0, 3.0,
                                 -0.1, 2.0));
  EXPECT_FALSE(isInsideWorkspace(
      {1.0, std::numeric_limits<double>::infinity(), 0.1},
      0.2, 6.0, -3.0, 3.0, -0.1, 2.0));
}

TEST(WorkspaceTest, RejectsFloorPointsAtSimulationBottleBoundary)
{
  EXPECT_FALSE(isInsideWorkspace({2.25, -0.12, 0.0176}, 0.2, 6.0,
                                 -3.0, 3.0, 0.05, 2.0));
  EXPECT_TRUE(isInsideWorkspace({2.25, -0.12, 0.05}, 0.2, 6.0,
                                -3.0, 3.0, 0.05, 2.0));
  EXPECT_TRUE(isInsideWorkspace({2.25, -0.12, 0.12}, 0.2, 6.0,
                                -3.0, 3.0, 0.05, 2.0));
}

}  // namespace garbage_localization

int main(int argc, char** argv)
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
