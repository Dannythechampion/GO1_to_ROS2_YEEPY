#include <algorithm>
#include <cstdint>
#include <limits>
#include <stdexcept>

#include <gtest/gtest.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

#include "go1_mapping/pcd_projection.hpp"

namespace
{

using go1_mapping::Grid;
using go1_mapping::ProjectionBounds;

pcl::PointXYZI point(const float x, const float y, const float z)
{
  pcl::PointXYZI result;
  result.x = x;
  result.y = y;
  result.z = z;
  result.intensity = 1.0F;
  return result;
}

TEST(PcdProjection, ComputesHeightRelativeZBounds)
{
  const ProjectionBounds bounds = go1_mapping::z_bounds(1.05);

  EXPECT_DOUBLE_EQ(bounds.min_z, -0.90);
  EXPECT_DOUBLE_EQ(bounds.max_z, 0.75);
}

TEST(PcdProjection, MarksOnlyObstacleCellsAndNeverInfersFreeSpace)
{
  pcl::PointCloud<pcl::PointXYZI> cloud;
  cloud.push_back(point(0.0F, 0.0F, 0.0F));
  cloud.push_back(point(2.0F, 1.0F, 1.0F));

  const Grid grid = go1_mapping::project_occupied(
    cloud, 0.05, go1_mapping::z_bounds(1.05));

  EXPECT_EQ(grid.resolution, 0.05);
  EXPECT_NE(
    std::find(grid.cells.begin(), grid.cells.end(), std::uint8_t{0}),
    grid.cells.end());
  EXPECT_NE(
    std::find(grid.cells.begin(), grid.cells.end(), std::uint8_t{205}),
    grid.cells.end());
  EXPECT_EQ(
    std::count(grid.cells.begin(), grid.cells.end(), std::uint8_t{254}), 0);
}

TEST(PcdProjection, RejectsInvalidInputs)
{
  pcl::PointCloud<pcl::PointXYZI> cloud;
  cloud.push_back(point(0.0F, 0.0F, 0.0F));

  EXPECT_THROW(go1_mapping::z_bounds(0.0), std::invalid_argument);
  EXPECT_THROW(
    go1_mapping::project_occupied(
      cloud, 0.0, ProjectionBounds{-1.0, 1.0}),
    std::invalid_argument);

  pcl::PointCloud<pcl::PointXYZI> invalid_cloud;
  invalid_cloud.push_back(point(
    std::numeric_limits<float>::quiet_NaN(), 0.0F, 0.0F));
  EXPECT_THROW(
    go1_mapping::project_occupied(
      invalid_cloud, 0.05, ProjectionBounds{-1.0, 1.0}),
    std::invalid_argument);
}

}  // namespace