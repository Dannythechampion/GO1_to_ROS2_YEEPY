#ifndef GO1_MAPPING__PCD_PROJECTION_HPP_
#define GO1_MAPPING__PCD_PROJECTION_HPP_

#include <cstddef>
#include <cstdint>
#include <vector>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

namespace go1_mapping
{

struct ProjectionBounds
{
  double min_z;
  double max_z;
};

struct Grid
{
  std::size_t width;
  std::size_t height;
  double resolution;
  double origin_x;
  double origin_y;
  std::vector<std::uint8_t> cells;
};

ProjectionBounds z_bounds(double sensor_height_m);

Grid project_occupied(
  const pcl::PointCloud<pcl::PointXYZI> & cloud,
  double resolution,
  ProjectionBounds bounds);

}  // namespace go1_mapping

#endif  // GO1_MAPPING__PCD_PROJECTION_HPP_