#include "go1_mapping/pcd_projection.hpp"

#include <algorithm>
#include <cmath>
#include <limits>
#include <stdexcept>

namespace go1_mapping
{
namespace
{

constexpr double kPaddingMeters = 1.0;
constexpr std::uint8_t kUnknown = 205;
constexpr std::uint8_t kOccupied = 0;
constexpr std::size_t kMaximumGridCells = 100000000;

bool finite_point(const pcl::PointXYZI & point)
{
  return std::isfinite(point.x) && std::isfinite(point.y) && std::isfinite(point.z);
}

std::size_t checked_dimension(const double extent, const double resolution)
{
  const double cells = std::ceil(extent / resolution);
  if (!std::isfinite(cells) || cells < 1.0 ||
    cells > static_cast<double>(std::numeric_limits<std::size_t>::max()))
  {
    throw std::length_error("projected grid dimension is invalid or too large");
  }
  return static_cast<std::size_t>(cells);
}

}  // namespace

ProjectionBounds z_bounds(const double sensor_height_m)
{
  if (!std::isfinite(sensor_height_m) || sensor_height_m <= 0.0) {
    throw std::invalid_argument("sensor height must be finite and greater than zero");
  }
  return ProjectionBounds{-sensor_height_m + 0.15, -sensor_height_m + 1.80};
}

Grid project_occupied(
  const pcl::PointCloud<pcl::PointXYZI> & cloud,
  const double resolution,
  const ProjectionBounds bounds)
{
  if (!std::isfinite(resolution) || resolution <= 0.0) {
    throw std::invalid_argument("grid resolution must be finite and greater than zero");
  }
  if (!std::isfinite(bounds.min_z) || !std::isfinite(bounds.max_z) ||
    bounds.min_z > bounds.max_z)
  {
    throw std::invalid_argument("projection z bounds must be finite and ordered");
  }

  double min_x = std::numeric_limits<double>::infinity();
  double max_x = -std::numeric_limits<double>::infinity();
  double min_y = std::numeric_limits<double>::infinity();
  double max_y = -std::numeric_limits<double>::infinity();
  std::size_t valid_points = 0;
  for (const auto & point : cloud.points) {
    if (!finite_point(point)) {
      continue;
    }
    min_x = std::min(min_x, static_cast<double>(point.x));
    max_x = std::max(max_x, static_cast<double>(point.x));
    min_y = std::min(min_y, static_cast<double>(point.y));
    max_y = std::max(max_y, static_cast<double>(point.y));
    ++valid_points;
  }
  if (valid_points == 0) {
    throw std::invalid_argument("point cloud has no finite XYZ points");
  }

  const double origin_x = min_x - kPaddingMeters;
  const double origin_y = min_y - kPaddingMeters;
  const double extent_x = (max_x - min_x) + 2.0 * kPaddingMeters;
  const double extent_y = (max_y - min_y) + 2.0 * kPaddingMeters;
  if (!std::isfinite(origin_x) || !std::isfinite(origin_y) ||
    !std::isfinite(extent_x) || !std::isfinite(extent_y))
  {
    throw std::length_error("projected grid bounds overflow");
  }

  const std::size_t width = checked_dimension(extent_x, resolution);
  const std::size_t height = checked_dimension(extent_y, resolution);
  if (height > std::numeric_limits<std::size_t>::max() / width) {
    throw std::length_error("projected grid cell count overflows size_t");
  }
  const std::size_t cell_count = width * height;
  if (cell_count > kMaximumGridCells) {
    throw std::length_error("projected grid exceeds the 100 million cell safety limit");
  }

  Grid result{
    width, height, resolution, origin_x, origin_y,
    std::vector<std::uint8_t>(cell_count, kUnknown)};
  for (const auto & point : cloud.points) {
    if (!finite_point(point) || point.z < bounds.min_z || point.z > bounds.max_z) {
      continue;
    }
    const double column_value = std::floor((static_cast<double>(point.x) - origin_x) / resolution);
    const double row_value = std::floor((static_cast<double>(point.y) - origin_y) / resolution);
    if (!std::isfinite(column_value) || !std::isfinite(row_value) ||
      column_value < 0.0 || row_value < 0.0 ||
      column_value >= static_cast<double>(width) ||
      row_value >= static_cast<double>(height))
    {
      continue;
    }
    const auto column = static_cast<std::size_t>(column_value);
    const auto row = static_cast<std::size_t>(row_value);
    result.cells[row * width + column] = kOccupied;
  }
  return result;
}

}  // namespace go1_mapping