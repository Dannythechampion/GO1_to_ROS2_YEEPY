#ifndef GO1_MAPPING__PCD_PROJECTION_HPP_
#define GO1_MAPPING__PCD_PROJECTION_HPP_

#include <cstddef>
#include <cstdint>
#include <filesystem>
#include <functional>
#include <string>
#include <vector>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

namespace go1_mapping
{

// Match the session writer's 256 MiB retained PointXYZI payload budget.
// PCL filter workspaces are additional, short-lived allocations.
inline constexpr std::size_t kMaximumMergedCloudBytes = 268435456U;

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

struct PublicationArtifact
{
  std::filesystem::path temporary;
  std::filesystem::path final;
};

struct PublicationOperations
{
  std::function<void(const std::filesystem::path &, const std::filesystem::path &)>
  link_no_replace;
  std::function<void(const std::filesystem::path &)> unlink_path;
  std::function<void(const std::filesystem::path &)> sync_directory;
};

ProjectionBounds z_bounds(double sensor_height_m);

std::size_t checked_merged_point_count(
  std::size_t existing_points,
  std::size_t incoming_points,
  std::size_t max_bytes = kMaximumMergedCloudBytes);

pcl::PointCloud<pcl::PointXYZI> finite_xyz_copy(
  const pcl::PointCloud<pcl::PointXYZI> & cloud,
  std::size_t max_bytes = kMaximumMergedCloudBytes);

pcl::PointCloud<pcl::PointXYZI> voxel_filter_finite(
  const pcl::PointCloud<pcl::PointXYZI> & cloud,
  double voxel_size,
  std::size_t max_bytes = kMaximumMergedCloudBytes);

Grid project_occupied(
  const pcl::PointCloud<pcl::PointXYZI> & cloud,
  double resolution,
  ProjectionBounds bounds);

void publish_artifacts(
  const std::vector<PublicationArtifact> & artifacts,
  const PublicationOperations & operations);

std::string yaml_quote(const std::string & value);

}  // namespace go1_mapping

#endif  // GO1_MAPPING__PCD_PROJECTION_HPP_
